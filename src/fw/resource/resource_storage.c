/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "resource_storage.h"
#include "resource_storage_impl.h"

#include <string.h>

#include "pbl/services/filesystem/app_file.h"
#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "system/version.h"
#include "pbl/util/size.h"

static const ResourceStoreImplementation *s_resource_store_impls[] = {
#define RESOURCE_IMPL(impl) &impl,
#include "resource_impl.def"
#undef RESOURCE_IMPL
};

// Check if our offset+length is within the resource entry's bounds.
// Truncate the length if we overrun the ending.
static uint32_t prv_check_resource_bounds(ResourceStoreEntry *entry, uint32_t store_offset,
                                          size_t num_bytes) {
  // If we haven't had the length set yet, just assume we're ok.
  if (entry->length == ENTRY_LENGTH_UNSET) {
    return num_bytes;
  }

  uint32_t resource_offset = store_offset - entry->offset;

  if (entry->length < resource_offset) {
    PBL_LOG_ERR("Resource offset past its own ending.");
    return 0;
  } else if ((resource_offset + num_bytes) > entry->length) {
    PBL_LOG_ERR("offset + length > resource size, truncated.");
    return entry->length - resource_offset;
  } else {
    return num_bytes;
  }
}

static uint32_t prv_read(ResourceStoreEntry *entry, uint32_t offset, void *data, size_t num_bytes) {
  num_bytes = prv_check_resource_bounds(entry, offset, num_bytes);
  if (!num_bytes) {
    return 0;
  }
  return entry->impl->read(entry, offset, data, num_bytes);
}

static void prv_get_manifest(ResourceStoreEntry *entry, ResourceManifest *manifest) {
  if (prv_read(entry, 0, manifest, sizeof(ResourceManifest)) != sizeof(ResourceManifest)) {
    *manifest = (ResourceManifest){0};
  }
}

static bool prv_read_res_table_entry(ResTableEntry *res_entry, ResourceStoreEntry *entry,
                                     uint32_t index) {
  uint32_t addr = MANIFEST_SIZE + index * TABLE_ENTRY_SIZE;

  return prv_read(entry, addr, res_entry, sizeof(ResTableEntry)) == sizeof(ResTableEntry);
}

static uint32_t prv_get_length(ResourceStoreEntry *entry) {
  return entry->impl->get_length(entry);
}

// entry_offset is the offset of the resource of interest.
// If we're doing the whole store, that ends up being 0.
static uint32_t prv_get_crc(ResourceStoreEntry *entry, uint32_t num_bytes, uint32_t entry_offset) {
  num_bytes = prv_check_resource_bounds(entry, entry_offset, num_bytes);
  if (!num_bytes) {
    return 0xFFFFFFFF;
  }
  return entry->impl->get_crc(entry, num_bytes, entry_offset);
}

T_STATIC uint32_t prv_get_store_length(ResourceStoreEntry *entry, ResourceManifest *manifest) {
  // Get the resource entry for the last entry
  ResTableEntry res_entry = {0};
  if (!prv_read_res_table_entry(&res_entry, entry, manifest->num_resources - 1)) {
    return 0;
  }
  // Get the full ending offset of the last resource.
  uint32_t resource_end_offset = res_entry.offset + res_entry.length;
  // Add the store's metadata size to the end address of the last resource.
  uint32_t store_length = resource_end_offset + resource_store_get_metadata_size(entry);

  // Catch an overflow if the store is enormous (unlikely unless corrupted)
  if (store_length < resource_end_offset) {
    // overflow
    PBL_LOG_ERR("Overflow while validating resource");
    return 0;
  }

  // Make sure the store's calculated length is not past the end of the store.
  if (prv_get_length(entry) < store_length) {
    return 0;
  }

  // Return the length of the store's resource data
  return resource_end_offset;
}

static bool prv_validate_store(ResourceManifest *manifest, ResourceStoreEntry *entry,
                               ResAppNum app_num) {
  uint32_t num_bytes;

  if ((num_bytes = prv_get_store_length(entry, manifest)) == 0) {
    PBL_LOG_WRN("Resource table check failed. Table or manifest may be corrupted");
    return false;
  }

  uint32_t calculated_crc = prv_get_crc(entry, num_bytes, 0);
  if (calculated_crc != manifest->version.crc) {
    PBL_LOG_WRN("Resource crc mismatch for app %"PRIu32".", app_num);
    PBL_LOG_WRN("0x%"PRIx32" != 0x%"PRIx32, calculated_crc, manifest->version.crc);


    PBL_LOG_WRN("PBL-28517: If you see this please let Brad know");

    const uint32_t calculated_crc_again = prv_get_crc(entry, num_bytes, 0);
    PBL_LOG_WRN("Num bytes is %"PRIu32, num_bytes);
    PBL_LOG_WRN("Calculated the CRC again, got 0x%"PRIx32, calculated_crc_again);

    return calculated_crc_again == manifest->version.crc;
  }

  return true;
}

static void prv_get_store_entry(ResAppNum app_num, uint32_t resource_id,
                                ResourceStoreEntry *entry) {
  *entry = (ResourceStoreEntry){
    .id = resource_id,
    .length = ENTRY_LENGTH_UNSET,
  };
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_resource_store_impls); i++) {
    entry->impl = s_resource_store_impls[i];
    PBL_ASSERTN(entry->impl->find_resource);
    if (entry->impl->find_resource(entry, app_num, resource_id)) {
      return;
    }
  }
  PBL_LOG_WRN("get_store_entry(%"PRIu32",%"PRIu32") failed to find appropriate store",
          app_num, resource_id);
  entry->impl = NULL;
}

static bool prv_validate_entry(ResourceStoreEntry *entry, ResourceManifest *manifest,
                               uint32_t resource_id) {
  if (entry->id > manifest->num_resources) {
    PBL_LOG_DBG("Out of bound resource %"PRId32" vs %"PRId32,
        entry->id, manifest->num_resources);
    return false;
  }

  ResTableEntry table_entry = {0};
  if (!prv_read_res_table_entry(&table_entry, entry, entry->id - 1)) {
    return false;
  }

  if (entry->id != table_entry.resource_id) {
    PBL_LOG_ERR("Resource table entry for %" PRIx32 " is corrupt!"
      "(%"PRIx32" != %"PRIx32")", resource_id, entry->id, table_entry.resource_id);
    return false;
  }

  const uint32_t resource_crc = prv_get_crc(entry, table_entry.length, table_entry.offset);
  if (resource_crc != table_entry.crc) {
    PBL_LOG_DBG("Bad resource CRC for %" PRIx32 ", %" PRIx32
            " vs %" PRIx32, resource_id, resource_crc, table_entry.crc);
    return false;
  }

  return true;
}

uint32_t resource_store_get_metadata_size(ResourceStoreEntry *entry) {
  return entry->impl->metadata_size(entry);
}

void resource_storage_clear(ResAppNum app_num) {
  ResourceStoreEntry entry;
  prv_get_store_entry(app_num, 0, &entry);
  if (entry.impl) {
    entry.impl->clear(&entry);
  }
}

static bool prv_get_manifest_by_id(ResAppNum app_num, uint32_t resource_id,
                                   ResourceManifest *manifest) {
  ResourceStoreEntry entry;
  prv_get_store_entry(app_num, resource_id, &entry);
  if (!entry.impl) {
    return false;
  }
  prv_get_manifest(&entry, manifest);
  return true;
}

ResourceVersion resource_storage_get_version(ResAppNum app_num, uint32_t resource_id) {
  ResourceManifest manifest;
  if (!prv_get_manifest_by_id(app_num, resource_id, &manifest)) {
    return (ResourceVersion) {0};
  }
  return manifest.version;
}

uint32_t resource_storage_get_num_entries(ResAppNum app_num, uint32_t resource_id) {
  ResourceManifest manifest;
  if (!prv_get_manifest_by_id(app_num, resource_id, &manifest)) {
    return 0;
  }
  return manifest.num_resources;
}

// if resource_id == 0 then check all of resource storage, else just validate
// that the resource requested is valid
bool resource_storage_check(ResAppNum app_num, uint32_t resource_id,
                            const ResourceVersion *expected_version) {
  ResourceStoreEntry entry;
  prv_get_store_entry(app_num, resource_id, &entry);
  if (!entry.impl) {
    return false;
  }
  return entry.impl->check(app_num, resource_id, &entry, expected_version);
}

void resource_storage_init(void) {
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_resource_store_impls); i++) {
    if (s_resource_store_impls[i]->init) {
      s_resource_store_impls[i]->init();
    }
  }
}

uint32_t resource_storage_read(ResourceStoreEntry *entry, uint32_t offset, void *data,
                               size_t num_bytes) {
  return prv_read(entry, offset + entry->offset, data, num_bytes);
}

void resource_storage_get_resource(ResAppNum app_num, uint32_t resource_id,
                                   ResourceStoreEntry *entry) {
  prv_get_store_entry(app_num, resource_id, entry);
  if (!entry->impl) {
    *entry = (ResourceStoreEntry){0};
    return;
  }

  if (!entry->impl->get_resource(entry)) {
    *entry = (ResourceStoreEntry){0};
    return;
  }
  PBL_ASSERTN(entry->length != ENTRY_LENGTH_UNSET);
}

ResourceCallbackHandle resource_watch(ResAppNum app_num, uint32_t resource_id,
                                      ResourceChangedCallback callback, void* data) {
  ResourceStoreEntry entry;
  prv_get_store_entry(app_num, resource_id, &entry);
  if (!entry.impl) {
    return NULL;
  }
  return entry.impl->watch(&entry, callback, data);
}

void resource_unwatch(ResourceCallbackHandle cb_handle) {
#ifndef CONFIG_RECOVERY_FW
  // TODO: Support unwatching not-files.
  g_file_impl.unwatch(cb_handle);
#endif
}

void resource_storage_get_file_name(char *name, size_t buf_length, ResAppNum resource_bank) {
  app_file_name_make(name, buf_length, resource_bank, APP_RESOURCES_FILENAME_SUFFIX,
                     strlen(APP_RESOURCES_FILENAME_SUFFIX));
}


void resource_storage_generic_init(void) {
}

void resource_storage_generic_clear(ResourceStoreEntry *entry) {
}

bool resource_storage_generic_check(ResAppNum app_num, uint32_t resource_id,
                                    ResourceStoreEntry *entry,
                                    const ResourceVersion *expected_version) {
  ResourceManifest manifest;
  prv_get_manifest(entry, &manifest);
  if (expected_version && !resource_version_matches(&manifest.version, expected_version)) {
    PBL_LOG_DBG("expected version <%#010"PRIx32", %"PRIu32">,",
            expected_version->crc, expected_version->timestamp);
    PBL_LOG_DBG("got <%#010"PRIx32", %"PRIu32">,",
            manifest.version.crc, manifest.version.timestamp);
    return false;
  }

  if (manifest.num_resources == 0) {
    // no resources, no need to read anything more
    return true;
  }

  if (resource_id == 0) {
    return (prv_validate_store(&manifest, entry, app_num));
  } else if (!prv_validate_entry(entry, &manifest, resource_id)) {
    PBL_LOG_WRN("Resource %"PRId32" check for App %"PRIu32" failed",
            resource_id, app_num);
    return false;
  }

  return true;
}

uint32_t resource_storage_generic_metadata_size(ResourceStoreEntry *entry) {
  return RESOURCE_STORE_METADATA_BYTES;
}

bool resource_storage_generic_get_resource(ResourceStoreEntry *entry) {
  ResourceManifest manifest;
  prv_get_manifest(entry, &manifest);
  if (entry->id > manifest.num_resources) {
    return false;
  }

  ResTableEntry table_entry;
  if (!prv_read_res_table_entry(&table_entry, entry, entry->id - 1)) {
    return false;
  }
  if ((table_entry.resource_id != entry->id) ||
      (table_entry.length == 0)) {
    // empty resource
    return false;
  }
  entry->offset = resource_store_get_metadata_size(entry) + table_entry.offset;
  entry->length = table_entry.length;
  return true;
}

uint32_t resource_storage_generic_get_length(ResourceStoreEntry *entry) {
  return entry->length;
}

uint32_t resource_storage_generic_get_crc(ResourceStoreEntry *entry, uint32_t num_bytes,
                                          uint32_t entry_offset) {
  return 0;
}

uint32_t resource_storage_generic_write(ResourceStoreEntry *entry, uint32_t offset, void *data,
                                        size_t num_bytes) {
  return 0;
}

ResourceCallbackHandle resource_storage_generic_watch(ResourceStoreEntry *entry,
                                                      ResourceChangedCallback callback,
                                                      void* data) {
  PBL_LOG_WRN("resource_watch not supported for resource type %d.",
          entry->impl->type);
  return NULL;
}

bool resource_storage_generic_unwatch(ResourceCallbackHandle cb_handle) {
  return false;
}
