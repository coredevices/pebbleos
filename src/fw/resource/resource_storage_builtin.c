/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "resource_storage_builtin.h"
#include "resource_storage_impl.h"

#include "kernel/memory_layout.h"
#include "pbl/services/process_management/app_storage.h"

#include <string.h>

extern const BuiltInResourceData g_builtin_resources[];
extern const uint32_t g_num_builtin_resources;

static uint32_t resource_storage_builtin_read(ResourceStoreEntry *entry, uint32_t offset,
                                              void *data, size_t num_bytes) {
  const BuiltInResourceData *builtin = entry->store_data;
  if (!builtin) {
    return 0;
  }
  memcpy(data, builtin->address + offset, num_bytes);
  return num_bytes;
}

bool resource_storage_builtin_bytes_are_readonly(const void *bytes) {
  if (bytes == NULL) {
    return false;
  }
  return memory_layout_is_pointer_in_region(memory_layout_get_microflash_region(), bytes);
}

static const uint8_t *resource_storage_builtin_readonly_bytes(ResourceStoreEntry *entry,
                                                              bool has_privileged_access) {
  const BuiltInResourceData *builtin = entry->store_data;
  if (!builtin) {
    return NULL;
  }
  return builtin->address;
}

static bool resource_storage_builtin_find_resource(ResourceStoreEntry *entry, ResAppNum app_num,
                                                   uint32_t resource_id) {
  if (app_num != SYSTEM_APP) {
    return false;
  }
  // Story time! This is closely related to PBL-14367
  // resource_id == 0 means get the store.
  // HOWEVER, both builtin and flash stores respond to (app_num,rsrc_id) == (SYSTEM_APP,*)
  // When we ask for (SYSTEM_APP,0), we always want to actually be getting the flash store.
  // As a result, we should return false when rsrc_id == 0.
  // In the future, we need to change this hideously gross behavior.
  // BUTTTTTTT, PRF only has builtin, so we _need_ to say yes on PRF!
  if (resource_id == 0) {
#ifdef CONFIG_RECOVERY_FW
    return true;
#else
    return false;
#endif
  }
  for (unsigned int i = 0; i < g_num_builtin_resources; ++i) {
    if (g_builtin_resources[i].resource_id == resource_id) {
      entry->store_data = &g_builtin_resources[i];
      return true;
    }
  }
  return false;
}

static bool resource_storage_builtin_get_resource(ResourceStoreEntry *entry) {
  const BuiltInResourceData *builtin = entry->store_data;
  if (!builtin) {
    return false;
  }
  entry->offset = 0;
  entry->length = builtin->num_bytes;
  return true;
}

bool resource_storage_builtin_check(ResAppNum app_num, uint32_t resource_id,
                                    ResourceStoreEntry *entry,
                                    const ResourceVersion *expected_version) {
  // Builtins don't have manifests and can't be corrupted because they're built into the micro
  // flash image.
  return true;
}

const ResourceStoreImplementation g_builtin_impl = {
  .type = ResourceStoreTypeBuiltIn,

  .init = resource_storage_generic_init,
  .clear = resource_storage_generic_clear,
  .check = resource_storage_builtin_check,

  .metadata_size = resource_storage_generic_metadata_size,
  .find_resource = resource_storage_builtin_find_resource,
  .get_resource = resource_storage_builtin_get_resource,

  .get_length = resource_storage_generic_get_length,
  .get_crc = resource_storage_generic_get_crc,
  .write = resource_storage_generic_write,
  .read = resource_storage_builtin_read,
  .readonly_bytes = resource_storage_builtin_readonly_bytes,

  .watch = resource_storage_generic_watch,
  .unwatch = resource_storage_generic_unwatch,
};
