/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <inttypes.h>
#include <stdbool.h>

#include "resource.h"

typedef enum {
  InvalidResourceStore = 0,
  /* System Bank */
  ResourceStoreTypeSystemBank,
  /* App Banks in PFS */
  ResourceStoreTypeAppFile,
  /* Baked in FW. E.g. Fallback Font */
  ResourceStoreTypeBuiltIn,
  /* Filesystem stored resources */
  ResourceStoreTypeFile,
} ResourceStoreType;

struct ResourceStoreImplementation;

typedef struct {
  uint32_t id; // Used when the store implementation needs to permute the resource_id
  const struct ResourceStoreImplementation *impl;
  uint32_t offset;
  uint32_t length;
  const void *store_data;
} ResourceStoreEntry;

// Used to flag that the ResourceStoreEntry hasn't had its length filled yet.
#define ENTRY_LENGTH_UNSET ((uint32_t)~0)

// The filename suffix we use to represent a resource file
#define APP_RESOURCES_FILENAME_SUFFIX "res"

typedef struct ResourceStoreImplementation {
  ResourceStoreType type;
  // None of these callbacks may be NULL. There are generic implementations of most that can be
  // used if nothing 'unique' needs to be done.

  void (*init)(void);
  void (*clear)(ResourceStoreEntry *entry);
  // if resource_id == 0 then check all of resource storage, else just validate
  // that the resource requested is valid
  bool (*check)(ResAppNum app_num, uint32_t resource_id, ResourceStoreEntry *entry,
                const ResourceVersion *expected_version);

  uint32_t (*metadata_size)(ResourceStoreEntry *entry);
  bool (*find_resource)(ResourceStoreEntry *entry, ResAppNum app_num, uint32_t resource_id);
  bool (*get_resource)(ResourceStoreEntry *entry);

  uint32_t (*get_length)(ResourceStoreEntry *entry);
  uint32_t (*get_crc)(ResourceStoreEntry *entry, uint32_t num_bytes, uint32_t entry_offset);
  uint32_t (*write)(ResourceStoreEntry *entry, uint32_t offset, void *data, size_t num_bytes);
  uint32_t (*read)(ResourceStoreEntry *entry, uint32_t offset, void *data, size_t num_bytes);
  const uint8_t *(*readonly_bytes)(ResourceStoreEntry *entry, bool has_privileged_access);

  ResourceCallbackHandle (*watch)(ResourceStoreEntry *entry, ResourceChangedCallback callback,
                                  void* data);
  bool (*unwatch)(ResourceCallbackHandle cb_handle);
} ResourceStoreImplementation;

void resource_storage_init(void);

void resource_storage_clear(ResAppNum app_num);

bool resource_storage_check(ResAppNum app_num, uint32_t resource_id,
                            const ResourceVersion *expected_version);

ResourceVersion resource_storage_get_version(ResAppNum app_num, uint32_t resource_id);

uint32_t resource_storage_get_num_entries(ResAppNum app_num, uint32_t resource_id);

void resource_storage_get_resource(ResAppNum app_num, uint32_t resource_id,
                                   ResourceStoreEntry *entry);

uint32_t resource_storage_read(ResourceStoreEntry *entry, uint32_t offset, void *data,
                               size_t num_bytes);

uint32_t resource_store_get_metadata_size(ResourceStoreEntry *entry);

// TODO PBL-21009: Move this somewhere else.
void resource_storage_get_file_name(char *name, size_t buf_length, ResAppNum app_num);
