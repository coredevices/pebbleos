/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/persist.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kernel/pbl_malloc.h"
#include "pbl/kernel/mutex.h"
#include "process_management/app_install_manager.h"
#include "pbl/services/filesystem/app_file.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/list.h"
#include "pbl/util/math.h"
#include "util/units.h"

PBL_LOG_MODULE_DEFINE(service_persist, CONFIG_SERVICE_PERSIST_LOG_LEVEL);

#define PERSIST_STORAGE_MAX_SPACE MiBYTES(1)
#define PERSIST_STORAGE_INITIAL_ALLOC KiBYTES(4)

typedef struct PersistStore {
  ListNode  list_node;
  Uuid uuid;
  SettingsFile file;
  bool file_open;
  uint8_t usage_count;          //!< How many clients are using this store
} PersistStore;

// Each open client has a PersistStore structure linked into this list. If both
// a worker and foreground app of the same UUID are running, then they share the
// same store.
static ListNode *s_client_stores;
static PBL_MUTEX_DEFINE(s_mutex);


static bool prv_uuid_list_filter(ListNode* node, void* data) {
  const Uuid *uuid = data;
  PersistStore* store = (PersistStore*)node;
  return uuid_equal(&store->uuid, uuid);
}

static PersistStore * prv_find_open_store(const Uuid *uuid) {
    return (PersistStore *)list_find(s_client_stores, prv_uuid_list_filter,
                                     (void *)uuid);
}

static ALWAYS_INLINE void prv_lock(void) {
  pbl_mutex_lock_lr(&s_mutex, PBL_FOREVER, (uintptr_t)__builtin_return_address(0));
}

static inline void prv_unlock(void) {
  pbl_mutex_unlock(&s_mutex);
}

// "ps" prefix + 32 hex chars (16-byte UUID) + NUL.
#define PERSIST_FILE_NAME_MAX_LENGTH sizeof("ps000102030405060708090a0b0c0d0e0f")

static status_t prv_get_file_name(char *name, size_t buf_len, const Uuid *uuid) {
  // Persist files are named "ps<uuid-hex>". The "ps" prefix indicates the file
  // is in SettingsFile format. The UUID is stable across reinstalls, so the
  // file follows the app regardless of its (volatile) AppInstallId.
  const uint8_t *b = (const uint8_t *)uuid;
  return snprintf(name, buf_len,
                  "ps%02x%02x%02x%02x%02x%02x%02x%02x"
                  "%02x%02x%02x%02x%02x%02x%02x%02x",
                  b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
                  b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
}

status_t persist_service_delete_file(const Uuid *uuid) {
  char name[PERSIST_FILE_NAME_MAX_LENGTH];

  status_t status = prv_get_file_name(name, sizeof(name), uuid);
  if (FAILED(status)) {
    return status;
  }
  return pfs_remove(name);
}

static bool prv_bad_persist_file_filter(const char *filename) {
  return is_app_file_name(filename) &&
         strcmp(filename + APP_FILE_NAME_PREFIX_LENGTH, "persist") == 0;
}

size_t persist_service_get_max_size(void) {
  return PERSIST_STORAGE_MAX_SPACE;
}

// Persist files used to be named "ps%06d", where the id was allocated by the
// legacy persist_map (the "pmap" file), a UUID->id table. They are now named
// "ps<uuid-hex>" directly. The following migrates existing files to the new
// scheme and removes the now-unused pmap.
// TODO: remove this migration once all devices have upgraded.

#define LEGACY_PMAP_FILE_NAME "pmap"
#define LEGACY_PERSIST_FILE_NAME_MAX_LENGTH sizeof("ps000001")
#define LEGACY_PMAP_EOF_ID ((int)(~0))

typedef struct PACKED {
  uint16_t version;
} LegacyPersistMapHeader;

typedef struct PACKED {
  int id;
  Uuid uuid;
} LegacyPersistMapField;

static status_t prv_copy_file(const char *from, const char *to) {
  int from_fd = pfs_open(from, OP_FLAG_READ, 0, 0);
  if (from_fd < 0) {
    return from_fd;
  }

  size_t size = pfs_get_file_size(from_fd);
  int to_fd = pfs_open(to, OP_FLAG_WRITE, FILE_TYPE_STATIC, size);
  if (to_fd < 0) {
    pfs_close(from_fd);
    return to_fd;
  }

  status_t rv = S_SUCCESS;
  const size_t chunk_size = 128;
  uint8_t *buf = kernel_malloc(chunk_size);
  if (buf == NULL) {
    rv = E_OUT_OF_MEMORY;
  } else {
    size_t remaining = size;
    while (remaining > 0) {
      size_t n = MIN(remaining, chunk_size);
      if ((rv = pfs_read(from_fd, buf, n)) != (int)n) {
        rv = (rv >= 0) ? E_INTERNAL : rv;
        break;
      }
      if ((rv = pfs_write(to_fd, buf, n)) != (int)n) {
        rv = (rv >= 0) ? E_INTERNAL : rv;
        break;
      }
      remaining -= n;
    }
    kernel_free(buf);
  }

  pfs_close(from_fd);
  pfs_close(to_fd);
  return rv;
}

static void prv_migrate_legacy_persist_files(void) {
  int fd = pfs_open(LEGACY_PMAP_FILE_NAME, OP_FLAG_READ, 0, 0);
  if (fd < 0) {
    // No legacy map, nothing to migrate.
    return;
  }

  pfs_seek(fd, sizeof(LegacyPersistMapHeader), FSeekSet);

  LegacyPersistMapField field;
  while (pfs_read(fd, (uint8_t *)&field, sizeof(field)) == (int)sizeof(field)) {
    if (field.id == LEGACY_PMAP_EOF_ID) {
      break;
    }

    char old_name[LEGACY_PERSIST_FILE_NAME_MAX_LENGTH];
    snprintf(old_name, sizeof(old_name), "ps%06d", field.id);

    char new_name[PERSIST_FILE_NAME_MAX_LENGTH];
    if (FAILED(prv_get_file_name(new_name, sizeof(new_name), &field.uuid))) {
      continue;
    }

    if (PASSED(prv_copy_file(old_name, new_name))) {
      pfs_remove(old_name);
    }
  }

  pfs_close(fd);
  pfs_remove(LEGACY_PMAP_FILE_NAME);
}

// Designed to be called once during reset
void persist_service_init(void) {

  prv_migrate_legacy_persist_files();

  // Find and delete any AppInstallId-indexed persist files. Due to PBL-16663
  // (affecting FW 3.0-dp5 thru -dp7), the AppInstallId in the file name may not
  // correspond to the app that the persist file originally belonged to. Since
  // we can't be sure that the persist files correspond to the current
  // AppInstallId, the safest thing to do is to simply blow them away.
  // TODO: remove this code before FW 3.0-golden.
  PFSFileListEntry *bad_file_list = pfs_create_file_list(
      prv_bad_persist_file_filter);
  PFSFileListEntry *iter = bad_file_list;
  while (iter) {
    pfs_remove(iter->name);
    iter = (PFSFileListEntry *)iter->list_node.next;
  }
  pfs_delete_file_list(bad_file_list);
}

// Return a pointer to the store for the given UUID. Each task that uses persist
// must call persist_service_client_open() to create/open the store during its
// startup and persist_service_client_close() during its shutdown.
//
// The SettingsFile is opened/created lazily. A persist file will not be
// created for an app unless it calls a persist function.
//
// The persist service mutex is locked when this function is called. It will
// only be unlocked after a call to persist_service_unlock(). While the global
// persist service mutex is currently used, the API is designed such that a
// per-file mutex could be used without altering the callers.
SettingsFile * persist_service_lock_and_get_store(const Uuid *uuid) {
  prv_lock();
  PersistStore *store = prv_find_open_store(uuid);
  PBL_ASSERTN(store);
  if (!store->file_open) {
    char filename[PERSIST_FILE_NAME_MAX_LENGTH];
    PBL_ASSERTN(PASSED(prv_get_file_name(filename, sizeof(filename), uuid)));
    PBL_ASSERTN(PASSED(settings_file_open_growable(&store->file, filename,
                                                   PERSIST_STORAGE_MAX_SPACE,
                                                   PERSIST_STORAGE_INITIAL_ALLOC)));
    store->file_open = true;
  }
  return &store->file;
}

void persist_service_unlock_store(SettingsFile *store) {
  prv_unlock();
}

// Create a store for a client of the given UUID it doesn't already exist. If it
// exists already (another client with the same UUID is running), then just
// increment its usage count. This is called by the process startup code
// (app_state_init() or worker_state_init()).
void persist_service_client_open(const Uuid *uuid) {
  prv_lock();
  {
    PersistStore *store = prv_find_open_store(uuid);
    if (store) {
      store->usage_count++;
    } else {
      store = kernel_malloc_check(sizeof(*store));
      *store = (PersistStore) {
        .uuid = *uuid,
        .usage_count = 1,
        .file_open = false,
      };
      s_client_stores = list_insert_before(s_client_stores, &store->list_node);
    }
  }
  prv_unlock();
}

// Release the store for the given UUID. Called by ProcessManager to clean up
// after a task exists. If there are no other processes using the same store, it
// will be freed
void persist_service_client_close(const Uuid *uuid) {
  prv_lock();
  {
    PersistStore *store = prv_find_open_store(uuid);
    PBL_ASSERTN(store &&
                list_contains(s_client_stores, &store->list_node) &&
                store->usage_count >= 1);

    if (--store->usage_count == 0) {
      if (store->file_open) {
        settings_file_close(&store->file);
      }
      list_remove(&store->list_node,
                  &s_client_stores /* &head */, NULL /* &tail */);
      kernel_free(store);
    }
  }
  prv_unlock();
}
