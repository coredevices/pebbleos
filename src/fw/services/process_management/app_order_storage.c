/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/process_management/app_order_storage.h"

#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "pbl/services/filesystem/pfs.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

PBL_LOG_MODULE_DEFINE(service_process_management, CONFIG_SERVICE_PROCESS_MANAGEMENT_LOG_LEVEL);

#define ORDER_FILE "lnc_ord"

typedef struct {
  PebbleMutex *order_mutex;
  bool file_known_missing;
} AppOrderData;

static AppOrderData s_data;

void app_order_storage_init(void) {
  s_data.order_mutex = mutex_create();
}

#if UNITTEST
void app_order_storage_reset_for_tests(void) {
  s_data.file_known_missing = false;
}
#endif

//! Must be called from the App Task
AppMenuOrderStorage *app_order_read_order(void) {
  PBL_ASSERT_TASK(PebbleTask_App);

  AppMenuOrderStorage *storage = NULL;
  bool delete_file = false;
  mutex_lock(s_data.order_mutex);

  // Early exit if we already know the file doesn't exist
  if (s_data.file_known_missing) {
    mutex_unlock(s_data.order_mutex);
    return NULL;
  }

  int fd;
  if ((fd = pfs_open(ORDER_FILE, OP_FLAG_READ, 0, 0)) < 0) {
    PBL_LOG_DBG("App menu order file does not exist");
    s_data.file_known_missing = true;
    mutex_unlock(s_data.order_mutex);
    return NULL;
  }

  // Check if it is an valid file
  if ((pfs_get_file_size(fd) % sizeof(AppInstallId)) != sizeof(uint8_t)) {
    PBL_LOG_ERR("Invalid order storage file");
    delete_file = true;
    goto cleanup;
  }

  // Read the number of AppInstallId's listed in the file
  uint8_t list_length;
  if (pfs_read(fd, &list_length, sizeof(uint8_t)) != sizeof(uint8_t)) {
    PBL_LOG_ERR("Could not read app menu order file");
    delete_file = true;
    goto cleanup;
  }

  // Allocate room for the order list array. Free'd by the caller of the function.
  storage = app_malloc(sizeof(AppMenuOrderStorage) + list_length * sizeof(AppInstallId));
  if (!storage) {
    PBL_LOG_ERR("Failed to malloc stored order install_id list");
    goto cleanup;
  }

  // read in entire list into array
  const int read_size = list_length * sizeof(AppInstallId);
  int rd_sz;
  if ((rd_sz = pfs_read(fd, (uint8_t *)storage->id_list, read_size)) != read_size) {
    PBL_LOG_ERR("Corrupted ordered install_id list (Rd %d of %d bytes)",
        rd_sz, read_size);
    app_free(storage);
    storage = NULL;
    delete_file = true;
    goto cleanup;
  }

  // set the list length
  storage->list_length = list_length;

cleanup:
  pfs_close(fd);
  if (delete_file) {
    pfs_remove(ORDER_FILE);
  }

  mutex_unlock(s_data.order_mutex);
  return storage;
}

//! Should be called on system task.
static void prv_app_order_write_order(AppMenuOrderStorage *storage) {
  mutex_lock(s_data.order_mutex);

  int storage_size = sizeof(AppMenuOrderStorage) + (storage->list_length * sizeof(AppInstallId));

  int fd = pfs_open(ORDER_FILE, OP_FLAG_OVERWRITE, FILE_TYPE_STATIC, storage_size);
  if (fd == E_DOES_NOT_EXIST) {
    // File doesn't exist, need to create a new file.
    fd = pfs_open(ORDER_FILE, OP_FLAG_WRITE, FILE_TYPE_STATIC, storage_size);
  }

  if (fd < 0) {
    PBL_LOG_ERR("Could not create app menu order file");
    goto cleanup;
  }

  // write back the whole file
  int wrote_storage_bytes = pfs_write(fd, (uint8_t *)storage, storage_size);

  if (wrote_storage_bytes != storage_size) {
    PBL_LOG_ERR("Failed to write all bytes of order list");
  }

  pfs_close(fd);

  // File now exists, clear the missing flag
  s_data.file_known_missing = false;

cleanup:
  kernel_free(storage);
  mutex_unlock(s_data.order_mutex);
}

typedef struct {
  const Uuid *uuid_list;
  uint8_t count;
  AppMenuOrderStorage *storage;
} UuidTranslateData;

// search for a UUID in a list of UUID's. Return the index in which it was found or -1 if not found.
int prv_uuid_search(const Uuid *find_me, const Uuid *uuid_list, uint8_t count) {
  for (int i = 0; i < count; i++) {
    if (uuid_equal(&uuid_list[i], find_me)) {
      return i;
    }
  }

  return -1;
}

// if an entry appears in the UUID list, place it's install_id in the correct index of
// storage->id_list
bool prv_enumerate_apps(AppInstallEntry *entry, void *data) {
  UuidTranslateData *my_data = (UuidTranslateData *) data;

  int idx = prv_uuid_search(&entry->uuid, my_data->uuid_list, my_data->count);

  if (idx < 0) {
    return true; // continue iterating
  }

  my_data->storage->id_list[idx] = entry->install_id;
  return true; // continue iterating
}


//! Should be called on system task.
void write_uuid_list_to_file(const Uuid *uuid_list, uint8_t count) {
  PBL_ASSERT_TASK(PebbleTask_KernelBackground);

  int storage_size = sizeof(AppMenuOrderStorage) + (count * sizeof(AppInstallId));
  AppMenuOrderStorage *storage = kernel_malloc(storage_size);
  memset(storage, 0, storage_size);

  UuidTranslateData data = {
    .uuid_list = uuid_list,
    .count = count,
    .storage = storage,
  };

  // go through all install entries
  app_install_enumerate_entries(prv_enumerate_apps, &data);
  storage->list_length = count;

  prv_app_order_write_order(storage);
}
