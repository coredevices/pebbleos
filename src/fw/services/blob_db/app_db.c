/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/app_db.h"

#include "pbl/util/uuid.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager_private.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/app_fetch_endpoint.h"
#include "pbl/kernel/mutex.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "system/status_codes.h"
#include "pbl/util/math.h"
#include "util/units.h"

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

#define SETTINGS_FILE_NAME   "appdb"
// Holds about ~150 app metadata blobs
#define SETTINGS_FILE_SIZE KiBYTES(20)

#define FIRST_VALID_INSTALL_ID (INSTALL_ID_INVALID + 1)

static AppInstallId s_next_unique_flash_app_id;

static struct {
  SettingsFile settings_file;
  struct pbl_mutex mutex;
} s_app_db;

//////////////////////
// Settings helpers
//////////////////////

struct AppDBInitData {
  AppInstallId max_id;
  uint32_t num_apps;
};

static status_t prv_lock_mutex_and_open_file(void) {
  pbl_mutex_lock(&s_app_db.mutex, PBL_FOREVER);
  status_t rv = settings_file_open_growable(&s_app_db.settings_file,
                                            SETTINGS_FILE_NAME,
                                            SETTINGS_FILE_SIZE,
                                            KiBYTES(4));
  if (rv != S_SUCCESS) {
    pbl_mutex_unlock(&s_app_db.mutex);
  }
  return rv;
}

static void prv_close_file_and_unlock_mutex(void) {
  settings_file_close(&s_app_db.settings_file);
  pbl_mutex_unlock(&s_app_db.mutex);
}

static status_t prv_cancel_app_fetch(AppInstallId app_id) {
  if (pebble_task_get_current() == PebbleTask_KernelBackground) {
    // if we are on kernel_bg, we can go ahead and cancel the app fetch instantly
    app_fetch_cancel_from_system_task(app_id);
    return S_SUCCESS;
  } else {
    // ignore the deletion and send back a failure message. The phone will retry later.
    return E_BUSY;
  }
}

//! SettingsFileEachCallback function is used to iterate over all keys and find the largest
//! AppInstallId currently being using.
static bool prv_each_inspect_ids(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if ((info->val_len == 0) || (info->key_len != sizeof(AppInstallId))) {
    return true; // continue iterating
  }

  struct AppDBInitData *data = context;

  AppInstallId app_id;
  info->get_key(file, (uint8_t *)&app_id, sizeof(AppInstallId));

  data->max_id = MAX(data->max_id, app_id);
  data->num_apps++;

  return true; // continue iterating
}

struct UuidFilterData {
  Uuid          uuid;
  AppInstallId  found_id;
};

//! SettingsFileEachCallback function is used to iterate over all entries and search for
//! the particular entry with the given UUID. If one is found, it will set the uuid_data->found_id
//! to a value other than INSTALL_ID_INVALID
static bool prv_db_filter_app_id(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if ((info->val_len == 0) || (info->key_len != sizeof(AppInstallId))) {
    return true; // continue iterating
  }

  struct UuidFilterData *uuid_data = (struct UuidFilterData *)context;

  AppInstallId app_id;
  AppDBEntry entry;
  info->get_key(file, (uint8_t *)&app_id, info->key_len);
  info->get_val(file, (uint8_t *)&entry, info->val_len);

  if (uuid_equal(&uuid_data->uuid, &entry.uuid)) {
    uuid_data->found_id = app_id;
    return false; // stop iterating
  }
  return true; // continue iterating
}

//! Retrieves the AppInstallId for a given UUID using the SettingsFile that is already open.
//! @note Requires holding the lock already
static AppInstallId prv_find_install_id_for_uuid(SettingsFile *file, const Uuid *uuid) {
  // used when iterating through all entries in our database.
  struct UuidFilterData filter_data = {
    .found_id = INSTALL_ID_INVALID,
    .uuid = *uuid,
  };

  settings_file_each(file, prv_db_filter_app_id, (void *)&filter_data);
  return filter_data.found_id;
}

/////////////////////////
// App DB Specific API
/////////////////////////

AppInstallId app_db_get_install_id_for_uuid(const Uuid *uuid) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  AppInstallId app_id = prv_find_install_id_for_uuid(&s_app_db.settings_file, uuid);

  prv_close_file_and_unlock_mutex();
  return app_id;
}

///////////////////////////
// App DB API
///////////////////////////


//! Fills an AppDBEntry for a given UUID. This is a wrapper around app_db_read to keep it uniform
//! with `app_db_get_app_entry_for_install_id`
status_t app_db_get_app_entry_for_uuid(const Uuid *uuid, AppDBEntry *entry) {
  return app_db_read((uint8_t *)uuid, sizeof(Uuid), (uint8_t *)entry, sizeof(AppDBEntry));
}

status_t app_db_get_app_entry_for_install_id(AppInstallId app_id, AppDBEntry *entry) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_get(&s_app_db.settings_file, (uint8_t *)&app_id, sizeof(AppInstallId),
      (uint8_t *)entry, sizeof(AppDBEntry));

  prv_close_file_and_unlock_mutex();
  return rv;
}

bool app_db_exists_install_id(AppInstallId app_id) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  bool exists = settings_file_exists(&s_app_db.settings_file, (uint8_t *)&app_id,
                                     sizeof(AppInstallId));

  prv_close_file_and_unlock_mutex();
  return exists;
}

typedef struct {
  AppDBEnumerateCb cb;
  void *data;
  AppDBEntry *entry_buf;
} EnumerateData;

static bool prv_enumerate_entries(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if ((info->val_len == 0) || (info->key_len != sizeof(AppInstallId))) {
    return true; // continue iteration
  }

  EnumerateData *cb_data = (EnumerateData *)context;

  AppInstallId id;
  info->get_key(file, (uint8_t *)&id, info->key_len);
  info->get_val(file, (uint8_t *)cb_data->entry_buf, info->val_len);

  // check return value
  cb_data->cb(id, cb_data->entry_buf, cb_data->data);

  return true; // continue iteration
}

void app_db_enumerate_entries(AppDBEnumerateCb cb, void *data) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return;
  }

  AppDBEntry *db_entry = kernel_malloc_check(sizeof(AppDBEntry));

  EnumerateData cb_data = {
    .cb = cb,
    .data = data,
    .entry_buf = db_entry,
  };
  settings_file_each(&s_app_db.settings_file, prv_enumerate_entries, &cb_data);

  prv_close_file_and_unlock_mutex();
  kernel_free(db_entry);
  return;
}

/////////////////////////
// Blob DB API
/////////////////////////

void app_db_init(void) {
  memset(&s_app_db, 0, sizeof(s_app_db));
  pbl_mutex_init(&s_app_db.mutex);

  // set to zero to reset unit test static variable.
  s_next_unique_flash_app_id = INSTALL_ID_INVALID;

  // Iterate through all entires and find the one with the highest AppInstallId. The next unique
  // is then one greater than the largest found.
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    WTF;
  }

  struct AppDBInitData data = { 0 };

  settings_file_each(&s_app_db.settings_file, prv_each_inspect_ids, &data);

  if (data.max_id == INSTALL_ID_INVALID) {
    s_next_unique_flash_app_id = (INSTALL_ID_INVALID + 1);
  } else {
    s_next_unique_flash_app_id = (data.max_id + 1);
  }

  PBL_LOG_INFO("Found %"PRIu32" apps. Next ID: %"PRIu32" ", data.num_apps,
          s_next_unique_flash_app_id);

  prv_close_file_and_unlock_mutex();
}

status_t app_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  if (key_len != UUID_SIZE ||
      val_len != sizeof(AppDBEntry)) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  PBL_ASSERTN(key_len == 16);
  PBL_ASSERTN(val_len > 0);

  bool new_install = false;
  AppInstallId app_id = prv_find_install_id_for_uuid(&s_app_db.settings_file, (const Uuid *)key);
  if (app_id == INSTALL_ID_INVALID) {
    new_install = true;
    app_id = s_next_unique_flash_app_id++;
  } else if (app_fetch_in_progress()) {
    PBL_LOG_WRN("Got an insert for an app that is currently being fetched, %"PRId32,
            app_id);
    rv = prv_cancel_app_fetch(app_id);
  }

  if (rv == S_SUCCESS) {
    rv = settings_file_set(&s_app_db.settings_file, (uint8_t *)&app_id,
                           sizeof(AppInstallId), val, val_len);
  }

  prv_close_file_and_unlock_mutex();

  if (rv == S_SUCCESS) {
    // app install something
    app_install_do_callbacks(new_install ? APP_AVAILABLE : APP_UPGRADED, app_id, NULL, NULL, NULL);
  }

  return rv;
}

int app_db_get_len(const uint8_t *key, int key_len) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  PBL_ASSERTN(key_len == 16);

  // should not increment !!!!
  AppInstallId app_id = prv_find_install_id_for_uuid(&s_app_db.settings_file, (Uuid *)key);

  if (app_id == INSTALL_ID_INVALID) {
    rv = 0;
  } else {
    rv = settings_file_get_len(&s_app_db.settings_file, (uint8_t *)&app_id, sizeof(AppInstallId));
  }

  prv_close_file_and_unlock_mutex();
  return rv;
}

status_t app_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_len) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  PBL_ASSERTN(key_len == 16);

  AppInstallId app_id = prv_find_install_id_for_uuid(&s_app_db.settings_file, (Uuid *)key);
  if (app_id == INSTALL_ID_INVALID) {
    rv = E_DOES_NOT_EXIST;
  } else {
    rv = settings_file_get(&s_app_db.settings_file, (uint8_t *)&app_id,
                           sizeof(AppInstallId), val_out, val_len);
  }

  prv_close_file_and_unlock_mutex();
  return rv;
}

status_t app_db_delete(const uint8_t *key, int key_len) {
  if (key_len != UUID_SIZE) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  PBL_ASSERTN(key_len == 16);

  AppInstallId app_id = prv_find_install_id_for_uuid(&s_app_db.settings_file, (Uuid *)key);

  if (app_id == INSTALL_ID_INVALID) {
    rv = E_DOES_NOT_EXIST;
  } else if (app_fetch_in_progress()) {
    PBL_LOG_WRN("Tried to delete an app that is currently being fetched, %"PRId32,
            app_id);
    rv = prv_cancel_app_fetch(app_id);
  }

  if (rv == S_SUCCESS) {
    rv = settings_file_delete(&s_app_db.settings_file, (uint8_t *)&app_id, sizeof(AppInstallId));
  }


  prv_close_file_and_unlock_mutex();

  if (rv == S_SUCCESS) {
    // uuid will be free'd by app_install_manager
    Uuid *uuid_copy = kernel_malloc_check(sizeof(Uuid));
    memcpy(uuid_copy, key, sizeof(Uuid));
    app_install_do_callbacks(APP_REMOVED, app_id, uuid_copy, NULL, NULL);
  }

  return rv;
}

status_t app_db_flush(void) {
  PBL_LOG_WRN("AppDB Flush initiated");

  if (app_fetch_in_progress()) {
    // cancels any app fetch
    status_t rv = prv_cancel_app_fetch(INSTALL_ID_INVALID);
    if (rv != S_SUCCESS) {
      return rv;
    }
  }

  app_install_do_callbacks(APP_DB_CLEARED, INSTALL_ID_INVALID, NULL, NULL, NULL);

  // let app install manager deal with deleting the cache and removing related timeline pins
  app_install_clear_app_db();

  // remove the settings file
  pbl_mutex_lock(&s_app_db.mutex, PBL_FOREVER);
  pfs_remove(SETTINGS_FILE_NAME);

  pbl_mutex_unlock(&s_app_db.mutex);
  PBL_LOG_WRN("AppDB Flush finished");
  return S_SUCCESS;
}

status_t app_db_compact(void) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }
  rv = settings_file_compact(&s_app_db.settings_file);
  prv_close_file_and_unlock_mutex();
  return rv;
}

//////////////////////
// Test functions
//////////////////////

// automated testing and app_install_manager prompt commands
int32_t app_db_check_next_unique_id(void) {
  return s_next_unique_flash_app_id;
}
