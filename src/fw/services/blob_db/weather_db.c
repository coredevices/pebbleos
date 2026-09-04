/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/weather_db.h"

#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/weather/weather_service.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/units.h"

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

#define SETTINGS_FILE_NAME "weatherdb"

#define SETTINGS_FILE_SIZE (KiBYTES(30))

static struct {
  SettingsFile settings_file;
  PebbleMutex *mutex;
} s_weather_db;

typedef struct WeatherDBIteratorData {
  WeatherDBIteratorCallback cb;
  void *cb_ctx;
} WeatherDBIteratorData;

///////////////////////////
// Weather DB API
///////////////////////////

static status_t prv_lock_mutex_and_open_file(void) {
  mutex_lock(s_weather_db.mutex);
  status_t rv = settings_file_open_growable(&s_weather_db.settings_file,
                                            SETTINGS_FILE_NAME,
                                            SETTINGS_FILE_SIZE,
                                            KiBYTES(4));
  if (rv != S_SUCCESS) {
    mutex_unlock(s_weather_db.mutex);
  }
  return rv;
}

static void prv_close_file_and_unlock_mutex(void) {
  settings_file_close(&s_weather_db.settings_file);
  mutex_unlock(s_weather_db.mutex);
}

// Every byte past the fixed fields is phone-controlled, so the trailing string
// block must be fully bounded by val_len before a record is accepted: the
// SerializedArray header, its data_size, and every pstring16 a reader can be
// handed. The walk mirrors pstring.c (traversal advances by the LOW byte of a
// pstring's length; the full uint16 length is what gets read back out).
static bool prv_strings_block_is_valid(const uint8_t *val, size_t val_len,
                                       size_t strings_offset) {
  if (strings_offset + sizeof(SerializedArray) > val_len) {
    return false;
  }
  const SerializedArray *strings = (const SerializedArray *)(val + strings_offset);
  const size_t block_size = strings->data_size;
  if (block_size > val_len - strings_offset - sizeof(SerializedArray)) {
    return false;
  }
  size_t off = 0;
  do {
    if (off + sizeof(uint16_t) > block_size) {
      return false;
    }
    uint16_t str_length;
    memcpy(&str_length, &strings->data[off], sizeof(str_length));
    if (str_length > block_size - off - sizeof(uint16_t)) {
      return false;
    }
    off += strings->data[off] + sizeof(uint16_t);
  } while (block_size - off >= sizeof(uint16_t));
  return true;
}

static bool prv_weather_db_for_each_cb(SettingsFile *file, SettingsRecordInfo *info,
                                       void *context) {
  if ((info->val_len < (int)MIN_ENTRY_SIZE) || (info->key_len != sizeof(WeatherDBKey))) {
    return true;
  }

  WeatherDBKey key;
  info->get_key(file, &key, info->key_len);

  WeatherDBEntry *entry = task_zalloc_check(info->val_len);
  info->get_val(file, entry, info->val_len);
  if (!weather_db_entry_is_supported(entry)) {
    PBL_LOG_WRN("Unsupported weather entry version: %" PRIu8, entry->version);
    goto cleanup;
  }

  const WeatherDBIteratorData *cb_data = context;
  cb_data->cb(&key, entry, cb_data->cb_ctx);

cleanup:
  task_free(entry);
  return true;
}

status_t weather_db_for_each(WeatherDBIteratorCallback callback, void *context) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  WeatherDBIteratorData data = (WeatherDBIteratorData) {
    .cb = callback,
    .cb_ctx = context
  };

  settings_file_each(&s_weather_db.settings_file,
                     prv_weather_db_for_each_cb,
                     &data);

  prv_close_file_and_unlock_mutex();
  return S_SUCCESS;
}

/////////////////////////
// Blob DB API
/////////////////////////

void weather_db_init(void) {
  memset(&s_weather_db, 0, sizeof(s_weather_db));

  s_weather_db.mutex = mutex_create();
}

status_t weather_db_flush(void) {
  if (!weather_service_supported_by_phone()) {
    // return E_RANGE, so the phone receives BLOB_DB_INVALID_DATABASE_ID and stops sending
    // unwelcome weather records
    return E_RANGE;
  }
  mutex_lock(s_weather_db.mutex);
  pfs_remove(SETTINGS_FILE_NAME);
  mutex_unlock(s_weather_db.mutex);

  return S_SUCCESS;
}

status_t weather_db_compact(void) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }
  rv = settings_file_compact(&s_weather_db.settings_file);
  prv_close_file_and_unlock_mutex();
  return rv;
}

status_t weather_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  if (!weather_service_supported_by_phone()) {
    return E_RANGE;
  }
  if (key_len != sizeof(WeatherDBKey) ||
      val_len < (int) MIN_ENTRY_SIZE ||
      val_len > (int) MAX_ENTRY_SIZE) {
    return E_INVALID_ARGUMENT;
  }

  const WeatherDBEntry *entry = (WeatherDBEntry *)val;
  if (!weather_db_entry_is_supported(entry)) {
    PBL_LOG_WRN("Unsupported weather entry on insert: %" PRIu8, entry->version);
    return E_INVALID_ARGUMENT;
  }
  // The record must carry the fixed fields for ITS version+minor (per-MINOR, never
  // "the newest": demanding the current minor's size rejects every record from a
  // phone that has not shipped the latest block yet), and the trailing string block
  // must lie fully inside val_len.
  const uint8_t minor =
      (entry->version >= WEATHER_DB_CURRENT_VERSION) ? entry->minor_version : 0;
  const size_t strings_offset = weather_db_entry_strings_offset(entry->version, minor);
  if (!prv_strings_block_is_valid(val, (size_t)val_len, strings_offset)) {
    PBL_LOG_WRN("Malformed v%" PRIu8 ".%" PRIu8 " weather record (len %d)",
                entry->version, minor, val_len);
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_set(&s_weather_db.settings_file, key, key_len, val, val_len);

  prv_close_file_and_unlock_mutex();
  return rv;
}

int weather_db_get_len(const uint8_t *key, int key_len) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return 0;
  }

  PBL_ASSERTN(key_len == sizeof(WeatherDBKey));

  int entry_len = settings_file_get_len(&s_weather_db.settings_file, key, key_len);

  prv_close_file_and_unlock_mutex();
  return entry_len;
}

status_t weather_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  PBL_ASSERTN(key_len == sizeof(WeatherDBKey));

  rv = settings_file_get(&s_weather_db.settings_file, key, key_len, val_out, val_out_len);
  if ((rv == S_SUCCESS) && !weather_db_entry_is_supported((WeatherDBEntry *)val_out)) {
    // We might as well clear out the unparseable entry
    PBL_LOG_WRN("Read an unsupported weather DB entry");
    settings_file_delete(&s_weather_db.settings_file, key, key_len);
    rv = E_DOES_NOT_EXIST;
  }

  prv_close_file_and_unlock_mutex();
  return rv;
}

status_t weather_db_delete(const uint8_t *key, int key_len) {
  if (!weather_service_supported_by_phone()) {
    return E_RANGE;
  }
  if (key_len != sizeof(WeatherDBKey)) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  if (!settings_file_exists(&s_weather_db.settings_file, key, key_len)) {
    prv_close_file_and_unlock_mutex();
    return E_DOES_NOT_EXIST;
  }

  rv = settings_file_delete(&s_weather_db.settings_file, key, key_len);

  prv_close_file_and_unlock_mutex();
  return rv;
}

//-----------------------------------------------------------------------------
// Testing code only

#if UNITTEST
// SettingsFile Helpers
typedef struct {
  uint16_t key_count;
  WeatherDBKey *keys;
} SettingsFileEachKeyHelper;

static bool prv_each_inspect_keys(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  if ((info->val_len == 0) || (info->key_len != sizeof(WeatherDBKey))) {
    // Invalid key, continue iterating
    return true;
  }

  SettingsFileEachKeyHelper *key_helper = context;

  if (key_helper->keys != NULL) {
    info->get_key(file, (uint8_t *)&key_helper->keys[key_helper->key_count], sizeof(WeatherDBKey));
  }

  key_helper->key_count++;

  // Continue iterating
  return true;
}

status_t weather_db_get_num_keys(uint16_t *val_out) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  SettingsFileEachKeyHelper key_helper = {
    .key_count = 0,
    .keys = NULL,
  };
  settings_file_each(&s_weather_db.settings_file, prv_each_inspect_keys, &key_helper);
  *val_out = key_helper.key_count;

  prv_close_file_and_unlock_mutex();
  return S_SUCCESS;
}

status_t weather_db_get_keys(WeatherDBKey *keys) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  SettingsFileEachKeyHelper key_helper = {
    .key_count = 0,
    .keys = keys,
  };
  settings_file_each(&s_weather_db.settings_file, prv_each_inspect_keys, &key_helper);

  prv_close_file_and_unlock_mutex();
  return S_SUCCESS;
}

status_t weather_db_insert_stale(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  // Quick and dirty insert which doesn't do any error checking. Used to insert stale entries
  // for testing
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_set(&s_weather_db.settings_file, key, key_len, val, val_len);

  prv_close_file_and_unlock_mutex();
  return rv;
}
#endif
