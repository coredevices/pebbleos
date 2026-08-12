/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/weather_db.h"

#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/weather/weather_service.h"
#include "pbl/services/weather/weather_types.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/units.h"

#include <string.h>

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

// A weather record's trailing strings are self-described (SerializedArray header
// + PascalString16s) and every length in them is phone-controlled. Validate the
// whole record against its real byte length before anything parses it: insert
// calls this at the phone trust boundary, and the read paths call it again so
// records stored before this gate existed can never reach the parsers.
static bool prv_entry_is_well_formed(const uint8_t *val, int val_len) {
  if (val_len < (int)MIN_ENTRY_SIZE || val_len > (int)MAX_ENTRY_SIZE) {
    return false;
  }
  const WeatherDBEntry *entry = (const WeatherDBEntry *)val;
  if (!weather_db_version_is_supported(entry->version)) {
    return false;
  }
  uint8_t minor = 0;
  if (entry->version == WEATHER_DB_CURRENT_VERSION) {
    // A minor this firmware never shipped could place the trailing strings
    // anywhere — refuse it rather than mis-locate them (same policy as the
    // major-version gate; a phone only writes v4 minors the watch advertises).
    // Older minors must keep working (weather_db_entry_strings_offset handles
    // every shipped rung).
    if (entry->minor_version > WEATHER_DB_CURRENT_MINOR_VERSION) {
      return false;
    }
    minor = entry->minor_version;
  }
  // The fixed fields for ITS minor plus the strings header must be present
  // (the bare strings offset admits a record truncated exactly at the header,
  // which sends the resolver reading out of bounds)...
  const size_t strings_off = weather_db_entry_strings_offset(entry->version, minor);
  if ((size_t)val_len < strings_off + sizeof(SerializedArray)) {
    return false;
  }
  // ...the self-described block must fit inside the record...
  const SerializedArray *sa = (const SerializedArray *)(val + strings_off);
  if ((size_t)val_len < strings_off + sizeof(SerializedArray) + sa->data_size) {
    return false;
  }
  // ...and every pstring must land inside the block (empty ones serialize as
  // just their 2-byte length; the last must end exactly at the block's end).
  const uint8_t *p = sa->data;
  size_t remaining = sa->data_size;
  while (remaining > 0) {
    if (remaining < sizeof(uint16_t)) {
      return false;
    }
    uint16_t str_length;
    memcpy(&str_length, p, sizeof(str_length));
    if (remaining < sizeof(uint16_t) + (size_t)str_length) {
      return false;
    }
    p += sizeof(uint16_t) + str_length;
    remaining -= sizeof(uint16_t) + (size_t)str_length;
  }
  return true;
}

static bool prv_weather_db_for_each_cb(SettingsFile *file, SettingsRecordInfo *info,
                                       void *context) {
  if ((info->val_len == 0) || (info->key_len != sizeof(WeatherDBKey))) {
    return true;
  }

  WeatherDBKey key;
  info->get_key(file, &key, info->key_len);

  WeatherDBEntry *entry = task_zalloc_check(info->val_len);
  info->get_val(file, entry, info->val_len);
  // Same structural validation as insert: a record stored before the insert
  // gate existed (or corrupted at rest) must not reach the string parsers.
  if (!prv_entry_is_well_formed((const uint8_t *)entry, info->val_len)) {
    PBL_LOG_WRN("Skipping malformed weather entry (version %" PRIu8 ", len %d)",
                entry->version, info->val_len);
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
  if (!weather_db_version_is_supported(entry->version)) {
    PBL_LOG_WRN("Unsupported weather entry version on insert: %" PRIu8, entry->version);
    return E_INVALID_ARGUMENT;
  }
  // Full structural validation: per-minor fixed size (per-MINOR, never "the
  // newest" — older phone apps keep working), unknown-future minors, and the
  // phone-controlled string-block lengths, all bounded against val_len.
  if (!prv_entry_is_well_formed(val, val_len)) {
    PBL_LOG_WRN("Malformed v%" PRIu8 " weather record rejected on insert (len %d)",
                entry->version, val_len);
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
  // Only inspect the buffer after a successful fetch — on failure settings_file
  // leaves it zeroed, and treating that as "unsupported entry" used to delete a
  // possibly-valid record (e.g. on E_RANGE) while masking the real status.
  if (rv == S_SUCCESS && !prv_entry_is_well_formed(val_out, val_out_len)) {
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
