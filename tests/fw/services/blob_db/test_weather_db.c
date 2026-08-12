/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/util/attributes.h"
#include "util/pstring.h"

#include "pbl/services/blob_db/weather_db.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/weather/weather_types.h"
#include "weather_data_shared.h"

// Fixture
////////////////////////////////////////////////////////////////

// Fakes
////////////////////////////////////////////////////////////////
#include "fake_pbl_malloc.h"
#include "fake_spi_flash.h"

// Stubs
////////////////////////////////////////////////////////////////
#include "stubs_analytics.h"
#include "stubs_hexdump.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_prompt.h"
#include "stubs_task_watchdog.h"
#include "stubs_pebble_tasks.h"
#include "stubs_sleep.h"

bool weather_service_supported_by_phone(void) {
  return true;
}
// Setup
////////////////////////////////////////////////////////////////

void test_weather_db__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  weather_db_init();
  weather_shared_data_init();
}

void test_weather_db__cleanup(void) {
  weather_shared_data_cleanup();
}

// Tests
////////////////////////////////////////////////////////////////
static void prv_db_iterator_cb(WeatherDBKey *key, WeatherDBEntry *entry, void *unused) {
  weather_shared_data_assert_entries_equal(key, entry,
      weather_shared_data_get_entry(weather_shared_data_get_index_of_key(key)));
}

void test_weather_db__get_entries(void) {
  cl_assert_equal_i(S_SUCCESS, weather_db_for_each(prv_db_iterator_cb, NULL));
}

void test_weather_db__check_records_in_db(void) {
  for (int index = 0; index < WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES; index++) {
    WeatherDBEntry *to_check = task_zalloc_check(weather_shared_data_get_entry_size(index));
    const WeatherDBKey *key = weather_shared_data_get_key(index);
    cl_assert_equal_i(S_SUCCESS, weather_db_read((uint8_t*)key,
                                                 sizeof(WeatherDBKey),
                                                 (uint8_t*)to_check,
                                                 weather_shared_data_get_entry_size(index)));

    WeatherDBEntry *original = weather_shared_data_get_entry(index);
    weather_shared_data_assert_entries_equal(key, to_check, original);
    task_free(to_check);
  }
}

void test_weather_db__check_small_record_not_inserted(void) {
  const size_t entry_size = MIN_ENTRY_SIZE - 1;
  void *entry = task_zalloc_check(entry_size);
  WeatherDBKey key = (WeatherDBKey) {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5
  };

  cl_assert_equal_i(E_INVALID_ARGUMENT, weather_db_insert((uint8_t*)&key,
                                                          sizeof(WeatherDBKey),
                                                          (uint8_t*)entry,
                                                          entry_size));
  task_free(entry);
}

void test_weather_db__check_too_large_record_not_inserted(void) {
  const size_t entry_size = MAX_ENTRY_SIZE + 1;
  void *entry = task_zalloc_check(entry_size);
  WeatherDBKey key = (WeatherDBKey) {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5
  };

  cl_assert_equal_i(E_INVALID_ARGUMENT, weather_db_insert((uint8_t*)&key,
                                                          sizeof(WeatherDBKey),
                                                          (uint8_t*)entry,
                                                          entry_size));
  task_free(entry);
}

static void prv_check_invalid_version_code_not_inserted(uint8_t version) {
  const WeatherDBEntry *existing_entry = weather_shared_data_get_entry(0);
  const size_t entry_size = sizeof(*existing_entry);

  WeatherDBEntry *new_entry = task_zalloc_check(entry_size);
  *new_entry = *existing_entry;
  new_entry->version = version;

  WeatherDBKey key = (WeatherDBKey) {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5
  };
  cl_assert_equal_i(E_INVALID_ARGUMENT, weather_db_insert((uint8_t*)&key,
                                                          sizeof(WeatherDBKey),
                                                          (uint8_t*)new_entry,
                                                          entry_size));
  task_free(new_entry);
}

void test_weather_db__lower_version_not_inserted(void) {
  // v3 is the supported LEGACY version — only versions below it are invalid.
  for (size_t version = 0; version < WEATHER_DB_LEGACY_VERSION; version++) {
    prv_check_invalid_version_code_not_inserted(version);
  }
}

void test_weather_db__higher_version_not_inserted(void) {
  prv_check_invalid_version_code_not_inserted(WEATHER_DB_CURRENT_VERSION + 1);
}

status_t weather_db_get_num_keys(uint16_t *val_out);

void test_weather_db__test_get_num_keys(void) {
  uint16_t num_keys;
  cl_assert_equal_i(S_SUCCESS, weather_db_get_num_keys(&num_keys));
  cl_assert_equal_i(num_keys, WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES);
}

status_t weather_db_get_keys(WeatherDBKey *keys);

void test_weather_db__test_get_keys(void) {
  WeatherDBKey keys[WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES];
  cl_assert_equal_i(S_SUCCESS, weather_db_get_keys(keys));

  for(int x = 0; x < WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES; x++) {
    cl_assert(weather_shared_data_get_key_exists(&keys[x]));
  }
}

void test_weather_db__read_stale_entries(void) {
  WeatherDBKey key = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
  size_t entry_size = weather_shared_data_insert_stale_entry(&key);
  uint8_t *buf = task_zalloc_check(entry_size);

  cl_assert_equal_i(E_DOES_NOT_EXIST, weather_db_read((uint8_t*)&key,
                                                      sizeof(WeatherDBKey),
                                                      buf,
                                                      entry_size));
}

// Structural validation of the phone-controlled record layout
////////////////////////////////////////////////////////////////

// Build a fully valid v4 (current-minor) record with a proper trailing strings
// block, returning its total size. Caller owns the buffer.
static WeatherDBEntry *prv_create_valid_v4_entry(size_t *size_out) {
  const WeatherDBEntry *base = weather_shared_data_get_entry(0);
  const size_t entry_size = weather_shared_data_get_entry_size(0);
  WeatherDBEntry *entry = task_zalloc_check(entry_size);
  memcpy(entry, base, entry_size);
  *size_out = entry_size;
  return entry;
}

static const WeatherDBKey s_probe_key = {
  9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9
};

// A record truncated exactly at the strings offset (no SerializedArray header
// at all) used to be accepted, sending the strings resolver out of bounds.
void test_weather_db__truncated_at_strings_offset_not_inserted(void) {
  size_t size;
  WeatherDBEntry *entry = prv_create_valid_v4_entry(&size);

  const size_t truncated_size =
      weather_db_entry_strings_offset(WEATHER_DB_CURRENT_VERSION,
                                      WEATHER_DB_CURRENT_MINOR_VERSION);
  cl_assert_equal_i(E_INVALID_ARGUMENT,
                    weather_db_insert((uint8_t *)&s_probe_key, sizeof(WeatherDBKey),
                                      (uint8_t *)entry, truncated_size));
  task_free(entry);
}

// SerializedArray.data_size is phone-controlled: it must never claim more
// bytes than the record actually holds.
void test_weather_db__inflated_data_size_not_inserted(void) {
  size_t size;
  WeatherDBEntry *entry = prv_create_valid_v4_entry(&size);

  SerializedArray *sa = weather_db_entry_get_strings(entry);
  sa->data_size = 0xFFFF;
  cl_assert_equal_i(E_INVALID_ARGUMENT,
                    weather_db_insert((uint8_t *)&s_probe_key, sizeof(WeatherDBKey),
                                      (uint8_t *)entry, size));
  task_free(entry);
}

// Each PascalString16.str_length is phone-controlled too: a string may not run
// past the end of the strings block.
void test_weather_db__oversized_pstring_not_inserted(void) {
  size_t size;
  WeatherDBEntry *entry = prv_create_valid_v4_entry(&size);

  SerializedArray *sa = weather_db_entry_get_strings(entry);
  uint16_t huge = 0xFFF0;
  memcpy(sa->data, &huge, sizeof(huge));   // first pstring's length
  cl_assert_equal_i(E_INVALID_ARGUMENT,
                    weather_db_insert((uint8_t *)&s_probe_key, sizeof(WeatherDBKey),
                                      (uint8_t *)entry, size));
  task_free(entry);
}

// A minor this firmware never shipped places its trailing strings at an
// unknown offset — it must be refused, not clamped to the newest known one.
void test_weather_db__higher_minor_not_inserted(void) {
  size_t size;
  WeatherDBEntry *entry = prv_create_valid_v4_entry(&size);

  entry->minor_version = WEATHER_DB_CURRENT_MINOR_VERSION + 1;
  cl_assert_equal_i(E_INVALID_ARGUMENT,
                    weather_db_insert((uint8_t *)&s_probe_key, sizeof(WeatherDBKey),
                                      (uint8_t *)entry, size));
  task_free(entry);
}

// Back-compat: an older-minor record (fixed fields end earlier, strings block
// directly after) must still be accepted — phones that have not shipped the
// newest appended block keep working.
void test_weather_db__older_minor_still_inserted(void) {
  size_t size;
  WeatherDBEntry *entry = prv_create_valid_v4_entry(&size);

  const size_t fixed = weather_db_entry_strings_offset(WEATHER_DB_CURRENT_VERSION, 0);
  // Two empty pstrings: header + 2x a bare 2-byte length.
  const uint16_t data_size = 2 * sizeof(uint16_t);
  const size_t v40_size = fixed + sizeof(SerializedArray) + data_size;
  uint8_t *v40 = task_zalloc_check(v40_size);
  memcpy(v40, entry, fixed);
  ((WeatherDBEntry *)v40)->minor_version = 0;
  memcpy(v40 + fixed, &data_size, sizeof(uint16_t));   // SerializedArray.data_size

  cl_assert_equal_i(S_SUCCESS,
                    weather_db_insert((uint8_t *)&s_probe_key, sizeof(WeatherDBKey),
                                      v40, v40_size));
  cl_assert_equal_i(S_SUCCESS,
                    weather_db_delete((uint8_t *)&s_probe_key, sizeof(WeatherDBKey)));
  task_free(v40);
  task_free(entry);
}

// A read that fails underneath (missing key) must surface the real status and
// must NOT trip the unsupported-entry self-clean on the zeroed buffer.
void test_weather_db__read_missing_key_returns_real_error(void) {
  uint8_t buf[MIN_ENTRY_SIZE];
  cl_assert_equal_i(E_DOES_NOT_EXIST,
                    weather_db_read((uint8_t *)&s_probe_key, sizeof(WeatherDBKey),
                                    buf, sizeof(buf)));
}
