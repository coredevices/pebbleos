/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "weather_data_shared.h"

#include "clar_asserts.h"

#include <pbl/drivers/rtc.h>
#include "kernel/pbl_malloc.h"
#include "pbl/services/blob_db/watch_app_prefs_db.h"
#include "pbl/services/blob_db/weather_db.h"
#include "pbl/services/weather/weather_service_private.h"

#define WEATHER_PREFS_DATA_SIZE (sizeof(SerializedWeatherAppPrefs) + \
                                (sizeof(Uuid) * WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES))
static uint8_t s_weather_app_prefs[WEATHER_PREFS_DATA_SIZE];

static const WeatherDBKey s_keys[] = {
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
  },
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2
  },
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3
  },
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4
  },
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5
  },
};

static WeatherDBEntry *s_entries[WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES];
static size_t s_entry_sizes[WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES];

static char *s_entry_names[] = {
  TEST_WEATHER_DB_LOCATION_PALO_ALTO,
  TEST_WEATHER_DB_LOCATION_KITCHENER,
  TEST_WEATHER_DB_LOCATION_WATERLOO,
  TEST_WEATHER_DB_LOCATION_RWC,
  TEST_WEATHER_DB_LOCATION_SF,
};

static char *s_entry_phrases[] = {
  TEST_WEATHER_DB_SHORT_PHRASE_SUNNY,
  TEST_WEATHER_DB_SHORT_PHRASE_PARTLY_CLOUDY,
  TEST_WEATHER_DB_SHORT_PHRASE_HEAVY_SNOW,
  TEST_WEATHER_DB_SHORT_PHRASE_HEAVY_RAIN,
  TEST_WEATHER_DB_SHORT_PHRASE_PARTLY_CLOUDY,
};

static const WeatherDBEntry s_entry_bases[] = {
  {
    .version = WEATHER_DB_CURRENT_VERSION,
    .is_current_location = true,
    .current_temp = 68,
    .current_weather_type = WeatherType_Sun,
    .today_high_temp = 68,
    .today_low_temp = 52,
    .tomorrow_weather_type = WeatherType_CloudyDay,
    .tomorrow_high_temp = 70,
    .tomorrow_low_temp = 60,
  },
  {
    .version = WEATHER_DB_CURRENT_VERSION,
    .is_current_location = false,
    .current_temp = -10,
    .current_weather_type = WeatherType_PartlyCloudy,
    .today_high_temp = 0,
    .today_low_temp = -11,
    .tomorrow_weather_type = WeatherType_CloudyDay,
    .tomorrow_high_temp = 2,
    .tomorrow_low_temp = -3,
  },
  {
    .version = WEATHER_DB_CURRENT_VERSION,
      .is_current_location = false,
    .current_temp = -99,
    .current_weather_type = WeatherType_HeavySnow,
    .today_high_temp = -98,
    .today_low_temp = -99,
    .tomorrow_weather_type = WeatherType_Sun,
    .tomorrow_high_temp = 2,
    .tomorrow_low_temp = 1,
  },
  {
    .version = WEATHER_DB_CURRENT_VERSION,
    .is_current_location = true,
    .current_temp = 60,
    .current_weather_type = WeatherType_HeavyRain,
    .today_high_temp = 70,
    .today_low_temp = 50,
    .tomorrow_weather_type = WeatherType_PartlyCloudy,
    .tomorrow_high_temp = 70,
    .tomorrow_low_temp = 60,
  },
  {
    .version = WEATHER_DB_CURRENT_VERSION,
    .is_current_location = true,
    .current_temp = 60,
    .current_weather_type = WeatherType_PartlyCloudy,
    .today_high_temp = 70,
    .today_low_temp = 50,
    .tomorrow_weather_type = WeatherType_PartlyCloudy,
    .tomorrow_high_temp = 70,
    .tomorrow_low_temp = 60,
  }
};

// Fake out watch_app_prefs calls
void watch_app_prefs_destroy_weather(SerializedWeatherAppPrefs *prefs) {}

SerializedWeatherAppPrefs *watch_app_prefs_get_weather(void) {
  SerializedWeatherAppPrefs *prefs = (SerializedWeatherAppPrefs *) s_weather_app_prefs;
  prefs->num_locations = WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES;
  for (int idx = 0; idx < WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES; idx++) {
    prefs->locations[idx] = s_keys[idx];
  }
  return prefs;
}

static WeatherDBEntry *prv_create_entry(const WeatherDBEntry *base_entry, char *location,
                                        char *phrase, size_t *size_out) {
  PascalString16List pstring16_list;
  PascalString16 *location_name;
  PascalString16 *short_phrase;
  size_t data_size;

  location_name = pstring_create_pstring16_from_string(location);
  short_phrase = pstring_create_pstring16_from_string(phrase);

  data_size = strlen(location) +
              strlen(phrase) +
              sizeof(uint16_t) * 2; // One for each string

  const size_t entry_size = sizeof(WeatherDBEntry) + data_size;
  WeatherDBEntry *entry = task_zalloc_check(entry_size);
  *entry = *base_entry;
  // The strings are appended after the FULL current struct, so the record must
  // carry the current minor — a lower minor moves the strings offset forward.
  entry->minor_version = WEATHER_DB_CURRENT_MINOR_VERSION;
  entry->pstring16s.data_size = data_size;
  entry->last_update_time_utc = rtc_get_time();

  pstring_project_list_on_serialized_array(&pstring16_list, &entry->pstring16s);
  pstring_add_pstring16_to_list(&pstring16_list, location_name);
  pstring_add_pstring16_to_list(&pstring16_list, short_phrase);

  pstring_destroy_pstring16(location_name);
  pstring_destroy_pstring16(short_phrase);

  *size_out = entry_size;
  return entry;
}

static void prv_initialize_entries(void) {
  for (int idx = 0; idx < WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES; idx++) {
    WeatherDBEntry *entry = prv_create_entry(&s_entry_bases[idx],
                                             s_entry_names[idx],
                                             s_entry_phrases[idx],
                                             &s_entry_sizes[idx]);

    // Make the last entry contain a timestamp that is too old to be included in weather_service
    // forecast list
    if (idx == WEATHER_DATA_SHARED_NUM_VALID_TIMESTAMP_ENTRIES) {
      entry->last_update_time_utc = (time_start_of_today() - SECONDS_PER_DAY - 1);
    }
    cl_assert_equal_i(S_SUCCESS, weather_db_insert((uint8_t*)&s_keys[idx],
                                                   sizeof(WeatherDBKey),
                                                   (uint8_t*)entry,
                                                   s_entry_sizes[idx]));
    s_entries[idx] = entry;
  }
}

void weather_shared_data_init(void) {
  rtc_set_time(1461765790); // 2016-04-27T14:03:10+00:00
  prv_initialize_entries();
}

void weather_shared_data_cleanup(void) {
  for (int i = 0; i < WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES; i++) {
    if (s_entries[i]) {
      task_free(s_entries[i]);
      s_entries[i] = NULL;
    }
  }

  // Flush DB
  cl_assert_equal_i(S_SUCCESS, weather_db_flush());
}


const WeatherDBKey *weather_shared_data_get_key(int index) {
  return &s_keys[index];
}

WeatherDBEntry *weather_shared_data_get_entry(int index) {
  return s_entries[index];
}

size_t weather_shared_data_get_entry_size(int index) {
  return s_entry_sizes[index];
}

char *weather_shared_data_get_entry_name(int index) {
  return s_entry_names[index];
}

char *weather_shared_data_get_entry_phrase(int index) {
  return s_entry_phrases[index];
}

int weather_shared_data_get_index_of_key(const WeatherDBKey *key) {
  for (int idx = 0; idx < WEATHER_DATA_SHARED_WEATHER_DB_NUM_DB_ENTRIES; idx++) {
    if (uuid_equal(key, &s_keys[idx])) {
      return idx;
    }
  }
  return -1;
}

void weather_shared_data_assert_entries_equal(const WeatherDBKey *key, WeatherDBEntry *to_check,
                                              WeatherDBEntry *original) {
  cl_assert_equal_i(to_check->version, original->version);
  cl_assert_equal_b(to_check->is_current_location, original->is_current_location);
  cl_assert_equal_i(to_check->current_temp, original->current_temp);
  cl_assert_equal_i(to_check->current_weather_type, original->current_weather_type);
  cl_assert_equal_i(to_check->today_high_temp, original->today_high_temp);
  cl_assert_equal_i(to_check->today_low_temp, original->today_low_temp);
  cl_assert_equal_i(to_check->tomorrow_weather_type, original->tomorrow_weather_type);
  cl_assert_equal_i(to_check->tomorrow_high_temp, original->tomorrow_high_temp);
  cl_assert_equal_i(to_check->tomorrow_low_temp, original->tomorrow_low_temp);
  cl_assert_equal_i(to_check->last_update_time_utc, original->last_update_time_utc);

  PascalString16List pstring16_list;
  pstring_project_list_on_serialized_array(&pstring16_list, &to_check->pstring16s);
  cl_assert_equal_i(pstring16_list.count, 2);

  PascalString16 *pstring;

  pstring = pstring_get_pstring16_from_list(&pstring16_list, 0);

  int index = weather_shared_data_get_index_of_key(key);
  if (index == -1) {
    cl_fail("key not found!");
  }

  cl_assert_equal_i(pstring->str_length, strlen(s_entry_names[index]));
  char loc[WEATHER_SERVICE_MAX_WEATHER_LOCATION_BUFFER_SIZE];
  pstring_pstring16_to_string(pstring, loc);
  cl_assert_equal_s(loc, s_entry_names[index]);

  pstring = pstring_get_pstring16_from_list(&pstring16_list, 1);
  cl_assert_equal_i(pstring->str_length, strlen(s_entry_phrases[index]));
  char phrase[WEATHER_SERVICE_MAX_SHORT_PHRASE_BUFFER_SIZE];
  pstring_pstring16_to_string(pstring, phrase);
  cl_assert_equal_s(phrase, s_entry_phrases[index]);
}

bool weather_shared_data_get_key_exists(WeatherDBKey *key) {
  return weather_shared_data_get_index_of_key(key) != -1;
}

status_t weather_db_insert_stale(const uint8_t *key, int key_len, const uint8_t *val, int val_len);

size_t weather_shared_data_insert_stale_entry(WeatherDBKey *key) {
  const WeatherDBEntry stale_entry = {
    // v3 is still parseable (legacy support); only older versions are stale.
    .version = WEATHER_DB_LEGACY_VERSION - 1,
    .is_current_location = true,
    .current_temp = 68,
    .current_weather_type = WeatherType_Sun,
    .today_high_temp = 68,
    .today_low_temp = 52,
    .tomorrow_weather_type = WeatherType_CloudyDay,
    .tomorrow_high_temp = 70,
    .tomorrow_low_temp = 60,
  };

  WeatherDBEntry *entry = prv_create_entry(&stale_entry,
                                           s_entry_names[0],
                                           s_entry_phrases[0],
                                           &s_entry_sizes[0]);

  cl_assert_equal_i(S_SUCCESS, weather_db_insert_stale((uint8_t*)key,
                                                       sizeof(WeatherDBKey),
                                                       (uint8_t*)entry,
                                                       s_entry_sizes[0]));
  return s_entry_sizes[0];
}
