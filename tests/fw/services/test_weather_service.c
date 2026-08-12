/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "./blob_db/weather_data_shared.h"

#include "applib/event_service_client.h"
#include "kernel/events.h"
#include "pbl/services/comm_session/session_remote_version.h"
#include "pbl/services/blob_db/weather_db.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/weather/weather_service.h"
#include "pbl/services/weather/weather_service_private.h"
#include "pbl/services/weather/weather_types.h"
#include "util/pstring.h"

// Fixture
////////////////////////////////////////////////////////////////

// Fakes
////////////////////////////////////////////////////////////////
#include "fake_pbl_malloc.h"
#include "fake_spi_flash.h"

// Stubs
////////////////////////////////////////////////////////////////
#include "stubs_analytics.h"
#include "stubs_events.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_hexdump.h"
#include "stubs_passert.h"
#include "stubs_prompt.h"
#include "stubs_task_watchdog.h"
#include "stubs_pebble_tasks.h"
#include "stubs_sleep.h"

static EventServiceInfo *s_event_info;
void event_service_client_subscribe(EventServiceInfo *service_info) {
  s_event_info = service_info;
}

void event_service_client_unsubscribe(EventServiceInfo *service_info) {}

void bt_persistent_storage_get_cached_system_capabilities(
    PebbleProtocolCapabilities *capabilities) {
  capabilities->weather_app_support = 1;
}

void test_weather_service__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  weather_db_init();
  weather_service_init();
  weather_shared_data_init();
}

void test_weather_service__cleanup(void) {
  weather_shared_data_cleanup();
}

static const WeatherLocationForecast s_forecasts[] = {
  {
    .location_name = TEST_WEATHER_DB_LOCATION_PALO_ALTO,
    .is_current_location = true,
    .current_temp = 68,
    .today_high = 68,
    .today_low = 52,
    .current_weather_type = WeatherType_Sun,
    .current_weather_phrase = TEST_WEATHER_DB_SHORT_PHRASE_SUNNY,
    .tomorrow_high = 70,
    .tomorrow_low = 60,
    .tomorrow_weather_type = WeatherType_CloudyDay
  },
  {
    .location_name = TEST_WEATHER_DB_LOCATION_KITCHENER,
    .is_current_location = false,
    .current_temp = -10,
    .today_high = 0,
    .today_low = -11,
    .current_weather_type = WeatherType_PartlyCloudy,
    .current_weather_phrase = TEST_WEATHER_DB_SHORT_PHRASE_PARTLY_CLOUDY,
    .tomorrow_high = 2,
    .tomorrow_low = -3,
    .tomorrow_weather_type = WeatherType_CloudyDay
  },
  {
    .location_name = TEST_WEATHER_DB_LOCATION_WATERLOO,
    .is_current_location = false,
    .current_temp = -99,
    .today_high = -98,
    .today_low = -99,
    .current_weather_type = WeatherType_HeavySnow,
    .current_weather_phrase = TEST_WEATHER_DB_SHORT_PHRASE_HEAVY_SNOW,
    .tomorrow_high = 2,
    .tomorrow_low = 1,
    .tomorrow_weather_type = WeatherType_Sun
  },
  {
    .location_name = TEST_WEATHER_DB_LOCATION_RWC,
    .is_current_location = true,
    .current_temp = 60,
    .today_high = 70,
    .today_low = 50,
    .current_weather_type = WeatherType_HeavyRain,
    .current_weather_phrase = TEST_WEATHER_DB_SHORT_PHRASE_HEAVY_RAIN,
    .tomorrow_high = 70,
    .tomorrow_low = 60,
    .tomorrow_weather_type = WeatherType_PartlyCloudy
  }
};

static void prv_assert_forecast_equal(const WeatherLocationForecast *to_check,
                                      const WeatherLocationForecast *original) {
  cl_assert_equal_s(to_check->location_name, original->location_name);
  cl_assert_equal_b(to_check->is_current_location, original->is_current_location);
  cl_assert_equal_i(to_check->current_temp, original->current_temp);
  cl_assert_equal_i(to_check->today_high, original->today_high);
  cl_assert_equal_i(to_check->today_low, original->today_low);
  cl_assert_equal_i(to_check->current_weather_type, original->current_weather_type);
  cl_assert_equal_s(to_check->current_weather_phrase, original->current_weather_phrase);
  cl_assert_equal_i(to_check->tomorrow_high, original->tomorrow_high);
  cl_assert_equal_i(to_check->tomorrow_low, original->tomorrow_low);
  cl_assert_equal_i(to_check->tomorrow_weather_type, original->tomorrow_weather_type);
}

void test_weather_service__get_data_for_all_locations(void) {
  size_t count_out;
  WeatherDataListNode *head = weather_service_locations_list_create(&count_out);
  WeatherLocationID id = 0;

  WeatherDataListNode *current = head;
  while (current) {
    cl_assert_equal_i(current->id, id);
    prv_assert_forecast_equal(&current->forecast, &s_forecasts[id]);
    id++;
    current = (WeatherDataListNode *)current->node.next;
  }

  cl_assert_equal_i(count_out, 4);
  cl_assert_equal_i(id, 4);

  weather_service_locations_list_destroy(head);
}

void test_weather_service__get_default_location_forecast_from_weather_db_update(void) {
  WeatherLocationForecast *forecast = weather_service_create_default_forecast();
  // no blob db events were fired during unit test, therefore forecast cache never updated
  cl_assert(!forecast);

  const int default_location_index = 0;
  const WeatherDBKey *default_location_key = weather_shared_data_get_key(default_location_index);

  PebbleEvent insert_event = (PebbleEvent) {
    .type = PEBBLE_BLOBDB_EVENT,
    .blob_db = {
      .db_id = BlobDBIdWeather,
      .type = BlobDBEventTypeInsert,
      .key = (uint8_t *)default_location_key,
      .key_len = sizeof(WeatherDBKey),
    }
  };

  s_event_info->handler(&insert_event, s_event_info->context);
  forecast = weather_service_create_default_forecast();

  cl_assert(forecast);
  prv_assert_forecast_equal(forecast, &s_forecasts[0]);
  weather_service_destroy_default_forecast(forecast);

  weather_db_flush();
  PebbleEvent flush_event = (PebbleEvent) {
    .type = PEBBLE_BLOBDB_EVENT,
    .blob_db = {
      .db_id = BlobDBIdWeather,
      .type = BlobDBEventTypeFlush,
      .key = NULL,
      .key_len = 0,
    }
  };

  s_event_info->handler(&flush_event, s_event_info->context);
  forecast = weather_service_create_default_forecast();
  cl_assert(!forecast);
}

void test_weather_service__get_default_location_forecast_from_watch_app_prefs_db_update(void) {
  WeatherLocationForecast *forecast = weather_service_create_default_forecast();
  // no blob db events were fired during unit test, therefore forecast cache never updated
  cl_assert(!forecast);

  const int default_location_index = 0;
  const WeatherDBKey *default_location_key = weather_shared_data_get_key(0);

  PebbleEvent insert_event = (PebbleEvent) {
    .type = PEBBLE_BLOBDB_EVENT,
    .blob_db = {
      .db_id = BlobDBIdWatchAppPrefs,
      .type = BlobDBEventTypeInsert,
      .key = (uint8_t *)PREF_KEY_WEATHER_APP,
      .key_len = sizeof(PREF_KEY_WEATHER_APP),
    }
  };

  s_event_info->handler(&insert_event, s_event_info->context);
  forecast = weather_service_create_default_forecast();

  cl_assert(forecast);
  prv_assert_forecast_equal(forecast, &s_forecasts[0]);
  weather_service_destroy_default_forecast(forecast);
}

void test_weather_service__skips_entry_with_empty_strings_blob(void) {
  // Overwrite location 0's record with one whose strings blob is valid but
  // EMPTY (data_size == 0): structurally legal, yet it resolves to zero
  // pstrings — the service must skip the record instead of dereferencing NULL.
  const WeatherDBKey *key = weather_shared_data_get_key(0);
  const WeatherDBEntry *base = weather_shared_data_get_entry(0);
  const size_t fixed = weather_db_entry_strings_offset(WEATHER_DB_CURRENT_VERSION,
                                                       WEATHER_DB_CURRENT_MINOR_VERSION);
  const size_t size = fixed + sizeof(SerializedArray);
  uint8_t *record = task_zalloc_check(size);
  memcpy(record, base, size);   // fixed fields from the valid fixture
  ((SerializedArray *)(record + fixed))->data_size = 0;
  cl_assert_equal_i(S_SUCCESS, weather_db_delete((uint8_t *)key, sizeof(WeatherDBKey)));
  cl_assert_equal_i(S_SUCCESS, weather_db_insert((uint8_t *)key, sizeof(WeatherDBKey),
                                                 record, size));
  task_free(record);

  size_t count_out = 0;
  WeatherDataListNode *head = weather_service_locations_list_create(&count_out);
  cl_assert_equal_i(count_out, 3);   // location 0 skipped, the other three intact
  weather_service_locations_list_destroy(head);
}
