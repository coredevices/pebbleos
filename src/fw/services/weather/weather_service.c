/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/weather/weather_service.h"
#include "pbl/services/weather/weather_service_private.h"

#include "applib/event_service_client.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/comm_session/session_remote_version.h"
#include "pbl/services/blob_db/watch_app_prefs_db.h"
#include "pbl/services/blob_db/weather_db.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DEFINE(service_weather, CONFIG_SERVICE_WEATHER_LOG_LEVEL);

static int prv_weather_data_list_node_comparator(void *a, void *b) {
  return ((WeatherDataListNode *)b)->id - ((WeatherDataListNode *)a)->id;
}

typedef struct WeatherDBIteratorContext {
  WeatherDataListNode *head;
  size_t count;
  SerializedWeatherAppPrefs *serialized_prefs;
} WeatherDBIteratorContext;

static PebbleMutex *s_mutex;
static WeatherLocationForecast *s_default_forecast;

static bool prv_entry_update_time_too_old_to_be_valid(const time_t update_time_utc) {
  const time_t oldest_valid_time_utc = time_start_of_today() - SECONDS_PER_DAY;
  return (update_time_utc < oldest_valid_time_utc);
}

static bool prv_fill_forecast_from_entry(WeatherDBEntry *entry,
                                         WeatherLocationForecast *forecast_out) {
  PascalString16List pstring16_list;
  // v3 and v4 records place the trailing strings at different offsets; locate
  // them by the record's version (see weather_db.h).
  pstring_project_list_on_serialized_array(&pstring16_list, weather_db_entry_get_strings(entry));
  PascalString16 *location_pstring =
      pstring_get_pstring16_from_list(&pstring16_list, WeatherDbStringIndex_LocationName);

  PascalString16 *phrase_pstring =
      pstring_get_pstring16_from_list(&pstring16_list, WeatherDbStringIndex_ShortPhrase);

  // The string block is phone-controlled; a record can carry fewer strings than we index.
  if (!location_pstring || !phrase_pstring) {
    PBL_LOG_ERR("Weather entry is missing its location/phrase strings");
    return false;
  }

  const bool is_valid_entry_update_time =
      (entry->last_update_time_utc != WEATHER_SERVICE_INVALID_DATA_LAST_UPDATE_TIME);
  const uint16_t location_pstring_length = location_pstring->str_length;

  if (!is_valid_entry_update_time || (location_pstring_length == 0)) {
    PBL_LOG_ERR("Invalid entry. Valid UT: %u, location length: %"PRIu16,
            is_valid_entry_update_time, location_pstring_length);
    return false;
  }

  if (prv_entry_update_time_too_old_to_be_valid(entry->last_update_time_utc)) {
    PBL_LOG_WRN("Weather entry too old to fill forecast");
    return false;
  }

  *forecast_out = (WeatherLocationForecast) {
    .is_current_location = entry->is_current_location,
    .current_temp = entry->current_temp,
    .today_high = entry->today_high_temp,
    .today_low = entry->today_low_temp,
    .current_weather_type = entry->current_weather_type,
    .tomorrow_high = entry->tomorrow_high_temp,
    .tomorrow_low = entry->tomorrow_low_temp,
    .tomorrow_weather_type = entry->tomorrow_weather_type,
    .time_updated_utc = entry->last_update_time_utc,
  };

  // add 1 for null terminator
  forecast_out->location_name = task_zalloc_check(location_pstring->str_length + 1);
  pstring_pstring16_to_string(location_pstring, forecast_out->location_name);

  forecast_out->current_weather_phrase = task_zalloc_check(phrase_pstring->str_length + 1);
  pstring_pstring16_to_string(phrase_pstring, forecast_out->current_weather_phrase);

  return true;
}

static bool prv_get_location_index(Uuid *location, SerializedWeatherAppPrefs *prefs,
                                   size_t *index_out) {
  if (!index_out) {
    return false;
  }

  for (size_t idx = 0; idx < prefs->num_locations; idx++) {
    if (uuid_equal(location, &prefs->locations[idx])) {
      *index_out = idx;
      return true;
    }
  }
  return false;
}

static void prv_add_to_list_if_valid(WeatherDBKey *key, WeatherDBEntry *entry,
                                     void *context) {
  WeatherDBIteratorContext *iterator_context = context;
  SerializedWeatherAppPrefs *prefs = iterator_context->serialized_prefs;
  char key_string_buffer[UUID_STRING_BUFFER_LENGTH] = {0};
  size_t location_index;

  if (!prv_get_location_index(key, prefs, &location_index)) {
    uuid_to_string(key, key_string_buffer);
    PBL_LOG_WRN("Weather location %s has no known ordering! Skipping",
            key_string_buffer);
    return; // location not found in ordering list, skip over
  }

  WeatherDataListNode *node = task_zalloc_check(sizeof(WeatherDataListNode));
  node->id = location_index;
  WeatherLocationForecast *forecast = &node->forecast;

  if (!prv_fill_forecast_from_entry(entry, forecast)) {
    uuid_to_string(key, key_string_buffer);
    PBL_LOG_WRN("Could not create forecast from %s's entry", key_string_buffer);
    task_free(node);
    return;
  }

  ListNode *to_add = (ListNode *)node;
  list_init(to_add);

  const bool ascending = true;
  iterator_context->head =
      (WeatherDataListNode *)list_sorted_add((ListNode *)iterator_context->head,
                                              to_add,
                                              prv_weather_data_list_node_comparator,
                                              ascending);
  iterator_context->count++;
}

static bool prv_get_default_location_key(WeatherDBKey *key_out) {
  SerializedWeatherAppPrefs *prefs = watch_app_prefs_get_weather();
  if (!prefs) {
    PBL_LOG_ERR("No SerializedWeatherAppPrefs available!");
    return false;
  }

  // Can occur if the user removes all weather locations from their mobile app
  if (prefs->num_locations == 0) {
    watch_app_prefs_destroy_weather(prefs);
    return false;
  }

  const size_t default_location_index = 0;
  *key_out = prefs->locations[default_location_index];

  watch_app_prefs_destroy_weather(prefs);
  return true;
}

static void prv_update_default_location_cache(void) {
  mutex_lock(s_mutex);
  weather_service_destroy_default_forecast(s_default_forecast);
  s_default_forecast = NULL;

  WeatherDBKey default_location_key;
  if (!prv_get_default_location_key(&default_location_key)) {
    goto cleanup;
  }

  const int entry_len = weather_db_get_len((uint8_t *)&default_location_key,
                                           sizeof(default_location_key));
  if (entry_len == 0) {
    goto cleanup;
  }
  WeatherDBEntry *entry = task_zalloc_check(entry_len);
  status_t rv = weather_db_read((uint8_t *)&default_location_key, sizeof(default_location_key),
                                (uint8_t *)entry, entry_len);
  if (rv != S_SUCCESS) {
    goto free_entry;
  }
  WeatherLocationForecast *forecast_out = task_zalloc_check(sizeof(WeatherLocationForecast));
  if (prv_fill_forecast_from_entry(entry, forecast_out)) {
    s_default_forecast = forecast_out;
  } else {
    task_free(forecast_out);
  }

free_entry:
  task_free(entry);
cleanup:
  mutex_unlock(s_mutex);
}

WeatherLocationForecast *weather_service_create_default_forecast(void) {
  mutex_lock(s_mutex);
  WeatherLocationForecast *forecast = NULL;
  if (s_default_forecast) {
    forecast = task_zalloc_check(sizeof(WeatherLocationForecast));
    *forecast = *s_default_forecast;
    const size_t location_name_length = strlen(s_default_forecast->location_name);
    forecast->location_name = task_zalloc_check(location_name_length + 1);
    strncpy(forecast->location_name, s_default_forecast->location_name, location_name_length);

    const size_t phrase_length = strlen(s_default_forecast->current_weather_phrase);
    forecast->current_weather_phrase = task_zalloc_check(phrase_length + 1);
    strncpy(forecast->current_weather_phrase, s_default_forecast->current_weather_phrase,
            phrase_length);
  }
  mutex_unlock(s_mutex);
  return forecast;
}

void weather_service_destroy_default_forecast(WeatherLocationForecast *forecast) {
  if (forecast) {
    task_free(forecast->location_name);
    task_free(forecast->current_weather_phrase);
    task_free(forecast);
  }
}

static void prv_blobdb_event_handler(PebbleEvent *event, void *context) {
  const PebbleBlobDBEvent *blobdb_event = &event->blob_db;
  const BlobDBId blobdb_id = blobdb_event->db_id;

  if ((blobdb_id != BlobDBIdWeather) && (blobdb_id != BlobDBIdWatchAppPrefs)) {
    // Only care about weather and weather ordering preferences
    return;
  }

  WeatherEventType type;

  const bool is_key_weather_app_pref = blobdb_event->key &&
      (memcmp(blobdb_event->key, PREF_KEY_WEATHER_APP, strlen(PREF_KEY_WEATHER_APP)) == 0);
  if (blobdb_id == BlobDBIdWatchAppPrefs &&
      ((blobdb_event->type == BlobDBEventTypeFlush) || is_key_weather_app_pref)) {
    type = WeatherEventType_WeatherOrderChanged;
  } else if (blobdb_id == BlobDBIdWeather) {
    type = blobdb_event->type == BlobDBEventTypeInsert ? WeatherEventType_WeatherDataAdded :
                                                         WeatherEventType_WeatherDataRemoved;
  } else {
    return;
  }

  prv_update_default_location_cache();

  PebbleEvent e = (PebbleEvent) {
    .type = PEBBLE_WEATHER_EVENT,
    .weather = (PebbleWeatherEvent) {
      .type = type,
    },
  };

  event_put(&e);
}

void weather_service_init(void) {
  s_mutex = mutex_create();

  static EventServiceInfo s_blobdb_event_info = {
    .type = PEBBLE_BLOBDB_EVENT,
    .handler = prv_blobdb_event_handler,
  };

  prv_update_default_location_cache();
  event_service_client_subscribe(&s_blobdb_event_info);
}

WeatherDataListNode *weather_service_locations_list_create(size_t *count_out) {
  WeatherDBIteratorContext context = (WeatherDBIteratorContext) {};
  SerializedWeatherAppPrefs *prefs = watch_app_prefs_get_weather();
  if (!prefs) {
    return NULL;
  }
  context.serialized_prefs = prefs;
  weather_db_for_each(prv_add_to_list_if_valid, &context);
  watch_app_prefs_destroy_weather(prefs);
  *count_out = context.count;

  return context.head;
}

WeatherDataListNode *weather_service_locations_list_get_location_at_index(WeatherDataListNode *head,
                                                                          unsigned int index) {
  return (WeatherDataListNode *)list_get_at(&head->node, index);
}

void weather_service_locations_list_destroy(WeatherDataListNode *head) {
  while (head) {
    task_free(head->forecast.location_name);
    task_free(head->forecast.current_weather_phrase);
    WeatherDataListNode *next = (WeatherDataListNode *)head->node.next;
    task_free(head);
    head = next;
  }
}

bool weather_service_supported_by_phone(void) {
  PebbleProtocolCapabilities capabilities;
  bt_persistent_storage_get_cached_system_capabilities(&capabilities);
  if (!capabilities.weather_app_support) {
    PBL_LOG_WRN("No weather support on phone");
  }
  return capabilities.weather_app_support;
}
