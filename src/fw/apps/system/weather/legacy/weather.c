/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// The stock upstream weather app: built ONLY for boards that do NOT take the
// rich weather app. This is the INVERSE of ../weather_platform.h's
// WEATHER_PLATFORM_TOUCH_COLOR, restated raw because that header pulls in the
// rich app's SDK-compat macro shims, which this firmware-native code must not
// see — keep the two conditions in sync.
#if !(defined(PBL_PLATFORM_EMERY) || defined(PBL_PLATFORM_GABBRO) || \
      (defined(CONFIG_TOUCH) && CONFIG_TOUCH))
#include "weather.h"
#include "layout.h"
#include "warning_dialog.h"

#include "applib/app.h"
#include "applib/event_service_client.h"
#include "applib/ui/click.h"
#include "applib/ui/content_indicator.h"
#include "applib/ui/ui.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/timeline/timeline.h"
#include "pbl/services/weather/weather_service.h"
#include "pbl/services/weather/weather_types.h"
#include "util/array.h"
#include "pbl/util/attributes.h"
#include "pbl/util/list.h"
#include "pbl/util/math.h"

typedef struct WeatherAppData {
  Window window;
  WeatherAppLayout layout;
  WeatherDataListNode *forecasts_list_head;
  size_t forecasts_count;
  unsigned int current_forecast_index;
  EventServiceInfo weather_event_info;
  WeatherAppWarningDialog *warning_dialog;
} WeatherAppData;

static bool prv_is_weather_forecast_recent(WeatherLocationForecast *forecast) {
  if (!forecast) {
    return false;
  }
  const time_t current_time_utc = rtc_get_time();
  const int recent_threshold_seconds = 5 * SECONDS_PER_HOUR / 2; // 2.5 hours
  const int seconds_since_forecast_was_updated = current_time_utc - forecast->time_updated_utc;
  return (seconds_since_forecast_was_updated < recent_threshold_seconds);
}

static void prv_warning_dialog_dismiss_cb(void) {
  WeatherAppData *data = app_state_get_user_data();
  data->warning_dialog = NULL;
}

static void prv_show_warning_dialog(WeatherAppData *data, bool exit_on_pop,
                                    const char *localized_text) {
  if (data->warning_dialog) {
    return; // only show one dialog at a time
  }
  if (exit_on_pop) {
    bool animated = false;
    app_window_stack_pop_all(animated);
  }
  data->warning_dialog = weather_app_warning_dialog_push(localized_text,
                                                         prv_warning_dialog_dismiss_cb);
}

static void prv_handle_weather(PebbleEvent *unused_event, void *unused_context) {
  // Unschedule any ongoing animations that would try to touch the weather data we're about to
  // update
  animation_unschedule_all();

  size_t forecasts_count_out = 0;
  WeatherDataListNode *forecasts_list_head =
      weather_service_locations_list_create(&forecasts_count_out);

  WeatherAppData *data = app_state_get_user_data();
  weather_service_locations_list_destroy(data->forecasts_list_head);
  WeatherAppLayout *layout = &data->layout;
  if (forecasts_count_out > 0) {
    weather_app_layout_set_data(layout, &forecasts_list_head->forecast);
    const bool multiple_forecasts_exist = (forecasts_count_out > 1);
    weather_app_layout_set_down_arrow_visible(layout, multiple_forecasts_exist);

    data->forecasts_list_head = forecasts_list_head;
    // Only show the first forecast if the number of forecasts has differed between fetches.
    // i.e. assume that the same number of forecasts means the locations have remained the same.
    if (data->forecasts_count != forecasts_count_out) {
      data->forecasts_count = forecasts_count_out;
      data->current_forecast_index = 0;
    }
  } else {
    /// Shown when there are no forecasts available to show the user
    const char *warning_text = i18n_get("No location information available. To see weather, add "\
                                        "locations in your Pebble mobile app.", data);
    const bool exit_on_pop = true;
    prv_show_warning_dialog(data, exit_on_pop, warning_text);
    weather_app_layout_set_down_arrow_visible(layout, false);
    weather_app_layout_set_data(layout, NULL);
  }
}

static void prv_main_window_appear(Window *window) {
  WeatherAppData *data = app_state_get_user_data();
  data->weather_event_info = (EventServiceInfo) {
    .type = PEBBLE_WEATHER_EVENT,
    .handler = prv_handle_weather,
  };
  event_service_client_subscribe(&data->weather_event_info);
}

static void prv_main_window_load(Window *window) {
  WeatherAppData *data = app_state_get_user_data();
  layer_add_child(&window->layer, &data->layout.root_layer);
}

static void prv_main_window_disappear(Window *window) {
  WeatherAppData *data = app_state_get_user_data();
  event_service_client_unsubscribe(&data->weather_event_info);
}

static void prv_up_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  WeatherAppData *data = app_state_get_user_data();
  const bool not_enough_items_to_scroll = (data->forecasts_count <= 1);
  if (not_enough_items_to_scroll) {
    return;
  }

  const bool is_down_pressed = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_DOWN);
  const int delta = is_down_pressed ? 1 : -1;
  data->current_forecast_index = positive_modulo(data->current_forecast_index + delta,
                                                 data->forecasts_count);

  WeatherDataListNode *node =
      weather_service_locations_list_get_location_at_index(data->forecasts_list_head,
                                                           data->current_forecast_index);
  weather_app_layout_animate(&data->layout, &node->forecast, is_down_pressed);
}

static void prv_main_window_click_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_up_down_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_up_down_click_handler);
}

static void prv_main_window_unload(Window *window) {
  WeatherAppData *data = app_state_get_user_data();
  weather_app_layout_deinit(&data->layout);
}

#if defined(CONFIG_SOC_QEMU)
// QEMU has no phone, which leaves this app both hidden from the launcher and
// dataless — untestable in the emulator. Seed the REAL pipeline instead
// (prefs ordering + weather records through the public, validating BlobDB
// insert APIs) so the emulator exercises the same code path a phone sync
// does. Emulator-only test affordance, compiled out on hardware — the same
// spirit as the rich app's QEMU synth.
#include "pbl/drivers/rtc.h"
#include "pbl/services/blob_db/api.h"
#include "pbl/services/blob_db/weather_db.h"
#include "pbl/services/weather/weather_service_private.h"

static void prv_qemu_seed_record(const Uuid *key, const char *location,
                                 const char *phrase, bool is_current,
                                 int temp, int high, int low, WeatherType type) {
  const uint16_t loc_len = (uint16_t)strlen(location);
  const uint16_t phr_len = (uint16_t)strlen(phrase);
  const uint16_t data_size = (uint16_t)(2 * sizeof(uint16_t) + loc_len + phr_len);
  const size_t total = sizeof(WeatherDBEntry) + data_size;

  WeatherDBEntry *e = app_zalloc_check(total);
  e->version = WEATHER_DB_CURRENT_VERSION;
  e->minor_version = WEATHER_DB_CURRENT_MINOR_VERSION;
  e->is_current_location = is_current;
  e->current_temp = (int16_t)temp;
  e->current_weather_type = type;
  e->today_high_temp = (int16_t)high;
  e->today_low_temp = (int16_t)low;
  e->tomorrow_weather_type = type;
  e->tomorrow_high_temp = (int16_t)(high - 2);
  e->tomorrow_low_temp = (int16_t)(low - 2);
  e->last_update_time_utc = rtc_get_time();
  // v4 extras at their unknown sentinels — the stock app reads only the v3
  // prefix, but the record should still be well-formed v4.
  e->today_feels_like_temp = WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  e->today_uv_index_x10 = -1;
  e->today_precip_probability = -1;
  e->today_wind_direction = 0xFFFF;
  e->latitude_e2 = INT16_MIN;
  e->longitude_e2 = INT16_MIN;
  e->location_utc_offset_min = INT16_MIN;
  e->today_wmo_code = 0xFF;
  e->today_humidity_pct = 0xFF;
  e->today_visibility_m = 0xFFFF;
  e->today_precip_sum_mm = 0xFFFF;
  e->today_wind_dir_deg = -1;
  for (int i = 0; i < WEATHER_DB_MAX_FORECAST_DAYS; i++) {
    e->daily_feels_like[i] = WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
    e->daily_wind_dir_deg[i] = -1;
  }
  memset(e->today_hourly_uv_x10, 0xFF, sizeof(e->today_hourly_uv_x10));

  // Trailing strings: [SerializedArray data_size][u16 len|loc][u16 len|phrase]
  e->pstring16s.data_size = data_size;
  uint8_t *p = e->pstring16s.data;
  memcpy(p, &loc_len, sizeof(loc_len));
  memcpy(p + sizeof(loc_len), location, loc_len);
  p += sizeof(loc_len) + loc_len;
  memcpy(p, &phr_len, sizeof(phr_len));
  memcpy(p + sizeof(phr_len), phrase, phr_len);

  blob_db_insert(BlobDBIdWeather, (uint8_t *)key, sizeof(*key),
                 (uint8_t *)e, (int)total);
  app_free(e);
}

static void prv_qemu_seed(void) {
  size_t count = 0;
  WeatherDataListNode *head = weather_service_locations_list_create(&count);
  weather_service_locations_list_destroy(head);
  if (count > 0) {
    return;   // already seeded (or a phone really synced something)
  }

  static const Uuid s_seed_keys[2] = {
    {0x9e, 0x4a, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
     0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0x00, 0x01},
    {0x9e, 0x4a, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
     0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0x00, 0x02},
  };

  // Ordering prefs first, so the service accepts both records' list positions.
  uint8_t prefs[1 + sizeof(s_seed_keys)];
  prefs[0] = 2;
  memcpy(&prefs[1], s_seed_keys, sizeof(s_seed_keys));
  blob_db_insert(BlobDBIdWatchAppPrefs, (uint8_t *)PREF_KEY_WEATHER_APP,
                 (int)strlen(PREF_KEY_WEATHER_APP), prefs, (int)sizeof(prefs));

  prv_qemu_seed_record(&s_seed_keys[0], "Mexico City", "Clear Sky",
                       true /* current */, 23, 28, 18, WeatherType_Sun);
  prv_qemu_seed_record(&s_seed_keys[1], "Reykjavik", "Light Snow",
                       false, -2, 1, -6, WeatherType_LightSnow);
}
#endif  // CONFIG_SOC_QEMU

static NOINLINE void prv_init(void) {
#if defined(CONFIG_SOC_QEMU)
  prv_qemu_seed();
#endif
  WeatherAppData *data = app_zalloc_check(sizeof(WeatherAppData));
  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Weather"));
  const WindowHandlers window_handlers = {
    .appear = prv_main_window_appear,
    .load = prv_main_window_load,
    .disappear = prv_main_window_disappear,
    .unload = prv_main_window_unload,
  };
  window_set_window_handlers(window, &window_handlers);

  window_set_click_config_provider(window, prv_main_window_click_provider);
  window_set_user_data(window, data);

  const GRect *layout_frame = &window->layer.bounds;
  WeatherAppLayout *layout = &data->layout;
  weather_app_layout_init(layout, layout_frame);
  window_set_user_data(window, layout);

  // Fetch initial data
  prv_handle_weather(NULL, NULL);

  if (data->forecasts_count == 0) {
    return;
  }

  const bool animated = true;
  app_window_stack_push(&data->window, animated);

  // Request the default forecast separately instead of using the forecast list in `data` to avoid
  // any potential race conditions
  WeatherLocationForecast *default_forecast = weather_service_create_default_forecast();
  const bool is_default_forecast_data_recent = prv_is_weather_forecast_recent(default_forecast);
  weather_service_destroy_default_forecast(default_forecast);

  // TODO PBL-38484: Consider using a different dialog for when data is stale but phone is connected
  if (!is_default_forecast_data_recent && !connection_service_peek_pebble_app_connection()) {
    /// Shown when there is no connection to the phone and the data that we have is not recent
    const char *warning_text = i18n_get("Unable to connect. Your weather data may be out of date; "\
                                        "try checking the connection on your phone.", data);
    const bool exit_on_pop = false;
    prv_show_warning_dialog(data, exit_on_pop, warning_text);
  }
}

static void prv_deinit(void) {
  WeatherAppData *data = app_state_get_user_data();
  i18n_free_all(data);
}

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

const PebbleProcessMd* weather_app_get_info() {
  const bool is_visible_in_launcher = weather_service_supported_by_phone();

  static const PebbleProcessMdSystem s_weather_app_info = {
    .common = {
      .main_func = prv_main,
      .uuid = UUID_WEATHER_DATA_SOURCE,
    },
    .name = i18n_noop("Weather"),
    .icon_resource_id = RESOURCE_ID_GENERIC_WEATHER_TINY,
  };

  return is_visible_in_launcher ? (const PebbleProcessMd *)&s_weather_app_info : NULL;
}
#endif  // platform gate
