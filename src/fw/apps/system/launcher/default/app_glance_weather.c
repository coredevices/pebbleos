/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_weather.h"

#include "app_glance_structured.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/timeline/timeline_resources.h"
#include "pbl/services/weather/weather_service.h"
#include "pbl/services/weather/weather_types.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/struct.h"

#include <stdio.h>

// Max size of the temperature and phrase displayed together
#define WEATHER_APP_GLANCE_MAX_STRING_BUFFER_SIZE (WEATHER_SERVICE_MAX_SHORT_PHRASE_BUFFER_SIZE + 5)

typedef struct LauncherAppGlanceWeather {
  char title[APP_NAME_SIZE_BYTES];
  char fallback_title[APP_NAME_SIZE_BYTES];
  char subtitle[WEATHER_APP_GLANCE_MAX_STRING_BUFFER_SIZE];
  KinoReel *icon;
  uint32_t icon_resource_id;
  EventServiceInfo weather_event_info;
} LauncherAppGlanceWeather;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWeather *weather_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(weather_glance, icon, NULL);
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWeather *weather_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(weather_glance, title, NULL);
}

static void prv_weather_glance_subtitle_dynamic_text_node_update(
    PBL_UNUSED GContext *ctx, PBL_UNUSED GTextNode *node, PBL_UNUSED const GRect *box,
    PBL_UNUSED const GTextNodeDrawConfig *config, PBL_UNUSED bool render, char *buffer, size_t buffer_size,
    void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceWeather *weather_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (weather_glance) {
    strncpy(buffer, weather_glance->subtitle, buffer_size);
    buffer[buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  return launcher_app_glance_structured_create_subtitle_text_node(
      structured_glance, prv_weather_glance_subtitle_dynamic_text_node_update);
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWeather *weather_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (weather_glance) {
    event_service_client_unsubscribe(&weather_glance->weather_event_info);
    kino_reel_destroy(weather_glance->icon);
  }
  app_free(weather_glance);
}

static uint32_t prv_get_weather_icon_resource_id_for_type(WeatherType type) {
  AppResourceInfo res_info;
  const bool lookup_success =
      timeline_resources_get_id_system(weather_type_get_timeline_resource_id(type),
                                       TimelineResourceSizeTiny, SYSTEM_APP, &res_info);
  return lookup_success ? res_info.res_id : (uint32_t)RESOURCE_ID_INVALID;
}

static void prv_weather_event_handler(PBL_UNUSED PebbleEvent *event, void *context) {
  LauncherAppGlanceStructured *structured_glance = context;
  LauncherAppGlanceWeather *weather_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  PBL_ASSERTN(weather_glance);

  WeatherLocationForecast *forecast = weather_service_create_default_forecast();

  // Update the icon for the forecast's weather type
  const WeatherType weather_type = NULL_SAFE_FIELD_ACCESS(forecast, current_weather_type,
                                                          WeatherType_Unknown);
  const uint32_t new_weather_icon_resource_id =
      prv_get_weather_icon_resource_id_for_type(weather_type);
  if (weather_glance->icon_resource_id != new_weather_icon_resource_id) {
    kino_reel_destroy(weather_glance->icon);
    weather_glance->icon = kino_reel_create_with_resource(new_weather_icon_resource_id);
    weather_glance->icon_resource_id = new_weather_icon_resource_id;
  }

  // Zero out the glance's title buffer
  const size_t weather_glance_title_size = sizeof(weather_glance->title);
  memset(weather_glance->title, 0, weather_glance_title_size);
  // Choose the title we should display based on whether or not we have a forecast
  const char *title = NULL_SAFE_FIELD_ACCESS(forecast, location_name,
                                             weather_glance->fallback_title);
  // Subtract 1 from the size as a shortcut for null terminating the title since we zero it out
  // above
  strncpy(weather_glance->title, title, weather_glance_title_size - 1);

  // Zero out the glance's subtitle buffer
  const size_t weather_glance_subtitle_size = sizeof(weather_glance->subtitle);
  memset(weather_glance->subtitle, 0, weather_glance_subtitle_size);
  // We'll only set the subtitle if we have a default forecast
  if (forecast) {
    if (forecast->current_temp == WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
      /// Shown when the current temperature is unknown
      const char *no_temperature_string = i18n_get("--°", weather_glance);
      // Subtract 1 from the size as a shortcut for null terminating the subtitle since we zero it
      // out above
      strncpy(weather_glance->subtitle, no_temperature_string, weather_glance_subtitle_size - 1);
    } else {
      /// Shown when today's temperature and conditions phrase is known (e.g. "52° - Fair")
      const char *temp_and_phrase_formatter = i18n_get("%i° - %s", weather_glance);
      /// Today's current temperature (e.g. "68°")
      const char *temp_only_formatter = i18n_get("%i°", weather_glance);
      const char *localized_phrase = i18n_get(forecast->current_weather_phrase, weather_glance);
      const char *formatter_string = strlen(localized_phrase) ? temp_and_phrase_formatter :
                                     temp_only_formatter;
      // It's safe to pass more arguments to snprintf() than might be used by formatter_string
      snprintf(weather_glance->subtitle, weather_glance_subtitle_size, formatter_string,
               forecast->current_temp, localized_phrase);
    }
  }

  i18n_free_all(weather_glance);

  weather_service_destroy_default_forecast(forecast);

  // Broadcast to the service that we changed the glance
  launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
}

static const LauncherAppGlanceStructuredImpl s_weather_structured_glance_impl = {
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_weather_create(const AppMenuNode *node) {
  if (!node) {
    return NULL;
  }

  LauncherAppGlanceWeather *weather_glance = app_zalloc_check(sizeof(*weather_glance));
  // Copy the name of the Weather app as a fallback title
  const size_t fallback_title_size = sizeof(weather_glance->fallback_title);
  strncpy(weather_glance->fallback_title, node->name, fallback_title_size);
  weather_glance->fallback_title[fallback_title_size - 1] = '\0';

  const bool should_consider_slices = false;
  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_weather_structured_glance_impl,
                                            should_consider_slices, weather_glance);
  PBL_ASSERTN(structured_glance);

  prv_weather_event_handler(NULL, structured_glance);

  weather_glance->weather_event_info = (EventServiceInfo) {
    .type = PEBBLE_WEATHER_EVENT,
    .handler = prv_weather_event_handler,
    .context = structured_glance,
  };
  event_service_client_subscribe(&weather_glance->weather_event_info);

  return &structured_glance->glance;
}
