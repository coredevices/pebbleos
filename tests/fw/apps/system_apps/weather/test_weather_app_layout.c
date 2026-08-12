/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/graphics/framebuffer.h"
#include "applib/graphics/graphics.h"
#include "apps/system/weather/weather_app_layout.h"

// weather_app_layout.h pulls in the app's pebble_compat.h, whose SDK-shim
// macros rewrite firmware names (window_stack_*, resource_*, malloc, time, ...).
// The layout .c under test needs them, but this test TU declares the real
// firmware signatures via the stub headers below — undo the shims here.
#undef window_stack_push
#undef window_stack_pop
#undef window_stack_pop_all
#undef window_stack_remove
#undef window_stack_get_top_window
#undef layer_get_bounds
#undef layer_get_frame
#undef layer_set_frame
#undef menu_layer_set_callbacks
#undef window_set_window_handlers
#undef graphics_text_layout_get_content_size
#undef graphics_fill_rect
#undef graphics_draw_rect
#undef graphics_draw_round_rect
#undef graphics_draw_bitmap_in_rect
#undef resource_get_handle
#undef resource_size
#undef resource_load
#undef calloc
#undef malloc
#undef malloc_try
#undef free
#undef time
#undef localtime
#include "applib/ui/app_window_stack.h"
#include "applib/ui/content_indicator.h"
#include "applib/ui/content_indicator_private.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window_private.h"
#include "resource/resource.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/timeline/timeline_resources.h"
#include "shell/system_theme.h"
#include "util/buffer.h"
#include "util/graphics.h"
#include "pbl/util/hash.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include "clar.h"

#include <stdio.h>

// Fakes
/////////////////////

#include "fake_content_indicator.h"
#include "fake_pbl_std.h"
#include "fake_spi_flash.h"
#include "fixtures/load_test_resources.h"

// Stubs
/////////////////////

#include "stubs_analytics.h"
#include "stubs_animation_timing.h"
#include "stubs_app_install_manager.h"
#include "stubs_app_state.h"
#include "stubs_app_timer.h"
#include "stubs_bootbits.h"
#include "stubs_click.h"
#include "stubs_i18n.h"
#include "stubs_layer.h"
#include "stubs_logging.h"
#include "stubs_memory_layout.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_process_info.h"
#include "stubs_pebble_tasks.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "stubs_sleep.h"
#include "stubs_status_bar_layer.h"
#include "stubs_syscalls.h"
#include "stubs_system_theme.h"
#include "stubs_task_watchdog.h"
#include "stubs_window_manager.h"
#include "stubs_window_stack.h"

// Helper Functions
/////////////////////

#include "fw/graphics/test_graphics.h"
#include "fw/graphics/util.h"

// Setup and Teardown
////////////////////////////////////

static GContext s_ctx;
static FrameBuffer *fb = NULL;

KinoReel *kino_reel_morph_square_create(KinoReel *from_reel, bool take_ownership) {
  return from_reel;
}

GContext *graphics_context_get_current_context(void) {
  return &s_ctx;
}

void test_weather_app_layout__initialize(void) {
  fb = malloc(sizeof(FrameBuffer));
  framebuffer_init(fb, &(GSize) {DISP_COLS, DISP_ROWS});

  const GContextInitializationMode context_init_mode = GContextInitializationMode_System;
  graphics_context_init(&s_ctx, fb, context_init_mode);
  // The layout measures text via app_graphics_text_layout_get_content_size,
  // which resolves its GContext through app_state — point the stub at ours.
  s_app_state_get_graphics_context = &s_ctx;

  framebuffer_clear(fb);

  // Setup resources
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(true /* write erase headers */);
  load_resource_fixture_in_flash(RESOURCES_FIXTURE_PATH, SYSTEM_RESOURCES_FIXTURE_NAME, false /* is_next */);

  resource_init();

  ContentIndicatorsBuffer *buffer = content_indicator_get_current_buffer();
  content_indicator_init_buffer(buffer);
}

void test_weather_app_layout__cleanup(void) {
  free(fb);
}

// Helpers
//////////////////////

// The redesigned layout takes TWO forecasts (today + the next day) instead of
// the old embedded tomorrow_* fields, plus an explicit location. Fill every
// v4 extension field with its unknown sentinel so renders stay deterministic,
// and mirror the synced current_* values into the current-hour fields the
// header prefers (the data source does the same when no hourly block exists).
static WeatherLocationForecast prv_make_forecast(const char *name, bool is_current_location,
                                                 int temp, int high, int low,
                                                 WeatherType type, const char *phrase) {
  return (WeatherLocationForecast) {
    .location_name = (char *)name,
    .is_current_location = is_current_location,
    .current_temp = temp,
    .today_high = high,
    .today_low = low,
    .today_uv = -1,
    .today_uv_now = -1,
    .today_precip_mm = -1,
    .today_wind_mph = -1,
    .today_wind_dir_deg = -1,
    .today_feels = WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP,
    .today_wmo = -1,
    .today_humidity = -1,
    .today_visibility_m = -1,
    .today_precip_sum_mm = -1,
    .current_weather_type = type,
    .current_weather_phrase = (char *)phrase,
    .current_type_now = type,
    .current_temp_now = temp,
    .current_phrase_now = (char *)phrase,
  };
}

static void prv_create_layout_for_forecast(const WeatherLocationForecast *today,
                                           const WeatherLocationForecast *next,
                                           WeatherAppLayout *layout, Window *window) {
  window_init(window, WINDOW_NAME("Weather"));
  weather_app_layout_init(layout, &s_ctx.dest_bitmap.bounds);
  // Mirror the app's own seeding order (weather_report.c prv_seed_day).
  weather_app_layout_set_fin_allowed(layout, false);
  weather_app_layout_set_data(layout, today, next);
  if (today) {
    weather_app_layout_set_location(layout, today->location_name);
  }
  window_set_user_data(window, layout);

  Layer *window_root_layer = window_get_root_layer(window);
  layer_add_child(window_root_layer, layout->root_layer);
  window_set_on_screen(window, true, true);
}

static void prv_create_layout_for_forecast_and_render(const WeatherLocationForecast *today,
                                                      const WeatherLocationForecast *next) {
  Window window;
  WeatherAppLayout layout = {};
  prv_create_layout_for_forecast(today, next, &layout, &window);
  window_render(&window, &s_ctx);
  weather_app_layout_deinit(&layout);
}

// Tests
//////////////////////

void test_weather_app_layout__render_palo_alto(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("PALO ALTO", false, 68, 68, 58, WeatherType_Sun, "Sunny");
  const WeatherLocationForecast next =
      prv_make_forecast("PALO ALTO", false, 62, 62, 52, WeatherType_PartlyCloudy, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

static void prv_render_long_strings_test(bool is_current_location) {
  const WeatherLocationForecast today =
      prv_make_forecast("QWERTYUIO ASEDDFFGHHJ", is_current_location, 68, 68, 58,
                        WeatherType_PartlyCloudy, "Cloudy with 90% chance of meatballs");
  const WeatherLocationForecast next =
      prv_make_forecast("QWERTYUIO ASEDDFFGHHJ", is_current_location, 62, 62, 52,
                        WeatherType_Sun, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
}

void test_weather_app_layout__render_longer_strings(void) {
  const bool is_current_location = false;
  prv_render_long_strings_test(is_current_location);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_longer_strings_for_current_location(void) {
  const bool is_current_location = true;
  prv_render_long_strings_test(is_current_location);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_large_numbers(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("PALO ALTO", false, -88, -88, -88, WeatherType_Sun, "Sunny");
  const WeatherLocationForecast next =
      prv_make_forecast("PALO ALTO", false, -99, -99, -99, WeatherType_PartlyCloudy, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_cloudy_light_snow(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("PALO ALTO", false, -88, -88, -88, WeatherType_CloudyDay, "Cloudy");
  const WeatherLocationForecast next =
      prv_make_forecast("PALO ALTO", false, -99, -99, -99, WeatherType_LightSnow, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_light_rain_heavy_rain(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("PALO ALTO", false, -88, -88, -88, WeatherType_LightRain, "Light Rain");
  const WeatherLocationForecast next =
      prv_make_forecast("PALO ALTO", false, -99, -99, -99, WeatherType_HeavyRain, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_generic_generic(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("HOUSTON", false, 110, 120, 85, WeatherType_Generic, "Humid AF");
  const WeatherLocationForecast next =
      prv_make_forecast("HOUSTON", false, 500, 500, 100, WeatherType_Generic, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_heavy_snow_rain_snow(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("PALO ALTO", false, -88, -88, -88, WeatherType_HeavySnow, "Heavy Snow");
  const WeatherLocationForecast next =
      prv_make_forecast("PALO ALTO", false, -99, -99, -99, WeatherType_RainAndSnow, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_current_location(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("PHILADELPHIA", true, 13, 15, -2, WeatherType_HeavySnow, "Heavy Snow");
  const WeatherLocationForecast next =
      prv_make_forecast("PHILADELPHIA", true, 26, 26, 3, WeatherType_RainAndSnow, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_long_current_location_name_pbl_38049(void) {
  const WeatherLocationForecast today =
      prv_make_forecast("DA'AN DISTRICT", true, 30, 33, 26, WeatherType_CloudyDay, "M Cloudy");
  const WeatherLocationForecast next =
      prv_make_forecast("DA'AN DISTRICT", true, 34, 34, 26, WeatherType_HeavyRain, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_all_unknown_values(void) {
  const WeatherLocationForecast today = prv_make_forecast(
      "PALO ALTO", false, WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP,
      WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP,
      WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP, WeatherType_Unknown, "");
  const WeatherLocationForecast next = prv_make_forecast(
      "PALO ALTO", false, WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP,
      WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP,
      WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP, WeatherType_Unknown, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_weather_app_layout__render_some_unknown_values(void) {
  const WeatherLocationForecast today = prv_make_forecast(
      "PALO ALTO", false, WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP, 99,
      WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP, WeatherType_Sun, "");
  const WeatherLocationForecast next = prv_make_forecast(
      "PALO ALTO", false, WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP,
      WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP, -99, WeatherType_Unknown, "");

  prv_create_layout_for_forecast_and_render(&today, &next);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

// renders a blank image
void test_weather_app_layout__render_empty_view(void) {
  prv_create_layout_for_forecast_and_render(NULL, NULL);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}
