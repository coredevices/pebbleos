/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "face.h"

#include "applib/app.h"
#include "applib/graphics/gdraw_command_image.h"
#include "applib/graphics/text.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/clock.h"
#include "util/time/time.h"

typedef struct {
  Window low_power_window;
  TextLayer low_power_time_layer;
  TextLayer low_power_date_layer;
  KinoLayer low_power_kino_layer;
  char time_text[6];
  char date_text[16];
} LowPowerFaceData;

static LowPowerFaceData *s_low_power_data;

static void prv_minute_tick(struct tm *tick_time, TimeUnits units_changed) {
  char *time_format;

  if (clock_is_24h_style()) {
    time_format = "%R";
  } else {
    time_format = "%I:%M";
  }

  strftime(s_low_power_data->time_text, sizeof(s_low_power_data->time_text),
            time_format, tick_time);

  // Remove leading zero from hour in case of 12h mode
  if (!clock_is_24h_style() && (s_low_power_data->time_text[0] == '0')) {
    text_layer_set_text(&s_low_power_data->low_power_time_layer,
                          s_low_power_data->time_text+1);
  } else {
    text_layer_set_text(&s_low_power_data->low_power_time_layer,
                          s_low_power_data->time_text);
  }

  if (units_changed & DAY_UNIT) {
    clock_get_date_tm(s_low_power_data->date_text, sizeof(s_low_power_data->date_text),
                      tick_time);
    text_layer_set_text(&s_low_power_data->low_power_date_layer,
                        s_low_power_data->date_text);
  }
}

static void deinit(void) {
  tick_timer_service_unsubscribe();
  kino_layer_deinit(&s_low_power_data->low_power_kino_layer);

  app_free(s_low_power_data);
}

static void init(void) {
  s_low_power_data = app_malloc_check(sizeof(*s_low_power_data));

  window_init(&s_low_power_data->low_power_window, "Low Power");
  window_set_background_color(&s_low_power_data->low_power_window, GColorLightGray);
  app_window_stack_push(&s_low_power_data->low_power_window, true /* Animated */);

#if PBL_DISPLAY_HEIGHT >= 200
  const GFont text_font = fonts_get_system_font(FONT_KEY_LECO_60_NUMBERS_AM_PM);
#else
  const GFont text_font = fonts_get_system_font(FONT_KEY_LECO_42_NUMBERS);
#endif
  const GTextAlignment text_alignment = GTextAlignmentCenter;
  const unsigned int font_height = fonts_get_font_height(text_font);
  const GTextOverflowMode text_overflow_mode = GTextOverflowModeTrailingEllipsis;
  const GSize text_size = app_graphics_text_layout_get_content_size("00:00", text_font,
                            s_low_power_data->low_power_window.layer.bounds, text_overflow_mode,
                            text_alignment);
#if PBL_DISPLAY_HEIGHT >= 200
  const int text_pos_y_adjust = -9;
#else
  const int text_pos_y_adjust = -20;
#endif
  const int text_pos_y = (DISP_ROWS / 2) - (font_height / 2) + text_pos_y_adjust;
  const GRect text_container_rect = GRect(0, text_pos_y, DISP_COLS, font_height);
  GRect text_frame = (GRect) { .size = text_size };
  grect_align(&text_frame, &text_container_rect, GAlignTop, false);

  kino_layer_init(&s_low_power_data->low_power_kino_layer,
                  &s_low_power_data->low_power_window.layer.bounds);
  kino_layer_set_reel_with_resource(&s_low_power_data->low_power_kino_layer,
                                    RESOURCE_ID_BATTERY_NEEDS_CHARGING);
  kino_layer_set_alignment(&s_low_power_data->low_power_kino_layer, GAlignBottom);
  // TODO PBL-30180: Design needs to revise icon so it doesn't have a rounded cap at the bottom
  s_low_power_data->low_power_kino_layer.layer.frame.origin.y += 2;
  layer_add_child(&s_low_power_data->low_power_window.layer,
                  &s_low_power_data->low_power_kino_layer.layer);

  text_layer_init_with_parameters(&s_low_power_data->low_power_time_layer,
                                  &s_low_power_data->low_power_window.layer.frame, NULL, text_font,
                                  GColorBlack, GColorClear, text_alignment, text_overflow_mode);
  layer_set_frame(&s_low_power_data->low_power_time_layer.layer, &text_frame);
  layer_add_child(&s_low_power_data->low_power_window.layer,
                    &s_low_power_data->low_power_time_layer.layer);

#if PBL_DISPLAY_HEIGHT >= 200
  const GFont date_font = fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD);
#else
  const GFont date_font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
#endif
  const unsigned int date_font_height = fonts_get_font_height(date_font);
  const GRect icon_bounds = kino_layer_get_reel_bounds(&s_low_power_data->low_power_kino_layer);
  const int icon_top_y =
      s_low_power_data->low_power_kino_layer.layer.frame.origin.y + icon_bounds.origin.y;
  const int time_bottom_y = text_frame.origin.y + text_frame.size.h;
  const GRect date_frame = (GRect) {
    .origin = { 0, time_bottom_y + (icon_top_y - time_bottom_y - (int)date_font_height) / 2 - 3 },
    .size   = { DISP_COLS, date_font_height },
  };
  text_layer_init_with_parameters(&s_low_power_data->low_power_date_layer,
                                  &date_frame, NULL, date_font,
                                  GColorBlack, GColorClear, text_alignment, text_overflow_mode);
  layer_add_child(&s_low_power_data->low_power_window.layer,
                  &s_low_power_data->low_power_date_layer.layer);

  // Because of the delay before the tick timer service first calls prv_minute_tick,
  // we call it ourselves to update the time right away
  struct tm current_time;
  clock_get_time_tm(&current_time);
  prv_minute_tick(&current_time, MINUTE_UNIT | HOUR_UNIT | DAY_UNIT);

  tick_timer_service_subscribe(MINUTE_UNIT, prv_minute_tick);
}

static void low_power_main(void) {
  init();

  app_event_loop();

  deinit();
}

const PebbleProcessMd* low_power_face_get_app_info() {
  static const PebbleProcessMdSystem s_app_md = {
    .common = {
      // UUID: e9475244-5bbe-4e0f-a637-a218af4c3110
      .uuid = {0xe9, 0x47, 0x52, 0x44, 0x5b, 0xbe, 0x4e, 0x0f, 0xa6, 0x37, 0xa2, 0x18, 0xaf, 0x4c, 0x31, 0x10},
      .main_func = low_power_main,
      .process_type = ProcessTypeWatchface,
      .visibility = ProcessVisibilityHidden,
    },
    .name = "Watch Only"
  };
  return (const PebbleProcessMd*) &s_app_md;
}
