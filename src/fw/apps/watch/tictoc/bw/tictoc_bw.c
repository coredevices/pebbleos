/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/ui.h"
#include "applib/unobstructed_area_service.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "util/time/time.h"

#include <locale.h>

typedef struct {
  Window window;
  TextLayer text_date_layer;
  TextLayer text_time_layer;
  Layer line_layer;
  char time_text[6];
  char date_text[13];
} TicTocData;

static void prv_line_layer_update_callback(Layer *me, GContext* ctx) {
  GRect bounds;
  layer_get_bounds(me, &bounds);
  GRect unobstructed_bounds;
  layer_get_unobstructed_bounds(me, &unobstructed_bounds);

  // Calculate how much we need to shift up
  int16_t obstruction = bounds.size.h - unobstructed_bounds.size.h;
  int16_t extra_shift = obstruction > 0 ? 10 : 0;
  int16_t line_y = 97 - obstruction + extra_shift;

  graphics_context_set_stroke_color(ctx, GColorWhite);
  graphics_draw_line(ctx, GPoint(8, line_y), GPoint(131, line_y));
  graphics_draw_line(ctx, GPoint(8, line_y + 1), GPoint(131, line_y + 1));
}

static void prv_update_layer_positions(void) {
  TicTocData *data = app_state_get_user_data();
  Layer *window_layer = window_get_root_layer(&data->window);

  GRect bounds;
  layer_get_bounds(window_layer, &bounds);
  GRect unobstructed_bounds;
  layer_get_unobstructed_bounds(window_layer, &unobstructed_bounds);

  // Calculate how much we need to shift up (maintains spacing between elements)
  int16_t obstruction = bounds.size.h - unobstructed_bounds.size.h;
  // Add extra shift when obstructed to reduce bottom padding
  int16_t extra_shift = obstruction > 0 ? 10 : 0;

  GRect date_frame;
  layer_get_frame(&data->text_date_layer.layer, &date_frame);
  date_frame.origin.y = 68 - obstruction + extra_shift;
  layer_set_frame(&data->text_date_layer.layer, &date_frame);

  GRect time_frame;
  layer_get_frame(&data->text_time_layer.layer, &time_frame);
  time_frame.origin.y = 92 - obstruction + extra_shift;
  layer_set_frame(&data->text_time_layer.layer, &time_frame);

  layer_mark_dirty(&data->line_layer);
}

static void prv_unobstructed_area_change_handler(AnimationProgress progress, void *context) {
  prv_update_layer_positions();
}

static void prv_minute_tick_handler(struct tm *tick_time, TimeUnits units_changed) {
  TicTocData *data = app_state_get_user_data();

  strftime(data->date_text, sizeof(data->date_text), i18n_get("%B %e", data), tick_time);
  text_layer_set_text(&data->text_date_layer, data->date_text);

  strftime(data->time_text, sizeof(data->time_text),
      clock_is_24h_style() ? "%R" : "%I:%M", tick_time);

  // Handle lack of non-padded hour format string for twelve hour clock.
  char *start_time_text = data->time_text;
  if (!clock_is_24h_style() && (data->time_text[0] == '0')) {
    start_time_text++;
  }

  text_layer_set_text(&data->text_time_layer, start_time_text);
}

static void prv_deinit(void) {
  TicTocData *data = app_state_get_user_data();
  app_unobstructed_area_service_unsubscribe();
  tick_timer_service_unsubscribe();
  i18n_free_all(data);
  app_free(data);
}

static void prv_init(void) {
  TicTocData *data = app_zalloc_check(sizeof(TicTocData));
  app_state_set_user_data(data);
  setlocale(LC_ALL, "");

  window_init(&data->window, WINDOW_NAME("TicToc"));
  window_set_background_color(&data->window, GColorBlack);

  text_layer_init(&data->text_date_layer, &GRect(8, 68, DISP_COLS - 8, DISP_ROWS - 68));
  text_layer_set_text_color(&data->text_date_layer, GColorWhite);
  text_layer_set_background_color(&data->text_date_layer, GColorClear);
  text_layer_set_font(&data->text_date_layer, fonts_get_system_font(FONT_KEY_ROBOTO_CONDENSED_21));
  layer_add_child(&data->window.layer, &data->text_date_layer.layer);

  text_layer_init(&data->text_time_layer, &GRect(7, 92, DISP_COLS - 7, DISP_ROWS - 92));
  text_layer_set_text_color(&data->text_time_layer, GColorWhite);
  text_layer_set_background_color(&data->text_time_layer, GColorClear);
  text_layer_set_font(&data->text_time_layer,
                      fonts_get_system_font(FONT_KEY_ROBOTO_BOLD_SUBSET_49));
  layer_add_child(&data->window.layer, &data->text_time_layer.layer);

  layer_init(&data->line_layer, &data->window.layer.frame);
  data->line_layer.update_proc = &prv_line_layer_update_callback;
  layer_add_child(&data->window.layer, &data->line_layer);

  tick_timer_service_subscribe(MINUTE_UNIT, prv_minute_tick_handler);

  struct tm time_struct;
  rtc_get_time_tm(&time_struct);
  prv_minute_tick_handler(&time_struct, MINUTE_UNIT);

  app_window_stack_push(&data->window, true);

  // Subscribe to unobstructed area changes
  UnobstructedAreaHandlers unobstructed_handlers = {
    .change = prv_unobstructed_area_change_handler
  };
  app_unobstructed_area_service_subscribe(unobstructed_handlers, NULL);

  // Set initial positions based on unobstructed area
  prv_update_layer_positions();
}

void tictoc_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}
