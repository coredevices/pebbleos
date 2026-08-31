/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/app_focus_service.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "util/time/time.h"
#include "pbl/util/trig.h"

#if PBL_ROUND
static const int MINUTE_HAND_MARGIN = 16;
static const int HOUR_HAND_MARGIN = 14 * 4;
#else
static const int MINUTE_HAND_MARGIN = 10;
static const int HOUR_HAND_MARGIN = 10 * 4;
#endif
static const int DOT_Y = 8;
static const int STROKE_WIDTH = 8;

typedef struct {
  int hours;
  int minutes;
} Time;

typedef struct {
  Window window;
  Layer canvas_layer;
  Time last_time;
} TicTocData;

static void prv_minute_tick_handler(struct tm *tick_time, TimeUnits changed) {
  TicTocData *data = app_state_get_user_data();

  // Store time
  data->last_time.hours = tick_time->tm_hour;
  data->last_time.hours -= (data->last_time.hours > 12) ? 12 : 0;
  data->last_time.minutes = tick_time->tm_min;

  // Redraw
  layer_mark_dirty(&data->canvas_layer);
}

static void prv_did_focus_handler(bool in_focus) {
  if (!in_focus) {
    return;
  }
  TicTocData *data = app_state_get_user_data();
  layer_mark_dirty(&data->canvas_layer);
}

static void prv_canvas_layer_update_proc(Layer *layer, GContext *ctx) {
  TicTocData *data = app_state_get_user_data();

  const GRect *bounds = &layer->bounds;
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_context_set_stroke_width(ctx, STROKE_WIDTH);
  graphics_context_set_antialiased(ctx, true);

  graphics_fill_rect(ctx, bounds);

  // Get unobstructed bounds to squish clock when obstructed
  GRect unobstructed_bounds;
  layer_get_unobstructed_bounds(layer, &unobstructed_bounds);

  GPoint center = grect_center_point(&unobstructed_bounds);
  int clock_radius = MIN(unobstructed_bounds.size.h, unobstructed_bounds.size.w) / 2;

  Time mode_time = data->last_time;

  // Calculate hand angles
  int minute_angle = TRIG_MAX_ANGLE * mode_time.minutes / 60;
  int hour_angle = (TRIG_MAX_ANGLE * (mode_time.hours * 60 + mode_time.minutes)) / (12 * 60);

  // Calculate hand lengths (with margins)
  int minute_hand_length = clock_radius - MINUTE_HAND_MARGIN;
  int hour_hand_length = clock_radius - HOUR_HAND_MARGIN;

  // Plot hands
  GPoint minute_hand = (GPoint) {
    .x = (int16_t)(sin_lookup(minute_angle) * (int32_t)minute_hand_length
                   / TRIG_MAX_RATIO) + center.x,
    .y = (int16_t)(-cos_lookup(minute_angle) * (int32_t)minute_hand_length
                   / TRIG_MAX_RATIO) + center.y
  };
  GPoint hour_hand = (GPoint) {
    .x = (int16_t)(sin_lookup(hour_angle) * (int32_t)hour_hand_length
                   / TRIG_MAX_RATIO) + center.x,
    .y = (int16_t)(-cos_lookup(hour_angle) * (int32_t)hour_hand_length
                   / TRIG_MAX_RATIO) + center.y
  };

  // Draw hands with positive length only
  if (clock_radius > MINUTE_HAND_MARGIN) {
    graphics_context_set_stroke_color(ctx, GColorWhite);
    graphics_draw_line(ctx, center, minute_hand);
  }

  if (clock_radius > HOUR_HAND_MARGIN) {
    graphics_context_set_stroke_color(ctx, PBL_IF_COLOR_ELSE(GColorRed, GColorWhite));
    graphics_draw_line(ctx, center, hour_hand);
    // fill a circle to make a cleaner center
    graphics_context_set_fill_color(ctx, PBL_IF_COLOR_ELSE(GColorRed, GColorWhite));
    graphics_fill_circle(ctx, center, STROKE_WIDTH / 2);
  }

  // Draw 12 o'clock indicator dot
  graphics_context_set_fill_color(ctx, GColorWhite);
  int center_x = unobstructed_bounds.origin.x + unobstructed_bounds.size.w / 2;
  graphics_fill_circle(ctx, GPoint(center_x, DOT_Y), 3);
}

static void prv_window_load(Window *window) {
  TicTocData *data = app_state_get_user_data();

  Layer *window_layer = window_get_root_layer(window);
  const GRect *window_bounds = &window_layer->bounds;

  layer_init(&data->canvas_layer, window_bounds);
  layer_set_update_proc(&data->canvas_layer, prv_canvas_layer_update_proc);
  layer_add_child(window_layer, &data->canvas_layer);

  AppFocusHandlers focus_handlers = { .did_focus = prv_did_focus_handler };
  app_focus_service_subscribe_handlers(focus_handlers);
}

static void prv_init() {
  TicTocData *data = app_zalloc_check(sizeof(TicTocData));
  app_state_set_user_data(data);

  window_init(&data->window, WINDOW_NAME("TicToc"));
  window_set_window_handlers(&data->window, &(WindowHandlers) {
    .load = prv_window_load
  });
  window_set_user_data(&data->window, data);
  app_window_stack_push(&data->window, true);

  struct tm time_struct;
  rtc_get_time_tm(&time_struct);
  prv_minute_tick_handler(&time_struct, MINUTE_UNIT);

  tick_timer_service_subscribe(MINUTE_UNIT, prv_minute_tick_handler);
}

static void prv_deinit() {
  TicTocData *data = app_state_get_user_data();

  app_focus_service_unsubscribe();
  tick_timer_service_unsubscribe();
  layer_deinit(&data->canvas_layer);
  window_deinit(&data->window);
  app_free(data);
}

void tictoc_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}