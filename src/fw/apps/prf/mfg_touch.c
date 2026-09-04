/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */


#include "applib/app.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/text.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "apps/prf/mfg_test_result.h"
#include "kernel/pbl_malloc.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include <pbl/logging/logging.h>
#include "applib/touch_service.h"
#include "pbl/services/light.h"
#include "pbl/util/math.h"

#define TOUCH_SUPPORT_DEBUG    0

#define TEST_TIMEOUT_S (30)
#define WINDOW_POP_TIME_S (3)

// Boustrophedon (snake) grid on both display shapes. On round displays each
// row only spans the chord of the circle at its height, so rows near the top
// and bottom hold fewer columns and only the middle runs full width.
#define GRID_ROWS (5)
#define GRID_COLS (5)
#define GRID_SPACING_X (PBL_DISPLAY_WIDTH / GRID_COLS)
#define GRID_SPACING_Y (PBL_DISPLAY_HEIGHT / GRID_ROWS)
#define MAX_DOTS (GRID_ROWS * GRID_COLS)
// Wall lines sit halfway between lanes: allow deviations right up to them
#define PATH_TOLERANCE_PX (GRID_SPACING_X / 2)
#if PBL_ROUND
// Keep dots this far inside the circle edge, where touch gets unreliable
#define ROUND_EDGE_INSET_PX (12)
#endif

#define DOT_RADIUS_PX (4)
#define DOT_CAPTURE_RADIUS_PX (14)
// Segments ahead of the current one a sample may match, so a fast drag with
// sparse samples cannot stall progress at a skipped dot. Kept well below the
// ring/row separation so a stroke cannot tunnel to another path section.
#define PATH_LOOKAHEAD_SEGMENTS (3)
#define TRACE_MIN_DIST_PX (3)
#define MAX_TRACE_POINTS (640)

typedef struct {
  Window window;
  GPoint dots[MAX_DOTS];
  uint8_t num_dots;
#if PBL_ROUND
  GPoint wall_start[GRID_ROWS - 1];
  GPoint wall_end[GRID_ROWS - 1];
#endif
  GPoint trace[MAX_TRACE_POINTS];
  uint16_t trace_len;
  uint8_t next_dot;
  uint8_t best_progress;
  bool tracking;
  TextLayer status;
  char status_string[35];
  uint32_t seconds_remaining;
  bool test_complete;
} AppData;

static void prv_complete_test(AppData *data, bool passed) {
  data->test_complete = true;
  data->seconds_remaining = WINDOW_POP_TIME_S;

  mfg_test_result_report(MfgTestId_Touch, passed, data->best_progress);

  sniprintf(data->status_string, sizeof(data->status_string), "%s", passed ? "PASS!" : "FAIL!");
  text_layer_set_text(&data->status, data->status_string);
  // The status overlay is transparent: repaint the whole window under it
  layer_mark_dirty(&data->window.layer);
}

static void prv_handle_second_tick(struct tm *tick_time, TimeUnits units_changed) {
  AppData *data = app_state_get_user_data();

  if (data->test_complete) {
    if (--data->seconds_remaining == 0) {
      app_window_stack_pop(true);
    }
    return;
  }

  if (data->seconds_remaining == 0) {
    prv_complete_test(data, false);
  } else {
    sniprintf(data->status_string, sizeof(data->status_string),
              "%" PRIu32 "s", data->seconds_remaining);
    text_layer_set_text(&data->status, data->status_string);
    layer_mark_dirty(&data->window.layer);
    data->seconds_remaining--;
  }
}

static uint32_t prv_dist_sq(GPoint a, GPoint b) {
  int32_t dx = a.x - b.x;
  int32_t dy = a.y - b.y;
  return dx * dx + dy * dy;
}

static uint32_t prv_dist_sq_to_segment(GPoint p, GPoint a, GPoint b) {
  int32_t abx = b.x - a.x;
  int32_t aby = b.y - a.y;
  int32_t apx = p.x - a.x;
  int32_t apy = p.y - a.y;
  int32_t len_sq = abx * abx + aby * aby;
  int32_t t_num = apx * abx + apy * aby;
  int32_t cx = 0;
  int32_t cy = 0;

  if (len_sq > 0 && t_num >= len_sq) {
    cx = abx;
    cy = aby;
  } else if (len_sq > 0 && t_num > 0) {
    cx = (abx * t_num) / len_sq;
    cy = (aby * t_num) / len_sq;
  }

  int32_t dx = apx - cx;
  int32_t dy = apy - cy;
  return dx * dx + dy * dy;
}

static void prv_reset_stroke(AppData *data) {
  // Keep the trace on screen so the operator can see where the stroke went
  // wrong; it is discarded when a new stroke starts
  data->next_dot = 0;
  data->tracking = false;
}

static void prv_trace_append(AppData *data, GPoint p) {
  if (data->trace_len == MAX_TRACE_POINTS) {
    return;
  }
  if (data->trace_len > 0 &&
      prv_dist_sq(p, data->trace[data->trace_len - 1]) <
          TRACE_MIN_DIST_PX * TRACE_MIN_DIST_PX) {
    return;
  }
  data->trace[data->trace_len++] = p;
}

//! Draw the snake polyline guide path in the current stroke color/width
static void prv_draw_guide_path(GContext *ctx, AppData *data) {
  for (uint8_t i = 0; i < data->num_dots - 1; i++) {
    graphics_draw_line(ctx, data->dots[i], data->dots[i + 1]);
  }
}

//! Draw a single maze-style wall between each pair of adjacent lanes, with a
//! gap where the path crosses over
static void prv_draw_walls(GContext *ctx, AppData *data) {
#if PBL_ROUND
  for (uint8_t k = 0; k < GRID_ROWS - 1; k++) {
    graphics_draw_line(ctx, data->wall_start[k], data->wall_end[k]);
  }
#else
  for (uint8_t k = 1; k < GRID_ROWS; k++) {
    int16_t y = GRID_SPACING_Y / 2 + (k - 1) * GRID_SPACING_Y + GRID_SPACING_Y / 2;
    if (k % 2 == 1) {
      // Rows connect on the right: wall spans from the left edge
      graphics_draw_line(ctx, GPoint(0, y),
                         GPoint(PBL_DISPLAY_WIDTH - GRID_SPACING_X, y));
    } else {
      graphics_draw_line(ctx, GPoint(GRID_SPACING_X, y),
                         GPoint(PBL_DISPLAY_WIDTH - 1, y));
    }
  }
#endif
}

static void prv_update_proc(struct Layer *layer, GContext* ctx) {
  AppData *data = app_state_get_user_data();

  // This update proc replaces the default window one: clear the background
  // ourselves so stale trace pixels do not linger
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, &layer->bounds);

  // Keep frames cheap: touch samples arrive faster than antialiased arcs
  // can be rendered, which would back up the app event queue
  graphics_context_set_antialiased(ctx, false);

  graphics_context_set_stroke_width(ctx, 3);

  graphics_context_set_stroke_color(ctx, GColorMelon);
  prv_draw_walls(ctx, data);

  graphics_context_set_stroke_color(ctx, GColorLightGray);
  prv_draw_guide_path(ctx, data);

  for (uint8_t i = 0; i < data->num_dots; i++) {
    graphics_context_set_fill_color(ctx, i < data->next_dot ? GColorGreen : GColorBlack);
    graphics_fill_circle(ctx, data->dots[i], DOT_RADIUS_PX);
  }

  graphics_context_set_stroke_color(ctx, GColorBlack);
  for (uint16_t i = 1; i < data->trace_len; i++) {
    graphics_draw_line(ctx, data->trace[i - 1], data->trace[i]);
  }
}

static void prv_touch_event_handler(const TouchEvent *event, void *context) {
  AppData *data = app_state_get_user_data();

  if (data->test_complete) {
    return;
  }

  GPoint p = GPoint(event->x, event->y);

#if TOUCH_SUPPORT_DEBUG
  PBL_LOG_INFO("type:%d x:%d y:%d", event->type, event->x, event->y);
#endif

  if (event->type == TouchEvent_Liftoff) {
    // The path must be completed in a single stroke
    prv_reset_stroke(data);
    layer_mark_dirty(&data->window.layer);
    return;
  }

  if (!data->tracking) {
    // Stroke starts anywhere within the corridor mouth around the first dot
    if (prv_dist_sq(p, data->dots[0]) <=
        PATH_TOLERANCE_PX * PATH_TOLERANCE_PX) {
      data->trace_len = 0;
      data->tracking = true;
      data->next_dot = 1;
      prv_trace_append(data, p);
      layer_mark_dirty(&data->window.layer);
    }
    return;
  }

  // Match the sample against the nearest guide segment, from the previous
  // one up to a small lookahead past the current one
  uint32_t best_dist_sq = UINT32_MAX;
  uint8_t best_end = data->next_dot;
  uint8_t first_end = (data->next_dot >= 2) ? data->next_dot - 1 : data->next_dot;
  for (uint8_t end = first_end;
       end < data->num_dots && end <= data->next_dot + PATH_LOOKAHEAD_SEGMENTS; end++) {
    uint32_t dist_sq = prv_dist_sq_to_segment(p, data->dots[end - 1], data->dots[end]);
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_end = end;
    }
  }

  if (best_dist_sq > PATH_TOLERANCE_PX * PATH_TOLERANCE_PX) {
    // Off the corridor (finger slip or distorted coordinates): reset stroke
    prv_reset_stroke(data);
    layer_mark_dirty(&data->window.layer);
    return;
  }

  // A sample on segment (best_end - 1 -> best_end) means all prior dots
  // were passed; the endpoint itself is captured within its radius
  uint8_t prev_next_dot = data->next_dot;
  uint16_t prev_trace_len = data->trace_len;
  if (best_end > data->next_dot) {
    data->next_dot = best_end;
  }
  while (data->next_dot < data->num_dots &&
         prv_dist_sq(p, data->dots[data->next_dot]) <=
             DOT_CAPTURE_RADIUS_PX * DOT_CAPTURE_RADIUS_PX) {
    data->next_dot++;
  }
  if (data->next_dot > data->best_progress) {
    data->best_progress = data->next_dot;
  }

  prv_trace_append(data, p);
  // Only request a redraw when something visible changed
  if (data->next_dot != prev_next_dot || data->trace_len != prev_trace_len) {
    layer_mark_dirty(&data->window.layer);
  }

  if (data->next_dot == data->num_dots) {
    prv_complete_test(data, true);
  }
}

static void prv_init_dots(AppData *data) {
#if PBL_ROUND
  const GPoint center = GPoint(PBL_DISPLAY_WIDTH / 2, PBL_DISPLAY_HEIGHT / 2);
  const int32_t radius = MIN(PBL_DISPLAY_WIDTH, PBL_DISPLAY_HEIGHT) / 2;
  const int32_t dot_radius = radius - ROUND_EDGE_INSET_PX;

  data->num_dots = 0;
  for (uint8_t row = 0; row < GRID_ROWS; row++) {
    int32_t y = GRID_SPACING_Y / 2 + row * GRID_SPACING_Y;
    int32_t dy = y - center.y;
    int32_t half_width = integer_sqrt(dot_radius * dot_radius - dy * dy);
    // As many columns as the chord fits at the nominal spacing, at least the
    // two chord ends
    uint8_t cols = CLIP(1 + (2 * half_width) / GRID_SPACING_X, 2, GRID_COLS);
    uint8_t first = data->num_dots;

    for (uint8_t col = 0; col < cols; col++) {
      uint8_t eff_col = (row % 2 == 0) ? col : cols - 1 - col;
      data->dots[data->num_dots].x =
          center.x - half_width + (2 * half_width * eff_col) / (cols - 1);
      data->dots[data->num_dots].y = y;
      data->num_dots++;
    }

    if (row > 0) {
      // Wall on the row boundary spans its chord, leaving a gap around the
      // connector between the two rows
      int16_t wall_y = row * GRID_SPACING_Y;
      int32_t wall_dy = wall_y - center.y;
      int32_t wall_half = integer_sqrt(radius * radius - wall_dy * wall_dy);
      GPoint prev_end = data->dots[first - 1];
      GPoint row_start = data->dots[first];
      if (row % 2 == 1) {
        // Rows connect on the right: wall spans from the left chord end
        int16_t gate_x = MIN(prev_end.x, row_start.x) - PATH_TOLERANCE_PX;
        data->wall_start[row - 1] = GPoint(center.x - wall_half, wall_y);
        data->wall_end[row - 1] = GPoint(gate_x, wall_y);
      } else {
        int16_t gate_x = MAX(prev_end.x, row_start.x) + PATH_TOLERANCE_PX;
        data->wall_start[row - 1] = GPoint(gate_x, wall_y);
        data->wall_end[row - 1] = GPoint(center.x + wall_half, wall_y);
      }
    }
  }
#else
  int16_t spacing_x = PBL_DISPLAY_WIDTH / GRID_COLS;
  int16_t spacing_y = PBL_DISPLAY_HEIGHT / GRID_ROWS;

  // Boustrophedon (snake) ordering: even rows left-to-right, odd rows right-to-left
  for (uint8_t row = 0; row < GRID_ROWS; row++) {
    for (uint8_t col = 0; col < GRID_COLS; col++) {
      uint8_t eff_col = (row % 2 == 0) ? col : GRID_COLS - 1 - col;
      data->dots[row * GRID_COLS + col].x = spacing_x / 2 + eff_col * spacing_x;
      data->dots[row * GRID_COLS + col].y = spacing_y / 2 + row * spacing_y;
    }
  }
  data->num_dots = GRID_ROWS * GRID_COLS;
#endif
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));
  *data = (AppData) {
    .seconds_remaining = TEST_TIMEOUT_S,
  };

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  Layer *layer = window_get_root_layer(window);
  layer_set_update_proc(layer, prv_update_proc);
  app_window_stack_push(window, true /* Animated */);

  prv_init_dots(data);
  layer_mark_dirty(&data->window.layer);

  TextLayer *status = &data->status;
  // Centered overlay; kept small so it covers as little of the path as possible
  text_layer_init(status,
                  &GRect(PBL_DISPLAY_WIDTH / 2 - 34, PBL_DISPLAY_HEIGHT / 2 - 20, 68, 40));
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_24));
  text_layer_set_text_alignment(status, GTextAlignmentCenter);
  text_layer_set_background_color(status, GColorWhite);
  layer_add_child(&window->layer, &status->layer);

  touch_service_subscribe(prv_touch_event_handler, NULL);

  tick_timer_service_subscribe(SECOND_UNIT, prv_handle_second_tick);
}

static void s_main(void) {
  light_enable(true);

  prv_handle_init();

  app_event_loop();

  touch_service_unsubscribe();
  light_enable(false);
}

const PebbleProcessMd* mfg_touch_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: a53e7d1c-d2ee-4592-96b9-5d33a46237db
    .common.uuid = { 0xa5, 0x3e, 0x7d, 0x1c, 0xd2, 0xee, 0x45, 0x92,
                     0x96, 0xb9, 0x5d, 0x33, 0xa4, 0x62, 0x37, 0xdb },
    .name = "MfgTouch",
  };
  return (const PebbleProcessMd*) &s_app_info;
}
