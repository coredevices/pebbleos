/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "watch_model.h"

#include "applib/app.h"
#include "applib/fonts/fonts.h"
#include "applib/graphics/gpath.h"
#include "applib/graphics/graphics_circle.h"
#include "applib/graphics/text.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "util/time/time.h"

typedef struct {
  Window window;
  ClockModel clock_model;
  GBitmap *bg_bitmap;
  GBitmap *tz_bitmap[NUM_NON_LOCAL_CLOCKS];
} MultiWatchData;

static GPath* prv_pointed_hand_path(GContext *ctx, ClockHand *hand) {
  uint32_t num_points = 5;
  if (hand->backwards_extension > 0) num_points = 9;

  GPoint *points = (GPoint*)malloc(num_points * sizeof(GPoint));
  if (!points) return NULL;

  points[0] = GPoint(hand->thickness / -2, hand->thickness); // top left
  points[1] = GPoint(hand->thickness / -2, -(hand->length - hand->thickness / 2)); // bottom left
  points[2] = GPoint(0, -(hand->length)); // point
  points[3] = GPoint(hand->thickness / 2, -(hand->length - hand->thickness / 2)); // bottom right
  points[4] = GPoint(hand->thickness / 2, hand->thickness); // top right
  if (hand->backwards_extension > 0) {
    points[5] = GPoint(hand->thickness / 4, hand->thickness); // bottom right
    points[6] = GPoint(hand->thickness / 4, hand->thickness + hand->backwards_extension); // top right
    points[7] = GPoint(hand->thickness / -4, hand->thickness + hand->backwards_extension); // top left
    points[8] = GPoint(hand->thickness / -4, hand->thickness); // bottom left
  }

  GPathInfo info = {
    .num_points = num_points,
    .points = points,
  };

  GPath *path = gpath_create(&info);
  return path;
}

static GPath* prv_square_hand_path(GContext *ctx, ClockHand *hand) {
  uint32_t num_points = 4;
  if (hand->backwards_extension > 0) num_points = 8;

  GPoint *points = (GPoint*)malloc(num_points * sizeof(GPoint));
  if (!points) return NULL;

  points[0] = GPoint(hand->thickness / -2, hand->thickness); // top left
  points[1] = GPoint(hand->thickness / -2, -(hand->length)); // bottom left
  points[2] = GPoint(hand->thickness / 2, -(hand->length)); // bottom right
  points[3] = GPoint(hand->thickness / 2, hand->thickness); // top right
  if (hand->backwards_extension > 0) {
    points[4] = GPoint(hand->thickness / 4, hand->thickness); // bottom right
    points[5] = GPoint(hand->thickness / 4, hand->thickness + hand->backwards_extension); // top right
    points[6] = GPoint(hand->thickness / -4, hand->thickness + hand->backwards_extension); // top left
    points[7] = GPoint(hand->thickness / -4, hand->thickness); // bottom left
  }

  GPathInfo info = {
    .num_points = num_points,
    .points = points,
  };

  GPath *path = gpath_create(&info);
  return path;
}

void watch_model_handle_change(ClockModel *model) {
  MultiWatchData *data = app_state_get_user_data();
  data->clock_model = *model;
  layer_mark_dirty(window_get_root_layer(&data->window));
}

static GPointPrecise prv_gpoint_from_polar(const GPointPrecise *center, uint32_t distance,
                                                int32_t angle) {
  return gpoint_from_polar_precise(center, distance << GPOINT_PRECISE_PRECISION, angle);
}

static void prv_graphics_draw_centered_text(GContext *ctx, const GSize *max_size,
                                            const GPoint *center, const GFont font,
                                            const GColor color, const char *text) {
  GSize text_size = app_graphics_text_layout_get_content_size(
      text, font, (GRect) { .size = *max_size }, GTextOverflowModeFill, GTextAlignmentCenter);
  GPoint text_center = *center;
  text_center.x -= text_size.w / 2 + 1;
  text_center.y -= text_size.h * 2 / 3;
  graphics_context_set_text_color(ctx, color);
  graphics_draw_text(ctx, text, font, (GRect) { .origin = text_center, .size = text_size },
                     GTextOverflowModeFill, GTextAlignmentCenter, NULL);
}

static void prv_draw_watch_hand_rounded(GContext *ctx, ClockHand *hand, GPointPrecise center) {
  GPointPrecise watch_hand_end = prv_gpoint_from_polar(&center, hand->length, hand->angle);
  if (hand->style == CLOCK_HAND_STYLE_ROUNDED_WITH_HIGHLIGHT) {
    graphics_context_set_stroke_color(ctx, GColorWhite);
    graphics_line_draw_precise_stroked_aa(ctx, center, watch_hand_end, hand->thickness + 2);
  }
  graphics_context_set_stroke_color(ctx, hand->color);
  graphics_line_draw_precise_stroked_aa(ctx, center, watch_hand_end, hand->thickness);
}

static void prv_draw_watch_hand_pointed(GContext *ctx, ClockHand *hand, GPoint center) {
  GPath *path = prv_pointed_hand_path(ctx, hand);
  graphics_context_set_fill_color(ctx, hand->color);
  gpath_rotate_to(path, hand->angle);
  gpath_move_to(path, center);
  gpath_draw_filled(ctx, path);
  gpath_destroy(path);
}

static void prv_draw_watch_hand_square(GContext *ctx, ClockHand *hand, GPoint center) {
  GPath *path = prv_square_hand_path(ctx, hand);
  graphics_context_set_fill_color(ctx, hand->color);
  gpath_rotate_to(path, hand->angle);
  gpath_move_to(path, center);
  gpath_draw_filled(ctx, path);
  gpath_destroy(path);
}

static void prv_draw_watch_hand(GContext *ctx, ClockHand *hand, GPointPrecise center) {
  switch (hand->style) {
    case CLOCK_HAND_STYLE_POINTED:
      prv_draw_watch_hand_pointed(ctx, hand, GPointFromGPointPrecise(center));
      break;
    case CLOCK_HAND_STYLE_SQUARE:
      prv_draw_watch_hand_square(ctx, hand, GPointFromGPointPrecise(center));
      break;
    case CLOCK_HAND_STYLE_ROUNDED:
    case CLOCK_HAND_STYLE_ROUNDED_WITH_HIGHLIGHT:
    default:
      prv_draw_watch_hand_rounded(ctx, hand, center);
      break;
  }
}

static GPointPrecise prv_get_clock_center_point(ClockLocation location, const GRect *bounds) {
  GPoint imprecise_center_point = {0};
  switch (location) {
    case CLOCK_LOCATION_TOP:
      imprecise_center_point = (GPoint) {
        .x = bounds->size.w / 2,
        .y = bounds->size.h / 4,
      };
      break;
    case CLOCK_LOCATION_RIGHT:
      imprecise_center_point = (GPoint) {
        .x = bounds->size.w * 3 / 4 - 5,
        .y = bounds->size.h / 2,
      };
      break;
    case CLOCK_LOCATION_BOTTOM:
      imprecise_center_point = (GPoint) {
        .x = bounds->size.w / 2,
        .y = bounds->size.h * 3 / 4 + 6,
      };
      break;
    case CLOCK_LOCATION_LEFT:
      imprecise_center_point = (GPoint) {
        .x = bounds->size.w / 4 + 4,
        .y = bounds->size.h / 2,
      };
      break;
    default:
      // aiming for width / 2 - 0.5 to get the true center
      return (GPointPrecise) {
        .x = { .integer = bounds->size.w / 2 - 1, .fraction = 3 },
        .y = { .integer = bounds->size.h / 2 - 1, .fraction = 3 }
      };
  }
  return GPointPreciseFromGPoint(imprecise_center_point);
}

static void prv_draw_clock_text(GContext *ctx, ClockText text, GPoint center) {
  MultiWatchData *data = app_state_get_user_data();
  const GRect *bounds = &window_get_root_layer(&data->window)->bounds;
  switch(text.location) {
    case CLOCK_TEXT_LOCATION_LEFT:
      const GRect box = (GRect) { .origin = GPoint(center.x - text.offset - bounds->size.w, center.y - text.font_size * 2 / 3), .size = bounds->size };
      graphics_draw_text(ctx, text.buffer, text.font, box,
                         GTextOverflowModeFill, GTextAlignmentRight, NULL);
      break;
    case CLOCK_TEXT_LOCATION_BOTTOM:
    default:
      const GPoint point = (GPoint) { center.x, center.y + text.offset };
      prv_graphics_draw_centered_text(ctx, &bounds->size, &point, text.font,
                                      text.color, text.buffer);
    break;
  }
}

static void prv_draw_clock_face(GContext *ctx, ClockFace *face) {
  MultiWatchData *data = app_state_get_user_data();
  const GRect *bounds = &window_get_root_layer(&data->window)->bounds;
  const GPointPrecise center = prv_get_clock_center_point(face->location, bounds);

  // Draw text.
  prv_draw_clock_text(ctx, face->text, GPointFromGPointPrecise(center));

  // Draw hands.
  prv_draw_watch_hand(ctx, &face->hour_hand, center);
  prv_draw_watch_hand(ctx, &face->minute_hand, center);

  // Draw bob.
  GRect bob_rect = (GRect) {
      .origin = GPoint(GPointFromGPointPrecise(center).x - face->bob_radius, GPointFromGPointPrecise(center).y - face->bob_radius),
      .size = GSize(face->bob_radius * 2, face->bob_radius * 2)
  };
  GRect bob_center_rect = (GRect) {
      .origin = GPoint(GPointFromGPointPrecise(center).x - face->bob_center_radius, GPointFromGPointPrecise(center).y - face->bob_center_radius),
      .size = GSize(face->bob_center_radius * 2, face->bob_center_radius * 2)
  };
  graphics_context_set_fill_color(ctx, face->bob_color);
  graphics_fill_oval(ctx, bob_rect, GOvalScaleModeFitCircle);
  graphics_context_set_fill_color(ctx, face->bob_center_color);
  graphics_fill_oval(ctx, bob_center_rect, GOvalScaleModeFitCircle);
}


static void prv_update_proc(Layer *layer, GContext *ctx) {
  MultiWatchData *data = app_state_get_user_data();

  const GRect *bounds = &layer->bounds;

  // Background.
  graphics_draw_bitmap_in_rect(ctx, data->bg_bitmap, bounds);
  graphics_context_set_compositing_mode(ctx, GCompOpSet);

  ClockModel *clock_model = &data->clock_model;

  // Draw the clocks.
  for (uint32_t i = 0; i < clock_model->num_non_local_clocks; ++i) {
    // Draw clock background
    GRect bitmap_bounds = gbitmap_get_bounds(data->tz_bitmap[i]);
    GPointPrecise bitmap_center = prv_get_clock_center_point(clock_model->non_local_clock[i].location, bounds);
    bitmap_bounds.origin.x = GPointFromGPointPrecise(bitmap_center).x - (bitmap_bounds.size.w / 2);
    bitmap_bounds.origin.y = GPointFromGPointPrecise(bitmap_center).y - (bitmap_bounds.size.h / 2);
    graphics_draw_bitmap_in_rect(ctx, data->tz_bitmap[i], &bitmap_bounds);
    // Draw clock foreground
    prv_draw_clock_face(ctx, &clock_model->non_local_clock[i]);
  }
  prv_draw_clock_face(ctx, &clock_model->local_clock);
}

static void prv_window_load(Window *window) {
  MultiWatchData *data = app_state_get_user_data();

  layer_set_update_proc(window_get_root_layer(window), prv_update_proc);

  watch_model_init();

  data->bg_bitmap = gbitmap_create_with_resource(data->clock_model.bg_bitmap_id);
  for (uint32_t i = 0; i < data->clock_model.num_non_local_clocks; ++i) {
    data->tz_bitmap[i] = gbitmap_create_with_resource(data->clock_model.non_local_clock[i].bg_bitmap_id);
  }
}

static void prv_window_unload(Window *window) {
  MultiWatchData *data = app_state_get_user_data();
  gbitmap_destroy(data->bg_bitmap);
  for (uint32_t i = 0; i < data->clock_model.num_non_local_clocks; ++i) {
    gbitmap_destroy(data->tz_bitmap[i]);
  }
}

static void prv_app_did_focus(bool did_focus) {
  if (!did_focus) {
    return;
  }
  app_focus_service_unsubscribe();
  watch_model_start_intro();
}

static void prv_init(void) {
  MultiWatchData *data = app_zalloc_check(sizeof(*data));
  app_state_set_user_data(data);

  window_init(&data->window, "TicToc");
  window_set_window_handlers(&data->window, &(WindowHandlers) {
    .load = prv_window_load,
    .unload = prv_window_unload,
  });
  const bool animated = true;
  app_window_stack_push(&data->window, animated);

  app_focus_service_subscribe_handlers((AppFocusHandlers) {
    .did_focus = prv_app_did_focus,
  });
}

static void prv_deinit(void) {
  MultiWatchData *data = app_state_get_user_data();
  window_destroy(&data->window);
  watch_model_cleanup();
}

void tictoc_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}
