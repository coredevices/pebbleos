/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "progress_layer.h"

#include "system/passert.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/layer.h"
#include "pbl/util/math.h"

static int16_t scale_progress_bar_width_px(unsigned int progress_percent, int16_t rect_width_px) {
  return ((progress_percent * (rect_width_px)) / 100);
}

void progress_layer_set_progress(ProgressLayer* progress_layer, unsigned int progress_percent) {
  // Can't use the clip macro here because it fails with uints
  progress_layer->progress_percent = MIN(100, progress_percent);
  layer_mark_dirty(&progress_layer->layer);
}

void progress_layer_update_proc(ProgressLayer* progress_layer, GContext* ctx) {
  const GRect *bounds = &progress_layer->layer.bounds;

  int16_t progress_bar_width_px = scale_progress_bar_width_px(progress_layer->progress_percent,
                                                              bounds->size.w);
  const GRect progress_bar = GRect(bounds->origin.x, bounds->origin.y, progress_bar_width_px,
                                   bounds->size.h);


  const int16_t corner_radius = progress_layer->corner_radius;
  graphics_context_set_fill_color(ctx, progress_layer->background_color);
  graphics_fill_round_rect(ctx, bounds, corner_radius, GCornersAll);

  // Draw the progress bar
  graphics_context_set_fill_color(ctx, progress_layer->foreground_color);
  graphics_fill_round_rect(ctx, &progress_bar, corner_radius, GCornersAll);

#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 1
  graphics_context_set_stroke_color(ctx, progress_layer->foreground_color);
  graphics_draw_round_rect(ctx, bounds, corner_radius);
#endif
}

void progress_layer_init(ProgressLayer* progress_layer, const GRect *frame) {
  *progress_layer = (ProgressLayer){};

  layer_init(&progress_layer->layer, frame);
  progress_layer->layer.update_proc = (LayerUpdateProc) progress_layer_update_proc;
  progress_layer->foreground_color = GColorBlack;
  progress_layer->background_color = GColorWhite;
  progress_layer->corner_radius = 1;
}

void progress_layer_deinit(ProgressLayer* progress_layer) {
  layer_deinit(&progress_layer->layer);
}

void progress_layer_set_foreground_color(ProgressLayer* progress_layer, GColor color) {
  progress_layer->foreground_color = color;
}

void progress_layer_set_background_color(ProgressLayer* progress_layer, GColor color) {
  progress_layer->background_color = color;
}

void progress_layer_set_corner_radius(ProgressLayer* progress_layer, uint16_t corner_radius) {
  progress_layer->corner_radius = corner_radius;
}
