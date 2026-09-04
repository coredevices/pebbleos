/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/default/compositor_port_hole_transitions.h"

#include "applib/graphics/framebuffer.h"
#include "applib/graphics/graphics_private.h"
#include "pbl/util/trig.h"
#include "pbl/services/compositor/compositor_transitions.h"

#include "resource/resource_ids.auto.h"
#include <pbl/logging/logging.h>

typedef struct {
  CompositorTransitionDirection direction;
  int16_t animation_offset_px;
} CompositorPortHoleTransitionData;

static CompositorPortHoleTransitionData s_data;

void compositor_port_hole_transition_draw_outer_ring(GContext *ctx, int16_t thickness,
                                                     GColor ring_color) {
  const uint16_t overdraw = 2;
  const GRect frame = (GRect) { .origin = GPointZero,
                                .size = graphics_context_get_framebuffer_size(ctx) };
  graphics_context_set_fill_color(ctx, ring_color);
  graphics_fill_radial(ctx, grect_inset(frame, GEdgeInsets(-overdraw)),
                       GOvalScaleModeFitCircle, thickness + overdraw, 0, TRIG_MAX_ANGLE);
}

// piecewise interpolator between 0 and to for the first half of ANIMATION_NORMALIZED_MAX
// and between -to and 0 for the second half
static int16_t prv_interpolate_two_ways_int16(AnimationProgress normalized_progress,
                                              int32_t discontinuity_progress, int16_t to) {
  if (normalized_progress < discontinuity_progress) {
    return interpolate_int16(animation_timing_scaled(normalized_progress, 0,
                                                     discontinuity_progress), 0, to);
  } else {
    return interpolate_int16(animation_timing_scaled(normalized_progress, discontinuity_progress,
                                                     ANIMATION_NORMALIZED_MAX), -to, 0);
  }
}

static void prv_port_hole_transition_animation_init(Animation *animation) {
  animation_set_duration(animation, PORT_HOLE_TRANSITION_DURATION_MS);
  s_data.animation_offset_px = 0;
}

static void prv_port_hole_transition_animation_update(GContext *ctx,
                                                      Animation *animation,
                                                      uint32_t distance_normalized) {
  const uint32_t transition_progress_threshold = ANIMATION_NORMALIZED_MAX / 2;
  const int32_t ring_max_thickness = 40;
  const bool direction_negative = ((s_data.direction == CompositorTransitionDirectionRight) ||
                                   (s_data.direction == CompositorTransitionDirectionDown));
  const bool direction_vertical = ((s_data.direction == CompositorTransitionDirectionDown) ||
                                   (s_data.direction == CompositorTransitionDirectionUp));

  const int16_t current_offset_px =
      prv_interpolate_two_ways_int16(distance_normalized, transition_progress_threshold,
                                     direction_negative ? ring_max_thickness :
                                                          -ring_max_thickness);

  if (distance_normalized > transition_progress_threshold) {
    // Second half of the transition
    const GRect rect = direction_vertical ? GRect(0, -current_offset_px, DISP_COLS, DISP_ROWS - current_offset_px) :
                                              GRect(-current_offset_px, 0, DISP_COLS - current_offset_px, DISP_ROWS);

    compositor_scaled_app_fb_copy(rect, false /* copy_relative_to_origin */);
  } else {
    // First half of the transition
    const int16_t diff = s_data.animation_offset_px - current_offset_px;
    if (direction_vertical) {
      graphics_private_move_pixels_vertically(&ctx->dest_bitmap, diff);
    } else {
      graphics_private_move_pixels_horizontally(&ctx->dest_bitmap, diff,
                                                false /* patch_garbage */);
    }
  }

  compositor_port_hole_transition_draw_outer_ring(ctx, ABS(current_offset_px), GColorBlack);
  s_data.animation_offset_px = current_offset_px;
}

const CompositorTransition *compositor_port_hole_transition_app_get(
    CompositorTransitionDirection direction) {
  if (compositor_transition_app_to_app_should_be_skipped()) {
    return NULL;
  }

  s_data.direction = direction;

  static const CompositorTransition s_impl = {
    .init = prv_port_hole_transition_animation_init,
    .update = prv_port_hole_transition_animation_update,
  };

  return &s_impl;
}
