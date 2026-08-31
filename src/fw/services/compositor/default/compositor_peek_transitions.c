/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/default/compositor_peek_transitions.h"

#include "applib/graphics/framebuffer.h"
#include "applib/graphics/gtypes.h"
#include "applib/ui/animation_interpolate.h"
#include "popups/timeline/peek.h"
#include "popups/timeline/peek_animations.h"

#define NUM_FRAMES (3)

typedef struct CompositorPeekTransitionData {
  int offset_y;
} CompositorPeekTransitionData;

static CompositorPeekTransitionData s_data;

static void prv_update_peek_transition_animation(GContext *ctx, Animation *animation,
                                                 uint32_t distance_normalized) {
  const AnimationProgress progress = distance_normalized;

  GRect box = TIMELINE_PEEK_FRAME_VISIBLE;
  const int16_t initial_offset_y = -4;
  const int16_t final_offset_y = 7;
  box.origin.y = interpolate_int16(progress, box.origin.y + initial_offset_y, final_offset_y);

  if (progress > (ANIMATION_NORMALIZED_MAX / NUM_FRAMES)) {
    timeline_peek_draw_background(ctx, &DISP_FRAME, 0);
    peek_animations_draw_compositor_background_speed_lines(
        ctx, GPoint(PEEK_ANIMATIONS_SPEED_LINES_OFFSET_X, 0));
  }
  const unsigned int num_concurrent = 3;
  timeline_peek_draw_background(ctx, &box, num_concurrent);

  const unsigned int concurrent_height = timeline_peek_get_concurrent_height(num_concurrent);
  const int16_t foreground_speed_line_offset_y = 2;
  const GPoint offset =
      gpoint_add(box.origin, GPoint(PEEK_ANIMATIONS_SPEED_LINES_OFFSET_X,
                                    concurrent_height + foreground_speed_line_offset_y));
  graphics_context_set_fill_color(ctx, GColorBlack);
  peek_animations_draw_compositor_foreground_speed_lines(ctx, offset);
}

static void prv_init_peek_transition_animation(Animation *animation) {
  animation_set_curve(animation, AnimationCurveLinear);
  animation_set_duration(animation, NUM_FRAMES * ANIMATION_TARGET_FRAME_INTERVAL_MS);
}

const CompositorTransition *compositor_peek_transition_timeline_get(void) {
  s_data = (CompositorPeekTransitionData) {};
  static const CompositorTransition s_impl = {
    .init = prv_init_peek_transition_animation,
    .update = prv_update_peek_transition_animation,
  };
  return &s_impl;
}
