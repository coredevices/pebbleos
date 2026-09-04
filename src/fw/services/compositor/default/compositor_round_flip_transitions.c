/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/default/compositor_round_flip_transitions.h"

#include "pbl/services/compositor/compositor_transitions.h"

#include "applib/graphics/framebuffer.h"
#include "pbl/util/attributes.h"
#include "pbl/util/trig.h"
#include "system/passert.h"

//! Packed so we can squeeze this into a void* as the animation context
typedef struct PACKED {
  union {
    struct {
      //! The direction of the animation of the visual elements
      CompositorTransitionDirection direction:3;
    };
    void *data;
  };
} RoundFlipTransitionAnimationConfiguration;

_Static_assert(sizeof(RoundFlipTransitionAnimationConfiguration) == sizeof(void *), "");

void compositor_round_flip_transitions_flip_animation_update(GContext *ctx,
                                                             uint32_t distance_normalized,
                                                             CompositorTransitionDirection dir,
                                                             GColor flip_lid_color) {
  graphics_context_set_fill_color(ctx, flip_lid_color);

  const int16_t circle_radius = DISP_COLS * 3 / 4;
  const GPoint display_center = GPoint(DISP_COLS / 2, DISP_ROWS / 2);
  // The flip overlap region is the intersection of the two large circles (think of a Venn diagram)
  const uint16_t flip_overlap_region_width = DISP_COLS / 4;

  // Flip halfway through the animation
  const uint32_t flip_distance = ANIMATION_NORMALIZED_MAX / 2;
  if (distance_normalized < flip_distance) {
    const int16_t flip_boundary_from_x = DISP_COLS;
    const int16_t flip_boundary_to_x = display_center.x - (flip_overlap_region_width / 2);
    const int16_t current_flip_boundary_x = interpolate_int16(distance_normalized,
                                                              flip_boundary_from_x,
                                                              flip_boundary_to_x);

    const GPoint circle_center = GPoint(current_flip_boundary_x - circle_radius + 1,
                                        display_center.y);
    if (dir == CompositorTransitionDirectionLeft) {
      graphics_fill_radial_internal(ctx, circle_center, circle_radius,
                                    DISP_COLS - circle_center.x + 1, 0, TRIG_MAX_ANGLE);
    } else {
      graphics_fill_circle(ctx, circle_center, circle_radius);
    }
  } else {
    const int16_t flip_boundary_from_x = display_center.x + (flip_overlap_region_width / 2);
    const int16_t flip_boundary_to_x = 0;
    const int16_t current_flip_boundary_x = interpolate_int16(distance_normalized,
                                                              flip_boundary_from_x,
                                                              flip_boundary_to_x);
    const GPoint circle_center = GPoint(current_flip_boundary_x + circle_radius - 1,
                                        display_center.y);
    if (dir == CompositorTransitionDirectionLeft) {
      graphics_fill_circle(ctx, circle_center, circle_radius);
    } else {
      graphics_fill_radial_internal(ctx, circle_center, circle_radius, circle_center.x + 1, 0,
                                    TRIG_MAX_ANGLE);
    }
  }
}

static void prv_round_flip_transition_animation_update(GContext *ctx, Animation *animation,
                                                       uint32_t distance_normalized) {
  // Unwrap our animation configuration from the context
  RoundFlipTransitionAnimationConfiguration config = {
    .data = animation_get_context(animation)
  };

  // Save a reference to the existing draw implementation
  const GDrawRawImplementation *saved_draw_implementation = ctx->draw_state.draw_implementation;

  // Replace the draw implementation with one that fills horizontal lines using the app framebuffer
  ctx->draw_state.draw_implementation = &g_compositor_transitions_app_fb_draw_implementation;

  // Note that the flip_lid_color here doesn't matter because we've replaced the draw implementation
  // However, we do have to specify a color that isn't invisible, otherwise nothing will be drawn
  compositor_round_flip_transitions_flip_animation_update(ctx, distance_normalized,
                                                          config.direction,
                                                          GColorBlack /* flip_lid_color */);

  // Restore the saved draw implementation
  ctx->draw_state.draw_implementation = saved_draw_implementation;
}

//! The transition direction here is the direction of the visual elements, not the motion
static void prv_configure_round_flip_transition_animation(Animation *animation,
                                                          CompositorTransitionDirection direction) {
  RoundFlipTransitionAnimationConfiguration config = {
    .direction = direction,
  };

  animation_set_curve(animation, AnimationCurveLinear);
  animation_set_duration(animation, ROUND_FLIP_ANIMATION_DURATION_MS);
  animation_set_handlers(animation, (AnimationHandlers) { 0 }, config.data);
  // If the visual elements will move to the right, we will just play the left animation backwards
  const bool should_animate_backwards = (direction == CompositorTransitionDirectionRight);
  animation_set_reverse(animation, should_animate_backwards);
}

static void prv_round_flip_transition_from_launcher_animation_init(Animation *animation) {
  prv_configure_round_flip_transition_animation(animation, CompositorTransitionDirectionRight);
}

static void prv_round_flip_transition_to_launcher_animation_init(Animation *animation) {
  prv_configure_round_flip_transition_animation(animation, CompositorTransitionDirectionLeft);
}

const CompositorTransition *compositor_round_flip_transition_get(bool flip_to_the_right) {
  if (compositor_transition_app_to_app_should_be_skipped()) {
    return NULL;
  }

  if (flip_to_the_right) {
    static const CompositorTransition s_impl = {
      .init = prv_round_flip_transition_to_launcher_animation_init,
      .update = prv_round_flip_transition_animation_update,
    };
    return &s_impl;
  } else {
    static const CompositorTransition s_impl = {
      .init = prv_round_flip_transition_from_launcher_animation_init,
      .update = prv_round_flip_transition_animation_update,
    };
    return &s_impl;
  }
}
