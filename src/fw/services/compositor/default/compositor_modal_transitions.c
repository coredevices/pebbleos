/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/default/compositor_modal_transitions.h"

#include "pbl/services/compositor/compositor_transitions.h"
#include "pbl/services/compositor/compositor_private.h"

#include "applib/graphics/framebuffer.h"
#include "pbl/util/trig.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/graphics/gdraw_command_sequence.h"
#include "apps/system/timeline/common.h"
#include "kernel/ui/kernel_ui.h"
#include "resource/resource_ids.auto.h"


// No animations will be shown on the following platforms
#if defined(CONFIG_RECOVERY_FW)
#define MODAL_CONTRACT_TO_MODAL_ANIMATION (RESOURCE_ID_INVALID)
#define MODAL_CONTRACT_FROM_MODAL_ANIMATION (RESOURCE_ID_INVALID)
#define MODAL_EXPAND_TO_APP_ANIMATION (RESOURCE_ID_INVALID)
#else
#define MODAL_CONTRACT_TO_MODAL_ANIMATION (RESOURCE_ID_MODAL_CONTRACT_TO_MODAL_SEQUENCE)
#define MODAL_CONTRACT_FROM_MODAL_ANIMATION (RESOURCE_ID_MODAL_CONTRACT_FROM_MODAL_SEQUENCE)
#define MODAL_EXPAND_TO_APP_ANIMATION (RESOURCE_ID_MODAL_EXPAND_TO_APP_SEQUENCE)
#endif

typedef struct {
  GColor outer_color;
  bool modal_is_destination;
  bool expanding;
  GDrawCommandSequence *animation_sequence;
} CompositorModalTransitionData;

static CompositorModalTransitionData s_data;

static void prv_modal_transition_animation_teardown_rect(Animation *animation);

static void prv_modal_transition_animation_init_sequence(const uint32_t resource_id) {
  prv_modal_transition_animation_teardown_rect(NULL);
  s_data.animation_sequence = gdraw_command_sequence_create_with_resource(resource_id);
}

static void prv_modal_transition_fill_update(GContext *ctx,
                                             uint32_t distance_normalized,
                                             bool inner) {
  const GColor replace_color = GColorGreen;
  const GColor stroke_color = TIMELINE_DOT_COLOR;
  compositor_transition_pdcs_animation_update(
      ctx, s_data.animation_sequence, distance_normalized, replace_color, stroke_color,
      s_data.outer_color /* overdraw color */, inner, NULL);
}

void prv_render_modal_if_necessary(void) {
  // Since modal windows don't have a framebuffer that we can use in the compositor animation,
  // draw the modal now (if one exists) so the modal compositor animations can draw on top of it,
  // revealing the relevant parts of the modal window throughout the animation
  if (s_data.modal_is_destination) {
    compositor_render_modal();
  }
}

static NOINLINE void prv_render_transition_rect(GContext *ctx, Animation *animation,
                                                uint32_t distance_normalized) {
  // If the modal is the destination, just draw the frame and fill its inner ring with the app's
  // frame buffer
  if (s_data.modal_is_destination) {
    prv_modal_transition_fill_update(ctx, distance_normalized, true /* fill inner */);
    return;
  }

  // For the first half of the animation where the app is the destination, draw the "contract
  // from modal" frame and fill its outer ring with the background color specified by
  // s_data.outer_color
  const uint32_t contract_to_dot_distance = ANIMATION_NORMALIZED_MAX / 2;
  if (distance_normalized < contract_to_dot_distance) {
    // Switch to the "contract from modal" animation if necessary (e.g. if the animation was
    // reversed in the future)
    if (s_data.expanding) {
      prv_modal_transition_animation_init_sequence(MODAL_CONTRACT_FROM_MODAL_ANIMATION);
      s_data.expanding = false;
    }
    distance_normalized = animation_timing_scaled(distance_normalized,
                                                  0,
                                                  contract_to_dot_distance);
    prv_modal_transition_fill_update(ctx, distance_normalized, false /* fill outer */);
  } else {
    // For the second half of the animation where the app is the destination, draw the "expand to
    // app" frame and fill its inner ring with the app's frame buffer
    // Switch to the "expand to app" animation if necessary
    if (!s_data.expanding) {
      prv_modal_transition_animation_init_sequence(MODAL_EXPAND_TO_APP_ANIMATION);
      s_data.expanding = true;
    }
    distance_normalized = animation_timing_scaled(distance_normalized,
                                                  contract_to_dot_distance,
                                                  ANIMATION_NORMALIZED_MAX);
    prv_modal_transition_fill_update(ctx, distance_normalized, true /* fill inner */);
  }
}

static void prv_modal_transition_animation_update_rect(GContext *ctx, Animation *animation,
                                                       uint32_t distance_normalized) {
  prv_render_modal_if_necessary();
  prv_render_transition_rect(ctx, animation, distance_normalized);
}

static NOINLINE void prv_render_transition_round(GContext *ctx, Animation *animation,
                                                 uint32_t distance_normalized) {
  const int16_t dot_radius = DOT_ANIMATION_STROKE_WIDTH / 2;
  const GRect display_bounds = ctx->draw_state.clip_box;
  const GPoint circle_center = grect_center_point(&display_bounds);

  // Calculate the inner/outer radii for the colored radial and the dot radial
  const int16_t dot_ring_outer_radius_from = (display_bounds.size.w / 2) + (dot_radius * 2);
  const int16_t dot_ring_outer_radius_to = dot_radius;
  const int16_t interpolated_dot_ring_outer_radius = interpolate_int16(distance_normalized,
                                                                       dot_ring_outer_radius_from,
                                                                       dot_ring_outer_radius_to);
  const int16_t dot_ring_inner_radius = interpolated_dot_ring_outer_radius - dot_radius;

  // Draw the dot ring
  graphics_context_set_fill_color(ctx, TIMELINE_DOT_COLOR);
  graphics_fill_radial_internal(ctx, circle_center, dot_ring_inner_radius,
                                interpolated_dot_ring_outer_radius, 0, TRIG_MAX_ANGLE);

  // Save a reference to the existing draw implementation
  const GDrawRawImplementation *saved_draw_implementation = ctx->draw_state.draw_implementation;

  // Replace the draw implementation with one that fills horizontal lines using the app framebuffer
  ctx->draw_state.draw_implementation = &g_compositor_transitions_app_fb_draw_implementation;

  // Fill the inside of the dot ring with the app framebuffer
  graphics_fill_circle(ctx, circle_center, dot_ring_inner_radius);

  // Restore the saved draw implementation
  ctx->draw_state.draw_implementation = saved_draw_implementation;
}

static void prv_modal_push_transition_animation_update_round(GContext *ctx, Animation *animation,
                                                             uint32_t distance_normalized) {
  prv_render_modal_if_necessary();
  prv_render_transition_round(ctx, animation, distance_normalized);
}

static void prv_modal_transition_animation_init_rect(Animation *animation) {
  const uint32_t resource_id = s_data.modal_is_destination ?
                                  MODAL_CONTRACT_TO_MODAL_ANIMATION :
                                  MODAL_CONTRACT_FROM_MODAL_ANIMATION;
  prv_modal_transition_animation_init_sequence(resource_id);

  // Tweaked from observations by the design team
  const uint32_t duration = s_data.modal_is_destination ? 310 : 800;
  if (s_data.animation_sequence) {
    animation_set_duration(animation, duration);
    animation_set_curve(animation, AnimationCurveLinear);
  }
}

static void prv_modal_push_transition_animation_init_round(Animation *animation) {
  const uint32_t duration_ms = 8 * ANIMATION_TARGET_FRAME_INTERVAL_MS;
  animation_set_duration(animation, duration_ms);
  animation_set_curve(animation, AnimationCurveLinear);
}

static void prv_modal_transition_animation_teardown_rect(Animation *animation) {
  if (s_data.animation_sequence) {
    gdraw_command_sequence_destroy(s_data.animation_sequence);
    s_data.animation_sequence = NULL;
  }
}

const CompositorTransition *prv_modal_transition_get_rect(bool modal_is_destination) {
  // NOTE: This initialization will set .expanding to false so we default to contracting to a dot
  s_data = (CompositorModalTransitionData) {
    .modal_is_destination = modal_is_destination,
    // TODO: PBL-19849
    // Decide on the logistics of the background color of the modal pop animation, including
    // providing a setter for it that apps can use
    .outer_color = GColorLightGray,
  };

  static const CompositorTransition s_impl = {
    .init = prv_modal_transition_animation_init_rect,
    .update = prv_modal_transition_animation_update_rect,
    .teardown = prv_modal_transition_animation_teardown_rect,
    .skip_modal_render_after_update = true, // This transition renders the modal itself
  };

  return &s_impl;
}

const CompositorTransition *prv_modal_transition_get_round(bool modal_is_destination) {
  s_data = (CompositorModalTransitionData) {
    .modal_is_destination = modal_is_destination,
  };

  if (!modal_is_destination) {
    return compositor_round_flip_transition_get(false /* flip_to_the_right */);
  } else {
    static const CompositorTransition s_impl = {
      .init = prv_modal_push_transition_animation_init_round,
      .update = prv_modal_push_transition_animation_update_round,
      .skip_modal_render_after_update = true, // This transition renders the modal itself
    };

    return &s_impl;
  }
}

const CompositorTransition* compositor_modal_transition_to_modal_get(bool modal_is_destination) {
  return PBL_IF_RECT_ELSE(prv_modal_transition_get_rect,
                          prv_modal_transition_get_round)(modal_is_destination);
}
