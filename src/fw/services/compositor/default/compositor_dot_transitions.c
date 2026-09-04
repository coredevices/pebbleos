/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/default/compositor_dot_transitions.h"

#include "pbl/services/compositor/compositor_private.h"
#include "pbl/services/compositor/compositor_transitions.h"

#include "apps/system/timeline/common.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/ui/animation_timing.h"
#include "applib/graphics/framebuffer.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/gpath.h"
#include "pbl/util/trig.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

static CompositorTransitionDirection prv_flip_transition_direction(
  CompositorTransitionDirection direction) {
  switch (direction) {
    case CompositorTransitionDirectionUp:
      return CompositorTransitionDirectionDown;
    case CompositorTransitionDirectionDown:
      return CompositorTransitionDirectionUp;
    case CompositorTransitionDirectionLeft:
      return CompositorTransitionDirectionRight;
    case CompositorTransitionDirectionRight:
      return CompositorTransitionDirectionLeft;
    default:
      return CompositorTransitionDirectionNone;
  }
}

#if PBL_RECT
//! linear interpolation between two GPoints, supports delay and clamping
//! @param delay value to postpone interpolation (in range 0..ANIMATION_NORMALIZED_MAX)
static GPoint prv_gpoint_interpolate(int32_t delay, int32_t normalized,
                                     const GPoint from, const GPoint to) {
  normalized = CLIP(normalized - delay, 0, ANIMATION_NORMALIZED_MAX);
  normalized = animation_timing_curve(normalized, AnimationCurveEaseInOut);
  GPoint result;
  result.x = interpolate_int16(normalized, from.x, to.x);
  result.y = interpolate_int16(normalized, from.y, to.y);
  return result;
}

//! Returns a new point halfway between two provided points
static GPoint prv_gpoint_mid(const GPoint a, const GPoint b) {
  return GPoint((a.x + b.x) / 2, (a.y + b.y) / 2);
}
//! Draw the "collapse" portion of the animation. This function can either work by drawing
//! an outer ring using the fill_cb, or by drawing and expanding inner portion using the fill_cb.
//! This behaviour is configured by the inner bool.
static void prv_collapse_animation(GContext *ctx, uint32_t distance_normalized, bool inner,
                                   GPathDrawFilledCallback ring_fill_cb) {

  const GRect bounds = ctx->draw_state.clip_box;
  PBL_ASSERTN(bounds.origin.x == 0 && bounds.origin.y == 0);

  int32_t rel_p = distance_normalized;

  // calculate dynamic positions for top-left (tl), top-right (tr), bottom-right (br), etc.
  // offset by stroke_width (sw) makes sure stroke is completely invisible at beginning/end
  const uint8_t sw = interpolate_int16(rel_p, 11, DOT_ANIMATION_STROKE_WIDTH);
  const GSize size = bounds.size;
  // outer points
  const GPoint tl = GPoint(0, 0);
  const GPoint tr = GPoint(size.w, 0);
  const GPoint br = GPoint(size.w, size.h);
  const GPoint bl = GPoint(0, size.h);

  const GPoint center = GPoint(size.w / 2, size.h / 2);

  // inner points
  // these magic numbers are nominators/denominators (e.g. 7) to reflect the visual effect of the
  // provided video
  const int32_t d = ANIMATION_NORMALIZED_MAX / 7;
  // pause at the end/beginning to create a total pause of 2*pause
  int32_t pause = 0;
  rel_p = rel_p * (7 + 4 + pause) / 7;
  // delays for each point between collapsing and expanding - hand-tweaked
  const GPoint scaled_tl = prv_gpoint_interpolate(0 * d, rel_p, tl, center);
  const GPoint scaled_tr = prv_gpoint_interpolate(1 * d, rel_p, tr, center);
  const GPoint scaled_bl = prv_gpoint_interpolate(3 * d, rel_p, bl, center);
  const GPoint scaled_br = prv_gpoint_interpolate(4 * d, rel_p, br, center);

  const GPoint scaled_l = prv_gpoint_mid(scaled_tl, scaled_bl);
  const GPoint l = GPoint(-sw, scaled_l.y);

  if (inner) {
    // gpath that creates the inner section
    GPoint path_points[] = { scaled_bl, scaled_br, scaled_tr, scaled_tl, };

    GPath path = {
      .num_points = ARRAY_LENGTH(path_points),
      .points = path_points
    };

    gpath_draw_filled_with_cb(ctx, &path, ring_fill_cb, NULL);
  } else {
    // gpath that creates a solid "ring"
    GPoint path_points[] = {
        tl, tr, br, bl,
        l, scaled_l,
        scaled_bl, scaled_br, scaled_tr, scaled_tl,
        scaled_l, l,
    };

    GPath path = {
      .num_points = ARRAY_LENGTH(path_points),
      .points = path_points
    };

    gpath_draw_filled_with_cb(ctx, &path, ring_fill_cb, NULL);
  }

  ctx->draw_state.stroke_width = sw;

  graphics_draw_line(ctx, scaled_tl, scaled_tr);
  graphics_draw_line(ctx, scaled_tr, scaled_br);
  graphics_draw_line(ctx, scaled_br, scaled_bl);
  graphics_draw_line(ctx, scaled_bl, scaled_tl);
}

//! Callback to be used with prv_collapse_animation to fill with the current fill_color
static void prv_gpath_draw_filled_cb(GContext *ctx, int16_t y,
                                     Fixed_S16_3 x_range_begin, Fixed_S16_3 x_range_end,
                                     Fixed_S16_3 delta_begin, Fixed_S16_3 delta_end,
                                     void *user_data) {

  const GRect fill_rect = GRect(x_range_begin.integer + 1, y,
                                x_range_end.integer - x_range_begin.integer - 1, 1);
  graphics_fill_rect(ctx, &fill_rect);
}
#endif // PBL_RECT

//! Draw a dumb dot at the supplied position with the supplied color
static void prv_draw_dot(GContext *ctx, GPoint pos, GColor color) {
  ctx->draw_state.stroke_width = DOT_ANIMATION_STROKE_WIDTH;
  graphics_context_set_stroke_color(ctx, color);
  graphics_draw_line(ctx, pos, pos);
}

//! Packed so we can squeeze this into a void* as the animation context
typedef struct PACKED {
  union {
    struct {
      //! Whether or not to collapse the starting screen of the animation to a dot
      bool collapse_starting_animation:1;
      //! The direction the animation is moving
      CompositorTransitionDirection direction:3;
      //! The animation's dot color after collapsing
      GColor collapse_dot_color;
      //! The animation's final dot color
      GColor final_dot_color;
      //! The background color during the animation
      GColor background_color;
    };
    void *data;
  };
} DotTransitionAnimationConfiguration;

_Static_assert(sizeof(DotTransitionAnimationConfiguration) == sizeof(void *), "");

#if PBL_RECT
static void prv_collapse_animation_update_rect(GContext *ctx,
                                               DotTransitionAnimationConfiguration config,
                                               uint32_t distance_normalized) {
  GPathDrawFilledCallback draw_filled_cb;
  bool inner;

  ctx->draw_state.fill_color = config.background_color;

  if (config.collapse_starting_animation) {
    // Don't blank here because this intended to be an "in place" operation. The data the makes up
    // the center of the collapse is only present in the system framebuffer at this point so we
    // need to be careful not to wipe it all out.

    // Draw in an outer ring that expands of the background color.
    draw_filled_cb = prv_gpath_draw_filled_cb;
    inner = false;
  } else {
    // First blank out any left overs from a previous frame to make sure we have a solid color
    // background.
    graphics_fill_rect(ctx, &ctx->draw_state.clip_box);

    // Draw in an expanding inner ring of the incoming app framebuffer.
    // Note that this only expands because we're running the animation backwards.
    draw_filled_cb = compositor_app_framebuffer_fill_callback;
    inner = true;
  }
  graphics_context_set_stroke_color(ctx, config.collapse_dot_color);

  prv_collapse_animation(ctx, distance_normalized, inner, draw_filled_cb);
}
#endif

void compositor_dot_transitions_collapsing_ring_animation_update(GContext *ctx,
                                                                 uint32_t distance_normalized,
                                                                 GColor outer_ring_color,
                                                                 GColor inner_ring_color) {
  const int16_t dot_radius = DOT_ANIMATION_STROKE_WIDTH / 2;
  const GRect bounds = ctx->draw_state.clip_box;
  const GPoint center = grect_center_point(&bounds);

  // Calculate the inner/outer radii for the outer radial and the inner radial
  const int16_t outer_radial_outer_radius = (bounds.size.w / 2) + (dot_radius * 2);
  const int16_t outer_radial_inner_radius_from = (bounds.size.w / 2) + dot_radius;
  const int16_t outer_radial_inner_radius_to = dot_radius;
  const int16_t interpolated_outer_radial_inner_radius = interpolate_int16(distance_normalized,
    outer_radial_inner_radius_from, outer_radial_inner_radius_to);
  const int16_t inner_radial_outer_radius = interpolated_outer_radial_inner_radius;
  const int16_t inner_radial_inner_radius = inner_radial_outer_radius - dot_radius;

  // Draw an outer ring to show the collapsing/expanding to/from a dot
  graphics_context_set_stroke_color(ctx, outer_ring_color);
  graphics_context_set_fill_color(ctx, outer_ring_color);

  graphics_fill_radial_internal(ctx, center, interpolated_outer_radial_inner_radius,
                                outer_radial_outer_radius, 0, TRIG_MAX_ANGLE);

  // The outer ring also has a small inner ring with a radial width equal to the dot radius
  graphics_context_set_stroke_color(ctx, inner_ring_color);
  graphics_context_set_fill_color(ctx, inner_ring_color);
  graphics_fill_radial_internal(ctx, center, inner_radial_inner_radius, inner_radial_outer_radius,
                                0,
                                TRIG_MAX_ANGLE);
}

#if PBL_ROUND
static void prv_collapse_animation_update_round(GContext *ctx,
                                                DotTransitionAnimationConfiguration config,
                                                uint32_t distance_normalized) {
  // If we're expanding, blit the app framebuffer into the system framebuffer (so below the ring)
  if (!config.collapse_starting_animation) {
    compositor_scaled_app_fb_copy(GRect(0, 0, DISP_COLS, DISP_ROWS), false /* copy_relative_to_origin */);
  }

  compositor_dot_transitions_collapsing_ring_animation_update(ctx, distance_normalized,
                                                              config.background_color,
                                                              config.collapse_dot_color);
}
#endif

static void prv_collapse_animation_update(GContext *ctx,
                                          DotTransitionAnimationConfiguration config,
                                          uint32_t distance_normalized) {
  PBL_IF_RECT_ELSE(prv_collapse_animation_update_rect,
                   prv_collapse_animation_update_round)(ctx, config, distance_normalized);
}

static void prv_static_dot_transition_animation_update(
    GContext *ctx, Animation *animation, uint32_t distance_normalized) {
  DotTransitionAnimationConfiguration config = {
    .data = animation_get_context(animation)
  };

  const uint32_t COLLAPSE_END_DISTANCE = 7 * (ANIMATION_NORMALIZED_MAX / 8);
  const GRect bounds = ctx->draw_state.clip_box;
  const GPoint center = grect_center_point(&bounds);

  if (distance_normalized < COLLAPSE_END_DISTANCE) {
    const uint32_t local_distance = animation_timing_scaled(distance_normalized,
                                                            0,
                                                            COLLAPSE_END_DISTANCE);
    prv_collapse_animation_update(ctx, config, local_distance);
  } else {
    prv_draw_dot(ctx, center, config.collapse_dot_color);
  }
}

static void prv_configure_dot_transition_animation(Animation *animation,
                                                   GColor collapse_dot_color,
                                                   GColor final_dot_color,
                                                   GColor background_color,
                                                   CompositorTransitionDirection direction,
                                                   uint32_t duration,
                                                   bool collapse_starting_animation) {
  // Flip the direction and dot colors if we aren't starting with a collapsing animation
  // because we reverse the animation below
  if (!collapse_starting_animation) {
    direction = prv_flip_transition_direction(direction);

    GColor swap_collapse_dot_color = collapse_dot_color;
    collapse_dot_color = final_dot_color;
    final_dot_color = swap_collapse_dot_color;
  }

  DotTransitionAnimationConfiguration config = {
    .collapse_starting_animation = collapse_starting_animation,
    .collapse_dot_color = collapse_dot_color,
    .final_dot_color = final_dot_color,
    .background_color = background_color,
    .direction = direction
  };

  animation_set_curve(animation, AnimationCurveLinear);
  animation_set_duration(animation, duration);
  animation_set_handlers(animation, (AnimationHandlers) { 0 }, config.data);
  animation_set_reverse(animation, !collapse_starting_animation);
}

static void prv_dot_transition_to_timeline_past_animation_init(Animation *animation) {
  prv_configure_dot_transition_animation(animation, TIMELINE_DOT_COLOR,
                                         TIMELINE_DOT_COLOR, TIMELINE_PAST_COLOR,
                                         CompositorTransitionDirectionUp,
                                         STATIC_DOT_ANIMATION_DURATION_MS, false);
}

static void prv_dot_transition_from_timeline_past_animation_init(Animation *animation) {
  prv_configure_dot_transition_animation(animation, TIMELINE_DOT_COLOR,
                                         TIMELINE_DOT_COLOR, TIMELINE_PAST_COLOR,
                                         CompositorTransitionDirectionDown,
                                         STATIC_DOT_ANIMATION_DURATION_MS, true);
}

static void prv_dot_transition_to_timeline_future_animation_init(Animation *animation) {
  prv_configure_dot_transition_animation(animation, TIMELINE_DOT_COLOR,
                                         TIMELINE_DOT_COLOR, TIMELINE_FUTURE_COLOR,
                                         CompositorTransitionDirectionUp,
                                         STATIC_DOT_ANIMATION_DURATION_MS, true);
}

static void prv_dot_transition_from_timeline_future_animation_init(Animation *animation) {
  prv_configure_dot_transition_animation(animation, TIMELINE_DOT_COLOR,
                                         TIMELINE_DOT_COLOR, TIMELINE_FUTURE_COLOR,
                                         CompositorTransitionDirectionDown,
                                         STATIC_DOT_ANIMATION_DURATION_MS, false);
}

static void prv_dot_transition_from_app_fetch_animation_init(Animation *animation) {
  prv_configure_dot_transition_animation(animation, GColorWhite,
                                         GColorWhite, GColorLightGray,
                                         CompositorTransitionDirectionNone,
                                         STATIC_DOT_ANIMATION_DURATION_MS, false);
}


const CompositorTransition* compositor_dot_transition_timeline_get(bool timeline_is_future,
                                                                   bool timeline_is_destination) {
  if (compositor_transition_app_to_app_should_be_skipped()) {
    return NULL;
  }

  if (timeline_is_future) {
    if (timeline_is_destination) {
      static const CompositorTransition s_impl = {
        .init = prv_dot_transition_to_timeline_future_animation_init,
        .update = prv_static_dot_transition_animation_update,
      };
      return &s_impl;
    } else {
      static const CompositorTransition s_impl = {
        .init = prv_dot_transition_from_timeline_future_animation_init,
        .update = prv_static_dot_transition_animation_update,
      };
      return &s_impl;
    }
  } else {
    if (timeline_is_destination) {
      static const CompositorTransition s_impl = {
        .init = prv_dot_transition_from_timeline_past_animation_init,
        .update = prv_static_dot_transition_animation_update,
      };
      return &s_impl;
    } else {
      static const CompositorTransition s_impl = {
        .init = prv_dot_transition_to_timeline_past_animation_init,
        .update = prv_static_dot_transition_animation_update,
      };
      return &s_impl;
    }
  }
}

const CompositorTransition* compositor_dot_transition_app_fetch_get(void) {
  if (compositor_transition_app_to_app_should_be_skipped()) {
    return NULL;
  }

  static const CompositorTransition s_impl = {
    .init =  prv_dot_transition_from_app_fetch_animation_init,
    .update = prv_static_dot_transition_animation_update,
  };
  return &s_impl;
}
