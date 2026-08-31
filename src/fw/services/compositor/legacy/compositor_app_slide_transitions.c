/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/legacy/compositor_app_slide_transitions.h"

#include "pbl/services/compositor/compositor_transitions.h"

#include "applib/graphics/bitblt.h"
#include "applib/graphics/framebuffer.h"
#include "applib/graphics/graphics.h"
#include "system/passert.h"

//! Packed so we can squeeze this into a void* as the animation context
typedef struct {
  union {
    //! The direction of the animation of the visual elements
    CompositorTransitionDirection direction;
    void *data;
  };
} AppSlideTransitionAnimationConfiguration;

_Static_assert(sizeof(AppSlideTransitionAnimationConfiguration) == sizeof(void *), "");

void compositor_app_slide_transition_animation_update(GContext *ctx,
                                                      uint32_t distance_normalized,
                                                      CompositorTransitionDirection dir) {
  const bool is_right = (dir == CompositorTransitionDirectionRight);
  const int16_t from = (int16_t)((is_right) ? -DISP_COLS : DISP_COLS);
  const int16_t to = 0;
  const int16_t app_fb_origin_x = interpolate_int16(distance_normalized, from, to);

  // When the window is past its destination (due to the moook), fill in the remaining pixels
  // with black
  if ((is_right && (app_fb_origin_x > to)) || (!is_right && (app_fb_origin_x < to))) {
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_rect(ctx, &DISP_FRAME);
  }

  const GPoint dest_bitmap_blit_offset = GPoint(app_fb_origin_x, 0);

  GBitmap src_bitmap = compositor_get_app_framebuffer_as_bitmap();
  GBitmap dest_bitmap = compositor_get_framebuffer_as_bitmap();
  bitblt_bitmap_into_bitmap(&dest_bitmap, &src_bitmap, dest_bitmap_blit_offset, GCompOpAssign,
                            GColorWhite);
  framebuffer_dirty_all(compositor_get_framebuffer());

  // Update modal position for transparent modals
  compositor_set_modal_transition_offset(dest_bitmap_blit_offset);
}

static void prv_transition_animation_update(GContext *ctx,
                                            Animation *animation,
                                            uint32_t distance_normalized) {
  // Unwrap our animation configuration from the context
  AppSlideTransitionAnimationConfiguration config = {
    .data = animation_get_context(animation)
  };

  compositor_app_slide_transition_animation_update(ctx,
                                                   distance_normalized,
                                                   config.direction);
}

//! The transition direction here is the direction of the visual elements, not the motion
static void prv_configure_transition_animation(Animation *animation,
                                               CompositorTransitionDirection direction) {
  AppSlideTransitionAnimationConfiguration config = {
    .direction = direction,
  };

  animation_set_handlers(animation, (AnimationHandlers) { 0 }, config.data);
  animation_set_custom_interpolation(animation, interpolate_moook);
  animation_set_duration(animation, interpolate_moook_duration());
}

static void prv_transition_from_launcher_animation_init(Animation *animation) {
  prv_configure_transition_animation(animation, CompositorTransitionDirectionRight);
}

static void prv_transition_to_launcher_animation_init(Animation *animation) {
  prv_configure_transition_animation(animation, CompositorTransitionDirectionLeft);
}

const CompositorTransition *compositor_app_slide_transition_get(bool flip_to_the_right) {
  if (compositor_transition_app_to_app_should_be_skipped()) {
    return NULL;
  }

  if (flip_to_the_right) {
    static const CompositorTransition s_impl = {
      .init = prv_transition_to_launcher_animation_init,
      .update = prv_transition_animation_update,
    };
    return &s_impl;
  } else {
    static const CompositorTransition s_impl = {
      .init = prv_transition_from_launcher_animation_init,
      .update = prv_transition_animation_update,
    };
    return &s_impl;
  }
}
