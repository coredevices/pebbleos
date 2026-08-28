/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/default/compositor_slide_transitions.h"

#include "pbl/services/compositor/compositor_private.h"
#include "pbl/services/compositor/compositor_transitions.h"

#include "apps/system/timeline/common.h"
#include "applib/graphics/bitblt.h"
#include "applib/graphics/framebuffer.h"
#include "applib/graphics/gtypes.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/action_bar_layer_private.h"
#include "applib/ui/animation_interpolate.h"
#include "popups/timeline/peek.h"
#include <pbl/logging/logging.h>

// TODO: PBL-31388 Factor out vertical compositor slide animations
// This does a similar transition to the legacy modal slide transition
// With a few tweaks, this compositor animation can drive both

typedef struct {
  int16_t offset_y;
  bool slide_up;
  bool timeline_is_destination;
  bool timeline_is_empty;
  GColor fill_color;
} CompositorSlideTransitionData;

static CompositorSlideTransitionData s_data;

static void prv_copy_framebuffer_rows(GBitmap *dest_bitmap, GBitmap *src_bitmap, int16_t start_row,
                                      int16_t end_row, int16_t dupe_row, int16_t shift_amount) {
  const int16_t delta = start_row > end_row ? -1 : 1;
  for (int16_t dest_row = start_row; dest_row != end_row; dest_row += delta) {
    src_bitmap->bounds = (GRect) {
      .origin.y = (dupe_row >= 0 ? dupe_row : dest_row) - shift_amount,
      .size = { DISP_COLS, 1 },
    };
    bitblt_bitmap_into_bitmap(dest_bitmap, src_bitmap, GPoint(0, dest_row), GCompOpAssign,
                              GColorWhite);
  }
}

static void prv_shift_framebuffer_rows(GBitmap *dest_bitmap, int16_t start_row, int16_t end_row,
                                       int16_t shift_amount) {
  GBitmap src_bitmap = *dest_bitmap;
  prv_copy_framebuffer_rows(dest_bitmap, &src_bitmap, start_row, end_row, -1, shift_amount);
}

static void prv_duplicate_framebuffer_row(GBitmap *dest_bitmap, int16_t start_row, int16_t end_row,
                                          GBitmap *src_bitmap, int16_t dupe_row) {
  const int16_t delta = start_row > end_row ? -1 : 1;
  for (int16_t dest_row = start_row; dest_row != end_row; dest_row += delta) {
    int16_t offset_y = (dupe_row >= 0 ? dupe_row : dest_row) - dest_row;
    compositor_scaled_app_fb_copy_offset(GRect(0, dest_row, DISP_COLS, 1), false /* copy_relative_to_origin */, offset_y);
  }
}

static void prv_slide_transition_animation_update(GContext *ctx, Animation *animation,
                                                  uint32_t distance_normalized) {
  const AnimationProgress progress = distance_normalized;
  const int last_offset_y = s_data.offset_y;
  const int delta_rows = s_data.slide_up ? -DISP_ROWS : DISP_ROWS;
  s_data.offset_y = interpolate_int16(progress, 0, delta_rows);
  // Advance one line to give movement from the very start only for the full curve
  const int advance = s_data.timeline_is_destination ? 0 : 1;

  // Whether shifting should occur. offset_y starts at 0, and |delta_rows| is DISP_ROWS
  const bool should_shift =
      s_data.slide_up ? (s_data.offset_y > delta_rows) : (s_data.offset_y < delta_rows);
  // Horizontal framebuffer lines are shifted vertically, these indicate the start and end of the
  // entire region to be shifted
  int shift_start_row;
  int shift_end_row;
  // Lines that shift with no replacement framebuffer line are filled if we are entering timeline
  int fill_offset_y;
  int fill_height;
  // Otherwise they are overdrawn with lines from the app framebuffer
  int app_offset_y;
  // When the app framebuffer overshoots, the line nearest the overshoot needs to be duplicated
  bool app_should_dupe;
  int app_dupe_row;

  if (s_data.slide_up) {
    s_data.offset_y -= advance;
    shift_start_row = 0;
    shift_end_row = DISP_ROWS + s_data.offset_y;
    fill_offset_y = shift_end_row;
    fill_height = DISP_ROWS - shift_end_row;
    app_offset_y = shift_end_row;
    app_should_dupe = (app_offset_y < 0);
    app_dupe_row = DISP_ROWS - 1;
  } else {
    s_data.offset_y += advance;
    shift_start_row = DISP_ROWS;
    shift_end_row = s_data.offset_y;
    fill_offset_y = 0;
    fill_height = shift_end_row;
    app_offset_y = shift_end_row - DISP_ROWS + 1;
    app_should_dupe = (app_offset_y > 0);
    app_dupe_row = 0;
  }

  GBitmap dest_bitmap = compositor_get_framebuffer_as_bitmap();
  if (should_shift) {
    const int shift_amount = s_data.offset_y - last_offset_y;
    prv_shift_framebuffer_rows(&dest_bitmap, shift_start_row, shift_end_row, shift_amount);
  }
  if (s_data.timeline_is_destination) {
    if (!s_data.timeline_is_empty) {
      graphics_context_set_fill_color(ctx, GColorWhite);
      const int content_width = DISP_COLS - TIMELINE_PEEK_ICON_BOX_WIDTH;
      const bool sidebar_on_right = action_bar_layer_is_on_right();
      const int content_x = sidebar_on_right ? 0 : TIMELINE_PEEK_ICON_BOX_WIDTH;
      const int sidebar_x = sidebar_on_right ? content_width : 0;
      graphics_fill_rect(ctx, &GRect(content_x, fill_offset_y, content_width, fill_height));
      graphics_context_set_fill_color(ctx, s_data.fill_color);
      graphics_fill_rect(ctx, &GRect(sidebar_x, fill_offset_y, TIMELINE_PEEK_ICON_BOX_WIDTH,
                                    fill_height));
    } else {
      graphics_context_set_fill_color(ctx, s_data.fill_color);
      graphics_fill_rect(ctx, &GRect(0, fill_offset_y, DISP_COLS, fill_height));
    }
  } else {
    compositor_scaled_app_fb_copy(GRect(0, app_offset_y, DISP_COLS, DISP_ROWS), false /* copy_relative_to_origin */);
    if (app_should_dupe) {
      GBitmap app_bitmap = compositor_get_app_framebuffer_as_bitmap();
      prv_duplicate_framebuffer_row(&dest_bitmap, app_dupe_row, app_offset_y + app_dupe_row,
                                    &app_bitmap, app_dupe_row);
    }
  }
  framebuffer_dirty_all(compositor_get_framebuffer());

  // Update modal position for transparent modals
  compositor_set_modal_transition_offset(GPoint(0, app_offset_y));
}

static void prv_slide_transition_animation_init(Animation *animation) {
  // Give a regular moook more time to stretch the anticipation
  const uint32_t duration = s_data.timeline_is_destination ? interpolate_moook_in_duration() :
                                                             interpolate_moook_duration();
  const InterpolateInt64Function interpolation =
      s_data.timeline_is_destination ? interpolate_moook_in_only : interpolate_moook;
  animation_set_duration(animation, duration);
  animation_set_custom_interpolation(animation, interpolation);
}

const CompositorTransition *prv_slide_transition_get(void) {
  static const CompositorTransition s_impl = {
    .init = prv_slide_transition_animation_init,
    .update = prv_slide_transition_animation_update,
  };
  return &s_impl;
}

const CompositorTransition *compositor_slide_transition_timeline_get(
    bool timeline_is_future, bool timeline_is_destination, bool timeline_is_empty) {
  s_data = (CompositorSlideTransitionData) {
    .slide_up = timeline_is_future ^ !timeline_is_destination,
    .fill_color = timeline_is_future ? TIMELINE_FUTURE_COLOR : TIMELINE_PAST_COLOR,
    .timeline_is_destination = timeline_is_destination,
    .timeline_is_empty = timeline_is_empty,
  };
  return prv_slide_transition_get();
}

const CompositorTransition *compositor_slide_transition_app_get(bool slide_up) {
  // timeline_is_destination unset: the incoming app framebuffer is drawn sliding in,
  // pushing the current framebuffer out.
  s_data = (CompositorSlideTransitionData) {
    .slide_up = slide_up,
  };
  return prv_slide_transition_get();
}
