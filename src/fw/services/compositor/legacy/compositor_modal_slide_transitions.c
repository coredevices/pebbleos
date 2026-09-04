/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/legacy/compositor_modal_slide_transitions.h"

#include "pbl/services/compositor/compositor_private.h"

#include "applib/graphics/framebuffer.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/animation_interpolate.h"
#include "kernel/ui/kernel_ui.h"
#include "pbl/util/math.h"

#include <string.h>

typedef struct {
  // the y offset of the modal currently within the display
  int32_t cur_modal_offset_y;
  bool modal_is_destination;
} CompositorModalSlideTransitionData;

static CompositorModalSlideTransitionData s_data;

static const int32_t DISP_ROWS_LAST_INDEX = DISP_ROWS - 1;

static void prv_modal_transition_push_update(GContext *ctx, uint32_t distance_normalized) {
  const int16_t new_modal_offset_y = interpolate_int16(distance_normalized,
                                                       DISP_ROWS_LAST_INDEX,
                                                       0);

  // The modal overshoots its destination by a few pixels. When this happens, fill in the pixels
  // at the bottom of the screen with black.
  if (new_modal_offset_y < 0) {
    graphics_fill_rect(
        ctx, &GRect(0, DISP_ROWS + new_modal_offset_y, DISP_COLS, -new_modal_offset_y));
  }

  gpoint_add_eq(&ctx->draw_state.drawing_box.origin, GPoint(0, new_modal_offset_y));
  compositor_render_modal();
}

static void prv_modal_transition_pop_update(GContext *ctx, uint32_t distance_normalized) {
  FrameBuffer *sys_frame_buffer = compositor_get_framebuffer();

  // This is the offset where the modal is to be drawn after the operations below.
  // NOTE: It has to be clipped since our moook interpolate function goes past the destination
  //       and would cause us to write into an invalid memory address in the framebuffer.
  const int32_t new_modal_offset_y = MIN(DISP_ROWS_LAST_INDEX,
                                         interpolate_int16(distance_normalized,
                                                           0,
                                                           DISP_ROWS_LAST_INDEX));
  // This is the delta between the new offset and the previous offset.
  const int32_t modal_offset_delta_y = new_modal_offset_y - s_data.cur_modal_offset_y;

  if (modal_offset_delta_y == 0) {
    // if we aren't going to move the modal, just bail
    return;
  }

  // Start from the bottom of the display (last row index) and copy rows from above to the
  // current line. If we did this the other way, we would lose data from the framebuffer.
  // This produces a sliding down effect.
  const uint32_t start_row = DISP_ROWS_LAST_INDEX;
  const uint32_t end_row = new_modal_offset_y;
  for (unsigned int dest_row = start_row; dest_row >= end_row; dest_row--) {
    int32_t fetch_row = dest_row - modal_offset_delta_y;
    if (fetch_row > DISP_ROWS_LAST_INDEX) {
      continue;
    }

    // copy a row from above and paste it into the destination.
    uint32_t *src_line = framebuffer_get_line(sys_frame_buffer, fetch_row);
    uint32_t *dest_line = framebuffer_get_line(sys_frame_buffer, dest_row);
    memcpy(dest_line, src_line, FRAMEBUFFER_BYTES_PER_ROW);
  }

  // update the current offset of the modal after all lines have been copied.
  s_data.cur_modal_offset_y = new_modal_offset_y;

  // As we move the modal down, we need to show the app that is underneath it.
  // We do this by copying the rows from the app's framebuffer into the system's.
  uint32_t *app_buffer = compositor_get_app_framebuffer_as_bitmap().addr;
  uint32_t *sys_buffer = sys_frame_buffer->buffer;
  memcpy(sys_buffer, app_buffer, FRAMEBUFFER_BYTES_PER_ROW * s_data.cur_modal_offset_y);

  // Render transparent modals over only the revealed app portion
  ctx->draw_state.clip_box.size.h = s_data.cur_modal_offset_y;
  compositor_render_modal();

  framebuffer_dirty_all(sys_frame_buffer);
}

static void prv_transition_animation_update(GContext *ctx, Animation *animation,
                                            uint32_t distance_normalized) {
  if (s_data.modal_is_destination) {
    prv_modal_transition_push_update(ctx, distance_normalized);
  } else {
    prv_modal_transition_pop_update(ctx, distance_normalized);
  }
}

#define NUM_MOOOK_FRAMES_MID 1

static int64_t prv_interpolate_moook_soft(int32_t normalized, int64_t from, int64_t to) {
  return interpolate_moook_soft(normalized, from, to, NUM_MOOOK_FRAMES_MID);
}

static void prv_transition_animation_init(Animation *animation) {
  // Tweaked from observations by the design team
  animation_set_custom_interpolation(animation, prv_interpolate_moook_soft);
  animation_set_duration(animation, interpolate_moook_soft_duration(NUM_MOOOK_FRAMES_MID));
}

const CompositorTransition* compositor_modal_transition_to_modal_get(bool modal_is_destination) {
  // Performs different operations on whether the modal is being pushed or popped.
  s_data = (CompositorModalSlideTransitionData) {
    .modal_is_destination = modal_is_destination,
  };

  static const CompositorTransition s_impl = {
    .init = prv_transition_animation_init,
    .update = prv_transition_animation_update,
    .skip_modal_render_after_update = true, // This transition renders the modal itself
  };

  return &s_impl;
}
