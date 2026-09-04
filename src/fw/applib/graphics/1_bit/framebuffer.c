/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/graphics/framebuffer.h"

#include "applib/graphics/gtypes.h"
#include "system/passert.h"

#include <stdint.h>
#include <string.h>

volatile const int FrameBuffer_MaxX = DISP_COLS;
volatile const int FrameBuffer_MaxY = DISP_ROWS;
volatile const int FrameBuffer_BytesPerRow = FRAMEBUFFER_BYTES_PER_ROW;

uint32_t *framebuffer_get_line(FrameBuffer *f, uint8_t y) {
  PBL_ASSERTN(y < f->size.h);

  return f->buffer + (y * ((f->size.w / 32) + 1));
}

inline size_t framebuffer_get_size_bytes(FrameBuffer *f) {
  // TODO: Make FRAMEBUFFER_SIZE_BYTES a macro which takes the cols and rows if we ever want to
  // support different size framebuffers for watches which have native 1-bit framebuffers where the
  // size is not just COLS * ROWS.
  return FRAMEBUFFER_SIZE_BYTES;
}

void framebuffer_clear(FrameBuffer *f) {
  memset(f->buffer, 0xff, framebuffer_get_size_bytes(f));
  framebuffer_dirty_all(f);
  f->is_dirty = true;
}

void framebuffer_mark_dirty_rect(FrameBuffer *f, GRect rect) {
  if (!f->is_dirty) {
    f->dirty_rect = rect;
  } else {
    f->dirty_rect = grect_union(&f->dirty_rect, &rect);
  }

  const GRect clip_rect = (GRect) { GPointZero, f->size };
  grect_clip(&f->dirty_rect, &clip_rect);

  f->is_dirty = true;
}
