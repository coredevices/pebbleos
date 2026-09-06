/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! @file framebuffer.c
//! Bitdepth independent routines for framebuffer.h
//! Bitdepth dependant routines can be found in the 1_bit & 8_bit folders in their
//! respective framebuffer.c files.

#include "applib/graphics/framebuffer.h"
#include "system/passert.h"

void framebuffer_init(FrameBuffer *fb, const GSize *size) {
  PBL_ASSERTN(!gsize_equal(size, &GSizeZero));
  fb->size = *size;
  framebuffer_reset_dirty(fb);
  // make sure the size is not bigger than the actual buffer size
  PBL_ASSERTN(framebuffer_get_size_bytes(fb) <= FRAMEBUFFER_SIZE_BYTES);
}

GBitmap framebuffer_get_as_bitmap(FrameBuffer *fb, const GSize *size) {
  PBL_ASSERTN(!gsize_equal(size, &GSizeZero));
#if PBL_ROUND
  const GBitmapDataRowInfoInternal *data_row_infos;
  if (fb->size.w == LEGACY_3X_DISP_COLS && fb->size.h == LEGACY_3X_DISP_ROWS) {
    data_row_infos = g_gbitmap_legacy_3x_data_row_infos;
  } else {
    data_row_infos = g_gbitmap_data_row_infos;
  }
#else
  const GBitmapDataRowInfoInternal *data_row_infos = NULL;
#endif

  return (GBitmap) {
    .addr = fb->buffer,
    .row_size_bytes = gbitmap_format_get_row_size_bytes(size->w, GBITMAP_NATIVE_FORMAT),
    .info = (BitmapInfo) {.format = GBITMAP_NATIVE_FORMAT, .version = GBITMAP_VERSION_CURRENT},
    .bounds = (GRect) { GPointZero, *size },
    .data_row_infos = data_row_infos,
  };
}

void framebuffer_dirty_all(FrameBuffer *fb) {
  PBL_ASSERTN(!gsize_equal(&fb->size, &GSizeZero));
  fb->dirty_rect = (GRect) { GPointZero, fb->size };
  fb->is_dirty = true;
}

void framebuffer_reset_dirty(FrameBuffer *fb) {
  PBL_ASSERTN(!gsize_equal(&fb->size, &GSizeZero));
  fb->dirty_rect = GRectZero;
  fb->is_dirty = false;
}

bool framebuffer_is_dirty(FrameBuffer *fb) {
  PBL_ASSERTN(!gsize_equal(&fb->size, &GSizeZero));
  return fb->is_dirty;
}

GSize framebuffer_get_size(FrameBuffer *fb) {
  return fb->size;
}
