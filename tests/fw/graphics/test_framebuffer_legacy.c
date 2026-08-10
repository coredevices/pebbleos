/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/graphics/graphics.h"
#include "applib/graphics/framebuffer.h"

#include "clar.h"
#include "util.h"

// Helper Functions
////////////////////////////////////
#include "test_graphics.h"
#include "8bit/test_framebuffer.h"

// Stubs
////////////////////////////////////
#include "graphics_common_stubs.h"
#include "stubs_applib_resource.h"

// The legacy 3.x framebuffer must be byte-for-byte identical to the chalk
// (spalding) packed circular layout, so chalk apps that access the
// framebuffer directly render correctly.

#define CHALK_GUARD_PAD 76
#define CHALK_FRAMEBUFFER_BYTES 25944

static FrameBuffer *fb = NULL;

void test_framebuffer_legacy__initialize(void) {
  fb = malloc(sizeof(FrameBuffer));
  framebuffer_init(fb, &(GSize) { LEGACY_3X_DISP_COLS, LEGACY_3X_DISP_ROWS });
}

void test_framebuffer_legacy__cleanup(void) {
  free(fb);
}

void test_framebuffer_legacy__chalk_packed_layout(void) {
  const GBitmapDataRowInfoInternal *infos = g_gbitmap_legacy_3x_data_row_infos;

  // Spot-check rows against the original chalk table
  cl_assert_equal_i(infos[0].offset, 0);
  cl_assert_equal_i(infos[0].min_x, 76);
  cl_assert_equal_i(infos[0].max_x, 103);
  cl_assert_equal_i(infos[1].offset, 33);
  cl_assert_equal_i(infos[1].min_x, 71);
  cl_assert_equal_i(infos[1].max_x, 108);
  cl_assert_equal_i(infos[90].min_x, 0);
  cl_assert_equal_i(infos[90].max_x, 179);
  cl_assert_equal_i(infos[179].offset, 25764);
  cl_assert_equal_i(infos[179].min_x, 76);
  cl_assert_equal_i(infos[179].max_x, 103);

  // First visible byte sits after the leading guard pad
  cl_assert_equal_i(infos[0].offset + infos[0].min_x, CHALK_GUARD_PAD);

  size_t next = infos[0].offset + infos[0].min_x;
  for (int y = 0; y < LEGACY_3X_DISP_ROWS; y++) {
    // Circular mask is symmetric vertically and horizontally
    cl_assert_equal_i(infos[y].min_x, infos[LEGACY_3X_DISP_ROWS - 1 - y].min_x);
    cl_assert_equal_i(infos[y].max_x, LEGACY_3X_DISP_COLS - 1 - infos[y].min_x);
    // Visible pixels of each row are packed back-to-back
    cl_assert_equal_i(infos[y].offset + infos[y].min_x, next);
    next += infos[y].max_x - infos[y].min_x + 1;
  }

  // 25792 visible bytes plus guard pads on both ends
  cl_assert_equal_i(next + CHALK_GUARD_PAD, CHALK_FRAMEBUFFER_BYTES);
}

void test_framebuffer_legacy__bitmap_uses_legacy_row_infos(void) {
  GBitmap bitmap = framebuffer_get_as_bitmap(fb, &fb->size);

  cl_assert(bitmap.info.format == GBitmapFormat8BitCircular);
  cl_assert(bitmap.data_row_infos == g_gbitmap_legacy_3x_data_row_infos);

  const GBitmapDataRowInfo row = gbitmap_get_data_row_info(&bitmap, 0);
  cl_assert_equal_i(row.min_x, 76);
  cl_assert_equal_i(row.max_x, 103);
  cl_assert(&row.data[row.min_x] == fb->buffer + CHALK_GUARD_PAD);
}
