/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gtypes.h"
#include "pbl/util/attributes.h"

#include <stdint.h>
#include <stdbool.h>

#define FRAMEBUFFER_BYTES_PER_ROW DISP_COLS
#define FRAMEBUFFER_SIZE_BYTES DISPLAY_FRAMEBUFFER_BYTES

#ifndef UNITTEST
typedef struct FrameBuffer {
  uint8_t buffer[FRAMEBUFFER_SIZE_BYTES];
  GSize size; //<! Active size of the framebuffer
  GRect dirty_rect; //<! Smallest rect covering all dirty pixels.
  bool is_dirty;
} FrameBuffer;
#else // UNITTEST
// For unit-tests, the framebuffer buffer is moved to the end of the struct
// and packed to allow for DUMA to catch memory overflows
typedef struct PACKED FrameBuffer {
  GSize size; //<! Active size of the framebuffer
  GRect dirty_rect; //<! Smallest rect covering all dirty pixels.
  bool is_dirty;
  uint8_t buffer[FRAMEBUFFER_SIZE_BYTES];
} FrameBuffer;
#endif

uint8_t* framebuffer_get_line(FrameBuffer* f, uint16_t y);
