/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gtypes.h"

#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 8
#include "applib/graphics/8_bit/framebuffer.h"
#else
#include "applib/graphics/1_bit/framebuffer.h"
#endif

#include <stdbool.h>

extern volatile const int FrameBuffer_MaxX;
extern volatile const int FrameBuffer_MaxY;

//! Initializes the framebuffer by setting the size.
void framebuffer_init(FrameBuffer *fb, const GSize *size);

//! Get the active buffer size in bytes
size_t framebuffer_get_size_bytes(FrameBuffer *f);

//! Clears the screen buffer.
//! Will not be visible on the display until graphics_flush_frame_buffer is called.
void framebuffer_clear(FrameBuffer* f);

//! Mark the given rect of pixels as dirty
void framebuffer_mark_dirty_rect(FrameBuffer* f, GRect rect);

//! Mark the entire framebuffer as dirty
void framebuffer_dirty_all(FrameBuffer* f);

//! Clear the dirty status for this framebuffer
void framebuffer_reset_dirty(FrameBuffer* f);

//! Query the dirty status for this framebuffer
bool framebuffer_is_dirty(FrameBuffer* f);

//! Creates a GBitmap struct that points to the framebuffer. Useful for using the framebuffer data
//! with graphics routines. Note that updating this bitmap won't mark the appropriate lines as
//! dirty in the framebuffer, so this will have to be done manually.
//! @note The size which is passed in should come from app_manager_get_framebuffer_size() for the
//! app framebuffer (or generated based on DISP_ROWS / DISP_COLS for the system framebuffer) to
//! protect against malicious apps changing their own framebuffer size.
GBitmap framebuffer_get_as_bitmap(FrameBuffer *f, const GSize *size);

//! Get the framebuffer size
GSize framebuffer_get_size(FrameBuffer *f);
