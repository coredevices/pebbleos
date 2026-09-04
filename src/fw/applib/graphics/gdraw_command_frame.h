/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gdraw_command_list.h"

#include <stdint.h>
#include <stdbool.h>

//! @file graphics/gdraw_command_frame.h
//! Defines the functions to manipulate \ref GDrawCommandFrame objects
//! @addtogroup Graphics
//! @{
//!   @addtogroup DrawCommand Draw Commands
//!   @{

struct GDrawCommandFrame;
typedef struct GDrawCommandSequence GDrawCommandSequence;

//! Draw command frames contain a list of commands to draw for that frame and a duration,
//! indicating the length of time for which the frame should be drawn in an animation sequence.
//! Frames form the building blocks of a \ref GDrawCommandSequence, which consists of multiple
//! frames.
typedef struct GDrawCommandFrame GDrawCommandFrame;

//! @internal
//! Use to validate a frame read from flash or copied from serialized data
//! @param size Size of the frame structure in memory, in bytes
bool gdraw_command_frame_validate(GDrawCommandFrame *frame, size_t size);

//! Draw a frame
//! @param ctx The destination graphics context in which to draw
//! @param sequence The sequence from which the frame comes from (this is required)
//! @param frame Frame to draw
//! @param offset Offset from draw context origin to draw the frame
void gdraw_command_frame_draw(GContext *ctx, GDrawCommandSequence *sequence,
                              GDrawCommandFrame *frame, GPoint offset);

//! @internal
void gdraw_command_frame_draw_processed(GContext *ctx, GDrawCommandSequence *sequence,
                                        GDrawCommandFrame *frame, GPoint offset,
                                        GDrawCommandProcessor *processor);

//! Set the duration of the frame
//! @param frame \ref GDrawCommandFrame for which to set the duration
//! @param duration duration of the frame in milliseconds
void gdraw_command_frame_set_duration(GDrawCommandFrame *frame, uint32_t duration);

//! Get the duration of the frame
//! @param frame \ref GDrawCommandFrame from which to get the duration
//! @return duration of the frame in milliseconds
uint32_t gdraw_command_frame_get_duration(GDrawCommandFrame *frame);

//! @internal
//! Get the size, in bytes, of the frame in memory
size_t gdraw_command_frame_get_data_size(GDrawCommandFrame *frame);

//! Get the command list of the frame
//! @param frame \ref GDrawCommandFrame from which to get the command list
//! @return command list
GDrawCommandList *gdraw_command_frame_get_command_list(GDrawCommandFrame *frame);

//!   @} // end addtogroup DrawCommand
//! @} // end addtogroup Graphics
