/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "gdraw_command_frame.h"

#include "applib/graphics/gtypes.h"

#include <stdint.h>
#include <stdbool.h>

//! @file graphics/gdraw_command_sequence.h
//! Defines the functions to manipulate \ref GDrawCommandSequence objects
//! @addtogroup Graphics
//! @{
//!   @addtogroup DrawCommand Draw Commands
//!   @{

struct GDrawCommandSequence;

//! Draw command sequences allow the animation of frames over time. Each sequence has a list of
//! frames that can be accessed by the elapsed duration of the animation (not maintained internally)
//! or by index. Sequences can be loaded from PDC file data.
typedef struct GDrawCommandSequence GDrawCommandSequence;

//! Creates a \ref GDrawCommandSequence from the specified resource (PDC file)
//! @param resource_id Resource containing data to load and create GDrawCommandSequence from.
//! @return GDrawCommandSequence pointer if the resource was loaded, NULL otherwise
GDrawCommandSequence *gdraw_command_sequence_create_with_resource(uint32_t resource_id);

//! @internal
GDrawCommandSequence *gdraw_command_sequence_create_with_resource_system(ResAppNum app_num,
                                                                         uint32_t resource_id);

//! Creates a \ref GDrawCommandSequence as a copy from a given sequence
//! @param sequence Sequence to copy
//! @return cloned sequence or NULL if the operation failed
GDrawCommandSequence *gdraw_command_sequence_clone(GDrawCommandSequence *sequence);

//! Deletes the \ref GDrawCommandSequence structure and frees associated data
//! @param sequence Pointer to the sequence to destroy
void gdraw_command_sequence_destroy(GDrawCommandSequence *sequence);

//! @internal
//! Use to validate a sequence read from flash or copied from serialized data
//! @param size Size of the sequence in memory, in bytes
bool gdraw_command_sequence_validate(GDrawCommandSequence *sequence, size_t size);

//! Get the frame that should be shown after the specified amount of elapsed time
//! The last frame will be returned if the elapsed time exceeds the total time
//! @param sequence \ref GDrawCommandSequence from which to get the frame
//! @param elapsed_ms elapsed time in milliseconds
//! @return pointer to \ref GDrawCommandFrame that should be displayed at the elapsed time
GDrawCommandFrame *gdraw_command_sequence_get_frame_by_elapsed(GDrawCommandSequence *sequence,
                                                               uint32_t elapsed_ms);

//! Get the frame at the specified index
//! @param sequence \ref GDrawCommandSequence from which to get the frame
//! @param index Index of frame to get
//! @return pointer to \ref GDrawCommandFrame at the specified index
GDrawCommandFrame *gdraw_command_sequence_get_frame_by_index(GDrawCommandSequence *sequence,
                                                             uint32_t index);

//! @internal
//! Get the size, in bytes, of the sequence in memory
size_t gdraw_command_sequence_get_data_size(GDrawCommandSequence *sequence);

//! Get the size of the bounding box surrounding all draw commands in the sequence. This bounding
//! box can be used to set the graphics context or layer bounds when drawing the frames in the
//! sequence.
//! @param sequence \ref GDrawCommandSequence from which to get the bounds
//! @return bounding box size
GSize gdraw_command_sequence_get_bounds_size(GDrawCommandSequence *sequence);

//! Set size of the bounding box surrounding all draw commands in the sequence. This bounding
//! box can be used to set the graphics context or layer bounds when drawing the frames in the
//! sequence.
//! @param sequence \ref GDrawCommandSequence for which to set the bounds
//! @param size bounding box size
void gdraw_command_sequence_set_bounds_size(GDrawCommandSequence *sequence, GSize size);

//! Get the play count of the sequence
//! @param sequence \ref GDrawCommandSequence from which to get the play count
//! @return play count of sequence
uint32_t gdraw_command_sequence_get_play_count(GDrawCommandSequence *sequence);

//! Set the play count of the sequence
//! @param sequence \ref GDrawCommandSequence for which to set the play count
//! @param play_count play count
void gdraw_command_sequence_set_play_count(GDrawCommandSequence *sequence, uint32_t play_count);

//! Get the total duration of the sequence.
//! @param sequence \ref GDrawCommandSequence from which to get the total duration
//! @return total duration of the sequence in milliseconds
uint32_t gdraw_command_sequence_get_total_duration(GDrawCommandSequence *sequence);

//! Get the number of frames in the sequence
//! @param sequence \ref GDrawCommandSequence from which to get the number of frames
//! @return number of frames in the sequence
uint32_t gdraw_command_sequence_get_num_frames(GDrawCommandSequence *sequence);

//!   @} // end addtogroup DrawCommand
//! @} // end addtogroup Graphics
