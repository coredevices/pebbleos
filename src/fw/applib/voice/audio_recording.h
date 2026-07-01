/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

//! @file voice/audio_recording.h
//! Defines the interface for recording microphone audio to the watch.
//! @addtogroup Microphone
//! @{
//!   @addtogroup AudioRecording Audio Recording
//! \brief Record microphone audio and store it on the watch.
//!
//! Unlike a dictation session, audio recording does not require a phone
//! connection: the captured audio is encoded and stored on the watch so it can
//! be transcribed or retrieved later. Only one recording can be active at a
//! time, and recording cannot run while a dictation session is in progress
//! (both use the single microphone).
//!
//! On platforms without a microphone, \ref audio_recording_start returns
//! \ref AUDIO_RECORDING_ID_INVALID and the other calls do nothing.
//! @{

//! Identifies a recording. Valid ids are non-zero.
typedef uint16_t AudioRecordingId;

//! Returned by \ref audio_recording_start when a recording cannot be started.
#define AUDIO_RECORDING_ID_INVALID ((AudioRecordingId)0)

//! Start recording microphone audio to the watch.
//! @return the new recording's id, or \ref AUDIO_RECORDING_ID_INVALID if a
//! recording or dictation is already in progress, storage is full, or the
//! platform has no microphone.
AudioRecordingId audio_recording_start(void);

//! Stop the active recording and store it for later use.
//! @param recording_id  id returned by \ref audio_recording_start
void audio_recording_stop(AudioRecordingId recording_id);

//! Cancel the active recording and discard its audio.
//! @param recording_id  id returned by \ref audio_recording_start
void audio_recording_cancel(AudioRecordingId recording_id);

//! @return true if a recording is currently capturing audio.
bool audio_recording_is_active(void);

//!   @} // end addtogroup AudioRecording
//! @} // end addtogroup Microphone
