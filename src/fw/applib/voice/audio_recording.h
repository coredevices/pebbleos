/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/voice/dictation_session.h"

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

//! Play a stored recording back through the watch speaker.
//! @param recording_id  id of a stored recording (e.g. from \ref audio_recording_stop)
//! @return true if playback started, false if the recording does not exist, a
//! recording or another playback is in progress, or the platform has no speaker.
bool audio_recording_play(AudioRecordingId recording_id);

//! Stop playback started by \ref audio_recording_play.
void audio_recording_stop_playback(void);

//! @return true if a recording is currently being played back.
bool audio_recording_is_playing(void);

//! Callback delivering the result of \ref audio_recording_transcribe.
//! @param recording_id  the recording that was transcribed
//! @param status        \ref DictationSessionStatusSuccess or a failure reason
//! @param transcription the transcribed text on success; freed after this call returns, so copy it
//!                      if it must be retained. NULL on failure.
//! @param context       the context passed to \ref audio_recording_transcribe
typedef void (*AudioTranscriptionCallback)(AudioRecordingId recording_id,
                                           DictationSessionStatus status, char *transcription,
                                           void *context);

//! Send a stored recording to the phone for transcription (speech-to-text). A progress screen is
//! shown while the audio is uploaded and transcribed, then the result is delivered to \a callback.
//! Requires a phone connection with voice support. Only mono recordings can be transcribed.
//! @param recording_id  id of a stored recording (e.g. from \ref audio_recording_stop)
//! @param buffer_size   size of the buffer to hold the transcription, or 0 to have one allocated
//!                      automatically when the result arrives
//! @param callback      invoked with the transcription result; must not be NULL
//! @param context       passed back to \a callback
//! @return true if the transcription was started, false if it could not be started (no callback,
//! recording missing or not mono, no voice-capable phone connection, or out of memory).
bool audio_recording_transcribe(AudioRecordingId recording_id, uint32_t buffer_size,
                                AudioTranscriptionCallback callback, void *context);

//!   @} // end addtogroup AudioRecording
//! @} // end addtogroup Microphone
