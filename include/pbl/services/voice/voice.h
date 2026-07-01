/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

#include "kernel/pebble_tasks.h"
#include "pbl/services/audio_endpoint.h"
#include "pbl/services/voice_endpoint.h"
#include "pbl/services/voice/transcription.h"
#include "pbl/services/voice/voice_recording.h"

#include "applib/graphics/utf8.h"

#include <sys/types.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
// Interface to UI

typedef enum {
  VoiceStatusSuccess,
  VoiceStatusTimeout,
  VoiceStatusErrorGeneric,
  VoiceStatusErrorConnectivity,
  VoiceStatusErrorDisabled,
  VoiceStatusRecognizerResponseError,
} VoiceStatus;

typedef AudioEndpointSessionId VoiceSessionId;

#define VOICE_SESSION_ID_INVALID AUDIO_ENDPOINT_SESSION_INVALID_ID

//! Start a dictation session. The voice service will set up a session with the phone and start
//! streaming audio to the phone when it is ready. A PebbleVoiceEvent will be created and sent to
//! the subscriber when one of the following events occur:
//! - the phone replies that it is ready (audio streaming starts immediately)
//!   - VoiceEventType -> VoiceEventTypeDictationReady
//!   - VoiceStatus -> VoiceStatusSuccess
//! - the phone replies that it cannot start a dictation session
//!   - VoiceEventType -> VoiceEventTypeDictationReady
//!   - VoiceStatus -> VoiceStatusError
//! - the phone does not reply to the session setup request after a timeout
//!   - VoiceEventType -> VoiceEventTypeDictationReady
//!   - VoiceStatus -> VoiceStatusTimeout
//! - the phone stops the dictation itself (after a ready event was received) and returns an error
//!   - VoiceEventType -> VoiceEventTypeDictationResult
//!   - VoiceStatus -> VoiceStatusError
//! - the phone stops the dictation itself (after a ready event was received) and then fails to
//! return a dictation result
//!   - VoiceEventType -> VoiceEventTypeDictationResult
//!   - VoiceStatus -> VoiceStatusTimeout
//! - the phone stops the dictation itself (after a ready event was received) and returns a
//! dictation result
//!   - VoiceEventType -> VoiceEventTypeDictationResult
//!   - VoiceStatus -> VoiceStatusSuccess
//!   - sentence is valid
//! @param session_type Type of session (dictation, command, NLP, etc)
//! @return session ID if the service is in the correct state to start a session,
//! \ref VOICE_SESSION_ID_INVALID otherwise
VoiceSessionId voice_start_dictation(VoiceEndpointSessionType session_type);

//! Start a transcription session sourced from a stored recording instead of the microphone. The
//! stored Speex frames are streamed to the phone; results arrive through the same path as
//! dictation. Only mono recordings can be transcribed.
//! @param recording_id  id of a stored recording (see voice_recording.h)
//! @return session ID if the session was started, \ref VOICE_SESSION_ID_INVALID otherwise
//! (busy, recording not found, or non-mono recording).
VoiceSessionId voice_start_dictation_from_recording(VoiceRecordingId recording_id);

//! @return the id of the recording whose file is held open by an active transcription stream, or
//! \ref VOICE_RECORDING_ID_INVALID if none. Removing an open PFS file panics, so deletion paths
//! must skip this recording.
VoiceRecordingId voice_transcribing_recording_id(void);

//! Call after a ready event has been received to end the audio streaming session and await the
//! dictation response. A VoiceEvent will be created and sent to the subscriber when one of the
//! following events occur:
//! - the phone responds with the transcription of the dictation session
//!   - VoiceEventType -> VoiceEventTypeDictationResult
//!   - VoiceStatus -> VoiceStatusSuccess
//!   - sentence is valid
//! - the phone reports an error with the transcription process
//!   - VoiceEventType -> VoiceEventTypeDictationResult
//!   - VoiceStatus -> VoiceStatusError
//! - the phone does not transmit the transcription after a timeout
//!   - VoiceEventType -> VoiceEventTypeDictationResult
//!   - VoiceStatus -> VoiceStatusTimeout
void voice_stop_dictation(VoiceSessionId session_id);

//! Cancel a dictation session. Can be called at any stage of the session
void voice_cancel_dictation(VoiceSessionId session_id);

//! Initialize the voice service
void voice_init(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Interface to voice control endpoint

//! Handle a session setup result received by voice control endpoint
//! @param result         Result code for session setup
//! @param session_type   Type of session (dictation, command, NLP, etc)
//! @param app_initiated  True if session was initiated by an app, false otherwise
void voice_handle_session_setup_result(VoiceEndpointResult result,
                                       VoiceEndpointSessionType session_type, bool app_initiated);

//! Handle a dictation session result received by voice control endpoint. The contents of the
//! transcription object will be copied
//! @param result         Result code for dictation session
//! @param session_id     Audio transfer session ID from which the transcription was derived
//! @param transcription  Transcription object - must be validated (using transcription_validate)
//! @param app_initiated  True if session was initiated by an app, false otherwise
//! @param app_uuid       Pointer to app UUID received. Freed after this call returns
void voice_handle_dictation_result(VoiceEndpointResult result, AudioEndpointSessionId session_id,
                                   Transcription *transcription, bool app_initiated,
                                   Uuid *app_uuid);

void voice_handle_nlp_result(VoiceEndpointResult result, AudioEndpointSessionId session_id,
                             char *reminder, time_t timestamp);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Syscalls

VoiceSessionId sys_voice_start_dictation(VoiceEndpointSessionType session_type);
VoiceSessionId sys_voice_start_dictation_from_recording(VoiceRecordingId recording_id);
void sys_voice_stop_dictation(VoiceSessionId session_id);
void sys_voice_cancel_dictation(VoiceSessionId session_id);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Process cleanup

void voice_kill_app_session(PebbleTask task);
