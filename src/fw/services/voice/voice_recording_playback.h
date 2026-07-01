/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/voice/voice_recording.h"

//! Initialize playback state. Called once during voice service startup.
void voice_recording_playback_init(void);

//! Open, decode, and stream a stored recording to the speaker.
//! The playback service owns all resources until playback ends or is stopped.
//! @return true if playback started.
bool voice_recording_playback_start(VoiceRecordingId id);

//! Stop the current playback and release its timer, decoder, and file.
//! Safe to call when playback is already stopped or has been preempted.
void voice_recording_playback_stop(void);

//! @return true while this service owns an active playback session.
bool voice_recording_playback_is_active(void);
