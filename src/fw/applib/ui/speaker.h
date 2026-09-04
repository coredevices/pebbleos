/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/speaker/note_sequence.h"
#include "pbl/services/speaker/speaker_finish_reason.h"
#include "pbl/services/speaker/speaker_pcm_format.h"
#include "pbl/services/speaker/track.h"

#include <stdbool.h>
#include <stdint.h>

//! @file speaker.h
//! @addtogroup UI
//! @{
//!   @addtogroup Speaker
//! \brief Controlling the speaker
//!
//! The Speaker API provides calls that let you play sounds through the watch's speaker.
//! You can play simple note sequences (melodies), single tones, or stream raw PCM audio.
//!
//! Note sequences are compact representations of melodies using MIDI-like note definitions,
//! supporting 4 basic waveforms: sine, square, triangle, and sawtooth.
//!
//! Raw PCM streaming allows apps to generate arbitrary audio in configurable formats.
//!   @{

//! Speaker status
typedef enum {
  SpeakerStatusIdle = 0,
  SpeakerStatusPlaying,
  SpeakerStatusDraining,
} SpeakerStatus;

//! Callback invoked when playback finishes.
//! @param reason Why playback ended
//! @param ctx User context
typedef void (*SpeakerFinishedCallback)(SpeakerFinishReason reason, void *ctx);

//! Play a sequence of notes on the speaker.
//! @param notes Array of SpeakerNote structs defining the melody
//! @param num_notes Number of notes in the array (max SPEAKER_MAX_NOTES)
//! @param volume Playback volume (0-100)
//! @return true if playback started successfully
bool speaker_play_notes(const SpeakerNote *notes, uint32_t num_notes, uint8_t volume);

//! Play N monophonic tracks in parallel, mixed (polyphony).
//! @param tracks Array of track descriptors (notes + optional sample).
//! @param num_tracks Number of tracks. Must be >= 1 and <= SPEAKER_MAX_TRACKS.
//! @param volume Playback volume (0-100).
//! @return true if playback started successfully.
bool speaker_play_tracks(const SpeakerTrack *tracks, uint32_t num_tracks, uint8_t volume);

//! Play a single tone on the speaker (convenience wrapper).
//! @param frequency_hz Tone frequency in Hz
//! @param duration_ms Tone duration in milliseconds (max 10000)
//! @param volume Playback volume (0-100)
//! @param waveform Waveform to use
//! @return true if playback started successfully
bool speaker_play_tone(uint16_t frequency_hz, uint32_t duration_ms,
                       uint8_t volume, SpeakerWaveform waveform);

//! Open a raw PCM stream for app-generated audio.
//! @param format PCM format specifying sample rate and bit depth
//! @param volume Playback volume (0-100)
//! @return true if stream opened successfully
bool speaker_stream_open(SpeakerPcmFormat format, uint8_t volume);

//! Write PCM data to the open stream.
//! @param data Buffer of PCM data in the format specified at open
//! @param num_bytes Number of bytes to write
//! @return Number of bytes actually written (may be less if buffer is full)
uint32_t speaker_stream_write(const void *data, uint32_t num_bytes);

//! Close the PCM stream. Buffered data will be played before stopping.
void speaker_stream_close(void);

//! Stop any active speaker playback immediately.
void speaker_stop(void);

//! Set the speaker volume.
//! @param volume Volume level (0-100)
void speaker_set_volume(uint8_t volume);

//! Get the current speaker status.
//! @return Current SpeakerStatus
SpeakerStatus speaker_get_status(void);

//! Check whether the speaker is currently muted system-wide. The watch can
//! mute the speaker either always-on (Settings → Sounds & Haptics) or for
//! the duration of Quiet Time (Settings → Quiet Time). Apps cannot
//! override the mute, but they can use this to adapt UI or skip long
//! sounds the user won't hear.
//! @return true if the speaker is currently muted
bool speaker_is_muted(void);

//! Register a callback invoked when speaker playback ends.
//! The callback runs on the app task.
//! @param cb Callback to invoke, or NULL to unregister.
//! @param ctx User context passed back to cb.
void speaker_set_finish_callback(SpeakerFinishedCallback cb, void *ctx);

//!   @} // end addtogroup Speaker
//! @} // end addtogroup UI
