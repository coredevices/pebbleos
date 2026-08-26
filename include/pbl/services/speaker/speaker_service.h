/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/speaker/limits.h"
#include "pbl/services/speaker/note_sequence.h"
#include "pbl/services/speaker/speaker_finish_reason.h"
#include "pbl/services/speaker/speaker_pcm_format.h"
#include "pbl/services/speaker/track.h"
#include "kernel/pebble_tasks.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  SpeakerPriorityApp = 0,
  SpeakerPriorityNotification,
  SpeakerPriorityCritical
} SpeakerPriority;

typedef enum {
  SpeakerStateIdle = 0,
  SpeakerStatePlaying,
  SpeakerStateDraining,    // source exhausted, playing remaining queued data
} SpeakerState;

typedef enum {
  SpeakerSourceNone = 0,
  SpeakerSourceNoteSeq,
  SpeakerSourceStream,
  SpeakerSourceTracks,
  SpeakerSourceTone,
} SpeakerSourceType;

typedef uint32_t SpeakerStreamId;

#define SPEAKER_STREAM_ID_INVALID ((SpeakerStreamId)0)

//! Initialize the speaker service. Called once at boot.
void speaker_service_init(void);

//! Play a note sequence on the speaker.
//! @param notes Array of notes (copied internally)
//! @param num_notes Number of notes
//! @param pri Priority level
//! @param vol Volume (0-100)
//! @return true if playback started, false if preempted by higher priority
bool speaker_service_play_note_seq(const SpeakerNote *notes, uint32_t num_notes,
                                   SpeakerPriority pri, uint8_t vol);

//! Play a single tone at an exact frequency on the speaker.
//! @param freq_hz Tone frequency in Hz (0 = silence/rest)
//! @param duration_ms Tone duration in milliseconds (max 10000)
//! @param waveform Waveform to use (SpeakerWaveform value)
//! @param velocity Per-note amplitude scale 0-127 (0 = use master volume)
//! @param pri Priority level
//! @param vol Volume (0-100)
//! @return true if playback started, false if preempted by higher priority
bool speaker_service_play_tone(uint16_t freq_hz, uint16_t duration_ms,
                               uint8_t waveform, uint8_t velocity,
                               SpeakerPriority pri, uint8_t vol);

//! Play a short fixed tone at an absolute output volume, bypassing the user's
//! speaker-volume preference, so settings UIs can preview a candidate volume
//! before it is saved. Mute still applies.
//! @param vol Absolute output volume (0-100)
//! @return true if playback started
bool speaker_service_play_volume_preview(uint8_t vol);

//! Play N monophonic tracks in parallel, mixed together.
//! Track arrays and any sample data are copied into kernel memory.
//! @param tracks Array of tracks. For each, its notes array and (optional)
//!               sample + sample data are copied.
//! @param num_tracks Number of tracks (<= SPEAKER_MAX_TRACKS).
//! @param pri Priority level.
//! @param vol Volume (0-100).
//! @return true if playback started; false on invalid args, exceeded limits,
//!         allocation failure, or priority blocked.
bool speaker_service_play_tracks(const SpeakerTrack *tracks, uint32_t num_tracks,
                                 SpeakerPriority pri, uint8_t vol);

//! Ask the service to post PEBBLE_SPEAKER_EVENT with the finish reason to the
//! given task whenever playback ends.
void speaker_service_register_finish(PebbleTask task);

//! Open a PCM stream for writing.
//! @param pri Priority level
//! @param vol Volume (0-100)
//! @param fmt PCM format (sample rate and bit depth)
//! @return true if stream opened, false if blocked by higher priority
bool speaker_service_stream_open(SpeakerPriority pri, uint8_t vol, SpeakerPcmFormat fmt);

//! Open a PCM stream and return a token for ownership-safe stream operations.
SpeakerStreamId speaker_service_stream_open_session(SpeakerPriority pri, uint8_t vol,
                                                    SpeakerPcmFormat fmt);

//! Write PCM data to the active stream.
//! @param data Source buffer
//! @param num_bytes Number of bytes to write
//! @return Number of bytes actually written (backpressure)
uint32_t speaker_service_stream_write(const void *data, uint32_t num_bytes);

//! Write only if the given stream session is still active.
//! @return true if the session is active; false if it was stopped or preempted.
bool speaker_service_stream_write_session(SpeakerStreamId id, const void *data, uint32_t num_bytes,
                                          uint32_t *written_out);

//! Close the active stream. Remaining buffered data will be drained.
void speaker_service_stream_close(void);

//! Close the stream only if the given session is still active.
void speaker_service_stream_close_session(SpeakerStreamId id);

//! Stop any active playback immediately.
void speaker_service_stop(void);

//! Stop playback only if the given stream session is still active.
void speaker_service_stream_stop_session(SpeakerStreamId id);

//! Set playback volume.
//! @param vol Volume 0-100
void speaker_service_set_volume(uint8_t vol);

//! Get current speaker state.
SpeakerState speaker_service_get_state(void);

//! Stop any playback initiated by the given task. Called on app exit.
void speaker_service_stop_for_task(PebbleTask task);

//! Set the task that owns the current playback session.
void speaker_service_set_owner_task(PebbleTask task);

//! @return true if the speaker is currently muted (always-on mute, or
//!         the DND-scoped mute while DND is active).
bool speaker_service_is_muted(void);

//! Notify the speaker service that the audio preferences (mute or system
//! volume cap) have changed. If a source is currently playing, the audio
//! output volume is re-applied immediately so the change takes effect on
//! live playback.
void speaker_service_handle_audio_prefs_changed(void);
