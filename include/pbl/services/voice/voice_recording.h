/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "kernel/pebble_tasks.h"
#include <pbl/util/uuid.h>

//! @file voice/voice_recording.h
//! On-device voice memo recording.
//!
//! Captures microphone audio, encodes it with the same Speex codec used by the
//! live dictation path (see voice_speex.h), and stores the encoded frames in a
//! PFS file. This works without a phone connection: speech-to-text can be
//! deferred and run later by replaying the stored frames to the phone.
//!
//! Recording and live dictation are mutually exclusive: they share the single
//! microphone device and the global Speex encoder.

typedef uint16_t VoiceRecordingId;

#define VOICE_RECORDING_ID_INVALID ((VoiceRecordingId)0)

//! Reason the last voice_recording_start / stop failed (for diagnostics/UI).
typedef enum {
  VoiceRecordingError_None = 0,
  VoiceRecordingError_Busy,         //!< a recording or playback is already active
  VoiceRecordingError_MicBusy,      //!< the microphone is in use (dictation)
  VoiceRecordingError_StorageFull,  //!< recording storage budget exhausted
  VoiceRecordingError_Codec,        //!< Speex encoder init failed
  VoiceRecordingError_NoSpace,      //!< not enough free flash for the file
  VoiceRecordingError_FileOpen,     //!< could not create the recording file
  VoiceRecordingError_Write,        //!< writing to flash failed
  VoiceRecordingError_MicStart,     //!< the microphone failed to start
} VoiceRecordingError;

//! @return the reason the most recent start/stop failed (cleared on success).
VoiceRecordingError voice_recording_last_error(void);

//! Return the recording quota usage in bytes.
//! Either output pointer may be NULL. Available space is the unused portion of
//! the recording quota, not the total free space in PFS.
void voice_recording_get_storage_usage(uint32_t *used_bytes_out,
                                       uint32_t *available_bytes_out);

//! Metadata describing a stored recording.
typedef struct {
  VoiceRecordingId id;
  uint32_t size_bytes;   //!< Size of the stored file on flash
  uint32_t duration_ms;  //!< Captured duration
  time_t created;        //!< Wall-clock time the recording started
  Uuid app_uuid;         //!< UUID of the creating app, or UUID_INVALID for system
} VoiceRecordingInfo;

//! Compact per-row data for listing UIs, which only need to label and select recordings.
//! Avoids holding a full VoiceRecordingInfo (dominated by the 16-byte creator UUID) per row.
typedef struct {
  VoiceRecordingId id;
  uint32_t duration_ms;  //!< Captured duration
  time_t created;        //!< Wall-clock time the recording started
} VoiceRecordingSummary;

//! Initialize the recording service. Cleans up any temporary files left by an
//! interrupted recording and primes the recording id allocator.
void voice_recording_init(void);

//! Start capturing a new recording from the microphone into a flash file.
//! @return the new recording id, or VOICE_RECORDING_ID_INVALID on failure
//! (microphone busy, storage budget exhausted, or no microphone support).
VoiceRecordingId voice_recording_start(void);

//! Stop an in-progress recording and finalize the stored file.
//! @return true if the recording was finalized successfully.
bool voice_recording_stop(VoiceRecordingId id);

//! Stop whichever recording is currently active and finalize its file. Used when
//! the caller cannot supply the originating id (e.g. a UI closing mid-recording).
void voice_recording_stop_active(void);

//! Abort an in-progress recording and discard its data.
void voice_recording_cancel(VoiceRecordingId id);

//! Cancel a recording owned by a process that is being terminated, and stop a playback that
//! process started.
void voice_recording_cleanup_task(PebbleTask task);

//! @return true if the active playback was started by the (non-system) app with this UUID.
bool voice_recording_playback_owned_by(const Uuid *app_uuid);

//! @return true if a recording is currently capturing.
bool voice_recording_in_progress(void);

//! @return true if the active or stored recording belongs to \a app_uuid.
bool voice_recording_is_owned_by(VoiceRecordingId id, const Uuid *app_uuid);

//! Reserve a stored recording against deletion while it is transcribed.
bool voice_recording_transcription_reserve(VoiceRecordingId id);

//! Release a transcription reservation.
void voice_recording_transcription_release(VoiceRecordingId id);

//! Enumerate stored recordings.
//! @param out  caller-provided array to fill
//! @param max  capacity of \a out
//! @return number of recordings written to \a out
uint32_t voice_recording_list(VoiceRecordingInfo *out, uint32_t max);

//! Enumerate per-row summaries of stored recordings, newest first.
//! @param out       caller-provided array to fill
//! @param max       capacity of \a out
//! @param has_more  if not NULL, set when more recordings exist than fit in \a out
//! @return number of summaries written to \a out
uint32_t voice_recording_list_summaries(VoiceRecordingSummary *out, uint32_t max, bool *has_more);

//! Enumerate a page of stored recordings.
//! @param out       caller-provided array to fill
//! @param max       capacity of \a out
//! @param offset    number of valid recordings to skip
//! @param has_more  set when at least one more recording follows this page
uint32_t voice_recording_list_page(VoiceRecordingInfo *out, uint32_t max, uint32_t offset,
                                   bool *has_more);

//! Enumerate stored recordings belonging to \a app_uuid.
uint32_t voice_recording_list_owned_by(VoiceRecordingInfo *out, uint32_t max, const Uuid *app_uuid);

//! Delete a stored recording.
//! @return true on success.
bool voice_recording_delete(VoiceRecordingId id);

//! Delete every stored recording created by \a app_uuid. Called when the app is
//! uninstalled so orphaned recordings do not consume the storage budget forever.
void voice_recording_delete_owned_by(const Uuid *app_uuid);

//! Decode and play a stored recording through the speaker. Returns false if the
//! recording could not be opened, the speaker is busy, or a capture is active.
bool voice_recording_play(VoiceRecordingId id);

//! Stop playback started by \ref voice_recording_play.
void voice_recording_stop_playback(void);

//! @return true if a recording is currently being played back.
bool voice_recording_is_playing(void);
