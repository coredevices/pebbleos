/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

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
  VoiceRecordingError_Save,         //!< finalizing the recording failed
} VoiceRecordingError;

//! @return the reason the most recent start/stop failed (cleared on success).
VoiceRecordingError voice_recording_last_error(void);

//! Metadata describing a stored recording.
typedef struct {
  VoiceRecordingId id;
  uint32_t size_bytes;   //!< Size of the stored file on flash
  uint32_t duration_ms;  //!< Captured duration
  time_t created;        //!< Wall-clock time the recording started
  Uuid app_uuid;         //!< UUID of the creating app, or UUID_INVALID for system
} VoiceRecordingInfo;

//! Initialize the recording service. Cleans up any temporary files left by an
//! interrupted recording and primes the recording id allocator.
void voice_recording_init(void);

//! Start capturing a new recording from the microphone into a flash file.
//! @return the new recording id, or VOICE_RECORDING_ID_INVALID on failure
//! (microphone busy, storage budget exhausted, or no microphone support).
VoiceRecordingId voice_recording_start(void);

//! Stop an in-progress recording and finalize the stored file.
void voice_recording_stop(VoiceRecordingId id);

//! Stop whichever recording is currently active and finalize its file. Used when
//! the caller cannot supply the originating id (e.g. a UI closing mid-recording).
void voice_recording_stop_active(void);

//! Abort an in-progress recording and discard its data.
void voice_recording_cancel(VoiceRecordingId id);

//! @return true if a recording is currently capturing.
bool voice_recording_in_progress(void);

//! Enumerate stored recordings.
//! @param out  caller-provided array to fill
//! @param max  capacity of \a out
//! @return number of recordings written to \a out
uint32_t voice_recording_list(VoiceRecordingInfo *out, uint32_t max);

//! @return total bytes occupied on flash by stored recordings.
uint32_t voice_recording_total_bytes(void);

//! Delete a stored recording.
//! @return true on success.
bool voice_recording_delete(VoiceRecordingId id);

//! Delete every stored recording.
void voice_recording_delete_all(void);

//! Decode and play a stored recording through the speaker. Returns false if the
//! recording could not be opened, the speaker is busy, or a capture is active.
bool voice_recording_play(VoiceRecordingId id);

//! Stop playback started by \ref voice_recording_play.
void voice_recording_stop_playback(void);

//! @return true if a recording is currently being played back.
bool voice_recording_is_playing(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Syscalls

VoiceRecordingId sys_voice_recording_start(void);
void sys_voice_recording_stop(VoiceRecordingId id);
void sys_voice_recording_cancel(VoiceRecordingId id);
bool sys_voice_recording_in_progress(void);
