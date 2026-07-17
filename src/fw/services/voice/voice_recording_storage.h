/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/voice/voice_recording.h"
#include "pbl/services/voice_endpoint.h"

#include <stdint.h>

typedef struct {
  uint16_t channels;             //!< Number of recorded audio channels
  AudioTransferInfoSpeex speex;  //!< Parameters required to decode the payload
  uint32_t created;              //!< Recording creation timestamp
  Uuid app_uuid;                 //!< Creator UUID, or UUID_INVALID for the system
  uint32_t frame_count;          //!< Number of encoded Speex frames
  uint32_t duration_ms;          //!< Recording duration
  uint32_t data_bytes;           //!< Encoded payload size, excluding the header
} VoiceRecordingStorageMetadata;

//! Remove abandoned temporary files and determine the next recording id.
void voice_recording_storage_init(VoiceRecordingId *next_id_out);

//! @return true if a stored recording file exists for \a id.
bool voice_recording_storage_id_in_use(VoiceRecordingId id);

//! @return bytes reserved for metadata at the start of each recording file.
uint32_t voice_recording_storage_header_size(void);

//! Create a preallocated temporary file and leave it open at the payload start.
//! @return an owned PFS descriptor, or a negative value on failure.
int voice_recording_storage_open_temp(VoiceRecordingId id, uint32_t payload_capacity);

//! Close an owned temporary descriptor and optionally remove its file.
void voice_recording_storage_close_temp(int fd, bool remove);

//! Remove a closed temporary file for the given recording.
void voice_recording_storage_remove_temp(VoiceRecordingId id);

//! Copy a closed temporary file into its final, exact-sized container.
//! The valid header is written last; the caller remains responsible for
//! removing the temporary file.
bool voice_recording_storage_finalize(VoiceRecordingId id,
                                      const VoiceRecordingStorageMetadata *metadata,
                                      VoiceRecordingError *error_out);

//! Open a valid recording and position it at the encoded payload.
//! @return an owned PFS descriptor, or a negative value on failure.
int voice_recording_storage_open_payload(VoiceRecordingId id, uint32_t *data_bytes_out);

//! Read the next length-prefixed encoded frame from an open payload descriptor and
//! decrement \a remaining_bytes by the bytes consumed.
//! @return the frame length in bytes, or 0 at end of payload or on a corrupt/truncated frame.
int voice_recording_storage_read_frame(int fd, uint32_t *remaining_bytes, uint8_t *frame_out,
                                       size_t frame_out_size);

//! Read the stored metadata (header) of a valid recording without opening the payload.
//! @return true on success.
bool voice_recording_storage_get_metadata(VoiceRecordingId id,
                                          VoiceRecordingStorageMetadata *out);

//! Fill an array with metadata from valid stored recordings.
//! @return number of entries written to @p out.
uint32_t voice_recording_storage_list(VoiceRecordingInfo *out, uint32_t max);

//! Fill an array with per-row summaries of valid stored recordings, in a single storage pass.
//! Listing UIs use this instead of voice_recording_storage_list() so they need not hold a full
//! VoiceRecordingInfo (which is dominated by the 16-byte creator UUID) for every recording.
//! @param has_more if not NULL, set to true when more recordings exist than fit in \a out.
//! @return number of entries written to \a out.
uint32_t voice_recording_storage_list_summaries(VoiceRecordingSummary *out, uint32_t max,
                                                bool *has_more);

//! Fill a page with metadata from valid stored recordings.
uint32_t voice_recording_storage_list_page(VoiceRecordingInfo *out, uint32_t max,
                                           uint32_t offset, bool *has_more);

//! Fill an array with metadata from recordings belonging to \a app_uuid.
uint32_t voice_recording_storage_list_owned_by(VoiceRecordingInfo *out, uint32_t max,
                                               const Uuid *app_uuid);

//! @return total bytes occupied by valid stored recordings.
uint32_t voice_recording_storage_total_bytes(void);

//! Remove a closed stored recording.
bool voice_recording_storage_delete(VoiceRecordingId id);

//! Remove every closed stored recording, except \a skip_id (pass
//! VOICE_RECORDING_ID_INVALID to remove them all).
void voice_recording_storage_delete_all(VoiceRecordingId skip_id);

//! Remove every stored recording belonging to \a app_uuid, except \a skip_id.
void voice_recording_storage_delete_owned_by(const Uuid *app_uuid, VoiceRecordingId skip_id);
