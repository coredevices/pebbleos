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

//! Read the stored metadata (header) of a valid recording without opening the payload.
//! @return true on success.
bool voice_recording_storage_get_metadata(VoiceRecordingId id,
                                          VoiceRecordingStorageMetadata *out);

//! Fill an array with metadata from valid stored recordings.
//! @return number of entries written to @p out.
uint32_t voice_recording_storage_list(VoiceRecordingInfo *out, uint32_t max);

//! @return total bytes occupied by valid stored recordings.
uint32_t voice_recording_storage_total_bytes(void);

//! Remove a closed stored recording.
bool voice_recording_storage_delete(VoiceRecordingId id);

//! Remove every closed stored recording, except \a skip_id (pass
//! VOICE_RECORDING_ID_INVALID to remove them all).
void voice_recording_storage_delete_all(VoiceRecordingId skip_id);
