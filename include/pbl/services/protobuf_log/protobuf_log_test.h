/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "protobuf_log.h"
#include "pbl/services/activity/activity.h"

#include <stdint.h>

// ---------------------------------------------------------------------------------------------
// Decode an encoded payload with MeasurementSets. Used for debugging and unit tests.
// @param[in] encoded_buf pointer to encoded data
// @param[in] encoded_buf_size size of encoded data
// @param[out] payload_sender_type returned payload sender type string
// @param[out] payload_sender_id returned payload sender id string
// @param[out] payload_sender_version_patch returned payload sender version patch string
// @param[out] payload_send_time returned payload send time
// @param[out] payload_sender_v_major returned payload sender version major digit
// @param[out] payload_sender_v_minor returned payload sender version minor digit
// @param[out] uuid returned UUID field
// @param[out] time_utc returned time_utc field
// @param[out] time_end_utc returned time_end_utc field
// @param[out] utc_to_local returned utc_to_local field
// @param[in:out] num_types on entry, length of the types array; on exit, returned number
//   of measurement types in the types array
// @param[out] types returned array of measurement types
// @param[in:out] num_samples on entry, length of the offset_sec array; on exit, returned
//   number of values in the offset_sec array
// @param[out] offset_sec returned offset_sec value for each measurement
// @param[in|out] num_values on entry, number of entries available in the values array; On exit,
//   the actual number of values written to the values array
// @param[out] values returned measurement values here. This will contain
//   num_types * num_measurements values if num_values on entry was large enough
// @param[out]
// @return true on success, false on failure
bool protobuf_log_private_mset_decode(ProtobufLogType *type,
                                 void *encoded_buf,
                                 uint32_t encoded_buf_size,
                                 char payload_sender_type[PLOG_MAX_SENDER_TYPE_LEN],
                                 char payload_sender_id[PLOG_MAX_SENDER_ID_LEN],
                                 char payload_sender_version_patch[FW_METADATA_VERSION_TAG_BYTES],
                                 uint32_t *payload_send_time,
                                 uint32_t *payload_sender_v_major,
                                 uint32_t *payload_sender_v_minor,
                                 Uuid *uuid,
                                 uint32_t *time_utc,
                                 uint32_t *time_end_utc,
                                 int32_t *utc_to_local,
                                 uint32_t *num_types,
                                 ProtobufLogMeasurementType *types,
                                 uint32_t *num_samples,
                                 uint32_t *offset_sec,
                                 uint32_t *num_values,
                                 uint32_t *values);

// ---------------------------------------------------------------------------------------------
// Decode an encoded payload with events. Used for debugging and unit tests.
// @param[in] encoded_buf pointer to encoded data
// @param[in] encoded_buf_size size of encoded data
// @param[out] payload_sender_type returned payload sender type string
// @param[out] payload_sender_id returned payload sender id string
// @param[out] payload_sender_version_patch returned payload sender version patch string
// @param[out] payload_send_time returned payload send time
// @param[out] payload_sender_v_major returned payload sender version major digit
// @param[out] payload_sender_v_minor returned payload sender version minor digit
// @param[out] num_events on entry, length of events array; on exit, returned number
//   of events in the events array
// @param[out] events array of events
// @param[out] event_uuids array of uuids for events
// @param[out] num_sessions on entry, length of sessions array; on exit, returned number
//   of sessions in the sessions array
// @param[out] sessions array of ActivitySessions for events. Indexed by events.
//   e.g. If there are three events and the first two are Unknown events and the third is an
//        of type ActivitySession, then it's activity session will be at sessions[2].
// @param[out]
// @return true on success, false on failure
bool protobuf_log_private_events_decode(ProtobufLogType *type,
                                        void *encoded_buf,
                                        uint32_t encoded_buf_size,
                                        char payload_sender_type[PLOG_MAX_SENDER_TYPE_LEN],
                                        char payload_sender_id[PLOG_MAX_SENDER_ID_LEN],
                                        char payload_sender_version_patch[FW_METADATA_VERSION_TAG_BYTES],
                                        uint32_t *payload_send_time,
                                        uint32_t *payload_sender_v_major,
                                        uint32_t *payload_sender_v_minor,
                                        uint32_t *num_events,
                                        pebble_pipeline_Event *events,
                                        Uuid *event_uuids,
                                        uint32_t *num_sessions,
                                        ActivitySession *sessions);
