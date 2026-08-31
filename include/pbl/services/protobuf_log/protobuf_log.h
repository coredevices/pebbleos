/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! This module handles the collection and sending of periodic protobuf payloads to the phone
//! using the protobuf schema defined at src/fw/idl/nanopb/*.proto and sent to the phone via
//! data logging.

#include "pbl/services/hrm/hrm_manager.h"
#include "pbl/services/data_logging/dls_private.h"
#include "system/version.h"

#include <stdbool.h>
#include <stdint.h>

// Auto generated header produced by compiling the .proto files in src/fw/idl
#include "nanopb/measurements.pb.h"
#include "nanopb/event.pb.h"

// Create an alias typedef for the auto-generated name
typedef pebble_pipeline_MeasurementSet_Type ProtobufLogMeasurementType;
typedef pebble_pipeline_ActivityType_InternalType ProtobufLogActivityType;

#define ProtobufLogMeasurementType_TimeMS           pebble_pipeline_MeasurementSet_Type_TimeMS
#define ProtobufLogMeasurementType_VMC              pebble_pipeline_MeasurementSet_Type_VMC
#define ProtobufLogMeasurementType_Steps            pebble_pipeline_MeasurementSet_Type_Steps
#define ProtobufLogMeasurementType_DistanceCM       pebble_pipeline_MeasurementSet_Type_DistanceCM
#define ProtobufLogMeasurementType_RestingGCalories pebble_pipeline_MeasurementSet_Type_RestingGCalories
#define ProtobufLogMeasurementType_ActiveGCalories  pebble_pipeline_MeasurementSet_Type_ActiveGCalories
#define ProtobufLogMeasurementType_BPM              pebble_pipeline_MeasurementSet_Type_BPM
#define ProtobufLogMeasurementType_RR               pebble_pipeline_MeasurementSet_Type_RR
#define ProtobufLogMeasurementType_Orientation      pebble_pipeline_MeasurementSet_Type_Orientation
#define ProtobufLogMeasurementType_Light            pebble_pipeline_MeasurementSet_Type_Light
#define ProtobufLogMeasurementType_Temperature      pebble_pipeline_MeasurementSet_Type_Temperature
#define ProtobufLogMeasurementType_HRQuality        pebble_pipeline_MeasurementSet_Type_HRQuality

#define ProtobufLogActivityType_UnknownType pebble_pipeline_ActivityType_InternalType_UnknownType
#define ProtobufLogActivityType_Sleep       pebble_pipeline_ActivityType_InternalType_Sleep
#define ProtobufLogActivityType_DeepSleep   pebble_pipeline_ActivityType_InternalType_DeepSleep
#define ProtobufLogActivityType_Nap         pebble_pipeline_ActivityType_InternalType_Nap
#define ProtobufLogActivityType_DeepNap     pebble_pipeline_ActivityType_InternalType_DeepNap
#define ProtobufLogActivityType_Walk        pebble_pipeline_ActivityType_InternalType_Walk
#define ProtobufLogActivityType_Run         pebble_pipeline_ActivityType_InternalType_Run
#define ProtobufLogActivityType_Open        pebble_pipeline_ActivityType_InternalType_Open

#define PLOG_MAX_SENDER_ID_LEN  64
#define PLOG_MAX_SENDER_TYPE_LEN  64
#define PLOG_MAX_SENDER_VERSION_PATCH_LEN  FW_METADATA_VERSION_TAG_BYTES
#define PLOG_PAYLOAD_SENDER_TYPE  "watch"

// Size of the data logging records we use
#define PLOG_DLS_RECORD_SIZE DLS_SESSION_MAX_BUFFERED_ITEM_SIZE

// Currently supported Payload types
//
// MEASUREMENTS
// - Used today for logging HR bpm and quality for each sample
// - Can also be used for logging minute level data with steps, lights, orientation, etc.
// - How to use:
//   - Create a `ProtobufLogConfig` struct with type `ProtobufLogType_Measurements`, the number of
//     types each sample will contain and the array of types.
//   - Call `protobuf_log_create` with the config.
//   - Call `protobuf_log_session_add_measurements` repeatedly with new samples, each containing
//     the same number of measurements, the number that as set in the `ProtobufLogConfig`.

// EVENTS
// - Used today for logging ActivitySession events
// - How to use:
//   - Create a `ProtobufLogConfig` struct with type `ProtobufLogType_Events`.
//   - Call `protobuf_log_create` with the config.
//   - Call `protobuf_log_add_event` repeatedly with new pebble_pipeline_Event's.

typedef enum ProtobufLogType {
  ProtobufLogType_Measurements,
  ProtobufLogType_Events,
} ProtobufLogType;

typedef struct ProtobufLogConfig {
  ProtobufLogType type;
  union {
    struct {
      uint8_t num_types;        // number of readings in each measurement
      ProtobufLogMeasurementType *types; // Array of measurement types.
    } measurements;
    struct {
      // empty for now
    } events;
  };
} ProtobufLogConfig;

// Handle returned when a new protobuf log session is created
typedef void *ProtobufLogRef;

// Signature of the transport callback that can be optionally provided to protobuf_log_create()
typedef bool (*ProtobufLogTransportCB)(uint8_t *buffer, size_t buf_size);


// Init the service
// @return true if successful
bool protobuf_log_init(void);

// Create a new protobuf log session.
// @param[in] config `ProtobufLogConfig` for the type of protobuf log session to create
// @param[in] transport optional callback that will be used to send the encoded data out. If
//   NULL, the data will be sent over data logging by default
// @param[in] max_msg_size optional max message size. This should almost always be 0 so that
//   a default size is used that fills the data logging record as fully as possible. Non-zero
//   values are mostly used for unit tests.
// @return new session pointer, or NULL if error occurred
ProtobufLogRef protobuf_log_create(ProtobufLogConfig *config,
                                   ProtobufLogTransportCB transport,
                                   size_t max_encoded_msg_size);

// Add a new measurement sample to the session. Once the amount of accumulated measurement data
// gets large enough, it will be automatically encoded and sent out.
// @param[in] session created by protobuf_log_create
// @param[in] sample_utc the UTC timestamp of this sample
// @param[in] num_values the number of values in the 'values' array. This must match
//   the 'num_types' value that was passed to protobuf_log_create() when the session was created
// @param[in] values array of measurement values for this sample
// @return true on success, false on failure
bool protobuf_log_session_add_measurements(ProtobufLogRef session, time_t sample_utc,
                                           uint32_t num_values, uint32_t *values);

// Add a new `pebble_pipeline_Event` to the session. Once the amount of accumulated event data
// gets large enough, it will be automatically encoded and sent out.
// @param[in] session created by protobuf_log_create
// @param[in] event the event to be added
// @return true on success, false on failure
bool protobuf_log_session_add_event(ProtobufLogRef session_ref, pebble_pipeline_Event *event);

// Immediately encode and send all payload data accumulated so far.
// @param[in] session created by protobuf_log_create
// @return true on success, false on failure
bool protobuf_log_session_flush(ProtobufLogRef session);

// Delete a session. This will first issue a protobuf_log_session_flush before deleting the
// session.
// @param[in] session created by protobuf_log_create
// @return true on success, false on failure
bool protobuf_log_session_delete(ProtobufLogRef session);
