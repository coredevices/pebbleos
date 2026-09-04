/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/protobuf_log/protobuf_log_test.h"

#include "pbl/services/protobuf_log/protobuf_log.h"
#include "pbl/services/protobuf_log/protobuf_log_private.h"
#include "pbl/services/protobuf_log/protobuf_log_activity_sessions.h"

#include "pb_decode.h"

#include "nanopb/payload.pb.h"
#include "nanopb/measurements.pb.h"
#include "system/passert.h"

#include <pbl/util/uuid.h>

// -----------------------------------------------------------------------------------------
// Callback used to decode uuid
static bool prv_decode_uuid(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  Uuid *ret_uuid = *(Uuid **)arg;
  size_t uuid_len = stream->bytes_left;
  PBL_ASSERTN(uuid_len == sizeof(Uuid));

  return pb_read(stream, (uint8_t *)ret_uuid, uuid_len);
}

// -----------------------------------------------------------------------------------------
// Callback used to decode types
typedef struct PLogTypesDecoderArg {
  uint32_t max_num_types;
  uint32_t *num_types;
  ProtobufLogMeasurementType *types;
} PLogTypesDecoderArg;

static bool prv_decode_types(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  PLogTypesDecoderArg *decoder_info = *(PLogTypesDecoderArg **)arg;

  uint64_t value;
  bool success = pb_decode_varint(stream, &value);
  if (*decoder_info->num_types < decoder_info->max_num_types) {
    decoder_info->types[(*decoder_info->num_types)++] = value;
  }
  return success;
}


typedef struct PLogMeasurementsDecoderArg {
  uint32_t max_num_samples;
  uint32_t *num_samples;
  uint32_t *offset_sec;
  uint32_t max_num_values;
  uint32_t *num_values;
  uint32_t *values;
} PLogMeasurementsDecoderArg;

// -----------------------------------------------------------------------------------------
// Callback used to decode the packed data in a measurement
static bool prv_decode_packed_measurement_data(pb_istream_t *stream, const pb_field_t *field,
                                               void **arg) {
  PLogMeasurementsDecoderArg *decoder_info = *(PLogMeasurementsDecoderArg **)arg;

  while (stream->bytes_left) {
    uint64_t value;
    if (!pb_decode_varint(stream, &value)) {
      return false;
    }
    if (*decoder_info->num_values < decoder_info->max_num_values) {
      decoder_info->values[(*decoder_info->num_values)++] = value;
    }
  }
  return true;
}


// -----------------------------------------------------------------------------------------
// Callback used to decode measurements. Called once for each measurement
static bool prv_decode_measurements(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  PLogMeasurementsDecoderArg *decoder_info = *(PLogMeasurementsDecoderArg **)arg;

  pebble_pipeline_Measurement msg = {
    .data = {
      .funcs.decode = prv_decode_packed_measurement_data,
      .arg = decoder_info,
    }
  };

  if (!pb_decode(stream, pebble_pipeline_Measurement_fields, &msg)) {
    return false;
  }
  if (*decoder_info->num_samples < decoder_info->max_num_samples) {
    decoder_info->offset_sec[(*decoder_info->num_samples)++] = msg.offset_sec;
  }
  return true;
}

typedef struct PLogMeasurementSetDecoderArg {
  Uuid *uuid;
  PLogTypesDecoderArg *types_decoder_arg;
  PLogMeasurementsDecoderArg *measurements_decoder_arg;
  uint32_t *time_utc;
  uint32_t *time_end_utc;
  int32_t *utc_to_local;
} PLogMeasurementSetDecoderArg;


// -----------------------------------------------------------------------------------------
// Callback used to decode a MeasurementSet. Called once for each MeasurementSet
static bool prv_decode_measurement_set(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  PLogMeasurementSetDecoderArg *decoder_info = *(PLogMeasurementSetDecoderArg **)arg;

  pebble_pipeline_MeasurementSet mset = {
    .uuid = {
      .funcs.decode = prv_decode_uuid,
      .arg = decoder_info->uuid,
    },
    .types = {
      .funcs.decode = prv_decode_types,
      .arg = decoder_info->types_decoder_arg,
    },
    .measurements = {
      .funcs.decode = prv_decode_measurements,
      .arg = decoder_info->measurements_decoder_arg,
    },
  };

  bool success = pb_decode(stream, pebble_pipeline_MeasurementSet_fields, &mset);
  *decoder_info->time_utc = mset.time_utc;
  *decoder_info->time_end_utc = mset.time_end_utc;
  *decoder_info->utc_to_local = mset.utc_to_local;
  return success;
}


typedef struct PLogEventsDecoderArg {
  uint32_t max_num_events;
  uint32_t *num_events;
  pebble_pipeline_Event *events;
  Uuid *event_uuids;
  uint32_t max_num_sessions;
  uint32_t *num_sessions;
  ActivitySession *sessions;
} PLogEventsDecoderArg;



// -----------------------------------------------------------------------------------------
// Callback used to decode a pebble_pipeline_Event. Called once for each Event
static bool prv_decode_events(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  PLogEventsDecoderArg *decoder_info = *(PLogEventsDecoderArg **)arg;

  const uint32_t event_idx = *decoder_info->num_events;

  bool success;
  pebble_pipeline_Event event = {
    .uuid = {
      .funcs.decode = prv_decode_uuid,
      .arg = &decoder_info->event_uuids[event_idx],
    },
  };

  success = pb_decode(stream, pebble_pipeline_Event_fields, &event);
  if (event.type == pebble_pipeline_Event_Type_ActivitySessionEvent) {
    protobuf_log_activity_sessions_decode(&event, &decoder_info->sessions[event_idx]);
    (*decoder_info->num_sessions)++;
  }

  if (success) {
    decoder_info->events[event_idx] = event;
    (*decoder_info->num_events)++;
  }
  return true;
}




// -----------------------------------------------------------------------------------------
// Callback used to decode the payload sender type
static bool prv_decode_sender_type(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  char *ret_sender_type = *(char **)arg;
  size_t str_len = stream->bytes_left;
  PBL_ASSERTN(str_len <= PLOG_MAX_SENDER_TYPE_LEN);

  return pb_read(stream, (uint8_t *)ret_sender_type, str_len);
}

// -----------------------------------------------------------------------------------------
// Callback used to decode the payload sender id
static bool prv_decode_sender_id(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  char *ret_sender_id = *(char **)arg;
  size_t str_len = stream->bytes_left;
  PBL_ASSERTN(str_len <= PLOG_MAX_SENDER_ID_LEN);

  return pb_read(stream, (uint8_t *)ret_sender_id, str_len);
}

// -----------------------------------------------------------------------------------------
// Callback used to decode the payload sender version patch
static bool prv_decode_sender_version_patch(pb_istream_t *stream, const pb_field_t *field,
                                            void **arg) {
  char *ret_sender_version_patch = *(char **)arg;
  size_t str_len = stream->bytes_left;
  PBL_ASSERTN(str_len <= PLOG_MAX_SENDER_VERSION_PATCH_LEN);

  return pb_read(stream, (uint8_t *)ret_sender_version_patch, str_len);
}

// ---------------------------------------------------------------------------------------------
// Decode an encoded message. Used for debugging and unit tests.
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
                                 uint32_t *values) {
  pb_istream_t stream = pb_istream_from_buffer(encoded_buf, encoded_buf_size);

  const uint32_t max_num_types = *num_types;
  const uint32_t max_num_values = *num_values;
  const uint32_t max_num_samples = *num_samples;

  *num_values = 0;
  *num_samples = 0;
  *num_types = 0;

  PLogTypesDecoderArg types_decoder_arg = {
    .max_num_types = max_num_types,
    .num_types = num_types,
    .types = types,
  };

  PLogMeasurementsDecoderArg measurements_decoder_arg = {
    .max_num_samples = max_num_samples,
    .num_samples = num_samples,
    .offset_sec = offset_sec,
    .max_num_values = max_num_values,
    .num_values = num_values,
    .values = values,
  };

  PLogMeasurementSetDecoderArg mset_decoder_arg = {
    .uuid = uuid,
    .types_decoder_arg = &types_decoder_arg,
    .measurements_decoder_arg = &measurements_decoder_arg,
    .time_utc = time_utc,
    .time_end_utc = time_end_utc,
    .utc_to_local = utc_to_local,
  };

  pebble_pipeline_Payload payload = {
    .sender = {
      .type = {
        .funcs.decode = prv_decode_sender_type,
        .arg = payload_sender_type,
      },
      .id = {
        .funcs.decode = prv_decode_sender_id,
        .arg = payload_sender_id,
      },
      .version = {
        .patch = {
          .funcs.decode = prv_decode_sender_version_patch,
          .arg = payload_sender_version_patch,
        },
      },
    },
    .measurement_sets = {
      .funcs.decode = prv_decode_measurement_set,
      .arg = &mset_decoder_arg,
    }
  };

  bool success = pb_decode(&stream, pebble_pipeline_Payload_fields, &payload);
  *payload_send_time = payload.send_time_utc;
  if (payload.sender.has_version && payload_sender_v_major && payload_sender_v_minor) {
    *payload_sender_v_major = payload.sender.version.major;
    *payload_sender_v_minor = payload.sender.version.minor;
  }
  *type = ProtobufLogType_Measurements;
  return success;
}

// ---------------------------------------------------------------------------------------------
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
                                        ActivitySession *sessions) {
  pb_istream_t stream = pb_istream_from_buffer(encoded_buf, encoded_buf_size);

  const uint32_t max_num_events = *num_events;
  const uint32_t max_num_sessions = *num_sessions;

  *num_events = 0;
  *num_sessions = 0;

  PLogEventsDecoderArg event_arg = {
    .max_num_events = max_num_events,
    .num_events = num_events,
    .events = events,
    .event_uuids = event_uuids,
    .max_num_sessions = max_num_sessions,
    .num_sessions = num_sessions,
    .sessions = sessions,
  };

  pebble_pipeline_Payload payload = {
    .sender = {
      .type = {
        .funcs.decode = prv_decode_sender_type,
        .arg = payload_sender_type,
      },
      .id = {
        .funcs.decode = prv_decode_sender_id,
        .arg = payload_sender_id,
      },
      .version = {
        .patch = {
          .funcs.decode = prv_decode_sender_version_patch,
          .arg = payload_sender_version_patch,
        },
      },
    },
    .events = {
      .funcs.decode = prv_decode_events,
      .arg = &event_arg
    }
  };

  bool success = pb_decode(&stream, pebble_pipeline_Payload_fields, &payload);
  *payload_send_time = payload.send_time_utc;
  if (payload.sender.has_version && payload_sender_v_major && payload_sender_v_minor) {
    *payload_sender_v_major = payload.sender.version.major;
    *payload_sender_v_minor = payload.sender.version.minor;
  }
  *type = ProtobufLogType_Events;
  return success;
}
