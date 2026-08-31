/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

typedef struct PLogPackedVarintsEncoderArg {
  uint16_t num_values;
  uint32_t *values;
} PLogPackedVarintsEncoderArg;

typedef struct PLogTypesEncoderArg {
  uint16_t num_types;
  ProtobufLogMeasurementType *types;
} PLogTypesEncoderArg;

typedef struct PLogBufferEncoderArg {
  uint16_t len;
  uint8_t *buffer;
} PLogBufferEncoderArg;

// -----------------------------------------------------------------------------------------
// Callback used to stuff in the uuid
bool protobuf_log_util_encode_uuid(pb_ostream_t *stream, const pb_field_t *field,
                                  void * const *arg);

// -----------------------------------------------------------------------------------------
// Callback used to stuff in a string
bool protobuf_log_util_encode_string(pb_ostream_t *stream, const pb_field_t *field,
                                    void * const *arg);

// -----------------------------------------------------------------------------------------
// Callback used to stuff in a packed array of varints
bool protobuf_log_util_encode_packed_varints(pb_ostream_t *stream, const pb_field_t *field,
                                            void * const *arg);

// -----------------------------------------------------------------------------------------
// Callback used to stuff in the array of types
bool protobuf_log_util_encode_measurement_types(pb_ostream_t *stream, const pb_field_t *field,
                                               void * const *arg);

// -----------------------------------------------------------------------------------------
// Callback used to stuff in a data buffer. Useful for MeasurementSets or Events
bool protobuf_log_util_encode_buffer(pb_ostream_t *stream, const pb_field_t *field,
                                    void * const *arg);
