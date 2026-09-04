/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/protobuf_log/protobuf_log_activity_sessions.h"
#include "pbl/services/protobuf_log/protobuf_log.h"
#include "pbl/services/protobuf_log/protobuf_log_private.h"

#include "pbl/services/activity/activity.h"

#include "nanopb/event.pb.h"
#include "system/passert.h"

#include <stdbool.h>


// -----------------------------------------------------------------------------------------
// Convert ActivitySessionType to the internal protobuf representation.
static ActivitySessionType prv_proto_type_to_activity_type(ProtobufLogActivityType type) {
  switch (type) {
    case ProtobufLogActivityType_UnknownType:
      return ActivitySessionType_None;
    case ProtobufLogActivityType_Sleep:
      return ActivitySessionType_Sleep;
    case ProtobufLogActivityType_DeepSleep:
      return ActivitySessionType_RestfulSleep;
    case ProtobufLogActivityType_Nap:
      return ActivitySessionType_Nap;
    case ProtobufLogActivityType_DeepNap:
      return ActivitySessionType_RestfulNap;
    case ProtobufLogActivityType_Walk:
      return ActivitySessionType_Walk;
    case ProtobufLogActivityType_Run:
      return ActivitySessionType_Run;
    case ProtobufLogActivityType_Open:
      return ActivitySessionType_Open;
  }
  WTF;
}

// -----------------------------------------------------------------------------------------
// Convert ActivitySessionType to the internal protobuf representation.
/*
static ProtobufLogActivityType prv_activity_type_to_proto_type(ActivitySessionType type) {
  switch (type) {
    case ActivitySessionType_None:
      return ProtobufLogActivityType_UnknownType;
    case ActivitySessionType_Sleep:
      return ProtobufLogActivityType_Sleep;
    case ActivitySessionType_RestfulSleep:
      return ProtobufLogActivityType_DeepSleep;
    case ActivitySessionType_Nap:
      return ProtobufLogActivityType_Nap;
    case ActivitySessionType_RestfulNap:
      return ProtobufLogActivityType_DeepNap;
    case ActivitySessionType_Walk:
      return ProtobufLogActivityType_Walk;
    case ActivitySessionType_Run:
      return ProtobufLogActivityType_Run;
    case ActivitySessionType_Open:
      return ProtobufLogActivityType_Open;
    case ActivitySessionTypeCount:
      break;
  }
  WTF;
}
*/

// -----------------------------------------------------------------------------------------
// Callback used to stuff in the sender.type field of a payload
// TODO: Don't force it to be one interval
/*
static bool prv_encode_intervals(pb_ostream_t *stream, const pb_field_t *field, void * const *arg) {
  if (!pb_encode_tag(stream, PB_WT_STRING, pebble_pipeline_ActivitySession_intervals_tag)) {
    return false;
  }

  const ActivitySession *session = *arg;
  pebble_pipeline_ActivityInterval msg = {
    .offset_sec = 0,
    .duration_sec = session->length_min
  };
  return pb_encode_submessage(stream, pebble_pipeline_ActivityInterval_fields, &msg);
}
*/

ProtobufLogRef protobuf_log_activity_sessions_create(void) {
  ProtobufLogConfig log_config = {
    .type = ProtobufLogType_Events,
    .events = {}
  };

  return protobuf_log_create(&log_config, NULL /*transport*/, 0 /*max_encoded_msg_size*/);
}

// TODO: Actually make sense of this. It is completely wrong.
bool protobuf_log_activity_sessions_add(ProtobufLogRef ref, time_t sample_utc,
                                        ActivitySession *session) {
  /*
  pebble_pipeline_Event event = {
    .type = pebble_pipeline_Event_Type_ActivitySessionEvent,
    .created_time_utc = sample_utc,
    .duration = session->length_min,
    .time_utc = session->start_utc,
    .activity_session = {
      .type = {
        // TODO: Custom types
        .which_type = pebble_pipeline_ActivityType_internal_type_tag,
        .type = {
          .internal_type = prv_activity_type_to_proto_type(session->type),
        }
      },
      .start_reason = (session->manual) ? pebble_pipeline_ActivitySession_StartReason_Manual
                                        : pebble_pipeline_ActivitySession_StartReason_Automatic,
      .intervals = {
        .funcs.encode = prv_encode_intervals,
        .arg = session,
      }
    }
  };
  */

  return true;
}

// TODO: Also make this exactly to spec
bool protobuf_log_activity_sessions_decode(pebble_pipeline_Event *event_in,
                                           ActivitySession *session_out) {
  pebble_pipeline_ActivitySession *activity = &event_in->activity_session;

  *session_out = (ActivitySession) {
    .start_utc = event_in->time_utc,
    .type = prv_proto_type_to_activity_type(activity->type.type.internal_type),
    .length_min = event_in->duration,
    .manual = (activity->start_reason == pebble_pipeline_ActivitySession_StartReason_Manual),
  };

  return true;
}
