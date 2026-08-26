/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/voice_endpoint.h"

#include "kernel/pbl_malloc.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/audio_endpoint.h"
#include "pbl/services/voice/voice.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/generic_attribute.h"
#include "pbl/util/math.h"
#include "pbl/util/uuid.h"

#include <stddef.h>
#include <sys/types.h>

#include "pbl/services/voice_endpoint_private.h"

PBL_LOG_MODULE_DEFINE(service_voice_endpoint, CONFIG_SERVICE_VOICE_ENDPOINT_LOG_LEVEL);

#define VOICE_CONTROL_ENDPOINT (11000)
#define VOICE_RECORDING_PAGE_MAX (24)

#ifdef CONFIG_MIC
_Static_assert(sizeof(VoiceRecordingProtocolInfo) == 30,
               "Voice recording protocol metadata layout changed");

static void prv_send_recording_command_response(CommSession *session, MsgId msg_id,
                                                uint8_t transaction_id,
                                                VoiceRecordingEndpointResult result) {
  const VoiceRecordingCommandResponse response = {
    .msg_id = msg_id,
    .transaction_id = transaction_id,
    .result = result,
  };
  comm_session_send_data(session, VOICE_CONTROL_ENDPOINT, (const uint8_t *)&response,
                         sizeof(response), COMM_SESSION_DEFAULT_TIMEOUT);
}

static void prv_send_recording_list_error(CommSession *session, uint8_t transaction_id,
                                          VoiceRecordingEndpointResult result) {
  const VoiceRecordingListResponse response = {
    .msg_id = MsgIdRecordingListResponse,
    .transaction_id = transaction_id,
    .result = result,
  };
  comm_session_send_data(session, VOICE_CONTROL_ENDPOINT, (const uint8_t *)&response,
                         sizeof(response), COMM_SESSION_DEFAULT_TIMEOUT);
}

static void prv_handle_recording_list_request(CommSession *session,
                                              const VoiceRecordingListRequest *request,
                                              size_t size) {
  if ((size < sizeof(*request)) || (request->limit == 0)) {
    const uint8_t transaction_id = (size > offsetof(VoiceRecordingListRequest, transaction_id)) ?
        request->transaction_id : 0;
    prv_send_recording_list_error(session, transaction_id,
                                  VoiceRecordingEndpointResultInvalidRequest);
    return;
  }

  const uint8_t limit = MIN(request->limit, VOICE_RECORDING_PAGE_MAX);
  VoiceRecordingInfo infos[VOICE_RECORDING_PAGE_MAX];
  bool has_more = false;
  const uint32_t count = voice_recording_list_page(infos, limit, request->offset, &has_more);
  const size_t response_size = sizeof(VoiceRecordingListResponse) +
                               count * sizeof(VoiceRecordingProtocolInfo);
  VoiceRecordingListResponse *response = kernel_zalloc_check(response_size);
  *response = (VoiceRecordingListResponse) {
    .msg_id = MsgIdRecordingListResponse,
    .transaction_id = request->transaction_id,
    .result = VoiceRecordingEndpointResultSuccess,
    .has_more = has_more,
    .count = count,
  };
  for (uint32_t i = 0; i < count; ++i) {
    response->recordings[i] = (VoiceRecordingProtocolInfo) {
      .id = infos[i].id,
      .size_bytes = infos[i].size_bytes,
      .duration_ms = infos[i].duration_ms,
      .created = infos[i].created,
      .app_uuid = infos[i].app_uuid,
    };
  }
  comm_session_send_data(session, VOICE_CONTROL_ENDPOINT, (const uint8_t *)response,
                         response_size, COMM_SESSION_DEFAULT_TIMEOUT);
  kernel_free(response);
}

static void prv_handle_recording_delete_request(CommSession *session,
                                                const VoiceRecordingIdRequest *request,
                                                size_t size) {
  if ((size < sizeof(*request)) || (request->recording_id == VOICE_RECORDING_ID_INVALID)) {
    const uint8_t transaction_id = (size > offsetof(VoiceRecordingIdRequest, transaction_id)) ?
        request->transaction_id : 0;
    prv_send_recording_command_response(session, MsgIdRecordingDeleteResponse, transaction_id,
                                        VoiceRecordingEndpointResultInvalidRequest);
    return;
  }
  VoiceRecordingEndpointResult result = VoiceRecordingEndpointResultSuccess;
  if (!voice_recording_delete(request->recording_id)) {
    result = (voice_transcribing_recording_id() == request->recording_id) ?
        VoiceRecordingEndpointResultBusy : VoiceRecordingEndpointResultNotFound;
  }
  prv_send_recording_command_response(session, MsgIdRecordingDeleteResponse,
                                      request->transaction_id, result);
}

static void prv_handle_recording_playback_request(CommSession *session,
                                                  const VoiceRecordingPlaybackRequest *request,
                                                  size_t size) {
  if (size < sizeof(*request)) {
    const uint8_t transaction_id =
        (size > offsetof(VoiceRecordingPlaybackRequest, transaction_id)) ?
        request->transaction_id : 0;
    prv_send_recording_command_response(session, MsgIdRecordingPlaybackResponse, transaction_id,
                                        VoiceRecordingEndpointResultInvalidRequest);
    return;
  }

  VoiceRecordingEndpointResult result = VoiceRecordingEndpointResultSuccess;
  switch (request->action) {
    case VoiceRecordingPlaybackActionPlay:
      if ((request->recording_id == VOICE_RECORDING_ID_INVALID) ||
          !voice_recording_play(request->recording_id)) {
        result = (voice_recording_in_progress() || voice_recording_is_playing() ||
                  (voice_transcribing_recording_id() != VOICE_RECORDING_ID_INVALID)) ?
            VoiceRecordingEndpointResultBusy : VoiceRecordingEndpointResultNotFound;
      }
      break;
    case VoiceRecordingPlaybackActionStop:
      voice_recording_stop_playback();
      break;
    default:
      result = VoiceRecordingEndpointResultInvalidRequest;
      break;
  }
  prv_send_recording_command_response(session, MsgIdRecordingPlaybackResponse,
                                      request->transaction_id, result);
}

static void prv_handle_recording_transcribe_request(CommSession *session,
                                                    const VoiceRecordingIdRequest *request,
                                                    size_t size) {
  if ((size < sizeof(*request)) || (request->recording_id == VOICE_RECORDING_ID_INVALID)) {
    const uint8_t transaction_id = (size > offsetof(VoiceRecordingIdRequest, transaction_id)) ?
        request->transaction_id : 0;
    prv_send_recording_command_response(session, MsgIdRecordingTranscribeResponse, transaction_id,
                                        VoiceRecordingEndpointResultInvalidRequest);
    return;
  }

  VoiceRecordingEndpointResult result = VoiceRecordingEndpointResultSuccess;
  if (voice_session_is_active()) {
    result = VoiceRecordingEndpointResultBusy;
  } else if (voice_start_dictation_from_recording(request->recording_id) ==
             VOICE_SESSION_ID_INVALID) {
    result = (voice_session_is_active() || voice_recording_in_progress() ||
              voice_recording_is_playing() ||
              (voice_transcribing_recording_id() != VOICE_RECORDING_ID_INVALID)) ?
        VoiceRecordingEndpointResultBusy : VoiceRecordingEndpointResultNotFound;
  }
  prv_send_recording_command_response(session, MsgIdRecordingTranscribeResponse,
                                      request->transaction_id, result);
}

static bool prv_handle_result_common(VoiceEndpointResult result,
                                     bool app_initiated,
                                     AudioEndpointSessionId session_id,
                                     GenericAttributeList *attr_list,
                                     size_t attr_list_size,
                                     Uuid **app_uuid_out) {

  GenericAttribute *uuid_attr = generic_attribute_find_attribute(attr_list,
                                                                 VEAttributeIdAppUuid,
                                                                 attr_list_size);
  if (app_initiated && !uuid_attr) {
    PBL_LOG_WRN("No app UUID found for dictation response from app-initiated "
        "session");
    voice_handle_dictation_result(VoiceEndpointResultFailInvalidMessage, session_id, NULL,
                                  app_initiated, NULL);
    return false;
  }

  Uuid *app_uuid = uuid_attr ? (Uuid *)uuid_attr->data : NULL;

  if (result != VoiceEndpointResultSuccess) {
    voice_handle_dictation_result(result, session_id, NULL, app_initiated, app_uuid);
    return false;
  }

  if (attr_list->num_attributes == 0) {
    PBL_LOG_WRN("No attributes in message");
    voice_handle_dictation_result(VoiceEndpointResultFailInvalidMessage, session_id, NULL,
                                  app_initiated, app_uuid);
    return false;
  }

  *app_uuid_out = app_uuid;
  return true;
}

static void prv_handle_dictation_result(VoiceSessionResultMsg *msg, size_t size) {
  const size_t attr_list_size = size - sizeof(VoiceSessionResultMsg) + sizeof(GenericAttributeList);
  const bool app_initiated = (msg->flags.app_initiated == 1);
  Uuid *app_uuid = NULL;

  if (!prv_handle_result_common(msg->result, app_initiated, msg->session_id,
                                &msg->attr_list, attr_list_size, &app_uuid)) {
    return;
  }

  GenericAttribute *transcription_attr = generic_attribute_find_attribute(&msg->attr_list,
      VEAttributeIdTranscription, attr_list_size);

  if (!transcription_attr || transcription_attr->length == 0) {
    PBL_LOG_WRN("No transcription attribute found");
    voice_handle_dictation_result(VoiceEndpointResultFailInvalidMessage, msg->session_id, NULL,
                                  app_initiated, app_uuid);
    return;
  }

  Transcription *transcription = (Transcription *)transcription_attr->data;
  bool valid = transcription_validate(transcription, transcription_attr->length);

  if (!valid) {
    PBL_LOG_WRN("Unrecognized transcription format received");
    voice_handle_dictation_result(VoiceEndpointResultFailInvalidRecognizerResponse,
                                  msg->session_id, NULL, app_initiated, app_uuid);
  }
  voice_handle_dictation_result(msg->result, msg->session_id, transcription,
                                app_initiated, app_uuid);
}

static void prv_handle_nlp_result(VoiceSessionResultMsg *msg, size_t size) {
  const size_t attr_list_size = size - sizeof(VoiceSessionResultMsg) + sizeof(GenericAttributeList);
  const bool app_initiated = (msg->flags.app_initiated == 1);
  Uuid *app_uuid = NULL;

  if (!prv_handle_result_common(msg->result, app_initiated, msg->session_id,
                                &msg->attr_list, attr_list_size, &app_uuid)) {
    return;
  }
  if (app_uuid) {
    PBL_LOG_WRN("Got an app UUID in a NLP result msg. Ignoring and continuing");
  }


  // The timestamp attribute is optional
  time_t timestamp = 0;
  GenericAttribute *timestamp_attr = generic_attribute_find_attribute(&msg->attr_list,
      VEAttributeIdTimestamp, attr_list_size);
  if (timestamp_attr && timestamp_attr->length == sizeof(uint32_t)) {
    uint32_t *timestamp_ptr = (uint32_t*)timestamp_attr->data;
    timestamp = *timestamp_ptr;
  }

  GenericAttribute *reminder_attr = generic_attribute_find_attribute(&msg->attr_list,
      VEAttributeIdReminder, attr_list_size);

  if (!reminder_attr || reminder_attr->length == 0) {
    PBL_LOG_WRN("No reminder attribute found");
    voice_handle_nlp_result(VoiceEndpointResultFailInvalidMessage, msg->session_id, NULL, 0);
    return;
  }
  char *reminder_str = kernel_zalloc_check(reminder_attr->length + 1);
  memcpy(reminder_str, reminder_attr->data, reminder_attr->length);
  reminder_str[reminder_attr->length] = '\0';

  voice_handle_nlp_result(msg->result, msg->session_id, reminder_str, timestamp);
  kernel_free(reminder_str);
}
#endif

#ifdef CONFIG_MIC
void voice_endpoint_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t size) {
  if (size < 1) {
    PBL_LOG_WRN("Empty voice endpoint message");
    return;
  }
  MsgId msg_id = data[0];
  switch (msg_id) {
    case MsgIdSessionSetup: {
      if (size >= sizeof(SessionSetupResultMsg)) {
        SessionSetupResultMsg *msg = (SessionSetupResultMsg *) data;
        
        // Validate result enum value to prevent crashes from invalid values
        VoiceEndpointResult result = msg->result;
        if (result > VoiceEndpointResultFailInvalidMessage) {
          PBL_LOG_ERR("Invalid VoiceEndpointResult value: %d, treating as invalid message", 
                  (int)result);
          result = VoiceEndpointResultFailInvalidMessage;
        }
        
        bool app_initiated = (msg->flags.app_initiated == 1);
        voice_handle_session_setup_result(result, msg->session_type, app_initiated);
      } else {
        PBL_LOG_WRN("Invalid size for session setup result message");
      }
      break;
    }
    case MsgIdDictationResult: {
      if (size >= sizeof(VoiceSessionResultMsg)) {
        VoiceSessionResultMsg *msg = (VoiceSessionResultMsg *) data;
        prv_handle_dictation_result(msg, size);
      } else {
        PBL_LOG_WRN("Invalid size for dictation result message %zu", size);
      }
      break;
    }
    case MsgIdNLPResult: {
      if (size >= sizeof(VoiceSessionResultMsg)) {
        VoiceSessionResultMsg *msg = (VoiceSessionResultMsg *) data;
        prv_handle_nlp_result(msg, size);
      } else {
        PBL_LOG_WRN("Invalid size for dictation result message %zu", size);
      }
      break;
    }
    case MsgIdRecordingListRequest:
      prv_handle_recording_list_request(session, (const VoiceRecordingListRequest *)data, size);
      break;
    case MsgIdRecordingDeleteRequest:
      prv_handle_recording_delete_request(session, (const VoiceRecordingIdRequest *)data, size);
      break;
    case MsgIdRecordingPlaybackRequest:
      prv_handle_recording_playback_request(session,
                                            (const VoiceRecordingPlaybackRequest *)data, size);
      break;
    case MsgIdRecordingTranscribeRequest:
      prv_handle_recording_transcribe_request(session, (const VoiceRecordingIdRequest *)data, size);
      break;
    default:
      // Ignore invalid message ID
      PBL_LOG_WRN("Invalid message ID");
      break;
  }

}
#else
void voice_endpoint_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t size) {
}
#endif

void voice_endpoint_setup_session(VoiceEndpointSessionType session_type,
    AudioEndpointSessionId session_id, AudioTransferInfoSpeex *info, Uuid *app_uuid) {

  CommSession *comm_session = comm_session_get_system_session();
  comm_session_set_responsiveness(comm_session, BtConsumerPpVoiceEndpoint, ResponseTimeMin,
                                  MIN_LATENCY_MODE_TIMEOUT_VOICE_SECS);

  // We're only sending one attribute now: the speex audio transfer info packet
  size_t size = sizeof(SessionSetupMsg) + sizeof(GenericAttribute) +
                sizeof(AudioTransferInfoSpeex) +
                (app_uuid ? (sizeof(Uuid) + sizeof(GenericAttribute)) : 0);
  SessionSetupMsg *msg = kernel_malloc_check(size);
  *msg = (SessionSetupMsg) {
    .msg_id = MsgIdSessionSetup,
    .session_type = session_type,
    .session_id = session_id,
    .attr_list.num_attributes = 1,
  };

  GenericAttribute *attr = msg->attr_list.attributes;
  if (app_uuid) {
    // set this after struct initialization because the rest of the fields in the bitfield are left
    // uninitialized if just one is set.
    msg->flags.app_initiated = 1;

    // we're also sending the app UUID
    msg->attr_list.num_attributes += 1;

    // add app UUID attribute
    attr = generic_attribute_add_attribute(attr, VEAttributeIdAppUuid, app_uuid, sizeof(Uuid));
  }

  attr = generic_attribute_add_attribute(attr, VEAttributeIdAudioTransferInfoSpeex, info,
      sizeof(AudioTransferInfoSpeex));

  size_t actual_size = (uint8_t *)attr - (uint8_t *)msg;
  PBL_ASSERTN(actual_size == size);

  comm_session_send_data(comm_session, VOICE_CONTROL_ENDPOINT, (uint8_t *)msg,
                         size, COMM_SESSION_DEFAULT_TIMEOUT);
  kernel_free(msg);
}
