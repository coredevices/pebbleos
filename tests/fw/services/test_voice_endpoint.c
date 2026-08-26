/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/voice/transcription.h"
#include "pbl/services/voice/voice.h"
#include "pbl/services/voice_endpoint.h"
#include "pbl/services/audio_endpoint.h"

#include "pbl/services/voice_endpoint_private.h"

#include "fake_session.h"
#include "fake_system_task.h"

#include "stubs_bt_lock.h"
#include "stubs_hexdump.h"
#include "stubs_pbl_malloc.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_rand_ptr.h"

#include <stdlib.h>
#include <string.h>

extern void voice_endpoint_protocol_msg_callback(CommSession *session, const uint8_t* data,
    size_t size);

static VoiceEndpointSessionType s_session_type;
static VoiceEndpointResult s_session_result;
static Transcription *s_transcription = NULL;
static AudioEndpointSessionId s_session_id;
static bool s_app_initiated;
static Uuid s_app_uuid;
static uint8_t s_num_attributes;
static char* s_reminder_str = NULL;
static time_t s_timestamp;

static const Uuid s_uuid_invalid = UUID_INVALID;

static Transport *s_transport;
static CommSession *s_comm_session;

static VoiceRecordingInfo s_recording_infos[2];
static uint32_t s_recording_info_count;
static uint32_t s_recording_list_max;
static uint32_t s_recording_list_offset;
static bool s_recording_list_has_more;
static uint32_t s_recording_list_call_count;

static bool s_recording_delete_result;
static VoiceRecordingId s_recording_delete_id;
static uint32_t s_recording_delete_call_count;

static bool s_recording_play_result;
static VoiceRecordingId s_recording_play_id;
static uint32_t s_recording_play_call_count;
static bool s_recording_in_progress;
static bool s_recording_is_playing;
static uint32_t s_recording_stop_playback_call_count;

static VoiceSessionId s_recording_transcription_result;
static VoiceRecordingId s_recording_transcription_id;
static uint32_t s_recording_transcription_call_count;
static bool s_voice_session_active;
static VoiceRecordingId s_transcribing_recording_id;

// Stubs

uint32_t voice_recording_list_page(VoiceRecordingInfo *out, uint32_t max, uint32_t offset,
                                   bool *has_more) {
  s_recording_list_call_count++;
  s_recording_list_max = max;
  s_recording_list_offset = offset;
  const uint32_t count = (s_recording_info_count < max) ? s_recording_info_count : max;
  memcpy(out, s_recording_infos, count * sizeof(*out));
  *has_more = s_recording_list_has_more;
  return count;
}

bool voice_recording_delete(VoiceRecordingId id) {
  s_recording_delete_call_count++;
  s_recording_delete_id = id;
  return s_recording_delete_result;
}

bool voice_recording_play(VoiceRecordingId id) {
  s_recording_play_call_count++;
  s_recording_play_id = id;
  return s_recording_play_result;
}

void voice_recording_stop_playback(void) {
  s_recording_stop_playback_call_count++;
  s_recording_is_playing = false;
}

bool voice_recording_in_progress(void) {
  return s_recording_in_progress;
}

bool voice_recording_is_playing(void) {
  return s_recording_is_playing;
}

VoiceSessionId voice_start_dictation_from_recording(VoiceRecordingId recording_id) {
  s_recording_transcription_call_count++;
  s_recording_transcription_id = recording_id;
  return s_recording_transcription_result;
}

bool voice_session_is_active(void) {
  return s_voice_session_active;
}

VoiceRecordingId voice_transcribing_recording_id(void) {
  return s_transcribing_recording_id;
}

void voice_handle_session_setup_result(VoiceEndpointResult result,
    VoiceEndpointSessionType session_type, bool app_initiated) {
  s_session_type = session_type;
  s_session_result = result;
  s_app_initiated = app_initiated;
}


void voice_handle_dictation_result(VoiceEndpointResult result,
    AudioEndpointSessionId session_id, Transcription *transcription, bool app_initiated,
    Uuid *app_uuid) {
  if (s_transcription) {
    free(s_transcription);
  }
  s_session_id = session_id;
  s_session_result = result;
  if (transcription && (result == VoiceEndpointResultSuccess)) {
    size_t size = sizeof(Transcription);
    uint8_t *end = (uint8_t *)transcription_iterate_sentences(transcription->sentences,
        transcription->sentence_count, NULL, NULL);
    size += end - (uint8_t *)transcription->sentences;
    s_transcription = malloc(size);
    memcpy(s_transcription, transcription, size);
  } else {
    s_transcription = NULL;
  }
  if (app_uuid) {
    memcpy(&s_app_uuid, app_uuid, sizeof(Uuid));
  }
  s_app_initiated = app_initiated;
}

void voice_handle_nlp_result(VoiceEndpointResult result, AudioEndpointSessionId session_id,
    char *reminder, time_t timestamp) {
  if (s_reminder_str) {
    free(s_reminder_str);
    s_reminder_str = NULL;
  }
  if (reminder) {
    s_reminder_str = malloc(strlen(reminder) + 1);
    strcpy(s_reminder_str, reminder);
  }

  s_timestamp = timestamp;
  s_session_result = result;
  s_session_id = session_id;
}


// setup and teardown
void test_voice_endpoint__initialize(void) {
  s_session_type = 0;
  s_session_result = -1;
  s_session_id = AUDIO_ENDPOINT_SESSION_INVALID_ID;

  s_app_initiated = false;
  s_app_uuid = UUID_INVALID;

  s_timestamp = 0;

  memset(s_recording_infos, 0, sizeof(s_recording_infos));
  s_recording_info_count = 0;
  s_recording_list_max = 0;
  s_recording_list_offset = 0;
  s_recording_list_has_more = false;
  s_recording_list_call_count = 0;

  s_recording_delete_result = true;
  s_recording_delete_id = VOICE_RECORDING_ID_INVALID;
  s_recording_delete_call_count = 0;

  s_recording_play_result = true;
  s_recording_play_id = VOICE_RECORDING_ID_INVALID;
  s_recording_play_call_count = 0;
  s_recording_in_progress = false;
  s_recording_is_playing = false;
  s_recording_stop_playback_call_count = 0;

  s_recording_transcription_result = 1;
  s_recording_transcription_id = VOICE_RECORDING_ID_INVALID;
  s_recording_transcription_call_count = 0;
  s_voice_session_active = false;
  s_transcribing_recording_id = VOICE_RECORDING_ID_INVALID;

  fake_comm_session_init();
  s_transport = fake_transport_create(TransportDestinationSystem, NULL, NULL);
  s_comm_session = fake_transport_set_connected(s_transport, true);
}

void test_voice_endpoint__cleanup(void) {
  fake_system_task_callbacks_cleanup();
  fake_comm_session_cleanup();
  if (s_transcription) {
    free(s_transcription);
    s_transcription = NULL;
  }
  if (s_reminder_str) {
    free(s_reminder_str);
    s_reminder_str = NULL;
  }
}

// tests

static void prv_send_recording_request(const uint8_t *request, size_t size) {
  voice_endpoint_protocol_msg_callback(s_comm_session, request, size);
  fake_comm_session_process_send_next();
}

static void prv_assert_recording_command_response(MsgId msg_id, uint8_t transaction_id,
                                                  VoiceRecordingEndpointResult result) {
  const uint8_t expected[] = {
    msg_id,
    0, 0, 0, 0,
    transaction_id,
    result,
  };
  fake_transport_assert_sent(s_transport, 0, 11000, expected, sizeof(expected));
}

static void prv_test_session_setup_msg(uint16_t endpoint_id, const uint8_t* data,
    unsigned int length) {
  cl_assert_equal_i(endpoint_id, 11000);

  size_t expected_len = sizeof(SessionSetupMsg) + sizeof(GenericAttribute) +
                        sizeof(AudioTransferInfoSpeex) +
                        (s_app_initiated ? (sizeof(GenericAttribute) + sizeof(Uuid)) : 0);
  cl_assert_equal_i(length, expected_len);

  SessionSetupMsg *msg = (SessionSetupMsg *)data;
  cl_assert_equal_i(msg->msg_id, MsgIdSessionSetup);
  cl_assert_equal_i(msg->session_type, s_session_type);
  cl_assert_equal_i(msg->session_id, s_session_id);
  cl_assert_equal_i(msg->attr_list.num_attributes, s_num_attributes);

  bool app_initiated = msg->flags.app_initiated != 0;
  cl_assert_equal_b(app_initiated, s_app_initiated);

  if (!app_initiated) {
    uint8_t expected_attr_data[] = {
      0x01,  // attribute id - attribute info speex
      0x1D, 0x00,  // attribute length
      '1', '.', '2', 'r', 'c', '1', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0x80, 0x3e, 0x00, 0x00,
      0x00, 0x32,
      0x04,
      0x40, 0x01,
    };

    cl_assert_equal_m(msg->attr_list.attributes, expected_attr_data, sizeof(expected_attr_data));
  } else {
    uint8_t expected_attr_data[] = {
      0x03,  // attribute id - app uuid
      (uint8_t)sizeof(Uuid), 0x00,
      0xa8, 0xc5, 0x63, 0x17, 0xa2, 0x89, 0x46, 0x5c,
      0xbe, 0xf1, 0x5b, 0x98, 0x0d, 0xfd, 0xb0, 0x8a,

      0x01,  // attribute id - attribute info speex
      0x1D, 0x00,  // attribute length
      '1', '.', '2', 'r', 'c', '1', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0x80, 0x3e, 0x00, 0x00,
      0x00, 0x32,
      0x04,
      0x40, 0x01,
    };
    cl_assert_equal_m(msg->attr_list.attributes, expected_attr_data, sizeof(expected_attr_data));
  }
}

void test_voice_endpoint__send_session_setup(void) {
  fake_transport_set_sent_cb(s_transport, prv_test_session_setup_msg);

  AudioTransferInfoSpeex transfer_info = (AudioTransferInfoSpeex) {
    .sample_rate = 16000,
    .bit_rate = 12800,
    .frame_size = 320,
    .bitstream_version = 4
  };
  strncat(transfer_info.version, "1.2rc1", sizeof(transfer_info.version));

  s_num_attributes = 1;
  s_session_type = VoiceEndpointSessionTypeDictation;
  s_session_id = 1;
  s_app_initiated = false;
  voice_endpoint_setup_session(VoiceEndpointSessionTypeDictation, s_session_id, &transfer_info,
      NULL);
  fake_comm_session_process_send_next();

  s_session_type = VoiceEndpointSessionTypeCommand;
  s_session_id = 2000;
  s_app_initiated = false;
  voice_endpoint_setup_session(VoiceEndpointSessionTypeCommand, s_session_id, &transfer_info, NULL);
  fake_comm_session_process_send_next();

  s_session_type = VoiceEndpointSessionTypeNLP;
  s_session_id = 2;
  s_app_initiated = false;
  voice_endpoint_setup_session(VoiceEndpointSessionTypeNLP, s_session_id, &transfer_info,
      NULL);
  fake_comm_session_process_send_next();
}

void test_voice_endpoint__send_session_setup_app_initiated(void) {
  fake_transport_set_sent_cb(s_transport, prv_test_session_setup_msg);

  AudioTransferInfoSpeex transfer_info = (AudioTransferInfoSpeex) {
    .sample_rate = 16000,
    .bit_rate = 12800,
    .frame_size = 320,
    .bitstream_version = 4
  };
  strncat(transfer_info.version, "1.2rc1", sizeof(transfer_info.version));

  s_session_type = VoiceEndpointSessionTypeDictation;
  s_session_id = 2;
  s_num_attributes = 2;
  s_app_initiated = true;
  Uuid app_uuid = {0xa8, 0xc5, 0x63, 0x17, 0xa2, 0x89, 0x46, 0x5c,
                   0xbe, 0xf1, 0x5b, 0x98, 0x0d, 0xfd, 0xb0, 0x8a};
  voice_endpoint_setup_session(VoiceEndpointSessionTypeDictation, s_session_id, &transfer_info,
      &app_uuid);
  fake_comm_session_process_send_next();
}


void test_voice_endpoint__handle_setup_response(void) {
  uint8_t setup_response[] = {
    0x01, // Message ID: Session setup
    0x00, 0x00, 0x00, 0x00, // flags
    0x01, // Session type: dictation
    0x00  // Result: Success
  };
  voice_endpoint_protocol_msg_callback(NULL, setup_response,
      sizeof(setup_response));
  fake_system_task_callbacks_invoke_pending();

  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
  cl_assert_equal_i(s_session_type, VoiceEndpointSessionTypeDictation);
  cl_assert_equal_b(s_app_initiated, false);

  // Use failure code
  setup_response[6] = VoiceEndpointResultFailServiceUnavailable;
  voice_endpoint_protocol_msg_callback(NULL, setup_response,
      sizeof(setup_response));
  fake_system_task_callbacks_invoke_pending();

  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailServiceUnavailable);
  cl_assert_equal_i(s_session_type, VoiceEndpointSessionTypeDictation);
  cl_assert_equal_b(s_app_initiated, false);

  // App initiated failure
  setup_response[1] = 0x01;
  voice_endpoint_protocol_msg_callback(NULL, setup_response,
      sizeof(setup_response));
  fake_system_task_callbacks_invoke_pending();

  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailServiceUnavailable);
  cl_assert_equal_i(s_session_type, VoiceEndpointSessionTypeDictation);
  cl_assert_equal_b(s_app_initiated, true);

  // App initiated success
  setup_response[6] = VoiceEndpointResultSuccess;
  voice_endpoint_protocol_msg_callback(NULL, setup_response,
      sizeof(setup_response));
  fake_system_task_callbacks_invoke_pending();

  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
  cl_assert_equal_i(s_session_type, VoiceEndpointSessionTypeDictation);
  cl_assert_equal_b(s_app_initiated, true);
}

void test_voice_endpoint__handle_dictation_result(void) {
  uint8_t dictation_result[] = {
    0x02,         // Message ID: Dictation result
    0x00, 0x00, 0x00, 0x00,  // flags
    0x11, 0x22,   // Audio streaming session ID
    0x00,         // Voice session result - success

    0x01,         // attribute list - num attributes

    0x02,         // attribute type - transcription
    0x2F, 0x00,   // attribute length

    // Transcription
    0x01,         // Transcription type
    0x02,         // Sentence count

    // Sentence #1
    0x02, 0x00,   // Word count

    // Word #1
    85,           // Confidence
    0x05, 0x00,   // Word length
    'H', 'e', 'l', 'l', 'o',

    // Word #2
    74,           // Confidence
    0x08, 0x00,   // Word length
    'c', 'o', 'm', 'p', 'u', 't', 'e', 'r',

    // Sentence #2
    0x03, 0x00,   // Word count

    // Word #1
    13,           // Confidence
    0x04, 0x00,   // Word length
    'h', 'e', 'l', 'l',

    // Word #1
    3,           // Confidence
    0x02, 0x00,   // Word length
    'o', 'h',

    // Word #2
    0,           // Confidence
    0x07, 0x00,   // Word length
    'c', 'o', 'm', 'p', 'u', 't', 'a',
  };

  // test valid message
  voice_endpoint_protocol_msg_callback(NULL, dictation_result, sizeof(dictation_result));
  fake_system_task_callbacks_invoke_pending();
  cl_assert(s_transcription != NULL);
  size_t offset = sizeof(VoiceSessionResultMsg) + sizeof(GenericAttribute);
  cl_assert_equal_m(s_transcription, &dictation_result[offset], sizeof(dictation_result) - offset);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
  cl_assert_equal_i(s_app_initiated, false);
  cl_assert_equal_m(&s_app_uuid, &s_uuid_invalid, sizeof(Uuid));

  // test size too small by 1
  s_session_id = 0;
  voice_endpoint_protocol_msg_callback(NULL, dictation_result,  sizeof(dictation_result) - 1);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_p(s_transcription, NULL);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailInvalidMessage);
  cl_assert_equal_i(s_app_initiated, false);
  cl_assert_equal_m(&s_app_uuid, &s_uuid_invalid, sizeof(Uuid));

  // test size larger than necessary by 1
  s_session_id = 0;
  voice_endpoint_protocol_msg_callback(NULL, dictation_result, sizeof(dictation_result) + 1);
  fake_system_task_callbacks_invoke_pending();
  cl_assert(s_transcription != NULL);
  cl_assert_equal_m(s_transcription, &dictation_result[offset], sizeof(dictation_result) - offset);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
  cl_assert_equal_i(s_app_initiated, false);
  cl_assert_equal_m(&s_app_uuid, &s_uuid_invalid, sizeof(Uuid));

  // test that we can handle a message with an attribute which is not a recognized attribute
  s_session_id = 0;
  s_session_result = VoiceEndpointResultSuccess;
  dictation_result[9] = 99;
  voice_endpoint_protocol_msg_callback(NULL, dictation_result, sizeof(dictation_result));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_p(s_transcription, NULL);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailInvalidMessage);
  cl_assert_equal_i(s_app_initiated, false);
  cl_assert_equal_m(&s_app_uuid, &s_uuid_invalid, sizeof(Uuid));
  dictation_result[9] = 2;

  // test that we can handle a response with no attributes
  s_session_id = 0;
  dictation_result[8] = 0;  // set num attributes field to 0
  voice_endpoint_protocol_msg_callback(NULL, dictation_result, sizeof(VoiceSessionResultMsg));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_p(s_transcription, NULL);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailInvalidMessage);
  cl_assert_equal_i(s_app_initiated, false);
  cl_assert_equal_m(&s_app_uuid, &s_uuid_invalid, sizeof(Uuid));
  dictation_result[8] = 1; //restore transcription type

  // test that we can handle a timeout error from the phone
  s_session_id = 0;
  dictation_result[7] = VoiceEndpointResultFailTimeout;  // indicate transcription failure
  voice_endpoint_protocol_msg_callback(NULL, dictation_result, sizeof(VoiceSessionResultMsg));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_p(s_transcription, NULL);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailTimeout);
  cl_assert_equal_i(s_app_initiated, false);
  cl_assert_equal_m(&s_app_uuid, &s_uuid_invalid, sizeof(Uuid));

  // test that we can handle an invalid length message
  s_session_id = 0;
  dictation_result[7] = VoiceEndpointResultFailInvalidMessage;  // indicate transcription failure
  voice_endpoint_protocol_msg_callback(NULL, dictation_result, sizeof(VoiceSessionResultMsg) - 1);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_session_id, 0);
  dictation_result[7] = VoiceEndpointResultSuccess; // restore transcription result
}

void test_voice_endpoint__handle_dictation_result_app_initiated(void) {
  uint8_t dictation_result_1[] = {
    0x02,         // Message ID: Dictation result
    0x01, 0x00, 0x00, 0x00,  // flags
    0x11, 0x22,   // Audio streaming session ID
    0x00,         // Voice session result - success

    0x02,         // attribute list - num attributes

    0x02,         // attribute type - transcription
    0x2F, 0x00,   // attribute length

    // Transcription
    0x01,         // Transcription type
    0x02,         // Sentence count

    // Sentence #1
    0x02, 0x00,   // Word count

    // Word #1
    85,           // Confidence
    0x05, 0x00,   // Word length
    'H', 'e', 'l', 'l', 'o',

    // Word #2
    74,           // Confidence
    0x08, 0x00,   // Word length
    'c', 'o', 'm', 'p', 'u', 't', 'e', 'r',

    // Sentence #2
    0x03, 0x00,   // Word count

    // Word #1
    13,           // Confidence
    0x04, 0x00,   // Word length
    'h', 'e', 'l', 'l',

    // Word #1
    3,           // Confidence
    0x02, 0x00,   // Word length
    'o', 'h',

    // Word #2
    0,           // Confidence
    0x07, 0x00,   // Word length
    'c', 'o', 'm', 'p', 'u', 't', 'a',

    0x03,         // attribute type - App UUID
    0x10, 0x00,   // attribute length

    0xa8, 0xc5, 0x63, 0x17, 0xa2, 0x89, 0x46, 0x5c,
    0xbe, 0xf1, 0x5b, 0x98, 0x0d, 0xfd, 0xb0, 0x8a,
  };

  // test valid app-initiated message
  voice_endpoint_protocol_msg_callback(NULL, dictation_result_1, sizeof(dictation_result_1));
  fake_system_task_callbacks_invoke_pending();
  cl_assert(s_transcription != NULL);
  size_t offset = sizeof(VoiceSessionResultMsg) + sizeof(GenericAttribute);
  cl_assert_equal_m(s_transcription, &dictation_result_1[offset], sizeof(dictation_result_1) -
      offset - sizeof(GenericAttribute) - sizeof(Uuid));
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
  cl_assert_equal_i(s_app_initiated, true);

  offset = sizeof(VoiceSessionResultMsg) + sizeof(GenericAttribute) + 0x2F + sizeof(GenericAttribute);
  cl_assert_equal_m(&s_app_uuid, &dictation_result_1[offset], sizeof(Uuid));
}

void test_voice_endpoint__handle_nlp_result(void) {
  uint8_t nlp_result[] = {
    0x03,         // Message ID: NLP result
    0x00, 0x00, 0x00, 0x00,  // flags
    0x11, 0x22,   // Audio streaming session ID
    0x00,         // Voice session result - success

    // attribute list
    0x02,         // num attributes

    0x04,         // attribute type - reminder
    0x04, 0x00,   // attribute length
    'P', 'h', 'i', 'l', // No null terminator

    0x05,         // attribute type - timestamp
    0x04, 0x00,   // attribute length
    0xE8, 0x17, 0x46, 0x57,  // approx May 25, 2016
  };

  // test valid message
  voice_endpoint_protocol_msg_callback(NULL, nlp_result, sizeof(nlp_result));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_s(s_reminder_str, "Phil");
  cl_assert_equal_i(s_timestamp, 0x574617E8);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
  cl_assert_equal_i(s_session_id, 0x2211);

  // test non nexistent timestamp msg
  nlp_result[8] = 1;
  s_session_id = 0;
  voice_endpoint_protocol_msg_callback(NULL, nlp_result,  sizeof(nlp_result) - 7);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_s(s_reminder_str, "Phil");
  cl_assert_equal_i(s_timestamp, 0);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
  nlp_result[8] = 2;

  // test that we can handle a message with an attribute which is not a recognized attribute
  s_session_id = 0;
  s_session_result = VoiceEndpointResultSuccess;
  nlp_result[9] = 99;
  voice_endpoint_protocol_msg_callback(NULL, nlp_result, sizeof(nlp_result));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_p(s_reminder_str, NULL);
  cl_assert_equal_i(s_timestamp, 0);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailInvalidMessage);
  nlp_result[9] = 4;

  // test that we can handle a response with no attributes
  s_session_id = 0;
  nlp_result[8] = 0;  // set num attributes field to 0
  voice_endpoint_protocol_msg_callback(NULL, nlp_result, sizeof(VoiceSessionResultMsg));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_p(s_reminder_str, NULL);
  cl_assert_equal_i(s_timestamp, 0);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailInvalidMessage);
  nlp_result[8] = 2; //restore transcription type

  // test that we can handle a timeout error from the phone
  s_session_id = 0;
  nlp_result[7] = VoiceEndpointResultFailTimeout;  // indicate transcription failure
  voice_endpoint_protocol_msg_callback(NULL, nlp_result, sizeof(VoiceSessionResultMsg));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_p(s_reminder_str, NULL);
  cl_assert_equal_i(s_timestamp, 0);
  cl_assert_equal_i(s_session_id, 0x2211);
  cl_assert_equal_i(s_session_result, VoiceEndpointResultFailTimeout);
  cl_assert_equal_i(s_app_initiated, false);
  cl_assert_equal_m(&s_app_uuid, &s_uuid_invalid, sizeof(Uuid));

  // test that we can handle an invalid length message
  s_session_id = 0;
  nlp_result[7] = VoiceEndpointResultFailInvalidMessage;  // indicate transcription failure
  voice_endpoint_protocol_msg_callback(NULL, nlp_result, sizeof(VoiceSessionResultMsg) - 1);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_session_id, 0);
  nlp_result[7] = VoiceEndpointResultSuccess; // restore transcription result
}

void test_voice_endpoint__handle_nlp_result_with_app_id(void) {
  const uint8_t nlp_result[] = {
    0x03,         // Message ID: NLP result
    0x00, 0x00, 0x00, 0x00,  // flags
    0x11, 0x22,   // Audio streaming session ID
    0x00,         // Voice session result - success

    // attribute list
    0x03,         // num attributes

    0x04,         // attribute type - reminder
    0x04, 0x00,   // attribute length
    'P', 'h', 'i', 'l', // No null terminator

    0x05,         // attribute type - timestamp
    0x04, 0x00,   // attribute length
    0xE8, 0x17, 0x46, 0x57,  // approx May 25, 2016

    0x03,         // attribute type - App UUID
    0x10, 0x00,   // attribute length

    0xa8, 0xc5, 0x63, 0x17, 0xa2, 0x89, 0x46, 0x5c,
    0xbe, 0xf1, 0x5b, 0x98, 0x0d, 0xfd, 0xb0, 0x8a,
  };

  voice_endpoint_protocol_msg_callback(NULL, nlp_result, sizeof(nlp_result));
  fake_system_task_callbacks_invoke_pending();
  // Just make sure we don't crash or do anything weird. We just ignore the app uuid for now
  cl_assert_equal_i(s_session_result, VoiceEndpointResultSuccess);
}

void test_voice_endpoint__handle_recording_list_request(void) {
  s_recording_infos[0] = (VoiceRecordingInfo) {
    .id = 0x1234,
    .size_bytes = 0x01020304,
    .duration_ms = 0x11121314,
    .created = 0x21222324,
    .app_uuid = UuidMake(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                         0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f),
  };
  s_recording_infos[1] = (VoiceRecordingInfo) {
    .id = 0xabcd,
    .size_bytes = 0x31323334,
    .duration_ms = 0x41424344,
    .created = 0x51525354,
    .app_uuid = UUID_INVALID,
  };
  s_recording_info_count = sizeof(s_recording_infos) / sizeof(s_recording_infos[0]);
  s_recording_list_has_more = true;

  const uint8_t request[] = {
    MsgIdRecordingListRequest,
    0, 0, 0, 0,
    0x5a,
    0x78, 0x56,
    0xff,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_list_call_count, 1);
  cl_assert_equal_i(s_recording_list_max, 24);
  cl_assert_equal_i(s_recording_list_offset, 0x5678);

  const uint8_t expected[] = {
    MsgIdRecordingListResponse,
    0, 0, 0, 0,
    0x5a,
    VoiceRecordingEndpointResultSuccess,
    1,
    2,

    0x34, 0x12,
    0x04, 0x03, 0x02, 0x01,
    0x14, 0x13, 0x12, 0x11,
    0x24, 0x23, 0x22, 0x21,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,

    0xcd, 0xab,
    0x34, 0x33, 0x32, 0x31,
    0x44, 0x43, 0x42, 0x41,
    0x54, 0x53, 0x52, 0x51,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  };
  fake_transport_assert_sent(s_transport, 0, 11000, expected, sizeof(expected));
}

void test_voice_endpoint__reject_invalid_recording_list_request(void) {
  const uint8_t request[] = {
    MsgIdRecordingListRequest,
    0, 0, 0, 0,
    0x5b,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_list_call_count, 0);
  uint8_t expected[] = {
    MsgIdRecordingListResponse,
    0, 0, 0, 0,
    0x5b,
    VoiceRecordingEndpointResultInvalidRequest,
    0,
    0,
  };
  fake_transport_assert_sent(s_transport, 0, 11000, expected, sizeof(expected));

  const uint8_t zero_limit_request[] = {
    MsgIdRecordingListRequest,
    0, 0, 0, 0,
    0x5c,
    0, 0,
    0,
  };
  prv_send_recording_request(zero_limit_request, sizeof(zero_limit_request));
  cl_assert_equal_i(s_recording_list_call_count, 0);
  expected[5] = 0x5c;
  fake_transport_assert_sent(s_transport, 0, 11000, expected, sizeof(expected));
}

void test_voice_endpoint__handle_recording_delete_request(void) {
  uint8_t request[] = {
    MsgIdRecordingDeleteRequest,
    0, 0, 0, 0,
    0x60,
    0x34, 0x12,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_delete_call_count, 1);
  cl_assert_equal_i(s_recording_delete_id, 0x1234);
  prv_assert_recording_command_response(MsgIdRecordingDeleteResponse, 0x60,
                                        VoiceRecordingEndpointResultSuccess);

  s_recording_delete_result = false;
  s_transcribing_recording_id = 0x1234;
  request[5] = 0x61;
  prv_send_recording_request(request, sizeof(request));
  prv_assert_recording_command_response(MsgIdRecordingDeleteResponse, 0x61,
                                        VoiceRecordingEndpointResultBusy);

  s_transcribing_recording_id = VOICE_RECORDING_ID_INVALID;
  request[5] = 0x62;
  prv_send_recording_request(request, sizeof(request));
  prv_assert_recording_command_response(MsgIdRecordingDeleteResponse, 0x62,
                                        VoiceRecordingEndpointResultNotFound);
}

void test_voice_endpoint__reject_invalid_recording_delete_request(void) {
  uint8_t request[] = {
    MsgIdRecordingDeleteRequest,
    0, 0, 0, 0,
    0x63,
    0, 0,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_delete_call_count, 0);
  prv_assert_recording_command_response(MsgIdRecordingDeleteResponse, 0x63,
                                        VoiceRecordingEndpointResultInvalidRequest);

  request[5] = 0x64;
  prv_send_recording_request(request, 6);
  cl_assert_equal_i(s_recording_delete_call_count, 0);
  prv_assert_recording_command_response(MsgIdRecordingDeleteResponse, 0x64,
                                        VoiceRecordingEndpointResultInvalidRequest);
}

void test_voice_endpoint__handle_recording_playback_request(void) {
  uint8_t request[] = {
    MsgIdRecordingPlaybackRequest,
    0, 0, 0, 0,
    0x70,
    0x34, 0x12,
    VoiceRecordingPlaybackActionPlay,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_play_call_count, 1);
  cl_assert_equal_i(s_recording_play_id, 0x1234);
  prv_assert_recording_command_response(MsgIdRecordingPlaybackResponse, 0x70,
                                        VoiceRecordingEndpointResultSuccess);

  s_recording_play_result = false;
  s_recording_in_progress = true;
  request[5] = 0x71;
  prv_send_recording_request(request, sizeof(request));
  prv_assert_recording_command_response(MsgIdRecordingPlaybackResponse, 0x71,
                                        VoiceRecordingEndpointResultBusy);

  s_recording_in_progress = false;
  request[5] = 0x72;
  prv_send_recording_request(request, sizeof(request));
  prv_assert_recording_command_response(MsgIdRecordingPlaybackResponse, 0x72,
                                        VoiceRecordingEndpointResultNotFound);

  request[5] = 0x73;
  request[8] = VoiceRecordingPlaybackActionStop;
  prv_send_recording_request(request, sizeof(request));
  cl_assert_equal_i(s_recording_stop_playback_call_count, 1);
  prv_assert_recording_command_response(MsgIdRecordingPlaybackResponse, 0x73,
                                        VoiceRecordingEndpointResultSuccess);

  request[5] = 0x74;
  request[8] = 0xff;
  prv_send_recording_request(request, sizeof(request));
  prv_assert_recording_command_response(MsgIdRecordingPlaybackResponse, 0x74,
                                        VoiceRecordingEndpointResultInvalidRequest);
}

void test_voice_endpoint__reject_truncated_recording_playback_request(void) {
  const uint8_t request[] = {
    MsgIdRecordingPlaybackRequest,
    0, 0, 0, 0,
    0x75,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_play_call_count, 0);
  cl_assert_equal_i(s_recording_stop_playback_call_count, 0);
  prv_assert_recording_command_response(MsgIdRecordingPlaybackResponse, 0x75,
                                        VoiceRecordingEndpointResultInvalidRequest);
}

void test_voice_endpoint__handle_recording_transcribe_request(void) {
  uint8_t request[] = {
    MsgIdRecordingTranscribeRequest,
    0, 0, 0, 0,
    0x80,
    0x34, 0x12,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_transcription_call_count, 1);
  cl_assert_equal_i(s_recording_transcription_id, 0x1234);
  prv_assert_recording_command_response(MsgIdRecordingTranscribeResponse, 0x80,
                                        VoiceRecordingEndpointResultSuccess);

  s_recording_transcription_result = VOICE_SESSION_ID_INVALID;
  s_voice_session_active = true;
  request[5] = 0x81;
  prv_send_recording_request(request, sizeof(request));
  cl_assert_equal_i(s_recording_transcription_call_count, 1);
  prv_assert_recording_command_response(MsgIdRecordingTranscribeResponse, 0x81,
                                        VoiceRecordingEndpointResultBusy);

  s_voice_session_active = false;
  request[5] = 0x82;
  prv_send_recording_request(request, sizeof(request));
  prv_assert_recording_command_response(MsgIdRecordingTranscribeResponse, 0x82,
                                        VoiceRecordingEndpointResultNotFound);
}

void test_voice_endpoint__reject_invalid_recording_transcribe_request(void) {
  uint8_t request[] = {
    MsgIdRecordingTranscribeRequest,
    0, 0, 0, 0,
    0x83,
    0, 0,
  };
  prv_send_recording_request(request, sizeof(request));

  cl_assert_equal_i(s_recording_transcription_call_count, 0);
  prv_assert_recording_command_response(MsgIdRecordingTranscribeResponse, 0x83,
                                        VoiceRecordingEndpointResultInvalidRequest);

  request[5] = 0x84;
  prv_send_recording_request(request, 6);
  cl_assert_equal_i(s_recording_transcription_call_count, 0);
  prv_assert_recording_command_response(MsgIdRecordingTranscribeResponse, 0x84,
                                        VoiceRecordingEndpointResultInvalidRequest);
}
