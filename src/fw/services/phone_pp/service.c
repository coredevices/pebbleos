/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/phone_pp.h"

#include "kernel/events.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/phone_call_util.h"
#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"

#include <stdbool.h>
#include <string.h>

PBL_LOG_MODULE_DEFINE(service_phone_pp, CONFIG_SERVICE_PHONE_PP_LOG_LEVEL);

#define CALLER_BUFFER_LENGTH 32

static const uint16_t PHONE_CTRL_ENDPOINT = 0x21;

static bool s_get_phone_state_enabled;

typedef enum PhoneCallState {
  CallStateIncoming,
  CallStateOutgoing,
  CallStateMissed,
  NumCallStates
} PhoneCallState;

enum PhoneCmd {
  PhoneCmdAnswer           = 0x01,
  PhoneCmdHangup           = 0x02,
  PhoneCmdGetStateRequest  = 0x03,
  PhoneCmdGetStateResponse = 0x83,
  PhoneCmdIncoming         = 0x04,
  PhoneCmdOutgoing         = 0x05,
  PhoneCmdMissed           = 0x06,
  PhoneCmdRing             = 0x07,
  PhoneCmdStart            = 0x08,
  PhoneCmdEnd              = 0x09
};

typedef struct {
  uint32_t cookie;
  char caller_number[CALLER_BUFFER_LENGTH];
  char caller_name[CALLER_BUFFER_LENGTH];
} PebbleCallInfo;


static bool get_call_info_from_msg(const uint8_t* msg, unsigned int length,
    PebbleCallInfo* info) {
  unsigned int msg_length = 0;
  info->cookie = *((uint32_t*) msg);
  msg += 4;
  msg_length += 4;

  uint8_t caller_number_size = *msg++;
  memcpy(&info->caller_number, msg,
      MIN(caller_number_size, sizeof(info->caller_number)));
  msg += caller_number_size;
  msg_length += caller_number_size + 1;

  uint8_t caller_name_size = *msg++;
  memcpy(&info->caller_name, msg,
      MIN(caller_name_size, sizeof(info->caller_name)));
  msg_length += caller_name_size + 1;

  // Ensure that we haven't run off the end of our buffer
  if (msg_length > length) {
    return false;
  }

  info->caller_number[CALLER_BUFFER_LENGTH - 1] = '\0';
  info->caller_name[CALLER_BUFFER_LENGTH - 1] = '\0';
  return true;
}

static void prv_put_call_disconnect_event(void) {
  PebbleEvent e = {
    .type = PEBBLE_PHONE_EVENT,
    .phone = {
      .type = PhoneEventType_Disconnect,
      .source = PhoneCallSource_PP,
      .call_identifier = 0, // Cookie is not yet implemented / used
    }
  };
  event_put(&e);
}

static void prv_put_call_end_event(void) {
  PebbleEvent e = {
    .type = PEBBLE_PHONE_EVENT,
    .phone = {
      .type = PhoneEventType_End,
      .source = PhoneCallSource_PP,
      .call_identifier = 0, // Cookie is not yet implemented / used
    }
  };
  event_put(&e);
}


static void prv_send_phone_command_to_handset(uint8_t cmd, uint8_t *data, unsigned length) {
  static uint8_t buffer[5];
  PBL_ASSERTN(length <= sizeof(buffer) - sizeof(cmd));

  *buffer = cmd;
  memcpy(buffer + sizeof(cmd), data, length);

  // PBL_LOG_DBG("Sending PhoneCmd: %d", cmd);
  CommSession *session = comm_session_get_system_session();
  if (!session) {
    // Looks like we disconnected...
    PBL_LOG_ERR("No CommSession for phone command, ending call");
    prv_put_call_disconnect_event();
  } else {
    comm_session_send_data(session, PHONE_CTRL_ENDPOINT, buffer,
        length + sizeof(cmd), COMM_SESSION_DEFAULT_TIMEOUT);
  }
}

void pp_answer_call(uint32_t cookie) {
  prv_send_phone_command_to_handset(PhoneCmdAnswer, (uint8_t*)&cookie, sizeof(cookie));
}

void pp_decline_call(uint32_t cookie) {
  prv_send_phone_command_to_handset(PhoneCmdHangup, (uint8_t*)&cookie, sizeof(cookie));
}

void pp_get_phone_state(void) {
  prv_send_phone_command_to_handset(PhoneCmdGetStateRequest, NULL, 0);
}

void pp_get_phone_state_set_enabled(bool enabled) {
  s_get_phone_state_enabled = enabled;
}

static bool prv_parse_msg_to_event(const uint8_t *iter, size_t length,
                                   PebbleEvent *event_out, bool is_state_response) {
  uint8_t msg_type = *iter++;
  --length;

  bool did_parse = false;
  PebbleCallInfo call_info;
  call_info = (PebbleCallInfo){};

  PebblePhoneCaller *caller = NULL;
  PhoneEventType type = ~0;

  switch (msg_type) {
    case PhoneCmdIncoming: {
      // PBL-34640 Generating incoming call events for phone state responses just gives us a bad
      // time. We can look at changing this later if iOS ever starts sending us cookies properly,
      // but it's not really worth the effort since it only applies to iOS 8
      if (is_state_response) {
        return false;
      }
      bool result = get_call_info_from_msg(iter, length, &call_info);
      if (!result) {
        PBL_LOG_ERR("Failed to read caller information from 'Incoming' phone event");
        return false;
      }

      type = PhoneEventType_Incoming;
      caller = phone_call_util_create_caller(call_info.caller_number, call_info.caller_name);
      did_parse = true;
      break;
    }

    case PhoneCmdStart: {
      type = PhoneEventType_Start;
      call_info.cookie = *((uint32_t*) iter);
      did_parse = true;
      break;
    }

    case PhoneCmdEnd: {
      type = PhoneEventType_End;
      call_info.cookie = *((uint32_t*) iter);
      did_parse = true;
      break;
    }

    case PhoneCmdRing: {
      // We generate rings internally
      // Return here so we don't log / hexdump
      return false;
    }

    case PhoneCmdOutgoing:
      // Return here so we don't log / hexdump
      return false;

    case PhoneCmdMissed:
      // Return here so we don't log / hexdump
      return false;
  }

  if (did_parse) {
    *event_out = (const PebbleEvent) {
      .type = PEBBLE_PHONE_EVENT,
      .phone = {
        .type = type,
        .source = PhoneCallSource_PP,
        .call_identifier = call_info.cookie,
        .caller = caller,
      }
    };
  } else {
    // Try to catch potentially malformed messages.
    PBL_LOG_ERR("Error parsing phone msg");
    PBL_HEXDUMP(LOG_LEVEL_INFO, iter, length);
  }

  return did_parse;
}

static void prv_parse_msg_and_emit_event(const uint8_t *msg, size_t length,
                                         bool is_state_response) {
  PebbleEvent e;
  if (prv_parse_msg_to_event(msg, length, &e, is_state_response)) {
    event_put(&e);
  }
}

void phone_protocol_msg_callback(CommSession *session, const uint8_t* iter, size_t length) {
  PBL_HEXDUMP(LOG_LEVEL_DEBUG, iter, length);

  // Get State Response is basically a list representing the state of current calls. It's
  // conveniently formatted exactly the same as the event messages, so just loop over them and
  // re-use the code that parses these messages:
  if (iter[0] == PhoneCmdGetStateResponse) {
    if (!s_get_phone_state_enabled) {
      return;
    }
    // Eat the command byte:
    --length;
    ++iter;
    uint8_t num_items = 0;
    while (length) {
      // First byte is the length of the item
      uint8_t item_length = *iter++;
      --length;
      ++num_items;
      if (length < item_length) {
        PBL_LOG_ERR("Malformed message");
        break;
      }
      prv_parse_msg_and_emit_event(iter, item_length, true /* is_state_response */);
      iter += item_length;
      length -= item_length;
    }
    if (num_items == 0) {
      // Generate fake call stop if there are no calls, to hide the phone UI in case it's showing
      prv_put_call_end_event();
    }
  } else {
    prv_parse_msg_and_emit_event(iter, length, false /* is_state_response */);
  }
}
