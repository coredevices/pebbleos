/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "kernel/events.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/notifications/alerts.h"
#include "pbl/services/phone_call.h"
#include "pbl/services/phone_call_util.h"

extern T_STATIC void prv_handle_phone_event(PebbleEvent *e, void *context);
extern T_STATIC void prv_handle_mobile_app_event(PebbleEvent *e, void *context);
extern T_STATIC void prv_handle_ancs_disconnected_event(PebbleEvent *e, void *context);
extern T_STATIC void prv_handle_remote_app_info_event(PebbleEvent *e, void *context);


///////////////////////////////////////////////////////////
// Stubs
///////////////////////////////////////////////////////////
#include "stubs_analytics.h"
#include "stubs_event_service_client.h"
#include "stubs_logging.h"
#include "stubs_new_timer.h"
#include "stubs_pbl_malloc.h"
#include "stubs_phone_call_util.h"
#include "stubs_session.h"
#include "stubs_system_task.h"

bool alerts_should_notify_for_type(AlertType type) {
  return true;
}

void ancs_perform_action(uint32_t notification_uid, uint8_t action_id) {}

void ancs_phone_call_temporarily_block_missed_calls(void) {}

void pp_answer_call(uint32_t cookie) {}

void pp_decline_call(uint32_t cookie) {}

void pp_get_phone_state(void) {}

void pp_get_phone_state_set_enabled(bool enabled) {}

// Phone UI stubs that allow us to track what phone_call.c is doing
static PhoneEventType s_last_phone_ui_event;
static bool s_last_show_ongoing_call_ui;
void phone_ui_handle_incoming_call(PebblePhoneCaller *caller, bool show_ongoing_call_ui,
                                   PhoneCallSource source) {
  s_last_phone_ui_event = PhoneEventType_Incoming;
  s_last_show_ongoing_call_ui = show_ongoing_call_ui;
}

void phone_ui_handle_outgoing_call(PebblePhoneCaller *caller) {
  s_last_phone_ui_event = PhoneEventType_Outgoing;
}

void phone_ui_handle_missed_call(void) {
  s_last_phone_ui_event = PhoneEventType_Missed;
}

void phone_ui_handle_call_start(bool can_decline) {
  s_last_phone_ui_event = PhoneEventType_Start;
}

void phone_ui_handle_call_end(bool call_accepted, bool disconnected) {
  s_last_phone_ui_event = PhoneEventType_End;
}

void phone_ui_handle_call_hide(void) {
  s_last_phone_ui_event = PhoneEventType_Hide;
}

void phone_ui_handle_caller_id(PebblePhoneCaller *caller) {
  s_last_phone_ui_event = PhoneEventType_CallerID;
}


///////////////////////////////////////////////////////////
// Helpers
///////////////////////////////////////////////////////////

#define ANCS_CALL_UID 1
#define ANCS_UNUSED_UID 2

// Whenever we check the last phone ui event, we reset s_last_phone_ui_event so we don't end up
// checking the same event twice and assume everything went well
#define ASSERT_LAST_EVENT(event) \
  cl_assert_equal_i(s_last_phone_ui_event, event); \
  s_last_phone_ui_event = PhoneEventType_Invalid;

static void prv_put_comm_session_event(bool app_connected) {
  PebbleEvent comm_session_event = {
    .type = PEBBLE_COMM_SESSION_EVENT,
    .bluetooth.comm_session_event = (PebbleCommSessionEvent) {
      .is_system = true,
      .is_open = app_connected,
    }
  };
  prv_handle_mobile_app_event(&comm_session_event, NULL);
}

static void prv_put_phone_event(PhoneEventType type, PhoneCallSource source,
                                uint32_t call_identifier) {
  PebbleEvent phone_event = {
    .type = PEBBLE_PHONE_EVENT,
    .phone = {
      .type = type,
      .source = source,
      .call_identifier = call_identifier,
      .caller = NULL,
    }
  };
  prv_handle_phone_event(&phone_event, NULL);
}

static void prv_put_remote_app_info_event(RemoteOS os) {
  PebbleEvent app_info_event = {
    .type = PEBBLE_REMOTE_APP_INFO_EVENT,
    .bluetooth = {
      .app_info_event = {
        .os = os,
      }
    }
  };
  prv_handle_remote_app_info_event(&app_info_event, NULL);
}

static void prv_put_incoming_call_event(PhoneCallSource source, bool app_connected) {
  prv_put_comm_session_event(app_connected);
  prv_put_phone_event(PhoneEventType_Incoming, source, ANCS_CALL_UID);
}

static void prv_call_end(void) {
  // Note: the source doesn't matter here, phone_call.c ignores it
  prv_put_phone_event(PhoneEventType_End, PhoneCallSource_PP, ANCS_CALL_UID);
}

static void prv_call_start(void) {
  // Note: the source doesn't matter here, phone_call.c ignores it
  prv_put_phone_event(PhoneEventType_Start, PhoneCallSource_PP, ANCS_CALL_UID);
}

static void prv_call_hide(uint32_t call_identifier) {
  // Note: the source doesn't matter here, phone_call.c ignores it
  prv_put_phone_event(PhoneEventType_Hide, PhoneCallSource_PP, call_identifier);
}

static void prv_ancs_disconnect(void) {
  PebbleEvent ancs_event = {
    .type = PEBBLE_ANCS_DISCONNECTED_EVENT,
  };
  prv_handle_ancs_disconnected_event(&ancs_event, NULL);
}


///////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////

void test_phone_call__initialize(void) {
  //fake_comm_session_init();
  phone_call_service_init();
  prv_call_end();
  s_last_phone_ui_event = PhoneEventType_Invalid;
//  s_transport = fake_transport_create(TransportDestinationSystem, NULL, NULL);
//  s_session = fake_transport_set_connected(s_transport, true /* connected */);
//  pp_get_phone_state_set_enabled(false);
}


// ---------------------------------------------------------------------------------------
void test_phone_call__cleanup(void) {
//  fake_comm_session_cleanup();
}


// ---------------------------------------------------------------------------------------
// Basic test for incoming calls over PP
void test_phone_call__pp_incoming(void) {
  // We should only allow incoming calls when connected to the mobile app (this should never really
  // happen for PP)
  prv_put_incoming_call_event(PhoneCallSource_PP, false);
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  prv_put_incoming_call_event(PhoneCallSource_PP, true);
  ASSERT_LAST_EVENT(PhoneEventType_Incoming);

  // Make sure we don't process incoming calls while we're in a call
  prv_put_incoming_call_event(PhoneCallSource_PP, true);
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  // Losing ANCS connectivity in this case shouldn't matter
  prv_ancs_disconnect();
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  // Losing mobile connection should end the call
  prv_put_comm_session_event(false /* app_connected */);
  ASSERT_LAST_EVENT(PhoneEventType_End);
}


// ---------------------------------------------------------------------------------------
// Basic test for incoming calls over ANCS on iOS 8 and below
void test_phone_call__ancs_legacy_incoming(void) {
  // We should only allow the incoming call if we're connected to the app for polling reasons
  prv_put_incoming_call_event(PhoneCallSource_ANCS_Legacy, false);
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  prv_put_incoming_call_event(PhoneCallSource_ANCS_Legacy, true);
  ASSERT_LAST_EVENT(PhoneEventType_Incoming);

  // Make sure we don't process incoming calls while we're in a call
  prv_put_incoming_call_event(PhoneCallSource_ANCS_Legacy, true);
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  // Losing ANCS connectivity in this case shouldn't matter
  prv_ancs_disconnect();
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  // Losing mobile app connection should end the call on the watch
  prv_put_comm_session_event(false /* app_connected */);
  ASSERT_LAST_EVENT(PhoneEventType_End);
}


// ---------------------------------------------------------------------------------------
// Basic test for incoming calls on iOS 9 and up
void test_phone_call__ancs_incoming(void) {
  // We should allow incoming calls with or without a mobile app on iOS 9
  prv_put_incoming_call_event(PhoneCallSource_ANCS, false);
  ASSERT_LAST_EVENT(PhoneEventType_Incoming);

  prv_call_end();
  ASSERT_LAST_EVENT(PhoneEventType_End);

  prv_put_incoming_call_event(PhoneCallSource_ANCS, true);
  ASSERT_LAST_EVENT(PhoneEventType_Incoming);

  // Make sure we don't process incoming calls while we're in a call
  prv_put_incoming_call_event(PhoneCallSource_ANCS, true);
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  // Losing connection to mobile app should have no effect if iOS 9
  prv_put_comm_session_event(false /* app_connected */);
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  // Losing ANCS here should end the call
  prv_ancs_disconnect();
  ASSERT_LAST_EVENT(PhoneEventType_End);
}


// ---------------------------------------------------------------------------------------
// Basic test for call start events
void test_phone_call__call_start(void) {
  // A call start event with ANCS should act as a call end in order to hide the phone ui
  prv_put_incoming_call_event(PhoneCallSource_ANCS_Legacy, true);
  ASSERT_LAST_EVENT(PhoneEventType_Incoming);

  prv_call_start();
  ASSERT_LAST_EVENT(PhoneEventType_End);

  // A call start event with PP should keep the phone ui up
  prv_put_incoming_call_event(PhoneCallSource_PP, true);
  ASSERT_LAST_EVENT(PhoneEventType_Incoming);

  prv_call_start();
  ASSERT_LAST_EVENT(PhoneEventType_Start);
}


// ---------------------------------------------------------------------------------------
// Make sure we handle ANCS notification removals properly
void test_phone_call__ancs_hide(void) {
  // Make sure we hide the call when ANCS tells us the notification was removed (but only if the
  // call id matches the current call)
  prv_put_incoming_call_event(PhoneCallSource_ANCS, false);
  ASSERT_LAST_EVENT(PhoneEventType_Incoming);

  prv_call_hide(ANCS_UNUSED_UID);
  ASSERT_LAST_EVENT(PhoneEventType_Invalid);

  prv_call_hide(ANCS_CALL_UID);
  ASSERT_LAST_EVENT(PhoneEventType_Hide);
}

void test_phone_call__pp_incoming_from_android_shows_the_ongoing_call_ui(void) {
  prv_put_remote_app_info_event(RemoteOSAndroid);

  prv_put_incoming_call_event(PhoneCallSource_PP, true);

  cl_assert_equal_i(s_last_phone_ui_event, PhoneEventType_Incoming);
  cl_assert_equal_b(s_last_show_ongoing_call_ui, true);
}

void test_phone_call__pp_incoming_from_ios_does_not(void) {
  // An iOS companion sends calls over PP as well, and can neither answer nor
  // end one, so counting the call would be counting a phone still ringing.
  prv_put_remote_app_info_event(RemoteOSiOS);

  prv_put_incoming_call_event(PhoneCallSource_PP, true);

  cl_assert_equal_i(s_last_phone_ui_event, PhoneEventType_Incoming);
  cl_assert_equal_b(s_last_show_ongoing_call_ui, false);
}

void test_phone_call__pp_incoming_after_the_ios_app_went_away(void) {
  prv_put_remote_app_info_event(RemoteOSiOS);
  prv_put_comm_session_event(false /* app_connected */);
  prv_call_end();
  s_last_phone_ui_event = PhoneEventType_Invalid;

  // A call from whoever connected next, arriving before its version response.
  prv_put_incoming_call_event(PhoneCallSource_PP, true);

  cl_assert_equal_i(s_last_phone_ui_event, PhoneEventType_Incoming);
  cl_assert_equal_b(s_last_show_ongoing_call_ui, true);
}
