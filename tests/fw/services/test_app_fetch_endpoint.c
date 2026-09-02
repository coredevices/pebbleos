/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/comm_session/session.h"
#include "pbl/services/app_fetch_endpoint.h"
#include <pbl/logging/logging.h>
#include "pbl/util/attributes.h"


#include <stdio.h>

// Fakes
////////////////////////////////////
#include "fake_events.h"
#include "fake_new_timer.h"
#include "fake_pbl_malloc.h"
#include "fake_session.h"
#include "fake_system_task.h"

// Stubs
////////////////////////////////////
#include "stubs_app_cache.h"
#include "stubs_bt_lock.h"
#include "stubs_hexdump.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_rand_ptr.h"

typedef struct {} EventServiceInfo;

void app_event_service_subscribe(EventServiceInfo * service_info) {
  return;
}

void put_bytes_cancel(void) {
}

void put_bytes_expect_init(uint32_t timeout_ms) {
}

void app_storage_delete_bank(uint32_t bank) {
}


typedef struct PACKED {
  uint16_t length;
  uint16_t endpoint_id;
} PebbleProtocolHeader;

enum {
  APP_FETCH_INSTALL_COMMAND = 0x01,
};

enum {
  APP_FETCH_INSTALL_RESPONSE = 0x01,
};

typedef struct AppFetchData {
  CommSession *session;
  size_t length;
  uint8_t data[];
} AppFetchData;

typedef struct PACKED {
  uint8_t command;
  Uuid uuid;
  uint32_t app_id;
} AppFetchRequest;

extern void app_fetch_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t length);

static const uint16_t APP_FETCH_ENDPOINT_ID = 6001;

static const uint32_t app_id_1 = 42;
static const Uuid uuid_1 = {0x6b, 0xf6, 0x21, 0x5b, 0xc9, 0x7f, 0x40, 0x9e,
                                 0x8c, 0x31, 0x4f, 0x55, 0x65, 0x72, 0x22, 0xb4};

Transport *s_transport;

/* Start of test */

void test_app_fetch_endpoint__initialize(void) {
  fake_comm_session_init();
  s_transport = fake_transport_create(TransportDestinationSystem, NULL, NULL);
  fake_transport_set_connected(s_transport, true /* connected */);
}

void test_app_fetch_endpoint__cleanup(void) {
  fake_transport_destroy(s_transport);
  s_transport = NULL;
  fake_comm_session_cleanup();
  fake_system_task_callbacks_cleanup();
}

/*************************************
 * Checking for valid INSERT command *
 *************************************/

 static const uint8_t s_app_fetch_success[] = {
   // Message Header
   0x01,                     // APP_FETCH_INSTALL_RESPONSE
   0x01,                     // ACK: 0x01
 };

static void prv_check_valid_app_fetch_request(uint16_t endpoint_id, const uint8_t* data,
    unsigned int data_length) {

  // AppFetchRequest request = *(AppFetchRequest *)data;
  PBL_LOG_DBG("sizeof: %lu length: %u", sizeof(AppFetchRequest), data_length);
  AppFetchRequest request = *(AppFetchRequest *)data;

  cl_assert_equal_i(endpoint_id, APP_FETCH_ENDPOINT_ID);
  cl_assert_equal_i(request.command, APP_FETCH_INSTALL_COMMAND);
  cl_assert_equal_i(request.app_id, app_id_1);
  cl_assert_equal_b(true, uuid_equal(&request.uuid, &uuid_1));

  // if anything is planning on being done relating to this ACK, then it will be done after this
  app_fetch_protocol_msg_callback(comm_session_get_system_session(),
                                  s_app_fetch_success, sizeof(s_app_fetch_success));
}

void test_app_fetch_endpoint__app_fetch_binaries(void) {
  // set the function that will validate the data in the send buffer
  fake_transport_set_sent_cb(s_transport, prv_check_valid_app_fetch_request);

  // queue of system task to send an app_fetch request
  app_fetch_binaries(&uuid_1, app_id_1, 0);

  // process system task events
  fake_system_task_callbacks_invoke_pending();
  fake_comm_session_process_send_next();

  // if anything is planning on being done relating to the ACK sent back from phone,
  // then it should be checked here.
}

static void prv_fetch_complete_app() {
  app_fetch_binaries(&uuid_1, app_id_1, false);
  app_fetch_put_bytes_event_handler(&(PebblePutBytesEvent){
    .type = PebblePutBytesEventTypeCleanup,
    .object_type = ObjectAppResources,
    .has_cookie = true,
  });
  fake_system_task_callbacks_invoke_pending();
  app_fetch_put_bytes_event_handler(&(PebblePutBytesEvent){
    .type = PebblePutBytesEventTypeCleanup,
    .object_type = ObjectWatchApp,
    .has_cookie = true,
  });
  fake_system_task_callbacks_invoke_pending();
}

void test_app_fetch_endpoint__fetch_complete(void) {
  prv_fetch_complete_app();

  const PebbleEvent e = fake_event_get_last();
  cl_assert_equal_i(PEBBLE_APP_FETCH_EVENT, e.type);
  cl_assert_equal_i(AppFetchEventTypeFinish, e.app_fetch.type);
}
