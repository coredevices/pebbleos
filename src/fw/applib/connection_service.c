/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "connection_service.h"
#include "connection_service_private.h"

#include "applib/event_service_client.h"
#include "kernel/events.h"
#include "syscall/syscall.h"
#include "system/passert.h"

#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"

static ConnectionServiceState* prv_get_state(void) {
  PebbleTask task = pebble_task_get_current();

  if (task == PebbleTask_App) {
    return app_state_get_connection_service_state();
  } else if (task == PebbleTask_Worker) {
    return worker_state_get_connection_service_state();
  }

  WTF;
}

static void prv_do_handle(PebbleEvent *e, void *context) {
  ConnectionServiceState *state = prv_get_state();
  bool connected = e->bluetooth.comm_session_event.is_open;

  ConnectionHandler handler =
      (e->bluetooth.comm_session_event.is_system ?
       state->handlers.pebble_app_connection_handler :
       state->handlers.pebblekit_connection_handler);

  if (handler) {
    handler(connected);
  }
}

bool connection_service_peek_pebble_app_connection(void) {
  return sys_mobile_app_is_connected_debounced();
}

bool connection_service_peek_pebblekit_connection(void) {
  return sys_pebblekit_is_connected_debounced();
}

void connection_service_unsubscribe(void) {
  ConnectionServiceState *state = prv_get_state();
  event_service_client_unsubscribe(&state->bcs_info);
  memset(&state->handlers, 0x0, sizeof(state->handlers));
}

void connection_service_subscribe(ConnectionHandlers conn_handlers) {
  ConnectionServiceState *state = prv_get_state();
  state->handlers = conn_handlers;
  event_service_client_subscribe(&state->bcs_info);
}

void connection_service_state_init(ConnectionServiceState *state) {
  *state = (ConnectionServiceState) {
    .bcs_info = {
      .type = PEBBLE_BT_CONNECTION_DEBOUNCED_EVENT,
      .handler = prv_do_handle,
    },
  };
}

// Deprecated routines kept around for backward compile compatibility

void bluetooth_connection_service_subscribe(ConnectionHandler handler) {
  ConnectionHandlers conn_handlers = {
    .pebble_app_connection_handler = handler,
    .pebblekit_connection_handler = NULL
  };
  connection_service_subscribe(conn_handlers);
}

void bluetooth_connection_service_unsubscribe(void) {
  connection_service_unsubscribe();
}

bool bluetooth_connection_service_peek(void) {
  return connection_service_peek_pebble_app_connection();
}
