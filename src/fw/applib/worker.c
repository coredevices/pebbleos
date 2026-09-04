/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "worker.h"

#include "process_management/worker_manager.h"
#include "applib/event_service_client.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>


// -------------------------------------------------------------------------------------------------
static bool prv_handle_event(PebbleEvent* event) {
  PebbleEventType type = event->type;

  switch (type) {
    case PEBBLE_CALLBACK_EVENT:
      event->callback.callback(event->callback.data);
      return true;

    default:
      PBL_LOG_VERBOSE("Received an unhandled event (%u)", event->type);
      return false;
  }
}


// -------------------------------------------------------------------------------------------------
void worker_event_loop(void) {
  // Event loop:
  while (1) {
    PebbleEvent event;

    sys_get_pebble_event(&event);

    if (event.type == PEBBLE_PROCESS_DEINIT_EVENT) {
      // We're done here. Return the app's main function.
      event_cleanup(&event);
      return;
    }

    event_service_client_handle_event(&event);

    prv_handle_event(&event);

    event_cleanup(&event);
  }
}


// -------------------------------------------------------------------------------------------------
void worker_launch_app(void) {
  sys_launch_app_for_worker();
}

