/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "touch_service.h"
#include "touch_service_private.h"

#include "applib/event_service_client.h"
#include "kernel/events.h"
#include "kernel/kernel_applib_state.h"
#include "kernel/pebble_tasks.h"
#include "pbl/services/touch/touch.h"
#include "process_state/app_state/app_state.h"
#include "syscall/syscall.h"
#include "system/passert.h"

//! @return the per-task touch service state, or NULL if the current task is
//! not permitted to use the touch service (e.g. background workers, or
//! watchfaces). Callers must no-op when this returns NULL.
static TouchServiceState *prv_get_state(void) {
  PebbleTask task = pebble_task_get_current();
  switch (task) {
    case PebbleTask_App:
      // Touch is reserved for watchapps; watchfaces must not consume it.
      if (sys_app_is_watchface()) {
        return NULL;
      }
      return app_state_get_touch_service_state();
    case PebbleTask_KernelMain:
      return kernel_applib_get_touch_service_state();
    case PebbleTask_Worker:
      // Touch is not available to background workers — they have no UI.
      return NULL;
    default:
      WTF;
  }
}

static void prv_handle_touch_event(PebbleEvent *e, void *context) {
  TouchServiceState *state = prv_get_state();
  if (!state || e->type != PEBBLE_TOUCH_EVENT) {
    return;
  }
  // The system slot sees each event first, then the app-facing raw slot.
  if (state->system_handler) {
    state->system_handler(&e->touch.event, state->system_context);
  }
  if (state->raw_handler) {
    state->raw_handler(&e->touch.event, state->raw_context);
  }
}

//! Bring the shared event-service subscription in line with the slots: keep it
//! while either slot is occupied, drop it once both are empty.
static void prv_update_subscription(TouchServiceState *state) {
  const bool want = (state->system_handler != NULL) || (state->raw_handler != NULL);
  if (want && !state->subscribed) {
    state->event_info = (EventServiceInfo) {
      .type = PEBBLE_TOUCH_EVENT,
      .handler = prv_handle_touch_event,
    };
    event_service_client_subscribe(&state->event_info);
    state->subscribed = true;
  } else if (!want && state->subscribed) {
    event_service_client_unsubscribe(&state->event_info);
    state->subscribed = false;
  }
}

void touch_service_set_system_handler(TouchServiceHandler handler, void *context) {
  TouchServiceState *state = prv_get_state();
  if (!state) {
    return;
  }
  state->system_handler = handler;
  state->system_context = context;
  prv_update_subscription(state);
}

void touch_service_subscribe(TouchServiceHandler handler, void *context) {
  TouchServiceState *state = prv_get_state();
  if (!state) {
    return;
  }
  state->raw_handler = handler;
  state->raw_context = context;
  sys_touch_reset();
  sys_touch_set_raw_subscribed(handler != NULL);
  prv_update_subscription(state);
}

void touch_service_unsubscribe(void) {
  TouchServiceState *state = prv_get_state();
  if (!state) {
    return;
  }
  // Clear only the app-facing raw slot; the system slot persists.
  state->raw_handler = NULL;
  state->raw_context = NULL;
  sys_touch_set_raw_subscribed(false);
  prv_update_subscription(state);
}

bool touch_service_is_enabled(void) {
  return sys_touch_service_is_enabled();
}

void app_touch_navigation_enable(bool enable) {
  sys_app_touch_navigation_enable(enable);
}

void touch_service_state_init(TouchServiceState *state) {
  *state = (TouchServiceState){ 0 };
}
