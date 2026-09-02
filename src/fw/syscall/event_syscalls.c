/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/kernel_applib_state.h"
#include "process_management/app_manager.h"
#include "process_management/worker_manager.h"
#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"
#include "pbl/services/event_service.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

static void prv_put_event_from_process(PebbleTask task, PebbleEvent *event) {
  if (!event_try_put_from_process(task, event)) {
    PBL_LOG_WRN("%s: From app queue is full! Dropped %p! Killing App",
        (task == PebbleTask_App ? "App" : "Worker"), event);
    syscall_failed();
  }
}

// Event types that an unprivileged app or worker is permitted to post directly via
// sys_send_pebble_event_to_kernel. Anything else (in particular events that carry a
// kernel-invoked callback pointer, like PEBBLE_CALLBACK_EVENT) is forbidden — the
// kernel would otherwise jump to an app-controlled address with kernel privileges.
static bool prv_event_type_allowed_from_user(PebbleEventType type) {
  switch (type) {
    case PEBBLE_PROCESS_KILL_EVENT:
    case PEBBLE_RENDER_READY_EVENT:
    case PEBBLE_DICTATION_EVENT:
    case PEBBLE_HEALTH_SERVICE_EVENT:
    case PEBBLE_PLUGIN_SERVICE_EVENT:
      return true;
    default:
      return false;
  }
}

DEFINE_SYSCALL(void, sys_send_pebble_event_to_kernel, PebbleEvent* event) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(event, sizeof(*event));
    if (!prv_event_type_allowed_from_user(event->type)) {
      PBL_LOG_ERR("Rejecting event type %u from unprivileged caller", event->type);
      syscall_failed();
    }
  }

  PebbleTask task = pebble_task_get_current();
  if (task == PebbleTask_App || task == PebbleTask_Worker) {
    prv_put_event_from_process(task, event);
  } else {
    event_put(event);
  }
}

DEFINE_SYSCALL(void, sys_current_process_schedule_callback,
               CallbackEventCallback async_cb, void *ctx) {
  // No userspace buffer assertion for ctx needed, because it won't be accessed by the kernel.

  PebbleEvent event = {
    .type = PEBBLE_CALLBACK_EVENT,
    .callback = {
      .callback = async_cb,
      .data = ctx,
    },
  };
  const PebbleTask task = pebble_task_get_current();
  PBL_ASSERTN(task == PebbleTask_App || task == PebbleTask_Worker);
  process_manager_send_event_to_process(task, &event);
}

DEFINE_SYSCALL(uint32_t, sys_process_events_waiting, PebbleTask task) {
  // process_manager_process_events_waiting -> prv_get_context_for_task
  // PBL_ASSERTN()s for anything other than App or Worker. Without this guard
  // an unprivileged app can pass any other PebbleTask value and panic the
  // kernel from a syscall. Pin the caller to its own task instead.
  if (PRIVILEGE_WAS_ELEVATED) {
    task = pebble_task_get_current();
    if (task != PebbleTask_App && task != PebbleTask_Worker) {
      syscall_failed();
    }
  }
  return process_manager_process_events_waiting(task);
}

// The handler structure is supplied by the app and its embedded ListNode pointers
// flow into kernel-mode list operations (notably list_remove on unsubscribe). Without
// validating the prev/next pointers, an app can craft them to point into kernel
// memory and use list_remove as a 4-byte arbitrary-write primitive (`*a = b; *b = a`).
static void prv_assert_list_node_in_userspace(const ListNode *node) {
  if (node != NULL) {
    syscall_assert_userspace_buffer(node, sizeof(*node));
  }
}

static void prv_assert_handler_list_node_in_userspace(const EventServiceInfo *handler) {
  prv_assert_list_node_in_userspace(handler->list_node.prev);
  prv_assert_list_node_in_userspace(handler->list_node.next);
}

DEFINE_SYSCALL(void, sys_event_service_client_subscribe, EventServiceInfo *handler) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(handler, sizeof(*handler));
    prv_assert_handler_list_node_in_userspace(handler);
  }

  PebbleTask task = pebble_task_get_current();

  // Get info
  struct pbl_msgq *event_queue;
  if (task == PebbleTask_App) {
    event_queue = app_manager_get_task_context()->to_process_event_queue;
  } else if (task == PebbleTask_Worker) {
    event_queue = worker_manager_get_task_context()->to_process_event_queue;
  } else if (task == PebbleTask_KernelMain) {
    // The event service always runs from KernelMain
    event_queue = event_kernel_to_kernel_event_queue();
  } else {
    WTF;
  }

  // Subscribe to the service!
  PebbleEvent event = {
    .type = PEBBLE_SUBSCRIPTION_EVENT,
    .subscription = {
      .subscribe = true,
      .task = task,
      .event_queue = event_queue,
      .event_type = handler->type,
    },
  };
  if (task == PebbleTask_KernelMain) {
    // The client is also KernelMain, just subscribe immediately without putting an event
    event_service_subscribe_from_kernel_main(&event.subscription);
  } else {
    prv_put_event_from_process(task, &event);
  }
}

DEFINE_SYSCALL(void, sys_event_service_client_unsubscribe, EventServiceInfo *state,
                                                           EventServiceInfo *handler) {
  PebbleTask task = pebble_task_get_current();

  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(handler, sizeof(*handler));
    prv_assert_handler_list_node_in_userspace(handler);
    // The user-supplied `state` pointer flows into list_find as the list head; an app
    // could craft it (and the chain it heads) to walk into kernel memory. Re-derive
    // the head from authoritative per-process state instead of trusting the caller.
    if (task == PebbleTask_App) {
      state = app_state_get_event_service_state();
    } else if (task == PebbleTask_Worker) {
      state = worker_state_get_event_service_state();
    } else {
      WTF;
    }
  }

  // Remove from handlers list
  list_remove(&handler->list_node, NULL, NULL);

  if (list_find(&state->list_node, event_service_filter, (void *) handler->type)) {
    // there are other handlers for this task, don't unsubscribe it
    return;
  }
  // Unsubscribe from the service!
  PebbleEvent event = {
    .type = PEBBLE_SUBSCRIPTION_EVENT,
    .subscription = {
      .subscribe = false,
      .task = task,
      .event_type = handler->type,
    },
  };
  prv_put_event_from_process(task, &event);
}
