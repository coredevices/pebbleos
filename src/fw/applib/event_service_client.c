/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/util/list.h"
#include "pbl/services/event_service.h"
#include "kernel/kernel_applib_state.h"

#include "event_service_client.h"
#include "pbl/util/list.h"
#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"

#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"


static EventServiceInfo *prv_get_state(void) {
  PebbleTask task = pebble_task_get_current();
  if (task == PebbleTask_App) {
    return app_state_get_event_service_state();
  } else if (task == PebbleTask_Worker) {
    return worker_state_get_event_service_state();
  } else if (task == PebbleTask_KernelMain) {
    return kernel_applib_get_event_service_state();
  } else {
    WTF;
  }
}

static int event_service_comparator(EventServiceInfo *a, EventServiceInfo *b) {
  return (b->type - a->type);
}

bool event_service_filter(ListNode *node, void *tp) {
  EventServiceInfo *info = (EventServiceInfo *)node;
  uint32_t type = (uint32_t)tp;
  return (info->type == type);
}

static void do_handle(EventServiceInfo *info, PebbleEvent *e) {
  PBL_ASSERTN(info->handler != NULL);
  info->handler(e, info->context);
}

void event_service_client_subscribe(EventServiceInfo *handler) {
  EventServiceInfo *state = prv_get_state();
  ListNode *list = &state->list_node;
  if (list_contains(list, &handler->list_node)) {
    PBL_LOG_DBG("Event service handler already subscribed");
    return;
  }
  // Add to handlers list
  list_sorted_add(list, &handler->list_node, (Comparator)event_service_comparator, true);

  sys_event_service_client_subscribe(handler);
}

void event_service_client_unsubscribe(EventServiceInfo *handler) {
  EventServiceInfo *state = prv_get_state();
  ListNode *list = &state->list_node;
  if (!list_contains(list, &handler->list_node)) {
    PBL_LOG_DBG("Event service handler not subscribed");
    return;
  }
  sys_event_service_client_unsubscribe(state, handler);
}

void event_service_client_handle_event(PebbleEvent *e) {
  EventServiceInfo *state = prv_get_state();
  const uintptr_t type = e->type;
  // find the first callback
  ListNode *handler = list_find(&state->list_node, event_service_filter, (void *) type);
  while (handler) {
    // find the next callback before we call the current one, because the CB may alter the list
    ListNode *next_handler = list_find_next(handler, event_service_filter, false, (void *) type);
    do_handle((EventServiceInfo *)handler, e);
    handler = next_handler;
  }

  sys_event_service_cleanup(e);
}
