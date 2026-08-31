/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app_wakeup.h"

#include "event_service_client.h"
#include "process_state/app_state/app_state.h"
#include "syscall/syscall.h"
#include "pbl/services/wakeup.h"
#include "kernel/events.h"

static void do_handle(PebbleEvent *e, void *context) {
  WakeupHandler wakeup_handler = app_state_get_wakeup_handler();
  if (wakeup_handler != NULL) {
    wakeup_handler(e->wakeup.wakeup_info.wakeup_id, e->wakeup.wakeup_info.wakeup_reason);
  }
}

void app_wakeup_service_subscribe(WakeupHandler handler) {
  if (handler) {
    app_state_set_wakeup_handler(handler);
    // Subscribe to PEBBLE_WAKEUP_EVENT
    EventServiceInfo *wakeup_event_info = app_state_get_wakeup_event_info();
    // NOTE: the individual fields of wakeup_event_info are assigned to
    // instead of writing
    //     *wakeup_event_info = (EventServiceInfo) { ... }
    // as the latter would zero out the ListNode embedded in the struct.
    // Doing so would corrupt the events list if the event was already
    // subscribed to (the app calls app_wakeup_service_subscribe twice).
    wakeup_event_info->type = PEBBLE_WAKEUP_EVENT;
    wakeup_event_info->handler = do_handle;
    event_service_client_subscribe(wakeup_event_info);
  }
}

WakeupId app_wakeup_schedule(time_t timestamp, int32_t cookie, bool notify_if_missed) {
  return sys_wakeup_schedule(timestamp, cookie, notify_if_missed);
}

void app_wakeup_cancel(WakeupId wakeup_id) {
  sys_wakeup_delete(wakeup_id);
}

void app_wakeup_cancel_all(void) {
  sys_wakeup_cancel_all_for_app();
}

bool app_wakeup_get_launch_event(WakeupId *wakeup_id, int32_t *cookie) {
  WakeupInfo wakeup_info;
  sys_process_get_wakeup_info(&wakeup_info);
  //If the id is invalid, return false
  if (wakeup_info.wakeup_id <= 0) {
    return false;
  }

  *wakeup_id = wakeup_info.wakeup_id;
  *cookie = wakeup_info.wakeup_reason;
  return true;
}

bool app_wakeup_query(WakeupId wakeup_id, time_t *timestamp) {
  time_t scheduled_time = sys_wakeup_query(wakeup_id);
  if (timestamp != NULL) {
    *timestamp = scheduled_time;
  }

  return (scheduled_time >= 0) ? true : false;
}
