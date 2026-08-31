/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/idle_watchdog.h"

#include "applib/event_service_client.h"
#include "comm/ble/gap_le_connection.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/system_task.h"
#include "system/reboot_reason.h"
#include "kernel/util/standby.h"

#define PRF_IDLE_TIMEOUT_MINUTES 10
static RegularTimerInfo s_is_idle_timer;
static EventServiceInfo s_bt_event_info;
static EventServiceInfo s_battery_event_info;
static EventServiceInfo s_button_event_info;
static bool s_running;

static void prv_handle_watchdog_timeout_cb(void *not_used) {
  GAPLEConnection *le_connection = gap_le_connection_any();

  if (le_connection) {
    // We are still connected, don't shut down
    return;
  }

  BatteryChargeState current_state = battery_get_charge_state();
  if (current_state.is_plugged) {
    // We are plugged in, don't shut down
    return;
  }

  enter_standby(RebootReasonCode_PrfIdle);
}

static void prv_handle_watchdog_timeout(void *not_used) {
  system_task_add_callback(prv_handle_watchdog_timeout_cb, NULL);
}

static void prv_watchdog_feed(PebbleEvent *e, void *context) {
  if (__atomic_load_n(&s_running, __ATOMIC_RELAXED)) {
    regular_timer_add_multiminute_callback(&s_is_idle_timer, PRF_IDLE_TIMEOUT_MINUTES);
  }
}

void prf_idle_watchdog_init(void) {
  s_bt_event_info = (EventServiceInfo) {
    .type = PEBBLE_BT_CONNECTION_EVENT,
    .handler = prv_watchdog_feed,
  };
  event_service_client_subscribe(&s_bt_event_info);

  s_battery_event_info = (EventServiceInfo) {
    .type = PEBBLE_BATTERY_CONNECTION_EVENT,
    .handler = prv_watchdog_feed,
  };
  event_service_client_subscribe(&s_battery_event_info);

  s_button_event_info = (EventServiceInfo) {
    .type = PEBBLE_BUTTON_DOWN_EVENT,
    .handler = prv_watchdog_feed,
  };
  event_service_client_subscribe(&s_button_event_info);
}

void prf_idle_watchdog_start(void) {
  s_is_idle_timer = (RegularTimerInfo) {
    .cb = prv_handle_watchdog_timeout,
  };
  regular_timer_add_multiminute_callback(&s_is_idle_timer, PRF_IDLE_TIMEOUT_MINUTES);
  __atomic_store_n(&s_running, true, __ATOMIC_RELAXED);
}

void prf_idle_watchdog_stop(void) {
  __atomic_store_n(&s_running, false, __ATOMIC_RELAXED);
  regular_timer_remove_callback(&s_is_idle_timer);
}
