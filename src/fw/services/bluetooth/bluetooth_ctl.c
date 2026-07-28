/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/bluetooth_ctl.h"

#include <bluetooth/init.h>
#include <string.h>

#include "comm/ble/gap_le.h"
#include "comm/ble/gatt_client_subscriptions.h"
#include "console/dbgserial.h"
#include <pbl/drivers/clocksource.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/bluetooth/ble_bas.h"
#include "pbl/services/bluetooth/ble_hid.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/bluetooth/dis.h"
#include "pbl/services/bluetooth/local_addr.h"
#include "pbl/services/bluetooth/local_id.h"
#include "pbl/services/bluetooth/pairability.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/system_task.h"
#include "pbl/services/bluetooth/ble_hrm.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DEFINE(service_bluetooth, CONFIG_SERVICE_BLUETOOTH_LOG_LEVEL);

static bool s_comm_initialized = false;
static bool s_comm_airplane_mode_on = false;
static bool s_comm_enabled = false;
static bool s_comm_is_running = false;
static bool s_comm_state_change_eval_is_scheduled;
static BtCtlModeOverride s_comm_override = BtCtlModeOverrideNone;
static PebbleMutex *s_comm_state_change_mutex;

bool bt_ctl_is_airplane_mode_on(void) { return s_comm_airplane_mode_on; }

bool bt_ctl_is_bluetooth_active(void) {
  if (s_comm_enabled) {
    if (s_comm_override == BtCtlModeOverrideRun) {
      return true;
    } else if (s_comm_override == BtCtlModeOverrideNone && !s_comm_airplane_mode_on) {
      return true;
    }
  }
  return false;
}

bool bt_ctl_is_bluetooth_running(void) { return s_comm_is_running; }

static void prv_put_disconnection_event(void) {
  PebbleEvent event = (PebbleEvent){.type = PEBBLE_BT_CONNECTION_EVENT,
                                    .bluetooth.connection = {
                                        .is_ble = true,
                                        .state = PebbleBluetoothConnectionEventStateDisconnected,
                                    }};
  PBL_LOG_DBG("New BT Conn change event, We are now disconnected");
  event_put(&event);
}

static void prv_comm_start(void) {
  if (s_comm_is_running) {
    return;
  }
  // Heap allocated to reduce stack usage
  BTDriverConfig *config = kernel_zalloc_check(sizeof(BTDriverConfig));
  dis_get_info(&config->dis_info);
#if defined(CONFIG_HRM) && !defined(CONFIG_RECOVERY_FW)
  config->is_hrm_supported_and_enabled = ble_hrm_is_supported_and_enabled();
  PBL_LOG_INFO("BLE HRM sharing prefs: is_enabled=%u",
          config->is_hrm_supported_and_enabled);
#endif
  // Register existing bondings before bringing the connection up: NimBLE
  // restores them before the link is established. The other backends use
  // no-op bonding handlers, so doing it early is harmless for them too.
  bt_persistent_storage_register_existing_ble_bondings();

  // Before the driver, not after: bt_driver_start() is where the HID GATT
  // service becomes subscribable, and a subscribe event landing on the host
  // task before ble_hid_init() has made its lock would race the press path.
  // Nothing in here touches the driver beyond asking whether the backend
  // supports the service at all, which is a per-backend constant.
  ble_hid_init();

  s_comm_is_running = bt_driver_start(config);
  kernel_free(config);

  if (s_comm_is_running) {
    bt_local_addr_init();
    gap_le_init();
    bt_local_id_configure_driver();
#if defined(CONFIG_HRM) && !defined(CONFIG_RECOVERY_FW)
    ble_hrm_init();
#endif
    ble_bas_init();
    bt_pairability_init();
  } else {
    PBL_LOG_ERR("BT driver failed to start!");
    // FIXME: PBL-36163 -- handle this better
  }
}

static void prv_comm_stop(void) {
  if (!s_comm_is_running) {
    return;
  }
  ble_bas_deinit();
  ble_hid_deinit();
#if defined(CONFIG_HRM) && !defined(CONFIG_RECOVERY_FW)
  ble_hrm_deinit();
#endif
  gap_le_deinit();

  // Should be the last thing to happen that touches the Bluetooth controller directly
  bt_driver_stop();
  s_comm_is_running = false;

  // This is a legacy event used to update the Settings app.
  prv_put_disconnection_event();
}

static void prv_send_state_change_event(void) {
  PBL_LOG_DBG("----> Sending a BT state event");
  PebbleEvent event = {
      .type = PEBBLE_BT_STATE_EVENT,
      .bluetooth =
          {
              .state =
                  {
                      .airplane = s_comm_airplane_mode_on,
                      .enabled = s_comm_enabled,
                      .override = s_comm_override,
                  },
          },
  };
  event_put(&event);
  if (s_comm_airplane_mode_on) {
    PBL_ANALYTICS_TIMER_STOP(connectivity_connected_time_ms);
    PBL_ANALYTICS_TIMER_STOP(connectivity_expected_time_ms);
  } else {
    PBL_ANALYTICS_TIMER_START(connectivity_expected_time_ms);
  }
}

static void prv_comm_state_change(void *context) {
  static bool s_first_run = true;
  mutex_lock(s_comm_state_change_mutex);
  s_comm_state_change_eval_is_scheduled = false;
  bool is_active_mode = bt_ctl_is_bluetooth_active();
  if (is_active_mode != s_comm_is_running) {
    if (is_active_mode) {
      prv_comm_start();
    } else {
      prv_comm_stop();
    }
    // Only send event if state changed successfully:
    if (is_active_mode == s_comm_is_running) {
      prv_send_state_change_event();
    }
  } else if (!s_comm_is_running && s_first_run) {
    PBL_LOG_DBG("Shutting down the BT stack on boot");
    bt_driver_power_down_controller_on_boot();
  }

  s_first_run = false;
  mutex_unlock(s_comm_state_change_mutex);
}

void bt_ctl_set_enabled(bool enabled) {
  if (!s_comm_initialized) {
    PBL_LOG_ERR("Error: Bluetooth isn't initialized yet");
    return;
  }
  mutex_lock(s_comm_state_change_mutex);
  s_comm_enabled = enabled;
  mutex_unlock(s_comm_state_change_mutex);
  prv_comm_state_change(NULL);
}

void bt_ctl_set_override_mode(BtCtlModeOverride override) {
  if (!s_comm_initialized) {
    PBL_LOG_ERR("Error: Bluetooth isn't initialized yet");
    return;
  }
  mutex_lock(s_comm_state_change_mutex);
  s_comm_override = override;
  mutex_unlock(s_comm_state_change_mutex);
  prv_comm_state_change(NULL);
}

static void prv_track_quick_airplane_mode_toggles(bool is_airplane_mode_currently_on) {
  // Track when coming out of airplane mode and we've gone into airplane mode less than 30 secs ago:
  static RtcTicks s_airplane_mode_last_toggle_ticks;
  const RtcTicks now_ticks = rtc_get_ticks();
  const uint64_t max_interval_secs = 30;
  if (((now_ticks - s_airplane_mode_last_toggle_ticks) < (max_interval_secs * RTC_TICKS_HZ)) &&
      is_airplane_mode_currently_on) {
    PBL_LOG_DBG("Quick airplane mode toggle detected!");
  }
  s_airplane_mode_last_toggle_ticks = now_ticks;
}

void bt_ctl_set_airplane_mode_async(bool enabled) {
  if (!s_comm_initialized) {
    PBL_LOG_ERR("Error: Bluetooth isn't initialized yet");
    return;
  }
  mutex_lock(s_comm_state_change_mutex);
  prv_track_quick_airplane_mode_toggles(!enabled);
  bt_persistent_storage_set_airplane_mode_enabled(enabled);
  s_comm_airplane_mode_on = enabled;
  bool should_schedule_eval = false;
  if (!s_comm_state_change_eval_is_scheduled) {
    should_schedule_eval = true;
    s_comm_state_change_eval_is_scheduled = true;
  }
  mutex_unlock(s_comm_state_change_mutex);
  if (should_schedule_eval) {
    system_task_add_callback(prv_comm_state_change, NULL);
  }
}

void bt_ctl_init(void) {
  s_comm_state_change_mutex = mutex_create();

  s_comm_airplane_mode_on = bt_persistent_storage_get_airplane_mode_enabled();
  s_comm_initialized = true;

  gatt_client_subscription_boot();
}

static void prv_bt_ctl_reset_bluetooth_callback(void *context) {
  PBL_LOG_DBG("Resetting Bluetooth");
  mutex_lock(s_comm_state_change_mutex);

  bool was_already_running = s_comm_is_running;

  prv_comm_stop();
  prv_comm_start();

  // It's possible a reset was triggered because the stack failed to boot up
  // correctly in which case we have never generated an event about the stack
  // booting up. Don't bother sending events if we are just returning the stack
  // to the state it is already in
  if (!was_already_running && s_comm_is_running) {
    prv_send_state_change_event();
  }
  mutex_unlock(s_comm_state_change_mutex);
}

void bt_ctl_reset_bluetooth(void) {
  if (bt_ctl_is_bluetooth_active()) {
    system_task_add_callback(prv_bt_ctl_reset_bluetooth_callback, NULL);
  } else {
    PBL_LOG_DBG("Bluetooth is disabled, reset aborted");
  }
}

void command_bt_airplane_mode(const char *new_mode) {
  // as tests run using command_bt_airplane_mode, will retain nomenclature
  // but work as override mode change
  BtCtlModeOverride override = BtCtlModeOverrideStop;
  if (strcmp(new_mode, "exit") == 0) {
    override = BtCtlModeOverrideNone;
  }
  bt_ctl_set_override_mode(override);
  bool new_state = bt_ctl_is_bluetooth_active();
  if (!new_state) {
    dbgserial_putstr("Entered airplane mode");
  } else {
    dbgserial_putstr("Left airplane mode");
  }
}
