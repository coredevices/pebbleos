/* SPDX-FileCopyrightText: 2025 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gh3x2x_tuning_service.h"

#include <FreeRTOS.h>
#include <bluetooth/init.h>
#include <comm/bt_lock.h>
#include <host/ble_hs.h>
#include <host/ble_hs_stop.h>
#include <host/util/util.h>
#include <kernel/pebble_tasks.h>
#include <nimble/nimble_port.h>
#include <pbl/os/tick.h>
#include <semphr.h>
#include <services/dis/ble_svc_dis.h>
#include <services/bas/ble_svc_bas.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <pbl/logging/logging.h>
#include <system/passert.h>

#include "nimble_store.h"

PBL_LOG_MODULE_DEFINE(bt, CONFIG_BT_LOG_LEVEL);

static const uint32_t s_bt_stack_start_stop_timeout_ms = 10000;

extern void pebble_pairing_service_init(void);
extern void ppog_reversed_service_init(void);
extern void nimble_discover_init(void);
extern void nimble_gattc_op_queue_init(void);

#if NIMBLE_CFG_CONTROLLER
static TaskHandle_t s_ll_task_handle;
#endif
static TaskHandle_t s_host_task_handle;
static SemaphoreHandle_t s_host_started;
static SemaphoreHandle_t s_host_stopped;
static DisInfo s_dis_info;
static struct ble_hs_stop_listener s_listener;

typedef enum {
  DriverStateStopped,
  DriverStateStarting,
  DriverStateStarted,
  DriverStateStopping,
} DriverState;

// Host start/stop run to completion or crash on timeout, so this stays in sync
// with the vendor's ble_hs_enabled_state (Stopped<->OFF, Started<->ON).
static DriverState s_driver_state = DriverStateStopped;

static void prv_sync_cb(void) {
  PBL_LOG_DBG("NimBLE host synchronized");
  xSemaphoreGive(s_host_started);
  bt_driver_handle_host_resynced();
}

static void prv_reset_cb(int reason) {
  PBL_LOG_WRN("NimBLE host reset (reason: 0x%04x)", (uint16_t)reason);
#ifdef CONFIG_SOC_SF32LB52
  // Controller stopped answering HCI. Crash so the coredump captures LCPU RAM
  // (core_dump wakes the LCPU itself); the reboot cold-recovers the controller.
  PBL_CROAK("NimBLE host reset 0x%04x; captured LCPU RAM", (uint16_t)reason);
#endif
}

static void prv_host_task_main(void *unused) {
  PBL_LOG_DBG("NimBLE host task started");

  ble_hs_cfg.sync_cb = prv_sync_cb;
  ble_hs_cfg.reset_cb = prv_reset_cb;

  nimble_port_run();
}

static void prv_ble_hs_stop_cb(int status, void *arg) { xSemaphoreGive(s_host_stopped); }

// ----------------------------------------------------------------------------------------
void bt_driver_init(void) {
  bt_lock_init();

  s_host_started = xSemaphoreCreateBinary();
  s_host_stopped = xSemaphoreCreateBinary();

  nimble_discover_init();
  nimble_gattc_op_queue_init();

  nimble_port_init();
  nimble_store_init();

  TaskParameters_t host_task_params = {
      .pvTaskCode = prv_host_task_main,
      .pcName = "NimbleHost",
      .usStackDepth = 5000 / sizeof(StackType_t),
      .uxPriority = (configMAX_PRIORITIES - 2) | portPRIVILEGE_BIT,
      .puxStackBuffer = NULL,
  };

  pebble_task_create(PebbleTask_BTHost, &host_task_params, &s_host_task_handle);
  PBL_ASSERTN(s_host_task_handle);

#if NIMBLE_CFG_CONTROLLER
  TaskParameters_t ll_task_params = {
      .pvTaskCode = nimble_port_ll_task_func,
      .pcName = "NimbleLL",
      .usStackDepth = (configMINIMAL_STACK_SIZE + 600) / sizeof(StackType_t),
      .uxPriority = (configMAX_PRIORITIES - 1) | portPRIVILEGE_BIT,
      .puxStackBuffer = NULL,
  };

  pebble_task_create(PebbleTask_BTController, &ll_task_params, &s_ll_task_handle);
  PBL_ASSERTN(s_ll_task_handle);
#endif
}

bool bt_driver_start(BTDriverConfig *config) {
  int rc;
  BaseType_t f_rc;

  if (s_driver_state == DriverStateStarted) {
    PBL_LOG_WRN("Driver already started; skipping start");
    return true;
  }

  if (s_driver_state != DriverStateStopped) {
    PBL_LOG_ERR("Unexpected driver state %u; refusing to start",
                  (unsigned)s_driver_state);
    return false;
  }

  s_driver_state = DriverStateStarting;
  // Drain a stale host_started signal (e.g. from an autonomous host re-sync)
  // so we wait for *this* start to sync.
  (void)xSemaphoreTake(s_host_started, 0);

  s_dis_info = config->dis_info;
  ble_svc_dis_model_number_set(s_dis_info.model_number);
  ble_svc_dis_serial_number_set(s_dis_info.serial_number);
  ble_svc_dis_firmware_revision_set(s_dis_info.fw_revision);
  ble_svc_dis_software_revision_set(s_dis_info.sw_revision);
  ble_svc_dis_manufacturer_name_set(s_dis_info.manufacturer);

  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_svc_dis_init();
  pebble_pairing_service_init();
  ble_svc_bas_init();
  ppog_reversed_service_init();

#ifdef CONFIG_GH3X2X_TUNING_SERVICE_ENABLED
  gh3x2x_tuning_service_init();
#endif

  ble_hs_sched_start();
  f_rc = xSemaphoreTake(s_host_started, milliseconds_to_ticks(s_bt_stack_start_stop_timeout_ms));
  if (f_rc != pdTRUE) {
    // core_dump wakes the LCPU itself, so its RAM is captured here too.
    PBL_CROAK("NimBLE host start timed out");
  }

  rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    PBL_LOG_ERR("Failed to ensure address: 0x%04x", (uint16_t)rc);
    goto err;
  }

  s_driver_state = DriverStateStarted;
  return true;

err:
  s_driver_state = DriverStateStopping;
  (void)xSemaphoreTake(s_host_stopped, 0);
  rc = ble_hs_stop(&s_listener, prv_ble_hs_stop_cb, NULL);
  if (rc == BLE_HS_EALREADY) {
    s_driver_state = DriverStateStopped;
    return false;
  } else if (rc != 0) {
    PBL_LOG_ERR("Failed to stop NimBLE host after start failure: 0x%04x", (uint16_t)rc);
    return false;
  }

  f_rc = xSemaphoreTake(s_host_stopped, milliseconds_to_ticks(s_bt_stack_start_stop_timeout_ms));
  PBL_ASSERT(f_rc == pdTRUE, "NimBLE host stop timed out after start failure");

  s_driver_state = DriverStateStopped;
  (void)ble_gatts_reset();

  return false;
}

void bt_driver_stop(void) {
  BaseType_t f_rc;

  s_driver_state = DriverStateStopping;
  (void)xSemaphoreTake(s_host_stopped, 0);
  ble_hs_stop(&s_listener, prv_ble_hs_stop_cb, NULL);
  f_rc = xSemaphoreTake(s_host_stopped, milliseconds_to_ticks(s_bt_stack_start_stop_timeout_ms));
  PBL_ASSERT(f_rc == pdTRUE, "NimBLE host stop timed out");
  s_driver_state = DriverStateStopped;

  ble_gatts_reset();

  nimble_store_unload();
}

void bt_driver_power_down_controller_on_boot(void) {}
