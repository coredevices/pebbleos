/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_launch.h"

#include "comm/ble/gatt_client_operations.h"
#include "pbl/services/comm_session/session.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

//! See https://pebbletechnology.atlassian.net/wiki/display/DEV/Pebble+GATT+Services

// -------------------------------------------------------------------------------------------------
// Static variables

static BLECharacteristic s_app_launch_characteristic = BLE_CHARACTERISTIC_INVALID;

// -------------------------------------------------------------------------------------------------

void app_launch_handle_service_discovered(BLECharacteristic *characteristics) {
  PBL_ASSERTN(characteristics);

  if (s_app_launch_characteristic != BLE_CHARACTERISTIC_INVALID) {
    PBL_LOG_WRN("Multiple app launch services!? Will use most recent one.");
  }

  s_app_launch_characteristic = *characteristics;

  // If there was no system session, try launching the Pebble app:
  if (!comm_session_get_system_session()) {
    app_launch_trigger();
  }
}

void app_launch_invalidate_all_references(void) {
  s_app_launch_characteristic = BLE_CHARACTERISTIC_INVALID;
}

void app_launch_handle_service_removed(
    BLECharacteristic *characteristics, uint8_t num_characteristics) {
  app_launch_invalidate_all_references();
}

// -------------------------------------------------------------------------------------------------

bool app_launch_can_handle_characteristic(BLECharacteristic characteristic) {
  return (characteristic == s_app_launch_characteristic);
}

// -------------------------------------------------------------------------------------------------

void app_launch_handle_disconnection(void) {
  s_app_launch_characteristic = BLE_CHARACTERISTIC_INVALID;
}

// -------------------------------------------------------------------------------------------------

void app_launch_trigger(void) {
  if (s_app_launch_characteristic == BLE_CHARACTERISTIC_INVALID) {
    return;
  }
  BTErrno err = gatt_client_op_read(s_app_launch_characteristic, GAPLEClientKernel);
  if (err != BTErrnoOK) {
    PBL_LOG_ERR("App relaunch failed: %u", err);
  }
}
