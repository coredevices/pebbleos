/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/gatt.h>
#include <bluetooth/bluetooth_types.h>

// -------------------------------------------------------------------------------------------------
// Gatt Client Discovery API calls

BTErrno bt_driver_gatt_start_discovery_range(
    const GAPLEConnection *connection, const ATTHandleRange *data) {
  return 0;
}

BTErrno bt_driver_gatt_stop_discovery(GAPLEConnection *connection) {
  return 0;
}

void bt_driver_gatt_handle_discovery_abandoned(void) {
}
