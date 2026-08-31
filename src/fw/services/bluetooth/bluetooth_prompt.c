/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "comm/ble/gap_le_connection.h"
#include "comm/bt_lock.h"

#include "console/prompt.h"
#include "pbl/services/bluetooth/bluetooth_ctl.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/bluetooth/local_id.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage.h"
#include "pbl/util/string.h"

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/id.h>

void command_bt_print_mac(void) {
  char addr_hex_str[BT_ADDR_FMT_BUFFER_SIZE_BYTES];
  bt_local_id_copy_address_hex_string(addr_hex_str);
  prompt_send_response(addr_hex_str);
}


//! @param bt_name A custom Bluetooth device name.
void command_bt_set_name(const char *bt_name) {
  bt_local_id_set_device_name(bt_name);
}

void command_bt_prefs_wipe(void) {
  bt_persistent_storage_delete_all_pairings();
}

void command_bt_sprf_nuke(void) {
  shared_prf_storage_wipe_all();
#ifdef CONFIG_RECOVERY_FW
  // Reset system to get caches (in s_intents, s_connections and controller-side caches) in sync.
  extern void factory_reset_set_reason_and_reset(void);
  factory_reset_set_reason_and_reset();
#endif
}

#ifdef CONFIG_RECOVERY_FW
void command_bt_status(void) {
  char buffer[64];

  prompt_send_response_fmt(buffer, sizeof(buffer), "Alive: %s",
                           bt_ctl_is_bluetooth_running() ? "yes" : "no");

  const char *prefix = "BT Chip Info: ";
  size_t prefix_length = strlen(prefix);
  strncpy(buffer, prefix, sizeof(buffer));
  bt_driver_id_copy_chip_info_string(buffer + prefix_length,
                                     sizeof(buffer) - prefix_length);
  prompt_send_response(buffer);

  char name[BT_DEVICE_NAME_BUFFER_SIZE];
  bt_lock();
  bool connected = false;
  GAPLEConnection *connection = gap_le_connection_any();
  if (connection) {
    const char *device_name = connection->device_name ?: "<Unknown>";
    strncpy(name, device_name, BT_DEVICE_NAME_BUFFER_SIZE);
    name[BT_DEVICE_NAME_BUFFER_SIZE - 1] = '\0';
    connected = true;
  }
  bt_unlock();

  prompt_send_response_fmt(buffer, sizeof(buffer), "Connected: %s", connected ? "yes" : "no");
  if (connected) {
    prompt_send_response_fmt(buffer, sizeof(buffer), "Device: %s", name);
  }
}
#endif // CONFIG_RECOVERY_FW
