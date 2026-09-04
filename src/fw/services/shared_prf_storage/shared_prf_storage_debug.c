/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "console/dbgserial.h"
#include "console/prompt.h"
#include <pbl/logging/logging.h>
#include "pbl/util/string.h"

#include "pbl/services/shared_prf_storage/shared_prf_storage.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage_debug.h"


#include <bluetooth/bluetooth_types.h>
#include <bluetooth/sm_types.h>
#include <pbl/btutil/sm_util.h>

void shared_prf_storage_dump_contents(void) {
  prompt_send_response("---Shared PRF Contents---\n------------------------\n");

  char buf[DISPLAY_BUF_LEN];
  SMPairingInfo pairing_info;
  char name[BT_DEVICE_NAME_BUFFER_SIZE];
  bool requires_address_pinning;
  uint8_t flags;
  if (shared_prf_storage_get_ble_pairing_data(&pairing_info, name, &requires_address_pinning,
                                              &flags)) {
    bluetooth_persistent_storage_debug_dump_ble_pairing_info(&buf[0], &pairing_info);
    prompt_send_response_fmt(buf, sizeof(buf),
                             "Req addr pin: %u, flags: %x, BLE Dev Name: %s",
                             requires_address_pinning, flags, name);
  } else {
    prompt_send_response("No BLE Data");
  }

  SM128BitKey keys[SMRootKeyTypeNum];
  if (shared_prf_storage_get_root_key(SMRootKeyTypeEncryption, &keys[SMRootKeyTypeEncryption]) &&
      shared_prf_storage_get_root_key(SMRootKeyTypeIdentity, &keys[SMRootKeyTypeIdentity])) {
    bluetooth_persistent_storage_debug_dump_root_keys(&keys[SMRootKeyTypeIdentity],
                                                      &keys[SMRootKeyTypeEncryption]);
  } else {
    prompt_send_response("Missing IRK and/or ERK root key(s)!");
  }

  BTDeviceAddress addr;

  if (shared_prf_storage_get_ble_pinned_address(&addr)) {
    prompt_send_response_fmt(buf, DISPLAY_BUF_LEN, "\nPinned address: "BT_DEVICE_ADDRESS_FMT,
                             BT_DEVICE_ADDRESS_XPLODE_PTR(&addr));
  }

  if (shared_prf_storage_get_local_device_name(name, BT_DEVICE_NAME_BUFFER_SIZE)) {
    prompt_send_response_fmt(buf, BT_DEVICE_NAME_BUFFER_SIZE, "Local device name: %s", name);
  } else {
    prompt_send_response("No Device Name");
  }

  prompt_send_response_fmt(buf, BT_DEVICE_NAME_BUFFER_SIZE, "Started Complete: %s",
                       bool_to_str(shared_prf_storage_get_getting_started_complete()));
}
