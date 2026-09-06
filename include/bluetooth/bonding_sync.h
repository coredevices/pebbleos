/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/attributes.h"

#include <bluetooth/sm_types.h>

//! Packed, because this is serialized for the host-controller protocol.
typedef struct PACKED BleBonding {
  SMPairingInfo pairing_info;
  //! True if the remote device is capable of talking PPoGATT.
  bool is_gateway:1;

  //! True if the local device address should be pinned.
  bool should_pin_address:1;

  //! @note bt_persistent_storage_... uses only 5 bits to store this!
  //! @see BleBondingFlag
  uint8_t flags:5;

  uint8_t rsvd:1;

  //! Valid iff should_pin_address is true
  BTDeviceAddress pinned_address;
} BleBonding;

typedef struct PACKED BleCCCD {
  //! The peer device.
  BTDeviceInternal peer;
  //! The characteristic value handle that this CCCD is associated with.
  uint16_t chr_val_handle;
  //! Flags for the CCCD.
  uint16_t flags;
  //! True if the value has changed.
  bool value_changed:1;
} BleCCCD;

//! Called by the FW after starting the Bluetooth stack to register existing bondings.
//! @note When the Bluetooth is torn down, there won't be any "remove" calls. If needed, the BT
//! driver lib should clean up itself in bt_driver_stop().
void bt_driver_handle_host_added_bonding(const BleBonding *bonding);

//! Called by the FW when a bonding is removed (i.e. user "Forgot" a bonding from Settings).
void bt_driver_handle_host_removed_bonding(const BleBonding *bonding);

//! Called by the FW when a CCCD entry is added.
void bt_driver_handle_host_added_cccd(const BleCCCD *cccd);

//! Called by the FW when a CCCD entry is removed.
void bt_driver_handle_host_removed_cccd(const BleCCCD *cccd);

//! Called by the BT driver after successfully pairing a new device.
//! @param bonding The newly created bonding.
//! @param addr The address that is used to refer to the connection. This is used to associate
//! the bonding with the GAPLEConnection.
extern void bt_driver_cb_handle_create_bonding(const BleBonding *bonding,
                                               const BTDeviceAddress *addr);
