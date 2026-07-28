/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"

#include "comm/ble/gap_le_connect.h"
#include "comm/ble/gap_le_slave_reconnect.h"

#include "comm/bt_lock.h"

#include "pbl/services/bluetooth/pairability.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage.h"

#include "comm/ble/kernel_le_client/kernel_le_client.h"

#include <pbl/logging/logging.h>

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/bonding_sync.h>
#include <pbl/btutil/bt_device.h>
#include <pbl/btutil/sm_util.h>

#include <string.h>

PBL_LOG_MODULE_DECLARE(service_bluetooth, CONFIG_SERVICE_BLUETOOTH_LOG_LEVEL);


//! This is just an interface for the shared PRF storage


//! These don't matter at all
#define BLE_BONDING_ID (0)

#define BT_CCCD_ID_MIN 0x80U

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Pairing Info

static void prv_call_ble_bonding_change_handlers(BTBondingID bonding, BtPersistBondingOp op) {
  gap_le_connect_handle_bonding_change(bonding, op);
  kernel_le_client_handle_bonding_change(bonding, op);
  bt_pairability_update_due_to_bonding_change();
}

static BTBondingID prv_bt_persistent_storage_store_ble_pairing(
    const SMPairingInfo *new_pairing_info, bool is_gateway, bool requires_address_pinning,
    uint8_t flags, const char *device_name, BtPersistBondingOp op) {
  if (new_pairing_info && is_gateway) {
    PBL_LOG_INFO("Storing BLE pairing: addr=" BT_DEVICE_ADDRESS_FMT " random=%u",
                 BT_DEVICE_ADDRESS_XPLODE(new_pairing_info->identity.address),
                 new_pairing_info->identity.is_random_address);
    PBL_LOG_INFO("Storing BLE pairing: irk=%u remote_enc=%u local_enc=%u",
                 new_pairing_info->is_remote_identity_info_valid,
                 new_pairing_info->is_remote_encryption_info_valid,
                 new_pairing_info->is_local_encryption_info_valid);
    shared_prf_storage_store_ble_pairing_data(new_pairing_info, device_name,
                                              requires_address_pinning,
                                              flags);
    prv_call_ble_bonding_change_handlers(BLE_BONDING_ID, op);
    return BLE_BONDING_ID;
  }

  PBL_LOG_WRN("BLE pairing not stored (gateway=%u)", is_gateway);
  return BT_BONDING_ID_INVALID;
}

bool bt_persistent_storage_set_ble_pinned_address(const BTDeviceAddress *addr) {
  shared_prf_storage_set_ble_pinned_address(addr);
  return true;
}

bool bt_persistent_storage_has_pinned_ble_pairings(void) {
  bool requires_address_pinning_out = false;
  shared_prf_storage_get_ble_pairing_data(NULL, NULL, &requires_address_pinning_out, NULL);
  return requires_address_pinning_out;
}

bool bt_persistent_storage_get_ble_pinned_address(BTDeviceAddress *address_out) {
  return shared_prf_storage_get_ble_pinned_address(address_out);
}

BTBondingID bt_persistent_storage_store_ble_pairing(const SMPairingInfo *new_pairing_info,
                                                    bool is_gateway, const char *device_name,
                                                    bool requires_address_pinning,
                                                    uint8_t flags) {
  // We only have one slot in PRF and all pairing info (except the device
  // name) will arrive in one-shot so anytime this routine gets called it
  // means we have 'added' a new pairing

  bool is_updating_existing = false;
  SMPairingInfo existing_pairing_info;
  if (shared_prf_storage_get_ble_pairing_data(&existing_pairing_info, NULL, NULL, NULL)) {
    if (sm_is_pairing_info_equal_identity(new_pairing_info, &existing_pairing_info)) {
      // Treat re-pairing an existing device as an "update" instead of deletion+addition,
      // because there is only one bonding ID that gets re-used, a deletion would otherwise cause a
      // disconnection to happen. See PBL-24737.
      PBL_LOG_INFO("Re-pairing previously paired LE device");
      is_updating_existing = true;
    } else {
      // Since we only have one slot, this means we are about to delete what was
      // already there so handle the deletion if a valid pairing was stored
      prv_call_ble_bonding_change_handlers(BLE_BONDING_ID, BtPersistBondingOpWillDelete);
    }
  }

  BtPersistBondingOp pairing_op =
      is_updating_existing ? BtPersistBondingOpDidChange : BtPersistBondingOpDidAdd;
  return (prv_bt_persistent_storage_store_ble_pairing(new_pairing_info, is_gateway,
                                               requires_address_pinning,
                                               flags, device_name, pairing_op));
}

bool bt_persistent_storage_update_ble_device_name(BTBondingID bonding, const char *device_name) {
  // A device name has come in, update the name of our currently paired device
  SMPairingInfo data = {};
  char existing_name[BT_DEVICE_NAME_BUFFER_SIZE] = {};
  bool requires_address_pinning = false;
  uint8_t flags = 0;
  if (!shared_prf_storage_get_ble_pairing_data(&data, existing_name, &requires_address_pinning,
                                               &flags)) {
    PBL_LOG_ERR("Tried to store device name, but pairing no longer around.");
    return false;
  }

  if (strncmp(existing_name, device_name, BT_DEVICE_NAME_BUFFER_SIZE) == 0) {
    // Unchanged: skip the flash rewrite and bonding change handlers. Returning
    // true means the caller may still emit a name-updated event; that's
    // harmless (the stored name is correct).
    return true;
  }
  // In PRF, only the gateway should get paired, so default to "true":
  return (BT_BONDING_ID_INVALID !=
          prv_bt_persistent_storage_store_ble_pairing(&data, true /* is_gateway */,
                                                      requires_address_pinning, flags,
                                                      device_name, BtPersistBondingOpDidChange));
}

//! @note Kept out of the shared PRF lock on purpose: this reaches into the BT
//! driver's own store lock, and nesting the two would invert the order the store
//! callbacks take them in.
static void prv_remove_pairing_from_bt_driver(const SMPairingInfo *pairing_info) {
  if (!bt_ctl_is_bluetooth_running()) {
    return;
  }
  BleBonding bonding = {
    .is_gateway = true,
    .pairing_info = *pairing_info,
  };
  bt_driver_handle_host_removed_bonding(&bonding);
}

static void prv_remove_ble_bonding_from_bt_driver(void) {
  SMPairingInfo pairing_info;
  if (!shared_prf_storage_get_ble_pairing_data(&pairing_info, NULL, NULL, NULL)) {
    return;
  }
  prv_remove_pairing_from_bt_driver(&pairing_info);
}

void bt_persistent_storage_delete_ble_pairing_by_id(BTBondingID bonding) {
  PBL_LOG_INFO("Deleting stored BLE pairing");
  prv_remove_ble_bonding_from_bt_driver();
  shared_prf_storage_erase_ble_pairing_data();
  prv_call_ble_bonding_change_handlers(bonding, BtPersistBondingOpWillDelete);
}

void bt_persistent_storage_delete_ble_pairing_by_addr(const BTDeviceInternal *device) {
  bt_persistent_storage_delete_ble_pairing_by_id(BLE_BONDING_ID);
}

//! The PRF twin of the normal FW's guarded delete. Re-pairing the same phone
//! reuses its identity address and its IRK and lands in this same single slot,
//! so only the LTK says whether what is stored is still the bonding this delete
//! was aimed at.
//! @note The check and the erase run as one transaction inside shared PRF
//! storage. The stored pairing is the thing being checked, so reading it here
//! and erasing it afterwards would let a re-pair that completes in between have
//! its fresh keys erased by a delete aimed at the pairing it replaced: this runs
//! on KernelBG for a deferred delete, and bt_persistent_storage_store_ble_pairing()
//! runs on KernelMain, which preempts it as soon as it blocks on flash.
bool bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(const BTDeviceInternal *device,
                                                                 const SMPairingInfo *expected) {
  SMPairingInfo erased;
  if (!shared_prf_storage_erase_ble_pairing_data_if_matches(expected, &erased)) {
    PBL_LOG_DBG("Stored BLE pairing no longer holds the keys to delete, skipping");
    return false;
  }

  PBL_LOG_INFO("Deleted stored BLE pairing");
  prv_remove_pairing_from_bt_driver(&erased);
  prv_call_ble_bonding_change_handlers(BLE_BONDING_ID, BtPersistBondingOpWillDelete);
  return true;
}

bool bt_persistent_storage_get_ble_pairing_by_id(BTBondingID bonding,
                                          SMIdentityResolvingKey *IRK_out,
                                          BTDeviceInternal *device_out,
                                          char *name_out) {
  SMPairingInfo data;
  char name[BT_DEVICE_NAME_BUFFER_SIZE];
  if (!shared_prf_storage_get_ble_pairing_data(&data, name, NULL, NULL)) {
    return false;
  }

  if (IRK_out) {
    *IRK_out = data.irk;
  }
  if (device_out) {
    *device_out = data.identity;
  }
  if (name_out) {
    strncpy(name_out, name, BT_DEVICE_NAME_BUFFER_SIZE);
    name_out[BT_DEVICE_NAME_BUFFER_SIZE - 1] = 0;
  }

  return true;
}

bool bt_persistent_storage_get_ble_pairing_by_addr(const BTDeviceInternal *device,
                                                 SMIdentityResolvingKey *IRK_out,
                                                 char name[BT_DEVICE_NAME_BUFFER_SIZE]) {
  BTDeviceInternal device_out = {};
  bool rv = bt_persistent_storage_get_ble_pairing_by_id(BLE_BONDING_ID, IRK_out, &device_out, name);
  return (rv && bt_device_equal(&device->opaque, &device_out.opaque));
}

void bt_persistent_storage_set_active_ble_gateway(BTBondingID bonding) {
}

BTBondingID bt_persistent_storage_get_ble_ancs_bonding(void) {
  return BLE_BONDING_ID;
}

bool bt_persistent_storage_is_ble_ancs_bonding(BTBondingID bonding) {
  return bt_persistent_storage_get_ble_pairing_by_id(BLE_BONDING_ID, NULL, NULL, NULL);
}

bool bt_persistent_storage_has_ble_ancs_bonding(void) {
  return bt_persistent_storage_get_ble_pairing_by_id(BLE_BONDING_ID, NULL, NULL, NULL);
}

bool bt_persistent_storage_has_active_ble_gateway_bonding(void) {
  return bt_persistent_storage_get_ble_pairing_by_id(BLE_BONDING_ID, NULL, NULL, NULL);
}

void bt_persistent_storage_for_each_ble_pairing(BtPersistBondingDBEachBLE cb, void *context) {
  return;
}

void bt_persistent_storage_register_existing_ble_bondings(void) {
  BleBonding bonding = {};
  uint8_t flags;
  if (!shared_prf_storage_get_ble_pairing_data(&bonding.pairing_info, NULL, NULL, &flags)) {
    PBL_LOG_INFO("No existing BLE bonding to register");
    return;
  }
  bonding.is_gateway = true;
  bonding.flags = flags;
  bt_driver_handle_host_added_bonding(&bonding);
}

// PRF does not support persistent CCCD storage, these are just stubs

BTCCCDID bt_persistent_storage_store_cccd(const BleCCCD *cccd) {
  return BT_CCCD_ID_MIN;
}

bool bt_persistent_storage_delete_cccd(const BTDeviceInternal *peer, uint16_t chr_val_handle) {
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Local Device Info

void bt_persistent_storage_set_active_gateway(BTBondingID bonding) {
  return;
}

bool bt_persistent_storage_get_active_gateway(BTBondingID *bonding_out,
                                              BtPersistBondingType *type_out) {
  return false;
}

bool bt_persistent_storage_is_unfaithful(void) {
  return true;
}

void bt_persistent_storage_set_unfaithful(bool is_unfaithful) {
  return;
}

bool bt_persistent_storage_get_root_key(SMRootKeyType key_type, SM128BitKey *key_out) {
  return shared_prf_storage_get_root_key(key_type, key_out);
}

void bt_persistent_storage_set_root_keys(SM128BitKey *keys_in) {
  shared_prf_storage_set_root_keys(keys_in);
}

bool bt_persistent_storage_get_local_device_name(char *local_device_name_out, size_t max_size) {
  return shared_prf_storage_get_local_device_name(local_device_name_out, max_size);
}

void bt_persistent_storage_set_local_device_name(char *local_device_name, size_t size) {
  shared_prf_storage_set_local_device_name(local_device_name);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Remote Device Info

void bt_persistent_storage_get_cached_system_capabilities(
    PebbleProtocolCapabilities *capabilities_out) {
}

void bt_persistent_storage_set_cached_system_capabilities(
    const PebbleProtocolCapabilities *capabilities) {
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Common

void bt_persistent_storage_init(void) {
}

void bt_persistent_storage_delete_all(void) {
}

void bt_persistent_storage_delete_all_pairings(void) {
  bt_persistent_storage_delete_ble_pairing_by_id(BLE_BONDING_ID);
}
