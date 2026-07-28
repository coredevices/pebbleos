/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "fake_shared_prf_storage.h"

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/sm_types.h>

static int s_prf_storage_ble_store_count;
static int s_prf_storage_ble_delete_count;
static int s_prf_storage_ble_delete_if_matches_count;

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Test functions

void fake_shared_prf_storage_reset_counts(void) {
  s_prf_storage_ble_store_count = 0;
  s_prf_storage_ble_delete_count = 0;
  s_prf_storage_ble_delete_if_matches_count = 0;
}

int fake_shared_prf_storage_get_ble_store_count(void) {
  return s_prf_storage_ble_store_count;
}

int fake_shared_prf_storage_get_ble_delete_count(void) {
  return s_prf_storage_ble_delete_count;
}

int fake_shared_prf_storage_get_ble_delete_if_matches_count(void) {
  return s_prf_storage_ble_delete_if_matches_count;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Custom Local Device Name

bool shared_prf_storage_get_local_device_name(char *local_device_name_out, size_t max_size) {
  return false;
}

void shared_prf_storage_set_local_device_name(char *local_device_name) {
  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Root Keys

bool shared_prf_storage_get_root_key(SMRootKeyType key_type, SM128BitKey *key_out) {
  return false;
}

void shared_prf_storage_set_root_keys(SM128BitKey *keys_in) {
  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Pairing Data

bool shared_prf_storage_get_ble_pairing_data(SMPairingInfo *pairing_info_out,
                                             char *name_out, bool *requires_address_pinning_out,
                                             uint8_t *flags) {
  return false;
}


void shared_prf_storage_store_ble_pairing_data(const SMPairingInfo *pairing_info,
                                               char *name, bool requires_address_pinning,
                                               uint8_t flags) {
  s_prf_storage_ble_delete_count++;
  s_prf_storage_ble_store_count++;
}

void shared_prf_storage_erase_ble_pairing_data(void) {
  s_prf_storage_ble_delete_count++;
}

bool shared_prf_storage_erase_ble_pairing_data_if_matches(const SMPairingInfo *expected,
                                                          SMPairingInfo *erased_out) {
  s_prf_storage_ble_delete_if_matches_count++;
  // shared_prf_storage_get_ble_pairing_data() has nothing to hand back here, so the real one would
  // find nothing to compare against and leave the slot alone.
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BT Classic Pairing Data

bool shared_prf_storage_get_bt_classic_pairing_data(BTDeviceAddress *addr_out,
                                                    char *device_name_out,
                                                    SM128BitKey *link_key_out,
                                                    uint8_t *platform_bits) {
  return false;
}

void shared_prf_storage_store_bt_classic_pairing_data(BTDeviceAddress *addr,
                                                      char *device_name,
                                                      SM128BitKey *link_key,
                                                      uint8_t platform_bits) {
}

void shared_prf_storage_store_platform_bits(uint8_t platform_bits) {
}

void shared_prf_storage_erase_bt_classic_pairing_data(void) {
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Getting Started Is Complete

bool shared_prf_storage_get_getting_started_complete(void) {
  return true;
}

void shared_prf_storage_set_getting_started_complete(bool set) {
  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Factory Reset

void shared_prf_storage_wipe_all(void) {
  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Pinned Address

bool shared_prf_storage_get_ble_pinned_address(BTDeviceAddress *address_out) {
  return false;
}

//! Stores the new BLE Pinned Address in the shared storage.
void shared_prf_storage_set_ble_pinned_address(const BTDeviceAddress *address) {
}
