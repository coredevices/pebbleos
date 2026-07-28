/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/sm_types.h>

//!
//! This module is used to share data between PRF and Normal FW
//!

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Custom Local Device Name

//! @param local_device_name_out Storage for the local device name.
//! @param max_size Size of the local_device_name_out buffer
//! @return true if there is a valid local device name stored, otherwise false (a zero-length string
//! will be assigned to local_device_name_out)
bool shared_prf_storage_get_local_device_name(char *local_device_name_out, size_t max_size);

//! Stores the customized local device name
//! @param local_device_name The device name to store
void shared_prf_storage_set_local_device_name(const char *local_device_name);

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Root Keys

//! Copies the BLE Encryption Root (ER) or Identity Root (IR) keys out of the shared storage.
//! @param key_out Storage into which ER or IR should be copied.
//! @param key_type The type of key to copy
//! @return true if ER and IR are copied, false if there are no keys have been found to copy.
bool shared_prf_storage_get_root_key(SMRootKeyType key_type, SM128BitKey *key_out);

//! Stores new BLE Encryption Root (ER) and Identity Root (IR) keys in the shared storage.
void shared_prf_storage_set_root_keys(SM128BitKey *keys_in);

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Pairing Data

//! Returns true if there is a valid pairing, otherwise false.
//! Out params are only valid if the function returns true
//! Pass in NULL for any values that you aren't interested in
bool shared_prf_storage_get_ble_pairing_data(SMPairingInfo *pairing_info_out,
                                             char *name_out, bool *requires_address_pinning_out,
                                             uint8_t *flags);

//! @param pairing_info Data structure containing all the pairing info available.
//! @param name Optional device name to store. Pass NULL if not available.
//! @param requires_address_pinning Whether the pairing requires address pinning.
//! @param flags Pairing flags to store.
void shared_prf_storage_store_ble_pairing_data(const SMPairingInfo *pairing_info,
                                               const char *name,
                                               bool requires_address_pinning,
                                               uint8_t flags);

void shared_prf_storage_erase_ble_pairing_data(void);

//! Erase the stored BLE pairing, but only if it still holds `expected`'s key
//! material.
//! @note The read and the erase happen under one lock. The stored pairing is
//! what is being checked, so a pairing written in between would otherwise be
//! erased by a delete aimed at the one it replaced.
//! @param expected The key material the stored pairing must still hold. Only the
//! halves `expected` marks valid are compared; no valid half never matches.
//! @param erased_out Optional. When the erase happened, the pairing that was
//! erased, so the caller can notify the BT driver outside this lock.
//! @return True if the stored pairing matched and was erased.
bool shared_prf_storage_erase_ble_pairing_data_if_matches(const SMPairingInfo *expected,
                                                          SMPairingInfo *erased_out);

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Pinned Address

//! Returns true if there is a valid pinned address, otherwise false.
//! Out params are only valid if the function returns true
//! Pass in NULL for any values that you aren't interested in
bool shared_prf_storage_get_ble_pinned_address(BTDeviceAddress *address_out);

//! Stores the new BLE Pinned Address in the shared storage.
void shared_prf_storage_set_ble_pinned_address(const BTDeviceAddress *address);

///////////////////////////////////////////////////////////////////////////////////////////////////
//! BT Classic Pairing Data

//! Returns true if there is a valid pairing, otherwise false.
//! Out params are only valid if the function returns true
//! Pass in NULL for any values that you aren't interested in
bool shared_prf_storage_get_bt_classic_pairing_data(BTDeviceAddress *addr_out,
                                                    char *device_name_out,
                                                    SM128BitKey *link_key_out,
                                                    uint8_t *platform_bits);

void shared_prf_storage_store_bt_classic_pairing_data(BTDeviceAddress *addr,
                                                      const char *device_name,
                                                      SM128BitKey *link_key,
                                                      uint8_t platform_bits);

void shared_prf_storage_store_platform_bits(uint8_t platform_bits);

void shared_prf_storage_erase_bt_classic_pairing_data(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Getting Started Is Complete

bool shared_prf_storage_get_getting_started_complete(void);

void shared_prf_storage_set_getting_started_complete(bool set);

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Factory Reset

void shared_prf_storage_wipe_all(void);

void shared_prf_storage_init(void);
