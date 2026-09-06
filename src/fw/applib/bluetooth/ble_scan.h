/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <bluetooth/bluetooth_types.h>

//! Callback that is called for each advertisement that is found while scanning
//! using ble_scan_start().
//! @param device The device from which the advertisement originated.
//! @param rssi The RSSI (Received Signal Strength Indication) of the
//! advertisement.
//! @param advertisement_data The payload of the advertisement. When there was
//! a scan response, this payload will contain the data of the scan response
//! as well. The information in the payload can be accessed using the
//! ble_ad_... functions, @see for example ble_ad_copy_local_name() and
//! ble_ad_includes_service().
//! @note The advertisement_data is cleaned up by the system automatically
//! immediately after returning from this callback. Do not keep around
//! any long-lived references around to the advertisement_data.
//! @note Do not use ble_ad_destroy() on the advertisement_data.
typedef void (*BLEScanHandler)(BTDevice device,
                               int8_t rssi,
                               const BLEAdData *advertisement_data);

//! Start scanning for advertisements. Pebble will scan actively, meaning it
//! will perform scan requests whenever the advertisement is scannable.
//! @param handler The callback to handle the found advertisements. It must not
//! be NULL.
//! @return BTErrnoOK if scanning started successfully, BTErrnoInvalidParameter
//! if the handler was invalid or BTErrnoInvalidState if scanning had already
//! been started.
BTErrno ble_scan_start(BLEScanHandler handler);

//! Stop scanning for advertisements.
//! @return BTErrnoOK if scanning stopped successfully, or TODO...
BTErrno ble_scan_stop(void);

//! @return True if the system is scanning for advertisements or false if not.
bool ble_scan_is_scanning(void);
