/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "system/status_codes.h"

//! Which HID input report carries a Consumer Control usage. The caller always
//! states it: there is no implicit routing, so a failed press always names the
//! path it was tried on.
//! @note Both forms are exposed on purpose, so the same Consumer usage can be
//! sent as a bitmap bit and as an Array selector and the two compared on real
//! phones across models and OS versions. @see <bluetooth/hid_service.h> for why
//! the Array form is not assumed safe and what has actually been measured.
//! @note If the Array form holds across the target phone matrix, report 1 goes
//! away. What follows is a pointer to that work, not an inventory of it. Start
//! from
//! `git grep -n 'BleHidReportPath\|BLE_HID_CONSUMER_\|_consumer_report\|hid_service_is_subscribed'`
//! which reaches the FW, the three driver backends, the prompt commands, the
//! Camera app and the tests; then the parts no identifier names -- report 1's
//! half of the NimBLE report map, its Report characteristic and its
//! report-reference, and the arity of "hid usage" / "hid tap". Dropping it from
//! the report map makes every bonded phone re-pair, so it is a release decision
//! rather than a cleanup.
typedef enum {
  //! Report 1, the fixed 7-usage bitfield. Fails for a usage it does not declare.
  BleHidReportPathBitmap = 0,
  //! Report 2, the 16-bit Consumer usage array.
  BleHidReportPathUsage,
} BleHidReportPath;

//! Sets up the HID remote service. Called when the BT stack comes up.
void ble_hid_init(void);

//! Tears down the HID remote service. Called when the BT stack goes down.
void ble_hid_deinit(void);

//! @param path The report the caller intends to use. Readiness is per report:
//! a peer can subscribe to one and not the other.
//! @return True if the watch can currently act as a HID remote over `path`
//! (driver supports it and a bonded device has subscribed to that report).
bool ble_hid_is_ready(BleHidReportPath path);

//! Presses a Consumer Control usage. It stays held until released, or until the
//! safety timeout expires and ble_hid releases it on the caller's behalf. Safe
//! to call from any task; the report goes out on KernelBG, so a success only
//! means the press was accepted, not that the report reached the phone.
//! @note On success the held state is published before this returns, so a
//! caller may release immediately afterwards without racing the send.
//! @note That forced release is best effort, not a guarantee. If the lift
//! cannot be sent -- KernelBG backed up, or the send failing with the peer
//! still subscribed -- it is retried for a bounded window past the safety
//! timeout, and if it still cannot go out ble_hid logs at ERR and forgets the
//! usage without having lifted it. The key is then left down on the phone until
//! something presses again; nothing in here can do better, since the entire
//! window was spent failing to send. A peer that unsubscribes or disconnects is
//! not that case: a HOGP host drops every key it knows about, so the state is
//! cleared with no report and nothing is left down.
//! @param usage A Consumer page usage, 1..BLE_HID_USAGE_MAX.
//! @param path Which report to send it on. Report 1 does not carry every usage
//! and is not silently swapped for report 2 when it cannot.
//! @return S_SUCCESS if the press was accepted, S_NO_ACTION_REQUIRED if the
//! same usage is already held on the same path, E_INVALID_ARGUMENT for a usage
//! out of range or a path that cannot carry it, E_INVALID_OPERATION if nothing
//! is subscribed to that report, E_BUSY if another usage is held, E_INTERNAL if
//! the work could not be queued.
status_t ble_hid_consumer_press(uint16_t usage, BleHidReportPath path);

//! Releases the currently held usage. Errors if `usage` is not what is held.
//! The release always goes out on the report the press used, so the caller
//! cannot release a report 2 press on report 1.
//! @return S_SUCCESS if the release was accepted, S_NO_ACTION_REQUIRED if
//! nothing is held, E_INVALID_ARGUMENT if something else is held, E_INTERNAL if
//! the work could not be queued -- in which case the safety timeout takes over,
//! with the best-effort caveat on ble_hid_consumer_press().
status_t ble_hid_consumer_release(uint16_t usage);

//! Raises/lowers the BLE connection responsiveness for the HID remote.
void ble_hid_set_low_latency(bool enabled);
