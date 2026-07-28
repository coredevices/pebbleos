/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <bluetooth/bluetooth_types.h>

//! Two input reports carry the same Consumer usages in two different forms, on
//! purpose: report 1 as a bitmap bit, report 2 as a 16-bit Array selector. Both
//! are exposed so the two can be compared on real phones across models and OS
//! versions. This is the one place that says why; everything else points here.
//!
//! The Array form is not assumed safe. Broken handling of 16-bit usage-range
//! Consumer arrays around iOS 16 is reported by HID device authors rather than
//! documented by Apple -- no advisory, release note or radar has been found to
//! cite -- so it is a reason to keep a fallback, not an established fact. The
//! bitmap is that fallback.
//!
//! Measured, as opposed to reported: on one iOS version, Volume Up and Volume
//! Down fire the camera shutter on both forms, and Snapshot 0x065 takes a
//! screenshot rather than a photo. No Android result either way.

//! Bits of the report 1 Consumer Control input report. @see the HID report map.
#define BLE_HID_CONSUMER_VOLUME_UP   (1 << 0)
#define BLE_HID_CONSUMER_VOLUME_DOWN (1 << 1)
#define BLE_HID_CONSUMER_PLAY_PAUSE  (1 << 2)
#define BLE_HID_CONSUMER_NEXT_TRACK  (1 << 3)
#define BLE_HID_CONSUMER_PREV_TRACK  (1 << 4)
#define BLE_HID_CONSUMER_MUTE        (1 << 5)
#define BLE_HID_CONSUMER_SNAPSHOT    (1 << 6)

//! Consumer page usage IDs of those same seven bits. The report map is built
//! from these, so it cannot drift from the mapping below.
#define BLE_HID_USAGE_VOLUME_UP   (0x0E9)
#define BLE_HID_USAGE_VOLUME_DOWN (0x0EA)
#define BLE_HID_USAGE_PLAY_PAUSE  (0x0CD)
#define BLE_HID_USAGE_NEXT_TRACK  (0x0B5)
#define BLE_HID_USAGE_PREV_TRACK  (0x0B6)
#define BLE_HID_USAGE_MUTE        (0x0E2)
#define BLE_HID_USAGE_SNAPSHOT    (0x065)

//! Largest usage report 2 can carry. @see Usage Maximum in the report map.
#define BLE_HID_USAGE_MAX (0x3FF)

//! The one usage -> report 1 bit table, so the report map and the send path
//! cannot disagree about which bit a usage is.
//! @return The report 1 bit for `usage`, or 0 if report 1 does not carry it.
static inline uint8_t ble_hid_consumer_usage_to_bit(uint16_t usage) {
  switch (usage) {
    case BLE_HID_USAGE_VOLUME_UP:
      return BLE_HID_CONSUMER_VOLUME_UP;
    case BLE_HID_USAGE_VOLUME_DOWN:
      return BLE_HID_CONSUMER_VOLUME_DOWN;
    case BLE_HID_USAGE_PLAY_PAUSE:
      return BLE_HID_CONSUMER_PLAY_PAUSE;
    case BLE_HID_USAGE_NEXT_TRACK:
      return BLE_HID_CONSUMER_NEXT_TRACK;
    case BLE_HID_USAGE_PREV_TRACK:
      return BLE_HID_CONSUMER_PREV_TRACK;
    case BLE_HID_USAGE_MUTE:
      return BLE_HID_CONSUMER_MUTE;
    case BLE_HID_USAGE_SNAPSHOT:
      return BLE_HID_CONSUMER_SNAPSHOT;
    default:
      return 0;
  }
}

//! @return True if the BT driver lib supports exposing the GATT HID service.
bool bt_driver_is_hid_service_supported(void);

//! Sends a one-byte Consumer Control input report (report 1) to the subscribed
//! device. Bit 0 = Volume Increment, bit 1 = Volume Decrement, bit 2 =
//! Play/Pause, bit 3 = Scan Next, bit 4 = Scan Previous, bit 5 = Mute,
//! bit 6 = Snapshot.
bool bt_driver_hid_service_send_consumer_report(uint8_t bits);

//! @return True if a connected device has subscribed to report 1.
bool bt_driver_hid_service_is_subscribed(void);

//! Sends a Consumer Control usage selector (report 2). The report map keeps
//! Logical Minimum and Usage Minimum equal, so the value on the wire is the
//! usage itself. 0 sits below both and is therefore "no controls asserted",
//! i.e. the release; anything above BLE_HID_USAGE_MAX is outside the range.
bool bt_driver_hid_service_send_consumer_usage(uint16_t usage);

//! @return True if a connected device has subscribed to report 2.
bool bt_driver_hid_service_usage_is_subscribed(void);

//! Called when a connected device (un)subscribes to a HID input report.
//! @param device The peer, or all-zero when the driver could not name it. That
//! happens on an unsubscribe caused by the link going away, which is the case
//! the FW most needs to hear about, so it is reported rather than swallowed.
//! An implementation must therefore not key off `device`.
//! @param is_subscribed The new state of the report that changed. One event per
//! report characteristic, so an implementation that cares about a specific
//! report should read it back with bt_driver_hid_service_is_subscribed() or
//! bt_driver_hid_service_usage_is_subscribed() rather than trust this flag.
//! @note There is exactly one implementation, in
//! src/fw/services/bluetooth/ble_hid.c. Keep it that way: a second one would
//! have to rediscover both of the above.
extern void bt_driver_cb_hid_service_update_subscription(const BTDeviceInternal *device,
                                                         bool is_subscribed);
