/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "applib/accel_service.h"
#include "applib/compass_service.h"
#include "applib/preferred_content_size.h"
#include <pbl/drivers/button_id.h>
#include "pbl/util/attributes.h"

// The QEMU protocols implemented
typedef enum {
  QemuProtocol_SPP = 1,
  QemuProtocol_Tap = 2,
  QemuProtocol_BluetoothConnection = 3,
  QemuProtocol_Compass = 4,
  QemuProtocol_Battery = 5,
  QemuProtocol_Accel = 6,
  QemuProtocol_Vibration = 7,
  QemuProtocol_Button = 8,
  QemuProtocol_TimeFormat = 9,
  QemuProtocol_TimelinePeek = 10,
  QemuProtocol_ContentSize = 11,
  QemuProtocol_HealthMetric = 12,
  QemuProtocol_HeartRate = 13,
} QemuProtocol;


// ---------------------------------------------------------------------------------------
// Structure of the data for various protocols

// For QemuProtocol_SPP, the data is raw Pebble Protocol

// QemuProtocol_Tap
typedef struct PACKED {
  uint8_t axis;              // 0: x-axis, 1: y-axis, 2: z-axis
  int8_t direction;         // either +1 or -1
} QemuProtocolTapHeader;


// QemuProtocol_BluetoothConnection
typedef struct PACKED {
  uint8_t connected;         // true if connected
} QemuProtocolBluetoothConnectionHeader;


// QemuProtocol_Compass
typedef struct PACKED {
  uint32_t magnetic_heading;      // 0x10000 represents 360 degrees
  CompassStatus calib_status:8;   // CompassStatus enum
} QemuProtocolCompassHeader;


// QemuProtocol_Battery
typedef struct PACKED {
  uint8_t battery_pct;            // from 0 to 100
  uint8_t charger_connected;
} QemuProtocolBatteryHeader;


// QemuProtocol_Accel request (to Pebble)
typedef struct PACKED {
  uint8_t     num_samples;
  AccelRawData samples[0];
} QemuProtocolAccelHeader;

// QemuProtocol_Accel response (back to host)
typedef struct PACKED {
  uint16_t     avail_space;   // Number of samples we can accept
} QemuProtocolAccelResponseHeader;


// QemuProtocol_Vibration notification (sent from Pebble to host)
typedef struct PACKED {
  uint8_t     on;             // non-zero if vibe is on, 0 if off
} QemuProtocolVibrationNotificationHeader;


// QemuProtocol_Button
typedef struct PACKED {
  // New button state. Bit x specifies the state of button x, where x is one of the
  // ButtonId enum values.
  uint8_t     button_state;
} QemuProtocolButtonHeader;


// QemuProtocol_TimeFormat
typedef struct PACKED {
  uint8_t is_24_hour; // non-zero if 24h format, 0 if 12h format
} QemuProtocolTimeFormatHeader;


// QemuProtocol_TimelinePeek
typedef struct PACKED {
  //! Decides whether the Timeline Peek will show. Timeline Peek will animate only when this state
  //! toggles, and subsequent interactions that manipulate Timeline Peek outside of this
  //! QemuProtocol packet apply without an animation. The state received by this packet is also
  //! persisted, for example if enabled is true, exiting the watchface will instantly hide the
  //! peek, but returning to the watchface will instantly show the peek since this state persists.
  bool enabled;
} QemuProtocolTimelinePeekHeader;


// QemuProtocol_ContentSize
typedef struct PACKED {
  //! New system content size.
  uint8_t size;
} QemuProtocolContentSizeHeader;
#if !UNITTEST
_Static_assert(sizeof(PreferredContentSize) == sizeof(((QemuProtocolContentSizeHeader *)0)->size),
               "sizeof(PreferredContentSize) grew, need to update QemuContentSize in libpebble2 !");
#endif


// QemuProtocol_HealthMetric
// Stable wire identifier for a health metric. Mapped to ActivityMetric on the
// firmware side so reordering ActivityMetric can't silently change the protocol.
// The host pebble tool uses the same numeric values.
typedef enum {
  QemuHealthMetric_Steps = 0,
  QemuHealthMetric_ActiveSeconds = 1,
  QemuHealthMetric_RestingCalories = 2,
  QemuHealthMetric_ActiveCalories = 3,
  QemuHealthMetric_DistanceMeters = 4,
  QemuHealthMetric_SleepTotalSeconds = 5,
  QemuHealthMetric_SleepRestfulSeconds = 6,
} QemuHealthMetric;

typedef struct PACKED {
  uint8_t metric;   // QemuHealthMetric
  int32_t value;    // metric value, big-endian
} QemuProtocolHealthMetricHeader;


// QemuProtocol_HeartRate
typedef struct PACKED {
  uint8_t bpm;
  int8_t quality;  // HRMQuality (signed: HRMQuality_OffWrist is -1)
} QemuProtocolHeartRateHeader;

// ---------------------------------------------------------------------------------------
// API
void qemu_serial_init(void);

void qemu_serial_send(QemuProtocol protocol, const uint8_t *data, uint32_t len);
