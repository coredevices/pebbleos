/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/accel_manager.h"
#include "pbl/services/imu/units.h"
#include "kernel/pebble_tasks.h"

#include <stdbool.h>

//! @addtogroup Foundation
//! @{
//!   @addtogroup EventService
//!   @{
//!     @addtogroup AccelerometerService
//!
//! \brief Using the Pebble accelerometer
//!
//! The AccelerometerService enables the Pebble accelerometer to detect taps,
//! perform measures at a given frequency, and transmit samples in batches to save CPU time
//! and processing.
//!
//! For available code samples, see the
//! <a href="https://github.com/pebble-examples/feature-accel-discs/">feature-accel-discs</a>
//! example app.
//!     @{

//! Enumerated values defining the three accelerometer axes.
typedef enum {
  //! Accelerometer's X axis. The positive direction along the X axis goes
  //! toward the right of the watch.
  ACCEL_AXIS_X = 0,
  //! Accelerometer's Y axis. The positive direction along the Y axis goes
  //! toward the top of the watch.
  ACCEL_AXIS_Y = 1,
  //! Accelerometer's Z axis. The positive direction along the Z axis goes
  //! vertically out of the watchface.
  ACCEL_AXIS_Z = 2,
} AccelAxisType;

// Make sure the AccelAxisType enum is compatible with the unified
// IMUCoordinateAxis enum.
_Static_assert(ACCEL_AXIS_X == (int)AXIS_X,
    "AccelAxisType incompatible with IMUCoordinateAxis");
_Static_assert(ACCEL_AXIS_Y == (int)AXIS_Y,
    "AccelAxisType incompatible with IMUCoordinateAxis");
_Static_assert(ACCEL_AXIS_Z == (int)AXIS_Z,
    "AccelAxisType incompatible with IMUCoordinateAxis");

#define ACCEL_DEFAULT_SAMPLING_RATE ACCEL_SAMPLING_25HZ
#define ACCEL_MINIMUM_SAMPLING_RATE ACCEL_SAMPLING_10HZ

//! Callback type for accelerometer tap events
//! @param axis the axis on which a tap was registered (x, y, or z)
//! @param direction the direction (-1 or +1) of the tap
typedef void (*AccelTapHandler)(AccelAxisType axis, int32_t direction);

//! Callback type for accelerometer data events
//! @param data Pointer to the collected accelerometer samples.
//! @param num_samples the number of samples stored in data.
typedef void (*AccelDataHandler)(AccelData *data, uint32_t num_samples);

//! Callback type for accelerometer raw data events
//! @param data Pointer to the collected accelerometer samples.
//! @param num_samples the number of samples stored in data.
//! @param timestamp the timestamp, in ms, of the first sample.
typedef void (*AccelRawDataHandler)(AccelRawData *data, uint32_t num_samples, uint64_t timestamp);

//! Subscribe to the accelerometer tap event service. Once subscribed, the handler
//! gets called on every tap event emitted by the accelerometer.
//! @param handler A callback to be executed on tap event
void accel_tap_service_subscribe(AccelTapHandler handler);

//! Unsubscribe from the accelerometer tap event service. Once unsubscribed,
//! the previously registered handler will no longer be called.
void accel_tap_service_unsubscribe(void);

//! @internal
//! Subscribe to the accelerometer double tap event service. Once subscribed, the handler
//! gets called on every double tap event emitted by the accelerometer.
//! @param handler A callback to be executed on double tap event
void accel_double_tap_service_subscribe(AccelTapHandler handler);

//! @internal
//! Unsubscribe from the accelerometer double tap event service. Once unsubscribed,
//! the previously registered handler will no longer be called.
void accel_double_tap_service_unsubscribe(void);

//! Subscribe to the accelerometer data event service. Once subscribed, the handler
//! gets called every time there are new accelerometer samples available.
//! @note Cannot use \ref accel_service_peek() when subscribed to accelerometer data events.
//! @param handler A callback to be executed on accelerometer data events
//! @param samples_per_update the number of samples to buffer, between 0 and 25.
void accel_data_service_subscribe(uint32_t samples_per_update, AccelDataHandler handler);

//! Subscribe to the accelerometer raw data event service. Once subscribed, the handler
//! gets called every time there are new accelerometer samples available.
//! @note Cannot use \ref accel_service_peek() when subscribed to accelerometer data events.
//! @param handler A callback to be executed on accelerometer data events
//! @param samples_per_update the number of samples to buffer, between 0 and 25.
void accel_raw_data_service_subscribe(uint32_t samples_per_update, AccelRawDataHandler handler);

//! Unsubscribe from the accelerometer data event service. Once unsubscribed,
//! the previously registered handler will no longer be called.
void accel_data_service_unsubscribe(void);

//! Change the accelerometer sampling rate.
//! @param rate The sampling rate in Hz (10Hz, 25Hz, 50Hz, and 100Hz possible)
int accel_service_set_sampling_rate(AccelSamplingRate rate);

//! Change the number of samples buffered between each accelerometer data event
//! @param num_samples the number of samples to buffer, between 0 and 25.
int accel_service_set_samples_per_update(uint32_t num_samples);

//! Peek at the last recorded reading.
//! @param[out] data a pointer to a pre-allocated AccelData item
//! @note Cannot be used when subscribed to accelerometer data events.
//! @return -1 if the accel is not running
//! @return -2 if subscribed to accelerometer events.
int accel_service_peek(AccelData *data);

//!     @} // end addtogroup AccelerometerService
//!   @} // end addtogroup EventService
//! @} // end addtogroup Foundation


//! @internal
typedef void (*AccelRawDataHandler__deprecated)(AccelRawData *data, uint32_t num_samples);

//! @internal
//! This is used to stay in the jump table where the old accel_data_service_subscribe was located.
//! Allows operation on AccelRawData data, which is the same as the previous version of AccelData.
void accel_data_service_subscribe__deprecated(uint32_t samples_per_update, AccelRawDataHandler__deprecated handler);
