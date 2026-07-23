/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/accel.h"
#include "drivers/rtc.h"
#include "pbl/services/regular_timer.h"

#define LIS2DW12_FIFO_SIZE 32
#define LIS2DW12_SAMPLE_SIZE_BYTES 6

typedef struct LIS2DW12State {
  bool initialized;
  bool rotated;
  bool shake_detection_enabled;
  bool double_tap_detection_enabled;
  uint32_t sampling_interval_us;
  uint8_t num_samples;
  uint8_t raw_sample_buf[LIS2DW12_FIFO_SIZE * LIS2DW12_SAMPLE_SIZE_BYTES];
  RegularTimerInfo int1_wdt_timer;
  RtcTicks last_fifo_read_tick;
  uint32_t int1_period_ms;
  uint32_t num_recoveries;
  uint8_t wk_ths_curr;
  uint8_t shake_stuck_passes;
  AccelDriverSample last_sample;
  bool last_sample_valid;
  bool recovery_pending;
  bool wu_active;
} LIS2DW12State;

typedef struct LIS2DW12Config {
  //! Driver state
  LIS2DW12State *state;
  //! I2C slave port configuration
  I2CSlavePort i2c;
  //! INT1 EXTI configuration
  ExtiConfig int1;
  //! INT1 input configuration (to read back the pad level)
  InputConfig int1_in;
  //! Axis mapping (0: X, 1: Y, 2: Z)
  uint8_t axis_map[3];
  //! Axis direction (1 upside, -1 downside)
  int8_t axis_dir[3];
} LIS2DW12Config;