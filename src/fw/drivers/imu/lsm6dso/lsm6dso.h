/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/accel.h"
#include "drivers/rtc.h"
#include "pbl/services/regular_timer.h"

// Accelerometer sample size (X, Y, Z, 16-bit each)
#define LSM6DSO_SAMPLE_SIZE_BYTES 6
// FIFO word size as read from FIFO_DATA_OUT (1 tag byte + 6 data bytes)
#define LSM6DSO_FIFO_WORD_SIZE_BYTES 7
// FIFO watermark threshold in samples (up to the 512-sample FIFO depth)
#define LSM6DSO_FIFO_THRESHOLD 128
// Static read buffer capacity (in samples), sized to the watermark
#define LSM6DSO_FIFO_SIZE LSM6DSO_FIFO_THRESHOLD

typedef struct LSM6DSOState {
  bool initialized;
  bool rotated;
  bool shake_detection_enabled;
  bool double_tap_detection_enabled;
  uint32_t sampling_interval_us;
  uint16_t num_samples;
  uint8_t raw_sample_buf[LSM6DSO_FIFO_SIZE * LSM6DSO_FIFO_WORD_SIZE_BYTES];
  RegularTimerInfo int1_wdt_timer;
  RtcTicks last_fifo_read_tick;
  uint32_t int1_period_ms;
  uint32_t num_recoveries;
  uint8_t wk_ths_curr;
  uint8_t shake_stuck_passes;
  bool recovery_pending;
  bool wu_active;
  bool int1_requeued;
} LSM6DSOState;

typedef struct LSM6DSOConfig {
  //! Driver state
  LSM6DSOState *state;
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
} LSM6DSOConfig;
