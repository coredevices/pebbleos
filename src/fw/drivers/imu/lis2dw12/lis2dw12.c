/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include <pbl/drivers/accel.h>
#include <pbl/drivers/exti.h>
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/rtc.h>
#include <pbl/drivers/gpio.h>
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/imu/units.h"
#include "pbl/services/regular_timer.h"
#include <pbl/logging/logging.h>
#include "system/status_codes.h"
#include "kernel/util/delay.h"
#include "kernel/util/sleep.h"
#include "pbl/util/math.h"

PBL_LOG_MODULE_DEFINE(driver_accel_lis2dw12, CONFIG_DRIVER_IMU_LOG_LEVEL);

// Implementation notes:
//
// - Peeking returns the last FIFO sample when sampling is active, otherwise
//   single-shot mode is used to perform the measurement
// - Low-power mode 1 (12-bit) is always used (minimum power mode)
// - ODR is limited to the [12.5, 200] Hz range
// - Shake detection uses 12.5Hz when no active sampling is ongoing
// - Wake-up duration absolute time depends on the ODR, a parameter that can
//   be changed depending on the sampling interval configuration. Value is NOT
//   adjusted automatically when ODR changes (we just have 2 bits...), so it is
//   possible to notice sensitivity changes when changing sampling interval.
// - INT1 mixes level (FIFO threshold) and latched (wake-up) sources on a
//   rising-edge EXTI, so a pad latched HIGH yields no further edges: shake
//   events die and the pending interrupt blocks deep sleep. The work handler
//   re-checks the pad after servicing; a watchdog recovers the stream (FIFO
//   cadence while sampling, fixed-rate pad poll in shake-only mode).

// Time to wait after reset (us)
#define LIS2DW12_RESET_TIME_US 5

// DRDY polling parameters for accel_peek single-shot mode
#define LIS2DW12_DRDY_POLL_DELAY_MS   (5)   /* ms between data-ready polls */
#define LIS2DW12_DRDY_POLL_TIMEOUT_MS (100) /* max wait (~5x 20ms at 50Hz ODR) */

// FIFO threshold periods without a FIFO read before the stream is considered
// stalled (>1x to tolerate scheduling jitter), and threshold lower bound
#define LIS2DW12_STALL_MARGIN 2U
#define LIS2DW12_STALL_MIN_MS 1000U

// INT1 watchdog poll rate in shake-only mode (no FIFO cadence to track)
#define LIS2DW12_SHAKE_WDT_PERIOD_S 5U

// Consecutive stuck-high watchdog passes before a full stream recovery
#define LIS2DW12_SHAKE_STUCK_PASSES_MAX 3U

// Scale range when in 12-bit mode (low-power mode 1)
#define LIS2DW12_S12_SCALE_RANGE (1U << (12U - 1U))

// Registers
#define LIS2DW12_WHO_AM_I 0x0FU
#define LIS2DW12_UNDOC 0x17U
#define LIS2DW12_CTRL1 0x20U
#define LIS2DW12_CTRL2 0x21U
#define LIS2DW12_CTRL3 0x22U
#define LIS2DW12_CTRL4_INT1_PAD_CTRL 0x23U
#define LIS2DW12_CTRL5_INT2_PAD_CTRL 0x24U
#define LIS2DW12_CTRL6 0x25U
#define LIS2DW12_STATUS 0x27U
#define LIS2DW12_OUT_X_L 0x28U
#define LIS2DW12_FIFO_CTRL 0x2EU
#define LIS2DW12_FIFO_SAMPLES 0x2FU
#define LIS2DW12_WAKE_UP_THS 0x34U
#define LIS2DW12_WAKE_UP_DUR 0x35U
#define LIS2DW12_ALL_INT_SRC 0x3BU
#define LIS2DW12_CTRL7 0x3FU

// WHO_AM_I fields
#define LIS2DW12_WHO_AM_I_VAL 0x44U

// UNDOC fields
#define LIS2DW12_UNDOC_ADDR_PULLUP_DIS (1U << 6U)

// CTRL1 fields
#define LIS2DW12_CTRL1_LP_MODE1 (0U << 0U)
#define LIS2DW12_CTRL1_MODE_LP (0U << 2U)
#define LIS2DW12_CTRL1_MODE_SINGLE (2U << 2U)
#define LIS2DW12_CTRL1_ODR_PD (0x0U << 4U)
#define LIS2DW12_CTRL1_ODR_1HZ6_LP_ONLY (0x1U << 4U)
#define LIS2DW12_CTRL1_ODR_12HZ5 (0x2U << 4U)
#define LIS2DW12_CTRL1_ODR_25HZ (0x3U << 4U)
#define LIS2DW12_CTRL1_ODR_50HZ (0x4U << 4U)
#define LIS2DW12_CTRL1_ODR_100HZ (0x5U << 4U)
#define LIS2DW12_CTRL1_ODR_200HZ (0x6U << 4U)
#define LIS2DW12_CTRL1_ODR_400HZ_HP_ONLY (0x7U << 4U)
#define LIS2DW12_CTRL1_ODR_800HZ_HP_ONLY (0x8U << 4U)
#define LIS2DW12_CTRL1_ODR_1K6HZ_HP_ONLY (0x9U << 4U)

// CTRL2 fields
#define LIS2DW12_CTRL2_SOFT_RESET (1U << 6U)
#define LIS2DW12_CTRL2_BOOT (1U << 7U)

// CTRL3 fields
#define LIS2DW12_CTRL3_SLP_MODE_1 (1U << 0U)
#define LIS2DW12_CTRL3_SLP_MODE_SEL_SLP_MODE_1 (1U << 1U)
#define LIS2DW12_CTRL3_LIR (1U << 4U)

// CTRL4_INT1_PAD_CTRL fields
#define LIS2DW12_CTRL4_INT1_PAD_CTRL_INT1_WU (1U << 5U)
#define LIS2DW12_CTRL4_INT1_PAD_CTRL_INT1_FTH (1U << 1U)

// CTRL5_INT2_PAD_CTRL fields
#define LIS2DW12_CTRL5_INT2_PAD_CTRL_INT2_OVR (1U << 3U)

// CTRL6 fields
#define LIS2DW12_CTRL6_FS_2G (0U << 4U)
#define LIS2DW12_CTRL6_FS_4G (1U << 4U)
#define LIS2DW12_CTRL6_FS_8G (2U << 4U)
#define LIS2DW12_CTRL6_FS_16G (3U << 4U)

// STATUS fields
#define LIS2DW12_STATUS_DRDY (1U << 0U)

// FIFO_CTRL fields
#define LIS2DW12_FIFO_CTRL_FTH_POS 0U
#define LIS2DW12_FIFO_CTRL_FTH_MASK 0x1FU
#define LIS2DW12_FIFO_CTRL_FTH(val) \
  (((val) << LIS2DW12_FIFO_CTRL_FTH_POS) & LIS2DW12_FIFO_CTRL_FTH_MASK)
#define LIS2DW12_FIFO_CTRL_FIFO_MODE_BYPASS (0x0U << 5U)
#define LIS2DW12_FIFO_CTRL_FIFO_MODE_FIFO (0x1U << 5U)
#define LIS2DW12_FIFO_CTRL_FIFO_MODE_CONT (0x6U << 5U)

// FIFO_SAMPLES fields
#define LIS2DW12_FIFO_SAMPLES_DIFF_POS 0U
#define LIS2DW12_FIFO_SAMPLES_DIFF_MASK 0x3FU
#define LIS2DW12_FIFO_SAMPLES_DIFF_GET(val) \
  (((val) & LIS2DW12_FIFO_SAMPLES_DIFF_MASK) >> LIS2DW12_FIFO_SAMPLES_DIFF_POS)
#define LIS2DW12_FIFO_SAMPLES_FIFO_OVR (1U << 6U)
#define LIS2DW12_FIFO_SAMPLES_FIFO_FTH (1U << 7U)

// WAKE_UP_THS fields
#define LIS2DW12_WAKE_UP_THS_WK_THS_POS 0U
#define LIS2DW12_WAKE_UP_THS_WK_THS_MASK 0x3FU
#define LIS2DW12_WAKE_UP_THS_WK_THS(val) \
  (((val) << LIS2DW12_WAKE_UP_THS_WK_THS_POS) & LIS2DW12_WAKE_UP_THS_WK_THS_MASK)

// WAKE_UP_DUR fields
#define LIS2DW12_WAKE_UP_DUR_WAKE_DUR_POS 5U
#define LIS2DW12_WAKE_UP_DUR_WAKE_DUR_MASK 0x60U
#define LIS2DW12_WAKE_UP_DUR_WAKE_DUR(val) \
  (((val) << LIS2DW12_WAKE_UP_DUR_WAKE_DUR_POS) & LIS2DW12_WAKE_UP_DUR_WAKE_DUR_MASK)

// ALL_INT_SRC fields
#define LIS2DW12_ALL_INT_SRC_WU_IA (1U << 1U)

// CTRL7 fields
#define LIS2DW12_CTRL7_INTERRUPTS_ENABLE (1U << 5U)
#define LIS2DW12_CTRL7_INT2_ON_INT1 (1U << 6U)

////////////////////////////////////////////////////////////////////////////////
// Private
////////////////////////////////////////////////////////////////////////////////

static bool prv_lis2dw12_write(uint8_t reg, const uint8_t *data, uint16_t len) {
  bool ret;

  i2c_use(&LIS2DW12->i2c);
  ret = i2c_write_register_block(&LIS2DW12->i2c, reg, len, data);
  i2c_release(&LIS2DW12->i2c);

  return ret;
}

static bool prv_lis2dw12_read(uint8_t reg, uint8_t *data, uint16_t len) {
  bool ret;

  i2c_use(&LIS2DW12->i2c);
  ret = i2c_read_register_block(&LIS2DW12->i2c, reg, len, data);
  i2c_release(&LIS2DW12->i2c);

  return ret;
}

static int16_t prv_raw_to_s12(const uint8_t *raw) {
  uint16_t val;

  val = ((raw[0] >> 4U) & 0xFU) | (raw[1] << 4U);
  if (val & 0x0800U) {
    val |= 0xF000U;
  }

  return (int16_t)val;
}

static int16_t prv_axis_raw_mg(IMUCoordinateAxis axis, const uint8_t *raw) {
  uint8_t offset;
  int16_t val;

  offset = LIS2DW12->axis_map[axis];

  val = LIS2DW12->axis_dir[axis] *
        (prv_raw_to_s12(&raw[offset * 2U]) * (int16_t)CONFIG_ACCEL_LIS2DW12_SCALE_MG) /
        (int16_t)LIS2DW12_S12_SCALE_RANGE;

  if (LIS2DW12->state->rotated && (axis == AXIS_X || axis == AXIS_Y)) {
    val *= -1;
  }

  return val;
}

static void prv_raw_to_mg(const uint8_t *raw, AccelDriverSample *sample) {
  sample->x = prv_axis_raw_mg(AXIS_X, raw);
  sample->y = prv_axis_raw_mg(AXIS_Y, raw);
  sample->z = prv_axis_raw_mg(AXIS_Z, raw);
}

static uint64_t prv_get_curr_system_time_us(void) {
  time_t time_s;
  uint16_t time_ms;

  rtc_get_time_ms(&time_s, &time_ms);

  return (((uint64_t)time_s) * 1000 + time_ms) * 1000ULL;
}

// Convert and dispatch the samples already sitting in raw_sample_buf. Kept free of
// device I/O so it can run after the I2C reads instead of in between them.
static void prv_lis2dw12_process_samples(uint8_t num_samples, uint64_t timestamp_us) {
  // Samples are 12-bit left-justified within a 16-bit little-endian word, so the
  // raw word equals the 12-bit value << 4; scale the denominator by 16 to match.
  int8_t rotate = LIS2DW12->state->rotated ? -1 : 1;
  AccelRawBatch batch = {
    .data = LIS2DW12->state->raw_sample_buf,
    .num_samples = num_samples,
    .stride = LIS2DW12_SAMPLE_SIZE_BYTES,
    .axis = {
      [AXIS_X] = {.offset = LIS2DW12->axis_map[AXIS_X] * 2U,
                  .sign = (int8_t)(LIS2DW12->axis_dir[AXIS_X] * rotate)},
      [AXIS_Y] = {.offset = LIS2DW12->axis_map[AXIS_Y] * 2U,
                  .sign = (int8_t)(LIS2DW12->axis_dir[AXIS_Y] * rotate)},
      [AXIS_Z] = {.offset = LIS2DW12->axis_map[AXIS_Z] * 2U, .sign = LIS2DW12->axis_dir[AXIS_Z]},
    },
    .scale_num = CONFIG_ACCEL_LIS2DW12_SCALE_MG,
    .scale_den = LIS2DW12_S12_SCALE_RANGE << 4U,
    .first_timestamp_us = timestamp_us,
    .sampling_interval_us = LIS2DW12->state->sampling_interval_us,
  };

  accel_cb_new_samples(&batch);

  // Keep the most recent sample around for accel_peek().
  uint8_t *last = &LIS2DW12->state->raw_sample_buf[(num_samples - 1) * LIS2DW12_SAMPLE_SIZE_BYTES];
  prv_raw_to_mg(last, &LIS2DW12->state->last_sample);
  LIS2DW12->state->last_sample.timestamp_us =
      timestamp_us + (num_samples - 1) * LIS2DW12->state->sampling_interval_us;
  LIS2DW12->state->last_sample_valid = true;
}

static bool prv_lis2dw12_enable_fifo(uint8_t num_samples) {
  bool ret;
  uint8_t val;

  val = LIS2DW12_FIFO_CTRL_FIFO_MODE_BYPASS;
  ret = prv_lis2dw12_write(LIS2DW12_FIFO_CTRL, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL register");
    return ret;
  }

  val = LIS2DW12_FIFO_CTRL_FTH(num_samples) | LIS2DW12_FIFO_CTRL_FIFO_MODE_CONT;
  ret = prv_lis2dw12_write(LIS2DW12_FIFO_CTRL, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL register");
    return ret;
  }

  PBL_LOG_DBG("FIFO enabled with threshold %" PRIu8, num_samples);

  return true;
}

//! Salvage and dispatch any samples queued in the FIFO
static void prv_lis2dw12_drain_fifo(void) {
  uint8_t val;
  uint8_t samples;

  if (!prv_lis2dw12_read(LIS2DW12_FIFO_SAMPLES, &val, 1)) {
    PBL_LOG_ERR("Could not read FIFO_SAMPLES register");
    return;
  }

  samples = MIN(LIS2DW12_FIFO_SAMPLES_DIFF_GET(val), LIS2DW12_FIFO_SIZE);
  if (samples == 0U) {
    return;
  }

  if (!prv_lis2dw12_read(LIS2DW12_OUT_X_L, LIS2DW12->state->raw_sample_buf,
                         samples * LIS2DW12_SAMPLE_SIZE_BYTES)) {
    PBL_LOG_ERR("Failed to read samples");
    return;
  }

  prv_lis2dw12_process_samples(samples, prv_get_curr_system_time_us());
  LIS2DW12->state->last_fifo_read_tick = rtc_get_ticks();
}

static void prv_lis2dw12_recover(void);

//! Single INT1 servicing pass; returns true if any INT source was handled.
//! fifo_progress is set when FIFO data was consumed (samples or overrun).
static bool prv_lis2dw12_service_int1(bool *fifo_progress) {
  bool ret;
  uint8_t val;
  bool action_taken = false;
  bool fifo_overrun = false;
  bool shake = false;
  uint8_t samples = 0U;
  uint64_t timestamp_us = 0U;

  // Read all device registers back-to-back to keep the I2C critical section
  // short, then convert/dispatch below. The sample processing has no dependency
  // on these reads, so deferring it avoids stretching the gap between the FIFO
  // read and the ALL_INT_SRC read if this task gets preempted mid-handler.
  if (LIS2DW12->state->num_samples > 0U) {
    ret = prv_lis2dw12_read(LIS2DW12_FIFO_SAMPLES, &val, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read FIFO_SAMPLES register");
      return false;
    }

    if ((val & LIS2DW12_FIFO_SAMPLES_FIFO_OVR) != 0U) {
      fifo_overrun = true;
    } else if ((val & LIS2DW12_FIFO_SAMPLES_FIFO_FTH) != 0U) {
      samples = LIS2DW12_FIFO_SAMPLES_DIFF_GET(val);
      if (samples > 0U) {
        if (!prv_lis2dw12_read(LIS2DW12_OUT_X_L, LIS2DW12->state->raw_sample_buf,
                               samples * LIS2DW12_SAMPLE_SIZE_BYTES)) {
          PBL_LOG_ERR("Failed to read samples");
          return false;
        }
        timestamp_us = prv_get_curr_system_time_us();
        LIS2DW12->state->last_fifo_read_tick = rtc_get_ticks();
      }
    }
  }

  if (LIS2DW12->state->shake_detection_enabled) {
    ret = prv_lis2dw12_read(LIS2DW12_ALL_INT_SRC, &val, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read ALL_INT_SRC register");
      return false;
    }

    shake = (val & LIS2DW12_ALL_INT_SRC_WU_IA) != 0U;
  }

  if (fifo_overrun) {
    PBL_LOG_WRN("FIFO overrun detected, recovering");
    prv_lis2dw12_recover();
    action_taken = true;
    *fifo_progress = true;
  } else if (samples > 0U) {
    prv_lis2dw12_process_samples(samples, timestamp_us);
    action_taken = true;
    *fifo_progress = true;
  }

  if (shake) {
    // WU_IA stays set while the wake-up condition persists (LIR), so only
    // dispatch on its assertion to avoid flooding events
    if (!LIS2DW12->state->wu_active) {
      PBL_LOG_DBG("Shake detected");
      // TODO: provide more info about the shake (axis, direction, etc.) or
      // refactor shake to be non-dimensional
      accel_cb_shake_detected(AXIS_Z, 0);
    }
    action_taken = true;
  }
  LIS2DW12->state->wu_active = shake;

  if (!action_taken) {
    PBL_LOG_WRN("INT1 triggered but no action taken");
  }

  return action_taken;
}

static void prv_lis2dw12_int1_work_handler(void) {
  bool fifo_progress = false;
  bool action_taken = prv_lis2dw12_service_int1(&fifo_progress);

  // Sources asserting while the pad is high produce no new edge: requeue on
  // FIFO progress, recover when nothing was serviced (stuck pad). A pad held
  // by a persistent wake-up condition is left to the stall watchdog.
  if (!gpio_input_read(&LIS2DW12->int1_in)) {
    return;
  }

  if (fifo_progress) {
    accel_offload_work(prv_lis2dw12_int1_work_handler);
  } else if (!action_taken) {
    prv_lis2dw12_recover();
  }
}

static void prv_lis2dw12_int1_irq_handler(bool *should_context_switch) {
  // A rising edge proves the pad was low, i.e. the wake-up source deasserted
  LIS2DW12->state->wu_active = false;
  // ... which also breaks any stuck-high streak the watchdog counted
  LIS2DW12->state->shake_stuck_passes = 0U;
  accel_offload_work_from_isr(prv_lis2dw12_int1_work_handler, should_context_switch);
}

static bool prv_configure_odr(uint32_t sampling_interval_us, bool shake_detection_enabled) {
  uint8_t val;
  bool ret;

  // If shake detection is enabled, ensure a minimum ODR of 12.5Hz (80ms)
  if (shake_detection_enabled && (sampling_interval_us == 0UL)) {
    sampling_interval_us = 80000UL;
  }

  val = LIS2DW12_CTRL1_LP_MODE1 | LIS2DW12_CTRL1_MODE_LP;

  if (sampling_interval_us == 0U) {
    val |= LIS2DW12_CTRL1_ODR_PD;
    sampling_interval_us = 0UL;
  } else if (sampling_interval_us >= 80000UL) {
    val |= LIS2DW12_CTRL1_ODR_12HZ5;
    sampling_interval_us = 80000UL;
  } else if (sampling_interval_us >= 40000UL) {
    val |= LIS2DW12_CTRL1_ODR_25HZ;
    sampling_interval_us = 40000UL;
  } else if (sampling_interval_us >= 20000UL) {
    val |= LIS2DW12_CTRL1_ODR_50HZ;
    sampling_interval_us = 20000UL;
  } else if (sampling_interval_us >= 10000UL) {
    val |= LIS2DW12_CTRL1_ODR_100HZ;
    sampling_interval_us = 10000UL;
  } else {
    val |= LIS2DW12_CTRL1_ODR_200HZ;
    sampling_interval_us = 5000UL;
  }

  PBL_LOG_DBG("Configuring ODR to %" PRIu32 " ms (%" PRIu32 " mHz)",
          sampling_interval_us / 1000UL,
          sampling_interval_us > 0UL ? 1000000000UL / sampling_interval_us : 0UL);

  ret = prv_lis2dw12_write(LIS2DW12_CTRL1, &val, 1);
  if (!ret) {
    return ret;
  }

  LIS2DW12->state->sampling_interval_us = sampling_interval_us;

  return true;
}

static bool prv_configure_int1(bool shake_detection_enabled, bool fifo_enabled) {
  bool ret;
  uint8_t ctrl4;
  uint8_t ctrl5;
  uint8_t ctrl7;

  ctrl4 = 0U;
  ctrl5 = 0U;

  if (shake_detection_enabled) {
    ctrl4 |= LIS2DW12_CTRL4_INT1_PAD_CTRL_INT1_WU;
  }

  if (fifo_enabled) {
    ctrl4 |= LIS2DW12_CTRL4_INT1_PAD_CTRL_INT1_FTH;
    ctrl5 |= LIS2DW12_CTRL5_INT2_PAD_CTRL_INT2_OVR;
  }

  ret = prv_lis2dw12_write(LIS2DW12_CTRL4_INT1_PAD_CTRL, &ctrl4, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL4_INT1_PAD_CTRL register");
    return ret;
  }

  PBL_LOG_DBG("INT1 configured: %02" PRIx8, ctrl4);

  ret = prv_lis2dw12_write(LIS2DW12_CTRL5_INT2_PAD_CTRL, &ctrl5, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL5_INT2_PAD_CTRL register");
    return ret;
  }

  PBL_LOG_DBG("INT2 configured: %02" PRIx8, ctrl5);

  ctrl7 = (ctrl4 == 0U && ctrl5 == 0U)
              ? 0U
              : LIS2DW12_CTRL7_INTERRUPTS_ENABLE | LIS2DW12_CTRL7_INT2_ON_INT1;
  ret = prv_lis2dw12_write(LIS2DW12_CTRL7, &ctrl7, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL7 register");
    return ret;
  }

  PBL_LOG_DBG("Enabled interrupts: %u", ctrl7 != 0U);

  return true;
}

//! Recover a dead INT1/FIFO stream by re-asserting ODR, FIFO and INT routing.
//! Routing is quiesced first so a latched-high pad produces a fresh edge.
static void prv_lis2dw12_recover(void) {
  uint8_t val;

  LIS2DW12->state->num_recoveries++;
  PBL_ANALYTICS_ADD(accel_stream_recovery_count, 1);
  PBL_LOG_WRN("Recovering accel stream (count %" PRIu32 ")",
          LIS2DW12->state->num_recoveries);

  if (!prv_configure_int1(false, false)) {
    return;
  }

  // Salvage queued samples
  if (LIS2DW12->state->num_samples > 0U) {
    prv_lis2dw12_drain_fifo();
  }

  // Clear any latched function INT source while routing is quiesced
  if (!prv_lis2dw12_read(LIS2DW12_ALL_INT_SRC, &val, 1)) {
    PBL_LOG_ERR("Could not read ALL_INT_SRC register");
    return;
  }

  if (!prv_configure_odr(LIS2DW12->state->sampling_interval_us,
                         LIS2DW12->state->shake_detection_enabled)) {
    PBL_LOG_ERR("Could not configure ODR");
    return;
  }

  if (LIS2DW12->state->num_samples > 0U) {
    if (!prv_lis2dw12_enable_fifo(LIS2DW12->state->num_samples)) {
      return;
    }
  }

  if (!prv_configure_int1(LIS2DW12->state->shake_detection_enabled,
                          LIS2DW12->state->num_samples > 0U)) {
    return;
  }

  LIS2DW12->state->wu_active = false;
  LIS2DW12->state->last_fifo_read_tick = rtc_get_ticks();
}

static uint32_t prv_ms_since_last_fifo_read(void) {
  RtcTicks ticks = rtc_get_ticks() - LIS2DW12->state->last_fifo_read_tick;
  return (uint32_t)((ticks * 1000) / RTC_TICKS_HZ);
}

static uint32_t prv_stall_threshold_ms(void) {
  return MAX(LIS2DW12_STALL_MARGIN * LIS2DW12->state->int1_period_ms,
             LIS2DW12_STALL_MIN_MS);
}

//! Arm/disarm the INT1 stall watchdog to match the active INT1 sources.
//! Call after num_samples or shake_detection_enabled changes.
static void prv_update_int1_watchdog(void) {
  regular_timer_remove_callback(&LIS2DW12->state->int1_wdt_timer);
  LIS2DW12->state->shake_stuck_passes = 0U;

  if (LIS2DW12->state->num_samples > 0U) {
    LIS2DW12->state->last_fifo_read_tick = rtc_get_ticks();
    LIS2DW12->state->int1_period_ms =
        (LIS2DW12->state->sampling_interval_us * LIS2DW12->state->num_samples) / 1000;
    regular_timer_add_multisecond_callback(
        &LIS2DW12->state->int1_wdt_timer,
        MAX(DIVIDE_CEIL(LIS2DW12->state->int1_period_ms, 1000UL), 1U));
  } else if (LIS2DW12->state->shake_detection_enabled) {
    regular_timer_add_multisecond_callback(&LIS2DW12->state->int1_wdt_timer,
                                           LIS2DW12_SHAKE_WDT_PERIOD_S);
  }
}

//! Shared by the INT1 watchdog and the accel_peek staleness trigger;
//! re-validates the stall at execution time.
static void prv_stall_check_work_cb(void) {
  uint32_t ms_since_last_read;

  LIS2DW12->state->recovery_pending = false;

  // Sampling may have stopped between scheduling and execution
  if (LIS2DW12->state->num_samples == 0U) {
    // Shake-only mode: re-run the servicing pass if the pad is stuck high
    // (reading the INT source clears the latch); escalate to a full recovery
    // after consecutive passes that never release it.
    if (LIS2DW12->state->shake_detection_enabled &&
        gpio_input_read(&LIS2DW12->int1_in)) {
      prv_lis2dw12_int1_work_handler();
      // A pad released by the pass is healthy; count only a still-high pad
      if (!gpio_input_read(&LIS2DW12->int1_in)) {
        LIS2DW12->state->shake_stuck_passes = 0U;
      } else if (++LIS2DW12->state->shake_stuck_passes >= LIS2DW12_SHAKE_STUCK_PASSES_MAX) {
        PBL_LOG_WRN("INT1 pad stuck high for %" PRIu8 " shake watchdog passes, recovering",
                    LIS2DW12->state->shake_stuck_passes);
        prv_lis2dw12_recover();
        LIS2DW12->state->shake_stuck_passes = 0U;
      }
    } else {
      LIS2DW12->state->shake_stuck_passes = 0U;
    }
    return;
  }

  ms_since_last_read = prv_ms_since_last_fifo_read();
  if (ms_since_last_read < prv_stall_threshold_ms()) {
    return;
  }

  PBL_LOG_WRN("FIFO stream stalled for %" PRIu32 " ms", ms_since_last_read);
  prv_lis2dw12_recover();
}

static void prv_int1_wdt_cb(void *data) {
  accel_offload_work(prv_stall_check_work_cb);
}

////////////////////////////////////////////////////////////////////////////////
// Accelerometer interface
////////////////////////////////////////////////////////////////////////////////

void accel_init(void) {
  bool ret;
  uint8_t val;

  // Check device ID
  ret = prv_lis2dw12_read(LIS2DW12_WHO_AM_I, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read WHO_AM_I register");
    return;
  }

  if (val != LIS2DW12_WHO_AM_I_VAL) {
    PBL_LOG_ERR("Unexpected id: 0x%02X!=0x%02X", val, LIS2DW12_WHO_AM_I_VAL);
    return;
  }

  // Perform a software reset (so we can rely on defaults)
  val = LIS2DW12_CTRL2_SOFT_RESET;
  ret = prv_lis2dw12_write(LIS2DW12_CTRL2, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL2 register");
    return;
  }

  delay_us(LIS2DW12_RESET_TIME_US);

  do {
    ret = prv_lis2dw12_read(LIS2DW12_CTRL2, &val, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read CTRL2 register");
      return;
    }
  } while ((val & LIS2DW12_CTRL2_BOOT) != 0U);

  // Disable ADDR pull-up if requested
  // NOTE: This is an undocumented register (provided by FAE)
#ifdef CONFIG_ACCEL_LIS2DW12_DISABLE_ADDR_PULLUP
  ret = prv_lis2dw12_read(LIS2DW12_UNDOC, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Failed to read LIS2DW12 register 0x17");
    return;
  }

  val |= LIS2DW12_UNDOC_ADDR_PULLUP_DIS;
  ret = prv_lis2dw12_write(LIS2DW12_UNDOC, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Failed to write LIS2DW12 register 0x17");
    return;
  }
#endif

  // Single-data conversion via SLP_MODE_1, latch function IRQ
  val = LIS2DW12_CTRL3_SLP_MODE_SEL_SLP_MODE_1 | LIS2DW12_CTRL3_LIR;
  ret = prv_lis2dw12_write(LIS2DW12_CTRL3, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL3 register");
    return;
  }

  // Configure scale
  switch (CONFIG_ACCEL_LIS2DW12_SCALE_MG) {
    case 2000U:
      val = LIS2DW12_CTRL6_FS_2G;
      break;
    case 4000U:
      val = LIS2DW12_CTRL6_FS_4G;
      break;
    case 8000U:
      val = LIS2DW12_CTRL6_FS_8G;
      break;
    case 16000U:
      val = LIS2DW12_CTRL6_FS_16G;
      break;
    default:
      PBL_LOG_ERR("Invalid scale: %d", CONFIG_ACCEL_LIS2DW12_SCALE_MG);
      return;
  }

  ret = prv_lis2dw12_write(LIS2DW12_CTRL6, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL6 register");
    return;
  }

  // Configure wake-up threshold defaults
  val = LIS2DW12_WAKE_UP_DUR_WAKE_DUR(CONFIG_ACCEL_LIS2DW12_WK_DUR_DEFAULT);
  ret = prv_lis2dw12_write(LIS2DW12_WAKE_UP_DUR, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_DUR register");
    return;
  }

  val = LIS2DW12_WAKE_UP_THS_WK_THS(CONFIG_ACCEL_LIS2DW12_WK_THS_DEFAULT);
  ret = prv_lis2dw12_write(LIS2DW12_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return;
  }

  LIS2DW12->state->wk_ths_curr = CONFIG_ACCEL_LIS2DW12_WK_THS_DEFAULT;

  // Enable INT1 external interrupt
  exti_configure_pin(LIS2DW12->int1, ExtiTrigger_Rising, prv_lis2dw12_int1_irq_handler);
  exti_enable(LIS2DW12->int1);

  LIS2DW12->state->int1_wdt_timer.cb = prv_int1_wdt_cb;
  LIS2DW12->state->initialized = true;
}

uint32_t accel_set_sampling_interval(uint32_t interval_us) {
  if (!LIS2DW12->state->initialized) {
    // Just pretend we can achieve any requested interval
    LIS2DW12->state->sampling_interval_us = interval_us;
  } else {
    // FIXME: we should technically stop and drain the FIFO here, otherwise
    // we may report existing samples in the FIFO buffer with an incorrect timestamp

    const uint32_t prev_interval_us = LIS2DW12->state->sampling_interval_us;
    if (!prv_configure_odr(interval_us, LIS2DW12->state->shake_detection_enabled)) {
      PBL_LOG_ERR("Could not configure ODR");
    }

    // Re-arm the watchdog only on an actual ODR change (a re-arm resets the
    // stall tracking, so repeat calls must not defer it)
    if (LIS2DW12->state->sampling_interval_us != prev_interval_us) {
      prv_update_int1_watchdog();
    }
  }

  PBL_LOG_DBG("Set sampling interval to %" PRIu32 " us",
          LIS2DW12->state->sampling_interval_us);

  return LIS2DW12->state->sampling_interval_us;
}

uint32_t accel_get_sampling_interval(void) {
  return LIS2DW12->state->sampling_interval_us;
}

uint32_t accel_get_max_num_samples(void) {
  return LIS2DW12_FIFO_SIZE;
}

void accel_set_num_samples(uint32_t num_samples) {
  bool ret;
  uint8_t val;

  if (!LIS2DW12->state->initialized) {
    return;
  }

  // Limit to FIFO threshold
  if (num_samples > LIS2DW12_FIFO_SIZE) {
    num_samples = LIS2DW12_FIFO_SIZE;
  }

  // Disable all INT1 before changing FIFO threshold
  prv_configure_int1(false, false);

  // Config change invalidates any queued stall check; re-arm the peek trigger
  LIS2DW12->state->recovery_pending = false;

  // Salvage queued samples
  if (LIS2DW12->state->num_samples > 0U) {
    prv_lis2dw12_drain_fifo();
  }

  if (num_samples == 0U) {
    // Bypass FIFO (disable)
    val = LIS2DW12_FIFO_CTRL_FIFO_MODE_BYPASS;
    if (!prv_lis2dw12_write(LIS2DW12_FIFO_CTRL, &val, 1)) {
      PBL_LOG_ERR("Could not write FIFO_CTRL register");
    }
  } else {
    // Configure FIFO in CONT mode with threshold
    ret = prv_lis2dw12_enable_fifo((uint8_t)num_samples);
    if (!ret) {
      PBL_LOG_ERR("Could not enable FIFO");
      return;
    }

    LIS2DW12->state->last_sample_valid = false;
  }

  // Re-configure INT1
  ret = prv_configure_int1(LIS2DW12->state->shake_detection_enabled, num_samples > 0U);
  if (!ret) {
    PBL_LOG_ERR("Could not configure INT1");
    return;
  }

  LIS2DW12->state->num_samples = num_samples;
  // Arm/disarm the INT1 liveness watchdog for the new source configuration
  prv_update_int1_watchdog();

  PBL_LOG_DBG("Set number of samples to %" PRIu32, num_samples);
}

int accel_peek(AccelDriverSample *data) {
  int err = 0;
  bool ret;
  uint8_t ctrl1;
  uint8_t ctrl1_bck;
  uint8_t ctrl3;
  uint8_t status;
  uint8_t raw[LIS2DW12_SAMPLE_SIZE_BYTES];

  if (!LIS2DW12->state->initialized) {
    return E_ERROR;
  }

  // If sampling is active, the FIFO batches samples for subscribers, so the
  // cached sample can be a full watermark period old. The FIFO is read through
  // the output registers, so peeking those directly would steal a sample from
  // the stream; drain the FIFO instead, which refreshes the cached sample and
  // dispatches the queued samples to subscribers.
  if (LIS2DW12->state->num_samples > 0U) {
    // Self-heal: a stalled stream would otherwise freeze peek data forever
    if (!LIS2DW12->state->recovery_pending &&
        (prv_ms_since_last_fifo_read() >= prv_stall_threshold_ms())) {
      LIS2DW12->state->recovery_pending = true;
      accel_offload_work(prv_stall_check_work_cb);
    }

    prv_lis2dw12_drain_fifo();

    if (!LIS2DW12->state->last_sample_valid) {
      return E_ERROR;
    }
    *data = LIS2DW12->state->last_sample;
    return 0;
  }

  // Save CTRL1
  ret = prv_lis2dw12_read(LIS2DW12_CTRL1, &ctrl1_bck, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read CTRL1 register");
    return E_ERROR;
  }

  // Configure single mode, ODR@50Hz (recommended ODR, see DT0102 rev1)
  ctrl1 = LIS2DW12_CTRL1_MODE_SINGLE | LIS2DW12_CTRL1_ODR_50HZ;
  ret = prv_lis2dw12_write(LIS2DW12_CTRL1, &ctrl1, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL1 register");
    return E_ERROR;
  }

  // Trigger single measurement by setting SLP_MODE_1 bit
  ret = prv_lis2dw12_read(LIS2DW12_CTRL3, &ctrl3, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read CTRL3 register");
    return E_ERROR;
  }

  ctrl3 |= LIS2DW12_CTRL3_SLP_MODE_1;
  ret = prv_lis2dw12_write(LIS2DW12_CTRL3, &ctrl3, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL3 register");
    return E_ERROR;
  }

  // Poll for data ready (timeout after 100ms, ~5x the expected 20ms at 50Hz ODR)
  uint32_t elapsed_ms = 0;
  do {
    ret = prv_lis2dw12_read(LIS2DW12_STATUS, &status, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read STATUS register");
      err = E_ERROR;
      goto end;
    }
    if ((status & LIS2DW12_STATUS_DRDY) == 0U) {
      if (elapsed_ms >= LIS2DW12_DRDY_POLL_TIMEOUT_MS) {
        PBL_LOG_ERR("DRDY timeout after %" PRIu32 " ms", elapsed_ms);
        err = E_ERROR;
        goto end;
      }
      psleep(LIS2DW12_DRDY_POLL_DELAY_MS);
      elapsed_ms += LIS2DW12_DRDY_POLL_DELAY_MS;
    }
  } while ((status & LIS2DW12_STATUS_DRDY) == 0U);

  // Read sample
  ret = prv_lis2dw12_read(LIS2DW12_OUT_X_L, raw, sizeof(raw));
  if (!ret) {
    PBL_LOG_ERR("Failed to read sample");
    err = E_ERROR;
    goto end;
  }

  // Convert to mg and populate timestamp
  prv_raw_to_mg(raw, data);
  data->timestamp_us = prv_get_curr_system_time_us();

end:
  // Restore CTRL1 (back to previous state, e.g. power-down or shake ODR)
  (void)prv_lis2dw12_write(LIS2DW12_CTRL1, &ctrl1_bck, 1);

  return err;
}

void accel_enable_shake_detection(bool on) {
  bool ret;

  if (!LIS2DW12->state->initialized) {
    return;
  }

  // Configure ODR (use current interval, will be adjusted if < 12.5Hz)
  ret = prv_configure_odr(LIS2DW12->state->sampling_interval_us, on);
  if (!ret) {
    PBL_LOG_ERR("Could not configure ODR");
    return;
  }

  if (on) {
    uint8_t val;

    // WU_IA latches even while unrouted (e.g. on ODR startup transients);
    // clear it so a phantom shake is not dispatched on enable
    if (!prv_lis2dw12_read(LIS2DW12_ALL_INT_SRC, &val, 1)) {
      PBL_LOG_ERR("Could not read ALL_INT_SRC register");
    }
  }

  // Configure INT1
  ret = prv_configure_int1(on, LIS2DW12->state->num_samples > 0U);
  if (!ret) {
    PBL_LOG_ERR("Could not configure INT1");
    return;
  }

  LIS2DW12->state->shake_detection_enabled = on;
  LIS2DW12->state->wu_active = false;
  // Arm/disarm the INT1 liveness watchdog now that shake routing changed
  prv_update_int1_watchdog();

  PBL_LOG_DBG("%s shake detection", on ? "Enabled" : "Disabled");
}

bool accel_get_shake_detection_enabled(void) {
  return LIS2DW12->state->shake_detection_enabled;
}

void accel_set_shake_sensitivity_high(bool sensitivity_high) {
  bool ret;
  uint8_t val;

  if (!LIS2DW12->state->initialized) {
    return;
  }

  val = LIS2DW12_WAKE_UP_THS_WK_THS(sensitivity_high ? CONFIG_ACCEL_LIS2DW12_WK_THS_MIN
                                                     : LIS2DW12->state->wk_ths_curr);
  ret = prv_lis2dw12_write(LIS2DW12_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return;
  }

  PBL_LOG_DBG("Configured shake sensitivity to %s",
          sensitivity_high ? "high" : "normal");
}

void accel_set_shake_sensitivity_percent(uint8_t percent) {
  bool ret;
  uint8_t val;
  uint8_t raw;

  if (!LIS2DW12->state->initialized) {
    return;
  }

  // Reverse mapping: 0 = max sensitivity (MIN threshold), 100 = min sensitivity (MAX threshold)
  // [0, 100] -> [wk_ths_max, wk_ths_min]
  raw = CONFIG_ACCEL_LIS2DW12_WK_THS_MAX -
        (percent * (CONFIG_ACCEL_LIS2DW12_WK_THS_MAX - CONFIG_ACCEL_LIS2DW12_WK_THS_MIN)) / 100U;

  val = LIS2DW12_WAKE_UP_THS_WK_THS(raw);
  ret = prv_lis2dw12_write(LIS2DW12_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return;
  }

  LIS2DW12->state->wk_ths_curr = raw;

  PBL_LOG_DBG("Configured shake sensitivity to %" PRIu8 " (%" PRIu8 ")", percent, raw);
}

void accel_enable_double_tap_detection(bool on) {
  // TODO: Implement
  PBL_LOG_WRN("Double-tap detection not implemented");
}

bool accel_get_double_tap_detection_enabled(void) {
  // TODO: Implement
  return false;
}

void accel_set_rotated(bool rotated) {
  LIS2DW12->state->rotated = rotated;
  PBL_LOG_DBG("Set rotated state to %s", rotated ? "true" : "false");
}
