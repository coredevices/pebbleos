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

PBL_LOG_MODULE_DEFINE(driver_accel_lsm6dso, CONFIG_DRIVER_IMU_LOG_LEVEL);

// Implementation notes:
//
// - Gyroscope is unused; only the accelerometer is configured.
// - Peeking returns the last FIFO sample when sampling is active, otherwise a
//   single-shot measurement is performed.
// - Ultra-low-power mode (XL_ULP_EN) is always used (minimum power). It is
//   enabled once at init while the accelerometer is powered down (as required
//   by the datasheet) and never toggled afterwards. The gyroscope is never
//   enabled, so ULP is allowed.
// - ODR is limited to the [12.5, 208] Hz range (the ULP ODR range).
// - Shake detection uses 12.5Hz when no active sampling is ongoing.
// - Wake-up duration absolute time depends on the ODR, a parameter that can
//   be changed depending on the sampling interval configuration. Value is NOT
//   adjusted automatically when ODR changes, so it is possible to notice
//   sensitivity changes when changing sampling interval.
// - INT1 mixes level (FIFO threshold) and latched (wake-up) sources on a
//   rising-edge EXTI, so a pad latched HIGH yields no further edges: shake
//   events die and the pending interrupt blocks deep sleep. The work handler
//   re-checks the pad after servicing; a watchdog recovers the stream (FIFO
//   cadence while sampling, fixed-rate pad poll in shake-only mode).

// Time to wait after reset (us)
#define LSM6DSO_RESET_TIME_US 5

// DRDY polling parameters for accel_peek single-shot mode
#define LSM6DSO_DRDY_POLL_DELAY_MS   (5)   /* ms between data-ready polls */
#define LSM6DSO_DRDY_POLL_TIMEOUT_MS (100) /* max wait (~5x 20ms at 52Hz ODR) */

// FIFO threshold periods without a FIFO read before the stream is considered
// stalled (>1x to tolerate scheduling jitter), and threshold lower bound
#define LSM6DSO_STALL_MARGIN 2U
#define LSM6DSO_STALL_MIN_MS 1000U

// INT1 watchdog poll rate in shake-only mode (no FIFO cadence to track)
#define LSM6DSO_SHAKE_WDT_PERIOD_S 5U

// Consecutive stuck-high watchdog passes before a full stream recovery
#define LSM6DSO_SHAKE_STUCK_PASSES_MAX 3U

// Scale range for 16-bit two's complement samples
#define LSM6DSO_S16_SCALE_RANGE (1U << (16U - 1U))

// FIFO tag identifying an accelerometer (XL) sample
#define LSM6DSO_FIFO_TAG_XL_NC 0x02U

// Maximum FIFO watermark (WTM[8:0] is 9 bits)
#define LSM6DSO_FIFO_WTM_MAX 511U

// Registers
#define LSM6DSO_FIFO_CTRL1 0x07U
#define LSM6DSO_FIFO_CTRL2 0x08U
#define LSM6DSO_FIFO_CTRL3 0x09U
#define LSM6DSO_FIFO_CTRL4 0x0AU
#define LSM6DSO_INT1_CTRL 0x0DU
#define LSM6DSO_WHO_AM_I 0x0FU
#define LSM6DSO_CTRL1_XL 0x10U
#define LSM6DSO_CTRL3_C 0x12U
#define LSM6DSO_CTRL5_C 0x14U
#define LSM6DSO_CTRL9_XL 0x18U
#define LSM6DSO_ALL_INT_SRC 0x1AU
#define LSM6DSO_STATUS_REG 0x1EU
#define LSM6DSO_OUTX_L_A 0x28U
#define LSM6DSO_FIFO_STATUS1 0x3AU
#define LSM6DSO_FIFO_STATUS2 0x3BU
#define LSM6DSO_TAP_CFG0 0x56U
#define LSM6DSO_TAP_CFG2 0x58U
#define LSM6DSO_WAKE_UP_THS 0x5BU
#define LSM6DSO_WAKE_UP_DUR 0x5CU
#define LSM6DSO_MD1_CFG 0x5EU
#define LSM6DSO_FIFO_DATA_OUT_TAG 0x78U

// WHO_AM_I value
#define LSM6DSO_WHO_AM_I_VAL 0x6CU

// CTRL1_XL fields
#define LSM6DSO_CTRL1_XL_ODR_OFF (0x0U << 4U)
#define LSM6DSO_CTRL1_XL_ODR_12HZ5 (0x1U << 4U)
#define LSM6DSO_CTRL1_XL_ODR_26HZ (0x2U << 4U)
#define LSM6DSO_CTRL1_XL_ODR_52HZ (0x3U << 4U)
#define LSM6DSO_CTRL1_XL_ODR_104HZ (0x4U << 4U)
#define LSM6DSO_CTRL1_XL_ODR_208HZ (0x5U << 4U)
#define LSM6DSO_CTRL1_XL_FS_2G (0x0U << 2U)
#define LSM6DSO_CTRL1_XL_FS_16G (0x1U << 2U)
#define LSM6DSO_CTRL1_XL_FS_4G (0x2U << 2U)
#define LSM6DSO_CTRL1_XL_FS_8G (0x3U << 2U)

// CTRL3_C fields
#define LSM6DSO_CTRL3_C_SW_RESET (1U << 0U)
#define LSM6DSO_CTRL3_C_IF_INC (1U << 2U)
#define LSM6DSO_CTRL3_C_BDU (1U << 6U)

// CTRL5_C fields
#define LSM6DSO_CTRL5_C_XL_ULP_EN (1U << 7U)

// CTRL9_XL fields
#define LSM6DSO_CTRL9_XL_I3C_DISABLE (1U << 1U)

// INT1_CTRL fields
#define LSM6DSO_INT1_CTRL_DRDY_XL (1U << 0U)
#define LSM6DSO_INT1_CTRL_FIFO_TH (1U << 3U)
#define LSM6DSO_INT1_CTRL_FIFO_OVR (1U << 4U)

// ALL_INT_SRC fields
#define LSM6DSO_ALL_INT_SRC_WU_IA (1U << 1U)

// STATUS_REG fields
#define LSM6DSO_STATUS_REG_XLDA (1U << 0U)

// FIFO_CTRL2 fields
#define LSM6DSO_FIFO_CTRL2_WTM8 (1U << 0U)

// FIFO_CTRL3 fields
#define LSM6DSO_FIFO_CTRL3_BDR_XL_12HZ5 0x1U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_26HZ 0x2U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_52HZ 0x3U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_104HZ 0x4U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_208HZ 0x5U

// FIFO_CTRL4 fields
#define LSM6DSO_FIFO_CTRL4_MODE_BYPASS 0x0U
#define LSM6DSO_FIFO_CTRL4_MODE_STREAM 0x6U

// FIFO_STATUS fields
#define LSM6DSO_FIFO_STATUS2_DIFF_HI_MASK 0x03U
#define LSM6DSO_FIFO_STATUS2_FIFO_OVR_IA (1U << 6U)
#define LSM6DSO_FIFO_STATUS2_FIFO_WTM_IA (1U << 7U)

// TAP_CFG0 fields (slope_fds left at 0 to select the slope filter for wake-up)
#define LSM6DSO_TAP_CFG0_LIR (1U << 0U)
#define LSM6DSO_TAP_CFG0_INT_CLR_ON_READ (1U << 6U)

// TAP_CFG2 fields
#define LSM6DSO_TAP_CFG2_INTERRUPTS_ENABLE (1U << 7U)

// WAKE_UP_THS fields
#define LSM6DSO_WAKE_UP_THS_WK_THS_POS 0U
#define LSM6DSO_WAKE_UP_THS_WK_THS_MASK 0x3FU
#define LSM6DSO_WAKE_UP_THS_WK_THS(val) \
  (((val) << LSM6DSO_WAKE_UP_THS_WK_THS_POS) & LSM6DSO_WAKE_UP_THS_WK_THS_MASK)

// WAKE_UP_DUR fields
#define LSM6DSO_WAKE_UP_DUR_WAKE_DUR_POS 5U
#define LSM6DSO_WAKE_UP_DUR_WAKE_DUR_MASK 0x60U
#define LSM6DSO_WAKE_UP_DUR_WAKE_DUR(val) \
  (((val) << LSM6DSO_WAKE_UP_DUR_WAKE_DUR_POS) & LSM6DSO_WAKE_UP_DUR_WAKE_DUR_MASK)

// MD1_CFG fields
#define LSM6DSO_MD1_CFG_INT1_WU (1U << 5U)

////////////////////////////////////////////////////////////////////////////////
// Private
////////////////////////////////////////////////////////////////////////////////

static bool prv_lsm6dso_write(uint8_t reg, const uint8_t *data, uint16_t len) {
  bool ret;

  i2c_use(&LSM6DSO->i2c);
  ret = i2c_write_register_block(&LSM6DSO->i2c, reg, len, data);
  i2c_release(&LSM6DSO->i2c);

  return ret;
}

static bool prv_lsm6dso_read(uint8_t reg, uint8_t *data, uint16_t len) {
  bool ret;

  i2c_use(&LSM6DSO->i2c);
  ret = i2c_read_register_block(&LSM6DSO->i2c, reg, len, data);
  i2c_release(&LSM6DSO->i2c);

  return ret;
}

static uint8_t prv_fs_bits(void) {
  switch (CONFIG_ACCEL_LSM6DSO_SCALE_MG) {
    case 2000U:
      return LSM6DSO_CTRL1_XL_FS_2G;
    case 4000U:
      return LSM6DSO_CTRL1_XL_FS_4G;
    case 8000U:
      return LSM6DSO_CTRL1_XL_FS_8G;
    case 16000U:
      return LSM6DSO_CTRL1_XL_FS_16G;
    default:
      return LSM6DSO_CTRL1_XL_FS_2G;
  }
}

static int16_t prv_raw_to_s16(const uint8_t *raw) {
  return (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8U));
}

static int16_t prv_axis_raw_mg(IMUCoordinateAxis axis, const uint8_t *raw) {
  uint8_t offset;
  int16_t val;

  offset = LSM6DSO->axis_map[axis];

  val = LSM6DSO->axis_dir[axis] *
        (int16_t)(((int32_t)prv_raw_to_s16(&raw[offset * 2U]) *
                   (int32_t)CONFIG_ACCEL_LSM6DSO_SCALE_MG) /
                  (int32_t)LSM6DSO_S16_SCALE_RANGE);

  if (LSM6DSO->state->rotated && (axis == AXIS_X || axis == AXIS_Y)) {
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
static void prv_lsm6dso_process_samples(uint16_t num_samples, uint64_t timestamp_us) {
  // Each FIFO word is a tag byte followed by 6 data bytes; point the batch at the
  // first data byte and let the service step over the tags via the stride.
  int8_t rotate = LSM6DSO->state->rotated ? -1 : 1;
  AccelRawBatch batch = {
    .data = &LSM6DSO->state->raw_sample_buf[1],
    .num_samples = num_samples,
    .stride = LSM6DSO_FIFO_WORD_SIZE_BYTES,
    .axis = {
      [AXIS_X] = {.offset = LSM6DSO->axis_map[AXIS_X] * 2U,
                  .sign = (int8_t)(LSM6DSO->axis_dir[AXIS_X] * rotate)},
      [AXIS_Y] = {.offset = LSM6DSO->axis_map[AXIS_Y] * 2U,
                  .sign = (int8_t)(LSM6DSO->axis_dir[AXIS_Y] * rotate)},
      [AXIS_Z] = {.offset = LSM6DSO->axis_map[AXIS_Z] * 2U, .sign = LSM6DSO->axis_dir[AXIS_Z]},
    },
    .scale_num = CONFIG_ACCEL_LSM6DSO_SCALE_MG,
    .scale_den = LSM6DSO_S16_SCALE_RANGE,
    .first_timestamp_us = timestamp_us,
    .sampling_interval_us = LSM6DSO->state->sampling_interval_us,
  };

  accel_cb_new_samples(&batch);
}

static uint8_t prv_get_bdr(uint32_t sampling_interval_us) {
  if (sampling_interval_us >= 80000UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_12HZ5;
  } else if (sampling_interval_us >= 38461UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_26HZ;
  } else if (sampling_interval_us >= 19230UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_52HZ;
  } else if (sampling_interval_us >= 9615UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_104HZ;
  } else {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_208HZ;
  }
}

static bool prv_lsm6dso_enable_fifo(uint16_t num_samples) {
  bool ret;
  uint8_t val;
  uint16_t wtm;

  // Bypass mode to flush the FIFO
  val = LSM6DSO_FIFO_CTRL4_MODE_BYPASS;
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL4, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL4 register");
    return ret;
  }

  // Watermark threshold WTM[8:0]: low 8 bits in FIFO_CTRL1, MSB in FIFO_CTRL2
  wtm = MIN(num_samples, LSM6DSO_FIFO_WTM_MAX);

  val = (uint8_t)(wtm & 0xFFU);
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL1, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL1 register");
    return ret;
  }

  val = (wtm >> 8U) & LSM6DSO_FIFO_CTRL2_WTM8;
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL2, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL2 register");
    return ret;
  }

  // Accelerometer batch data rate (gyro left not batched)
  val = prv_get_bdr(LSM6DSO->state->sampling_interval_us);
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL3, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL3 register");
    return ret;
  }

  // Continuous (stream) mode
  val = LSM6DSO_FIFO_CTRL4_MODE_STREAM;
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL4, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL4 register");
    return ret;
  }

  PBL_LOG_DBG("FIFO enabled with threshold %" PRIu16, wtm);

  return true;
}

//! Salvage and dispatch any samples queued in the FIFO
static void prv_lsm6dso_drain_fifo(void) {
  uint8_t status[2];
  uint16_t samples;

  if (!prv_lsm6dso_read(LSM6DSO_FIFO_STATUS1, status, 2)) {
    PBL_LOG_ERR("Could not read FIFO_STATUS registers");
    return;
  }

  samples = (((uint16_t)(status[1] & LSM6DSO_FIFO_STATUS2_DIFF_HI_MASK)) << 8U) | status[0];
  samples = MIN(samples, LSM6DSO_FIFO_SIZE);
  if (samples == 0U) {
    return;
  }

  if (!prv_lsm6dso_read(LSM6DSO_FIFO_DATA_OUT_TAG, LSM6DSO->state->raw_sample_buf,
                        samples * LSM6DSO_FIFO_WORD_SIZE_BYTES)) {
    PBL_LOG_ERR("Failed to read samples");
    return;
  }

  prv_lsm6dso_process_samples(samples, prv_get_curr_system_time_us());
  LSM6DSO->state->last_fifo_read_tick = rtc_get_ticks();
}

static void prv_lsm6dso_recover(void);
static uint32_t prv_ms_since_last_fifo_read(void);

//! INT1 servicing pass kind, logged by the no-action diagnostic
typedef enum {
  Lsm6dsoInt1PassInitial = 0,
  Lsm6dsoInt1PassRequeue = 1,
  Lsm6dsoInt1PassRetry = 2,
} Lsm6dsoInt1Pass;

//! Single INT1 servicing pass; returns true if any INT source was handled.
//! fifo_progress is set when FIFO data was consumed (samples or overrun).
static bool prv_lsm6dso_service_int1(Lsm6dsoInt1Pass pass, bool *fifo_progress) {
  bool ret;
  uint8_t fifo_status[2] = {0U, 0U};
  uint8_t all_int_src = 0U;
  bool action_taken = false;
  bool fifo_overrun = false;
  bool shake = false;
  uint16_t samples = 0U;
  uint64_t timestamp_us = 0U;

  // Read all device registers back-to-back to keep the I2C critical section
  // short, then convert/dispatch below. The sample processing has no dependency
  // on these reads, so deferring it avoids stretching the gap between the FIFO
  // read and the ALL_INT_SRC read if this task gets preempted mid-handler.
  if (LSM6DSO->state->num_samples > 0U) {
    // WTM/OVR flags and DIFF must come from one burst read: a split read can
    // catch the FIFO counter mid-update and see WTM_IA set with DIFF reading 0
    ret = prv_lsm6dso_read(LSM6DSO_FIFO_STATUS1, fifo_status, 2);
    if (!ret) {
      PBL_LOG_ERR("Could not read FIFO_STATUS registers");
      return false;
    }

    if ((fifo_status[1] & LSM6DSO_FIFO_STATUS2_FIFO_OVR_IA) != 0U) {
      fifo_overrun = true;
    } else if ((fifo_status[1] & LSM6DSO_FIFO_STATUS2_FIFO_WTM_IA) != 0U) {
      samples = (((uint16_t)(fifo_status[1] & LSM6DSO_FIFO_STATUS2_DIFF_HI_MASK)) << 8U) |
                fifo_status[0];
      if (samples > LSM6DSO_FIFO_SIZE) {
        samples = LSM6DSO_FIFO_SIZE;
      }

      if (samples > 0U) {
        if (!prv_lsm6dso_read(LSM6DSO_FIFO_DATA_OUT_TAG, LSM6DSO->state->raw_sample_buf,
                              samples * LSM6DSO_FIFO_WORD_SIZE_BYTES)) {
          PBL_LOG_ERR("Failed to read samples");
          return false;
        }
        timestamp_us = prv_get_curr_system_time_us();
        LSM6DSO->state->last_fifo_read_tick = rtc_get_ticks();
      }
    }
  }

  if (LSM6DSO->state->shake_detection_enabled) {
    ret = prv_lsm6dso_read(LSM6DSO_ALL_INT_SRC, &all_int_src, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read ALL_INT_SRC register");
      return false;
    }

    shake = (all_int_src & LSM6DSO_ALL_INT_SRC_WU_IA) != 0U;
  }

  if (fifo_overrun) {
    PBL_LOG_WRN("FIFO overrun detected, recovering");
    prv_lsm6dso_recover();
    action_taken = true;
    *fifo_progress = true;
  } else if (samples > 0U) {
    prv_lsm6dso_process_samples(samples, timestamp_us);
    action_taken = true;
    *fifo_progress = true;
  }

  if (shake) {
    // WU_IA stays set while the wake-up condition persists (LIR), so only
    // dispatch on its assertion to avoid flooding events
    if (!LSM6DSO->state->wu_active) {
      PBL_LOG_DBG("Shake detected");
      // TODO: provide more info about the shake (axis, direction, etc.) or
      // refactor shake to be non-dimensional
      accel_cb_shake_detected(AXIS_Z, 0);
    }
    action_taken = true;
  }
  LSM6DSO->state->wu_active = shake;

  if (!action_taken) {
    // Registers not read this pass (gated on num_samples/shake state) log as 0
    PBL_LOG_WRN("INT1 triggered but no action taken (FIFO_STATUS2 0x%02" PRIx8
                " ALL_INT_SRC 0x%02" PRIx8 " num_samples %" PRIu16 " shake_en %d pass %u"
                " last_read %" PRIu32 "ms ago)",
                fifo_status[1], all_int_src, LSM6DSO->state->num_samples,
                LSM6DSO->state->shake_detection_enabled, (unsigned int)pass,
                prv_ms_since_last_fifo_read());
  }

  return action_taken;
}

static void prv_lsm6dso_int1_work_handler(void) {
  bool fifo_progress = false;
  // Best-effort pass tagging: an ISR enqueue racing a pending requeue gets
  // labeled as a requeue; diagnostic only
  Lsm6dsoInt1Pass pass =
      LSM6DSO->state->int1_requeued ? Lsm6dsoInt1PassRequeue : Lsm6dsoInt1PassInitial;
  bool action_taken;

  LSM6DSO->state->int1_requeued = false;
  action_taken = prv_lsm6dso_service_int1(pass, &fifo_progress);

  // Sources asserting while the pad is high produce no new edge: requeue on
  // FIFO progress, recover when nothing was serviced (stuck pad). A pad held
  // by a persistent wake-up condition is left to the stall watchdog.
  if (!gpio_input_read(&LSM6DSO->int1_in)) {
    return;
  }

  if (fifo_progress) {
    LSM6DSO->state->int1_requeued = true;
    accel_offload_work(prv_lsm6dso_int1_work_handler);
    return;
  }

  if (action_taken) {
    return;
  }

  // A source asserting between the status reads and the pad re-check is
  // indistinguishable from a stuck pad, and recovering would discard it.
  // Retry the servicing pass once (a latched wake-up or a fresh FIFO
  // threshold is visible on re-read) and only recover if the pad is still
  // high with nothing serviced.
  action_taken = prv_lsm6dso_service_int1(Lsm6dsoInt1PassRetry, &fifo_progress);
  if (fifo_progress) {
    LSM6DSO->state->int1_requeued = true;
    accel_offload_work(prv_lsm6dso_int1_work_handler);
  } else if (!action_taken && gpio_input_read(&LSM6DSO->int1_in)) {
    prv_lsm6dso_recover();
  }
}

static void prv_lsm6dso_int1_irq_handler(bool *should_context_switch) {
  // A rising edge proves the pad was low, i.e. the wake-up source deasserted
  LSM6DSO->state->wu_active = false;
  // ... which also breaks any stuck-high streak the watchdog counted
  LSM6DSO->state->shake_stuck_passes = 0U;
  accel_offload_work_from_isr(prv_lsm6dso_int1_work_handler, should_context_switch);
}

static bool prv_configure_odr(uint32_t sampling_interval_us, bool shake_detection_enabled) {
  uint8_t val;
  bool ret;

  // If shake detection is enabled, ensure a minimum ODR of 12.5Hz (80ms)
  if (shake_detection_enabled && (sampling_interval_us == 0UL)) {
    sampling_interval_us = 80000UL;
  }

  if (sampling_interval_us == 0U) {
    val = LSM6DSO_CTRL1_XL_ODR_OFF;
    sampling_interval_us = 0UL;
  } else if (sampling_interval_us >= 80000UL) {
    val = LSM6DSO_CTRL1_XL_ODR_12HZ5;
    sampling_interval_us = 80000UL;
  } else if (sampling_interval_us >= 38461UL) {
    val = LSM6DSO_CTRL1_XL_ODR_26HZ;
    sampling_interval_us = 38461UL;
  } else if (sampling_interval_us >= 19230UL) {
    val = LSM6DSO_CTRL1_XL_ODR_52HZ;
    sampling_interval_us = 19230UL;
  } else if (sampling_interval_us >= 9615UL) {
    val = LSM6DSO_CTRL1_XL_ODR_104HZ;
    sampling_interval_us = 9615UL;
  } else {
    val = LSM6DSO_CTRL1_XL_ODR_208HZ;
    sampling_interval_us = 4807UL;
  }

  val |= prv_fs_bits();

  PBL_LOG_DBG("Configuring ODR to %" PRIu32 " ms (%" PRIu32 " mHz)",
          sampling_interval_us / 1000UL,
          sampling_interval_us > 0UL ? 1000000000UL / sampling_interval_us : 0UL);

  ret = prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &val, 1);
  if (!ret) {
    return ret;
  }

  // Allow the accelerometer to stabilize after an ODR/power-mode change
  if (val != (LSM6DSO_CTRL1_XL_ODR_OFF | prv_fs_bits())) {
    psleep(10);
  }

  LSM6DSO->state->sampling_interval_us = sampling_interval_us;

  return true;
}

static bool prv_configure_int1(bool shake_detection_enabled, bool fifo_enabled) {
  bool ret;
  uint8_t int1_ctrl;
  uint8_t md1_cfg;
  uint8_t tap_cfg2;

  int1_ctrl = 0U;
  md1_cfg = 0U;

  if (fifo_enabled) {
    int1_ctrl |= LSM6DSO_INT1_CTRL_FIFO_TH | LSM6DSO_INT1_CTRL_FIFO_OVR;
  }

  if (shake_detection_enabled) {
    md1_cfg |= LSM6DSO_MD1_CFG_INT1_WU;
  }

  ret = prv_lsm6dso_write(LSM6DSO_INT1_CTRL, &int1_ctrl, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write INT1_CTRL register");
    return ret;
  }

  PBL_LOG_DBG("INT1_CTRL configured: %02" PRIx8, int1_ctrl);

  // Basic interrupt functions must be enabled for the wake-up event routing
  tap_cfg2 = shake_detection_enabled ? LSM6DSO_TAP_CFG2_INTERRUPTS_ENABLE : 0U;
  ret = prv_lsm6dso_write(LSM6DSO_TAP_CFG2, &tap_cfg2, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write TAP_CFG2 register");
    return ret;
  }

  ret = prv_lsm6dso_write(LSM6DSO_MD1_CFG, &md1_cfg, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write MD1_CFG register");
    return ret;
  }

  PBL_LOG_DBG("MD1_CFG configured: %02" PRIx8, md1_cfg);

  return true;
}

//! Recover a dead INT1/FIFO stream by re-asserting ODR, FIFO and INT routing.
//! Routing is quiesced first so a latched-high pad produces a fresh edge.
static void prv_lsm6dso_recover(void) {
  uint8_t val;

  LSM6DSO->state->num_recoveries++;
  PBL_ANALYTICS_ADD(accel_stream_recovery_count, 1);
  PBL_LOG_WRN("Recovering accel stream (count %" PRIu32 ")",
          LSM6DSO->state->num_recoveries);

  if (!prv_configure_int1(false, false)) {
    return;
  }

  // Salvage queued samples
  if (LSM6DSO->state->num_samples > 0U) {
    prv_lsm6dso_drain_fifo();
  }

  // Clear any latched function INT source while routing is quiesced
  if (!prv_lsm6dso_read(LSM6DSO_ALL_INT_SRC, &val, 1)) {
    PBL_LOG_ERR("Could not read ALL_INT_SRC register");
    return;
  }

  if (val != 0U) {
    PBL_LOG_WRN("Discarded latched INT sources during recovery (ALL_INT_SRC 0x%02" PRIx8 ")", val);
  }

  if (!prv_configure_odr(LSM6DSO->state->sampling_interval_us,
                         LSM6DSO->state->shake_detection_enabled)) {
    PBL_LOG_ERR("Could not configure ODR");
    return;
  }

  if (LSM6DSO->state->num_samples > 0U) {
    if (!prv_lsm6dso_enable_fifo(LSM6DSO->state->num_samples)) {
      return;
    }
  }

  if (!prv_configure_int1(LSM6DSO->state->shake_detection_enabled,
                          LSM6DSO->state->num_samples > 0U)) {
    return;
  }

  LSM6DSO->state->wu_active = false;
  LSM6DSO->state->last_fifo_read_tick = rtc_get_ticks();
}

static uint32_t prv_ms_since_last_fifo_read(void) {
  RtcTicks ticks = rtc_get_ticks() - LSM6DSO->state->last_fifo_read_tick;
  return (uint32_t)((ticks * 1000) / RTC_TICKS_HZ);
}

static uint32_t prv_stall_threshold_ms(void) {
  return MAX(LSM6DSO_STALL_MARGIN * LSM6DSO->state->int1_period_ms,
             LSM6DSO_STALL_MIN_MS);
}

//! Arm/disarm the INT1 stall watchdog to match the active INT1 sources.
//! Call after num_samples or shake_detection_enabled changes.
static void prv_update_int1_watchdog(void) {
  regular_timer_remove_callback(&LSM6DSO->state->int1_wdt_timer);
  LSM6DSO->state->shake_stuck_passes = 0U;

  if (LSM6DSO->state->num_samples > 0U) {
    LSM6DSO->state->last_fifo_read_tick = rtc_get_ticks();
    LSM6DSO->state->int1_period_ms =
        (LSM6DSO->state->sampling_interval_us * LSM6DSO->state->num_samples) / 1000;
    regular_timer_add_multisecond_callback(
        &LSM6DSO->state->int1_wdt_timer,
        MAX(DIVIDE_CEIL(LSM6DSO->state->int1_period_ms, 1000UL), 1U));
  } else if (LSM6DSO->state->shake_detection_enabled) {
    regular_timer_add_multisecond_callback(&LSM6DSO->state->int1_wdt_timer,
                                           LSM6DSO_SHAKE_WDT_PERIOD_S);
  }
}

//! Shared by the INT1 watchdog and the accel_peek staleness trigger;
//! re-validates the stall at execution time.
static void prv_stall_check_work_cb(void) {
  uint32_t ms_since_last_read;

  LSM6DSO->state->recovery_pending = false;

  // Sampling may have stopped between scheduling and execution
  if (LSM6DSO->state->num_samples == 0U) {
    // Shake-only mode: re-run the servicing pass if the pad is stuck high
    // (reading the INT source clears the latch); escalate to a full recovery
    // after consecutive passes that never release it.
    if (LSM6DSO->state->shake_detection_enabled &&
        gpio_input_read(&LSM6DSO->int1_in)) {
      prv_lsm6dso_int1_work_handler();
      // A pad released by the pass is healthy; count only a still-high pad
      if (!gpio_input_read(&LSM6DSO->int1_in)) {
        LSM6DSO->state->shake_stuck_passes = 0U;
      } else if (++LSM6DSO->state->shake_stuck_passes >= LSM6DSO_SHAKE_STUCK_PASSES_MAX) {
        PBL_LOG_WRN("INT1 pad stuck high for %" PRIu8 " shake watchdog passes, recovering",
                    LSM6DSO->state->shake_stuck_passes);
        prv_lsm6dso_recover();
        LSM6DSO->state->shake_stuck_passes = 0U;
      }
    } else {
      LSM6DSO->state->shake_stuck_passes = 0U;
    }
    return;
  }

  ms_since_last_read = prv_ms_since_last_fifo_read();
  if (ms_since_last_read < prv_stall_threshold_ms()) {
    return;
  }

  PBL_LOG_WRN("FIFO stream stalled for %" PRIu32 " ms", ms_since_last_read);
  prv_lsm6dso_recover();
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
  ret = prv_lsm6dso_read(LSM6DSO_WHO_AM_I, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read WHO_AM_I register");
    return;
  }

  if (val != LSM6DSO_WHO_AM_I_VAL) {
    PBL_LOG_ERR("Unexpected id: 0x%02X!=0x%02X", val, LSM6DSO_WHO_AM_I_VAL);
    return;
  }

  // Perform a software reset (so we can rely on defaults)
  val = LSM6DSO_CTRL3_C_SW_RESET;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL3_C, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL3_C register");
    return;
  }

  delay_us(LSM6DSO_RESET_TIME_US);

  do {
    ret = prv_lsm6dso_read(LSM6DSO_CTRL3_C, &val, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read CTRL3_C register");
      return;
    }
  } while ((val & LSM6DSO_CTRL3_C_SW_RESET) != 0U);

  // Enable register auto-increment and block data update
  val = LSM6DSO_CTRL3_C_IF_INC | LSM6DSO_CTRL3_C_BDU;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL3_C, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL3_C register");
    return;
  }

  // Disable I3C interface
  val = LSM6DSO_CTRL9_XL_I3C_DISABLE;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL9_XL, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL9_XL register");
    return;
  }

  // Ultra-low-power mode (accelerometer still powered down here, as required to
  // toggle XL_ULP_EN). Gyroscope stays in power-down, so ULP is allowed.
  val = LSM6DSO_CTRL5_C_XL_ULP_EN;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL5_C, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL5_C register");
    return;
  }

  // Configure scale (ODR off until sampling is requested)
  val = LSM6DSO_CTRL1_XL_ODR_OFF | prv_fs_bits();
  ret = prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL1_XL register");
    return;
  }

  // Slope filter for wake-up, latch interrupts and clear them on read
  val = LSM6DSO_TAP_CFG0_LIR | LSM6DSO_TAP_CFG0_INT_CLR_ON_READ;
  ret = prv_lsm6dso_write(LSM6DSO_TAP_CFG0, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write TAP_CFG0 register");
    return;
  }

  // Configure wake-up threshold defaults
  val = LSM6DSO_WAKE_UP_DUR_WAKE_DUR(CONFIG_ACCEL_LSM6DSO_WK_DUR_DEFAULT);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_DUR, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_DUR register");
    return;
  }

  val = LSM6DSO_WAKE_UP_THS_WK_THS(CONFIG_ACCEL_LSM6DSO_WK_THS_DEFAULT);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return;
  }

  LSM6DSO->state->wk_ths_curr = CONFIG_ACCEL_LSM6DSO_WK_THS_DEFAULT;

  // Enable INT1 external interrupt
  exti_configure_pin(LSM6DSO->int1, ExtiTrigger_Rising, prv_lsm6dso_int1_irq_handler);
  exti_enable(LSM6DSO->int1);

  LSM6DSO->state->int1_wdt_timer.cb = prv_int1_wdt_cb;
  LSM6DSO->state->initialized = true;
}

uint32_t accel_set_sampling_interval(uint32_t interval_us) {
  if (!LSM6DSO->state->initialized) {
    // Just pretend we can achieve any requested interval
    LSM6DSO->state->sampling_interval_us = interval_us;
  } else {
    // FIXME: we should technically stop and drain the FIFO here, otherwise
    // we may report existing samples in the FIFO buffer with an incorrect timestamp

    const uint32_t prev_interval_us = LSM6DSO->state->sampling_interval_us;
    if (!prv_configure_odr(interval_us, LSM6DSO->state->shake_detection_enabled)) {
      PBL_LOG_ERR("Could not configure ODR");
    }

    // Re-arm the watchdog only on an actual ODR change (a re-arm resets the
    // stall tracking, so repeat calls must not defer it)
    if (LSM6DSO->state->sampling_interval_us != prev_interval_us) {
      prv_update_int1_watchdog();
    }
  }

  PBL_LOG_DBG("Set sampling interval to %" PRIu32 " us",
          LSM6DSO->state->sampling_interval_us);

  return LSM6DSO->state->sampling_interval_us;
}

uint32_t accel_get_sampling_interval(void) {
  return LSM6DSO->state->sampling_interval_us;
}

uint32_t accel_get_max_num_samples(void) {
  return LSM6DSO_FIFO_THRESHOLD;
}

void accel_set_num_samples(uint32_t num_samples) {
  bool ret;
  uint8_t val;

  if (!LSM6DSO->state->initialized) {
    return;
  }

  // Limit to FIFO threshold
  if (num_samples > LSM6DSO_FIFO_THRESHOLD) {
    num_samples = LSM6DSO_FIFO_THRESHOLD;
  }

  // Disable all INT1 before changing FIFO threshold
  prv_configure_int1(false, false);

  // Config change invalidates any queued stall check; re-arm the peek trigger
  LSM6DSO->state->recovery_pending = false;

  // Salvage queued samples
  if (LSM6DSO->state->num_samples > 0U) {
    prv_lsm6dso_drain_fifo();
  }

  if (num_samples == 0U) {
    // Bypass FIFO (disable)
    val = LSM6DSO_FIFO_CTRL4_MODE_BYPASS;
    if (!prv_lsm6dso_write(LSM6DSO_FIFO_CTRL4, &val, 1)) {
      PBL_LOG_ERR("Could not write FIFO_CTRL4 register");
    }
  } else {
    // Configure FIFO in continuous mode with threshold
    ret = prv_lsm6dso_enable_fifo((uint16_t)num_samples);
    if (!ret) {
      PBL_LOG_ERR("Could not enable FIFO");
      return;
    }
  }

  // Re-configure INT1
  ret = prv_configure_int1(LSM6DSO->state->shake_detection_enabled, num_samples > 0U);
  if (!ret) {
    PBL_LOG_ERR("Could not configure INT1");
    return;
  }

  LSM6DSO->state->num_samples = num_samples;
  // Arm/disarm the INT1 liveness watchdog for the new source configuration
  prv_update_int1_watchdog();

  PBL_LOG_DBG("Set number of samples to %" PRIu32, num_samples);
}

int accel_peek(AccelDriverSample *data) {
  int err = 0;
  bool ret;
  uint8_t ctrl1;
  uint8_t ctrl1_bck;
  uint8_t status;
  uint8_t raw[LSM6DSO_SAMPLE_SIZE_BYTES];

  if (!LSM6DSO->state->initialized) {
    return E_ERROR;
  }

  // If sampling is active, the FIFO batches samples for subscribers, so the
  // cached sample can be a full watermark period old. Read the output
  // registers instead: they update at ODR independently of the FIFO.
  if (LSM6DSO->state->num_samples > 0U) {
    // Self-heal: a stalled stream would otherwise freeze peek data forever
    if (!LSM6DSO->state->recovery_pending &&
        (prv_ms_since_last_fifo_read() >= prv_stall_threshold_ms())) {
      LSM6DSO->state->recovery_pending = true;
      accel_offload_work(prv_stall_check_work_cb);
    }

    ret = prv_lsm6dso_read(LSM6DSO_OUTX_L_A, raw, sizeof(raw));
    if (!ret) {
      PBL_LOG_ERR("Failed to read sample");
      return E_ERROR;
    }

    prv_raw_to_mg(raw, data);
    data->timestamp_us = prv_get_curr_system_time_us();

    return 0;
  }

  // Save CTRL1_XL
  ret = prv_lsm6dso_read(LSM6DSO_CTRL1_XL, &ctrl1_bck, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read CTRL1_XL register");
    return E_ERROR;
  }

  // Enable continuous conversion at 52Hz to obtain a single measurement
  ctrl1 = LSM6DSO_CTRL1_XL_ODR_52HZ | prv_fs_bits();
  ret = prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &ctrl1, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL1_XL register");
    return E_ERROR;
  }

  // Poll for data ready (timeout after 100ms, ~5x the expected 20ms at 52Hz ODR)
  uint32_t elapsed_ms = 0;
  do {
    ret = prv_lsm6dso_read(LSM6DSO_STATUS_REG, &status, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read STATUS_REG register");
      err = E_ERROR;
      goto end;
    }
    if ((status & LSM6DSO_STATUS_REG_XLDA) == 0U) {
      if (elapsed_ms >= LSM6DSO_DRDY_POLL_TIMEOUT_MS) {
        PBL_LOG_ERR("DRDY timeout after %" PRIu32 " ms", elapsed_ms);
        err = E_ERROR;
        goto end;
      }
      psleep(LSM6DSO_DRDY_POLL_DELAY_MS);
      elapsed_ms += LSM6DSO_DRDY_POLL_DELAY_MS;
    }
  } while ((status & LSM6DSO_STATUS_REG_XLDA) == 0U);

  // Read sample
  ret = prv_lsm6dso_read(LSM6DSO_OUTX_L_A, raw, sizeof(raw));
  if (!ret) {
    PBL_LOG_ERR("Failed to read sample");
    err = E_ERROR;
    goto end;
  }

  // Convert to mg and populate timestamp
  prv_raw_to_mg(raw, data);
  data->timestamp_us = prv_get_curr_system_time_us();

end:
  // Restore CTRL1_XL (back to previous state, e.g. power-down or shake ODR)
  (void)prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &ctrl1_bck, 1);

  return err;
}

void accel_enable_shake_detection(bool on) {
  bool ret;

  if (!LSM6DSO->state->initialized) {
    return;
  }

  // Configure ODR (use current interval, will be adjusted if < 12.5Hz)
  ret = prv_configure_odr(LSM6DSO->state->sampling_interval_us, on);
  if (!ret) {
    PBL_LOG_ERR("Could not configure ODR");
    return;
  }

  if (on) {
    uint8_t val;

    // WU_IA may be latched from a previous enablement or an ODR startup
    // transient; clear it so a phantom shake is not dispatched on enable
    if (!prv_lsm6dso_read(LSM6DSO_ALL_INT_SRC, &val, 1)) {
      PBL_LOG_ERR("Could not read ALL_INT_SRC register");
    }
  }

  // Configure INT1
  ret = prv_configure_int1(on, LSM6DSO->state->num_samples > 0U);
  if (!ret) {
    PBL_LOG_ERR("Could not configure INT1");
    return;
  }

  LSM6DSO->state->shake_detection_enabled = on;
  LSM6DSO->state->wu_active = false;
  // Arm/disarm the INT1 liveness watchdog now that shake routing changed
  prv_update_int1_watchdog();

  PBL_LOG_DBG("%s shake detection", on ? "Enabled" : "Disabled");
}

bool accel_get_shake_detection_enabled(void) {
  return LSM6DSO->state->shake_detection_enabled;
}

void accel_set_shake_sensitivity_high(bool sensitivity_high) {
  bool ret;
  uint8_t val;

  if (!LSM6DSO->state->initialized) {
    return;
  }

  val = LSM6DSO_WAKE_UP_THS_WK_THS(sensitivity_high ? CONFIG_ACCEL_LSM6DSO_WK_THS_MIN
                                                    : LSM6DSO->state->wk_ths_curr);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_THS, &val, 1);
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

  if (!LSM6DSO->state->initialized) {
    return;
  }

  // Reverse mapping: 0 = max sensitivity (MIN threshold), 100 = min sensitivity (MAX threshold)
  // [0, 100] -> [wk_ths_max, wk_ths_min]
  raw = CONFIG_ACCEL_LSM6DSO_WK_THS_MAX -
        (percent * (CONFIG_ACCEL_LSM6DSO_WK_THS_MAX - CONFIG_ACCEL_LSM6DSO_WK_THS_MIN)) / 100U;

  val = LSM6DSO_WAKE_UP_THS_WK_THS(raw);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return;
  }

  LSM6DSO->state->wk_ths_curr = raw;

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
  LSM6DSO->state->rotated = rotated;
  PBL_LOG_DBG("Set rotated state to %s", rotated ? "true" : "false");
}
