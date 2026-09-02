/* SPDX-FileCopyrightText: 2025 Matthew Wardrop */
/* SPDX-FileCopyrightText: 2025 Bob Wei */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/i2c.h>
#include <pbl/drivers/mag.h>
#include "kernel/events.h"
#include "kernel/util/sleep.h"
#include "pbl/kernel/mutex.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/util/math.h"

#include <pbl/drivers/imu/mmc5603nj/mmc5603nj.h>
#include <pbl/drivers/imu/mmc5603nj/registers.h>

PBL_LOG_MODULE_DEFINE(driver_mag_mmc5603nj, CONFIG_DRIVER_IMU_LOG_LEVEL);

// Forward declarations of private methods
static bool prv_mmc5603nj_read(uint8_t reg_addr, uint8_t data_len, uint8_t *data);
static bool prv_mmc5603nj_write(uint8_t reg_addr, uint8_t data);
static bool prv_mmc5603nj_init(void);
static bool prv_mmc5603nj_check_whoami(void);
static bool prv_mmc5603nj_reset(void);
static bool prv_mmc5603nj_set_sample_rate_hz(uint8_t rate_hz);
#ifndef CONFIG_RECOVERY_FW
static bool prv_configure_polling(void);
static void prv_mmc5603nj_polling_callback(void *data);
#endif
static bool prv_mmc5603nj_is_data_ready(void);
static MagReadStatus prv_mmc5603nj_get_sample(MagData *sample);
typedef enum {
  X_AXIS = 0,
  Y_AXIS = 1,
  Z_AXIS = 2,
} axis_t;
static int16_t prv_get_axis_projection(axis_t axis, int16_t *raw_vector);

// Runtime state
static bool s_initialized = false;
static int s_use_refcount = 0;
static PBL_MUTEX_DEFINE(s_mag_mutex);
static uint8_t s_sample_rate_hz = 0;
#ifndef CONFIG_RECOVERY_FW
static TimerID s_polling_timer = TIMER_INVALID_ID;
static uint16_t s_polling_interval_ms = 0;
#endif
static bool s_measurement_ready = false;

// watch rotation
static bool s_rotated_180 = false;

// MMC5603NJ entrypoints

void mag_init(void) {
  if (prv_mmc5603nj_init()) {
    PBL_LOG_DBG("MMC5603NJ: Initialization complete");
  } else {
    PBL_LOG_ERR("MMC5603NJ: Initialization failed");
  }
}

// mag.h implementation

void mag_use(void) {
  PBL_ASSERTN(s_initialized);
  pbl_mutex_lock(&s_mag_mutex, PBL_FOREVER);
  ++s_use_refcount;
  pbl_mutex_unlock(&s_mag_mutex);
}

void mag_start_sampling(void) {
  mag_use();
  mag_change_sample_rate(MagSampleRate5Hz);
}

void mag_release(void) {
  PBL_ASSERTN(s_initialized);
  pbl_mutex_lock(&s_mag_mutex, PBL_FOREVER);
  PBL_ASSERTN(s_use_refcount != 0);
  --s_use_refcount;
  if (s_use_refcount == 0) {
    if (!prv_mmc5603nj_set_sample_rate_hz(0)) {
      PBL_LOG_ERR("MMC5603NJ: Failed to disable sensor on release");
    }
  }
  pbl_mutex_unlock(&s_mag_mutex);
}

// callers responsibility to know if there is valid data to be read
MagReadStatus mag_read_data(MagData *data) {
  pbl_mutex_lock(&s_mag_mutex, PBL_FOREVER);
  MagReadStatus rv = prv_mmc5603nj_get_sample(data);
  pbl_mutex_unlock(&s_mag_mutex);
  return rv;
}

bool mag_change_sample_rate(MagSampleRate rate) {
  pbl_mutex_lock(&s_mag_mutex, PBL_FOREVER);

  if (s_use_refcount == 0) {
    pbl_mutex_unlock(&s_mag_mutex);
    return true;
  }

  uint8_t rate_hz = 0;
  switch (rate) {
    case MagSampleRate20Hz:
      rate_hz = 20;
      break;
    case MagSampleRate5Hz:
      rate_hz = 5;
      break;
    default:
      pbl_mutex_unlock(&s_mag_mutex);
      return false;
  }

  bool rv = prv_mmc5603nj_set_sample_rate_hz(rate_hz);
  pbl_mutex_unlock(&s_mag_mutex);
  return rv;
}

// I2C read/write helpers

static bool prv_mmc5603nj_read(uint8_t reg_addr, uint8_t data_len, uint8_t *data) {
  i2c_use(I2C_MMC5603NJ);
  bool rv = i2c_write_block(I2C_MMC5603NJ, 1, &reg_addr);
  if (!rv) {
    PBL_LOG_ERR("MMC5603NJ: I2C write failed for register 0x%02x", reg_addr);
  }
  rv = i2c_read_block(I2C_MMC5603NJ, data_len, data);
  if (!rv) {
    PBL_LOG_ERR("MMC5603NJ: I2C data read failed for register 0x%02x", reg_addr);
  }
  i2c_release(I2C_MMC5603NJ);
  return rv;
}

static bool prv_mmc5603nj_write(uint8_t reg_addr, uint8_t data) {
  i2c_use(I2C_MMC5603NJ);
  uint8_t d[2] = {reg_addr, data};
  bool rv = i2c_write_block(I2C_MMC5603NJ, 2, d);
  if (!rv) {
    PBL_LOG_ERR("MMC5603NJ: I2C write failed for register 0x%02x", reg_addr);
  }
  i2c_release(I2C_MMC5603NJ);
  return rv;
}

// Initialization

static bool prv_mmc5603nj_init(void) {
  if (s_initialized) {
    return true;
  }

  if (!prv_mmc5603nj_check_whoami()) {
    PBL_LOG_ERR("MMC5603NJ: WHO_AM_I check failed. Wrong device?");
    return false;
  }

  // Reset the device
  if (!prv_mmc5603nj_reset()) {
    PBL_LOG_ERR("MMC5603NJ: Failed to reset");
    return false;
  }

  s_initialized = true;
  return true;
}

// Ask the compass for a 8-bit value that's programmed into the IC at the
// factory. Useful as a sanity check to make sure everything came up properly.
bool prv_mmc5603nj_check_whoami(void) {
  uint8_t whoami = 0;
  if (!prv_mmc5603nj_read(MMC5603NJ_REG_WHO_AM_I, 1, &whoami)) {
    return false;
  }
  return (whoami == MMC5603NJ_WHO_AM_I_VALUE);
}

static bool prv_mmc5603nj_reset(void) {
  // Software reset
  if (!prv_mmc5603nj_write(MMC5603NJ_REG_CTRL1,
                           MMC5603NJ_CTRL1_BANDWIDTH_6ms6 | MMC5603NJ_CTRL1_SW_RESET)) {
    PBL_LOG_ERR("MMC5603NJ: Failed to reset device.");
    return false;
  }
  psleep(MMC5603NJ_SW_RESET_DELAY_MS);

  // Set operation
  if (!prv_mmc5603nj_write(MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_DO_SET)) {
    PBL_LOG_ERR("MMC5603NJ: Failed to set coils.");
    return false;
  }
  psleep(MMC5603NJ_SET_DELAY_MS);

  // Reset operation
  if (!prv_mmc5603nj_write(MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_DO_RESET)) {
    PBL_LOG_ERR("MMC5603NJ: Failed to reset coils.");
    return false;
  }
  psleep(MMC5603NJ_SET_DELAY_MS);
  return true;
}

// Configure ODR

bool prv_mmc5603nj_set_sample_rate_hz(uint8_t rate_hz) {
  if (rate_hz == s_sample_rate_hz) {
    return true;
  }

  PBL_LOG_DBG("MMC5603NJ: Setting sample rate to %d Hz", rate_hz);

  // Reset device runtime status (disabling continuous mode/etc)
  if (!prv_mmc5603nj_write(MMC5603NJ_REG_CTRL2, 0x00)) {
    return false;
  }

  // Do one final read to reset any data ready flags
  MagData discard_sample;
  prv_mmc5603nj_get_sample(&discard_sample);

  if (rate_hz > 0) {
    // Set new sampling rate
    if (!prv_mmc5603nj_write(MMC5603NJ_REG_ODR, rate_hz)) {
      PBL_LOG_ERR("MMC5603NJ: Failed to update ODR.");
      return false;
    }

    // Retrigger calculation of measurements rates
    if (!prv_mmc5603nj_write(MMC5603NJ_REG_CTRL0,
                             MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_CMM_FREQ_EN)) {
      PBL_LOG_ERR("MMC5603NJ: Failed to trigger measurement calculation update.");
      return false;
    }

    // Start continuous mode
    if (!prv_mmc5603nj_write(MMC5603NJ_REG_CTRL2, MMC5603NJ_CTRL2_AUTOSET_PRD_100 |
                                                      MMC5603NJ_CTRL2_PRD_SET_EN |
                                                      MMC5603NJ_CTRL2_CMM_EN)) {
      PBL_LOG_ERR("MMC5603NJ: Failed to start continuous mode.");
      return false;
    }
  }

  s_sample_rate_hz = rate_hz;

#ifndef CONFIG_RECOVERY_FW
  if (!prv_configure_polling()) {
    PBL_LOG_ERR("MMC5603NJ: Failed to configure polling");
    return false;
  }
#endif

  return true;
}

#ifndef CONFIG_RECOVERY_FW
// Configure polling (to simulate data-ready interrupts)

static bool prv_configure_polling(void) {
  uint16_t polling_interval_ms = s_sample_rate_hz == 0 ? 0 : DIVIDE_CEIL(1000.0, s_sample_rate_hz);

  if (s_polling_interval_ms == polling_interval_ms) {
    return true;
  }

  if (s_polling_timer != TIMER_INVALID_ID) {
    new_timer_stop(s_polling_timer);
    new_timer_delete(s_polling_timer);
    s_polling_timer = TIMER_INVALID_ID;
  }

  if (polling_interval_ms > 0) {
    s_polling_timer = new_timer_create();
    if (s_polling_timer == TIMER_INVALID_ID) {
      PBL_LOG_ERR("MMC5603NJ: Failed to create polling timer");
      return false;
    }
    new_timer_start(s_polling_timer, polling_interval_ms, prv_mmc5603nj_polling_callback, NULL,
                    TIMER_START_FLAG_REPEATING);
  }
  s_polling_interval_ms = polling_interval_ms;
  return true;
}

static void prv_mmc5603nj_polling_callback(void *data) {
  if (s_use_refcount == 0 || s_sample_rate_hz == 0) {
    return;
  }

  // Check if data is ready by reading status register
  if (prv_mmc5603nj_is_data_ready()) {
    // Post event to trigger data processing
    s_measurement_ready = true;
    PebbleEvent e = {
        .type = PEBBLE_ECOMPASS_SERVICE_EVENT,
    };
    event_put(&e);
  }
}
#endif

// Samples
bool prv_mmc5603nj_is_data_ready(void) {
  uint8_t status = 0;
  prv_mmc5603nj_read(MMC5603NJ_REG_STATUS1, 1, &status);
  return (status & MMC5603NJ_STATUS1_MEAS_M_DONE_MASK) > 0;
}

static MagReadStatus prv_mmc5603nj_get_sample(MagData *sample) {
  // Check if sensor enabled.
  if (s_sample_rate_hz == 0) {
    return MagReadMagOff;
  }

  // Check if data is ready
  if (!s_measurement_ready) {  // Avoid multiple status checks if we already know data is ready
    if (prv_mmc5603nj_is_data_ready()) {
      s_measurement_ready = true;
    }
  }
  if (!s_measurement_ready) {
    return MagReadCommunicationFail;
  }
  s_measurement_ready = false;  // Clear state since we will be reading the measurement below

  // Ready data
  uint8_t raw_data[6];  // Only need 16 bit precision per axis
  if (!prv_mmc5603nj_read(MMC5603NJ_REG_XOUT0, sizeof(raw_data), raw_data)) {
    return MagReadCommunicationFail;
  }

  int16_t raw_vector[3];
  for (uint8_t axis = 0; axis < 3; axis++) {
    uint16_t raw_axis_value = ((uint16_t)raw_data[2 * axis] << 8) | raw_data[2 * axis + 1];
    raw_vector[axis] =
        (int16_t)(raw_axis_value - (1 << 15));  // offset by 2^15 for uint -> int alignment
  }

  // Convert raw counts to milliGauss (mG)
  // Sensitivity: 1024 counts/G (16-bit operation)
  // Formula: mG = (raw_counts * 1000) / 1024
  int16_t raw_x = s_rotated_180 ? prv_get_axis_projection(X_AXIS, raw_vector) * -1
                                : prv_get_axis_projection(X_AXIS, raw_vector);
  int16_t raw_y = s_rotated_180 ? prv_get_axis_projection(Y_AXIS, raw_vector) * -1
                                : prv_get_axis_projection(Y_AXIS, raw_vector);
  int16_t raw_z = prv_get_axis_projection(Z_AXIS, raw_vector);

  sample->x = ((int32_t)raw_x * 1000) / 1024;
  sample->y = ((int32_t)raw_y * 1000) / 1024;
  sample->z = ((int32_t)raw_z * 1000) / 1024;

  return MagReadSuccess;
}

void mag_set_rotated(bool rotated) {
  s_rotated_180 = rotated;
}

static int16_t prv_get_axis_projection(axis_t axis, int16_t *raw_vector) {
  uint8_t axis_offset = BOARD_CONFIG_MAG.mag_config.axes_offsets[axis];
  bool invert = BOARD_CONFIG_MAG.mag_config.axes_inverts[axis];

  return (invert ? -1 : 1) * raw_vector[axis_offset];
}
