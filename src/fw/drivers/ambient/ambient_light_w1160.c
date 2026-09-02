/* SPDX-FileCopyrightText: 2025 Core Devices */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include "console/prompt.h"
#include <pbl/drivers/ambient_light.h>
#include <pbl/drivers/i2c.h>
#include "kernel/util/sleep.h"
#include "pbl/kernel/mutex.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include <inttypes.h>

PBL_LOG_MODULE_DEFINE(driver_ambient_w1160, CONFIG_DRIVER_AMBIENT_LOG_LEVEL);

// Registers
#define W1160_STATE_REG             0x00
#define W1160_IT_FAST1_REG          0x02
#define W1160_IT_FAST2_REG          0x03
#define W1160_SAMPLE_FAST1_REG      0x05
#define W1160_SAMPLE_FAST2_REG      0x06
#define W1160_SAMPLE_SLOW1_REG      0x07
#define W1160_SAMPLE_SLOW2_REG      0x08
#define W1160_FLAG1_REG             0x10
#define W1160_DATA1_ALS_REG         0x13
#define W1160_DATA2_ALS_REG         0x14
#define W1160_DATA_GC_REG           0x17
#define W1160_POWER_MODE_REG        0x3D
#define W1160_PDT_ID_REG            0x3E
#define W1160_FIFOCTRL1_REG         0x60
#define W1160_FIFOCTRL1_REG_VALUE   0x20
#define W1160_FIFO1_WM_LV_REG       0x61
#define W1160_FIFO2_WM_LV_REG       0x62
#define W1160_FIFOCTRL2_REG         0x63
#define W1160_FIFO_FCNT1_REG        0x64
#define W1160_FIFO_FCNT2_REG        0x65
#define W1160_FIFO_OUT_REG          0x66
#define W1160_FIFO_FLAG_REG         0x67
#define W1160_FIFOCTRL3_REG         0x68
#define W1160_FIFOCTRL4_REG         0x69
#define W1160_AGCCTRL1_REG          0x6A
#define W1160_MANUAL_GAIN_CTRL_REG  0x6B
#define W1160_AGCCTRL2_REG          0x6C
#define W1160_THD_SAT_GC_REG        0x6D
#define W1160_IT_SLOW1_REG          0x6E
#define W1160_IT_SLOW2_REG          0x6F
#define W1160_SOFT_RESET_REG        0x80
// Configuration register bits
#define W1160_ALSCTRL_REG           0xA4

#define W1160_POR_WAIT_TIME         (10)     /* ms */
#define W1160_AGCCTRL1_AGC_EN       (0<<7)   /* 1:en; 0:dis */
#define W1160_AGCCTRL1_MGC_EN       (1)      /* 1:en; 0:dis */
#define W1160_AGCCTRL2_SEL_MODE     (1<<2)   /* must be 1 */
#define W1160_AGCCTRL2_12B_MODE     (0<<3)   /* 1:12bit */
#define W1160_ALSCTRL_SAT_EN        (0<<1)   /* 1:en; 0:dis */
#define W1160_ALSCTRL_DATA_FORMAT12 (0<<2)   /* 1:12bit 0:16bit */
#define W1160_DATA_GC_LVL           (15<<4)  /* gc lelvel 15 */
#define W1160_SAT_GC_CONFIG         (0x0A)
#define W1160_SLOW_IT_CONFIG1       (0x03)   /* 99ms ((0x3E7+1) * 99us) */
#define W1160_SLOW_IT_CONFIG2       (0xE7)
#define W1160_SLOW_ST_CONFIG1       (0x07)   /* 200ms ((0x7CF+1) * 100us) */
#define W1160_SLOW_ST_CONFIG2       (0xCF)
#define W1160_SAMPLING_EN           (1<<1)   /* 1:en 0:dis */
#define W1160_SAMPLING_DIS          (0<<1)   /* 1:en 0:dis */
#define W1160_CHIP_ID               (0xE5)
#define W1160_FLG_ALS_DR            (1<<7)

#define W1160_RESULT_EXPONENT_SHIFT (12)
#define W1160_RESULT_MANTISSA_MASK  (0x0FFF)

#define W1160_ALS_POLL_DELAY_MS     (5)    /* ms between data-ready polls */
#define W1160_ALS_POLL_TIMEOUT_MS   (200)  /* max wait for ALS data-ready */

static bool s_initialized;
static uint32_t s_sensor_light_dark_threshold;

// DATA_ALS reads zero while SAMPLING_EN=0, so we cache the last known-good
// value. Cache is served during suspend windows and invalidated when the
// active window drops (so the next unprimed read does a fresh one-shot
// instead of returning a stale value from an earlier primed window).
#define W1160_SETTLE_AFTER_ENABLE_MS (W1160_ALS_POLL_TIMEOUT_MS)
static PBL_MUTEX_DEFINE(s_state_mutex);
static bool s_active;
static bool s_sampling_active;
static RtcTicks s_sampling_started_ticks;
static uint16_t s_cached_value;
static bool s_cache_valid;

static bool prv_read_register(uint8_t register_address, uint8_t *result) {
  i2c_use(I2C_W1160);
  bool rv = i2c_read_register_block(I2C_W1160, register_address, 1, result);
  i2c_release(I2C_W1160);
  return rv;
}

static bool prv_write_register(uint8_t register_address, uint8_t datum) {
  i2c_use(I2C_W1160);
  bool rv = i2c_write_register_block(I2C_W1160, register_address, 1, &datum);
  i2c_release(I2C_W1160);
  return rv;
}

static bool prv_read_data_register(uint16_t *als_out) {
  uint8_t hi, lo;
  if (!prv_read_register(W1160_DATA1_ALS_REG, &hi)) {
    return false;
  }
  if (!prv_read_register(W1160_DATA2_ALS_REG, &lo)) {
    return false;
  }
  *als_out = (((uint16_t)hi) << 8) | lo;
  return true;
}

void ambient_light_init(void) {
  uint8_t chip_id;
  bool rv;

  s_sensor_light_dark_threshold = BOARD_CONFIG.ambient_light_dark_threshold;

  psleep(W1160_POR_WAIT_TIME);

  rv = prv_read_register(W1160_PDT_ID_REG, &chip_id);
  if (!rv) {
    PBL_LOG_ERR("Could not read W1160 chip ID");
    return;
  }

  if (chip_id != W1160_CHIP_ID) {
    PBL_LOG_ERR("Unexpected W1160 chip ID: 0x%02x", chip_id);
    return;
  }

  rv = prv_write_register(W1160_AGCCTRL1_REG, W1160_AGCCTRL1_AGC_EN);
  rv &= prv_write_register(W1160_MANUAL_GAIN_CTRL_REG, W1160_AGCCTRL1_MGC_EN);
  rv &= prv_write_register(W1160_DATA_GC_REG, W1160_DATA_GC_LVL);
  rv &= prv_write_register(W1160_AGCCTRL2_REG, W1160_AGCCTRL2_SEL_MODE|W1160_AGCCTRL2_12B_MODE);
  rv &= prv_write_register(W1160_ALSCTRL_REG, W1160_ALSCTRL_SAT_EN|W1160_ALSCTRL_DATA_FORMAT12);
  rv &= prv_write_register(W1160_THD_SAT_GC_REG, W1160_SAT_GC_CONFIG);
  rv &= prv_write_register(W1160_IT_SLOW1_REG, W1160_SLOW_IT_CONFIG1);
  rv &= prv_write_register(W1160_IT_SLOW2_REG, W1160_SLOW_IT_CONFIG2);
  rv &= prv_write_register(W1160_SAMPLE_SLOW1_REG, W1160_SLOW_ST_CONFIG1);
  rv &= prv_write_register(W1160_SAMPLE_SLOW2_REG, W1160_SLOW_ST_CONFIG2);

  PBL_ASSERT(rv, "Failed to initialize W1160");

  ambient_light_common_init();
  s_initialized = true;
}

// Block-poll FLG_ALS_DR, then read DATA_ALS. Caller must have SAMPLING_EN=1.
static bool prv_read_data_polled(uint16_t *als_out) {
  uint8_t flag;
  uint32_t elapsed = 0;
  while (true) {
    if (!prv_read_register(W1160_FLAG1_REG, &flag)) {
      PBL_LOG_ERR("Could not read W1160 FLAG1");
      return false;
    }
    if (flag & W1160_FLG_ALS_DR) {
      break;
    }
    if (elapsed >= W1160_ALS_POLL_TIMEOUT_MS) {
      PBL_LOG_ERR("W1160 ALS data-ready timeout");
      return false;
    }
    psleep(W1160_ALS_POLL_DELAY_MS);
    elapsed += W1160_ALS_POLL_DELAY_MS;
  }
  return prv_read_data_register(als_out);
}

// True if SAMPLING_EN has been on long enough for DATA_ALS to hold a real
// sample. Caller holds s_state_mutex and has checked s_sampling_active.
static bool prv_sampling_has_settled_locked(void) {
  const RtcTicks now = rtc_get_ticks();
  const RtcTicks elapsed = now - s_sampling_started_ticks;
  return elapsed >= ((RtcTicks)W1160_SETTLE_AFTER_ENABLE_MS * RTC_TICKS_HZ / 1000U);
}

uint32_t ambient_light_get_light_level(void) {
  if (!s_initialized) {
    return 0UL;
  }

  pbl_mutex_lock(&s_state_mutex, PBL_FOREVER);

  if (s_active) {
    if (s_sampling_active && prv_sampling_has_settled_locked()) {
      uint16_t als = 0;
      bool ok = prv_read_data_register(&als);
      if (ok) {
        s_cached_value = als;
        s_cache_valid = true;
      }
      pbl_mutex_unlock(&s_state_mutex);
      return ok ? als : 0UL;
    }

    if (s_cache_valid) {
      uint16_t cached = s_cached_value;
      pbl_mutex_unlock(&s_state_mutex);
      return cached;
    }

    if (!s_sampling_active) {
      pbl_mutex_unlock(&s_state_mutex);
      return 0UL;
    }
    uint16_t als = 0;
    bool ok = prv_read_data_polled(&als);
    if (ok) {
      s_cached_value = als;
      s_cache_valid = true;
    }
    pbl_mutex_unlock(&s_state_mutex);
    return ok ? als : 0UL;
  }

  // Unprimed one-shot: enable, poll, read, disable.
  if (!prv_write_register(W1160_STATE_REG, W1160_SAMPLING_EN)) {
    PBL_LOG_ERR("Could not enable W1160 sampling");
    pbl_mutex_unlock(&s_state_mutex);
    return 0UL;
  }

  uint16_t als = 0;
  bool ok = prv_read_data_polled(&als);
  if (ok) {
    s_cached_value = als;
    s_cache_valid = true;
  }

  if (!prv_write_register(W1160_STATE_REG, W1160_SAMPLING_DIS)) {
    PBL_LOG_ERR("Could not disable W1160 sampling");
    pbl_mutex_unlock(&s_state_mutex);
    return 0UL;
  }

  pbl_mutex_unlock(&s_state_mutex);
  return ok ? als : 0UL;
}

void ambient_light_driver_set_state(bool active, bool sampling) {
  if (!s_initialized) {
    return;
  }
  pbl_mutex_lock(&s_state_mutex, PBL_FOREVER);
  if (sampling != s_sampling_active) {
    const uint8_t reg = sampling ? W1160_SAMPLING_EN : W1160_SAMPLING_DIS;
    if (prv_write_register(W1160_STATE_REG, reg)) {
      s_sampling_active = sampling;
      if (sampling) {
        s_sampling_started_ticks = rtc_get_ticks();
      }
    } else {
      PBL_LOG_ERR("Could not write W1160 STATE_REG");
    }
  }
  if (s_active && !active) {
    // Window closed: drop the cache so the next unprimed read is fresh.
    s_cache_valid = false;
  }
  s_active = active;
  pbl_mutex_unlock(&s_state_mutex);
}

void command_als_read(void) {
  char buffer[16] = {0};
  prompt_send_response_fmt(buffer, sizeof(buffer), "%"PRIu32"", ambient_light_get_light_level());
}

uint32_t ambient_light_get_dark_threshold(void) {
  return s_sensor_light_dark_threshold;
}

void ambient_light_set_dark_threshold(uint32_t new_threshold) {
  PBL_ASSERTN(new_threshold <= AMBIENT_LIGHT_LEVEL_MAX);
  s_sensor_light_dark_threshold = new_threshold;
}

bool ambient_light_is_light(void) {
  // The threshold lives in the lux domain (see ambient_light_level_to_lux).
  return ambient_light_level_to_lux(ambient_light_get_light_level()) >
         s_sensor_light_dark_threshold;
}

AmbientLightLevel ambient_light_level_to_enum(uint32_t light_level) {
  const uint32_t k_delta_threshold = BOARD_CONFIG.ambient_k_delta_threshold;
  if (light_level < (s_sensor_light_dark_threshold - k_delta_threshold)) {
    return AmbientLightLevelVeryDark;
  } else if (light_level < s_sensor_light_dark_threshold) {
    return AmbientLightLevelDark;
  } else if (light_level < (s_sensor_light_dark_threshold + k_delta_threshold)) {
    return AmbientLightLevelLight;
  } else {
    return AmbientLightLevelVeryLight;
  }
}
