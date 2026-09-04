/* SPDX-FileCopyrightText: 2025 Core Devices */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include "console/prompt.h"
#include <pbl/drivers/ambient_light.h>
#include <pbl/drivers/i2c.h>
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include <inttypes.h>

PBL_LOG_MODULE_DEFINE(driver_ambient_opt3001, CONFIG_DRIVER_AMBIENT_LOG_LEVEL);

static uint32_t s_sensor_light_dark_threshold;
static bool s_initialized = false;

#define OPT3001_RESULT 0x00
#define OPT3001_RESULT_EXPONENT_SHIFT 12
#define OPT3001_RESULT_MANTISSA_MASK  0x0FFF
#define OPT3001_CONFIG 0x01
#define OPT3001_CONFIG_RANGE_AUTO       0xC000
#define OPT3001_CONFIG_CONVTIME_100MSEC 0x0000
#define OPT3001_CONFIG_MODE_CONTINUOUS  0x0600
#define OPT3001_CONFIG_MODE_SINGLESHOT  0x0200
#define OPT3001_MFGID 0x7E
#define OPT3001_MFGID_VAL 0x5449 /* "TI" */
#define OPT3001_DEVID 0x7F
#define OPT3001_DEVID_VAL 0x3001

static bool prv_read_register(uint8_t register_address, uint16_t *result) {
  uint8_t buf[2];
  i2c_use(I2C_OPT3001);
  bool rv = i2c_read_register_block(I2C_OPT3001, register_address, 2, buf);
  *result = buf[0] << 8 | buf[1];
  i2c_release(I2C_OPT3001);
  return rv;
}

static bool prv_write_register(uint8_t register_address, uint16_t datum) {
  i2c_use(I2C_OPT3001);
  uint8_t block[3] = { register_address, datum >> 8, datum & 0xFF };
  bool rv = i2c_write_block(I2C_OPT3001, 3, block);
  i2c_release(I2C_OPT3001);
  return rv;
}

static uint32_t prv_get_default_ambient_light_dark_threshold(void) {
  PBL_ASSERTN(BOARD_CONFIG.ambient_light_dark_threshold != 0);
  return BOARD_CONFIG.ambient_light_dark_threshold;
}

void ambient_light_init(void) {
  s_sensor_light_dark_threshold = prv_get_default_ambient_light_dark_threshold();

  uint16_t mf, id;
  bool ok = prv_read_register(OPT3001_MFGID, &mf) && prv_read_register(OPT3001_DEVID, &id);
  if (!ok) {
    PBL_LOG_ERR("failed to read OPT3001 ID registers");
    return;
  }

  if (mf != OPT3001_MFGID_VAL || id != OPT3001_DEVID_VAL) {
    PBL_LOG_ERR("OPT3001 read successfully, but had incorrect manuf %04x, id %04x", mf, id);
    return;
  }

  if (BOARD_CONFIG.als_always_on) {
    prv_write_register(OPT3001_CONFIG, OPT3001_CONFIG_RANGE_AUTO | OPT3001_CONFIG_CONVTIME_100MSEC | OPT3001_CONFIG_MODE_CONTINUOUS);
  }

  ambient_light_common_init();
  s_initialized = true;
}

void ambient_light_driver_set_state(bool active, bool sampling) {
  // OPT3001 is configured at boot per BOARD_CONFIG.als_always_on; no gate needed.
  (void)active;
  (void)sampling;
}

uint32_t ambient_light_get_light_level(void) {
  if (!s_initialized) {
    return BOARD_CONFIG.ambient_light_dark_threshold;
  }
  
  if (!BOARD_CONFIG.als_always_on) {
    prv_write_register(OPT3001_CONFIG, OPT3001_CONFIG_RANGE_AUTO | OPT3001_CONFIG_CONVTIME_100MSEC | OPT3001_CONFIG_MODE_SINGLESHOT);
  }

  uint16_t result;
  prv_read_register(OPT3001_RESULT, &result);
  uint32_t exp = result >> OPT3001_RESULT_EXPONENT_SHIFT;
  uint32_t mant = result & OPT3001_RESULT_MANTISSA_MASK;

  uint32_t level = mant << exp;

  return level;
}

void command_als_read(void) {
  char buffer[16];
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
  // if the sensor is not enabled, always return that it is dark
  // The threshold lives in the lux domain (see ambient_light_level_to_lux).
  return s_initialized && ambient_light_level_to_lux(ambient_light_get_light_level()) >
                              s_sensor_light_dark_threshold;
}

AmbientLightLevel ambient_light_level_to_enum(uint32_t light_level) {
  if (!s_initialized) {
    // if the sensor is not enabled, always return that it is very dark
    return AmbientLightLevelUnknown;
  }

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
