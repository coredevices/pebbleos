/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/backlight.h>

#include "board/board.h"
#include <pbl/drivers/i2c.h>
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DEFINE(driver_backlight_aw2016, CONFIG_DRIVER_BACKLIGHT_LOG_LEVEL);

#define AW2016_REG_RSTR 0x00U
#define AW2016_REG_RSTR_CHIP_ID 0x09U
#define AW2016_REG_RSTR_RST 0x55U

#define AW2016_REG_GCR1 0x01U
#define AW2016_REG_GCR1_CHGDIS_DIS (1U << 1U)
#define AW2016_REG_GCR1_CHIPEN_EN (1U << 0U)
#define AW2016_REG_GCR1_CHIPEN_DIS 0U

#define AW2016_REG_GCR2 0x04U
#define AW2016_REG_GCR2_IMAX_30MA 1U

#define AW2016_REG_LCTR (0x30)
#define AW2016_REG_LCTR_EXP_LINEAR (1U << 3U)
#define AW2016_REG_LCTR_LE3_EN (1U << 2U)
#define AW2016_REG_LCTR_LE2_EN (1U << 1U)
#define AW2016_REG_LCTR_LE1_EN (1U << 0U)

#define AW2016_REG_LCFG1 0x31U
#define AW2016_REG_LCFG1_CUR_MAX 0x0FU

#define AW2016_REG_LCFG2 0x32U
#define AW2016_REG_LCFG2_CUR_MAX 0x0FU

#define AW2016_REG_LCFG3 0x33U
#define AW2016_REG_LCFG3_CUR_MAX 0x0FU

#define AW2016_REG_PWM1 0x34U

#define AW2016_REG_PWM2 0x35U

#define AW2016_REG_PWM3 0x36U

static uint8_t s_brightness;
static uint32_t s_rgb_current_color = BACKLIGHT_COLOR_WARM_WHITE;

static bool prv_read_register(uint8_t register_address, uint8_t *value) {
  bool ret;

  i2c_use(I2C_AW2016);
  ret = i2c_read_register_block(I2C_AW2016, register_address, 1, value);
  i2c_release(I2C_AW2016);

  return ret;
}

static bool prv_write_register(uint8_t register_address, uint8_t value) {
  bool ret;

  i2c_use(I2C_AW2016);
  ret = i2c_write_register_block(I2C_AW2016, register_address, 1, &value);
  i2c_release(I2C_AW2016);

  return ret;
}

static bool prv_configure_registers(void) {
  bool ret;
  ret = prv_write_register(AW2016_REG_GCR2, AW2016_REG_GCR2_IMAX_30MA);
  ret &= prv_write_register(AW2016_REG_LCTR, AW2016_REG_LCTR_EXP_LINEAR | AW2016_REG_LCTR_LE3_EN |
                                                 AW2016_REG_LCTR_LE2_EN | AW2016_REG_LCTR_LE1_EN);
  ret &= prv_write_register(AW2016_REG_LCFG1, AW2016_REG_LCFG1_CUR_MAX);
  ret &= prv_write_register(AW2016_REG_LCFG2, AW2016_REG_LCFG2_CUR_MAX);
  ret &= prv_write_register(AW2016_REG_LCFG3, AW2016_REG_LCFG3_CUR_MAX);
  return ret;
}

void backlight_init(void) {
  uint8_t value = 0;
  bool ret;

  // A wedged AW2016 or I2C bus must not be fatal here: asserting on boot bricks
  // the watch into a recovery/factory-reset loop. A dead backlight is preferable.
  ret = prv_read_register(AW2016_REG_RSTR, &value);
  if (!ret || (value != AW2016_REG_RSTR_CHIP_ID)) {
    PBL_LOG_ERR("AW2016 not detected (i2c_ok=%d id=0x%02x); backlight disabled", ret, value);
    return;
  }

  ret = prv_write_register(AW2016_REG_RSTR, AW2016_REG_RSTR_RST);
  ret &= prv_write_register(AW2016_REG_GCR1, AW2016_REG_GCR1_CHGDIS_DIS);
  if (!ret) {
    PBL_LOG_ERR("AW2016 init write failed (i2c)");
  }
}

void backlight_set_brightness(uint8_t brightness) {
  bool ret;

  if (brightness > 100U) {
    brightness = 100U;
  }

  if (s_brightness == brightness) {
    return;
  }

  const uint8_t previous_brightness = s_brightness;
  s_brightness = brightness;

  if (brightness == 0U) {
    ret = prv_write_register(AW2016_REG_GCR1,
                             AW2016_REG_GCR1_CHGDIS_DIS | AW2016_REG_GCR1_CHIPEN_DIS);
    if (!ret) {
      PBL_LOG_ERR("AW2016 disable failed (i2c)");
    }
  } else {
    if (previous_brightness == 0U) {
      ret = prv_write_register(AW2016_REG_GCR1,
                               AW2016_REG_GCR1_CHGDIS_DIS | AW2016_REG_GCR1_CHIPEN_EN);
      ret &= prv_configure_registers();
      if (!ret) {
        PBL_LOG_ERR("AW2016 enable failed (i2c)");
        return;
      }
    }

    backlight_set_color(s_rgb_current_color);
  }
}

void backlight_set_color(uint32_t rgb_color) {
  bool ret;
  uint8_t red, green, blue;

  red = ((rgb_color & 0x00FF0000) >> 16) * s_brightness / 100;
  green = ((rgb_color & 0x0000FF00) >> 8) * s_brightness / 100;
  blue = (rgb_color & 0x000000FF) * s_brightness / 100;

  ret = prv_write_register(AW2016_REG_PWM1, red);
  ret &= prv_write_register(AW2016_REG_PWM2, green);
  ret &= prv_write_register(AW2016_REG_PWM3, blue);

  if (!ret) {
    PBL_LOG_ERR("AW2016 set color failed (i2c)");
    return;
  }

  s_rgb_current_color = rgb_color;
}

uint32_t backlight_get_color(void) {
  return s_rgb_current_color;
}

uint8_t backlight_get_level(uint8_t brightness) {
  // 255-step current control: every brightness value is distinct.
  return brightness;
}

void backlight_refresh(void) {
  bool ret;

  if (s_brightness == 0U) {
    return;
  }

  ret = prv_write_register(AW2016_REG_GCR1,
                           AW2016_REG_GCR1_CHGDIS_DIS | AW2016_REG_GCR1_CHIPEN_EN);
  ret &= prv_configure_registers();
  if (!ret) {
    PBL_LOG_ERR("AW2016 refresh failed (i2c)");
    return;
  }

  backlight_set_color(s_rgb_current_color);
}
