#pragma once

#include <pbl/drivers/backlight/pwm.h>
#include <pbl/drivers/imu/lsm6dso/lsm6dso.h>
#include <pbl/drivers/pmic/npm1300.h>
#include "pbl/services/imu/units.h"

#define BT_VENDOR_ID 0x0EEA
#define BT_VENDOR_NAME "Core Devices LLC"

#define BOARD_LSE_MODE RCC_LSE_Bypass

#define BOARD_RTC_INST NRF_RTC1
#define BOARD_RTC_IRQN RTC1_IRQn

static const BoardConfig BOARD_CONFIG = {
  .ambient_light_dark_threshold = 100,
  .ambient_k_delta_threshold = 30,
  .als_always_on = true,

  .backlight_on_percent = 25,

  .mic_config = {
    .gain = 40,
  }
};

static const BoardConfigButton BOARD_CONFIG_BUTTON = {
  .buttons = {
    [BUTTON_ID_BACK] =
        { "Back",   { NRFX_GPIOTE_INSTANCE(0), 2, NRF_GPIO_PIN_MAP(0, 28) }, NRF_GPIO_PIN_PULLUP },
    [BUTTON_ID_UP] =
        { "Up",     { NRFX_GPIOTE_INSTANCE(0), 3, NRF_GPIO_PIN_MAP(0, 29) }, NRF_GPIO_PIN_PULLUP },
    [BUTTON_ID_SELECT] =
        { "Select", { NRFX_GPIOTE_INSTANCE(0), 4, NRF_GPIO_PIN_MAP(0, 30) }, NRF_GPIO_PIN_PULLUP },
    [BUTTON_ID_DOWN] =
        { "Down",   { NRFX_GPIOTE_INSTANCE(0), 5, NRF_GPIO_PIN_MAP(0, 31) }, NRF_GPIO_PIN_PULLUP },
  },
  .active_high = false,
  .timer = NRFX_TIMER_INSTANCE(1),
};

static const BoardConfigPower BOARD_CONFIG_POWER = {
  .pmic_int = { NRFX_GPIOTE_INSTANCE(0), 1, NRF_GPIO_PIN_MAP(1, 12) },
  .pmic_int_gpio = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 12) },
  .low_power_threshold = 2,
  .battery_capacity_hours = 450,
};

static const BoardConfigActuator BOARD_CONFIG_VIBE = {
  .ctl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 2), true }, // LRA_EN
};

static const BoardConfigAccel BOARD_CONFIG_ACCEL = {
  .default_motion_sensitivity = 55U, // Medium
};

static const BoardConfigMag BOARD_CONFIG_MAG = {
  .mag_config = {
    .axes_offsets[AXIS_X] = 1,
    .axes_offsets[AXIS_Y] = 0,
    .axes_offsets[AXIS_Z] = 2,
    .axes_inverts[AXIS_X] = false,
    .axes_inverts[AXIS_Y] = true,
    .axes_inverts[AXIS_Z] = false,
  },
};

extern UARTDevice * const DBG_UART;

extern PwmState BACKLIGHT_PWM_STATE;
static const BacklightPwmConfig BACKLIGHT_PWM = {
  .ctl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 8), true },
  .pwm = {
    .state = &BACKLIGHT_PWM_STATE,
    .output = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 26), true },
    .peripheral = NRFX_PWM_INSTANCE(0)
  },
  .max_duty_cycle_percent = 67,
};

static const BoardConfigSharpDisplay BOARD_CONFIG_DISPLAY = {
  .spi = NRFX_SPIM_INSTANCE(3),

  .clk = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 6), true },
  .mosi = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 8), true },
  .cs = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 3), true },

  .on_ctrl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 4), true },

  .extcomin = {
    .rtc = NRF_RTC2,
    .gpiote = NRF_GPIOTE,
    .gpiote_ch = 6,
    .psel = NRF_GPIO_PIN_MAP(1, 15),
    // 120Hz/5% (feeds flip-flop, generating 60Hz/50% signal to EXTCOMIN)
    .period_us = 1000000 / 120,
    .pulse_us = (1000000 / 120) / 20,
  },
};

extern QSPIPort * const QSPI;
extern QSPIFlash * const QSPI_FLASH;

extern MicDevice * const MIC;
extern AudioDevice * const AUDIO;

extern I2CSlavePort * const I2C_NPM1300;
extern I2CSlavePort * const I2C_DRV2604;
extern I2CSlavePort * const I2C_OPT3001;
extern I2CSlavePort * const I2C_DA7212;
extern I2CSlavePort * const I2C_MMC5603NJ;
extern I2CSlavePort * const I2C_BMP390;
extern const LSM6DSOConfig *const LSM6DSO;

extern const Npm1300Config NPM1300_CONFIG;
