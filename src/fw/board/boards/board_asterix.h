#pragma once

#include "services/imu/units.h"
#include "util/size.h"

#define BT_VENDOR_ID 0x0EEA
#define BT_VENDOR_NAME "Core Devices LLC"

#define BOARD_LSE_MODE RCC_LSE_Bypass

#define BOARD_RTC_INST NRF_RTC1

static const BoardConfig BOARD_CONFIG = {
  .ambient_light_dark_threshold = 100,
  .ambient_k_delta_threshold = 30,
  .photo_en = { },
  .als_always_on = true, // XXX: some day, we will probably want to poll this less frequently, but the ALS on this device is slow, so we pay the 3.7uA for now

  // new sharp display requires 30/60Hz so we feed it directly from PMIC... XXX: some day
  .lcd_com = { 0 },

  .backlight_on_percent = 25,
  .backlight_max_duty_cycle_percent = 67,

  .power_5v0_options = OptionNotPresent,
  .power_ctl_5v0 = { 0 },
  
  .dbgserial_int = {
    .peripheral = NRFX_GPIOTE_INSTANCE(0), 
    .channel = 0,
    .gpio_pin = NRF_GPIO_PIN_MAP(0, 5),
  },

  .has_mic = true,
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
  .button_com = { 0 },
  .active_high = false,
  .timer = NRFX_TIMER_INSTANCE(1),
};

static const BoardConfigPower BOARD_CONFIG_POWER = {
  .pmic_int = { NRFX_GPIOTE_INSTANCE(0), 1, NRF_GPIO_PIN_MAP(1, 12) },
  .pmic_int_gpio = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 12) },

  .battery_vmon_scale = { /* TODO */
    // Battery voltage is scaled down by a pair of resistors:
    //  - R13 on the top @ 47k
    //  - R15 on the bottom @ 30.1k
    //   (R13 + R15) / R15 = 77.1 / 30.1
    .numerator = 771,
    .denominator = 301,
  },

  .vusb_stat = { .gpio_pin = GPIO_Pin_NULL, },
  .chg_stat = { },
  .chg_fast = { },
  .chg_en = { },
  .has_vusb_interrupt = false,

  .wake_on_usb_power = false,

  .charging_status_led_voltage_compensation = 0,

#if defined(IS_BIGBOARD) && !defined(BATTERY_DEBUG)
  // We don't use the same batteries on all bigboards, so set a safe cutoff voltage of 4.2V.
  // Please do not change this!
  .charging_cutoff_voltage = 4200,
#else
  .charging_cutoff_voltage = 4300,
#endif

  .low_power_threshold = 5,

  // Based on measurements from v4.0-beta16.
  // Typical Connected Current at VBAT without HRM ~520uA
  // Added draw with HRM on : ~1.5mA ==> Average impact (5% per hour + 1 hour continuous / day)
  //    (.05 * 23/24 + 1.0 * 1/24) * 1.5mA = ~134uA
  // Assume ~150uA or so for notifications & user interaction
  // Total Hours = 125 mA * hr / (.520 + .134 + 150)mA = 155 hours
  .battery_capacity_hours = 155 /* TODO */,
};

static const BoardConfigMag BOARD_CONFIG_MAG = {
  .mag_config = {
#ifdef IS_BIGBOARD
    .axes_offsets[AXIS_X] = 1,
    .axes_offsets[AXIS_Y] = 0,
    .axes_offsets[AXIS_Z] = 2,
    .axes_inverts[AXIS_X] = false,
    .axes_inverts[AXIS_Y] = true,
    .axes_inverts[AXIS_Z] = true,
#else
    .axes_offsets[AXIS_X] = 1,
    .axes_offsets[AXIS_Y] = 0,
    .axes_offsets[AXIS_Z] = 2,
    .axes_inverts[AXIS_X] = true,
    .axes_inverts[AXIS_Y] = true,
    .axes_inverts[AXIS_Z] = false,
#endif
  },
  .mag_int_gpio = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 3) },
  .mag_int = { .peripheral = NRFX_GPIOTE_INSTANCE(0), .channel = 6, .gpio_pin = NRF_GPIO_PIN_MAP(0, 3), },
};

static const BoardConfigActuator BOARD_CONFIG_VIBE = {
  .ctl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 2), true }, // LRA_EN
  .vsys_scale = 3300,
};

#if 0

static const BoardConfigAccel BOARD_CONFIG_ACCEL = {
  .accel_config = {
    .axes_offsets[AXIS_X] = 0,
    .axes_offsets[AXIS_Y] = 1,
    .axes_offsets[AXIS_Z] = 2,
#if IS_BIGBOARD
    .axes_inverts[AXIS_X] = false,
    .axes_inverts[AXIS_Y] = false,
    .axes_inverts[AXIS_Z] = false,
#else
    .axes_inverts[AXIS_X] = true,
    .axes_inverts[AXIS_Y] = true,
    .axes_inverts[AXIS_Z] = true,
#endif
    // This is affected by the acceleromter's configured ODR, so this value
    // will need to be tuned again once we stop locking the BMA255 to an ODR of
    // 125 Hz.
    .shake_thresholds[AccelThresholdHigh] = 64,
    .shake_thresholds[AccelThresholdLow] = 0xf,
    .double_tap_threshold = 12500,
  },
  .accel_int_gpios = {
    [0] = { GPIOA, GPIO_Pin_6 },
    [1] = { GPIOA, GPIO_Pin_3 },
  },
  .accel_ints = {
    [0] = { EXTI_PortSourceGPIOA, 6 },
    [1] = { EXTI_PortSourceGPIOA, 3 }
  },
};



#define ACCESSORY_UART_IS_SHARED_WITH_BT 1
static const BoardConfigAccessory BOARD_CONFIG_ACCESSORY = {
  .exti = { EXTI_PortSourceGPIOA, 11 },
};

static const BoardConfigBTCommon BOARD_CONFIG_BT_COMMON = {
  .controller = DA14681,
  .reset = { GPIOC, GPIO_Pin_5, true },
  .wakeup = {
    .int_gpio = { GPIOC, GPIO_Pin_4 },
    .int_exti = { EXTI_PortSourceGPIOC, 4 },
  },
};

static const BoardConfigBTSPI BOARD_CONFIG_BT_SPI = {
  .cs = { GPIOB, GPIO_Pin_1, false },
};

static const BoardConfigMCO1 BOARD_CONFIG_MCO1 = {
  .output_enabled = true,
  .af_cfg = {
    .gpio = GPIOA,
    .gpio_pin = GPIO_Pin_8,
    .gpio_pin_source = GPIO_PinSource8,
    .gpio_af = GPIO_AF_MCO,
  },
  .an_cfg = {
    .gpio = GPIOA,
    .gpio_pin = GPIO_Pin_8,
  },
};


#define DIALOG_TIMER_IRQ_HANDLER TIM6_IRQHandler
static const TimerIrqConfig BOARD_BT_WATCHDOG_TIMER = {
  .timer = {
    .peripheral = TIM6,
    .config_clock = RCC_APB1Periph_TIM6,
  },
  .irq_channel = TIM6_IRQn,
};

extern DMARequest * const COMPOSITOR_DMA;
extern DMARequest * const SHARP_SPI_TX_DMA;

extern UARTDevice * const QEMU_UART;
#endif
extern UARTDevice * const DBG_UART;
#if 0
extern UARTDevice * const ACCESSORY_UART;

extern UARTDevice * const BT_TX_BOOTROM_UART;
extern UARTDevice * const BT_RX_BOOTROM_UART;

extern I2CSlavePort * const I2C_AS3701B;
#endif

extern PwmState BACKLIGHT_PWM_STATE;
static const BoardConfigActuator BOARD_CONFIG_BACKLIGHT = {
  .options = ActuatorOptions_Pwm | ActuatorOptions_Ctl,
  .ctl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 8), true },
  .pwm = {
    .state = &BACKLIGHT_PWM_STATE,
    .output = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 26), true },
    .peripheral = NRFX_PWM_INSTANCE(0)
  },
};

extern PwmState DISPLAY_EXTCOMIN_STATE;
static const BoardConfigSharpDisplay BOARD_CONFIG_DISPLAY = {
  .spi = NRFX_SPIM_INSTANCE(3),

  .clk = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 6), true },
  .mosi = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 8), true },
  .cs = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 3), true },

  .on_ctrl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 4), true },

  .extcomin = {
    .state = &DISPLAY_EXTCOMIN_STATE,
    .output = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 15), true },
    .peripheral = NRFX_PWM_INSTANCE(1),
  },
};

extern const VoltageMonitorDevice * VOLTAGE_MONITOR_ALS;
extern const VoltageMonitorDevice * VOLTAGE_MONITOR_BATTERY;

extern const TemperatureSensor * const TEMPERATURE_SENSOR;

extern HRMDevice * const HRM;

extern QSPIPort * const QSPI;
extern QSPIFlash * const QSPI_FLASH;

extern MicDevice * const MIC;

extern I2CSlavePort * const I2C_NPM1300;
extern I2CSlavePort * const I2C_DRV2604;
extern I2CSlavePort * const I2C_OPT3001;
extern I2CSlavePort * const I2C_DA7212;
