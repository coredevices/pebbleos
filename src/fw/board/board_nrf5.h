/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "display.h"

#include <pbl/drivers/button_id.h>
#include "debug/power_tracking.h"

#include <stdint.h>
#include <stdbool.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable" 
#include <hal/nrf_gpio.h>
#include <nrfx_spim.h>
#include <nrfx_gpiote.h>
#include <nrfx_timer.h>
#include <nrfx_pwm.h>
#include <nrfx_pdm.h>
#pragma GCC diagnostic pop

#define GPIO_Port_NULL (NULL)
#define GPIO_Pin_NULL ((uint16_t)-1)
//! Guaranteed invalid IRQ priority
#define IRQ_PRIORITY_INVALID (1 << __NVIC_PRIO_BITS)

// This is generated in order to facilitate the check within the IRQ_MAP macro below
enum {
#define IRQ_DEF(num, irq) IS_VALID_IRQ__##irq,
#if defined(CONFIG_SOC_NRF52)
#  include "irq_nrf52.def"
#else
#  error need IRQ table for new micro family
#endif
#undef IRQ_DEF
};

//! Creates a trampoline to the interrupt handler defined within the driver
#define IRQ_MAP(irq, handler, device) \
  void irq##_IRQHandler(void) { \
    handler(device); \
  } \
  _Static_assert(IS_VALID_IRQ__##irq || true, "(See comment below)")

#define IRQ_MAP_NRFX(irq, handler) \
  void irq##_IRQHandler(void) { \
    handler(); \
  } \
  _Static_assert(IS_VALID_IRQ__##irq || true, "(See comment below)")

/*
 * The above static assert checks that the requested IRQ is valid by checking that the enum
 * value (generated above) is declared. The static assert itself will not trip, but you will get
 * a compilation error from that line if the IRQ does not exist within irq_nrf52.def.
 */

typedef struct {
  nrfx_gpiote_t peripheral;
  uint8_t channel;
  uint32_t gpio_pin; ///< The result of NRF_GPIO_PIN_MAP(port, pin).
} GpioteConfig;

typedef GpioteConfig ExtiConfig; /* compatibility */

typedef struct {
  const char* const name; ///< Name for debugging purposes.
  GpioteConfig gpiote;
  nrf_gpio_pin_pull_t pull;
} ButtonConfig;

typedef struct {
  const uint32_t gpio_pin; ///< The result of NRF_GPIO_PIN_MAP(port, pin).
} ButtonComConfig;

#define NRF5_GPIO_RESOURCE_EXISTS ((void *)1)
typedef struct {
  void *gpio; ///< For compatibility, GPIO_RESOURCE_EXISTS if this is in use, NULL if not.
  const uint32_t gpio_pin; ///< The result of NRF_GPIO_PIN_MAP(port, pin).
} InputConfig;

typedef struct {
  void *gpio; ///< For compatibility, GPIO_RESOURCE_EXISTS if this is in use, NULL if not.
  const uint32_t gpio_pin; ///< The result of NRF_GPIO_PIN_MAP(port, pin).
  bool active_high; ///< Pin is active high or active low
} OutputConfig;

//! Alternate function pin configuration
//! Used to configure a pin for use by a peripheral
typedef struct {
  void *gpio; ///< For compatibility, GPIO_RESOURCE_EXISTS if this is in use, NULL if not.
  const uint32_t gpio_pin; ///< The result of NRF_GPIO_PIN_MAP(port, pin).
} AfConfig;

typedef struct {
  uint16_t value;
  uint16_t resolution;
  int enabled;
  nrf_pwm_sequence_t seq;
} PwmState;

typedef struct {
  OutputConfig output;
  nrfx_pwm_t peripheral;
  PwmState *state;
} PwmConfig;

typedef struct {
  int axes_offsets[3];
  bool axes_inverts[3];
} MagConfig;

typedef struct {
  AfConfig i2s_ck;
  AfConfig i2s_sd;
  NRF_SPIM_Type *spi;
  uint32_t spi_clock_ctrl;
  nrf_pdm_gain_t gain;
} MicConfig;

typedef struct {
  // Audio Configuration
  /////////////////////////////////////////////////////////////////////////////
  const MicConfig mic_config;

  // Ambient Light Configuration
  /////////////////////////////////////////////////////////////////////////////
  const uint32_t ambient_light_dark_threshold;
  const uint32_t ambient_k_delta_threshold;
  // Raw-count -> lux conversion: lux = (level - offset) * num / den.
  // den == 0 means no conversion available for this board.
  const uint32_t ambient_light_lux_dark_offset;
  const uint32_t ambient_light_lux_num;
  const uint32_t ambient_light_lux_den;
  const OutputConfig photo_en;
  const bool als_always_on;

  const uint8_t backlight_on_percent; // percent of max possible brightness
} BoardConfig;

// Button Configuration
/////////////////////////////////////////////////////////////////////////////
typedef struct {
  const ButtonConfig buttons[NUM_BUTTONS];
  const ButtonComConfig button_com;
  const bool active_high;
  nrfx_timer_t timer;
} BoardConfigButton;

// Power Configuration
/////////////////////////////////////////////////////////////////////////////
typedef struct {
  const GpioteConfig pmic_int;
  const InputConfig pmic_int_gpio;

  //! Percentage for watch only mode
  const uint8_t low_power_threshold;

  //! Approximate hours of battery life
  const uint16_t battery_capacity_hours;
} BoardConfigPower;

typedef struct {
  uint8_t default_motion_sensitivity;
} BoardConfigAccel;

typedef struct {
  const MagConfig mag_config;
  const InputConfig mag_int_gpio;
  const GpioteConfig mag_int;
} BoardConfigMag;

typedef struct {
  const OutputConfig ctl;
} BoardConfigActuator;

typedef struct {
  NRF_RTC_Type *rtc;
  NRF_GPIOTE_Type *gpiote;
  uint8_t gpiote_ch;
  uint32_t psel;
  uint32_t period_us;
  uint32_t pulse_us;
} NrfLowPowerPWM;

typedef struct {
  nrfx_spim_t spi;

  const OutputConfig mosi;
  const OutputConfig clk;
  const OutputConfig cs;

  const OutputConfig on_ctrl;
  const nrf_gpio_pin_drive_t on_ctrl_otype;

  const NrfLowPowerPWM extcomin;
} BoardConfigSharpDisplay;

typedef const struct UARTDevice UARTDevice;
typedef const struct I2CBus I2CBus;
typedef const struct I2CSlavePort I2CSlavePort;
typedef const struct HRMDevice HRMDevice;
typedef const struct MicDevice MicDevice;
typedef const struct QSPIPort QSPIPort;
typedef const struct QSPIFlash QSPIFlash;
typedef const struct AudioDevice AudioDevice;

void board_early_init(void);
void board_init(void);

#include <pbl/drivers/i2c/definitions.h>

#include "board_definitions.h"
