/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "display.h"

#include <stdint.h>
#include <stdbool.h>

#include "bf0_hal_pinmux.h"
#include <pbl/drivers/button_id.h>

#define IRQ_PRIORITY_INVALID (1 << __NVIC_PRIO_BITS)

enum {
  #define IRQ_DEF(num, irq) IS_VALID_IRQ__##irq,
  #include "irq_sf32lb52.def"
  #undef IRQ_DEF
};

//! Creates a trampoline to the interrupt handler defined within the driver
#define IRQ_MAP(irq, handler, device) \
  void irq##_IRQHandler(void) { \
    handler(device); \
  } \
  _Static_assert(IS_VALID_IRQ__##irq || true, "(See comment below)")
/*
 * The above static assert checks that the requested IRQ is valid by checking that the enum
 * value (generated above) is declared. The static assert itself will not trip, but you will get
 * a compilation error from that line if the IRQ does not exist within irq_sf32lb.def.
 */

#define GPIO_Port_NULL NULL
#define GPIO_Pin_NULL 0U

typedef enum {
  GPIO_OType_PP,
  GPIO_OType_OD,
} GPIOOType_TypeDef;

typedef enum {
  GPIO_PuPd_NOPULL,
  GPIO_PuPd_UP,
  GPIO_PuPd_DOWN,
} GPIOPuPd_TypeDef;

typedef struct {
  GPIO_TypeDef* const peripheral; ///< One of GPIOX. For example, GPIOA.
  const uint32_t gpio_pin; ///< One of GPIO_Pin_X.
  GPIOPuPd_TypeDef pull; ///< Pull-up / pull-down configuration for the pin
} ExtiConfig;

typedef struct {
  void *gpio;
  uint8_t gpio_pin;
} InputConfig;

typedef struct {
  void *gpio;
  uint8_t gpio_pin;
  bool active_high;
} OutputConfig;

typedef struct {
  int pad;
  pin_function func;
  int flags;
} Pinmux;

typedef struct {
  GPT_HandleTypeDef handle;
  GPT_ClockConfigTypeDef clock_config;
  uint16_t value;
  uint16_t resolution;
  int enabled;
  uint16_t channel;
  uint8_t  is_comp;
} PwmState;

typedef struct {
  Pinmux pwm_pin;
  PwmState *state;
} PwmConfig;

typedef struct {
  const OutputConfig ctl;
} BoardConfigActuator;

typedef struct {
  uint8_t backlight_on_percent;
  //ambient light config
  uint32_t ambient_light_dark_threshold;
  uint32_t ambient_k_delta_threshold;
  // Raw-count -> lux conversion: lux = (level - offset) * num / den.
  // den == 0 means no conversion available for this board.
  uint32_t ambient_light_lux_dark_offset;
  uint32_t ambient_light_lux_num;
  uint32_t ambient_light_lux_den;
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  // Default RGB backlight color (packed 0x00RRGGBB), applied when no app
  // override is set. User-preference overrides this via backlight_set_color().
  uint32_t backlight_default_color;
#endif
} BoardConfig;

typedef struct {
  const char* name;
  GPIO_TypeDef* const port;
  uint8_t pin;
  GPIOPuPd_TypeDef pull;
  bool active_high;
} ButtonConfig;

typedef struct {
  ButtonConfig buttons[NUM_BUTTONS];
  GPT_TypeDef *timer;
  IRQn_Type timer_irqn;
} BoardConfigButton;

typedef struct {
  ExtiConfig pmic_int;
  //! Percentage for watch only mode
  const uint8_t low_power_threshold;
  //! Approximate hours of battery life
  const uint16_t battery_capacity_hours;
} BoardConfigPower;

typedef struct {
  uint8_t default_motion_sensitivity;
} BoardConfigAccel;

typedef struct {
  int axes_offsets[3];
  bool axes_inverts[3];
} MagConfig;

typedef struct {
  const MagConfig mag_config;
} BoardConfigMag;

#include <pbl/drivers/flash/qspi_flash_definitions.h>
#include <pbl/drivers/qspi_definitions.h>
#include <pbl/drivers/uart/sf32lb.h>
#include <pbl/drivers/display/sf32lb/display_jdi.h>
#include <pbl/drivers/mic/sf32lb52/pdm_definitions.h>
#include <pbl/drivers/speaker/sf32lb52/audio_definitions.h>

typedef const struct UARTDevice UARTDevice;
typedef const struct I2CBus I2CBus;
typedef const struct I2CSlavePort I2CSlavePort;
typedef const struct HRMDevice HRMDevice;
typedef const struct MicDevice MicDevice;
typedef const struct QSPIPort QSPIPort;
typedef const struct QSPIFlash QSPIFlash;
typedef const struct DisplayJDIDevice DisplayJDIDevice;
typedef const struct AudioDevice AudioDevice;

#include <pbl/drivers/i2c/definitions.h>
#include <pbl/drivers/i2c/sf32lb.h>

void board_early_init(void);
void board_init(void);

#include "board_definitions.h"
