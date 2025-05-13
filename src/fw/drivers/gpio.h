/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdbool.h>

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF5_COMPATIBLE
#define SF32LB52_COMPATIBLE
#include <mcu.h>

#include "board/board.h"

#ifdef MICRO_FAMILY_NRF5
#include <hal/nrf_gpio.h>

typedef enum {
  GPIO_OType_PP,
  GPIO_OType_OD,
} GPIOOType_TypeDef;

typedef enum {
  GPIO_PuPd_NOPULL,
  GPIO_PuPd_UP,
  GPIO_PuPd_DOWN,
} GPIOPuPd_TypeDef;

typedef enum {
  GPIO_Speed_2MHz,
  GPIO_Speed_50MHz,
  GPIO_Speed_200MHz
} GPIOSpeed_TypeDef;

#endif

#ifdef MICRO_FAMILY_NRF5

void gpio_use(uint32_t pin);
void gpio_release(uint32_t pin);

#else

void gpio_use(GPIO_TypeDef* GPIOx);
void gpio_release(GPIO_TypeDef* GPIOx);

#endif

//! Initialize a GPIO as an output.
//!
//! @param pin_config the BOARD_CONFIG pin configuration struct
//! @param otype the output type of the pin (GPIO_OType_PP or GPIO_OType_OD)
//! @param speed the output slew rate
//! @note The slew rate should be set as low as possible for the
//!       pin function to minimize ringing and RF interference.
void gpio_output_init(const OutputConfig *pin_config, GPIOOType_TypeDef otype,
                      GPIOSpeed_TypeDef speed);

//! Assert or deassert the output pin.
//!
//! Asserting the output drives the pin high if pin_config.active_high
//! is true, and drives it low if pin_config.active_high is false.
void gpio_output_set(const OutputConfig *pin_config, bool asserted);

#ifndef MICRO_FAMILY_NRF5

//! Configure a GPIO alternate function.
//!
//! @param pin_config the BOARD_CONFIG pin configuration struct
//! @param otype the output type of the pin (GPIO_OType_PP or GPIO_OType_OD)
//! @param speed the output slew rate
//! @param pupd pull-up or pull-down configuration
//! @note The slew rate should be set as low as possible for the
//!       pin function to minimize ringing and RF interference.
void gpio_af_init(const AfConfig *af_config, GPIOOType_TypeDef otype,
                  GPIOSpeed_TypeDef speed, GPIOPuPd_TypeDef pupd);

//! Configure a GPIO alternate function pin to minimize power consumption.
//!
//! Once a pin has been configured for low power, it is no longer
//! connected to its alternate function. \ref gpio_af_init will need to
//! be called again on the pin in order to configure it in alternate
//! function mode again.
void gpio_af_configure_low_power(const AfConfig *af_config);

//! Configure a GPIO alternate function pin to drive a constant output.
//!
//! Once a pin has been configured as a fixed output, it is no longer
//! connected to its alternate function. \ref gpio_af_init will need to
//! be called again on the pin in order to configure it in alternate
//! function mode again.
void gpio_af_configure_fixed_output(const AfConfig *af_config, bool asserted);

#endif

//! Configure all GPIOs in the system to optimize for power consumption.
//! At poweron most GPIOs can be configured as analog inputs instead of the
//! default digital input. This allows digital filtering logic to be shut down,
//! saving quite a bit of power.
void gpio_init_all(void);

//! Configure gpios as inputs (suitable for things like exti lines)
void gpio_input_init(const InputConfig *input_cfg);

//! Configure gpio as an input with internal pull up/pull down configured.
void gpio_input_init_pull_up_down(const InputConfig *input_cfg, GPIOPuPd_TypeDef pupd);

//! @return bool the current state of the GPIO pin
bool gpio_input_read(const InputConfig *input_cfg);

//! Configure gpios as analog inputs. Useful for unused GPIOs as this is their lowest power state.
void gpio_analog_init(const InputConfig *input_cfg);
