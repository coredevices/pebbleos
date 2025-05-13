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

#include <stdint.h>
#include <stdbool.h>

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF5_COMPATIBLE
#define SF32LB52_COMPATIBLE
#include <mcu.h>

#include "board/board.h"

#define NUM_CONVERSIONS 40

#if defined(MICRO_FAMILY_NRF5)
# include <hal/nrf_saadc.h>

typedef const struct VoltageMonitorDevice {
  NRF_SAADC_Type *const adc; ///< One of ADCX. For example ADC1.
  const uint8_t adc_channel; ///< One of ADC_Channel_*
  const nrf_saadc_input_t input;
} VoltageMonitorDevice;

#elif defined(MICRO_FAMILY_SF32LB52)

// TODO(SF32LB52): Add implementation
typedef const struct VoltageMonitorDevice {
} VoltageMonitorDevice;

#else

typedef const struct VoltageMonitorDevice {
  ADC_TypeDef *const adc; ///< One of ADCX. For example ADC1.
  const uint8_t adc_channel; ///< One of ADC_Channel_*
  uint32_t clock_ctrl;  ///< Peripheral clock control flag
  const InputConfig input;
} VoltageMonitorDevice;

#endif

//! The current voltage numbers from the given ADC, read using adc_read().
//! Each _total value is a sum of \ref NUM_CONVERSIONS samples where each sample
//! is a number in the scale [0, 4095].
typedef struct {
  uint32_t vmon_total;
  uint32_t vref_total;
} VoltageReading;

void voltage_monitor_init(void);
void voltage_monitor_device_init(const VoltageMonitorDevice *device);

//! Get a voltage reading from the given ADC.
//! Implementation is hardware specific, since Vref is only available on ADC1.
//!
//! On the STM32F412xG, which only has a single ADC:
//!   - ADC1 is configured in scan mode, and will scan the ADC channel given in \ref adc, and Vref.
//!
//! On all other F2 and F4 platforms, which have multiple ADCs:
//!   - ADC1 is configured only to pull Vref.
//!   - The given adc must not be ADC1
//!
//! @param device Pointer to the Voltage Monitor Device to be used.
//! @param reading_out Pointer to a VoltageReading struct which the results will be returned in.
void voltage_monitor_read(const VoltageMonitorDevice *device, VoltageReading *reading_out);
void voltage_monitor_read_temp(const VoltageMonitorDevice *device, VoltageReading *reading_out);
