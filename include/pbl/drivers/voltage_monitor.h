/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include "board/board.h"

#define NUM_CONVERSIONS 40

#if defined(CONFIG_SOC_NRF52)
# include <hal/nrf_saadc.h>

typedef const struct VoltageMonitorDevice {
  NRF_SAADC_Type *const adc; ///< One of ADCX. For example ADC1.
  const uint8_t adc_channel; ///< One of ADC_Channel_*
  const nrf_saadc_input_t input;
} VoltageMonitorDevice;

#elif defined(CONFIG_SOC_SF32LB52) || defined(CONFIG_QEMU)

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
//!
//! @param device Pointer to the Voltage Monitor Device to be used.
//! @param reading_out Pointer to a VoltageReading struct which the results will be returned in.
void voltage_monitor_read(const VoltageMonitorDevice *device, VoltageReading *reading_out);
void voltage_monitor_read_temp(const VoltageMonitorDevice *device, VoltageReading *reading_out);
