/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

struct AnalogTemperatureSensor {
  const VoltageMonitorDevice *voltage_monitor;
  int32_t millivolts_ref;
  int32_t millidegrees_ref;
  int32_t slope_numerator;
  int32_t slope_denominator;
};
