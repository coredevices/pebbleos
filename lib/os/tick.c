/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "FreeRTOS.h"

#include <stdint.h>

TickType_t milliseconds_to_ticks(uint32_t milliseconds) {
  return ((uint64_t)milliseconds * configTICK_RATE_HZ) / 1000;
}

TickType_t ticks_to_milliseconds(uint32_t ticks) {
  return ((uint64_t)ticks * 1000) / configTICK_RATE_HZ;
}
