/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "FreeRTOS.h"
#include "pbl/kernel/types.h"
#include "portmacro.h"

#include <stdint.h>

TickType_t milliseconds_to_ticks(uint32_t milliseconds) {
  return ((uint64_t)milliseconds * configTICK_RATE_HZ) / 1000;
}

TickType_t ticks_to_milliseconds(uint32_t ticks) {
  return ((uint64_t)ticks * 1000) / configTICK_RATE_HZ;
}

pbl_tick_t pbl_ms_to_ticks(uint32_t ms) {
  return ((uint64_t)ms * configTICK_RATE_HZ) / 1000;
}

uint32_t pbl_ticks_to_ms(pbl_tick_t ticks) {
  return ((uint64_t)ticks * 1000) / configTICK_RATE_HZ;
}
