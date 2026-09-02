/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/types.h"

#include <stdint.h>

pbl_tick_t pbl_ms_to_ticks(uint32_t ms) {
  return ((uint64_t)ms * PBL_TICK_HZ) / 1000;
}

uint32_t pbl_ticks_to_ms(pbl_tick_t ticks) {
  return ((uint64_t)ticks * 1000) / PBL_TICK_HZ;
}
