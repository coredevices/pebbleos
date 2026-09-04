/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/soc/nrf/sleep.h"

#include "system/passert.h"

#include "FreeRTOS.h"

static int s_block_count;

void soc_nrf_sleep_full_block(void) {
  portENTER_CRITICAL();
  ++s_block_count;
  portEXIT_CRITICAL();
}

void soc_nrf_sleep_full_release(void) {
  portENTER_CRITICAL();
  PBL_ASSERTN(s_block_count > 0);
  --s_block_count;
  portEXIT_CRITICAL();
}

bool soc_nrf_sleep_full_is_allowed(void) {
  return s_block_count == 0;
}
