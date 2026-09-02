/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "pbl/soc/nrf/sleep.h"

#include "system/passert.h"

static int s_block_count;

void soc_nrf_sleep_full_block(void) {
  pbl_irq_lock();
  ++s_block_count;
  pbl_irq_unlock();
}

void soc_nrf_sleep_full_release(void) {
  pbl_irq_lock();
  PBL_ASSERTN(s_block_count > 0);
  --s_block_count;
  pbl_irq_unlock();
}

bool soc_nrf_sleep_full_is_allowed(void) {
  return s_block_count == 0;
}
