/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "pbl/soc/sf32lb/sleep.h"

#include "system/passert.h"

//! One refcount per blockable level. SOC_SF32LB_ACTIVE cannot be blocked
//! (the CPU is always allowed to stay active), so index 0 is unused.
static int s_block_count[SOC_SF32LB_DEEPSLEEP + 1];

void soc_sf32lb_sleep_block(SocSf32lbSleepLevel level) {
  PBL_ASSERTN(level > SOC_SF32LB_ACTIVE && level <= SOC_SF32LB_DEEPSLEEP);
  pbl_irq_lock();
  ++s_block_count[level];
  pbl_irq_unlock();
}

void soc_sf32lb_sleep_release(SocSf32lbSleepLevel level) {
  PBL_ASSERTN(level > SOC_SF32LB_ACTIVE && level <= SOC_SF32LB_DEEPSLEEP);
  pbl_irq_lock();
  PBL_ASSERTN(s_block_count[level] > 0);
  --s_block_count[level];
  pbl_irq_unlock();
}

SocSf32lbSleepLevel soc_sf32lb_sleep_max_level(void) {
  // The shallowest blocked level forbids itself and everything deeper, so the
  // deepest permitted level is one step shallower than it.
  for (SocSf32lbSleepLevel level = SOC_SF32LB_WFI; level <= SOC_SF32LB_DEEPSLEEP; ++level) {
    if (s_block_count[level] > 0) {
      return level - 1;
    }
  }
  return SOC_SF32LB_DEEPSLEEP;
}
