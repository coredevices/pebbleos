/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <cmsis_core.h>

#include "kernel/util/idle.h"

#include "pbl/kernel/idle.h"

void pbl_soc_idle(pbl_tick_t max_ticks) {
  if (!idle_is_allowed()) {
    return;
  }

  __disable_irq();

  if (pbl_idle_confirm()) {
    __DSB();
    __WFI();
    __ISB();
  }

  __enable_irq();
}

bool pbl_soc_tick_enable(void) {
  return false;
}

void dump_current_runtime_stats(void) {
}

void pbl_analytics_external_collect_cpu_stats(void) {
}