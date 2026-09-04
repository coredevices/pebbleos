/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel.h"

static volatile uint32_t s_nesting;

void pbl_irq_lock(void) {
  arch_irq_disable();
  s_nesting++;
}

void pbl_irq_unlock(void) {
  KERNEL_ASSERT(s_nesting > 0);
  if (--s_nesting == 0) {
    arch_irq_enable();
  }
}

void irq_reset(void) { s_nesting = 0; }

bool pbl_in_isr(void) { return arch_in_isr(); }

bool pbl_irq_is_locked(void) { return s_nesting > 0; }
