/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/kernel/irq.h>

void pbl_irq_lock(void) {
}

void pbl_irq_unlock(void) {
}

bool pbl_irq_is_locked(void) {
  return false;
}

bool pbl_in_isr(void) {
  return false;
}

void sys_psleep(int millis) {
}
