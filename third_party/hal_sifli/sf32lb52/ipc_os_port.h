/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "bf0_hal.h"
#include "pbl/kernel/irq.h"

static inline int os_interrupt_disable(void) {
  pbl_irq_lock();
  return 0;
}

static inline void os_interrupt_enable(int mask) {
  pbl_irq_unlock();
}

#define os_interrupt_enter()
#define os_interrupt_exit()

#define os_interrupt_start(irq_number, priority, sub_priority) \
  do {                                                         \
    HAL_NVIC_SetPriority(irq_number, priority, sub_priority);  \
    HAL_NVIC_EnableIRQ(irq_number);                            \
  } while (0)

#define os_interrupt_stop(irq_number) HAL_NVIC_DisableIRQ(irq_number)
