/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "pbl/mcu/interrupts.h"

#include "kernel_freertos.h"

_Static_assert(configMAX_SYSCALL_INTERRUPT_PRIORITY == PBL_IRQ_PRIO_MAX_SYSCALL,
               "FreeRTOSConfig.h and CONFIG_KERNEL_IRQ_PRIO_MAX_SYSCALL disagree");
_Static_assert(configKERNEL_INTERRUPT_PRIORITY == PBL_IRQ_PRIO_KERNEL,
               "FreeRTOSConfig.h and CONFIG_KERNEL_IRQ_PRIO_KERNEL disagree");

void pbl_irq_lock(void) { portENTER_CRITICAL(); }

void pbl_irq_unlock(void) { portEXIT_CRITICAL(); }

bool pbl_in_isr(void) { return mcu_state_is_isr(); }

bool pbl_irq_is_locked(void) { return portIN_CRITICAL(); }
