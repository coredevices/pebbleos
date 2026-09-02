/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

//! NVIC priority of the kernel's own interrupts (tick, context switch).
#define PBL_IRQ_PRIO_KERNEL CONFIG_KERNEL_IRQ_PRIO_KERNEL
//! Highest NVIC priority (lowest value) from which kernel calls are allowed.
#define PBL_IRQ_PRIO_MAX_SYSCALL CONFIG_KERNEL_IRQ_PRIO_MAX_SYSCALL

//! Masks interrupts up to PBL_IRQ_PRIO_MAX_SYSCALL. Nestable; usable from
//! threads and ISRs.
void pbl_irq_lock(void);
void pbl_irq_unlock(void);

bool pbl_in_isr(void);
bool pbl_irq_is_locked(void);
