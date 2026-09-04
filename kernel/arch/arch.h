/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pbl/kernel/debug.h"
#include "pbl/kernel/thread.h"

//! What the portable kernel needs from an architecture.

void arch_init(void);

//! Builds the initial context so that the first switch to @p t runs
//! @p entry(@p arg) on the thread's stack.
void arch_thread_init(struct pbl_thread *t, void (*entry)(void *), void *arg);

void arch_thread_regions_set(struct pbl_thread *t, const MpuRegion *const *regions);

//! Starts running pbl_cur; never returns.
void arch_start(void) __attribute__((noreturn));

//! Asks for a context switch. From a thread it happens once interrupts are
//! unlocked; from an ISR, when the ISR returns.
void arch_switch_request(void);

//! Ends the calling thread; never returns.
void arch_thread_exit(void) __attribute__((noreturn));

//! Another thread has been aborted and will never be switched to again.
void arch_thread_aborted(struct pbl_thread *t);

bool arch_in_isr(void);
void arch_irq_disable(void);
void arch_irq_enable(void);

void arch_thread_saved_regs(const struct pbl_thread *t, struct pbl_thread_saved_regs *regs);
void arch_thread_info_regs(const struct pbl_thread *t, uint32_t regs[PBL_THREAD_REG_COUNT]);

//! Idle-thread hook: nothing is runnable for up to @p max_ticks (0 when a
//! timeout is already due, PBL_TICK_FOREVER when none is pending).
void arch_idle(pbl_tick_t max_ticks);
