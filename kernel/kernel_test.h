/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

//! Host-only harness for the kernel on the POSIX arch.

//! Runs the scheduler with the threads created so far until a thread calls
//! pbl_test_kernel_stop(); then tears every thread down.
void pbl_test_kernel_run(void);
void pbl_test_kernel_stop(void);

//! Brackets kernel calls that should behave as if made from an ISR.
void pbl_test_isr_enter(void);
void pbl_test_isr_exit(void);

//! Delivers @p ticks tick interrupts.
void pbl_test_tick(uint32_t ticks);

//! Forgets every thread and resets the clock; called by the harness.
void sched_reset_for_test(void);
