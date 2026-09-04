/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/types.h"

//! Starts the scheduler; never returns.
void pbl_kernel_start(void) __attribute__((noreturn));

//! The scheduler has started.
bool pbl_kernel_is_started(void);

//! The scheduler has started and preemption is not locked.
bool pbl_kernel_is_running(void);

//! Disables preemption; ISRs still run. Nestable.
void pbl_sched_lock(void);
void pbl_sched_unlock(void);
bool pbl_sched_is_locked(void);

pbl_tick_t pbl_uptime_ticks(void);
