/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/types.h"

//! Implemented by the SoC. Runs on the idle thread when nothing is runnable
//! for up to @p max_ticks; may sleep. Once interrupts are masked it must
//! check pbl_idle_confirm() before sleeping, and report through
//! pbl_idle_slept() any ticks that passed while the tick interrupt was off.
void pbl_soc_idle(pbl_tick_t max_ticks);

//! Implemented by the SoC. Sets up the tick interrupt when the scheduler
//! starts. @return false to let the kernel program SysTick itself.
bool pbl_soc_tick_enable(void);

//! @return false if something became runnable since the idle thread decided
//! to sleep. Call with interrupts masked.
bool pbl_idle_confirm(void);

//! Accounts @p elapsed ticks of sleep during which no tick interrupt ran.
void pbl_idle_slept(pbl_tick_t elapsed);

//! The tick interrupt body, for SoCs that own the SysTick handler.
void pbl_kernel_tick_isr(void);
