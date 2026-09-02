/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/idle.h"

#include "kernel_freertos.h"

void vPortSuppressTicksAndSleep(TickType_t expected_idle_ticks) {
  pbl_soc_idle(expected_idle_ticks);
}

bool vPortEnableTimer(void) { return pbl_soc_tick_enable(); }

bool pbl_idle_confirm(void) { return eTaskConfirmSleepModeStatus() != eAbortSleep; }

void pbl_idle_slept(pbl_tick_t elapsed) { vTaskStepTick(elapsed); }

void pbl_kernel_tick_isr(void) {
  extern void xPortSysTickHandler(void);
  xPortSysTickHandler();
}
