/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "pbl/kernel/sched.h"

#include "kernel_freertos.h"

void pbl_kernel_start(void) {
  vTaskStartScheduler();
  for (;;) {
  }
}

bool pbl_kernel_is_started(void) { return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED; }

bool pbl_kernel_is_running(void) { return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING; }

void pbl_sched_lock(void) { vTaskSuspendAll(); }

void pbl_sched_unlock(void) { xTaskResumeAll(); }

bool pbl_sched_is_locked(void) { return xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED; }

pbl_tick_t pbl_uptime_ticks(void) {
  return pbl_in_isr() ? xTaskGetTickCountFromISR() : xTaskGetTickCount();
}
