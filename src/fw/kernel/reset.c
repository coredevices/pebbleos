/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "system/reset.h"

#include "board/board.h"
#include "system/bootbits.h"
#include "kernel/core_dump.h"
#include "kernel/util/fw_reset.h"
#include "pbl/mcu/interrupts.h"

#include <pbl/drivers/flash.h>
#include "system/reboot_reason.h"

#ifdef CONFIG_SOC_SF32LB52
#include <bf0_hal.h>
#endif

#include "FreeRTOS.h"
#include "task.h"

void system_reset_prepare(void) {
  fw_prepare_for_reset();
  flash_stop();
}

NORETURN system_reset(void) {
  static bool failure_occurred = false;

  bool already_failed = failure_occurred;
  if (!failure_occurred) {
    // Don't overwrite failure_occurred if a failure has already occurred
    failure_occurred = boot_bit_test(BOOT_BIT_SOFTWARE_FAILURE_OCCURRED);
  }

  // Skip safe teardown if doing so the first time already caused a second reset attempt; or
  // if we're in a critical section, interrupt or if the scheduler has been suspended
  if (!already_failed && !mcu_state_is_isr() && !portIN_CRITICAL() &&
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
    system_reset_prepare();
    reboot_reason_set_restarted_safely();
  }

  // If a software failure occcured, do a core dump before resetting
  if (failure_occurred) {
    core_dump_reset(false /* don't force overwrite */);
  } else {
    system_hard_reset();
  }
}

void system_reset_callback(void *data) {
  system_reset();
  (void)data;
}

NORETURN system_hard_reset(void) {
  // Don't do anything fancy here. We may be in a context where nothing works, not even
  // interrupts. Just reset us.

#ifdef CONFIG_SOC_SF32LB52
  HAL_PMU_Reboot();
#else
  NVIC_SystemReset();
#endif

  __builtin_unreachable();
}

