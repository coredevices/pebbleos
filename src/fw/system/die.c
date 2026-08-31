/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "logging/logging_private.h"
#include "logging/pulse_logging.h"
#include "system/bootbits.h"
#include "system/reset.h"

#include <cmsis_core.h>

#if defined(CONFIG_NO_WATCHDOG)
#include "FreeRTOS.h"
#endif

void prepare_for_software_failure(void) {
#ifdef CONFIG_PULSE_EVERYWHERE
  pulse_logging_log_buffer_flush();
#endif

#ifndef CONFIG_MFG
  boot_bit_set(BOOT_BIT_SOFTWARE_FAILURE_OCCURRED);
#endif
}

NORETURN reset_due_to_software_failure(void) {
  prepare_for_software_failure();

#if defined(CONFIG_NO_WATCHDOG)
  // Don't reset right away, leave it in a state we can inspect

  __disable_irq();
  while (1) {
    continue;
  }
#endif

  PBL_LOG_FROM_FAULT_HANDLER("Resetting!");
  system_reset();
}
