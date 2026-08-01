/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/util/standby.h"

#include <pbl/drivers/display/display.h>
#include <pbl/drivers/pmic.h>
#ifdef CONFIG_HIBERNATE
#include "bf0_hal_pmu.h"
#endif
#include "system/bootbits.h"
#include <pbl/logging/logging.h>
#include "system/reset.h"
#include "system/passert.h"

#ifdef CONFIG_PMIC
static NORETURN prv_enter_standby(void) {
  pmic_power_off();

  PBL_CROAK("We were not shut down!");
}
#else
static NORETURN prv_enter_standby(void) {
  boot_bit_set(BOOT_BIT_STANDBY_MODE_REQUESTED);
  system_hard_reset();
}
#endif

NORETURN enter_standby(RebootReasonCode reason) {
  PBL_LOG_ALWAYS("Preparing to enter standby mode.");

  RebootReason reboot_reason = { reason, 0 };
  reboot_reason_set(&reboot_reason);

  display_clear();
  display_set_enabled(false);

  system_reset_prepare();
  reboot_reason_set_restarted_safely();

  prv_enter_standby();
}

#ifdef CONFIG_HIBERNATE
NORETURN enter_mcu_shutdown(RebootReasonCode reason) {
  PBL_LOG_ALWAYS("Preparing to enter MCU shutdown.");

  RebootReason reboot_reason = { reason, 0 };
  reboot_reason_set(&reboot_reason);

  display_clear();
  display_set_enabled(false);

  system_reset_prepare();
  reboot_reason_set_restarted_safely();

  // Hibernate keeps the RTC/backup domain powered, so the time is preserved.
  // The watch wakes only from the back key (PA34) and cold-boots from there.

  // Use the low-power 10 kHz clock for the wake counter: it draws less
  // current and gives a big enough range for multi-second debounces.
  HAL_PMU_LpCLockSelect(PMU_LPCLK_RC10);

  // Disable all wakeup sources, then enable only the back key on wakeup pin 0.
  hwp_pmuc->WER = 0;
  // ~2 s debounce at 10 kHz (20000 cycles) to keep the hold time short while
  // still ignoring brief accidental presses.
  hwp_pmuc->WKUP_CNT = 0x00004E20;
  HAL_PMU_SelectWakeupPin(0, 10);    // PA34 (OBELIX back key)
  HAL_PMU_EnablePinWakeup(0, 1);     // low-level, active low

  HAL_PMU_EnterHibernate();

  PBL_CROAK("MCU shutdown returned unexpectedly.");
}
#endif
