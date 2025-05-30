/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kernel/util/standby.h"

#include "drivers/imu.h"
#include "drivers/rtc.h"
#include "drivers/flash.h"
#include "drivers/pmic.h"
#include "drivers/pwr.h"
#include "drivers/periph_config.h"
#include "system/bootbits.h"
#include "system/logging.h"
#include "system/reset.h"

#include "drivers/display/display.h"

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF5_COMPATIBLE
#define SF32LB_COMPATIBLE
#include <mcu.h>

#include "FreeRTOS.h"
#include "task.h"

//! If we don't have a PMIC entering standby is a little more complicated.
//! See platform/tintin/boot/src/standby.c.
//! We set a bootbit and reboot, and then the bootloader is responsible for really winding us down.
//! This is necessary because:
//!    - When entering standby, a system reset is the only way to disable the IWDG
//!    - When shutting down, it simplifies waiting on the charger to be removed,
//!      and allows for us to handle other boot bits (eg. Force PRF) before powering down.
//!
//! @param boot_bit Boot bit to set. It should cause the bootloader to shut down or enter standby.
static NORETURN prv_enter_standby_non_pmic(BootBitValue boot_bit) {
  // The I2C bus is not initialized in the bootloader.
  // Put the Accelerometer into low power mode before resetting
  imu_power_down();

  boot_bit_set(boot_bit);

  PBL_LOG(LOG_LEVEL_ALWAYS, "Rebooting to enter Standby mode.");
  reboot_reason_set_restarted_safely();

  system_hard_reset();
}

#if CAPABILITY_HAS_PMIC
static NORETURN prv_enter_standby_pmic(void) {
  reboot_reason_set_restarted_safely();

#if defined(TARGET_QEMU)
#if MICRO_FAMILY_STM32F7
  WTF; // Unsupported
#else
  // QEMU does not implement i2c devices, like the PMIC, yet. Let's turn off instead
  // by going into standby mode using the power control of the STM32. We can't use
  // prv_enter_standby_non_pmic() because PMIC based boards don't support that feature in their
  // bootloader.
  periph_config_enable(PWR, RCC_APB1Periph_PWR);
  pwr_enable_wakeup(true);
  PWR_EnterSTANDBYMode();
#endif
#endif

  PBL_LOG(LOG_LEVEL_ALWAYS, "Using the PMIC to enter standby mode.");
  pmic_power_off();
  PBL_CROAK("PMIC didn't shut us down!");
}
#endif

NORETURN enter_standby(RebootReasonCode reason) {
  PBL_LOG(LOG_LEVEL_ALWAYS, "Preparing to enter standby mode.");

  RebootReason reboot_reason = { reason, 0 };
  reboot_reason_set(&reboot_reason);

  // Wipe display
  display_clear();

  /* skip BT teardown if BT isn't working */
  system_reset_prepare(reason == RebootReasonCode_DialogBootFault);

#if PLATFORM_SILK && RECOVERY_FW
  // For Silk PRF & MFG firmwares, fully shutdown the watch using the bootloader.
  // Always entering full shutdown in these two situations will guarantee a much
  // better shelf-life, and ensure that watches are shipped in full shutdown mode.
  //
  // Request the bootloader to completely power down as the last thing it does,
  // rather than jumping into the fw. The bootloader may spin on the charger
  // connection status, as we cannot shutdown while the charger is plugged.
  // Luckily, we never try to power down the watch while plugged when in PRF.
  prv_enter_standby_non_pmic(BOOT_BIT_SHUTDOWN_REQUESTED);
#elif CAPABILITY_HAS_PMIC
  prv_enter_standby_pmic();
#else
  // Request the bootloader to enter standby mode immediately after the system is reset.
  prv_enter_standby_non_pmic(BOOT_BIT_STANDBY_MODE_REQUESTED);
#endif
}
