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

#include "setup.h"

#include "kernel/util/stop.h"
#include "system/logging.h"
#include "system/passert.h"

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF5_COMPATIBLE
#define SF32LB52_COMPATIBLE
#include <mcu.h>

void enable_mcu_debugging(void) {
#ifndef RELEASE
#if defined(MICRO_FAMILY_NRF52840)
//  NRF_APPROTECT->APPROTECT.DISABLE = 1;
#elif defined(MICRO_FAMILY_SF32LB52)
// TODO(SF32LB52): implement
#else
  DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP, ENABLE);
  // Stop RTC, IWDG & TIM2 during debugging
  // Note: TIM2 is used by the task watchdog
  DBGMCU_APB1PeriphConfig(DBGMCU_RTC_STOP | DBGMCU_TIM2_STOP | DBGMCU_IWDG_STOP,
                          ENABLE);
#endif
#endif
}

void disable_mcu_debugging(void) {
#if defined(MICRO_FAMILY_NRF52840)
//  NRF_APPROTECT->APPROTECT.DISABLE = 0;
#elif MICRO_FAMILY_SF32LB52
// TODO(SF32LB52): implement
#else
  DBGMCU->CR = 0;
  DBGMCU->APB1FZ = 0;
  DBGMCU->APB2FZ = 0;
#endif
}

void command_low_power_debug(char *cmd) {
  bool low_power_debug_on = (strcmp(cmd, "on") == 0);

#ifdef MICRO_FAMILY_STM32F4
  sleep_mode_enable(!low_power_debug_on);
#endif

  if (low_power_debug_on) {
    enable_mcu_debugging();
  } else {
    disable_mcu_debugging();
  }
}
