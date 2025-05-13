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

#pragma once

#if MICRO_FAMILY_NRF5

#define RTC_BKP_BOOTBIT_DR                      0
#define STUCK_BUTTON_REGISTER                   1
#define BOOTLOADER_VERSION_REGISTER             2
#define CURRENT_TIME_REGISTER                   3
#define CURRENT_INTERVAL_TICKS_REGISTER         4
#define REBOOT_REASON_REGISTER_1                5
#define REBOOT_REASON_REGISTER_2                6
#define REBOOT_REASON_STUCK_TASK_PC             7
#define REBOOT_REASON_STUCK_TASK_LR             8
#define REBOOT_REASON_STUCK_TASK_CALLBACK       9
#define REBOOT_REASON_DROPPED_EVENT             10
// formerly REBOOT_REASON_MUTEX_PC
#define RTC_BKP_FLASH_ERASE_PROGRESS            11
#define RTC_TIMEZONE_ABBR_START                 12
#define RTC_TIMEZONE_ABBR_END_TZID_DSTID        13
#define RTC_TIMEZONE_GMTOFFSET                  14
#define RTC_TIMEZONE_DST_START                  15
#define RTC_TIMEZONE_DST_END                    16
#define MAG_XY_CORRECTION_VALS                  17
#define MAG_Z_CORRECTION_VAL                    18
#define SLOT_OF_LAST_LAUNCHED_APP               19

// We want a checksum stored in the retained registers, to prove that they
// are not *totally* full of noise from system-on reset -- otherwise, we
// zero them out and give up.
#define NRF_RETAINED_REGISTER_CRC               31

extern void retained_write(uint8_t id, uint32_t value);
extern uint32_t retained_read(uint8_t id);

#elif MICRO_FAMILY_SF32LB52
#define RTC_BKP_BOOTBIT_DR                      0
#define STUCK_BUTTON_REGISTER                   1
#define BOOTLOADER_VERSION_REGISTER             2
#define CURRENT_TIME_REGISTER                   3
#define CURRENT_INTERVAL_TICKS_REGISTER         4
#define REBOOT_REASON_REGISTER_1                5
#define REBOOT_REASON_REGISTER_2                6
#define REBOOT_REASON_STUCK_TASK_PC             7
#define REBOOT_REASON_STUCK_TASK_LR             8
#define REBOOT_REASON_STUCK_TASK_CALLBACK       9
#define REBOOT_REASON_DROPPED_EVENT             10

#else
#define RTC_BKP_BOOTBIT_DR                      RTC_BKP_DR0
#define STUCK_BUTTON_REGISTER                   RTC_BKP_DR1
#define BOOTLOADER_VERSION_REGISTER             RTC_BKP_DR2
#define CURRENT_TIME_REGISTER                   RTC_BKP_DR3
#define CURRENT_INTERVAL_TICKS_REGISTER         RTC_BKP_DR4
#define REBOOT_REASON_REGISTER_1                RTC_BKP_DR5
#define REBOOT_REASON_REGISTER_2                RTC_BKP_DR6
#define REBOOT_REASON_STUCK_TASK_PC             RTC_BKP_DR7
#define REBOOT_REASON_STUCK_TASK_LR             RTC_BKP_DR8
#define REBOOT_REASON_STUCK_TASK_CALLBACK       RTC_BKP_DR9
#define REBOOT_REASON_DROPPED_EVENT             RTC_BKP_DR10
// formerly REBOOT_REASON_MUTEX_PC
#define RTC_BKP_FLASH_ERASE_PROGRESS            RTC_BKP_DR11
#define RTC_TIMEZONE_ABBR_START                 RTC_BKP_DR12
#define RTC_TIMEZONE_ABBR_END_TZID_DSTID        RTC_BKP_DR13
#define RTC_TIMEZONE_GMTOFFSET                  RTC_BKP_DR14
#define RTC_TIMEZONE_DST_START                  RTC_BKP_DR15
#define RTC_TIMEZONE_DST_END                    RTC_BKP_DR16
#define MAG_XY_CORRECTION_VALS                  RTC_BKP_DR17
#define MAG_Z_CORRECTION_VAL                    RTC_BKP_DR18
#define SLOT_OF_LAST_LAUNCHED_APP               RTC_BKP_DR19
#endif
