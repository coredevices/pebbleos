/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "reboot_reason.h"

#include "pbl/mcu/interrupts.h"
#include "pbl/os/tick.h"
#include <pbl/logging/logging.h>

#ifdef CONFIG_SOC_SF32LB52
#include <bf0_hal.h>
#endif

#ifdef CONFIG_QEMU
extern void RTC_WriteBackupRegister(uint32_t reg_id, uint32_t value);
extern uint32_t RTC_ReadBackupRegister(uint32_t reg_id);
#endif

#include "FreeRTOS.h"
#include "task.h"

_Static_assert(sizeof(RebootReason) == sizeof(uint32_t[4]), "RebootReason is a funny size");

void reboot_reason_set(RebootReason *reason) {
#ifdef CONFIG_SOC_NRF52
  uint32_t *raw = (uint32_t*)reason;

  if (retained_read(REBOOT_REASON_REGISTER_1)) {
    // It's not safe to log if we're called from an ISR or from a FreeRTOS critical section (basepri != 0)
    if (!mcu_state_is_isr() && __get_BASEPRI() == 0
            && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      PBL_LOG_WRN("Reboot reason is already set");
    }
    return;
  }

  retained_write(REBOOT_REASON_REGISTER_1, raw[0]);
  retained_write(REBOOT_REASON_STUCK_TASK_PC, raw[1]);
  retained_write(REBOOT_REASON_STUCK_TASK_LR, raw[2]);
  retained_write(REBOOT_REASON_STUCK_TASK_CALLBACK, raw[3]);
#elif defined(CONFIG_SOC_SF32LB52)
  uint32_t *raw = (uint32_t*)reason;

  if (HAL_Get_backup(REBOOT_REASON_REGISTER_1)) {
    // It's not safe to log if we're called from an ISR or from a FreeRTOS critical section (basepri != 0)
    if (!mcu_state_is_isr() && __get_BASEPRI() == 0
            && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      PBL_LOG_WRN("Reboot reason is already set");
    }
    return;
  }

  HAL_Set_backup(REBOOT_REASON_REGISTER_1, raw[0]);
  HAL_Set_backup(REBOOT_REASON_STUCK_TASK_PC, raw[1]);
  HAL_Set_backup(REBOOT_REASON_STUCK_TASK_LR, raw[2]);
  HAL_Set_backup(REBOOT_REASON_STUCK_TASK_CALLBACK, raw[3]);
#elif defined(CONFIG_QEMU)
  uint32_t *raw = (uint32_t*)reason;

  if (RTC_ReadBackupRegister(REBOOT_REASON_REGISTER_1)) {
    // It's not safe to log if we're called from an ISR or from a FreeRTOS critical section (basepri != 0)
    if (!mcu_state_is_isr() && __get_BASEPRI() == 0
            && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      PBL_LOG_WRN("Reboot reason is already set");
    }
    return;
  }

  RTC_WriteBackupRegister(REBOOT_REASON_REGISTER_1, raw[0]);
  RTC_WriteBackupRegister(REBOOT_REASON_STUCK_TASK_PC, raw[1]);
  RTC_WriteBackupRegister(REBOOT_REASON_STUCK_TASK_LR, raw[2]);
  RTC_WriteBackupRegister(REBOOT_REASON_STUCK_TASK_CALLBACK, raw[3]);
#endif
}

void reboot_reason_set_restarted_safely(void) {
  RebootReason reason;
  reboot_reason_get(&reason);
  reason.restarted_safely = true;

#ifdef CONFIG_SOC_NRF52
  uint32_t* raw = (uint32_t *)&reason;
  retained_write(REBOOT_REASON_REGISTER_1, *raw);
#elif defined(CONFIG_SOC_SF32LB52)
  uint32_t* raw = (uint32_t *)&reason;
  HAL_Set_backup(REBOOT_REASON_REGISTER_1, *raw);
#elif defined(CONFIG_QEMU)
  uint32_t* raw = (uint32_t *)&reason;
  RTC_WriteBackupRegister(REBOOT_REASON_REGISTER_1, *raw);
#endif
}

void reboot_reason_get(RebootReason *reason) {
#ifdef CONFIG_SOC_NRF52
  uint32_t *raw = (uint32_t *)reason;
  raw[0] = retained_read(REBOOT_REASON_REGISTER_1);
  raw[1] = retained_read(REBOOT_REASON_STUCK_TASK_PC);
  raw[2] = retained_read(REBOOT_REASON_STUCK_TASK_LR);
  raw[3] = retained_read(REBOOT_REASON_STUCK_TASK_CALLBACK);
#elif defined(CONFIG_SOC_SF32LB52)
  uint32_t *raw = (uint32_t *)reason;
  raw[0] = HAL_Get_backup(REBOOT_REASON_REGISTER_1);
  raw[1] = HAL_Get_backup(REBOOT_REASON_STUCK_TASK_PC);
  raw[2] = HAL_Get_backup(REBOOT_REASON_STUCK_TASK_LR);
  raw[3] = HAL_Get_backup(REBOOT_REASON_STUCK_TASK_CALLBACK);
#elif defined(CONFIG_QEMU)
  uint32_t *raw = (uint32_t *)reason;
  raw[0] = RTC_ReadBackupRegister(REBOOT_REASON_REGISTER_1);
  raw[1] = RTC_ReadBackupRegister(REBOOT_REASON_STUCK_TASK_PC);
  raw[2] = RTC_ReadBackupRegister(REBOOT_REASON_STUCK_TASK_LR);
  raw[3] = RTC_ReadBackupRegister(REBOOT_REASON_STUCK_TASK_CALLBACK);
#endif
}

void reboot_reason_clear(void) {
#ifdef CONFIG_SOC_NRF52
  retained_write(REBOOT_REASON_REGISTER_1, 0);
  retained_write(REBOOT_REASON_STUCK_TASK_PC, 0);
  retained_write(REBOOT_REASON_STUCK_TASK_LR, 0);
  retained_write(REBOOT_REASON_STUCK_TASK_CALLBACK, 0);
#elif defined(CONFIG_SOC_SF32LB52)
  HAL_Set_backup(REBOOT_REASON_REGISTER_1, 0);
  HAL_Set_backup(REBOOT_REASON_STUCK_TASK_PC, 0);
  HAL_Set_backup(REBOOT_REASON_STUCK_TASK_LR, 0);
  HAL_Set_backup(REBOOT_REASON_STUCK_TASK_CALLBACK, 0);
#elif defined(CONFIG_QEMU)
  RTC_WriteBackupRegister(REBOOT_REASON_REGISTER_1, 0);
  RTC_WriteBackupRegister(REBOOT_REASON_STUCK_TASK_PC, 0);
  RTC_WriteBackupRegister(REBOOT_REASON_STUCK_TASK_LR, 0);
  RTC_WriteBackupRegister(REBOOT_REASON_STUCK_TASK_CALLBACK, 0);
#endif
}

uint32_t reboot_get_slot_of_last_launched_app(void) {
#ifdef CONFIG_SOC_NRF52
  return retained_read(SLOT_OF_LAST_LAUNCHED_APP);
#elif defined(CONFIG_SOC_SF32LB52)
  return HAL_Get_backup(SLOT_OF_LAST_LAUNCHED_APP);
#elif defined(CONFIG_QEMU)
  return RTC_ReadBackupRegister(SLOT_OF_LAST_LAUNCHED_APP);
#endif
}

void reboot_set_slot_of_last_launched_app(uint32_t app_slot) {
#ifdef CONFIG_SOC_NRF52
  retained_write(SLOT_OF_LAST_LAUNCHED_APP, app_slot);
#elif defined(CONFIG_SOC_SF32LB52)
  HAL_Set_backup(SLOT_OF_LAST_LAUNCHED_APP, app_slot);
#elif defined(CONFIG_QEMU)
  RTC_WriteBackupRegister(SLOT_OF_LAST_LAUNCHED_APP, app_slot);
#endif
}

