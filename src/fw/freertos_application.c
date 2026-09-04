/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/memory_layout.h"
#include "kernel/pbl_malloc.h"
#include "process_management/worker_manager.h"
#include "syscall/syscall_internal.h"

#include "FreeRTOS.h"
#include "task.h"

void vApplicationStackOverflowHook(TaskHandle_t task_handle, signed char *name) {
  PebbleTask task = pebble_task_get_task_for_handle(task_handle);

  // If the task is application or worker, ignore this hook. We have a memory protection region
  // setup at the bottom of those stacks and the code that catches MPU violiations to that
  // area in fault_handling.c has the logic to safely kill those user tasks without forcing
  // a reboot.
  if ((task != PebbleTask_App) && (task != PebbleTask_Worker)) {
    PBL_LOG_SYNC_ERR("Stack overflow [task: %s]", name);
    RebootReason reason = {
      .code = RebootReasonCode_StackOverflow,
      .data8[0] = task
    };
    reboot_reason_set(&reason);

    reset_due_to_software_failure();
  }
}

bool xApplicationIsAllowedToRaisePrivilege(uint32_t caller_pc) {
  // This function is called by portSVCHandler with the PC value of the
  // function which initiated the SVC call requesting privilege elevation.

  // The memory_region.c functions are not used for this check as this function
  // is in a hot code-path and needs to execute as quickly as possible.

  // mcu_call_unprivileged() has one internal re-entry SVC. It is not part of
  // the generated syscall island; accept it only while the current task is
  // returning from mcu_call_unprivileged(). The setup path rechecks the PC
  // before rewriting the exception frame.
  if (mcu_call_unprivileged_reentry_is_allowed(caller_pc)) {
    return true;
  }

  // All syscall functions are lumped together in one place in the firmware
  // image to reduce the attack surface. Don't allow privilege to be raised by
  // any code outside of that region, even if that code is in flash.
  // See WHT-114 and PBL-34044.
  extern const uint32_t __syscall_text_start__[];
  extern const uint32_t __syscall_text_end__[];
  const uint32_t priv_code_start = (uint32_t) __syscall_text_start__;
  const uint32_t priv_code_end = (uint32_t) __syscall_text_end__;
  return (caller_pc >= priv_code_start && caller_pc < priv_code_end);
}

#undef vPortFree
void vPortFree(void* pv) {
  kernel_free(pv);
}

#undef pvPortMalloc
void* pvPortMalloc(size_t xSize) {
  return kernel_malloc(xSize);
}