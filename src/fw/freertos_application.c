/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/memory_layout.h"
#include "kernel/pbl_malloc.h"
#include "process_management/worker_manager.h"
#include "syscall/syscall_internal.h"
#include "system/reboot_reason.h"
#include "system/reset.h"

#include <pbl/logging/logging.h>

#include "pbl/kernel/thread.h"

#include "FreeRTOS.h"
#include "task.h"

bool pbl_kernel_privilege_raise_allowed(uint32_t caller_pc) {
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
