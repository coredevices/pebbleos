/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/thread.h"

#include "kernel_freertos.h"

// Lets debuggers with FreeRTOS awareness find the ready lists.
const int __attribute__((used)) uxTopUsedPriority = configMAX_PRIORITIES - 1;

// Boards without a dedicated syscall stack run syscalls on the caller's.
__attribute__((weak)) uint32_t *pbl_kernel_syscall_stack(uintptr_t *base_out) {
  (void)base_out;
  return NULL;
}

uint32_t *xApplicationGetSyscallStack(uintptr_t *base_out) {
  return pbl_kernel_syscall_stack(base_out);
}

void vSetupSyscallRegisters(uintptr_t orig_sp, uintptr_t *lr_ptr) {
  pbl_kernel_syscall_entered(orig_sp, lr_ptr);
}

bool xApplicationIsAllowedToRaisePrivilege(uint32_t caller_pc) {
  return pbl_kernel_privilege_raise_allowed(caller_pc);
}

void vApplicationStackOverflowHook(TaskHandle_t task_handle, signed char *name) {
  pbl_thread_stack_overflow(kfr_thread_from_handle(task_handle), (const char *)name);
}
