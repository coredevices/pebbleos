/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "passert.h"

#include "system/reboot_reason.h"
#include "kernel/fault_handling.h"

#include "kernel/pebble_tasks.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "pbl/util/heap.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#define CORE_NUMBER 0

static NORETURN handle_passert_failed_vargs(const char* filename, int line_number,
    uintptr_t lr, const char* expr, const char* fmt, va_list fmt_args) {
  char buffer[160];

  pbl_log_sync(LOG_LEVEL_ALWAYS, filename, line_number, "*** ASSERTION FAILED: %s", expr);

  if (fmt) {
    vsniprintf(buffer, sizeof(buffer), fmt, fmt_args);
    pbl_log_sync(LOG_LEVEL_ALWAYS, filename, line_number, "%s", buffer);
  }

  trigger_fault(RebootReasonCode_Assert, lr);
}

static NORETURN handle_passert_failed(const char* filename, int line_number,
    uintptr_t lr, const char *expr, const char* fmt, ...) {
  va_list fmt_args;
  va_start(fmt_args, fmt);

  handle_passert_failed_vargs(filename, line_number, lr, expr, fmt, fmt_args);

  va_end(fmt_args);
}

NORETURN passert_failed(const char* filename, int line_number, const char* message, ...) {
  va_list fmt_args;
  va_start(fmt_args, message);

  handle_passert_failed_vargs(filename, line_number,
    (uintptr_t)__builtin_return_address(0), "ASSERT", message, fmt_args);

  va_end(fmt_args);
}

NORETURN passert_failed_hashed(uint32_t packed_loghash, ...) {
  uintptr_t saved_lr = (uintptr_t) __builtin_return_address(0);
  PBL_LOG_ALWAYS("ASSERTION at LR 0x%x", saved_lr);

  va_list fmt_args;
  va_start(fmt_args, packed_loghash);

  pbl_log_hashed_vargs(false, CORE_NUMBER, packed_loghash, fmt_args);

  va_end(fmt_args);

  trigger_fault(RebootReasonCode_Assert, saved_lr);
}

NORETURN passert_failed_hashed_with_lr(uint32_t lr, uint32_t packed_loghash, ...) {
  PBL_LOG_ALWAYS("ASSERTION at LR 0x%"PRIx32, lr);

  va_list fmt_args;
  va_start(fmt_args, packed_loghash);

  pbl_log_hashed_vargs(false, CORE_NUMBER, packed_loghash, fmt_args);

  va_end(fmt_args);

  trigger_fault(RebootReasonCode_Assert, lr);
}

NORETURN passert_failed_hashed_no_message_with_lr(uint32_t lr) {
  PBL_LOG_ALWAYS("ASSERTION at LR 0x%"PRIx32, lr);

  trigger_fault(RebootReasonCode_Assert, lr);
}

NORETURN passert_failed_hashed_no_message(void) {
  passert_failed_hashed_no_message_with_lr((uint32_t)__builtin_return_address(0));
}

NORETURN passert_failed_no_message_with_lr(const char* filename, int line_number, uint32_t lr) {
  handle_passert_failed(filename, line_number, lr, "ASSERTN", NULL);
}

NORETURN passert_failed_no_message(const char* filename, int line_number) {
  handle_passert_failed(filename, line_number,
    (uintptr_t)__builtin_return_address(0), "ASSERTN", NULL);
}

NORETURN wtf(void) {
  uintptr_t saved_lr = (uintptr_t) __builtin_return_address(0);
  PBL_LOG_ALWAYS("*** WTF %p", (void *)saved_lr);
  trigger_fault(RebootReasonCode_Assert, saved_lr);
}

void passert_check_task(PebbleTask expected_task) {
  uintptr_t saved_lr = (uintptr_t) __builtin_return_address(0);

  if (pebble_task_get_current() != expected_task) {
    PBL_LOG_ALWAYS("LR: %p. Incorrect task! Expected <%s> got <%s>",
            (void*) saved_lr, pebble_task_get_name(expected_task),
            pebble_task_get_name(pebble_task_get_current()));
    trigger_fault(RebootReasonCode_Assert, saved_lr);
  }
}

void passert_check_not_task(PebbleTask unexpected_task) {
  uintptr_t saved_lr = (uintptr_t) __builtin_return_address(0);

  if (pebble_task_get_current() == unexpected_task) {
    PBL_LOG_ALWAYS("LR: %p. Incorrect task! Can't be <%s>",
            (void*) saved_lr, pebble_task_get_name(unexpected_task));
    trigger_fault(RebootReasonCode_Assert, saved_lr);
  }
}

//! Assert function called by the STM peripheral library's
//! 'assert_param' method. See stm32f2xx_conf.h for more information.
void assert_failed(uint8_t* file, uint32_t line) {
  register uintptr_t lr __asm("lr");
  uintptr_t saved_lr = lr;

  handle_passert_failed((const char*) file, line, saved_lr, "STM32", "STM32 peripheral library tripped an assert");
}

extern void command_dump_malloc_kernel(void);

NORETURN croak_oom(size_t bytes, int saved_lr, Heap *heap_ptr) {
  unsigned int used = 0, free_bytes = 0, max_free = 0;
  if (heap_ptr) {
    heap_calc_totals(heap_ptr, &used, &free_bytes, &max_free);
  }
  PBL_LOG_ALWAYS("CROAK OOM: Failed to alloc %d bytes at LR: 0x%x (used %u, free %u, max_free %u)",
                 bytes, saved_lr, used, free_bytes, max_free);

#ifdef CONFIG_MALLOC_INSTRUMENTATION
  command_dump_malloc_kernel();
#endif

  trigger_oom_fault(bytes, saved_lr, heap_ptr);
}

#ifdef CONFIG_SOC_NRF52
NORETURN app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
  PBL_LOG_ALWAYS("nRF error %ld (pc %ld, info %ld)", id, pc, info);
  trigger_fault(RebootReasonCode_Assert, pc);
}

NORETURN app_error_handler_bare(uint32_t error_code) {
  app_error_fault_handler(error_code, (uint32_t)__builtin_return_address(0), 0);
}
#endif
