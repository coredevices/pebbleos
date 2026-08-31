/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/pbl_malloc.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"

void os_log(const char *filename, int line, const char *string) {
  pbl_log(LOG_LEVEL_INFO, filename, line, string);
}

NORETURN os_assertion_failed(const char *filename, int line) {
  passert_failed_no_message(filename, line);
}

NORETURN os_assertion_failed_lr(const char *filename, int line, uint32_t lr) {
  passert_failed_no_message_with_lr(filename, line, lr);
}

void *os_malloc(size_t size) {
  return kernel_malloc(size);
}

void *os_malloc_check(size_t size) {
  return kernel_malloc_check(size);
}

void os_free(void *ptr) {
  kernel_free(ptr);
}
