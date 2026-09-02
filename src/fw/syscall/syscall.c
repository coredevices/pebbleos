/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/thread.h"
#include "time.h"

#include "syscall.h"

#include "syscall_internal.h"

#include <pbl/drivers/rtc.h>
#include "pbl/mcu/privilege.h"
#include "pbl/kernel/types.h"
#include "process_management/worker_manager.h"
#include "logging/logging_private.h"
#include <pbl/logging/logging.h>
#include "pbl/util/string.h"

DEFINE_SYSCALL(int, sys_test, int arg) {
  uint32_t ipsr;
  __asm volatile("mrs %0, ipsr" : "=r" (ipsr));

  PBL_LOG_DBG("Inside test kernel function! Privileged? %s Arg %u IPSR: %"PRIu32,
          bool_to_str(mcu_state_is_privileged()), arg, ipsr);

  return arg * 2;
}

DEFINE_SYSCALL(time_t, sys_get_time, void) {
  return rtc_get_time();
}

DEFINE_SYSCALL(void, sys_get_time_ms, time_t *t, uint16_t *out_ms) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(t, sizeof(*t));
    syscall_assert_userspace_buffer(out_ms, sizeof(*out_ms));
  }

  rtc_get_time_ms(t, out_ms);
}

DEFINE_SYSCALL(RtcTicks, sys_get_ticks, void) {
  return rtc_get_ticks();
}

DEFINE_SYSCALL(void, sys_pbl_log, LogBinaryMessage* log_message, bool async) {
  // log_message points at a struct whose trailing message[] is sized by the
  // embedded message_length byte. Without a check, an app can hand us any
  // kernel address: log_level and message_length steer the formatting code,
  // and kernel_pbl_log_flash() ends up writing `sizeof(*log_message) +
  // message_length` bytes straight from that address to the flash log channel
  // — a generic kernel-memory exfiltration primitive.
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(log_message, sizeof(*log_message));
    syscall_assert_userspace_buffer(log_message,
                                    sizeof(*log_message) + log_message->message_length);
  }
  kernel_pbl_log(log_message, async);
}

DEFINE_SYSCALL(void, sys_copy_timezone_abbr, char* timezone_abbr, time_t time) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(timezone_abbr, TZ_LEN);
  }
  time_get_timezone_abbr(timezone_abbr, time);
}

DEFINE_SYSCALL(struct tm*, sys_gmtime_r, const time_t *timep, struct tm *result) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(timep, sizeof(*timep));
    syscall_assert_userspace_buffer(result, sizeof(*result));
  }
  return gmtime_r(timep, result);
}

DEFINE_SYSCALL(struct tm*, sys_localtime_r, const time_t *timep, struct tm *result) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(timep, sizeof(*timep));
    syscall_assert_userspace_buffer(result, sizeof(*result));
  }
  return localtime_r(timep, result);
}

//! System call to exit an application gracefully.
DEFINE_SYSCALL(NORETURN, sys_exit, void) {
  process_manager_task_exit();
}

DEFINE_SYSCALL(void, sys_psleep, int millis) {
  pbl_thread_sleep(PBL_MSEC(millis));
}
