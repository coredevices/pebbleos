/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdlib.h>

#include "pbl/services/analytics/analytics.h"
#include "pbl/services/analytics/backend.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include "kernel/memory_layout.h"
#include "syscall/syscall_internal.h"
#include "system/reboot_reason.h"
#include "system/version.h"
#include "pbl/util/size.h"
#include "util/time/time.h"

#define HEARTBEAT_PERIOD_SEC 3600
#define ANALYTICS_STRING_MAX_LEN 64

extern void pbl_analytics_external_collect_battery(void);
extern void pbl_analytics_external_collect_cpu_stats(void);
extern void pbl_analytics_external_collect_task_cpu_stats(void);
extern void pbl_analytics_external_collect_stack_free(void);
extern void pbl_analytics_external_collect_pfs_stats(void);
extern void pbl_analytics_external_collect_kernel_heap_stats(void);
extern void pbl_analytics_external_collect_backlight_stats(void);
extern void pbl_analytics_external_collect_vibe_stats(void);
extern void pbl_analytics_external_collect_speaker_stats(void);
extern void pbl_analytics_external_collect_settings(void);

#if defined(ANALYTICS_NATIVE) || defined(ANALYTICS_MEMFAULT)

#ifdef ANALYTICS_NATIVE
extern void pbl_analytics__native_init(void);
extern void pbl_analytics__native_heartbeat(void);

extern const struct pbl_analytics_backend_ops pbl_analytics__native_ops;
#endif

#ifdef ANALYTICS_MEMFAULT
extern void pbl_analytics__memfault_init(void);
extern void pbl_analytics__memfault_heartbeat(void);

extern const struct pbl_analytics_backend_ops pbl_analytics__memfault_ops;
#endif

static TimerID s_heartbeat_timer;

static void (*const s_init[])(void) = {
#ifdef ANALYTICS_NATIVE
    pbl_analytics__native_init,
#endif
#ifdef ANALYTICS_MEMFAULT
    pbl_analytics__memfault_init,
#endif
};

static void (*const s_heartbeat[])(void) = {
#ifdef ANALYTICS_NATIVE
    pbl_analytics__native_heartbeat,
#endif
#ifdef ANALYTICS_MEMFAULT
    pbl_analytics__memfault_heartbeat,
#endif
};

static const struct pbl_analytics_backend_ops *s_backend_ops[] = {
#ifdef ANALYTICS_NATIVE
    &pbl_analytics__native_ops,
#endif
#ifdef ANALYTICS_MEMFAULT
    &pbl_analytics__memfault_ops,
#endif
};

static void prv_heartbeat_system_task_cb(void *data) {
  static bool s_reboot_reason_counted = false;

  PBL_ANALYTICS_SET_STRING(fw_version, TINTIN_METADATA.version_tag);
  PBL_ANALYTICS_SET_UNSIGNED(last_reboot_reason, reboot_reason_get_last_reboot_reason());
  PBL_ANALYTICS_SET_UNSIGNED(uptime_s, time_get_uptime_seconds());

  if (!s_reboot_reason_counted) {
    s_reboot_reason_counted = true;
    if (reboot_reason_get_last_reboot_reason() >= RebootReasonCode_Watchdog) {
      PBL_ANALYTICS_ADD(unexpected_reboot_count, 1);
    }
  }

  pbl_analytics_external_collect_battery();
  pbl_analytics_external_collect_cpu_stats();
  pbl_analytics_external_collect_task_cpu_stats();
  pbl_analytics_external_collect_stack_free();
  pbl_analytics_external_collect_pfs_stats();
  pbl_analytics_external_collect_kernel_heap_stats();
  pbl_analytics_external_collect_backlight_stats();
  pbl_analytics_external_collect_vibe_stats();
  pbl_analytics_external_collect_speaker_stats();
  pbl_analytics_external_collect_settings();

  for (size_t i = 0U; i < ARRAY_LENGTH(s_heartbeat); i++) {
    s_heartbeat[i]();
  }
}

static void prv_heartbeat_timer_cb(void *data) {
  system_task_add_callback(prv_heartbeat_system_task_cb, NULL);
}

void pbl_analytics_init(void) {
  for (size_t i = 0U; i < ARRAY_LENGTH(s_init); i++) {
    s_init[i]();
  }

  s_heartbeat_timer = new_timer_create();

  new_timer_start(s_heartbeat_timer, HEARTBEAT_PERIOD_SEC * 1000, prv_heartbeat_timer_cb, NULL,
                  TIMER_START_FLAG_REPEATING);
}

// Apps pass `key` straight through to backends that use it as an unchecked
// index into fixed-size kernel tables (`s_key_to_integer[]`, `s_pbl_to_memfault[]`,
// `s_string_ptrs[]`, `s_string_lens[]`, ...). An out-of-range key reads
// arbitrary kernel bytes; in the set_string path those bytes become a kernel
// pointer + length that strncpy() then writes the (validated) app string into,
// turning analytics into an arbitrary kernel write primitive. Bound the key
// here once and let the backends keep their direct indexing.
static bool prv_analytics_key_in_range(enum pbl_analytics_key key) {
  return ((unsigned)key < PBL_ANALYTICS_KEY_COUNT);
}

DEFINE_SYSCALL(void, sys_pbl_analytics_set_signed, enum pbl_analytics_key key,
               int32_t signed_value) {
  if (!prv_analytics_key_in_range(key)) {
    return;
  }
  for (size_t i = 0U; i < ARRAY_LENGTH(s_backend_ops); i++) {
    s_backend_ops[i]->set_signed(key, signed_value);
  }
}

DEFINE_SYSCALL(void, sys_pbl_analytics_set_unsigned, enum pbl_analytics_key key,
               uint32_t unsigned_value) {
  if (!prv_analytics_key_in_range(key)) {
    return;
  }
  for (size_t i = 0U; i < ARRAY_LENGTH(s_backend_ops); i++) {
    s_backend_ops[i]->set_unsigned(key, unsigned_value);
  }
}

DEFINE_SYSCALL(void, sys_pbl_analytics_set_string, enum pbl_analytics_key key,
               const char *value) {
  if (!prv_analytics_key_in_range(key)) {
    return;
  }
  if (PRIVILEGE_WAS_ELEVATED) {
    if (!memory_layout_is_cstring_in_region(
          memory_layout_get_app_region(), value, ANALYTICS_STRING_MAX_LEN)) {
      syscall_failed();
    }
  }

  for (size_t i = 0U; i < ARRAY_LENGTH(s_backend_ops); i++) {
    s_backend_ops[i]->set_string(key, value);
  }
}

DEFINE_SYSCALL(void, sys_pbl_analytics_timer_start, enum pbl_analytics_key key) {
  if (!prv_analytics_key_in_range(key)) {
    return;
  }
  for (size_t i = 0U; i < ARRAY_LENGTH(s_backend_ops); i++) {
    s_backend_ops[i]->timer_start(key);
  }
}

DEFINE_SYSCALL(void, sys_pbl_analytics_timer_stop, enum pbl_analytics_key key) {
  if (!prv_analytics_key_in_range(key)) {
    return;
  }
  for (size_t i = 0U; i < ARRAY_LENGTH(s_backend_ops); i++) {
    s_backend_ops[i]->timer_stop(key);
  }
}

DEFINE_SYSCALL(void, sys_pbl_analytics_add, enum pbl_analytics_key key, int32_t amount) {
  if (!prv_analytics_key_in_range(key)) {
    return;
  }
  for (size_t i = 0U; i < ARRAY_LENGTH(s_backend_ops); i++) {
    s_backend_ops[i]->add(key, amount);
  }
}

void command_analytics_heartbeat(void) {
  system_task_add_callback(prv_heartbeat_system_task_cb, NULL);
}

#else  // No analytics backend: provide no-op stubs.

void pbl_analytics_init(void) {}
DEFINE_SYSCALL(void, sys_pbl_analytics_set_signed, enum pbl_analytics_key key,
               int32_t signed_value) {}
DEFINE_SYSCALL(void, sys_pbl_analytics_set_unsigned, enum pbl_analytics_key key,
               uint32_t unsigned_value) {}
DEFINE_SYSCALL(void, sys_pbl_analytics_set_string, enum pbl_analytics_key key,
               const char *value) {}
DEFINE_SYSCALL(void, sys_pbl_analytics_timer_start, enum pbl_analytics_key key) {}
DEFINE_SYSCALL(void, sys_pbl_analytics_timer_stop, enum pbl_analytics_key key) {}
DEFINE_SYSCALL(void, sys_pbl_analytics_add, enum pbl_analytics_key key, int32_t amount) {}
void command_analytics_heartbeat(void) {}

#endif