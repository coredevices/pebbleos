/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "resource.h"

#include "process_management/app_manager.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>

//! @file resource_syscalls.c
//! The landing place for untrusted code to use resources.

DEFINE_SYSCALL(size_t, sys_resource_size, ResAppNum app_num, uint32_t resource_id) {
  if (PRIVILEGE_WAS_ELEVATED) {
    if (pebble_task_get_current() == PebbleTask_Worker) {
      // Not allowed from workers
      syscall_failed();
    }
  }

  return resource_size(app_num, resource_id);
}

DEFINE_SYSCALL(size_t, sys_resource_load_range, ResAppNum app_num,
               uint32_t id, uint32_t start_offset, uint8_t *data, size_t num_bytes) {

  if (PRIVILEGE_WAS_ELEVATED) {
    if (pebble_task_get_current() == PebbleTask_Worker) {
      // Not allowed from workers
      syscall_failed();
    }

    syscall_assert_userspace_buffer(data, num_bytes);
  }

  return resource_load_byte_range_system(app_num, id, start_offset, data, num_bytes);
}

DEFINE_SYSCALL(bool, sys_resource_bytes_are_readonly, void *ptr) {
  return resource_bytes_are_readonly(ptr);
}

DEFINE_SYSCALL(const uint8_t *, sys_resource_read_only_bytes, ResAppNum app_num, uint32_t
               resource_id, size_t *num_bytes_out) {
  bool caller_is_privileged = true;
  if (PRIVILEGE_WAS_ELEVATED) {
    caller_is_privileged = false;
    if (pebble_task_get_current() == PebbleTask_Worker) {
      // Not allowed from workers
      syscall_failed();
    }

    // num_bytes_out is optional, so it's perfectly safe for an app to pass in NULL here.
    if (num_bytes_out) {
      syscall_assert_userspace_buffer(num_bytes_out, sizeof(*num_bytes_out));
    }
  }

  return resource_get_readonly_bytes(app_num, resource_id, num_bytes_out, caller_is_privileged);
}

DEFINE_SYSCALL(bool, sys_resource_is_valid, ResAppNum app_num, uint32_t resource_id) {
  return resource_is_valid(app_num, resource_id);
}

DEFINE_SYSCALL(uint32_t, sys_resource_get_and_cache, ResAppNum app_num, uint32_t resource_id) {
  return resource_get_and_cache(app_num, resource_id);
}
