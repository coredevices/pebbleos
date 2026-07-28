/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <unistd.h>

#include "pbl/services/system_task.h"

bool system_task_add_callback(SystemTaskEventCallback cb, void *data) {
  cb(data);
  return true;
}

bool system_task_add_callback_nonblocking(SystemTaskEventCallback cb, void *data) {
  cb(data);
  return true;
}

uint32_t system_task_get_available_space(void) {
  return 0;
}
