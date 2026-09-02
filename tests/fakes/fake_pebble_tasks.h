/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "stubs_worker_manager.h"

#include "kernel/pebble_tasks.h"


static PebbleTask s_current_task = PebbleTask_KernelMain;

PebbleTask pebble_task_get_current(void) {
  return s_current_task;
}

void stub_pebble_tasks_set_current(PebbleTask task) {
  s_current_task = task;
}

const char* pebble_task_get_name(PebbleTask task) {
  return "App <Stub>";
}

