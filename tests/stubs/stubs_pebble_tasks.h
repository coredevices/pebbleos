/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/pebble_tasks.h"

PebbleTask pebble_task_get_current(void) {
  return 0;
}

struct pbl_thread *pebble_task_get_thread(PebbleTask task) {
  return NULL;
}

const char* pebble_task_get_name(PebbleTask task) {
  return NULL;
}

void pebble_task_unregister(PebbleTask task) {
}

struct pbl_thread *pebble_task_create(PebbleTask pebble_task, struct pbl_thread_attr *attr) {
  return NULL;
}
