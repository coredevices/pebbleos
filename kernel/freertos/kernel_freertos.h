/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <errno.h>

#include "pbl/kernel/thread.h"
#include "pbl/kernel/types.h"
#include "pbl/os/assert.h"

#include "FreeRTOS.h"
#include "task.h"

//! TLS slot holding the struct pbl_thread back-pointer of a FreeRTOS task.
#define KFR_TLS_THREAD (configNUM_THREAD_LOCAL_STORAGE_POINTERS - 1)

_Static_assert(configMAX_PRIORITIES == CONFIG_KERNEL_NUM_PRIORITIES,
               "FreeRTOSConfig.h and CONFIG_KERNEL_NUM_PRIORITIES disagree");

static inline TickType_t kfr_ticks(pbl_timeout_t t) {
  return pbl_timeout_is_forever(t) ? portMAX_DELAY : (TickType_t)t.ticks;
}

static inline int kfr_wait_error(pbl_timeout_t t) {
  return pbl_timeout_is_no_wait(t) ? -EBUSY : -EAGAIN;
}

static inline TaskHandle_t kfr_handle(const struct pbl_thread *t) {
  return t ? (TaskHandle_t)t->backend.handle : NULL;
}

struct pbl_thread *kfr_thread_from_handle(TaskHandle_t h);
