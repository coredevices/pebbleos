/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

//! FreeRTOS keeps its own control blocks; objects only carry the handle.
struct pbl_thread_backend {
  void *handle;
  void (*entry)(void *);
  void *arg;
  bool dead;  // the FreeRTOS task is gone; its TCB may already be freed
};

struct pbl_mutex_backend {
  void *handle;
};

struct pbl_sem_backend {
  void *handle;
};

struct pbl_msgq_backend {
  void *handle;
};

struct pbl_poll_group_backend {
  void *handle;
};

#define PBL_SEM_BACKEND_INITIALIZER(initial) { .handle = NULL }

#define PBL_KERNEL_MSGQ_NEEDS_BUF 0
//! Objects need pbl_kernel_init() to create their FreeRTOS control blocks.
#define PBL_KERNEL_KOBJ_RUNTIME_INIT 1
