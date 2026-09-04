/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pthread.h>
#include <stdbool.h>

//! Each kernel thread is a pthread that runs only while the kernel says so.
struct pbl_arch_thread {
  pthread_t tid;
  pthread_cond_t wake;
  void (*entry)(void *);
  void *arg;
  bool created;
  bool run;
};
