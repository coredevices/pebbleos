/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pbl_arch_thread.h"

struct pbl_thread;
struct pbl_msgq;
struct pbl_poll_group;

//! Threads blocked on an object, highest priority first, FIFO within a priority.
struct pbl_waitq {
  struct pbl_thread *head;
};

struct pbl_thread_backend {
  void *sp;  // saved stack pointer; must stay first
  struct pbl_arch_thread arch;
  struct pbl_thread *next;          // ready list or wait queue link
  struct pbl_thread *timeout_next;  // timeout list link
  struct pbl_thread *all_next;      // list of every live thread
  struct pbl_waitq *waitq;          // object blocked on, NULL for a plain sleep
  uint32_t wake_at;
  uint32_t run_time;
  uint32_t switched_in_at;
  uint32_t number;
  uint8_t state;
  uint8_t base_prio;
  uint8_t mutexes_held;
  bool on_timeout_list;
  int wake_rc;
};

struct pbl_mutex_backend {
  struct pbl_waitq waitq;
};

struct pbl_sem_backend {
  struct pbl_waitq waitq;
  uint32_t count;
};

struct pbl_msgq_backend {
  struct pbl_waitq getters;
  struct pbl_waitq putters;
  uint32_t head;
  uint32_t tail;
  uint32_t count;
};

struct pbl_poll_group_backend {
  struct pbl_waitq waitq;
  struct pbl_msgq *cursor;
};

#define PBL_SEM_BACKEND_INITIALIZER(initial) { .count = (initial) }

#define PBL_KERNEL_MSGQ_NEEDS_BUF 1
//! PBL_*_DEFINE initialisers are complete; pbl_kernel_init() has nothing to do.
#define PBL_KERNEL_KOBJ_RUNTIME_INIT 0
