/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "kernel.h"

#define STACK_FILL 0xa5a5a5a5u

static uint32_t s_next_id;

int pbl_thread_create(struct pbl_thread *t, const struct pbl_thread_attr *attr) {
  KERNEL_ASSERT(attr->stack != NULL && attr->stack_size >= 128);
  KERNEL_ASSERT(attr->prio <= PBL_PRIO_MAX);

  // A struct is only reusable once its previous thread is gone.
  KERNEL_ASSERT(t->id == 0 || t->backend.state == PBL_THREAD_DEAD);

  memset(t, 0, sizeof(*t));
  t->id = ++s_next_id;
  strncpy(t->name, attr->name, sizeof(t->name) - 1);
  t->prio = attr->prio;
  t->privileged = attr->privileged;
  t->stack = attr->stack;
  t->stack_size = attr->stack_size;

  uint32_t *words = attr->stack;
  for (size_t i = 0; i < attr->stack_size / sizeof(uint32_t); i++) {
    words[i] = STACK_FILL;
  }

  arch_thread_init(t, attr->entry, attr->arg);
  arch_thread_regions_set(t, attr->regions);

  pbl_irq_lock();
  sched_thread_start(t);
  pbl_irq_unlock();
  return 0;
}

void pbl_thread_abort(struct pbl_thread *t) {
  pbl_irq_lock();
  bool self = (t == NULL || t == pbl_cur);
  sched_thread_remove(self ? pbl_cur : t);
  if (!self) {
    arch_thread_aborted(t);
  }
  pbl_irq_unlock();
  if (self) {
    // The switch requested above takes over; this never returns.
    arch_thread_exit();
  }
}

void pbl_thread_suspend(struct pbl_thread *t) {
  pbl_irq_lock();
  sched_thread_suspend(t ? t : pbl_cur);
  pbl_irq_unlock();
}

void pbl_thread_resume(struct pbl_thread *t) {
  pbl_irq_lock();
  sched_thread_resume(t);
  pbl_irq_unlock();
}

struct pbl_thread *pbl_thread_current(void) { return pbl_cur; }

struct pbl_thread *pbl_thread_idle(void) { return sched_idle_thread(); }

void pbl_thread_prio_set(struct pbl_thread *t, pbl_prio_t prio) {
  KERNEL_ASSERT(prio <= PBL_PRIO_MAX);
  pbl_irq_lock();
  // Keep an inherited boost; only the base moves.
  pbl_prio_t effective = t->prio > t->backend.base_prio ? (prio > t->prio ? prio : t->prio) : prio;
  sched_prio_set(t, prio, effective);
  pbl_irq_unlock();
}

pbl_prio_t pbl_thread_prio_get(const struct pbl_thread *t) { return t->backend.base_prio; }

enum pbl_thread_state pbl_thread_state(const struct pbl_thread *t) {
  return (enum pbl_thread_state)t->backend.state;
}

void pbl_thread_regions_set(struct pbl_thread *t, const MpuRegion *const *regions) {
  pbl_irq_lock();
  arch_thread_regions_set(t, regions);
  pbl_irq_unlock();
}
