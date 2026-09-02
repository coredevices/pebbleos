/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/sem.h"

#include "kernel.h"

void pbl_sem_init(struct pbl_sem *s, uint32_t initial, uint32_t limit) {
  KERNEL_ASSERT(limit > 0 && initial <= limit);
  *s = (struct pbl_sem)PBL_SEM_INITIALIZER(initial, limit);
  s->backend.count = initial;
}

void pbl_sem_kobj_init(void *s) {
  struct pbl_sem *sem = s;
  pbl_sem_init(sem, sem->initial, sem->limit);
}

void pbl_sem_deinit(struct pbl_sem *s) { (void)s; }

int pbl_sem_take(struct pbl_sem *s, pbl_timeout_t timeout) {
  int rc = 0;
  pbl_irq_lock();
  if (s->backend.count > 0) {
    s->backend.count--;
  } else if (pbl_timeout_is_no_wait(timeout) || arch_in_isr()) {
    rc = -EBUSY;
  } else {
    // A give hands the token straight to the woken thread.
    rc = sched_block(&s->backend.waitq, timeout);
  }
  pbl_irq_unlock();
  return rc;
}

void pbl_sem_give(struct pbl_sem *s) {
  pbl_irq_lock();
  struct pbl_thread *t = waitq_pop(&s->backend.waitq);
  if (t) {
    sched_wake(t, 0);
  } else if (s->backend.count < s->limit) {
    s->backend.count++;
  }
  pbl_irq_unlock();
}

void pbl_sem_reset(struct pbl_sem *s) {
  pbl_irq_lock();
  s->backend.count = s->initial;
  pbl_irq_unlock();
}

uint32_t pbl_sem_count(const struct pbl_sem *s) { return s->backend.count; }
