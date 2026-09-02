/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/mutex.h"

#include "kernel.h"

void pbl_mutex_init(struct pbl_mutex *m) { *m = (struct pbl_mutex)PBL_MUTEX_INITIALIZER; }

void pbl_mutex_kobj_init(void *m) { pbl_mutex_init(m); }

void pbl_mutex_deinit(struct pbl_mutex *m) { KERNEL_ASSERT(m->count == 0); }

int pbl_mutex_lock_lr(struct pbl_mutex *m, pbl_timeout_t timeout, uintptr_t lr) {
  KERNEL_ASSERT(!arch_in_isr());
  int rc = 0;

  pbl_irq_lock();
  struct pbl_thread *me = pbl_cur;
  if (m->owner == NULL) {
    m->owner = me;
    m->count = 1;
    me->backend.mutexes_held++;
  } else if (m->owner == me) {
    m->count++;
  } else if (pbl_timeout_is_no_wait(timeout)) {
    rc = -EBUSY;
  } else {
    sched_inherit(m->owner, me->prio);
    // On success the previous owner handed the mutex over to us.
    rc = sched_block(&m->backend.waitq, timeout);
  }
#if CONFIG_KERNEL_MUTEX_LOCK_LR
  if (rc == 0 && m->count == 1) {
    m->lock_lr = lr;
  }
#else
  (void)lr;
#endif
  pbl_irq_unlock();
  return rc;
}

void pbl_mutex_unlock(struct pbl_mutex *m) {
  pbl_irq_lock();
  KERNEL_ASSERT(m->owner == pbl_cur && m->count > 0);
  if (--m->count == 0) {
    struct pbl_thread *prev = m->owner;
    prev->backend.mutexes_held--;
    m->lock_lr = 0;

    struct pbl_thread *next = waitq_pop(&m->backend.waitq);
    if (next) {
      m->owner = next;
      m->count = 1;
      next->backend.mutexes_held++;
      sched_wake(next, 0);
    } else {
      m->owner = NULL;
    }
    sched_disinherit(prev);
  }
  pbl_irq_unlock();
}

bool pbl_mutex_is_owner(const struct pbl_mutex *m) {
  return m->count > 0 && m->owner == pbl_cur;
}

void pbl_mutex_assert_held(const struct pbl_mutex *m, bool held) {
  KERNEL_ASSERT(pbl_mutex_is_owner(m) == held);
}
