/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include "pbl/kernel/irq.h"
#include "pbl/kernel/mutex.h"

#include "kernel_freertos.h"
#include "light_mutex.h"

void pbl_mutex_init(struct pbl_mutex *m) {
  *m = (struct pbl_mutex)PBL_MUTEX_INITIALIZER;
  m->backend.handle = xLightMutexCreate();
  OS_ASSERT(m->backend.handle != NULL);
}

void pbl_mutex_kobj_init(void *m) { pbl_mutex_init(m); }

void pbl_mutex_deinit(struct pbl_mutex *m) {
  OS_ASSERT(m->count == 0);
  vLightMutexDelete(m->backend.handle);
  m->backend.handle = NULL;
}

int pbl_mutex_lock_lr(struct pbl_mutex *m, pbl_timeout_t timeout, uintptr_t lr) {
  OS_ASSERT_LR(!pbl_in_isr(), lr);

  if (xLightMutexLockRecursive(m->backend.handle, kfr_ticks(timeout)) != pdTRUE) {
    return kfr_wait_error(timeout);
  }
  if (m->count++ == 0) {
    m->owner = pbl_thread_current();
#if CONFIG_KERNEL_MUTEX_LOCK_LR
    m->lock_lr = lr;
#endif
  }
  return 0;
}

static bool prv_held_by_current(const struct pbl_mutex *m) {
  return xLightMutexGetHolder(m->backend.handle) == xTaskGetCurrentTaskHandle();
}

void pbl_mutex_unlock(struct pbl_mutex *m) {
  OS_ASSERT(m->count > 0 && prv_held_by_current(m));
  if (--m->count == 0) {
    m->owner = NULL;
    m->lock_lr = 0;
  }
  xLightMutexUnlockRecursive(m->backend.handle);
}

bool pbl_mutex_is_owner(const struct pbl_mutex *m) {
  return m->count > 0 && prv_held_by_current(m);
}

void pbl_mutex_assert_held(const struct pbl_mutex *m, bool held) {
  OS_ASSERT(pbl_mutex_is_owner(m) == held);
}
