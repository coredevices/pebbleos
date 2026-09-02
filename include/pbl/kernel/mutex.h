/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/types.h"

//! Recursive mutex. Not usable from ISRs.
struct pbl_mutex {
  struct pbl_thread *owner;
  uint32_t count;
  uintptr_t lock_lr;  // return address of the outermost lock, for diagnostics
  struct pbl_mutex_backend backend;
};

#define PBL_MUTEX_INITIALIZER { .owner = NULL, .count = 0, .lock_lr = 0 }

#define PBL_MUTEX_DEFINE(name)                   \
  struct pbl_mutex name = PBL_MUTEX_INITIALIZER; \
  PBL_KOBJ_REGISTER(name, pbl_mutex_kobj_init)

void pbl_mutex_init(struct pbl_mutex *m);
void pbl_mutex_kobj_init(void *m);

//! Required before the memory of a dynamically allocated mutex is reused.
void pbl_mutex_deinit(struct pbl_mutex *m);

//! @return 0, -EAGAIN on timeout, -EBUSY with PBL_NO_WAIT.
int pbl_mutex_lock_lr(struct pbl_mutex *m, pbl_timeout_t timeout, uintptr_t lr);

static inline int pbl_mutex_lock(struct pbl_mutex *m, pbl_timeout_t timeout) {
  return pbl_mutex_lock_lr(m, timeout, (uintptr_t)__builtin_return_address(0));
}

void pbl_mutex_unlock(struct pbl_mutex *m);

bool pbl_mutex_is_owner(const struct pbl_mutex *m);

//! Asserts that ownership by the calling thread matches @p held.
void pbl_mutex_assert_held(const struct pbl_mutex *m, bool held);
