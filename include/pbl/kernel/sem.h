/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/types.h"

//! Counting semaphore; a binary semaphore is limit 1. Usable from ISRs with PBL_NO_WAIT.
struct pbl_sem {
  uint32_t initial;
  uint32_t limit;
  struct pbl_sem_backend backend;
};

#define PBL_SEM_INITIALIZER(init, lim) \
  { .initial = (init), .limit = (lim), .backend = PBL_SEM_BACKEND_INITIALIZER(init) }

#define PBL_SEM_DEFINE(name, initial, limit) \
  struct pbl_sem name = PBL_SEM_INITIALIZER(initial, limit)

void pbl_sem_init(struct pbl_sem *s, uint32_t initial, uint32_t limit);

//! Required before the memory of a dynamically allocated semaphore is reused.
void pbl_sem_deinit(struct pbl_sem *s);

//! @return 0, -EAGAIN on timeout, -EBUSY with PBL_NO_WAIT.
int pbl_sem_take(struct pbl_sem *s, pbl_timeout_t timeout);
void pbl_sem_give(struct pbl_sem *s);
void pbl_sem_reset(struct pbl_sem *s);
uint32_t pbl_sem_count(const struct pbl_sem *s);
