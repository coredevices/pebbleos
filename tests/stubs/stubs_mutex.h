/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/kernel/mutex.h>

void pbl_mutex_init(struct pbl_mutex *m) {
}

void pbl_mutex_deinit(struct pbl_mutex *m) {
}

int pbl_mutex_lock_lr(struct pbl_mutex *m, pbl_timeout_t timeout, uintptr_t lr) {
  return 0;
}

void pbl_mutex_unlock(struct pbl_mutex *m) {
}

bool pbl_mutex_is_owner(const struct pbl_mutex *m) {
  return true;
}

void pbl_mutex_assert_held(const struct pbl_mutex *m, bool held) {
}
