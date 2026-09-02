/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/kernel/sem.h>

void pbl_sem_init(struct pbl_sem *s, uint32_t initial, uint32_t limit) {
}

void pbl_sem_kobj_init(void *s) {
}

void pbl_sem_deinit(struct pbl_sem *s) {
}

int pbl_sem_take(struct pbl_sem *s, pbl_timeout_t timeout) {
  return 0;
}

void pbl_sem_give(struct pbl_sem *s) {
}

void pbl_sem_reset(struct pbl_sem *s) {
}

uint32_t pbl_sem_count(const struct pbl_sem *s) {
  return 0;
}
