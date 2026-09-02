/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/list.h"

#include <pbl/kernel/sem.h>

#include <errno.h>
#include <stdlib.h>

//! Counting semaphore fake. A yield callback stands in for the other task:
//! it runs while a take would block and returns the ticks it consumed.
typedef pbl_tick_t (*FakeSemYieldCallback)(struct pbl_sem *sem);

typedef struct FakeSem {
  ListNode node;
  struct pbl_sem *sem;
  uint32_t count;
  uint32_t limit;
  FakeSemYieldCallback yield_cb;
} FakeSem;

static FakeSem *s_fake_sem_list;

static bool prv_fake_sem_find(ListNode *node, void *context) {
  return ((FakeSem *)node)->sem == context;
}

static FakeSem *prv_fake_sem_get(struct pbl_sem *s) {
  FakeSem *fake = (FakeSem *)list_find((ListNode *)s_fake_sem_list, prv_fake_sem_find, s);
  if (fake == NULL) {
    fake = malloc(sizeof(FakeSem));
    *fake = (FakeSem) { .sem = s, .count = s->initial, .limit = s->limit ? s->limit : 1 };
    s_fake_sem_list = (FakeSem *)list_prepend((ListNode *)s_fake_sem_list, (ListNode *)fake);
  }
  return fake;
}

void fake_sem_reset(void) {
  ListNode *iter = (ListNode *)s_fake_sem_list;
  while (iter) {
    ListNode *next = iter->next;
    free(iter);
    iter = next;
  }
  s_fake_sem_list = NULL;
}

void fake_sem_set_yield_callback(struct pbl_sem *s, FakeSemYieldCallback yield_cb) {
  prv_fake_sem_get(s)->yield_cb = yield_cb;
}

void pbl_sem_init(struct pbl_sem *s, uint32_t initial, uint32_t limit) {
  *s = (struct pbl_sem)PBL_SEM_INITIALIZER(initial, limit);
  FakeSem *fake = prv_fake_sem_get(s);
  fake->count = initial;
  fake->limit = limit;
}

void pbl_sem_kobj_init(void *s) {
  struct pbl_sem *sem = s;
  pbl_sem_init(sem, sem->initial, sem->limit);
}

void pbl_sem_deinit(struct pbl_sem *s) {
  FakeSem *fake = prv_fake_sem_get(s);
  list_remove((ListNode *)fake, (ListNode **)&s_fake_sem_list, NULL);
  free(fake);
}

int pbl_sem_take(struct pbl_sem *s, pbl_timeout_t timeout) {
  FakeSem *fake = prv_fake_sem_get(s);
  pbl_tick_t waited = 0;
  while (true) {
    if (fake->count > 0) {
      fake->count--;
      return 0;
    }
    if (pbl_timeout_is_no_wait(timeout) || !fake->yield_cb) {
      return pbl_timeout_is_no_wait(timeout) ? -EBUSY : -EAGAIN;
    }
    waited += fake->yield_cb(s);
    if (!pbl_timeout_is_forever(timeout) && waited >= timeout.ticks) {
      return -EAGAIN;
    }
  }
}

void pbl_sem_give(struct pbl_sem *s) {
  FakeSem *fake = prv_fake_sem_get(s);
  if (fake->count < fake->limit) {
    fake->count++;
  }
}

void pbl_sem_reset(struct pbl_sem *s) {
  prv_fake_sem_get(s)->count = s->initial;
}

uint32_t pbl_sem_count(const struct pbl_sem *s) {
  return prv_fake_sem_get((struct pbl_sem *)s)->count;
}
