/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "system/passert.h"
#include "pbl/util/list.h"

#include <pbl/kernel/mutex.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct FakePebbleMutex {
  ListNode node;
  struct pbl_mutex *mutex;
  uint32_t lock_count;
} FakePebbleMutex;

static FakePebbleMutex *s_mutex_list;
static bool s_asserts_disabled;
static bool s_assert_triggered;

//
// Helpers
//

static bool prv_find_by_mutex(ListNode *node, void *context) {
  return ((FakePebbleMutex *)node)->mutex == context;
}

// Mutexes defined with PBL_MUTEX_DEFINE are never initialised under test, so
// any mutex is registered on first use.
static FakePebbleMutex *prv_get(struct pbl_mutex *m) {
  FakePebbleMutex *fake =
      (FakePebbleMutex *)list_find((ListNode *)s_mutex_list, prv_find_by_mutex, m);
  if (fake == NULL) {
    fake = malloc(sizeof(FakePebbleMutex));
    *fake = (FakePebbleMutex) { .mutex = m };
    s_mutex_list = (FakePebbleMutex *)list_prepend((ListNode *)s_mutex_list, (ListNode *)fake);
  }
  return fake;
}

static bool prv_list_foreach_assert_unlocked(ListNode *node, void *context) {
  FakePebbleMutex *fake = (FakePebbleMutex *)node;
  bool *failed = context;
  if (fake->lock_count != 0) {
    // If this is failing, set your breakpoint here to find out which mutex
    printf("Mutex (%p) was not unlocked when fake_mutex_assert_all_unlocked called\n",
           fake->mutex);
    s_assert_triggered = true;
    cl_assert(s_asserts_disabled);
    *failed = true;
  }
  return true;
}

//
// Fake Mutex API
//

void fake_mutex_reset(bool assert_all_unlocked) {
  ListNode *iter = (ListNode *)s_mutex_list;
  while (iter) {
    ListNode *next = iter->next;
    if (assert_all_unlocked) {
      FakePebbleMutex *fake = (FakePebbleMutex *)iter;
      cl_assert_equal_i(0, fake->lock_count);
    }
    free(iter);
    iter = next;
  }
  s_mutex_list = NULL;
  s_asserts_disabled = false;
  s_assert_triggered = false;
}

void fake_mutex_assert_all_unlocked(void) {
  bool failed;
  list_foreach((ListNode *)s_mutex_list, prv_list_foreach_assert_unlocked, &failed);
}

bool fake_mutex_all_unlocked(void) {
  ListNode *iter = (ListNode *)s_mutex_list;
  while (iter) {
    if (((FakePebbleMutex *)iter)->lock_count != 0) {
      return false;
    }
    iter = iter->next;
  }
  return true;
}

void fake_mutex_set_should_assert(bool should_assert) {
  s_asserts_disabled = !should_assert;
}

bool fake_mutex_get_assert_triggered(void) {
  return s_assert_triggered;
}

//
// Mutex API
//

void pbl_mutex_init(struct pbl_mutex *m) {
  *m = (struct pbl_mutex)PBL_MUTEX_INITIALIZER;
  prv_get(m)->lock_count = 0;
}

void pbl_mutex_deinit(struct pbl_mutex *m) {
  FakePebbleMutex *fake = prv_get(m);
  list_remove((ListNode *)fake, (ListNode **)&s_mutex_list, NULL);
  free(fake);
}

int pbl_mutex_lock_lr(struct pbl_mutex *m, pbl_timeout_t timeout, uintptr_t lr) {
  FakePebbleMutex *fake = prv_get(m);
  fake->lock_count++;
  m->count = fake->lock_count;
  return 0;
}

void pbl_mutex_unlock(struct pbl_mutex *m) {
  FakePebbleMutex *fake = prv_get(m);
  if (fake->lock_count == 0) {
    s_assert_triggered = true;
    cl_assert_(s_asserts_disabled, "pbl_mutex_unlock called with mutex that was not locked");
    return;
  }
  fake->lock_count--;
  m->count = fake->lock_count;
}

bool pbl_mutex_is_owner(const struct pbl_mutex *m) {
  return prv_get((struct pbl_mutex *)m)->lock_count > 0;
}

void pbl_mutex_assert_held(const struct pbl_mutex *m, bool held) {
  cl_assert_equal_b(pbl_mutex_is_owner(m), held);
}
