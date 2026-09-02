/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "fake_mutex.h"

// Setup and Teardown
////////////////////////////////////

void test_fake_mutex__initialize(void) {
}

void test_fake_mutex__cleanup(void) {
  const bool assert_all_unlocked = false;
  fake_mutex_reset(assert_all_unlocked);
}

// Tests
////////////////////////////////////

void test_fake_mutex__normal_mutex(void) {
  struct pbl_mutex mutex;
  pbl_mutex_init(&mutex);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  pbl_mutex_unlock(&mutex);
  cl_assert_equal_b(fake_mutex_all_unlocked(), true);
}

void test_fake_mutex__leave_unlocked(void) {
  struct pbl_mutex mutex;
  pbl_mutex_init(&mutex);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  cl_assert_equal_b(fake_mutex_all_unlocked(), false);
}

void test_fake_mutex__double_unlock(void) {
  fake_mutex_set_should_assert(false);

  struct pbl_mutex mutex;
  pbl_mutex_init(&mutex);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  pbl_mutex_unlock(&mutex);
  pbl_mutex_unlock(&mutex);
  cl_assert_equal_b(true, fake_mutex_get_assert_triggered());
}

void test_fake_mutex__recursive(void) {
  struct pbl_mutex mutex;
  pbl_mutex_init(&mutex);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  pbl_mutex_unlock(&mutex);
  pbl_mutex_unlock(&mutex);
  pbl_mutex_unlock(&mutex);
  cl_assert_equal_b(fake_mutex_all_unlocked(), true);
}

void test_fake_mutex__recursive_mismatched_counts(void) {
  struct pbl_mutex mutex;
  pbl_mutex_init(&mutex);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  pbl_mutex_lock(&mutex, PBL_FOREVER);
  pbl_mutex_unlock(&mutex);
  cl_assert_equal_b(fake_mutex_all_unlocked(), false);
}

void test_fake_mutex__defined_without_init(void) {
  static PBL_MUTEX_DEFINE(s_mutex);
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  cl_assert_equal_b(pbl_mutex_is_owner(&s_mutex), true);
  pbl_mutex_unlock(&s_mutex);
  cl_assert_equal_b(fake_mutex_all_unlocked(), true);
}
