/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/mcu/interrupts.h"
#include "pbl/os/assert.h"
#include "pbl/os/malloc.h"
#include "pbl/os/mutex.h"
#include "pbl/os/tick.h"

#include "FreeRTOS.h"
#include "light_mutex.h"
#include "task.h"

typedef struct {
    uint32_t lr;
    LightMutexHandle_t freertos_mutex;
} PebbleMutexCommon;

struct pebble_mutex_t {
  PebbleMutexCommon common;
};

struct pebble_recursive_mutex_t {
  PebbleMutexCommon common;
};

// macros should only be called while we hold the mutex we are logging so no
// additional locking is needed
#define LOG_LOCKED(logged_lr, new_lr)    \
  if (logged_lr == 0) {              \
    logged_lr = new_lr;              \
  }

#define LOG_UNLOCKED(logged_lr, nest_count) \
  if (nest_count == 1) {                \
    logged_lr = 0;                      \
  }

static PebbleMutexCommon *create_pebble_mutex(LightMutexHandle_t freertos_mutex) {
  PebbleMutexCommon *mutex = os_malloc_check(sizeof(PebbleMutex));
  *mutex = (PebbleMutexCommon) {
    .freertos_mutex = freertos_mutex,
    .lr = 0
  };

  return mutex;
}

PebbleMutex * mutex_create(void) {
  LightMutexHandle_t freertos_mutex = xLightMutexCreate();
  OS_ASSERT(freertos_mutex != NULL);
  PebbleMutex *mutex = (PebbleMutex *)create_pebble_mutex(freertos_mutex);
  return mutex;
}

void mutex_destroy(PebbleMutex * handle) {
  OS_ASSERT(handle != NULL);
  LightMutexHandle_t mutex = handle->common.freertos_mutex;
  vLightMutexDelete(mutex);
  os_free(handle);
}

void mutex_lock(PebbleMutex * handle) {
  uintptr_t myLR = (uintptr_t) __builtin_return_address(0);

  OS_ASSERT(!mcu_state_is_isr());

  xLightMutexLock(handle->common.freertos_mutex, portMAX_DELAY);
  LOG_LOCKED(handle->common.lr, myLR);
}

bool mutex_lock_with_timeout(PebbleMutex * handle, uint32_t timeout_ms) {
  uintptr_t myLR = (uintptr_t) __builtin_return_address(0);

  OS_ASSERT(!mcu_state_is_isr());

  TickType_t timeout_ticks = milliseconds_to_ticks(timeout_ms);
  LightMutexHandle_t mutex = handle->common.freertos_mutex;

  if (xLightMutexLock(mutex, timeout_ticks) == pdTRUE) {
    LOG_LOCKED(handle->common.lr, myLR);
    return (true);
  }

  return (false);
}

void mutex_lock_with_lr(PebbleMutex * handle, uint32_t myLR) {
  OS_ASSERT(!mcu_state_is_isr());

  xLightMutexLock(handle->common.freertos_mutex, portMAX_DELAY);
  LOG_LOCKED(handle->common.lr, myLR);
}

void mutex_unlock(PebbleMutex * handle) {
  OS_ASSERT(!mcu_state_is_isr());
  LOG_UNLOCKED(handle->common.lr, 1)
  xLightMutexUnlock(handle->common.freertos_mutex);
}

static void prv_assert_held_by_curr_task(PebbleMutex * handle, bool is_held, uint32_t lr) {
  LightMutexHandle_t mutex = handle->common.freertos_mutex;
  OS_ASSERT_LR((xLightMutexGetHolder(mutex) == xTaskGetCurrentTaskHandle()) == is_held,
                 lr);
}

void mutex_assert_held_by_curr_task(PebbleMutex * handle, bool is_held) {
  uintptr_t saved_lr = (uintptr_t) __builtin_return_address(0);
  prv_assert_held_by_curr_task(handle, is_held, saved_lr);
}

void mutex_assert_recursive_held_by_curr_task(PebbleRecursiveMutex * handle, bool is_held) {
  uintptr_t saved_lr = (uintptr_t) __builtin_return_address(0);
  prv_assert_held_by_curr_task((PebbleMutex *) handle, is_held, saved_lr);
}

PebbleRecursiveMutex * mutex_create_recursive(void) {
  LightMutexHandle_t freertos_mutex = xLightMutexCreateRecursive();
  OS_ASSERT(freertos_mutex != NULL);
  PebbleRecursiveMutex * mutex = (PebbleRecursiveMutex *)create_pebble_mutex(freertos_mutex);

  return mutex;
}

void mutex_lock_recursive(PebbleRecursiveMutex * handle) {
  uintptr_t myLR = (uintptr_t) __builtin_return_address(0);
  OS_ASSERT(!mcu_state_is_isr());
  xLightMutexLockRecursive(handle->common.freertos_mutex, portMAX_DELAY);
  LOG_LOCKED(handle->common.lr, myLR);
}

bool mutex_lock_recursive_with_timeout_and_lr(PebbleRecursiveMutex * handle,
    uint32_t timeout_ms, uint32_t myLR) {
  OS_ASSERT(!mcu_state_is_isr());

  TickType_t timeout_ticks = milliseconds_to_ticks(timeout_ms);
  LightMutexHandle_t mutex = handle->common.freertos_mutex;

  if (xLightMutexLockRecursive(mutex, timeout_ticks) == pdTRUE) {
    LOG_LOCKED(handle->common.lr, myLR);
    return (true);
  }

  return (false);
}

bool mutex_lock_recursive_with_timeout(PebbleRecursiveMutex * handle, uint32_t timeout_ms) {
  uintptr_t myLR = (uintptr_t) __builtin_return_address(0);

  OS_ASSERT(!mcu_state_is_isr());

  TickType_t timeout_ticks = milliseconds_to_ticks(timeout_ms);
  LightMutexHandle_t mutex = handle->common.freertos_mutex;

  if (xLightMutexLockRecursive(mutex, timeout_ticks) == pdTRUE) {
    LOG_LOCKED(handle->common.lr, myLR);
    return (true);
  }

  return (false);
}

bool mutex_is_owned_recursive(PebbleRecursiveMutex * handle) {
  LightMutexHandle_t mutex = handle->common.freertos_mutex;
  void* holder = xLightMutexGetHolder(mutex);
  void* current = xTaskGetCurrentTaskHandle();
  return (holder == current);
}

void mutex_unlock_recursive(PebbleRecursiveMutex * handle) {
  OS_ASSERT(!mcu_state_is_isr());
  LightMutexHandle_t mutex = handle->common.freertos_mutex;
  LOG_UNLOCKED(handle->common.lr, uxLightMutexGetRecursiveCallCount(mutex));
  xLightMutexUnlockRecursive(mutex);
}

