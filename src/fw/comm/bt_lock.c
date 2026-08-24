/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bt_lock.h"

#include "system/passert.h"

#include "pbl/util/misc.h"
#include "portmacro.h"

// NOTE: The s_bt_lock is the global Bluetooth lock that is used by the firmware
// *and* by Bluetopia. It gets handed to Bluetopia using bt_lock_get() in
// BTPSKRNL.c when Bluetopia is initialized. The firmware uses this lock to
// protect Bluetooth-related state that is read and written from the Bluetooth
// callback task (PebbleTask_BTHost) and other tasks. If we created our
// own mutex for this purpose, we would encounter dead-lock situations.
// For example:
//   Task1: Bluetopia code -> grabs BT stack lock -> Pebble callback -> grabs pebble mutex
//   Task2: Pebble code -> grabs pebble mutex -> calls into Bluetopia -> tries to grab BT stack lock

static PebbleRecursiveMutex *s_bt_lock;

void bt_lock_init(void) {
  // Never free'd.
  PBL_ASSERTN(!s_bt_lock);
  s_bt_lock = mutex_create_recursive();
}

PebbleRecursiveMutex *bt_lock_get(void) {
  return s_bt_lock;
}

void bt_lock(void) {
  uint32_t myLR = PBL_RETURN_ADDRESS();
  mutex_lock_recursive_with_timeout_and_lr(s_bt_lock, portMAX_DELAY, myLR);
}

void bt_unlock(void) {
  mutex_unlock_recursive(s_bt_lock);
}

void bt_lock_assert_held(bool is_held) {
  mutex_assert_recursive_held_by_curr_task(s_bt_lock, is_held);
}

bool bt_lock_is_held(void) {
  return mutex_is_owned_recursive(s_bt_lock);
}
