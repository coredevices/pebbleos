/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bt_lock.h"

#include "system/passert.h"

// NOTE: The s_bt_lock is the global Bluetooth lock that is used by the firmware
// *and* by Bluetopia. It gets handed to Bluetopia using bt_lock_get() in
// BTPSKRNL.c when Bluetopia is initialized. The firmware uses this lock to
// protect Bluetooth-related state that is read and written from the Bluetooth
// callback task (PebbleTask_BTHost) and other tasks. If we created our
// own mutex for this purpose, we would encounter dead-lock situations.
// For example:
//   Task1: Bluetopia code -> grabs BT stack lock -> Pebble callback -> grabs pebble mutex
//   Task2: Pebble code -> grabs pebble mutex -> calls into Bluetopia -> tries to grab BT stack lock

static PBL_MUTEX_DEFINE(s_bt_lock);

void bt_lock_init(void) {
}

struct pbl_mutex *bt_lock_get(void) {
  return &s_bt_lock;
}

void bt_lock(void) {
  register uint32_t LR __asm ("lr");
  uint32_t myLR = LR;
  pbl_mutex_lock_lr(&s_bt_lock, PBL_FOREVER, myLR);
}

void bt_unlock(void) {
  pbl_mutex_unlock(&s_bt_lock);
}

void bt_lock_assert_held(bool is_held) {
  pbl_mutex_assert_held(&s_bt_lock, is_held);
}

bool bt_lock_is_held(void) {
  return pbl_mutex_is_owner(&s_bt_lock);
}
