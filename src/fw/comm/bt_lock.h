/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/mutex.h"

void bt_lock_init(void);

//! Function to get the shared mutex. This is used in BTPSKRNL.c to hand
//! Bluetopia this mutex to use as its BSC_LockBluetoothStack mutex.
struct pbl_mutex *bt_lock_get(void);

//! Lock the shared Bluetooth recursive lock to protect Bluetooth related state.
void bt_lock(void);

//! Unlock the shared Bluetooth recursive lock.
void bt_unlock(void);

//! Asserts whether the bt_lock() is held or not.
void bt_lock_assert_held(bool is_held);

//! Returns true if the bt lock is held, else false
bool bt_lock_is_held(void);
