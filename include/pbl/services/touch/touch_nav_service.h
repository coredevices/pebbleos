/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

//! Run the master touch-navigation enable/disable transaction.
//!
//! Coordinates the kernel and app twins of the touch-nav bridge and the permanent sensor hold in
//! the mandated order (see \ref touch_nav_transaction_apply). The pref value itself is persisted by
//! the shell pref system before this is called; this flips the runtime gate and juggles the
//! subscriptions and hold.
//!
//! @param enable true to turn touch navigation on, false to turn it off
void touch_nav_set_enabled(bool enable);

//! Notify the touch-nav service that the master "Touch" pref changed WITHOUT changing the
//! effective system state (the Touch Navigation sub-pref is off). Re-evaluates the app twin for
//! the running app: an explicitly opted-in app follows the master pref alone.
void touch_nav_master_changed(void);
