/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/events.h"

//! @file
//!
//! Button lock: holding Back+Down for a configurable duration
//! (shell_prefs_get_button_lock_hold_ms, 0 = disabled) locks all button and
//! touch input; holding the combo again unlocks.
//!
//! The locked state is intentionally RAM-only: a reboot always unlocks. The
//! hardware reset combo is handled at ISR level in the button driver and is
//! unaffected by the lock.

//! Create resources used by the button lock. Called from shell_event_loop_init.
void button_lock_init(void);

bool button_lock_is_locked(void);

//! Feed every button event on KernelMain before any other handling.
//! @return true if the event must be swallowed (masked from all tasks).
bool button_lock_handle_button_event(PebbleEvent *e);
