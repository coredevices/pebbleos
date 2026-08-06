/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

//! Interaction session gating raw touch navigation.
//!
//! Raw touchdowns only navigate (and hold the backlight) while the session is
//! active. Deliberate interaction arms it: the backlight wake gesture or a
//! button press. Use extends it; it expires after the backlight timeout, as if
//! the light had come on — tracked as its own deadline because bright ambient
//! (or DnD) keeps the actual backlight off. The idle watchface is the only
//! guarded surface: any other foreground UI, a focused modal, or a lit
//! backlight counts as active.
//!
//! All entry points run on KernelMain (event-loop handlers); the module is
//! unlocked by design.

typedef enum TouchSessionArmSource {
  TouchSessionArmSource_WakeGesture,
  TouchSessionArmSource_Button,
} TouchSessionArmSource;

//! Open (or re-open) the session: deliberate interaction observed.
void touch_session_arm(TouchSessionArmSource source);

//! Push the session deadline out, but only while the session is active --
//! gated contact must not keep itself alive.
void touch_session_extend(void);

//! @return true while touch may navigate: within the armed deadline, the
//! foreground UI is not a watchface, a focused modal is up, or the backlight
//! is on. Always true in recovery firmware (mfg touch test).
bool touch_session_is_active(void);

//! Close the session deadline immediately. The environmental overrides in
//! touch_session_is_active() still apply.
void touch_session_reset(void);
