/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/speaker/note_sequence.h"

#include <stdint.h>

//! Short chirps played when a notification arrives, deliberately briefer than
//! the alarm tones: a notification sound must not outstay the vibe.
typedef enum NotificationSound {
  NotificationSound_None = 0,
  NotificationSound_Ping,
  NotificationSound_Doorbell,
  NotificationSound_Trill,
  NotificationSound_Ascent,
  NotificationSound_Glass,  // replaced Bell; existing stored prefs map here
  NotificationSound_Pop,
  NotificationSound_Count,
} NotificationSound;

//! Play a notification sound on the speaker at the given volume. Synthesized
//! sounds go through the note-sequence source; sampled sounds (e.g. Bell)
//! through the track player. Out-of-range values fall back to Ping;
//! NotificationSound_None is a no-op.
//! @return true if playback started.
bool notification_sounds_play(NotificationSound sound, uint8_t volume);

//! Get the i18n_noop()'d display name for a sound (including "Off" for None).
//! Caller wraps with i18n_get() at display time.
const char *notification_sounds_get_name(NotificationSound sound);

//! Next sound in the cycle order for the settings UI, wrapping from the last
//! sound back to None.
NotificationSound notification_sounds_cycle_next(NotificationSound sound);
