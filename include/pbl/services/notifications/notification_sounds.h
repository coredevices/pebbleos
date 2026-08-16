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
  NotificationSound_Count,
} NotificationSound;

//! Look up the SpeakerNote sequence for a notification sound.
//! @param sound The sound to look up. Must not be NotificationSound_None.
//! @param notes_out Receives a pointer to the static note array.
//! @param count_out Receives the number of notes in the array.
//! Falls back to NotificationSound_Ping for out-of-range or None values.
void notification_sounds_get(NotificationSound sound, const SpeakerNote **notes_out,
                             uint32_t *count_out);

//! Get the i18n_noop()'d display name for a sound (including "Off" for None).
//! Caller wraps with i18n_get() at display time.
const char *notification_sounds_get_name(NotificationSound sound);

//! Next sound in the cycle order for the settings UI, wrapping from the last
//! sound back to None.
NotificationSound notification_sounds_cycle_next(NotificationSound sound);
