/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "alerts_private.h"
#include "pbl/services/notifications/notification_sounds.h"

typedef enum FirstUseSource {
  FirstUseSourceManualDNDActionMenu = 0,
  FirstUseSourceManualDNDSettingsMenu,
  FirstUseSourceSmartDND,
  FirstUseSourceDismiss
} FirstUseSource;

typedef enum MuteBitfield {
  MuteBitfield_None     = 0b00000000,
  MuteBitfield_Always   = 0b01111111,
  MuteBitfield_Weekdays = 0b00111110,
  MuteBitfield_Weekends = 0b01000001,
} MuteBitfield;

typedef enum {
  DndNotificationModeHide = 0,
  DndNotificationModeShow = 1,
} DndNotificationMode;

//! Set notification display mode when DND is active
//! @param mode The display mode (Show or Hide)
void alerts_preferences_dnd_set_show_notifications(DndNotificationMode mode);

//! @return The notification display mode when DND is active
DndNotificationMode alerts_preferences_dnd_get_show_notifications(void);

//! Set whether the backlight should turn on with motion when DND is active
//! @param enable true to allow motion backlight, false to suppress it
void alerts_preferences_dnd_set_motion_backlight(bool enable);

//! @return Whether motion backlight is enabled when DND is active
bool alerts_preferences_dnd_get_motion_backlight(void);

//! Set whether tap / double-tap gestures should turn on the backlight when DND is active
//! @param enable true to allow touch backlight, false to suppress it
void alerts_preferences_dnd_set_touch_backlight(bool enable);

//! @return Whether touch backlight is enabled when DND is active
bool alerts_preferences_dnd_get_touch_backlight(void);

//! Set whether the speaker should be muted while DND is active.
//! @param enable true to mute the speaker during DND, false to allow audio
void alerts_preferences_dnd_set_mute_speaker(bool enable);

//! @return Whether the speaker is muted while DND is active
bool alerts_preferences_dnd_get_mute_speaker(void);

//! Set whether notifications should auto-dismiss even when DND is active.
//! When false (default), notifications stay on screen during DND.
//! @param enable true to allow auto-dismiss during DND, false to suppress it
void alerts_preferences_dnd_set_auto_dismiss(bool enable);

//! @return Whether auto-dismiss is allowed while DND is active
bool alerts_preferences_dnd_get_auto_dismiss(void);

//! Set the always-on speaker mute. When set, the speaker is silenced
//! regardless of DND state.
//! @param muted true to mute the speaker, false to allow audio
void alerts_preferences_set_speaker_muted(bool muted);

//! @return Whether the speaker is always-on muted
bool alerts_preferences_get_speaker_muted(void);

//! Set the system-wide speaker volume cap. Per-playback volumes are scaled
//! by this value before being applied to the audio hardware.
//! @param volume Volume cap, 0-100. Values outside the range are clamped.
void alerts_preferences_set_speaker_volume(uint8_t volume);

//! @return The system-wide speaker volume cap (0-100). Defaults to 100.
uint8_t alerts_preferences_get_speaker_volume(void);

//! Set the sound played when a notification arrives.
void alerts_preferences_set_notification_sound(NotificationSound sound);

//! @return The sound played when a notification arrives. Defaults to
//! NotificationSound_None (silent); out-of-range stored values also read
//! back as None.
NotificationSound alerts_preferences_get_notification_sound(void);

//! Checks whether a given "first use" dialog has been shown and sets it as complete
//! @param source The "first use" bit to check
//! @return true if the dialog has already been shown, false otherwise
bool alerts_preferences_check_and_set_first_use_complete(FirstUseSource source);
