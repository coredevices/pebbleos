/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/notifications/alerts_preferences_private.h"
#include "pbl/util/attributes.h"

VibeScoreId WEAK alerts_preferences_get_vibe_score_for_client(VibeClient client) {
  return VibeScoreId_Invalid;
}

VibeIntensity WEAK alerts_preferences_get_vibe_intensity(void) {
  return VibeIntensityLow;
}

bool WEAK alerts_preferences_get_notification_alternative_design(void) {
  return false;
}

DndNotificationMode WEAK alerts_preferences_dnd_get_show_notifications(void) {
  return DndNotificationModeShow;
}

bool WEAK alerts_preferences_dnd_get_auto_dismiss(void) {
  return false;
}

bool WEAK alerts_preferences_get_notification_vibe_delay(void) {
  return false;
}

NotificationStatusBarStyle WEAK alerts_preferences_get_notification_status_bar_style(void) {
  return NotificationStatusBarStyle_Default;
}

bool WEAK alerts_preferences_get_respect_phone_silence(void) {
  return false;
}
