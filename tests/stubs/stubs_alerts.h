/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/notifications/alerts_private.h"
#include "pbl/util/attributes.h"

AlertMask s_alert_mask;

AlertMask WEAK alerts_get_mask(void) {
  return s_alert_mask;
}

void WEAK alerts_set_mask(AlertMask mask) {
  s_alert_mask = mask;
}

uint32_t WEAK alerts_get_notification_window_timeout_ms(void) {
  return 0;
}

void WEAK alerts_incoming_alert_analytics() {}

void WEAK alerts_set_notification_vibe_timestamp() {}

bool WEAK alerts_should_enable_backlight_for_type(AlertType type) {
  return false;
}

bool WEAK alerts_should_notify_for_type(AlertType type) {
  return false;
}

bool WEAK alerts_should_vibrate_for_type(AlertType type) {
  return false;
}

bool WEAK alerts_should_play_sound_for_type(AlertType type) {
  return false;
}