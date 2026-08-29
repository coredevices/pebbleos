/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/notifications/alerts.h"
#include "pbl/services/notifications/alerts_private.h"

#include <pbl/drivers/battery.h>
#include <pbl/drivers/rtc.h>
#include "kernel/low_power.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/firmware_update.h"
#include "pbl/services/notifications/do_not_disturb.h"
#include "pbl/services/notifications/alerts_preferences_private.h"

static const int NOTIFICATION_VIBE_HOLDOFF_MS = 3000;
static RtcTicks s_notification_vibe_tick_timestamp = 0;

//////////////////
// Private Functions
//////////////////

static int64_t prv_get_ms_since_last_notification_vibe(void) {
  RtcTicks current_ticks = rtc_get_ticks();
  int64_t millis_since_last_vibe =
      (current_ticks - s_notification_vibe_tick_timestamp) * 1000 / RTC_TICKS_HZ; // x1000 for ms
  return millis_since_last_vibe;
}

//////////////////
// Public Functions
//////////////////

void alerts_incoming_alert_analytics() {
  if (do_not_disturb_is_active()) {
    PBL_ANALYTICS_ADD(notification_received_dnd_count, 1);
  }
}

bool alerts_should_notify_for_type(AlertType type) {
  if (low_power_is_active()) {
    return false;
  }

  if (firmware_update_is_in_progress()) {
    return false;
  }

  return alerts_preferences_get_alert_mask() & type;
}

bool alerts_should_enable_backlight_for_type(AlertType type) {
  if (!alerts_preferences_get_notification_backlight()) {
    return false;
  }

  if (do_not_disturb_is_active() && !(alerts_preferences_dnd_get_mask() & type)) {
    return false;
  }

  return alerts_should_notify_for_type(type);
}

bool alerts_should_vibrate_for_type(AlertType type) {
  if (do_not_disturb_is_active() && !(alerts_preferences_dnd_get_mask() & type)) {
    return false;
  }

  if (!alerts_should_notify_for_type(type)) {
    return false;
  }

  if (battery_is_usb_connected()) {
    return false;
  }

  if (prv_get_ms_since_last_notification_vibe() < NOTIFICATION_VIBE_HOLDOFF_MS) {
    return false;
  }

  return alerts_preferences_get_vibrate();
}

bool alerts_should_play_sound_for_type(AlertType type) {
  if (alerts_preferences_get_notification_sound() == NotificationSound_None) {
    return false;
  }

  if (do_not_disturb_is_active() && !(alerts_preferences_dnd_get_mask() & type)) {
    return false;
  }

  if (!alerts_should_notify_for_type(type)) {
    return false;
  }

  // Reuse the vibe holdoff so a notification storm doesn't chirp continuously.
  // Unlike vibes, sounds still play on the charger: the watch may be off-wrist
  // where a vibe would go unnoticed but a chirp would not.
  if (prv_get_ms_since_last_notification_vibe() < NOTIFICATION_VIBE_HOLDOFF_MS) {
    return false;
  }

  return true;
}

bool alerts_get_vibrate(void) {
  return alerts_preferences_get_vibrate();
}

AlertMask alerts_get_mask(void) {
  return alerts_preferences_get_alert_mask();
}

AlertMask alerts_get_dnd_mask(void) {
  return alerts_preferences_dnd_get_mask();
}

uint32_t alerts_get_notification_window_timeout_ms(void) {
  return alerts_preferences_get_notification_window_timeout_ms();
}

void alerts_set_vibrate(bool enable) {
  alerts_preferences_set_vibrate(enable);
}

void alerts_set_mask(AlertMask mask) {
  alerts_preferences_set_alert_mask(mask);
}

void alerts_set_dnd_mask(AlertMask mask) {
  alerts_preferences_dnd_set_mask(mask);
}

void alerts_set_notification_vibe_timestamp() {
  // if we do vibrate, update timestamp of last vibration
  s_notification_vibe_tick_timestamp = rtc_get_ticks();
}

void alerts_set_notification_window_timeout_ms(uint32_t timeout_ms) {
  alerts_preferences_set_notification_window_timeout_ms(timeout_ms);
}

void alerts_init() {
  alerts_preferences_init();
  do_not_disturb_init();
  vibe_intensity_init();
}
