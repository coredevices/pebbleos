/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/events.h"
#include "pbl/services/notifications/alerts_private.h"
#include "pbl/services/notifications/do_not_disturb.h"
#include "pbl/services/vibes/vibe_intensity.h"
#include "util/time/time.h"
#include "pbl/services/vibes/vibe_client.h"
#include "pbl/services/vibes/vibe_score_info.h"

#define NOTIF_WINDOW_TIMEOUT_INFINITE ((uint32_t)~0)
#define NOTIF_WINDOW_TIMEOUT_DEFAULT (3 * MS_PER_MINUTE)

void alerts_preferences_init(void);

AlertMask alerts_preferences_get_alert_mask(void);

void alerts_preferences_set_alert_mask(AlertMask mask);

AlertMask alerts_preferences_dnd_get_mask(void);

void alerts_preferences_dnd_set_mask(AlertMask mask);

uint32_t alerts_preferences_get_notification_window_timeout_ms(void);

void alerts_preferences_set_notification_window_timeout_ms(uint32_t timeout_ms);

bool alerts_preferences_get_notification_alternative_design(void);

void alerts_preferences_set_notification_alternative_design(bool alternative);

bool alerts_preferences_get_notification_vibe_delay(void);

void alerts_preferences_set_notification_vibe_delay(bool delay);

bool alerts_preferences_get_notification_backlight(void);

void alerts_preferences_set_notification_backlight(bool enable);

typedef enum {
  NotificationStatusBarStyle_Default = 0,
  NotificationStatusBarStyle_Bold = 1,
  NotificationStatusBarStyle_LargeBold = 2,
  NotificationStatusBarStyleCount,
} NotificationStatusBarStyle;

NotificationStatusBarStyle alerts_preferences_get_notification_status_bar_style(void);

void alerts_preferences_set_notification_status_bar_style(NotificationStatusBarStyle style);

bool alerts_preferences_get_vibrate(void);

void alerts_preferences_set_vibrate(bool enable);

VibeIntensity alerts_preferences_get_vibe_intensity(void);

void alerts_preferences_set_vibe_intensity(VibeIntensity intensity);

VibeScoreId alerts_preferences_get_vibe_score_for_client(VibeClient client);

void alerts_preferences_set_vibe_score_for_client(VibeClient client, VibeScoreId id);

bool alerts_preferences_dnd_is_manually_enabled(void);

void alerts_preferences_dnd_set_manually_enabled(bool enable);

void alerts_preferences_dnd_get_schedule(DoNotDisturbScheduleType type,
                                         DoNotDisturbSchedule *schedule_out);

void alerts_preferences_dnd_set_schedule(DoNotDisturbScheduleType type,
                                         const DoNotDisturbSchedule *schedule);

bool alerts_preferences_dnd_is_schedule_enabled(DoNotDisturbScheduleType type);

void alerts_preferences_dnd_set_schedule_enabled(DoNotDisturbScheduleType type, bool enable);

bool alerts_preferences_dnd_is_smart_enabled(void);

void alerts_preferences_dnd_set_smart_enabled(bool enable);

//! Lock the alerts preferences mutex. Must be paired with alerts_preferences_unlock().
void alerts_preferences_lock(void);

//! Unlock the alerts preferences mutex. Must be paired with alerts_preferences_lock().
void alerts_preferences_unlock(void);

//! Process a BlobDB event for notification preferences. For BlobDBEventTypeInsert
//! events, this method will update the internal global copy of that preference based on the
//! new value that was placed into the backing store.
//! @param[in] event pointer to the blob DB event
void alerts_preferences_handle_blob_db_event(PebbleBlobDBEvent *event);

