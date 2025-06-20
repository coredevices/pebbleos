/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "services/normal/notifications/alerts_preferences.h"
#include "services/normal/notifications/alerts_preferences_private.h"

#include "drivers/rtc.h"
#include "popups/notifications/notification_window.h"
#include "services/common/analytics/analytics.h"
#include "services/normal/notifications/do_not_disturb.h"
#include "services/normal/settings/settings_file.h"
#include "services/normal/vibes/vibe_intensity.h"
#include "system/passert.h"
#include "os/mutex.h"
#include "util/bitset.h"

#include <string.h>

#define FILE_NAME "notifpref"
#define FILE_LEN (1024)

static PebbleMutex *s_mutex;

///////////////////////////////////
//! Preference keys
///////////////////////////////////

#define PREF_KEY_MASK "mask"
static AlertMask s_mask = AlertMaskAllOn;

#define PREF_KEY_DND_INTERRUPTIONS_MASK "dndInterruptionsMask"
static AlertMask s_dnd_interruptions_mask = AlertMaskAllOff;

#define PREF_KEY_VIBE "vibe"
static bool s_vibe_on_notification = true;

#define PREF_KEY_VIBE_INTENSITY "vibeIntensity"
static VibeIntensity s_vibe_intensity = DEFAULT_VIBE_INTENSITY;

#if CAPABILITY_HAS_VIBE_SCORES
#define PREF_KEY_VIBE_SCORE_NOTIFICATIONS ("vibeScoreNotifications")
static VibeScoreId s_vibe_score_notifications = DEFAULT_VIBE_SCORE_NOTIFS;

#define PREF_KEY_VIBE_SCORE_INCOMING_CALLS ("vibeScoreIncomingCalls")
static VibeScoreId s_vibe_score_incoming_calls = DEFAULT_VIBE_SCORE_INCOMING_CALLS;

#define PREF_KEY_VIBE_SCORE_ALARMS ("vibeScoreAlarms")
static VibeScoreId s_vibe_score_alarms = DEFAULT_VIBE_SCORE_ALARMS;
#endif

#define PREF_KEY_DND_MANUALLY_ENABLED "dndManuallyEnabled"
static bool s_do_not_disturb_manually_enabled = false;

#define PREF_KEY_DND_SMART_ENABLED "dndSmartEnabled"
static bool s_do_not_disturb_smart_dnd_enabled = false;

#define PREF_KEY_FIRST_USE_COMPLETE "firstUseComplete"
static uint32_t s_first_use_complete = 0;

#define PREF_KEY_NOTIF_WINDOW_TIMEOUT "notifWindowTimeout"
static uint32_t s_notif_window_timeout_ms = NOTIF_WINDOW_TIMEOUT_DEFAULT;

#define PREF_KEY_DELAYED_VIBRATION "delayedVibration"
static bool s_delayed_vibration_enabled = true;

///////////////////////////////////
//! Legacy preference keys
///////////////////////////////////

#define PREF_KEY_LEGACY_DND_SCHEDULE "dndSchedule"
static DoNotDisturbSchedule s_legacy_dnd_schedule = {
  .from_hour = 0,
  .to_hour = 6,
};

#define PREF_KEY_LEGACY_DND_SCHEDULE_ENABLED "dndEnabled"
static bool s_legacy_dnd_schedule_enabled = false;

#define PREF_KEY_LEGACY_DND_MANUAL_FIRST_USE "dndManualFirstUse"
#define PREF_KEY_LEGACY_DND_SMART_FIRST_USE "dndSmartFirstUse"

///////////////////////////////////
//! Variables
///////////////////////////////////

typedef struct DoNotDisturbScheduleConfig {
  DoNotDisturbSchedule schedule;
  bool enabled;
} DoNotDisturbScheduleConfig;

typedef struct DoNotDisturbScheduleConfigKeys {
  const char *schedule_pref_key;
  const char *enabled_pref_key;
} DoNotDisturbScheduleConfigKeys;

static DoNotDisturbScheduleConfig s_dnd_schedule[NumDNDSchedules];

static const DoNotDisturbScheduleConfigKeys s_dnd_schedule_keys[NumDNDSchedules] = {
  [WeekdaySchedule] = {
    .schedule_pref_key = "dndWeekdaySchedule",
    .enabled_pref_key = "dndWeekdayScheduleEnabled",
  },
  [WeekendSchedule] = {
    .schedule_pref_key = "dndWeekendSchedule",
    .enabled_pref_key = "dndWeekendScheduleEnabled",
  }
};

static void prv_migrate_legacy_dnd_schedule(SettingsFile *file) {
  // If Weekday schedule does not exist, assume that the other 3 settings files are missing as well
  // Set the new schedules to the legacy schedule and delete the legacy schedule
  if (!settings_file_exists(file, s_dnd_schedule_keys[WeekdaySchedule].schedule_pref_key,
                           strlen(s_dnd_schedule_keys[WeekdaySchedule].schedule_pref_key))) {
#define SET_PREF_ALREADY_OPEN(key, value) \
    settings_file_set(file, key, strlen(key), value, sizeof(value));

    s_dnd_schedule[WeekdaySchedule].schedule = s_legacy_dnd_schedule;
    SET_PREF_ALREADY_OPEN(s_dnd_schedule_keys[WeekdaySchedule].schedule_pref_key,
                          &s_dnd_schedule[WeekdaySchedule].schedule);
    s_dnd_schedule[WeekdaySchedule].enabled = s_legacy_dnd_schedule_enabled;
    SET_PREF_ALREADY_OPEN(s_dnd_schedule_keys[WeekdaySchedule].enabled_pref_key,
                          &s_dnd_schedule[WeekdaySchedule].enabled);
    s_dnd_schedule[WeekendSchedule].schedule = s_legacy_dnd_schedule;
    SET_PREF_ALREADY_OPEN(s_dnd_schedule_keys[WeekendSchedule].schedule_pref_key,
                          &s_dnd_schedule[WeekendSchedule].schedule);
    s_dnd_schedule[WeekendSchedule].enabled = s_legacy_dnd_schedule_enabled;
    SET_PREF_ALREADY_OPEN(s_dnd_schedule_keys[WeekendSchedule].enabled_pref_key,
                          &s_dnd_schedule[WeekendSchedule].enabled);
#undef SET_PREF_ALREADY_OPEN

#define DELETE_PREF(key) \
    do { \
      if (settings_file_exists(file, key, strlen(key))) { \
        settings_file_delete(file, key, strlen(key)); \
      } \
    } while (0)

    DELETE_PREF(PREF_KEY_LEGACY_DND_SCHEDULE);
    DELETE_PREF(PREF_KEY_LEGACY_DND_SCHEDULE_ENABLED);
#undef DELETE_PREF
  }
}

#if !PLATFORM_TINTIN
static void prv_migrate_legacy_first_use_settings(SettingsFile *file) {
  // These don't need to be initialized since settings_file_get will clear them on error
  uint8_t manual_dnd_first_use_complete;
  bool smart_dnd_first_use_complete;

  // Migrate the old first use dialog prefs
#define RESTORE_AND_DELETE_PREF(key, var) \
  do { \
    if (settings_file_get(file, key, strlen(key), &var, sizeof(var)) == S_SUCCESS) { \
      settings_file_delete(file, key, strlen(key)); \
    } \
  } while (0)

  RESTORE_AND_DELETE_PREF(PREF_KEY_LEGACY_DND_MANUAL_FIRST_USE, manual_dnd_first_use_complete);
  RESTORE_AND_DELETE_PREF(PREF_KEY_LEGACY_DND_SMART_FIRST_USE, smart_dnd_first_use_complete);

  s_first_use_complete |= manual_dnd_first_use_complete << FirstUseSourceManualDNDActionMenu;
  s_first_use_complete |= smart_dnd_first_use_complete << FirstUseSourceSmartDND;

#undef RESTORE_AND_DELETE_PREF
}
#endif

#if CAPABILITY_HAS_VIBE_SCORES
static void prv_save_all_vibe_scores_to_file(SettingsFile *file) {
#define SET_PREF_ALREADY_OPEN(key, value) \
    settings_file_set(file, key, strlen(key), &value, sizeof(value));

  SET_PREF_ALREADY_OPEN(PREF_KEY_VIBE_SCORE_NOTIFICATIONS, s_vibe_score_notifications);
  SET_PREF_ALREADY_OPEN(PREF_KEY_VIBE_SCORE_INCOMING_CALLS, s_vibe_score_incoming_calls);
  SET_PREF_ALREADY_OPEN(PREF_KEY_VIBE_SCORE_ALARMS, s_vibe_score_alarms);
#undef SET_PREF_ALREADY_OPEN
}

static VibeScoreId prv_return_default_if_invalid(VibeScoreId id, VibeScoreId default_id) {
  return vibe_score_info_is_valid(id) ? id : default_id;
}

// Uses the default vibe pattern id if the given score isn't valid
static void prv_ensure_valid_vibe_scores(void) {
  s_vibe_score_notifications = prv_return_default_if_invalid(s_vibe_score_notifications,
                                                             DEFAULT_VIBE_SCORE_NOTIFS);
  s_vibe_score_incoming_calls = prv_return_default_if_invalid(s_vibe_score_incoming_calls,
                                                              DEFAULT_VIBE_SCORE_INCOMING_CALLS);
  s_vibe_score_alarms = prv_return_default_if_invalid(s_vibe_score_alarms,
                                                      DEFAULT_VIBE_SCORE_ALARMS);
}

static void prv_set_vibe_scores_based_on_legacy_intensity(VibeIntensity intensity) {
  if (intensity == VibeIntensityHigh) {
    s_vibe_score_notifications = VibeScoreId_StandardShortPulseHigh;
    s_vibe_score_incoming_calls = VibeScoreId_StandardLongPulseHigh;
    s_vibe_score_alarms = VibeScoreId_StandardLongPulseHigh;
  } else {
    s_vibe_score_notifications = VibeScoreId_StandardShortPulseLow;
    s_vibe_score_incoming_calls = VibeScoreId_StandardLongPulseLow;
    s_vibe_score_alarms = VibeScoreId_StandardLongPulseLow;
  }
}

static void prv_migrate_vibe_intensity_to_vibe_scores(SettingsFile *file) {
  // We use the existence of the notifications vibe score pref as a shallow measurement of whether
  // or not the user has migrated to vibe scores
  const bool user_has_migrated_to_vibe_scores =
    settings_file_exists(file, PREF_KEY_VIBE_SCORE_NOTIFICATIONS,
                         strlen(PREF_KEY_VIBE_SCORE_NOTIFICATIONS));

  if (!user_has_migrated_to_vibe_scores) {
    // If the user previously set a vibration intensity, set the vibe scores based on that intensity
    if (settings_file_exists(file, PREF_KEY_VIBE_INTENSITY, strlen(PREF_KEY_VIBE_INTENSITY))) {
      prv_set_vibe_scores_based_on_legacy_intensity(s_vibe_intensity);
    } else if (rtc_is_timezone_set()) {
      // Otherwise, if the timezone has been set, then we assume this is a user on 3.10 and lower
      // that has not touched their vibe intensity preferences.
      // rtc_is_timezone_set() was chosen because it is a setting that gets written when the user
      // connects their watch to a phone
      prv_set_vibe_scores_based_on_legacy_intensity(DEFAULT_VIBE_INTENSITY);
    }
  }

  // PREF_KEY_VIBE, which used to track whether the user enabled/disabled vibrations, has been
  // deprecated in favor of the "disabled vibe score", VibeScoreId_Disabled, so switch to using it
  // and delete PREF_KEY_VIBE from the settings file if PREF_KEY_VIBE exists in the settings file
  if (settings_file_exists(file, PREF_KEY_VIBE, strlen(PREF_KEY_VIBE))) {
    if (!s_vibe_on_notification) {
      s_vibe_score_notifications = VibeScoreId_Disabled;
      s_vibe_score_incoming_calls = VibeScoreId_Disabled;
    }
    settings_file_delete(file, PREF_KEY_VIBE, strlen(PREF_KEY_VIBE));
  }
}
#endif

void alerts_preferences_init(void) {
  s_mutex = mutex_create();

  SettingsFile file = {{0}};
  if (settings_file_open(&file, FILE_NAME, FILE_LEN) != S_SUCCESS) {
    return;
  }

#define RESTORE_PREF(key, var) \
  do { \
    __typeof__(var) _tmp; \
    if (settings_file_get( \
        &file, key, strlen(key), &_tmp, sizeof(_tmp)) == S_SUCCESS) { \
      var = _tmp; \
    } \
  } while (0)

  RESTORE_PREF(PREF_KEY_MASK, s_mask);
  RESTORE_PREF(PREF_KEY_VIBE, s_vibe_on_notification);
  RESTORE_PREF(PREF_KEY_VIBE_INTENSITY, s_vibe_intensity);
#if CAPABILITY_HAS_VIBE_SCORES
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_NOTIFICATIONS, s_vibe_score_notifications);
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_INCOMING_CALLS, s_vibe_score_incoming_calls);
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_ALARMS, s_vibe_score_alarms);
#endif
  RESTORE_PREF(PREF_KEY_DND_MANUALLY_ENABLED, s_do_not_disturb_manually_enabled);
  RESTORE_PREF(PREF_KEY_DND_SMART_ENABLED, s_do_not_disturb_smart_dnd_enabled);
  RESTORE_PREF(PREF_KEY_DND_INTERRUPTIONS_MASK, s_dnd_interruptions_mask);
  RESTORE_PREF(PREF_KEY_LEGACY_DND_SCHEDULE, s_legacy_dnd_schedule);
  RESTORE_PREF(PREF_KEY_LEGACY_DND_SCHEDULE_ENABLED, s_legacy_dnd_schedule_enabled);
  RESTORE_PREF(s_dnd_schedule_keys[WeekdaySchedule].schedule_pref_key,
               s_dnd_schedule[WeekdaySchedule].schedule);
  RESTORE_PREF(s_dnd_schedule_keys[WeekdaySchedule].enabled_pref_key,
               s_dnd_schedule[WeekdaySchedule].enabled);
  RESTORE_PREF(s_dnd_schedule_keys[WeekendSchedule].schedule_pref_key,
               s_dnd_schedule[WeekendSchedule].schedule);
  RESTORE_PREF(s_dnd_schedule_keys[WeekendSchedule].enabled_pref_key,
               s_dnd_schedule[WeekendSchedule].enabled);
  RESTORE_PREF(PREF_KEY_FIRST_USE_COMPLETE, s_first_use_complete);
  RESTORE_PREF(PREF_KEY_NOTIF_WINDOW_TIMEOUT, s_notif_window_timeout_ms);
  RESTORE_PREF(PREF_KEY_DELAYED_VIBRATION, s_delayed_vibration_enabled);
#undef RESTORE_PREF

  prv_migrate_legacy_dnd_schedule(&file);

  // tintin watches don't have these prefs, so we can pull this out to save on codespace
#if !PLATFORM_TINTIN
  prv_migrate_legacy_first_use_settings(&file);
#endif
#if CAPABILITY_HAS_VIBE_SCORES
  prv_migrate_vibe_intensity_to_vibe_scores(&file);
  prv_ensure_valid_vibe_scores();
  prv_save_all_vibe_scores_to_file(&file);
#endif

  settings_file_close(&file);
}

// Convenience macro for setting a string key to a non-pointer value.
#define SET_PREF(key, value) \
  prv_set_pref(key, strlen(key), &value, sizeof(value))
static void prv_set_pref(const void *key, size_t key_len, const void *value,
                         size_t value_len) {
  mutex_lock(s_mutex);
  SettingsFile file = {{0}};
  if (settings_file_open(&file, FILE_NAME, FILE_LEN) != S_SUCCESS) {
    goto cleanup;
  }
  settings_file_set(&file, key, key_len, value, value_len);
  settings_file_close(&file);
cleanup:
  mutex_unlock(s_mutex);
}

AlertMask alerts_preferences_get_alert_mask(void) {
  if (s_mask == AlertMaskAllOnLegacy) {
    // Migration for notification settings previously configured under
    // old bit setup.
    alerts_preferences_set_alert_mask(AlertMaskAllOn);
  }
  return s_mask;
}

void alerts_preferences_set_alert_mask(AlertMask mask) {
  s_mask = mask;
  SET_PREF(PREF_KEY_MASK, s_mask);
}

uint32_t alerts_preferences_get_notification_window_timeout_ms(void) {
  return s_notif_window_timeout_ms;
}

void alerts_preferences_set_notification_window_timeout_ms(uint32_t timeout_ms) {
  s_notif_window_timeout_ms = timeout_ms;
  SET_PREF(PREF_KEY_NOTIF_WINDOW_TIMEOUT, s_notif_window_timeout_ms);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Vibes

bool alerts_preferences_get_vibrate(void) {
  return s_vibe_on_notification;
}

void alerts_preferences_set_vibrate(bool enable) {
  s_vibe_on_notification = enable;
  SET_PREF(PREF_KEY_VIBE, s_vibe_on_notification);
}

VibeIntensity alerts_preferences_get_vibe_intensity(void) {
  return s_vibe_intensity;
}

void alerts_preferences_set_vibe_intensity(VibeIntensity intensity) {
  s_vibe_intensity = intensity;
  SET_PREF(PREF_KEY_VIBE_INTENSITY, s_vibe_intensity);
}

#if CAPABILITY_HAS_VIBE_SCORES
VibeScoreId alerts_preferences_get_vibe_score_for_client(VibeClient client) {
  switch (client) {
    case VibeClient_Notifications:
      return s_vibe_score_notifications;
    case VibeClient_PhoneCalls:
      return s_vibe_score_incoming_calls;
    case VibeClient_Alarms:
      return s_vibe_score_alarms;
    default:
      WTF;
  }
}

void alerts_preferences_set_vibe_score_for_client(VibeClient client, VibeScoreId id) {
  const char *key = NULL;
  switch (client) {
    case VibeClient_Notifications: {
      s_vibe_score_notifications = id;
      key = PREF_KEY_VIBE_SCORE_NOTIFICATIONS;
      break;
    }
    case VibeClient_PhoneCalls: {
      s_vibe_score_incoming_calls = id;
      key = PREF_KEY_VIBE_SCORE_INCOMING_CALLS;
      break;
    }
    case VibeClient_Alarms: {
      s_vibe_score_alarms = id;
      key = PREF_KEY_VIBE_SCORE_ALARMS;
      break;
    }
    default: {
      WTF;
    }
  }
  SET_PREF(key, id);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
//! DND

void alerts_preferences_dnd_set_mask(AlertMask mask) {
  s_dnd_interruptions_mask = mask;
  SET_PREF(PREF_KEY_DND_INTERRUPTIONS_MASK, s_dnd_interruptions_mask);
}

AlertMask alerts_preferences_dnd_get_mask(void) {
  return s_dnd_interruptions_mask;
}

bool alerts_preferences_dnd_is_manually_enabled(void) {
  return s_do_not_disturb_manually_enabled;
}

void alerts_preferences_dnd_set_manually_enabled(bool enable) {
  s_do_not_disturb_manually_enabled = enable;
  SET_PREF(PREF_KEY_DND_MANUALLY_ENABLED, s_do_not_disturb_manually_enabled);
}

void alerts_preferences_dnd_get_schedule(DoNotDisturbScheduleType type,
                                         DoNotDisturbSchedule *schedule_out) {
  *schedule_out = s_dnd_schedule[type].schedule;
};

void alerts_preferences_dnd_set_schedule(DoNotDisturbScheduleType type,
                                         const DoNotDisturbSchedule *schedule) {
  s_dnd_schedule[type].schedule = *schedule;
  SET_PREF(s_dnd_schedule_keys[type].schedule_pref_key, s_dnd_schedule[type].schedule);
};

bool alerts_preferences_dnd_is_schedule_enabled(DoNotDisturbScheduleType type) {
  return s_dnd_schedule[type].enabled;
}

void alerts_preferences_dnd_set_schedule_enabled(DoNotDisturbScheduleType type, bool on) {
  s_dnd_schedule[type].enabled = on;
  SET_PREF(s_dnd_schedule_keys[type].enabled_pref_key, s_dnd_schedule[type].enabled);
}

bool alerts_preferences_check_and_set_first_use_complete(FirstUseSource source) {
  if (s_first_use_complete & (1 << source)) {
    return true;
  };

  s_first_use_complete |= (1 << source);
  SET_PREF(PREF_KEY_FIRST_USE_COMPLETE, s_first_use_complete);
  return false;
}

bool alerts_preferences_dnd_is_smart_enabled(void) {
  return s_do_not_disturb_smart_dnd_enabled;
}

void alerts_preferences_dnd_set_smart_enabled(bool enable) {
  s_do_not_disturb_smart_dnd_enabled = enable;
  SET_PREF(PREF_KEY_DND_SMART_ENABLED, s_do_not_disturb_smart_dnd_enabled);
}

bool alerts_preferences_get_delayed_vibration(void) {
  return s_delayed_vibration_enabled;
}

void alerts_preferences_set_delayed_vibration(bool enable) {
  s_delayed_vibration_enabled = enable;
  SET_PREF(PREF_KEY_DELAYED_VIBRATION, s_delayed_vibration_enabled);
}

void analytics_external_collect_alerts_preferences(void) {
  uint8_t alerts_dnd_prefs_bitmask = 0;
  alerts_dnd_prefs_bitmask |= (alerts_preferences_dnd_is_manually_enabled() << 0);
  alerts_dnd_prefs_bitmask |= (alerts_preferences_dnd_is_smart_enabled() << 1);
  alerts_dnd_prefs_bitmask |= (alerts_preferences_dnd_is_schedule_enabled(WeekdaySchedule) << 2);
  alerts_dnd_prefs_bitmask |= (alerts_preferences_dnd_is_schedule_enabled(WeekendSchedule) << 3);
  analytics_set(ANALYTICS_DEVICE_METRIC_ALERTS_DND_PREFS_BITMASK,
                alerts_dnd_prefs_bitmask, AnalyticsClient_System);
  analytics_set(ANALYTICS_DEVICE_METRIC_ALERTS_MASK,
                (uint8_t) alerts_preferences_get_alert_mask(), AnalyticsClient_System);
}
