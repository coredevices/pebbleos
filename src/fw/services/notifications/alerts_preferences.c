/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/notifications/alerts_preferences.h"
#include "pbl/services/notifications/alerts_preferences_private.h"

#include <pbl/drivers/rtc.h>
#include "pbl/services/notifications/do_not_disturb.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/vibes/vibe_intensity.h"
#include "system/passert.h"
#include "pbl/kernel/mutex.h"

#include <string.h>

#define FILE_NAME "notifpref"
#define FILE_LEN (1024)

static PBL_MUTEX_DEFINE(s_mutex);

///////////////////////////////////
//! Preference keys
///////////////////////////////////

#define PREF_KEY_MASK "mask"
static AlertMask s_mask = AlertMaskAllOn;

#define PREF_KEY_DND_INTERRUPTIONS_MASK "dndInterruptionsMask"
static AlertMask s_dnd_interruptions_mask = AlertMaskAllOff;

#define PREF_KEY_DND_SHOW_NOTIFICATIONS "dndShowNotifications"
static DndNotificationMode s_dnd_show_notifications = DndNotificationModeShow;

#define PREF_KEY_DND_MOTION_BACKLIGHT "dndMotionBacklight"
static bool s_dnd_motion_backlight = true;

#define PREF_KEY_DND_TOUCH_BACKLIGHT "dndTouchBacklight"
static bool s_dnd_touch_backlight = true;

#define PREF_KEY_DND_MUTE_SPEAKER "dndMuteSpeaker"
static bool s_dnd_mute_speaker = false;

#define PREF_KEY_DND_AUTO_DISMISS "dndAutoDismiss"
static bool s_dnd_auto_dismiss = false;

#define PREF_KEY_SPEAKER_MUTED "speakerMuted"
static bool s_speaker_muted = false;

#define PREF_KEY_SPEAKER_VOLUME "speakerVolume"
static uint8_t s_speaker_volume = 100;

#define PREF_KEY_VIBE "vibe"
static bool s_vibe_on_notification = true;

#define PREF_KEY_VIBE_INTENSITY "vibeIntensity"
static VibeIntensity s_vibe_intensity = DEFAULT_VIBE_INTENSITY;

#define PREF_KEY_VIBE_SCORE_NOTIFICATIONS ("vibeScoreNotifications")
static VibeScoreId s_vibe_score_notifications = DEFAULT_VIBE_SCORE_NOTIFS;

#define PREF_KEY_VIBE_SCORE_INCOMING_CALLS ("vibeScoreIncomingCalls")
static VibeScoreId s_vibe_score_incoming_calls = DEFAULT_VIBE_SCORE_INCOMING_CALLS;

#define PREF_KEY_VIBE_SCORE_ALARMS ("vibeScoreAlarms")
static VibeScoreId s_vibe_score_alarms = DEFAULT_VIBE_SCORE_ALARMS;

#define PREF_KEY_VIBE_SCORE_HOURLY ("vibeScoreHourly")
static VibeScoreId s_vibe_score_hourly = DEFAULT_VIBE_SCORE_HOURLY;

#define PREF_KEY_VIBE_SCORE_ON_DISCONNECT ("vibeScoreOnDisconnect")
static VibeScoreId s_vibe_score_on_disconnect = DEFAULT_VIBE_SCORE_ON_DISCONNECT;

#define PREF_KEY_DND_MANUALLY_ENABLED "dndManuallyEnabled"
static bool s_do_not_disturb_manually_enabled = false;

#define PREF_KEY_DND_SMART_ENABLED "dndSmartEnabled"
static bool s_do_not_disturb_smart_dnd_enabled = false;

#define PREF_KEY_FIRST_USE_COMPLETE "firstUseComplete"
static uint32_t s_first_use_complete = 0;

#define PREF_KEY_NOTIF_WINDOW_TIMEOUT "notifWindowTimeout"
static uint32_t s_notif_window_timeout_ms = NOTIF_WINDOW_TIMEOUT_DEFAULT;

#define PREF_KEY_NOTIF_DESIGN_STYLE "notifDesignStyle"
static bool s_notification_alternative_design = false;  // true = alternative (black banner), false = standard (default)

#define PREF_KEY_NOTIF_VIBE_DELAY "notifVibeDelay"
static bool s_notification_vibe_delay = true;  // true = vibe at end of animation (default), false = vibe immediately

#define PREF_KEY_NOTIF_BACKLIGHT "notifBacklight"
static bool s_notification_backlight = true;  // true = enable backlight (default), false = disable backlight

#define PREF_KEY_NOTIF_STATUS_BAR_STYLE "notifStatusBarStyle"
static NotificationStatusBarStyle s_notification_status_bar_style = NotificationStatusBarStyle_Default;

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

// Persist any vibe score values that were modified by migration. Skipping
// unchanged keys avoids bumping their settings_file timestamps on every boot,
// which would otherwise make the watch win every sync conflict against the
// phone and leave settings_blob_db's INSERT_WITH_TIMESTAMP path permanently
// rejecting the user's chosen value.
static void prv_save_changed_vibe_scores_to_file(SettingsFile *file,
                                                 VibeScoreId orig_notifications,
                                                 VibeScoreId orig_incoming_calls,
                                                 VibeScoreId orig_alarms,
                                                 VibeScoreId orig_hourly,
                                                 VibeScoreId orig_on_disconnect) {
#define SET_PREF_IF_CHANGED(key, value, orig) \
  do { \
    if ((value) != (orig)) { \
      settings_file_set(file, key, strlen(key), &(value), sizeof(value)); \
    } \
  } while (0)

  SET_PREF_IF_CHANGED(PREF_KEY_VIBE_SCORE_NOTIFICATIONS, s_vibe_score_notifications,
                      orig_notifications);
  SET_PREF_IF_CHANGED(PREF_KEY_VIBE_SCORE_INCOMING_CALLS, s_vibe_score_incoming_calls,
                      orig_incoming_calls);
  SET_PREF_IF_CHANGED(PREF_KEY_VIBE_SCORE_ALARMS, s_vibe_score_alarms, orig_alarms);
  SET_PREF_IF_CHANGED(PREF_KEY_VIBE_SCORE_HOURLY, s_vibe_score_hourly, orig_hourly);
  SET_PREF_IF_CHANGED(PREF_KEY_VIBE_SCORE_ON_DISCONNECT, s_vibe_score_on_disconnect, 
                      orig_on_disconnect);
#undef SET_PREF_IF_CHANGED
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
  s_vibe_score_hourly = prv_return_default_if_invalid(s_vibe_score_hourly,
                                                      DEFAULT_VIBE_SCORE_HOURLY);
  s_vibe_score_on_disconnect = prv_return_default_if_invalid(s_vibe_score_on_disconnect,
                                                            DEFAULT_VIBE_SCORE_ON_DISCONNECT);
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

void alerts_preferences_init(void) {

  SettingsFile file = {{0}};
  if (settings_file_open(&file, FILE_NAME, FILE_LEN) != S_SUCCESS) {
    return;
  }

  // The notif-prefs file may contain records keyed with strlen() (local writes)
  // or strlen()+1 (legacy phone-pushed records from before settings_blob_db
  // canonicalised the key length). Settings file lookups match key_len exactly,
  // so probe both lengths to remain backwards compatible with records written
  // by older firmware.
#define RESTORE_PREF(key, var) \
  do { \
    __typeof__(var) _tmp; \
    if (settings_file_get( \
            &file, key, strlen(key), &_tmp, sizeof(_tmp)) == S_SUCCESS || \
        settings_file_get( \
            &file, key, strlen(key) + 1, &_tmp, sizeof(_tmp)) == S_SUCCESS) { \
      var = _tmp; \
    } \
  } while (0)

  RESTORE_PREF(PREF_KEY_MASK, s_mask);
  RESTORE_PREF(PREF_KEY_VIBE, s_vibe_on_notification);
  RESTORE_PREF(PREF_KEY_VIBE_INTENSITY, s_vibe_intensity);
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_NOTIFICATIONS, s_vibe_score_notifications);
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_INCOMING_CALLS, s_vibe_score_incoming_calls);
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_ALARMS, s_vibe_score_alarms);
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_HOURLY, s_vibe_score_hourly);
  RESTORE_PREF(PREF_KEY_VIBE_SCORE_ON_DISCONNECT, s_vibe_score_on_disconnect);
  RESTORE_PREF(PREF_KEY_DND_MANUALLY_ENABLED, s_do_not_disturb_manually_enabled);
  RESTORE_PREF(PREF_KEY_DND_SMART_ENABLED, s_do_not_disturb_smart_dnd_enabled);
  RESTORE_PREF(PREF_KEY_DND_INTERRUPTIONS_MASK, s_dnd_interruptions_mask);
  RESTORE_PREF(PREF_KEY_DND_SHOW_NOTIFICATIONS, s_dnd_show_notifications);
  RESTORE_PREF(PREF_KEY_DND_MOTION_BACKLIGHT, s_dnd_motion_backlight);
  RESTORE_PREF(PREF_KEY_DND_TOUCH_BACKLIGHT, s_dnd_touch_backlight);
  RESTORE_PREF(PREF_KEY_DND_MUTE_SPEAKER, s_dnd_mute_speaker);
  RESTORE_PREF(PREF_KEY_SPEAKER_MUTED, s_speaker_muted);
  RESTORE_PREF(PREF_KEY_SPEAKER_VOLUME, s_speaker_volume);
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
  RESTORE_PREF(PREF_KEY_NOTIF_DESIGN_STYLE, s_notification_alternative_design);
  RESTORE_PREF(PREF_KEY_NOTIF_VIBE_DELAY, s_notification_vibe_delay);
  RESTORE_PREF(PREF_KEY_NOTIF_BACKLIGHT, s_notification_backlight);
  RESTORE_PREF(PREF_KEY_NOTIF_STATUS_BAR_STYLE, s_notification_status_bar_style);
  RESTORE_PREF(PREF_KEY_DND_AUTO_DISMISS, s_dnd_auto_dismiss);
#undef RESTORE_PREF

  prv_migrate_legacy_dnd_schedule(&file);

  const VibeScoreId orig_vibe_score_notifications = s_vibe_score_notifications;
  const VibeScoreId orig_vibe_score_incoming_calls = s_vibe_score_incoming_calls;
  const VibeScoreId orig_vibe_score_alarms = s_vibe_score_alarms;
  const VibeScoreId orig_vibe_score_hourly = s_vibe_score_hourly;
  const VibeScoreId orig_vibe_score_on_disconnect = s_vibe_score_on_disconnect;

  prv_migrate_legacy_first_use_settings(&file);
  prv_migrate_vibe_intensity_to_vibe_scores(&file);
  prv_ensure_valid_vibe_scores();
  // RESTORE_PREF writes straight into the globals, bypassing the setter that
  // clamps this, so an out-of-range stored value has to be caught here.
  if (s_speaker_volume > 100) {
    s_speaker_volume = 100;
  }
  prv_save_changed_vibe_scores_to_file(&file, orig_vibe_score_notifications,
                                       orig_vibe_score_incoming_calls,
                                       orig_vibe_score_alarms,
                                       orig_vibe_score_hourly,
                                       orig_vibe_score_on_disconnect);

  settings_file_close(&file);
}

// Convenience macro for setting a string key to a non-pointer value.
#define SET_PREF(key, value) \
  prv_set_pref(key, strlen(key), &value, sizeof(value))
static void prv_set_pref(const void *key, size_t key_len, const void *value,
                         size_t value_len) {
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  SettingsFile file = {{0}};
  if (settings_file_open(&file, FILE_NAME, FILE_LEN) != S_SUCCESS) {
    goto cleanup;
  }
  settings_file_set(&file, key, key_len, value, value_len);
  settings_file_close(&file);
cleanup:
  pbl_mutex_unlock(&s_mutex);
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

bool alerts_preferences_get_notification_alternative_design(void) {
  return s_notification_alternative_design;
}

void alerts_preferences_set_notification_alternative_design(bool alternative) {
  s_notification_alternative_design = alternative;
  SET_PREF(PREF_KEY_NOTIF_DESIGN_STYLE, s_notification_alternative_design);
}

bool alerts_preferences_get_notification_vibe_delay(void) {
  return s_notification_vibe_delay;
}

void alerts_preferences_set_notification_vibe_delay(bool delay) {
  s_notification_vibe_delay = delay;
  SET_PREF(PREF_KEY_NOTIF_VIBE_DELAY, s_notification_vibe_delay);
}

bool alerts_preferences_get_notification_backlight(void) {
  return s_notification_backlight;
}

void alerts_preferences_set_notification_backlight(bool enable) {
  s_notification_backlight = enable;
  SET_PREF(PREF_KEY_NOTIF_BACKLIGHT, s_notification_backlight);
}

NotificationStatusBarStyle alerts_preferences_get_notification_status_bar_style(void) {
  return s_notification_status_bar_style;
}

void alerts_preferences_set_notification_status_bar_style(NotificationStatusBarStyle style) {
  s_notification_status_bar_style = style;
  SET_PREF(PREF_KEY_NOTIF_STATUS_BAR_STYLE, s_notification_status_bar_style);
}

bool alerts_preferences_get_speaker_muted(void) {
  return s_speaker_muted;
}

void alerts_preferences_set_speaker_muted(bool muted) {
  s_speaker_muted = muted;
  SET_PREF(PREF_KEY_SPEAKER_MUTED, s_speaker_muted);
}

uint8_t alerts_preferences_get_speaker_volume(void) {
  return s_speaker_volume;
}

void alerts_preferences_set_speaker_volume(uint8_t volume) {
  if (volume > 100) {
    volume = 100;
  }
  s_speaker_volume = volume;
  SET_PREF(PREF_KEY_SPEAKER_VOLUME, s_speaker_volume);
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

VibeScoreId alerts_preferences_get_vibe_score_for_client(VibeClient client) {
  switch (client) {
    case VibeClient_Notifications:
      return s_vibe_score_notifications;
    case VibeClient_PhoneCalls:
      return s_vibe_score_incoming_calls;
    case VibeClient_Alarms:
      return s_vibe_score_alarms;
    case VibeClient_Hourly:
      return s_vibe_score_hourly;
    case VibeClient_OnDisconnect:
      return s_vibe_score_on_disconnect;
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
    case VibeClient_Hourly: {
      s_vibe_score_hourly = id;
      key = PREF_KEY_VIBE_SCORE_HOURLY;
      break;
    }
    case VibeClient_OnDisconnect: {
      s_vibe_score_on_disconnect = id;
      key = PREF_KEY_VIBE_SCORE_ON_DISCONNECT;
      break;
    }
    default: {
      WTF;
    }
  }
  SET_PREF(key, id);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! DND

void alerts_preferences_dnd_set_mask(AlertMask mask) {
  s_dnd_interruptions_mask = mask;
  SET_PREF(PREF_KEY_DND_INTERRUPTIONS_MASK, s_dnd_interruptions_mask);
}

AlertMask alerts_preferences_dnd_get_mask(void) {
  return s_dnd_interruptions_mask;
}

void alerts_preferences_dnd_set_show_notifications(DndNotificationMode mode) {
  s_dnd_show_notifications = mode;
  SET_PREF(PREF_KEY_DND_SHOW_NOTIFICATIONS, s_dnd_show_notifications);
}

DndNotificationMode alerts_preferences_dnd_get_show_notifications(void) {
  return s_dnd_show_notifications;
}

void alerts_preferences_dnd_set_motion_backlight(bool enable) {
  s_dnd_motion_backlight = enable;
  SET_PREF(PREF_KEY_DND_MOTION_BACKLIGHT, s_dnd_motion_backlight);
}

bool alerts_preferences_dnd_get_motion_backlight(void) {
  return s_dnd_motion_backlight;
}

void alerts_preferences_dnd_set_touch_backlight(bool enable) {
  s_dnd_touch_backlight = enable;
  SET_PREF(PREF_KEY_DND_TOUCH_BACKLIGHT, s_dnd_touch_backlight);
}

bool alerts_preferences_dnd_get_touch_backlight(void) {
  return s_dnd_touch_backlight;
}

void alerts_preferences_dnd_set_mute_speaker(bool enable) {
  s_dnd_mute_speaker = enable;
  SET_PREF(PREF_KEY_DND_MUTE_SPEAKER, s_dnd_mute_speaker);
}

bool alerts_preferences_dnd_get_mute_speaker(void) {
  return s_dnd_mute_speaker;
}

void alerts_preferences_dnd_set_auto_dismiss(bool enable) {
  s_dnd_auto_dismiss = enable;
  SET_PREF(PREF_KEY_DND_AUTO_DISMISS, s_dnd_auto_dismiss);
}

bool alerts_preferences_dnd_get_auto_dismiss(void) {
  return s_dnd_auto_dismiss;
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

void alerts_preferences_lock(void) {
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
}

void alerts_preferences_unlock(void) {
  pbl_mutex_unlock(&s_mutex);
}

//! Keys that feed do_not_disturb_is_active() or the DND schedule timer
static bool prv_is_dnd_state_key(const char *key) {
  if (strcmp(key, PREF_KEY_DND_MANUALLY_ENABLED) == 0 ||
      strcmp(key, PREF_KEY_DND_SMART_ENABLED) == 0) {
    return true;
  }
  for (int i = 0; i < NumDNDSchedules; i++) {
    if (strcmp(key, s_dnd_schedule_keys[i].schedule_pref_key) == 0 ||
        strcmp(key, s_dnd_schedule_keys[i].enabled_pref_key) == 0) {
      return true;
    }
  }
  return false;
}

void alerts_preferences_handle_blob_db_event(PebbleBlobDBEvent *event) {
  if (event->type != BlobDBEventTypeInsert) {
    return;
  }

  const uint8_t *key = event->key;
  int key_len = event->key_len;
  const char *matched_key = NULL;

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  SettingsFile file = {{0}};
  if (settings_file_open(&file, FILE_NAME, FILE_LEN) != S_SUCCESS) {
    pbl_mutex_unlock(&s_mutex);
    return;
  }

  // Helper macro to reload a single preference from file if key matches
  // key_len may or may not include the null terminator depending on BlobDB protocol
  // IMPORTANT: settings_file_get uses exact key_len matching, so we must use the same
  // key_len that was used when writing the record (i.e., key_len from the event)
#define RELOAD_IF_MATCH(pref_key, var) \
  do { \
    size_t _pref_strlen = strlen(pref_key); \
    if ((key_len == (int)_pref_strlen || key_len == (int)(_pref_strlen + 1)) && \
        memcmp(key, pref_key, _pref_strlen) == 0) { \
      __typeof__(var) _tmp; \
      if (settings_file_get(&file, key, key_len, &_tmp, sizeof(_tmp)) == S_SUCCESS) { \
        var = _tmp; \
        matched_key = pref_key; \
      } \
      goto done; \
    } \
  } while (0)

  RELOAD_IF_MATCH(PREF_KEY_MASK, s_mask);
  RELOAD_IF_MATCH(PREF_KEY_DND_INTERRUPTIONS_MASK, s_dnd_interruptions_mask);
  RELOAD_IF_MATCH(PREF_KEY_DND_SHOW_NOTIFICATIONS, s_dnd_show_notifications);
  RELOAD_IF_MATCH(PREF_KEY_VIBE_INTENSITY, s_vibe_intensity);
  RELOAD_IF_MATCH(PREF_KEY_VIBE_SCORE_NOTIFICATIONS, s_vibe_score_notifications);
  RELOAD_IF_MATCH(PREF_KEY_VIBE_SCORE_INCOMING_CALLS, s_vibe_score_incoming_calls);
  RELOAD_IF_MATCH(PREF_KEY_VIBE_SCORE_ALARMS, s_vibe_score_alarms);
  RELOAD_IF_MATCH(PREF_KEY_DND_MANUALLY_ENABLED, s_do_not_disturb_manually_enabled);
  RELOAD_IF_MATCH(PREF_KEY_DND_SMART_ENABLED, s_do_not_disturb_smart_dnd_enabled);
  RELOAD_IF_MATCH(s_dnd_schedule_keys[WeekdaySchedule].schedule_pref_key,
                  s_dnd_schedule[WeekdaySchedule].schedule);
  RELOAD_IF_MATCH(s_dnd_schedule_keys[WeekdaySchedule].enabled_pref_key,
                  s_dnd_schedule[WeekdaySchedule].enabled);
  RELOAD_IF_MATCH(s_dnd_schedule_keys[WeekendSchedule].schedule_pref_key,
                  s_dnd_schedule[WeekendSchedule].schedule);
  RELOAD_IF_MATCH(s_dnd_schedule_keys[WeekendSchedule].enabled_pref_key,
                  s_dnd_schedule[WeekendSchedule].enabled);
  RELOAD_IF_MATCH(PREF_KEY_NOTIF_WINDOW_TIMEOUT, s_notif_window_timeout_ms);
  RELOAD_IF_MATCH(PREF_KEY_NOTIF_DESIGN_STYLE, s_notification_alternative_design);
  RELOAD_IF_MATCH(PREF_KEY_NOTIF_VIBE_DELAY, s_notification_vibe_delay);
  RELOAD_IF_MATCH(PREF_KEY_NOTIF_BACKLIGHT, s_notification_backlight);
  RELOAD_IF_MATCH(PREF_KEY_NOTIF_STATUS_BAR_STYLE, s_notification_status_bar_style);
  RELOAD_IF_MATCH(PREF_KEY_DND_MOTION_BACKLIGHT, s_dnd_motion_backlight);
  RELOAD_IF_MATCH(PREF_KEY_DND_TOUCH_BACKLIGHT, s_dnd_touch_backlight);
  RELOAD_IF_MATCH(PREF_KEY_DND_MUTE_SPEAKER, s_dnd_mute_speaker);
  RELOAD_IF_MATCH(PREF_KEY_SPEAKER_MUTED, s_speaker_muted);
  RELOAD_IF_MATCH(PREF_KEY_DND_AUTO_DISMISS, s_dnd_auto_dismiss);
  RELOAD_IF_MATCH(PREF_KEY_SPEAKER_VOLUME, s_speaker_volume);

#undef RELOAD_IF_MATCH

done:
  settings_file_close(&file);
  pbl_mutex_unlock(&s_mutex);

  if (matched_key) {
    // DND state keys are reloaded behind the DND service's back; kick it so the
    // change re-arms the schedule timer and fires PEBBLE_DO_NOT_DISTURB_EVENT.
    if (prv_is_dnd_state_key(matched_key)) {
      do_not_disturb_handle_pref_synced();
    }
    PebbleEvent pref_event = {
      .type = PEBBLE_PREF_CHANGE_EVENT,
      .pref_change = {
        .key = matched_key,
        .key_len = strlen(matched_key) + 1,
      },
    };
    event_put(&pref_event);
  }
}