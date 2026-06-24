/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/preferred_content_size.h"
#include "apps/system_app_ids.h"
#include "board/board.h"
#include "pbl/os/mutex.h"
#include "process_management/app_install_manager.h"
#include "process_management/process_manager.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_insights.h"
#include "pbl/services/activity/insights_settings.h"
#include "pbl/services/settings/settings_file.h"
#include "shell/prefs.h"
#include "shell/prefs_private.h"

static PebbleMutex *s_mutex;

#define PREF_KEY_CLOCK_24H "clock24h"
static bool s_is_24h_style;

#define PREF_KEY_DEFAULT_WATCHFACE "watchface"
static Uuid s_default_watchface = UUID_INVALID_INIT;

#define PREF_KEY_CONTENT_SIZE "contentSize"
static uint8_t s_content_size;
#if !UNITTEST
_Static_assert(sizeof(PreferredContentSize) == sizeof(s_content_size),
               "sizeof(PreferredContentSize) grew, pref needs to be migrated!");
#endif

#ifdef CONFIG_APP_SCALING
#define PREF_KEY_LEGACY_APP_RENDER_MODE "legacyAppRenderMode"
static uint8_t s_legacy_app_render_mode = 1; // Default to scaled mode
#endif

void shell_prefs_init(void) {
  s_mutex = mutex_create();
  mutex_lock(s_mutex);
  SettingsFile file = {};
  if (settings_file_open(&file, SHELL_PREFS_FILE_NAME, SHELL_PREFS_FILE_LEN) != S_SUCCESS) {
    goto cleanup;
  }
  if (settings_file_get(&file, PREF_KEY_CLOCK_24H, sizeof(PREF_KEY_CLOCK_24H),
                        &s_is_24h_style, sizeof(s_is_24h_style)) != S_SUCCESS) {
    // The setting likely doesn't exist yet so set it to the default (true)
    s_is_24h_style = true;
  }
  if (settings_file_get(&file, PREF_KEY_DEFAULT_WATCHFACE, sizeof(PREF_KEY_DEFAULT_WATCHFACE),
                        &s_default_watchface, sizeof(s_default_watchface)) != S_SUCCESS) {
    s_default_watchface = UUID_INVALID;
  }
  if (settings_file_get(&file, PREF_KEY_CONTENT_SIZE, sizeof(PREF_KEY_CONTENT_SIZE),
                        &s_content_size, sizeof(s_content_size)) != S_SUCCESS) {
    s_content_size = PreferredContentSizeDefault;
  }
  settings_file_close(&file);
cleanup:
  mutex_unlock(s_mutex);
}

static bool prv_pref_set(const char *key, const void *val, size_t val_len) {
  SettingsFile file = {};
  status_t rv;
  if ((rv = settings_file_open(&file, SHELL_PREFS_FILE_NAME, SHELL_PREFS_FILE_LEN)) != S_SUCCESS) {
    goto cleanup;
  }
  rv = settings_file_set(&file, key, strnlen(key, SETTINGS_KEY_MAX_LEN) + 1, val, val_len);
  settings_file_close(&file);
cleanup:
  return (rv == S_SUCCESS);
}

bool shell_prefs_get_clock_24h_style(void) {
  return s_is_24h_style;
}

void shell_prefs_set_clock_24h_style(bool is_24h_style) {
  mutex_lock(s_mutex);
  if (prv_pref_set(PREF_KEY_CLOCK_24H, &s_is_24h_style, sizeof(s_is_24h_style))) {
    s_is_24h_style = is_24h_style;
  }
  mutex_unlock(s_mutex);
}

bool shell_prefs_is_timezone_source_manual(void) {
  // Force things to automatic
  return false;
}

void shell_prefs_set_timezone_source_manual(bool manual) {
}

bool shell_prefs_is_time_source_manual(void) {
  // Force things to automatic
  return false;
}

void shell_prefs_set_time_source_manual(bool manual) {
}

int16_t shell_prefs_get_automatic_timezone_id(void) {
  // Invalid
  return -1;
}

void shell_prefs_set_automatic_timezone_id(int16_t timezone_id) {
}


void prefs_private_lock(void) {
  mutex_lock(s_mutex);
}

void prefs_private_unlock(void) {
  mutex_unlock(s_mutex);
}

// Exported function used by blob_db API to set the backing store for a specific key.
// Not used by the SDK shell
bool prefs_private_write_backing(const uint8_t *key, size_t key_len, const void *value,
                               int value_len) {
  return false;
}


// Exported function used by blob_db API to get the length of a value in our backing store
// Not used by the SDK shell
int prefs_private_get_backing_len(const uint8_t *key, size_t key_len) {
  return 0;
}


// Exported function used by blob_db API to read our backing store
// Not used by the SDK shell
bool prefs_private_read_backing(const uint8_t *key, size_t key_len, void *value, int value_len) {
  return false;
}

void watchface_set_default_install_id(AppInstallId app_id) {
  mutex_lock(s_mutex);
  Uuid uuid;
  app_install_get_uuid_for_install_id(app_id, &uuid);
  if (prv_pref_set(PREF_KEY_DEFAULT_WATCHFACE, &uuid, sizeof(uuid))) {
    s_default_watchface = uuid;
  }
  mutex_unlock(s_mutex);
}

static bool prv_set_default_any_watchface_enumerate_callback(AppInstallEntry *entry, void *data) {
  if (!app_install_entry_is_watchface(entry) ||
      app_install_entry_is_hidden(entry)) {
    return true; // continue search
  }

  watchface_set_default_install_id(entry->install_id);
  return false;
}

AppInstallId watchface_get_default_install_id(void) {
  AppInstallId app_id = app_install_get_id_for_uuid(&s_default_watchface);
  AppInstallEntry entry;
  if ((app_id == INSTALL_ID_INVALID) ||
      !app_install_get_entry_for_install_id(app_id, &entry) ||
      !app_install_entry_is_watchface(&entry)) {
    app_install_enumerate_entries(prv_set_default_any_watchface_enumerate_callback, NULL);
    app_id = app_install_get_id_for_uuid(&s_default_watchface);
  }
  return app_id;
}

void system_theme_set_content_size(PreferredContentSize content_size) {
  mutex_lock(s_mutex);
  const uint8_t content_size_uint = content_size;
  if (content_size >= NumPreferredContentSizes) {
    PBL_LOG_WRN("Ignoring attempt to set content size to invalid size %d",
            content_size);
  } else if (prv_pref_set(PREF_KEY_CONTENT_SIZE, &content_size_uint, sizeof(content_size_uint))) {
    s_content_size = content_size;
  }
  mutex_unlock(s_mutex);
}

PreferredContentSize system_theme_get_content_size(void) {
  return system_theme_convert_host_content_size_to_runtime_platform(
      (PreferredContentSize)s_content_size);
}

bool activity_prefs_tracking_is_enabled(void) {
  return true;
}

void activity_prefs_tracking_set_enabled(bool enable) {
}

bool activity_prefs_activity_insights_are_enabled(void) {
  return false;
}

void activity_prefs_activity_insights_set_enabled(bool enable) {
}

bool activity_prefs_sleep_insights_are_enabled(void) {
  return false;
}

void activity_prefs_sleep_insights_set_enabled(bool enable) {
}

uint8_t activity_prefs_get_health_app_opened_version(void) {
  return 0;
}

void activity_prefs_set_height_mm(uint16_t height_mm) {
}

uint16_t activity_prefs_get_height_mm(void) {
  return 0;
}

void activity_prefs_set_weight_dag(uint16_t weight_dag) {
}

uint16_t activity_prefs_get_weight_dag(void) {
  return 0;
}

void activity_prefs_set_gender(ActivityGender gender) {
}

ActivityGender activity_prefs_get_gender(void) {
  return ActivityGenderOther;
}

void activity_prefs_set_age_years(uint8_t age_years) {
}

uint8_t activity_prefs_get_age_years(void) {
  return 0;
}

bool activity_prefs_heart_rate_is_enabled(void) {
  return true;
}

#ifdef CONFIG_HRM
HRMonitoringInterval activity_prefs_get_hrm_measurement_interval(void) {
  return HRMonitoringInterval_10Min;
}

void activity_prefs_set_hrm_measurement_interval(HRMonitoringInterval interval) {
}

bool activity_prefs_hrm_activity_tracking_is_enabled(void) {
  return true;
}

void activity_prefs_set_hrm_activity_tracking_enabled(bool enabled) {
}

bool activity_prefs_ble_hrm_sharing_is_enabled(void) {
  return false;
}

void activity_prefs_ble_hrm_sharing_set_enabled(bool enabled) {
}
#endif

ActivityInsightSettings *activity_prefs_get_sleep_reward_settings(void) {
  static ActivityInsightSettings s_settings = { 0 };
  return &s_settings;
}

void activity_prefs_set_activation_delay_insight_fired(ActivationDelayInsightType type) {
}

bool activity_prefs_has_activation_delay_insight_fired(ActivationDelayInsightType type) {
  return false;
}

bool activity_prefs_get_health_app_opened(void) {
  return false;
}

void activity_prefs_set_activated(void) {
}

time_t activity_prefs_get_activation_time(void) {
  return 0;
}

UnitsDistance shell_prefs_get_units_distance(void) {
  return UnitsDistance_Miles;
}

GColor shell_prefs_get_theme_highlight_color(void) {
  return PBL_IF_COLOR_ELSE(GColorVividCerulean, GColorBlack);
}

void shell_prefs_set_theme_highlight_color(GColor color) {
  // Not used in SDK shell
}

#ifdef CONFIG_APP_SCALING
LegacyAppRenderMode shell_prefs_get_legacy_app_render_mode(void) {
  return (LegacyAppRenderMode)s_legacy_app_render_mode;
}

void shell_prefs_set_legacy_app_render_mode(LegacyAppRenderMode mode) {
  uint8_t mode_value = (uint8_t)mode;
  prv_pref_set(PREF_KEY_LEGACY_APP_RENDER_MODE, &mode_value, sizeof(mode_value));
}
#endif

// Exported function used by blob_db API to handle settings events
// Not used by the SDK shell
void prefs_private_handle_blob_db_event(PebbleBlobDBEvent *event) {

}

bool shell_prefs_get_menu_scroll_wrap_around_enable(void) {
  return false;
}

void shell_prefs_set_menu_scroll_wrap_around_enable(bool enable) {
  // Not used in SDK shell
}

MenuScrollVibeBehavior shell_prefs_get_menu_scroll_vibe_behavior(void) {
  return MenuScrollNoVibe;
}

void shell_prefs_set_menu_scroll_vibe_behavior(MenuScrollVibeBehavior behavior) {
  // Not used in SDK shell
}

bool shell_prefs_get_vibe_log_info_enabled(void) {
  return false;
}

void shell_prefs_set_vibe_log_info_enabled(bool enabled) {
  // Not used in SDK shell
}

bool shell_prefs_get_settings_dbs_compacted_v1(void) {
  return true;
}

void shell_prefs_set_settings_dbs_compacted_v1(bool done) {
  // Not used in SDK shell
}
