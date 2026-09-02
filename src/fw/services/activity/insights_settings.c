/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <inttypes.h>
#include <string.h>

#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/insights_settings.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include <pbl/logging/logging.h>
#include "pbl/util/size.h"

PBL_LOG_MODULE_DECLARE(service_activity, CONFIG_SERVICE_ACTIVITY_LOG_LEVEL);

#define ACTIVITY_INSIGHTS_SETTINGS_FILENAME "insights"
#define ACTIVITY_INSIGHTS_SETTINGS_DEFAULT_FILE_SIZE 4096


#define ACTIVITY_INSIGHTS_SETTINGS_VERSION_KEY "version"
#define ACTIVITY_INSIGHTS_SETTINGS_DEFAULT_VERSION 0

#define ACTIVITY_INSIGHTS_SETTINGS_CURRENT_STRUCT_VERSION 4

static PBL_MUTEX_DEFINE(s_insight_settings_mutex);

#define ACTIVITY_INSIGHTS_SETTINGS_SLEEP_REWARD_DEFAULT { \
  .version = ACTIVITY_INSIGHTS_SETTINGS_CURRENT_STRUCT_VERSION, \
  .enabled = false, \
  .reward = { \
    .min_days_data = 6, \
    .continuous_min_days_data = 2, \
    .target_qualifying_days = 2, \
    .target_percent_of_median = 120, \
    .notif_min_interval_seconds = 7 * SECONDS_PER_DAY, \
    .sleep.trigger_after_wakeup_seconds = 2 * SECONDS_PER_HOUR \
  } \
}

#define ACTIVITY_INSIGHTS_SETTINGS_SLEEP_SUMMARY_DEFAULT { \
  .version = ACTIVITY_INSIGHTS_SETTINGS_CURRENT_STRUCT_VERSION, \
  .enabled = true, \
  .summary = { \
    .above_avg_threshold = 10, \
    .below_avg_threshold = -10, \
    .fail_threshold = -50, \
    .sleep = { \
      .max_fail_minutes = 7 * MINUTES_PER_HOUR, \
      .trigger_notif_seconds = 30 * SECONDS_PER_MINUTE, \
      .trigger_notif_activity = 10, \
      .trigger_notif_active_minutes = 2 \
    } \
  } \
}

#define ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_REWARD_DEFAULT { \
  .version = ACTIVITY_INSIGHTS_SETTINGS_CURRENT_STRUCT_VERSION, \
  .enabled = false, \
  .reward = {\
    .min_days_data = 6, \
    .continuous_min_days_data = 0, \
    .target_qualifying_days = 0, \
    .target_percent_of_median = 150, \
    .notif_min_interval_seconds = 1 * SECONDS_PER_DAY, \
    .activity = { \
      .trigger_active_minutes = 2, \
      .trigger_steps_per_minute = 50 \
    } \
  } \
}

#define ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SUMMARY_DEFAULT { \
  .version = ACTIVITY_INSIGHTS_SETTINGS_CURRENT_STRUCT_VERSION, \
  .enabled = true, \
  .summary = { \
    .above_avg_threshold = 10, \
    .below_avg_threshold = -10, \
    .fail_threshold = -50, \
    .activity = { \
      .trigger_minute = (20 * MINUTES_PER_HOUR) + 30, \
      .update_threshold_steps = 1000, \
      .update_max_interval_seconds = 30 * SECONDS_PER_MINUTE, \
      .show_notification = true, \
      .max_fail_steps = 10000, \
    } \
  } \
}

#define ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SESSION_DEFAULT { \
  .version = ACTIVITY_INSIGHTS_SETTINGS_CURRENT_STRUCT_VERSION, \
  .enabled = true, \
  .session = { \
    .show_notification = true, \
    .activity = { \
      .trigger_elapsed_minutes = 20, \
      .trigger_cooldown_minutes = 10, \
    }, \
  } \
}

typedef struct {
  const char *key;
  ActivityInsightSettings default_val;
} AISDefault;

static const AISDefault AIS_DEFAULTS[] = {
  {
    .key = ACTIVITY_INSIGHTS_SETTINGS_SLEEP_REWARD,
    .default_val = ACTIVITY_INSIGHTS_SETTINGS_SLEEP_REWARD_DEFAULT
  },
  {
    .key = ACTIVITY_INSIGHTS_SETTINGS_SLEEP_SUMMARY,
    .default_val = ACTIVITY_INSIGHTS_SETTINGS_SLEEP_SUMMARY_DEFAULT
  },
  {
    .key = ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_REWARD,
    .default_val = ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_REWARD_DEFAULT
  },
  {
    .key = ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SUMMARY,
    .default_val = ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SUMMARY_DEFAULT
  },
  {
    .key = ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SESSION,
    .default_val = ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SESSION_DEFAULT
  },
};

// Return true if we successfully opened the file
static bool prv_open_settings_and_lock(SettingsFile *file) {
  pbl_mutex_lock(&s_insight_settings_mutex, PBL_FOREVER);
  if (settings_file_open(file, ACTIVITY_INSIGHTS_SETTINGS_FILENAME,
                         ACTIVITY_INSIGHTS_SETTINGS_DEFAULT_FILE_SIZE) == S_SUCCESS) {
    return true;
  } else {
    pbl_mutex_unlock(&s_insight_settings_mutex);
    return false;
  }
}

// Close the settings file and release the lock
static void prv_close_settings_and_unlock(SettingsFile *file) {
  settings_file_close(file);
  pbl_mutex_unlock(&s_insight_settings_mutex);
}

void activity_insights_settings_init(void) {
  if (!activity_is_initialized()) {
    return;
  }
  // Create our mutex

  SettingsFile file;
  if (settings_file_open(&file,
                         ACTIVITY_INSIGHTS_SETTINGS_FILENAME,
                         ACTIVITY_INSIGHTS_SETTINGS_DEFAULT_FILE_SIZE) == S_SUCCESS) {
    if (!settings_file_exists(&file,
                              ACTIVITY_INSIGHTS_SETTINGS_VERSION_KEY,
                              strlen(ACTIVITY_INSIGHTS_SETTINGS_VERSION_KEY))) {
      // init version to 0
      const uint16_t default_version = ACTIVITY_INSIGHTS_SETTINGS_DEFAULT_VERSION;
      settings_file_set(&file,
          ACTIVITY_INSIGHTS_SETTINGS_VERSION_KEY,
          strlen(ACTIVITY_INSIGHTS_SETTINGS_VERSION_KEY),
          &default_version,
          sizeof(uint16_t));
    }

    settings_file_close(&file);
    return;
  }

  PBL_LOG_ERR("Failed to create activity insights settings file");
}

uint16_t activity_insights_settings_get_version(void) {
  if (!activity_is_initialized()) {
    return ACTIVITY_INSIGHTS_SETTINGS_DEFAULT_VERSION;
  }
  uint16_t version = ACTIVITY_INSIGHTS_SETTINGS_DEFAULT_VERSION;
  SettingsFile file;
  if (prv_open_settings_and_lock(&file)) {
    settings_file_get(&file,
                      ACTIVITY_INSIGHTS_SETTINGS_VERSION_KEY,
                      strlen(ACTIVITY_INSIGHTS_SETTINGS_VERSION_KEY),
                      &version,
                      sizeof(uint16_t));
    prv_close_settings_and_unlock(&file);
  }
  return version;
}

bool activity_insights_settings_read(const char *insight_name,
                                     ActivityInsightSettings *settings_out) {
  if (!activity_is_initialized()) {
    return false;
  }
  bool rv = false;
  *settings_out = (ActivityInsightSettings) {};

  SettingsFile file;
  if (prv_open_settings_and_lock(&file)) {
    if (settings_file_get(&file,
                          insight_name, strlen(insight_name),
                          settings_out, sizeof(*settings_out)) != S_SUCCESS) {
      PBL_LOG_DBG("Didn't find insight with key %s", insight_name);
      goto close;
    }

    if (settings_out->version != ACTIVITY_INSIGHTS_SETTINGS_CURRENT_STRUCT_VERSION) {
      // versions don't match, bail out!
      PBL_LOG_WRN("activity insights struct version mismatch");
      goto close;
    }

    rv = true;
close:
    prv_close_settings_and_unlock(&file);
  }

  if (!rv) {
    // Use default value if we didn't find anything else
    for (unsigned i = 0; i < ARRAY_LENGTH(AIS_DEFAULTS); ++i) {
      if (strcmp(insight_name, AIS_DEFAULTS[i].key) == 0) {
        PBL_LOG_DBG("Using default for insight %s", insight_name);
        *settings_out = AIS_DEFAULTS[i].default_val;
        rv = true;
      }
    }
  }
  return rv;
}

bool activity_insights_settings_write(const char *insight_name,
                                      ActivityInsightSettings *settings) {
  if (!activity_is_initialized()) {
    return false;
  }
  bool rv = false;

  SettingsFile file;
  if (prv_open_settings_and_lock(&file)) {
    if (settings_file_set(&file,
                          insight_name, strlen(insight_name),
                          settings, sizeof(*settings)) != S_SUCCESS) {
      PBL_LOG_WRN("Unable to save insight setting with key %s", insight_name);
    } else {
      rv = true;
    }
    prv_close_settings_and_unlock(&file);
  }
  return rv;
}

PFSCallbackHandle activity_insights_settings_watch(PFSFileChangedCallback callback) {
  if (!activity_is_initialized()) {
    return 0; // Return invalid handle
  }
  return pfs_watch_file(ACTIVITY_INSIGHTS_SETTINGS_FILENAME, callback, FILE_CHANGED_EVENT_CLOSED,
                        NULL);
}

void activity_insights_settings_unwatch(PFSCallbackHandle cb_handle) {
  if (!activity_is_initialized()) {
    return;
  }
  pfs_unwatch_file(cb_handle);
}
