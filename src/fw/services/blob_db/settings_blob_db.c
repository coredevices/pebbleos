/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/settings_blob_db.h"
#include "pbl/services/blob_db/sync.h"
#include "pbl/services/blob_db/api.h"

#include "kernel/pbl_malloc.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/notifications/alerts_preferences_private.h"
#include "pbl/services/settings/settings_file.h"
#include "shell/prefs.h"
#include "shell/prefs_private.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/services/system_task.h"
#include "pbl/util/list.h"
#include "pbl/util/size.h"

#include <string.h>

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

//! Notification preferences file name and size
#define NOTIF_PREFS_FILE_NAME "notifpref"
#define NOTIF_PREFS_FILE_LEN (1024)

//! Flag to suppress change callback during phone-originated INSERTs.
//! This prevents queuing unnecessary sync callbacks that would flood the system task queue.
static bool s_suppress_change_callback = false;

//! Flag to track if a sync callback is already pending.
//! This coalesces multiple setting changes into a single sync callback.
static bool s_sync_callback_pending = false;

//! Settings Whitelist
//!
//! Only these settings will be synced via BlobDB.
//! This prevents sensitive data (Bluetooth pairing, debug flags, etc.) from syncing.
static const char *s_syncable_settings[] = {
  // Clock preferences
  "clock24h",
  "timezoneSource",
  "automaticTimezoneID",

  // Display preferences
  "unitsDistance",
  "textStyle",
  "stationaryMode",
#ifdef CONFIG_ORIENTATION_MANAGER
  "displayOrientationLeftHanded",
#endif

  // Motion preferences
#ifdef CONFIG_ACCEL_SENSITIVITY
  "motionSensitivity",
#endif

  // Backlight preferences
  "lightEnabled",
  "lightAmbientSensorEnabled",
  "lightTimeoutMs",
  "lightIntensity",
  "lightMotion",
  "lightTouch",
  "lightAmbientThreshold",
#ifdef CONFIG_DYNAMIC_BACKLIGHT
  "lightDynamicMode",
#endif
  "lightPreset",
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  "lightColor",
#endif

  // Language preferences
  "langEnglish",
  "language",

  // App preferences
  "qlUp",
  "qlDown",
  "qlSelect",
  "qlBack",
  "qlSetupOpened",
  "qlSingleClickUp",
  "qlSingleClickDown",
  "qlComboBackUp",
  "qlComboUpDown",

  // UI theming
  "settingsMenuHighlightColor",
  "appsMenuHighlightColor",

  // Timeline preferences
  "timelineQuickViewEnabled",
  "timelineQuickViewBeforeTimeMin",
#if TIMELINE_PEEK_WATCHFACE_FIT_SUPPORTED
  "timelineQuickViewWatchfaceFit",
#endif
  "timelineSettingsOpened",

  // Activity preferences
  "activityPreferences",
  "activityHealthAppOpened",
  "activityWorkoutAppOpened",
  "alarmsAppOpened",
  "hrmPreferences",
  "heartRatePreferences",

  // Menu wrap around preferences
  "menuScrollWrapAround",
  "menuScrollVibeBehavior",

  // Worker preferences
  "workerId",

  // Music preferences
  "musicShowVolumeControls",
  "musicShowProgressBar",
  "musicShowAlbumArt",
};

static const size_t s_num_syncable_settings = ARRAY_LENGTH(s_syncable_settings);

//! Notification preferences from notifpref file that should be synced
static const char *s_syncable_notif_prefs[] = {
  "mask",
  "dndInterruptionsMask",
  "dndShowNotifications",
  "vibeIntensity",
  "vibeScoreNotifications",
  "vibeScoreIncomingCalls",
  "vibeScoreAlarms",
  "dndManuallyEnabled",
  "dndSmartEnabled",
  "dndWeekdaySchedule",
  "dndWeekdayScheduleEnabled",
  "dndWeekendSchedule",
  "dndWeekendScheduleEnabled",
  "notifWindowTimeout",
  "notifDesignStyle",
  "notifVibeDelay",
  "notifBacklight",
  "dndMotionBacklight",
  "dndTouchBacklight",
  "dndAutoDismiss",
};

static const size_t s_num_syncable_notif_prefs = ARRAY_LENGTH(s_syncable_notif_prefs);

static bool s_initialized = false;

//! Check if the connected phone supports Settings BlobDB sync
//! Returns true if the phone advertises settings_sync_support capability
bool settings_blob_db_phone_supports_sync(void) {
  PebbleProtocolCapabilities capabilities;
  bt_persistent_storage_get_cached_system_capabilities(&capabilities);
  return capabilities.settings_sync_support;
}

//! Find a key's canonical whitelist entry, or NULL if it isn't in the list.
//! Handles both key_len with null terminator (strlen+1) and without (strlen)
static const char *prv_find_in_list(const uint8_t *key, int key_len,
                                    const char **list, size_t list_len) {
  for (size_t i = 0; i < list_len; i++) {
    const char *list_key = list[i];
    size_t list_key_strlen = strlen(list_key);
    // Accept key_len matching strlen (no null) or strlen+1 (with null)
    if ((key_len == (int)list_key_strlen || key_len == (int)(list_key_strlen + 1)) &&
        memcmp(key, list_key, list_key_strlen) == 0) {
      return list_key;
    }
  }
  return NULL;
}

//! Check if a key matches an entry in a given list
static bool prv_is_key_in_list(const uint8_t *key, int key_len,
                               const char **list, size_t list_len) {
  return prv_find_in_list(key, key_len, list, list_len) != NULL;
}

//! Canonicalize a whitelisted shell-pref key to the form the backing store uses: shell-pref keys
//! are stored WITH the trailing null (prefs_private_read_backing looks them up with strlen+1),
//! while the phone may send the key either way. Settings-file records match on exact key_len, so
//! without this a null-less phone write lands under a key the pref readers never find and the
//! setting silently fails to apply. Points the caller at the whitelist's own string so the null
//! byte is guaranteed to be present. Returns false if the key isn't a whitelisted shell pref.
static bool prv_canonical_shell_key(const uint8_t **key, int *key_len) {
  const char *canonical = prv_find_in_list(*key, *key_len, s_syncable_settings,
                                           s_num_syncable_settings);
  if (!canonical) {
    return false;
  }
  *key = (const uint8_t *)canonical;
  *key_len = (int)strlen(canonical) + 1;
  return true;
}

//! Check if a setting key is in the shell/prefs sync whitelist
static bool prv_is_shell_pref(const uint8_t *key, int key_len) {
  return prv_is_key_in_list(key, key_len, s_syncable_settings, s_num_syncable_settings);
}

//! Check if a setting key is in the notifpref sync whitelist
static bool prv_is_notif_pref(const uint8_t *key, int key_len) {
  return prv_is_key_in_list(key, key_len, s_syncable_notif_prefs, s_num_syncable_notif_prefs);
}

//! Check if a setting key is syncable (either shell pref or notif pref)
static bool prv_is_syncable(const uint8_t *key, int key_len) {
  return prv_is_shell_pref(key, key_len) || prv_is_notif_pref(key, key_len);
}

//! Normalize key_len for notif-pref writes by stripping a trailing null.
//!
//! alerts_preferences (the local owner of the notif-prefs file) uses strlen()
//! without a trailing null when reading and writing. The phone-side BlobDB
//! protocol historically sends keys with a trailing null (key_len = strlen+1).
//! Settings file records are matched on exact key_len, so a phone write
//! creates a record that alerts_preferences_init's RESTORE_PREF cannot find,
//! causing it to fall back to the static default at boot. Normalizing the
//! incoming key_len makes phone-originated writes land at the same canonical
//! key length the local readers use.
static int prv_canonical_notif_key_len(const uint8_t *key, int key_len) {
  if (key_len > 0 && key[key_len - 1] == '\0') {
    return key_len - 1;
  }
  return key_len;
}

//! Lock the appropriate mutex for file access
static void prv_lock_for_file(bool is_notif) {
  if (is_notif) {
    alerts_preferences_lock();
  } else {
    prefs_private_lock();
  }
}

//! Unlock the appropriate mutex for file access
static void prv_unlock_for_file(bool is_notif) {
  if (is_notif) {
    alerts_preferences_unlock();
  } else {
    prefs_private_unlock();
  }
}

//! Kernel background callback to sync all dirty settings.
//! This is coalesced - only one instance runs at a time.
static void prv_deferred_sync_callback(void *data) {
  s_sync_callback_pending = false;

  // Only sync if we have an active connection to the phone
  if (!comm_session_get_system_session()) {
    PBL_LOG_DBG("No connection to phone, skipping settings sync");
    return;
  }

  // Only sync if the phone supports settings sync
  if (!settings_blob_db_phone_supports_sync()) {
    return;
  }

  // Sync all dirty settings using the existing dirty list mechanism
  blob_db_sync_db(BlobDBIdSettings);
}

//! Callback for settings changes - defers sync to avoid re-entrancy
//! The callback is invoked while the settings file is still open, so we can't
//! immediately sync (which would try to open the file again). Instead, we defer
//! the sync to run after the file operations complete.
//!
//! This callback is coalesced - multiple setting changes result in at most one
//! queued callback, preventing system task queue overflow.
static void prv_settings_change_callback(SettingsFile *file, const void *key, int key_len,
                                         time_t last_modified) {
  // Skip callback if suppressed (during phone-originated INSERTs)
  if (s_suppress_change_callback) {
    return;
  }

  // Only sync whitelisted settings
  if (!prv_is_syncable((const uint8_t *)key, key_len)) {
    return;
  }

  // Coalesce multiple changes into a single callback to avoid queue overflow
  if (s_sync_callback_pending) {
    return;
  }

  s_sync_callback_pending = true;
  system_task_add_callback(prv_deferred_sync_callback, NULL);
}

// BlobDB Interface Implementation

void settings_blob_db_init(void) {
  if (s_initialized) {
    return;
  }

  // Register callback to sync settings immediately when they change
  settings_file_set_change_callback(prv_settings_change_callback);

  s_initialized = true;
  PBL_LOG_DBG("Settings BlobDB initialized (%u whitelisted settings)",
          (unsigned int) s_num_syncable_settings);
}

status_t settings_blob_db_insert(const uint8_t *key, int key_len,
                                 const uint8_t *val, int val_len) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // Determine which file to use based on key type
  const char *file_name;
  int file_len;
  bool is_notif_pref = prv_is_notif_pref(key, key_len);

  if (is_notif_pref) {
    file_name = NOTIF_PREFS_FILE_NAME;
    file_len = NOTIF_PREFS_FILE_LEN;
    key_len = prv_canonical_notif_key_len(key, key_len);
  } else if (prv_canonical_shell_key(&key, &key_len)) {
    file_name = SHELL_PREFS_FILE_NAME;
    file_len = SHELL_PREFS_FILE_LEN;
  } else {
    char key_str[128];
    size_t copy_len = (key_len > 0 && (size_t)key_len < sizeof(key_str)) ?
                      (size_t)key_len : sizeof(key_str) - 1;
    memcpy(key_str, key, copy_len);
    key_str[copy_len] = '\0';
    PBL_LOG_WRN("Rejecting non-whitelisted setting: %s", key_str);
    return E_INVALID_OPERATION;
  }

  prv_lock_for_file(is_notif_pref);

  SettingsFile file;
  status_t status = settings_file_open(&file, file_name, file_len);
  if (FAILED(status)) {
    prv_unlock_for_file(is_notif_pref);
    return status;
  }

  // Suppress change callback - we don't want to sync back to phone for phone-originated INSERTs.
  // Keep suppressed through prefs_private_handle_blob_db_event() because validation failures
  // can trigger prefs_private_write_backing() which would fire the callback again.
  s_suppress_change_callback = true;
  status = settings_file_set(&file, key, key_len, val, val_len);

  if (PASSED(status)) {
    // Mark as synced - the record came from the phone so it's already in sync
    settings_file_mark_synced(&file, key, key_len);
  }
  settings_file_close(&file);

  // Unlock before calling event handlers - they acquire the same mutex internally
  prv_unlock_for_file(is_notif_pref);

  // Update the in-memory prefs state after successful write
  if (PASSED(status)) {
    PebbleBlobDBEvent event = {
      .db_id = BlobDBIdSettings,
      .type = BlobDBEventTypeInsert,
      .key = (uint8_t *)key,
      .key_len = key_len,
    };
    if (is_notif_pref) {
      alerts_preferences_handle_blob_db_event(&event);
    } else {
      prefs_private_handle_blob_db_event(&event);
    }
  }

  s_suppress_change_callback = false;
  return status;
}

int settings_blob_db_get_len(const uint8_t *key, int key_len) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // Determine which file to use based on key type
  const char *file_name;
  int file_len;
  bool is_notif = prv_is_notif_pref(key, key_len);
  if (is_notif) {
    file_name = NOTIF_PREFS_FILE_NAME;
    file_len = NOTIF_PREFS_FILE_LEN;
    key_len = prv_canonical_notif_key_len(key, key_len);
  } else {
    file_name = SHELL_PREFS_FILE_NAME;
    file_len = SHELL_PREFS_FILE_LEN;
    prv_canonical_shell_key(&key, &key_len);
  }

  prv_lock_for_file(is_notif);

  SettingsFile file;
  status_t status = settings_file_open(&file, file_name, file_len);
  if (FAILED(status)) {
    prv_unlock_for_file(is_notif);
    return status;
  }

  int len = settings_file_get_len(&file, key, key_len);
  settings_file_close(&file);

  prv_unlock_for_file(is_notif);
  return len;
}

status_t settings_blob_db_read(const uint8_t *key, int key_len,
                               uint8_t *val_out, int val_len) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // Determine which file to use based on key type
  const char *file_name;
  int file_len;
  bool is_notif = prv_is_notif_pref(key, key_len);
  if (is_notif) {
    file_name = NOTIF_PREFS_FILE_NAME;
    file_len = NOTIF_PREFS_FILE_LEN;
    key_len = prv_canonical_notif_key_len(key, key_len);
  } else {
    file_name = SHELL_PREFS_FILE_NAME;
    file_len = SHELL_PREFS_FILE_LEN;
    prv_canonical_shell_key(&key, &key_len);
  }

  prv_lock_for_file(is_notif);

  SettingsFile file;
  status_t status = settings_file_open(&file, file_name, file_len);
  if (FAILED(status)) {
    prv_unlock_for_file(is_notif);
    return status;
  }

  status = settings_file_get(&file, key, key_len, val_out, val_len);
  settings_file_close(&file);

  prv_unlock_for_file(is_notif);
  return status;
}

status_t settings_blob_db_delete(const uint8_t *key, int key_len) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // Determine which file to use based on key type
  const char *file_name;
  int file_len;
  bool is_notif = prv_is_notif_pref(key, key_len);
  if (is_notif) {
    file_name = NOTIF_PREFS_FILE_NAME;
    file_len = NOTIF_PREFS_FILE_LEN;
    key_len = prv_canonical_notif_key_len(key, key_len);
  } else if (prv_canonical_shell_key(&key, &key_len)) {
    file_name = SHELL_PREFS_FILE_NAME;
    file_len = SHELL_PREFS_FILE_LEN;
  } else {
    // Only allow whitelisted settings to be deleted
    return E_INVALID_OPERATION;
  }

  prv_lock_for_file(is_notif);

  SettingsFile file;
  status_t status = settings_file_open(&file, file_name, file_len);
  if (FAILED(status)) {
    prv_unlock_for_file(is_notif);
    return status;
  }

  status = settings_file_delete(&file, key, key_len);
  settings_file_close(&file);

  prv_unlock_for_file(is_notif);
  return status;
}

// Dirty list management

typedef struct {
  BlobDBDirtyItem *dirty_list;
  BlobDBDirtyItem *dirty_list_tail;
} BuildDirtyListContext;

static bool prv_build_dirty_list_callback(SettingsFile *file,
                                          SettingsRecordInfo *info,
                                          void *context) {
  BuildDirtyListContext *ctx = (BuildDirtyListContext *)context;

  // Skip settings that are already synced
  if (!info->dirty) {
    return true;
  }

  // Read the key to check whitelist
  uint8_t key_buf[SETTINGS_KEY_MAX_LEN];
  info->get_key(file, key_buf, info->key_len);

  // Only include whitelisted settings
  if (!prv_is_syncable(key_buf, info->key_len)) {
    return true; // Skip, continue iteration
  }

  // Allocate dirty item
  BlobDBDirtyItem *item = kernel_malloc_check(sizeof(BlobDBDirtyItem) + info->key_len);
  list_init((ListNode *)item);
  item->last_updated = (time_t)info->last_modified;
  item->key_len = info->key_len;
  memcpy(item->key, key_buf, info->key_len);

  // Add to list
  if (ctx->dirty_list == NULL) {
    ctx->dirty_list = item;
    ctx->dirty_list_tail = item;
  } else {
    ctx->dirty_list_tail = (BlobDBDirtyItem *)list_append(
        (ListNode *)ctx->dirty_list_tail, (ListNode *)item);
  }

  return true; // Continue iteration
}

BlobDBDirtyItem *settings_blob_db_get_dirty_list(void) {
  if (!s_initialized) {
    return NULL;
  }

  BuildDirtyListContext ctx = { .dirty_list = NULL, .dirty_list_tail = NULL };

  // Iterate shell prefs file
  prefs_private_lock();
  SettingsFile file;
  status_t status = settings_file_open(&file, SHELL_PREFS_FILE_NAME, SHELL_PREFS_FILE_LEN);
  if (PASSED(status)) {
    settings_file_each(&file, prv_build_dirty_list_callback, &ctx);
    settings_file_close(&file);
  }
  prefs_private_unlock();

  // Iterate notif prefs file
  alerts_preferences_lock();
  status = settings_file_open(&file, NOTIF_PREFS_FILE_NAME, NOTIF_PREFS_FILE_LEN);
  if (PASSED(status)) {
    settings_file_each(&file, prv_build_dirty_list_callback, &ctx);
    settings_file_close(&file);
  }
  alerts_preferences_unlock();

  return ctx.dirty_list;
}

status_t settings_blob_db_mark_synced(const uint8_t *key, int key_len) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // Determine which file to use based on key type
  const char *file_name;
  int file_len;
  bool is_notif = prv_is_notif_pref(key, key_len);
  if (is_notif) {
    file_name = NOTIF_PREFS_FILE_NAME;
    file_len = NOTIF_PREFS_FILE_LEN;
  } else {
    file_name = SHELL_PREFS_FILE_NAME;
    file_len = SHELL_PREFS_FILE_LEN;
  }

  prv_lock_for_file(is_notif);

  SettingsFile file;
  status_t status = settings_file_open(&file, file_name, file_len);
  if (FAILED(status)) {
    prv_unlock_for_file(is_notif);
    return status;
  }

  status = settings_file_mark_synced(&file, key, key_len);
  settings_file_close(&file);

  prv_unlock_for_file(is_notif);
  return status;
}

status_t settings_blob_db_is_dirty(bool *is_dirty_out) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // Quick check: iterate and return true on first dirty whitelisted setting
  typedef struct {
    bool found_dirty;
  } IsDirtyContext;

  bool is_dirty_callback(SettingsFile *file, SettingsRecordInfo *info, void *context) {
    IsDirtyContext *ctx = (IsDirtyContext *)context;

    if (!info->dirty) {
      return true; // Continue
    }

    // Check if whitelisted
    uint8_t key_buf[SETTINGS_KEY_MAX_LEN];
    info->get_key(file, key_buf, info->key_len);

    if (prv_is_syncable(key_buf, info->key_len)) {
      ctx->found_dirty = true;
      return false; // Stop iteration
    }

    return true; // Continue
  }

  IsDirtyContext ctx = { .found_dirty = false };

  // Check shell prefs file
  prefs_private_lock();
  SettingsFile file;
  status_t status = settings_file_open(&file, SHELL_PREFS_FILE_NAME, SHELL_PREFS_FILE_LEN);
  if (PASSED(status)) {
    settings_file_each(&file, is_dirty_callback, &ctx);
    settings_file_close(&file);
  }
  prefs_private_unlock();

  // If already found dirty, no need to check notif prefs
  if (!ctx.found_dirty) {
    alerts_preferences_lock();
    status = settings_file_open(&file, NOTIF_PREFS_FILE_NAME, NOTIF_PREFS_FILE_LEN);
    if (PASSED(status)) {
      settings_file_each(&file, is_dirty_callback, &ctx);
      settings_file_close(&file);
    }
    alerts_preferences_unlock();
  }

  *is_dirty_out = ctx.found_dirty;
  return S_SUCCESS;
}

status_t settings_blob_db_flush(void) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // SettingsFile writes are already atomic, no explicit flush needed
  PBL_LOG_DBG("Settings BlobDB flush (no-op for SettingsFile)");
  return S_SUCCESS;
}

status_t settings_blob_db_mark_all_dirty(void) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  PBL_LOG_INFO("Marking all settings as dirty for full sync");

  status_t result = S_SUCCESS;

  // Mark shell prefs file dirty
  prefs_private_lock();
  SettingsFile file;
  status_t status = settings_file_open(&file, SHELL_PREFS_FILE_NAME, SHELL_PREFS_FILE_LEN);
  if (PASSED(status)) {
    status = settings_file_mark_all_dirty(&file);
    settings_file_close(&file);
    if (FAILED(status)) {
      result = status;
    }
  } else {
    result = status;
  }
  prefs_private_unlock();

  // Mark notif prefs file dirty
  alerts_preferences_lock();
  status = settings_file_open(&file, NOTIF_PREFS_FILE_NAME, NOTIF_PREFS_FILE_LEN);
  if (PASSED(status)) {
    status = settings_file_mark_all_dirty(&file);
    settings_file_close(&file);
    if (FAILED(status) && PASSED(result)) {
      result = status;
    }
  }
  alerts_preferences_unlock();

  return result;
}

// Context for finding last_modified timestamp
typedef struct {
  const uint8_t *search_key;
  int search_key_len;
  time_t last_modified;
  bool found;
} GetTimestampContext;

static bool prv_get_timestamp_callback(SettingsFile *file, SettingsRecordInfo *info,
                                       void *context) {
  GetTimestampContext *ctx = (GetTimestampContext *)context;

  if (info->key_len != ctx->search_key_len) {
    return true; // Continue
  }

  uint8_t key_buf[SETTINGS_KEY_MAX_LEN];
  info->get_key(file, key_buf, info->key_len);

  if (memcmp(key_buf, ctx->search_key, info->key_len) == 0) {
    ctx->last_modified = (time_t)info->last_modified;
    ctx->found = true;
    return false; // Stop iteration
  }

  return true; // Continue
}

status_t settings_blob_db_insert_with_timestamp(const uint8_t *key, int key_len,
                                                const uint8_t *val, int val_len,
                                                time_t timestamp) {
  if (!s_initialized) {
    return E_INTERNAL;
  }

  // Determine which file to use based on key type
  const char *file_name;
  int file_len;
  bool is_notif_pref = prv_is_notif_pref(key, key_len);

  if (is_notif_pref) {
    file_name = NOTIF_PREFS_FILE_NAME;
    file_len = NOTIF_PREFS_FILE_LEN;
    key_len = prv_canonical_notif_key_len(key, key_len);
  } else if (prv_canonical_shell_key(&key, &key_len)) {
    file_name = SHELL_PREFS_FILE_NAME;
    file_len = SHELL_PREFS_FILE_LEN;
  } else {
    return E_INVALID_OPERATION;
  }

  prv_lock_for_file(is_notif_pref);

  SettingsFile file;
  status_t status = settings_file_open(&file, file_name, file_len);
  if (FAILED(status)) {
    prv_unlock_for_file(is_notif_pref);
    return status;
  }

  // Check if existing value has a newer timestamp
  GetTimestampContext ctx = {
    .search_key = key,
    .search_key_len = key_len,
    .last_modified = 0,
    .found = false,
  };
  settings_file_each(&file, prv_get_timestamp_callback, &ctx);

  if (ctx.found && ctx.last_modified > timestamp) {
    // Watch data is newer - reject the insert
    settings_file_close(&file);
    prv_unlock_for_file(is_notif_pref);
    PBL_LOG_DBG("Rejecting stale data: watch=%lu phone=%lu",
            (unsigned long)ctx.last_modified, (unsigned long)timestamp);
    return E_INVALID_OPERATION;
  }

  // Phone data is newer or equal, or key doesn't exist - do the insert
  // Suppress change callback - we don't want to sync back to phone for phone-originated INSERTs.
  // Keep suppressed through prefs_private_handle_blob_db_event() because validation failures
  // can trigger prefs_private_write_backing() which would fire the callback again.
  s_suppress_change_callback = true;
  status = settings_file_set(&file, key, key_len, val, val_len);

  if (PASSED(status)) {
    // Mark as synced - the record came from the phone so it's already in sync
    settings_file_mark_synced(&file, key, key_len);
  }
  settings_file_close(&file);

  // Unlock before calling event handlers - they acquire the same mutex internally
  prv_unlock_for_file(is_notif_pref);

  // Update the in-memory prefs state after successful write
  if (PASSED(status)) {
    PebbleBlobDBEvent event = {
      .db_id = BlobDBIdSettings,
      .type = BlobDBEventTypeInsert,
      .key = (uint8_t *)key,
      .key_len = key_len,
    };
    if (is_notif_pref) {
      alerts_preferences_handle_blob_db_event(&event);
    } else {
      prefs_private_handle_blob_db_event(&event);
    }
  }

  s_suppress_change_callback = false;
  return status;
}
