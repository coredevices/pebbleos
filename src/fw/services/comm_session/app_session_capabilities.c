/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "process_management/app_install_types.h"
#include "process_management/app_manager.h"
#include "pbl/services/comm_session/app_session_capabilities.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/os/mutex.h"
#include "system/passert.h"
#include "util/units.h"

#define APP_SESSION_CAPABILITIES_CACHE_FILENAME "app_comm"

#define APP_SESSION_CAPABILITIES_CACHE_FILE_MAX_USED_SPACE (KiBYTES(2))

static PebbleMutex *s_mutex;

static status_t prv_open(SettingsFile *settings_file) {
  return settings_file_open(settings_file, APP_SESSION_CAPABILITIES_CACHE_FILENAME,
                            APP_SESSION_CAPABILITIES_CACHE_FILE_MAX_USED_SPACE);
}

static status_t prv_open_locked(SettingsFile *settings_file) {
  mutex_lock(s_mutex);
  status_t status = prv_open(settings_file);
  if (FAILED(status)) {
    mutex_unlock(s_mutex);
  }
  return status;
}

static void prv_close_and_unlock(SettingsFile *settings_file) {
  settings_file_close(settings_file);
  mutex_unlock(s_mutex);
}

bool comm_session_current_app_session_cache_has_capability(CommSessionCapability capability) {
  CommSession *app_session = comm_session_get_current_app_session();

  const Uuid app_uuid = app_manager_get_current_app_md()->uuid;

  SettingsFile settings_file;
  status_t open_status = prv_open_locked(&settings_file);
  bool file_open = PASSED(open_status);

  uint64_t cached_capabilities = 0;
  if (file_open) {
    settings_file_get(&settings_file,
                      &app_uuid, sizeof(app_uuid),
                      &cached_capabilities, sizeof(cached_capabilities));
  }

  uint64_t new_capabilities = cached_capabilities;
  if (app_session) {
    // Connected, grab fresh capabilities data:
    new_capabilities = comm_session_get_capabilities(app_session);

    if (file_open && (new_capabilities != cached_capabilities)) {
      settings_file_set(&settings_file,
                        &app_uuid, sizeof(app_uuid),
                        &new_capabilities, sizeof(new_capabilities));
    }
  }

  if (file_open) {
    prv_close_and_unlock(&settings_file);
  }

  return ((new_capabilities & capability) != 0);
}

static void prv_rewrite_cb(SettingsFile *old_file,
                           SettingsFile *new_file,
                           SettingsRecordInfo *info,
                           void *context) {
  if (!info->val_len) {
    return; // Cache for this app has been deleted, don't rewrite it
  }
  Uuid key;
  uint64_t val;
  info->get_key(old_file, &key, sizeof(key));
  info->get_val(old_file, &val, sizeof(val));
  settings_file_set(new_file, &key, sizeof(key), &val, sizeof(val));
}

void comm_session_app_session_capabilities_evict(const Uuid *app_uuid) {
  SettingsFile settings_file;
  if (PASSED(prv_open_locked(&settings_file))) {
    settings_file_delete(&settings_file, app_uuid, sizeof(*app_uuid));
    prv_close_and_unlock(&settings_file);
  }
}

void comm_session_app_session_capabilities_init(void) {
  s_mutex = mutex_create();
  
  PBL_ASSERTN(s_mutex != NULL);

  SettingsFile settings_file;
  if (PASSED(prv_open_locked(&settings_file))) {
    settings_file_rewrite(&settings_file, prv_rewrite_cb, NULL);
    prv_close_and_unlock(&settings_file);
  }
}
