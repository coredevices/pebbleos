/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_install_manager.h"
#include "app_install_manager_private.h"

#include "app_custom_icon.h"
#include "app_manager.h"
#include "worker_manager.h"

#include "applib/event_service_client.h"
#include "apps/system_app_registry.h"
#include "console/prompt.h"
#include <pbl/drivers/task_watchdog.h>
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "kernel/pebble_tasks.h"
#include "kernel/util/sleep.h"
#include "resource/resource.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/comm_session/app_session_capabilities.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/app_cache.h"
#include "pbl/services/blob_db/app_db.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/persist.h"
#include "pbl/services/process_management/app_storage.h"
#ifdef CONFIG_MIC
#include "pbl/services/voice/voice_recording.h"
#endif
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/circular_cache.h"
#include "pbl/util/size.h"

#include <pbl/os/mutex.h>
#include <pbl/util/attributes.h>

typedef struct PACKED RecentApp {
  AppInstallId id;
  time_t last_activity;
  bool can_expire;
} RecentApp;

// The number of applications to store in the circular cache.
// These are used to detect which application shave recently communicated
#define NUM_RECENT_APPS 5
#define CACHE_ENTRY_SIZE (sizeof(RecentApp))
#define CACHE_BUFFER_SIZE (NUM_RECENT_APPS * CACHE_ENTRY_SIZE)
#define RECENT_APP_LAST_ACTIVITY_INVALID (0)

typedef struct RecentAppCache {
  PebbleRecursiveMutex *mutex;
  CircularCache cache;
  uint8_t cache_buffer[CACHE_BUFFER_SIZE];
} RecentAppCache;

static RecentAppCache s_recent_apps;

//! timeout for an app that has OnCommunication visibility (given in seconds)
static int32_t VISIBILITY_ON_ACTIVITY_TIMEOUT_SECONDS = (5 * SECONDS_PER_MINUTE);

static bool prv_app_install_entry_from_app_db_entry(AppInstallId id, AppDBEntry *db_entry,
                                                    AppInstallEntry *install_entry);

static AppInstallId s_pending_app_deletion;
static AppInstallId s_pending_worker_deletion;

// PBL-31769: This should be moved to send_text.c
#if defined(APP_ID_SEND_TEXT)
static EventServiceInfo s_capabilities_event_info;
#endif

//////////////////
// Misc helpers
//////////////////

static const AppRegistryEntry *prv_get_registry_list_entry(AppInstallId id,
                                                           unsigned int *record_order_out) {
  if (app_install_id_from_app_db(id)) {
    return NULL;
  }

  for (int i = 0; i < (int)ARRAY_LENGTH(APP_RECORDS); i++) {
    if (APP_RECORDS[i].id == id) {
      if (record_order_out) {
        *record_order_out = i + 1;
      }
      return &APP_RECORDS[i];
    }
  }
  return NULL;
}

// optimization: sort the UUID's and then search more quickly. This will most likely
// require many changes to the JSON generation script.
AppInstallId app_get_install_id_for_uuid_from_registry(const Uuid *uuid) {
  for (uint16_t i = 0; i < ARRAY_LENGTH(APP_RECORDS); i++) {
    const AppRegistryEntry *reg_entry = &APP_RECORDS[i];

    if (reg_entry->type == AppInstallStorageFw) {
      const PebbleProcessMd *md = reg_entry->md_fn();
      if (md && uuid_equal(&md->uuid, uuid)) {
        return reg_entry->id;
      }
    } else if ((reg_entry->type == AppInstallStorageResources)
        && uuid_equal(&reg_entry->uuid, uuid)) {
      return reg_entry->id;
    }
  }
  return INSTALL_ID_INVALID;
}

bool app_install_is_prioritized(AppInstallId install_id) {
  if (install_id == INSTALL_ID_INVALID) {
    return false;
  }

  bool rv = false;
  mutex_lock_recursive(s_recent_apps.mutex);
  {
    RecentApp *app = circular_cache_get(&s_recent_apps.cache, &install_id);
    if (app) {
      const int32_t time_since_activity = time_get_uptime_seconds() - app->last_activity;
      if (app->can_expire && (time_since_activity < VISIBILITY_ON_ACTIVITY_TIMEOUT_SECONDS)) {
        // The recent app should eventually expire and we are still below the threshold
        rv = true;
      } else if (!app->can_expire && app->last_activity != RECENT_APP_LAST_ACTIVITY_INVALID) {
        // The recent app should never expire and we haven't been manually expired yet.
        rv = true;
      }
    }
  }
  mutex_unlock_recursive(s_recent_apps.mutex);
  return rv;
}

void app_install_unmark_prioritized(AppInstallId install_id) {
  if (install_id == INSTALL_ID_INVALID) {
    return;
  }

  mutex_lock_recursive(s_recent_apps.mutex);
  {
    RecentApp *app = circular_cache_get(&s_recent_apps.cache, &install_id);
    if (app) {
      app->last_activity = RECENT_APP_LAST_ACTIVITY_INVALID;
    }
  }
  mutex_unlock_recursive(s_recent_apps.mutex);
}

void app_install_mark_prioritized(AppInstallId install_id, bool can_expire) {
  if (install_id == INSTALL_ID_INVALID) {
    return;
  }

  mutex_lock_recursive(s_recent_apps.mutex);
  {
    const time_t cur_time = time_get_uptime_seconds();
    RecentApp *app = circular_cache_get(&s_recent_apps.cache, &install_id);
    if (app) {
      app->last_activity = cur_time;
      app->can_expire = can_expire;
    } else {
      RecentApp app = {
        .id = install_id,
        .last_activity = cur_time,
        .can_expire = can_expire,
      };
      circular_cache_push(&s_recent_apps.cache, &app);
    }
  }
  mutex_unlock_recursive(s_recent_apps.mutex);
}

#if UNITTEST
void app_install_manager_flush_recent_communication_timestamps(void) {
  circular_cache_flush(&s_recent_apps.cache);
  memset(&s_recent_apps.cache_buffer, 0, sizeof(s_recent_apps.cache));
}
#endif

bool app_install_entry_is_watchface(const AppInstallEntry *entry) {
  return (entry->process_type == ProcessTypeWatchface);
}

bool app_install_entry_has_worker(const AppInstallEntry *entry) {
  return (entry->has_worker);
}

bool app_install_entry_is_hidden(const AppInstallEntry *entry) {
  switch (entry->visibility) {
    case ProcessVisibilityHidden:
      return true;
    case ProcessVisibilityShownOnCommunication:
      // make icon hidden (return true) if app has not recently communicated
      return !app_install_is_prioritized(entry->install_id);
    case ProcessVisibilityShown:
      return false;
    case ProcessVisibilityQuickLaunch:
      return true;
  }
  return false;
}

bool app_install_entry_is_quick_launch_visible_only(const AppInstallEntry *entry) {
  return (entry->visibility == ProcessVisibilityQuickLaunch);
}

bool app_install_entry_is_SDK_compatible(const AppInstallEntry *entry) {
  return (entry->sdk_version.major == PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR &&
          entry->sdk_version.minor <= PROCESS_INFO_CURRENT_SDK_VERSION_MINOR);
}

T_STATIC ListNode *s_head_callback_node_list = NULL;

void app_install_register_callback(struct AppInstallCallbackNode *callback_node) {
  PBL_ASSERTN(callback_node->node.next == NULL);
  PBL_ASSERTN(callback_node->node.prev == NULL);
  PBL_ASSERTN(s_head_callback_node_list != &callback_node->node);
  PBL_ASSERTN(callback_node->callbacks != NULL);

  callback_node->registered_by = pebble_task_get_current();

  s_head_callback_node_list = list_prepend(s_head_callback_node_list, &callback_node->node);
}

void app_install_deregister_callback(struct AppInstallCallbackNode *callback_node) {
  PBL_ASSERTN(callback_node->node.next != NULL
              || callback_node->node.prev != NULL
              || s_head_callback_node_list == &callback_node->node);
  list_remove(&callback_node->node, &(s_head_callback_node_list), NULL);
}

void app_install_cleanup_registered_app_callbacks(void) {
  struct AppInstallCallbackNode *iter = (struct AppInstallCallbackNode *) s_head_callback_node_list;
  while (iter) {
    if (iter->registered_by == PebbleTask_App) {
      list_remove((ListNode *)&iter->node, &s_head_callback_node_list, NULL);
    }
    iter = (struct AppInstallCallbackNode *) list_get_next(&iter->node);
  }
}

static void app_install_invoke_callbacks(InstallEventType event_type, AppInstallId install_id) {
  struct AppInstallCallbackNode *callback_node = (struct AppInstallCallbackNode *) s_head_callback_node_list;
  while (callback_node) {
    if (callback_node->callbacks[event_type]) {
      callback_node->callbacks[event_type](install_id, callback_node->data);
    }
    callback_node = (struct AppInstallCallbackNode *) list_get_next(&callback_node->node);
  }
}

typedef struct {
  AppInstallEnumerateCb cb;
  void *data;
  AppInstallEntry *entry_buf;
} EnumerateData;

static void prv_app_install_enumerate_app_db(AppInstallId install_id, AppDBEntry *db_entry,
    void *data) {
  EnumerateData *cb_data = (EnumerateData *)data;

  prv_app_install_entry_from_app_db_entry(install_id, db_entry, cb_data->entry_buf);
  cb_data->cb(cb_data->entry_buf, cb_data->data);
}

void app_install_enumerate_entries(AppInstallEnumerateCb cb, void *data) {
  // Keep this off of the stack. This function presses the limits of our stack.
  AppInstallEntry *entry = kernel_malloc_check(sizeof(AppInstallEntry));

  // Iterate over the registry
  for (uint32_t i = 0; i < ARRAY_LENGTH(APP_RECORDS); i++) {
    if (app_install_get_entry_for_install_id(APP_RECORDS[i].id, entry)) {
      // if a false is returned from the function, then stop iterating.
      if (cb(entry, data) == false) {
        kernel_free(entry);
        return;
      }
    }
  }

  // Iterate over AppDB applications
  EnumerateData cb_data = {
    .cb = cb,
    .data = data,
    .entry_buf = entry,
  };
  app_db_enumerate_entries(prv_app_install_enumerate_app_db, &cb_data);

  kernel_free(entry);
}

AppInstallId app_install_get_id_for_uuid(const Uuid *uuid) {
  if (uuid_is_invalid(uuid) || uuid_is_system(uuid)) {
    // Don't allow lookups by system uuid, there will be a bunch of apps with that uuid
    return INSTALL_ID_INVALID;
  }

  // search in system registry first, if found return the ID.
  AppInstallId id = app_get_install_id_for_uuid_from_registry(uuid);
  if (id != INSTALL_ID_INVALID) {
    return id;
  }

  // registry miss, now lets search in the app_db
  id = app_db_get_install_id_for_uuid(uuid);
  if (id != INSTALL_ID_INVALID) {
    return id;
  }

  return INSTALL_ID_INVALID;
}

static void prv_app_install_delete(AppInstallId id, Uuid *uuid, bool app_upgrade,
                                   bool delete_cache) {
  if (!app_upgrade) {
    // remove settings associated with the app
    pin_db_delete_with_parent(uuid);
  }

  if (delete_cache) {
    // only log when we actually delete the cache entry. This is so we don't print out 100 logs
    // during an app cache clear
    PBL_LOG_DBG("Deleting app with id %"PRId32"", id);
    app_cache_remove_entry(id);
  }
}

static void prv_delete_pending_id(AppInstallId *app_id) {
  if (*app_id != INSTALL_ID_INVALID) {
    // app cache will delete the app binaries even if the entry for the app_id does not exist
    app_cache_remove_entry(*app_id);
    *app_id = INSTALL_ID_INVALID;
  }
}

static void prv_process_pending_deletions(void) {
  prv_delete_pending_id(&s_pending_app_deletion);
  prv_delete_pending_id(&s_pending_worker_deletion);
  PBL_LOG_DBG("Finished processing pending deletions");
}

typedef struct InstallCallbackData {
  //! We can't have multiple callbacks in flight at once. Only invoke a new set of callbacks
  //! if this is false.
  bool callback_in_progress;

  //! We may have to pause doing callbacks to wait for the app or worker to close. If so, this is set to
  //! true.
  bool callback_paused_for_app;
  bool callback_paused_for_worker;

  InstallEventType install_type;

  AppInstallId install_id;
  Uuid *uuid;

  //! Callback to call when we're doing issuing this callback.
  InstallCallbackDoneCallback done_callback;

  void* callback_data;
} InstallCallbackData;

InstallCallbackData s_install_callback_data;

static bool prv_ids_equal(AppInstallId one, AppInstallId two) {
  return ((one == two) && (one != INSTALL_ID_INVALID));
}

static void app_install_launcher_task_callback(void *context) {
  if (!s_install_callback_data.callback_paused_for_app &&
      !s_install_callback_data.callback_paused_for_worker) {
    // Only close the app the first time around.

    if (s_install_callback_data.install_type == APP_UPGRADED ||
        s_install_callback_data.install_type == APP_REMOVED ||
        s_install_callback_data.install_type == APP_DB_CLEARED) {

      const AppInstallId to_kill = s_install_callback_data.install_id;

      // Close the current app if it is the one we are trying to remove/upgrade
      // OR
      // If we are doing an APP_DB_CLEAR and the currently running app is from the app_db,
      // also clear it.
      const AppInstallId cur_app_id = app_manager_get_current_app_id();
      if (prv_ids_equal(cur_app_id, to_kill) ||
            ((s_install_callback_data.install_type == APP_DB_CLEARED) &&
             (app_install_id_from_app_db(cur_app_id)))) {
        PBL_LOG_DBG("close and delay callbacks for app closing");

        s_install_callback_data.callback_paused_for_app = true;
        s_pending_app_deletion = cur_app_id;
        app_manager_close_current_app(true);
      }

      // Close the current worker if it is the one we are trying to remove/upgrade
      // OR
      // If we are doing an APP_DB_CLEAR and the currently running worker is from the app_db,
      // also clear it.
      const AppInstallId cur_worker_id = worker_manager_get_current_worker_id();
      if (prv_ids_equal(cur_worker_id, to_kill) ||
          ((s_install_callback_data.install_type == APP_DB_CLEARED) &&
           (app_install_id_from_app_db(cur_worker_id)))) {
        PBL_LOG_DBG("close and delay callbacks for worker closing");

        s_install_callback_data.callback_paused_for_worker = true;
        s_pending_worker_deletion = cur_worker_id;
        worker_manager_handle_remove_current_worker();
      }

      if (s_install_callback_data.callback_paused_for_app ||
          s_install_callback_data.callback_paused_for_worker) {
        // We're trying to remove or upgrade our currently running app. We now have
        // to wait until the app actually closes before continuing to notify the rest
        // of the system that we've removed or upgraded the app.
        return;
      }
    }
  }

  PBL_LOG_DBG("app_install_invoke_callbacks");
  app_install_invoke_callbacks(s_install_callback_data.install_type,
                               s_install_callback_data.install_id);

  bool app_upgrade = false;

  switch (s_install_callback_data.install_type) {
    case APP_UPGRADED:
      app_upgrade = true;
      /* fallthrough */
    case APP_REMOVED:
      prv_app_install_delete(s_install_callback_data.install_id, s_install_callback_data.uuid,
          app_upgrade, true /* delete cache entry */);
      // Only delete the app's persist file when the user explicitly removes the
      // app, not during an AppDB clear.
      if (!app_upgrade) {
        persist_service_delete_file(s_install_callback_data.uuid);
#if !defined(CONFIG_RECOVERY_FW)
        comm_session_app_session_capabilities_evict(s_install_callback_data.uuid);
#ifdef CONFIG_MIC
        voice_recording_delete_owned_by(s_install_callback_data.uuid);
#endif
#endif
      }
      break;
    case APP_DB_CLEARED:
      prv_process_pending_deletions();
      break;
    default:
      break;
  }

  if (s_install_callback_data.done_callback) {
    s_install_callback_data.done_callback(s_install_callback_data.callback_data);
  }

  if (s_install_callback_data.uuid) {
    kernel_free(s_install_callback_data.uuid);
  }

  s_install_callback_data = (InstallCallbackData) {
    .callback_in_progress = false
  };
}

bool app_install_do_callbacks(InstallEventType event_type, AppInstallId install_id,
    Uuid *uuid, InstallCallbackDoneCallback done_callback, void* callback_data) {
  if (s_install_callback_data.callback_in_progress) {
    PBL_LOG_ERR("Failed to do app callbacks, already in progress");
    return false;
  }

  s_install_callback_data = (InstallCallbackData) {
    .callback_in_progress = true,
    .install_id = install_id,
    .uuid = uuid,
    .install_type = event_type,
    .done_callback = done_callback,
    .callback_data = callback_data
  };

  launcher_task_add_callback(app_install_launcher_task_callback, NULL);

  return true;
}

const char *app_install_get_custom_app_name(AppInstallId install_id) {
  const char *name = app_custom_get_title(install_id);
  if (name) {
    return name;
  }

  return NULL;
}

uint32_t app_install_entry_get_icon_resource_id(const AppInstallEntry *entry) {
  return entry->icon_resource_id;
}

ResAppNum app_install_get_app_icon_bank(const AppInstallEntry *entry) {
  if (app_install_id_from_system(entry->install_id)) {
    return SYSTEM_APP;
  } else {
    return entry->install_id;
  }
}

bool app_install_is_app_running(AppInstallId id) {
  return app_manager_get_task_context()->install_id == id;
}

bool app_install_is_worker_running(AppInstallId id) {
  return worker_manager_get_task_context()->install_id == id;
}

void app_install_notify_app_closed(void) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);
  // If we've previously paused doing app callbacks to wait for the app to close, resume them
  // now if the worker is also done
  if (s_install_callback_data.callback_paused_for_app) {
    if (!s_install_callback_data.callback_paused_for_worker)  {
      app_install_launcher_task_callback(NULL);
    } else {
      s_install_callback_data.callback_paused_for_app = false;
    }
  }
}

void app_install_notify_worker_closed(void) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);
  // If we've previously paused doing app callbacks to wait for the app to close, resume them
  // now if the worker is also done
  if (s_install_callback_data.callback_paused_for_worker) {
    if (!s_install_callback_data.callback_paused_for_app)  {
      app_install_launcher_task_callback(NULL);
    } else {
      s_install_callback_data.callback_paused_for_worker = false;
    }
  }
}

//////////////////
// 3.0 Functions
//////////////////

static int prv_cmp_recent_apps(void *a, void *b) {
  RecentApp *app_a = (RecentApp *)a;
  RecentApp *app_b = (RecentApp *)b;

  return !(app_a->id == app_b->id);
}

// PBL-31769: This should be moved to send_text.c
#if defined(APP_ID_SEND_TEXT)
static void prv_capabilities_changed_event_handler(PebbleEvent *event, void *context) {
  // We only care if send text support changed right now
  if (!event->capabilities.flags_diff.send_text_support) {
    return;
  }

  const PebbleProcessMd *md = app_install_get_md(APP_ID_SEND_TEXT, false /* worker */);
  const InstallEventType event_type = (md ? APP_AVAILABLE : APP_REMOVED);
  app_install_invoke_callbacks(event_type, APP_ID_SEND_TEXT);
  app_install_release_md(md);
}
#endif

void app_install_manager_init(void) {
  circular_cache_init(&s_recent_apps.cache, s_recent_apps.cache_buffer, sizeof(RecentApp),
                      NUM_RECENT_APPS, prv_cmp_recent_apps);
  s_recent_apps.mutex = mutex_create_recursive();

  // PBL-31769: This should be moved to send_text.c
#if defined(APP_ID_SEND_TEXT)
  s_capabilities_event_info = (EventServiceInfo) {
    .type = PEBBLE_CAPABILITIES_CHANGED_EVENT,
    .handler = prv_capabilities_changed_event_handler,
  };
  event_service_client_subscribe(&s_capabilities_event_info);
#endif
}

bool app_install_id_from_system(AppInstallId id) {
  return (id < INSTALL_ID_INVALID);
}

bool app_install_id_from_app_db(AppInstallId id) {
  return (id > INSTALL_ID_INVALID);
}

static GColor prv_hard_coded_color_for_3rd_party_apps(Uuid *uuid) {

  // Remove this from Recovery FW for code size savings.
#if !defined(CONFIG_RECOVERY_FW)

  // this is a temporary solution to enable custom colors for 3rd-party apps
  // please replace this, once PBL-19673 landed
  typedef struct {
    Uuid uuid;
    uint8_t color_argb;
  } ColorMapping;

  static const ColorMapping mappings[] = {
    #include "app_install_manager_known_apps.h"
  };

  for (size_t i = 0; i < ARRAY_LENGTH(mappings); i++) {
    if (uuid_equal(uuid, &mappings[i].uuid)) {
      return (GColor){.argb = mappings[i].color_argb};
    }
  }

#endif

  return GColorClear;
}


static GColor prv_valid_color_from_uuid(GColor color, Uuid *uuid) {
#ifdef CONFIG_BOARD_ASTERIX
  return GColorClear;
#endif

  color = gcolor_closest_opaque(color);
  if (!gcolor_equal(color, GColorClear)) {
    return color;
  }

  color = prv_hard_coded_color_for_3rd_party_apps(uuid);
  if (!gcolor_equal(color, GColorClear)) {
    return color;
  }

  // if color isn't provided, build hash over uuid and pick from selected fall-back colors
  GColor fall_back_colors[] = {GColorFromHEX(0x0000aa), GColorFromHEX(0x005500),
    GColorFromHEX(0x550055), GColorFromHEX(0xff0055), GColorFromHEX(0xaa0000)};
  uint8_t uuid_byte_sum = 0;
  for (uint8_t *b = &uuid->byte0; b <= &uuid->byte15; b++) {
    uuid_byte_sum += *b;
  }
  return fall_back_colors[uuid_byte_sum % ARRAY_LENGTH(fall_back_colors)];
}

static bool prv_app_install_entry_from_app_db_entry(AppInstallId id, AppDBEntry *db_entry,
                                                    AppInstallEntry *entry) {

  *entry = (AppInstallEntry) {
    .install_id = id,
    .type = AppInstallStorageFlash,
    .visibility = process_metadata_flags_visibility(db_entry->info_flags),
    // PebbleTask_App because the flag parsing function needs it, and we can assume all
    // applications registered with the manager are applications, not workers.
    .process_type = process_metadata_flags_process_type(db_entry->info_flags, PebbleTask_App),
    .has_worker = process_metadata_flags_has_worker(db_entry->info_flags),
    .icon_resource_id = db_entry->icon_resource_id,
    .uuid = db_entry->uuid,
    .color = prv_valid_color_from_uuid(db_entry->app_face_bg_color, (Uuid *)&db_entry->uuid),
    .sdk_version = db_entry->sdk_version,
  };

  strncpy(entry->name, db_entry->name, APP_NAME_SIZE_BYTES);
  return true;
}

static bool prv_app_install_entry_from_resource_registry_entry(const AppRegistryEntry *reg_entry,
    AppInstallEntry *entry) {
  PebbleProcessInfo *app_header = kernel_malloc_check(sizeof(PebbleProcessInfo));
  bool rv = false;

  if (resource_load_byte_range_system(SYSTEM_APP, reg_entry->bin_resource_id, 0,
        (uint8_t *)app_header, sizeof(*app_header)) != sizeof(*app_header)) {
    PBL_LOG_WRN("Stored app with resource id %d not found in resources",
            reg_entry->bin_resource_id);
    goto done;
  }
  *entry = (AppInstallEntry) {
    .install_id = reg_entry->id,
    .type = AppInstallStorageResources,
    .visibility = process_metadata_flags_visibility(app_header->flags),
    // PebbleTask_App because the flag parsing function needs it, and we can assume all
    // applications registered with the manager are applications, not workers.
    .process_type = process_metadata_flags_process_type(app_header->flags, PebbleTask_App),
    .has_worker = process_metadata_flags_has_worker(app_header->flags),
    .icon_resource_id = reg_entry->icon_resource_id,
    .uuid = reg_entry->uuid,
    .color = prv_valid_color_from_uuid(reg_entry->color, (Uuid *) &reg_entry->uuid),
    .sdk_version = app_header->sdk_version,
  };

  i18n_get_with_buffer(reg_entry->name, entry->name, APP_NAME_SIZE_BYTES);
  rv = true;
done:
  kernel_free(app_header);
  return rv;
}

bool prv_app_install_entry_from_fw_registry_entry(const AppRegistryEntry *reg_entry,
    AppInstallEntry *entry) {

  const PebbleProcessMdSystem *md = (PebbleProcessMdSystem *) reg_entry->md_fn();

  if (!md) {
    return false;
  }

  *entry = (AppInstallEntry) {
    .install_id = reg_entry->id,
    .type = AppInstallStorageFw,
    .visibility = md->common.visibility,
    .process_type = md->common.process_type,
    .has_worker = md->common.has_worker,
    .icon_resource_id = md->icon_resource_id,
    .uuid = md->common.uuid,
    .color = prv_valid_color_from_uuid(reg_entry->color, (Uuid *) &md->common.uuid),
    .sdk_version = process_metadata_get_sdk_version((PebbleProcessMd *)md),
  };

  i18n_get_with_buffer(md->name, entry->name, APP_NAME_SIZE_BYTES);
  return true;
}

bool app_install_get_entry_for_install_id(AppInstallId install_id, AppInstallEntry *entry) {
  if ((install_id == INSTALL_ID_INVALID) || (entry == NULL)) {
    return false;
  }

  unsigned int record_order = 0;
  const AppRegistryEntry *reg_entry = prv_get_registry_list_entry(install_id, &record_order);
  if (reg_entry) {
    bool rv = false;
    // switch on registry type
    switch (reg_entry->type) {
      case AppInstallStorageFw:
        rv = prv_app_install_entry_from_fw_registry_entry(reg_entry, entry);
        break;
      case AppInstallStorageResources:
        rv = prv_app_install_entry_from_resource_registry_entry(reg_entry, entry);
        break;
      case AppInstallStorageInvalid:
      case AppInstallStorageFlash:
        PBL_LOG_DBG("Invalid app registry type %d", reg_entry->type);
        break;
    }
    if (rv) {
      entry->record_order = record_order;
    }
    return rv;
  } else if (app_db_exists_install_id(install_id)) {
    AppDBEntry *db_entry = kernel_malloc_check(sizeof(AppDBEntry));
    bool rv = (app_db_get_app_entry_for_install_id(install_id, db_entry) == S_SUCCESS);
    if (rv) {
      rv = prv_app_install_entry_from_app_db_entry(install_id, db_entry, entry);
    }
    kernel_free(db_entry);
    return rv;
  }

  PBL_LOG_ERR("Failed to get entry for id %"PRId32, install_id);
  return false;
}

bool app_install_get_uuid_for_install_id(AppInstallId install_id, Uuid *uuid_out) {
  PBL_ASSERTN(uuid_out);
  AppInstallEntry entry;
  if (app_install_get_entry_for_install_id(install_id, &entry)) {
    *uuid_out = entry.uuid;
    return true;
  } else {
    *uuid_out = UUID_INVALID;
    return false;
  }
}

bool app_install_is_watchface(AppInstallId app_id) {
  AppInstallEntry entry;
  if (!app_install_get_entry_for_install_id(app_id, &entry)) {
    return false;
  }
  return app_install_entry_is_watchface(&entry);
}

static const PebbleProcessMd *prv_get_md_for_reg_entry(const AppRegistryEntry *reg_entry) {
  switch (reg_entry->type) {
    case AppInstallStorageFw:
      // If its a FW app, just return the Md
      return reg_entry->md_fn();
    case AppInstallStorageResources: {
      // If its a RESOURCE app, we much read from the resource pack and populate an Md
      PebbleProcessInfo app_header;
      if (resource_load_byte_range_system(SYSTEM_APP, reg_entry->bin_resource_id, 0,
            (uint8_t *)&app_header, sizeof(app_header)) != sizeof(app_header)) {
        PBL_LOG_WRN("Stored app with resource id %d not found in resources",
                reg_entry->bin_resource_id);
        return NULL;
      }

      // Convert to PebbleProcessMd. Set the correct icon_id from the passed in argument
      app_header.icon_resource_id = reg_entry->icon_resource_id;
      // freed in process_manager.c
      PebbleProcessMdResource *md = kernel_malloc_check(sizeof(PebbleProcessMdResource));
      process_metadata_init_with_resource_header(md, &app_header, reg_entry->bin_resource_id,
          PebbleTask_App);
      const PebbleProcessMd *const_md = (PebbleProcessMd *)md;
      return const_md;
    }
    default:
      return NULL;
  }
}

static const PebbleProcessMd *prv_get_md_for_flash_id(AppInstallId id, bool worker) {
#ifdef CONFIG_RECOVERY_FW
  return NULL;
#endif

  PebbleProcessInfo app_header;
  uint8_t build_id_buffer[BUILD_ID_EXPECTED_LEN];
  const PebbleTask task = worker ? PebbleTask_Worker : PebbleTask_App;
  if (GET_APP_INFO_SUCCESS !=
      app_storage_get_process_info(&app_header, build_id_buffer, id, task)) {
    PBL_LOG_WRN("Failed to get app from flash with id %"PRIu32, id);
    return NULL;
  }

  // freed in process_manager.c
  PebbleProcessMdFlash *md = kernel_malloc_check(sizeof(PebbleProcessMdFlash));
  process_metadata_init_with_flash_header(md, &app_header, id, task, build_id_buffer);
  const PebbleProcessMd *const_md = (PebbleProcessMd *)md;
  return const_md;
}


// PebbleProcessMd is freed in process_manager.c when the application quits
const PebbleProcessMd *app_install_get_md(AppInstallId id, bool worker) {
  const AppRegistryEntry *reg_entry = prv_get_registry_list_entry(id, NULL /* record_order */);
  if (reg_entry) {
    return prv_get_md_for_reg_entry(reg_entry);
  } else if (app_db_exists_install_id(id)) {
    return prv_get_md_for_flash_id(id, worker);
  }

  // Not a registered app, fail.
  PBL_LOG_ERR("Can't get PebbleProcessMd for app id %"PRId32, id);
  return NULL;
}

void app_install_release_md(const PebbleProcessMd *md) {
  if (!md) {
    return;
  }

  switch (md->process_storage) {
  case ProcessStorageBuiltin:
    break;
  case ProcessStorageFlash:
  case ProcessStorageResource:
    kernel_free((PebbleProcessMd*) md);
  }
}

static void prv_enumerate_app_db_delete(AppInstallId install_id, AppDBEntry *db_entry,
    void *data) {
  PBL_ASSERTN(app_install_id_from_app_db(install_id));
  task_watchdog_bit_set(pebble_task_get_current());

  const bool gracefully = true;
  if (app_manager_get_current_app_id() == install_id) {
    process_manager_put_kill_process_event(PebbleTask_App, gracefully);
  }

  if (worker_manager_get_current_worker_id() == install_id) {
    process_manager_put_kill_process_event(PebbleTask_Worker, gracefully);
  }

  // We are not deleting the cache here because it will be deleted quicker in filesystem iteration
  // This way, it can clean up much quicker than searching through the filesystem every time
  const bool app_upgrade = false;
  const bool delete_cache = false;
  prv_app_install_delete(install_id, &db_entry->uuid, app_upgrade, delete_cache);
}

void app_install_clear_app_db(void) {
  app_db_enumerate_entries(prv_enumerate_app_db_delete, NULL);
  app_cache_flush();
}
