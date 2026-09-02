/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/app_cache.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "kernel/pebble_tasks.h"
#include "pbl/services/process_management/app_storage.h"
#include "pbl/services/system_task.h"
#include "pbl/services/filesystem/app_file.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/settings/settings_file.h"
#include "shell/normal/quick_launch.h"
#include "shell/normal/watchface.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/list.h"
#include "pbl/util/math.h"
#include "util/time/time.h"
#include "util/units.h"

PBL_LOG_MODULE_DEFINE(service_app_cache, CONFIG_SERVICE_APP_CACHE_LOG_LEVEL);

//! @file
//! App Cache

//! The App Cache keeps track of the install date, last launch, launch count, and size of an
//! application.
//!
//! A priority can also be calculated for each entry. It is calculated by a simple last used
//! algorithm (TODO Improve: PBL-13209) which will help determine which application
//! needs to be evicted in order to free up more space for other application binaries.
//!
//! When an entry is added into the app cache, it means the binaries now reside on the watch. On
//! this function call, a callback is initiated to check if we need to free space for a possible
//! future application. If so, the applications with the lowest priority that add up to or are
//! greater than the space needed will be removed.
//!
//! It is assumed that there will ALWAYS be space for a single application of maximum size based
//! on the platform. The only time when this isn't true is the time between "add_entry" and the
//! callback to clean up the cache.

#define APP_CACHE_FILE_NAME "appcache"

//! each cache entry is ~16 bytes, 4000 / 16 = 250 apps
#define APP_CACHE_MAX_SIZE 4000

//! Keep enough room for the maximum sized application based on platform, plus a little more room.
//! Source: https://pebbletechnology.atlassian.net/wiki/display/DEV/PBW+3.0
#if defined(CONFIG_BOARD_ASTERIX) || defined(CONFIG_BOARD_OBELIX) || defined(UNITTEST)
#define APP_SPACE_BUFFER KiBYTES(300)
#else
#define APP_SPACE_BUFFER MiBYTES(4)
#endif

#define MAX_PRIORITY ((uint32_t)~0)

// 8 quick launch apps, 1 default watchface, 1 default worker
#define DO_NOT_EVICT_LIST_SIZE (10)

static PBL_MUTEX_DEFINE(s_app_cache_mutex);

//! Actual data structure stored in flash about an app cache entry
typedef struct PACKED {
  time_t    install_date;
  time_t    last_launch;
  uint32_t  total_size;
  uint16_t  launch_count;
} AppCacheEntry;

typedef struct {
  ListNode node;
  AppInstallId id;
  uint32_t size;
  uint32_t priority;
} EvictListNode;

typedef struct {
  EvictListNode *list;
  uint32_t bytes_needed;
  uint32_t bytes_in_list;
  const AppInstallId do_not_evict[DO_NOT_EVICT_LIST_SIZE];
} EachEvictData;

//! Takes the information given in entry and calculates a new priority for the app.
//!
//! Policy rules:
//! 1. App that has least recently launched or been installed app is evicted.
static uint32_t prv_calculate_priority(AppCacheEntry *entry) {
  return (uint32_t) MAX(entry->last_launch, entry->install_date);
}

//! Comparator for EvictListNode
static int evict_node_comparator(void *a, void *b) {
  EvictListNode *a_node = (EvictListNode *)a;
  EvictListNode *b_node = (EvictListNode *)b;

  if (b_node->priority > a_node->priority) {
    return 1;
  } else if (b_node->priority < a_node->priority) {
    return -1;
  } else {
    // bigger applications to have a lower priority
    if (b_node->size < a_node->size) {
      return 1;
    } else if (b_node->size > a_node->size) {
      return -1;
    } else {
      return 0;
    }
  }
}

//! Trim the applications with highest priority while still keeping (bytes_in_list > bytes_needed)
static void prv_trim_top_priorities(EvictListNode **list_node, uint32_t *bytes_in_list,
                                              uint32_t bytes_needed) {
  EvictListNode *node = *list_node;
  while (node) {
    EvictListNode *temp = node;
    if (node->size <= (*bytes_in_list - bytes_needed)) {
      *bytes_in_list -= node->size;
      node = (EvictListNode *)list_pop_head((ListNode *)node);
      kernel_free(temp);
    } else {
      break;
    }
  }
  *list_node = node;
}

//! Check if we need to free up some space in the cache. If so, do it.
static void prv_cleanup_app_cache_if_needed(void *data) {
  uint32_t pfs_space = get_available_pfs_space();

  if (pfs_space < APP_SPACE_BUFFER) {
    const uint32_t to_free = (APP_SPACE_BUFFER - pfs_space);
    PBL_LOG_DBG("Cache OOS: Need to free %"PRIu32" bytes, PFS avail space: %"PRIu32"",
        to_free, pfs_space);
    app_cache_free_up_space(to_free);
  }
}

static void prv_delete_cache_callback(void *data) {
  app_cache_flush();
}

static void prv_delete_cached_files(void) {
  pfs_remove_files(is_app_file_name);
}

static bool prv_is_in_list(AppInstallId id, const AppInstallId list[], uint8_t len) {
  for (unsigned int i = 0; i < len; i++) {
    if (list[i] == id) {
      return true;
    }
  }
  return false;
}

//////////////////////
// Settings Helpers
//////////////////////

//! Settings iterator function that finds the entry with the lowest calculated priority
static bool prv_each_free_up_space(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if ((info->key_len != sizeof(AppInstallId)) || (info->val_len != sizeof(AppCacheEntry))) {
    PBL_LOG_WRN("Invalid cache entry with key_len: %u and val_len: %u, flushing",
            info->key_len, info->val_len);
    system_task_add_callback(prv_delete_cache_callback, NULL);
    return false; // stop iterating, delete the file and binaries
  }

  EachEvictData *data = (EachEvictData *)context;

  AppInstallId id;
  AppCacheEntry entry;

  info->get_key(file, (uint8_t *)&id, info->key_len);
  info->get_val(file, (uint8_t *)&entry, info->val_len);

  // create node
  EvictListNode *node = kernel_malloc_check(sizeof(EvictListNode));
  list_init((ListNode *)node);

  // give them an extremely high priority so that we only remove them if we really NEED to
  // This list contains defaults that we shouldn't be removing.
  uint32_t priority = 0;
  if (prv_is_in_list(id, data->do_not_evict, DO_NOT_EVICT_LIST_SIZE)) {
    priority = MAX_PRIORITY;
  }

  *node = (EvictListNode) {
    .id = id,
    .size = entry.total_size,
    .priority = MAX(priority, prv_calculate_priority(&entry)),
  };

  data->list = (EvictListNode *)list_sorted_add((ListNode *)data->list, (ListNode *)node,
      evict_node_comparator, false);
  data->bytes_in_list += node->size;

  if (data->bytes_in_list > data->bytes_needed) {
    prv_trim_top_priorities(&data->list, &data->bytes_in_list, data->bytes_needed);
  }

  return true; // continue iterating
}

//////////////////////////
// AppCache API's
//////////////////////////

//! Updates metadata within the cache entry for the given AppInstallId. Will update such fields as
//! launch count, last launch, and priority
status_t app_cache_app_launched(AppInstallId app_id) {
  status_t rv;
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    SettingsFile file;
    rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
    if (rv != S_SUCCESS) {
      goto unlock;
    }

    AppCacheEntry entry = { 0 };
    rv = settings_file_get(&file, (uint8_t *)&app_id, sizeof(AppInstallId),
        (uint8_t *)&entry, sizeof(AppCacheEntry));

    if (rv == S_SUCCESS) {
      entry.last_launch = rtc_get_time();
      entry.launch_count += 1;

      rv = settings_file_set(&file, (uint8_t *)&app_id, sizeof(AppInstallId),
          (uint8_t *)&entry, sizeof(AppCacheEntry));
    } else {
      app_storage_delete_app(app_id);
      settings_file_delete(&file, (uint8_t *)&app_id, sizeof(AppInstallId));
    }

    settings_file_close(&file);
  }
unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);
  return rv;
}

//! Asks the app cache to remove 'bytes_needed' bytes of application binaries to free up space
//! for other things.
status_t app_cache_free_up_space(uint32_t bytes_needed) {
  if (bytes_needed == 0) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv;
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    SettingsFile file;
    rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
    if (rv != S_SUCCESS) {
      goto unlock;
    }

    // we don't want to remove any default apps or quick launch apps, so keep them in a list.
    EachEvictData evict_data = (EachEvictData) {
      .bytes_needed = bytes_needed,
      .do_not_evict = {
#ifndef CONFIG_SHELL_SDK
        quick_launch_get_app(BUTTON_ID_UP),
        quick_launch_get_app(BUTTON_ID_SELECT),
        quick_launch_get_app(BUTTON_ID_DOWN),
        quick_launch_get_app(BUTTON_ID_BACK),
        quick_launch_single_click_get_app(BUTTON_ID_UP),
        quick_launch_single_click_get_app(BUTTON_ID_DOWN),
        quick_launch_combo_back_up_get_app(),
        quick_launch_combo_up_down_get_app(),
#endif
        watchface_get_default_install_id(),
        worker_preferences_get_default_worker(),
      },
    };

    settings_file_each(&file, prv_each_free_up_space, &evict_data);
    settings_file_close(&file);

    // remove all nodes found
    EvictListNode *node = evict_data.list;
    while (node) {
      EvictListNode *temp = node;
      PBL_LOG_DBG("Deleting application binaries for app id: %"PRIu32", size: %"PRIu32,
          node->id, node->size);
      app_cache_remove_entry(node->id);
      node = (EvictListNode *)list_pop_head((ListNode *)node);
      kernel_free(temp);
    }
  }
unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);
  return rv;
}

//////////////////////
// AppCache Helpers
//////////////////////

// Remove the filename entry in the PFSFileList (via context) that corresponds to the
// app install id passed in via info
static bool prv_remove_matching_resource_file_callback(SettingsFile *file,
                                                       SettingsRecordInfo *info,
                                                       void *context) {
  AppInstallId id;
  // examine the SettingsRecordInfo and extract the AppInstallId from it
  info->get_key(file, (uint8_t *)&id, info->key_len);
  // the context passed in is really a pointer to the resource_list
  PFSFileListEntry **resource_list = context;
  PFSFileListEntry *iter = *resource_list;
  while (iter) {
    // grab the next entry right now since we may delete the node we're looking at
    PFSFileListEntry *next = (PFSFileListEntry *)iter->list_node.next;
    if (app_file_parse_app_id(iter->name) == id) {
      // the AppInstallId of the file matches the one in the cache so we can remove this
      // entry from the resource_list (since we don't want to delete it)
      // note: resource_list may be updated if we happen to remove the first entry in the list
      list_remove(&(iter->list_node), (ListNode**)resource_list, NULL);
      kernel_free(iter);  // free up the memory for the node we just removed
      break; // we can quit now that we've found a match for this id
    }
    iter = next;
  }
  return true;
}

// Delete files from resource_list that don't correspond to entries in the app cache
static void prv_app_cache_find_and_delete_orphans(PFSFileListEntry **resource_list) {
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);

  SettingsFile file;
  status_t rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
  if (rv != S_SUCCESS) {
    pbl_mutex_unlock(&s_app_cache_mutex);
    return;
  }
  // resource_list contains all of the resource files we found.  We only
  // want to delete orphans so we can remove any entries from the list that correspond
  // to items in the app cache...
  // prv_remove_matching_resource_file_callback scans resource_list and removes the entry
  // corresponding to the passed-in application's id
  settings_file_each(&file, prv_remove_matching_resource_file_callback, resource_list);
  settings_file_close(&file);

  pbl_mutex_unlock(&s_app_cache_mutex);

  // resource_list now only contains filenames of resource files that don't have corresponding
  // entries in the app cache. We can safely delete these files.
  PFSFileListEntry *iter = *resource_list;
  while (iter) {
    PBL_LOG_INFO("Orphaned resource file removed: %s", iter->name);
    pfs_remove(iter->name);
    iter = (PFSFileListEntry *)iter->list_node.next;
  }
}

// The bug addressed in PBL-34010 caused resource files to remain in the filesystem even
// after the associated application had been deleted.  This function attempts to find such
// orphaned files and remove them.  Note: further to the bug in PBL-34010, this function will
// remove any resource files that are not related to apps currently in the cache.
static void prv_purge_orphaned_resource_files(void) {
  // create a list of all app resource files in the filesystem
  PFSFileListEntry *resource_files = pfs_create_file_list(is_app_resource_file_name);
  // delete app resource files that don't correspond to entries in the app cache
  prv_app_cache_find_and_delete_orphans(&resource_files);
  pfs_delete_file_list(resource_files);
}

//////////////////////////
// AppCache Settings API's
//////////////////////////

//! Set up the app cache
void app_cache_init(void) {

  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    // if no cache file exists, then we should go ahead and clean up any files that are left over
    int fd = pfs_open(APP_CACHE_FILE_NAME, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
    if (fd < 0) {
      prv_delete_cached_files();
      goto unlock;
    }
    pfs_close(fd);
  }

unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);

  prv_purge_orphaned_resource_files();
}

//! Adds an entry with the given AppInstallId to the cache
status_t app_cache_add_entry(AppInstallId app_id, uint32_t total_size) {
  status_t rv;
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    SettingsFile file;
    rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
    if (rv != S_SUCCESS) {
      goto unlock;
    }

    AppCacheEntry entry = {
      .install_date = rtc_get_time(),
      .last_launch = 0,
      .launch_count = 0,
      .total_size = total_size,
    };

    rv = settings_file_set(&file, (uint8_t *)&app_id, sizeof(AppInstallId),
        (uint8_t *)&entry, sizeof(AppCacheEntry));

    settings_file_close(&file);

    // cleanup the cache if we need to
    system_task_add_callback(prv_cleanup_app_cache_if_needed, NULL);
  }
unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);
  return rv;
}

//! Tests if an entry with the given AppInstallId is in the cache
bool app_cache_entry_exists(AppInstallId app_id) {
  bool exists = false;
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    SettingsFile file;
    status_t rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
    if (rv != S_SUCCESS) {
      goto unlock;
    }

    exists = settings_file_exists(&file, (uint8_t *)&app_id, sizeof(AppInstallId));

    if (exists && !app_storage_app_exists(app_id)) {
      settings_file_delete(&file, (uint8_t *)&app_id, sizeof(AppInstallId));
      exists = false;
    }

    settings_file_close(&file);
  }
unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);
  return exists;
}

//! Removes an entry with the given AppInstallId from the cache
status_t app_cache_remove_entry(AppInstallId app_id) {
  status_t rv;
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    SettingsFile file;
    rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
    if (rv != S_SUCCESS) {
      goto unlock;
    }

    rv = settings_file_delete(&file, (uint8_t *)&app_id, sizeof(AppInstallId));
    if (rv == S_SUCCESS) {
      // Will delete an app from the filesystem.
      app_storage_delete_app(app_id);
    }

    settings_file_close(&file);
  }

  if (rv == S_SUCCESS) {
    PebbleEvent e = {
      .type = PEBBLE_APP_CACHE_EVENT,
      .app_cache_event = {
        .cache_event_type = PebbleAppCacheEvent_Removed,
        .install_id = app_id,
      },
    };
    event_put(&e);
  }
unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);
  return rv;
}

void app_cache_flush(void) {
  PBL_ASSERT_TASK(PebbleTask_KernelBackground);

  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    pfs_remove(APP_CACHE_FILE_NAME);
    prv_delete_cached_files();
  }
  pbl_mutex_unlock(&s_app_cache_mutex);
}

////////////////////////////////
// Testing only
////////////////////////////////

static bool prv_each_get_size(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  if ((info->key_len != sizeof(AppInstallId)) || (info->val_len != sizeof(AppCacheEntry))) {
    return true; // continue iterating
  }

  uint32_t *cache_size = (uint32_t *)context;
  AppCacheEntry entry;
  info->get_val(file, (uint8_t *)&entry, info->val_len);
  *cache_size += entry.total_size;

  return true; // continue iterating
}

uint32_t app_cache_get_size(void) {
  uint32_t cache_size = 0;
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    SettingsFile file;
    status_t rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
    if (rv != S_SUCCESS) {
      goto unlock;
    }

    settings_file_each(&file, prv_each_get_size, &cache_size);
    settings_file_close(&file);
  }
unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);
  return cache_size;
}

typedef struct {
  AppInstallId id;
  uint32_t priority;
} AppCacheEachData;

//! Settings iterator function that finds the entry with the lowest calculated priority
static bool prv_each_min_priority(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if ((info->key_len != sizeof(AppInstallId)) || (info->val_len != sizeof(AppCacheEntry))) {
    return true; // continue iterating
  }

  AppCacheEachData *to_evict = (AppCacheEachData *)context;

  AppInstallId id;
  AppCacheEntry entry;

  info->get_key(file, (uint8_t *)&id, info->key_len);
  info->get_val(file, (uint8_t *)&entry, info->val_len);

  uint32_t entry_priority = prv_calculate_priority(&entry);
  if (entry_priority < to_evict->priority) {
    to_evict->id = id;
    to_evict->priority = entry_priority;
  }

  return true; // continue iterating
}

//! Find the entry in the app cache with the lowest calculated priority
AppInstallId app_cache_get_next_eviction(void) {
  AppInstallId ret_value = INSTALL_ID_INVALID;
  pbl_mutex_lock(&s_app_cache_mutex, PBL_FOREVER);
  {
    SettingsFile file;
    status_t rv = settings_file_open(&file, APP_CACHE_FILE_NAME, APP_CACHE_MAX_SIZE);
    if (rv != S_SUCCESS) {
      goto unlock;
    }

    // set max so that any application will have a lower priority.
    AppCacheEachData to_evict = {
      .id = INSTALL_ID_INVALID,
      .priority = MAX_PRIORITY,
    };
    settings_file_each(&file, prv_each_min_priority, (void *)&to_evict);

    settings_file_close(&file);
    ret_value = to_evict.id;
  }
unlock:
  pbl_mutex_unlock(&s_app_cache_mutex);
  return ret_value;
}
