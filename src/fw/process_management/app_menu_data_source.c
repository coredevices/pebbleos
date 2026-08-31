/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_menu_data_source.h"

#include "applib/ui/menu_layer.h"
#include "apps/system_app_ids.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/process_management/app_order_storage.h"
#include "system/passert.h"
#include "pbl/util/size.h"

#include <string.h>

static void add_app_with_install_id(const AppInstallEntry *entry, AppMenuDataSource *source);
static bool remove_app_with_install_id(const AppInstallId install_id, AppMenuDataSource *source);
static AppMenuNode * prv_find_node_with_install_id(const AppInstallId install_id,
                                               const AppMenuDataSource * const source);
static void prv_unload_node(const AppMenuDataSource *source, AppMenuNode *node);

////////////////////////////////
// List helper functions
////////////////////////////////

static bool prv_is_app_filtered_out(AppInstallEntry *entry, AppMenuDataSource * const source) {
  return (source->callbacks.filter && (source->callbacks.filter(source, entry) == false));
}

/////////////////////////
// Order List helpers
/////////////////////////

//! Place these in the order that is desired in the Launcher.
//! Set `move_on_activity` to true if you only want the item to jump to the top during communication
//! The movement will not happen while looking at the launcher, it will only refresh on a
//! close->open
static const struct {
  AppInstallId install_id;
  bool move_on_activity;
} s_override_table[] = {
  { APP_ID_SPORTS, false },
  { APP_ID_GOLF, false },
#ifdef APP_ID_WORKOUT
  { APP_ID_WORKOUT, true },
#endif
  { APP_ID_MUSIC, true },
};

// Returns 0 if not in table. Otherwise, returns the rank in `s_override_table`. Rank is
// where the lowest index returns the highest rank
static int prv_override_index(AppInstallId app_id) {
  const uint32_t num_overrides = ARRAY_LENGTH(s_override_table);
  for (uint32_t i = 0; i < num_overrides; i++) {
    AppInstallId cur_id = s_override_table[i].install_id;
    if (cur_id == app_id) {
      const bool should_move = (!s_override_table[i].move_on_activity ||
                                app_install_is_prioritized(cur_id));
      return (should_move) ? (num_overrides - i + 1) : 0;
    }
  }
  return 0;
}

static int prv_app_override_comparator(AppInstallId app_id, AppInstallId new_id) {
  return (prv_override_index(app_id) - prv_override_index(new_id));
}

static int prv_comparator_ascending_zero_last(unsigned int a, unsigned int b) {
  return ((a != 0) && (b != 0)) ? (b - a) : // Sort in ascending order
                                  (a - b);  // 0 should be sorted last so invert the sort
}

T_STATIC int prv_app_node_comparator(void *app_node_ref, void *new_node_ref) {
  const AppMenuNode *app_node = app_node_ref;
  const AppMenuNode *new_node = new_node_ref;

  const bool is_app_quick_launch = (app_node->visibility == ProcessVisibilityQuickLaunch);
  const bool is_new_quick_launch = (new_node->visibility == ProcessVisibilityQuickLaunch);
  const int override_cmp_rv = prv_app_override_comparator(app_node->install_id,
                                                          new_node->install_id);
  if (is_app_quick_launch != is_new_quick_launch) {
    // Quick Launch only apps are first
    return (is_app_quick_launch ? 1 : 0) - (is_new_quick_launch ? 1 : 0);
  } else if (override_cmp_rv) {
    // Apps that override storage, record, and install order
    return (override_cmp_rv);
  } else if (app_node->storage_order != new_node->storage_order) {
    // Storage order (smallest first)
    return prv_comparator_ascending_zero_last(app_node->storage_order, new_node->storage_order);
  } else if (app_node->record_order != new_node->record_order) {
    // Record order (smallest first)
    return prv_comparator_ascending_zero_last(app_node->record_order, new_node->record_order);
  } else {
    // AppInstallId (smallest first)
    return new_node->install_id - app_node->install_id;
  }
}

static void prv_set_storage_order(AppMenuDataSource *source, AppMenuNode *menu_node,
                                  AppMenuOrderStorage *storage, bool update_and_take_ownership) {
  if (!storage) {
    return;
  }

  for (int i = 0; i < storage->list_length; i++) {
    const AppInstallId storage_app_id = storage->id_list[i];
    if (storage_app_id == INSTALL_ID_INVALID) {
      continue;
    }

    const AppMenuStorageOrder new_storage_order =
        (AppMenuStorageOrder)i + AppMenuStorageOrderGeneralOrderOffset;

    if (menu_node && (menu_node->install_id == storage_app_id)) {
      menu_node->storage_order = new_storage_order;
      if (!update_and_take_ownership) {
        break;
      } else {
        continue;
      }
    }

    if (update_and_take_ownership) {
      AppMenuNode *other_node = prv_find_node_with_install_id(storage_app_id, source);
      if (other_node) {
        other_node->storage_order = new_storage_order;
      }
    }
  }
  if (update_and_take_ownership) {
    app_free(storage);
  }
}

static void prv_sorted_add(AppMenuDataSource *source, AppMenuNode *menu_node) {
  // Update the entire list order only if we've just read the order in this context. If we haven't
  // just read the order, then we're building a list starting from an empty list, so just set the
  // order for the new node.
  prv_set_storage_order(source, menu_node, source->order_storage ?: app_order_read_order(),
                        (source->order_storage == NULL));

  // If we're adding the Settings app node to the list and it hasn't received a storage order,
  // then give it its default order
  if ((menu_node->install_id == APP_ID_SETTINGS) &&
      (menu_node->storage_order == AppMenuStorageOrder_NoOrder)) {
    menu_node->storage_order = AppMenuStorageOrder_SettingsDefaultOrder;
  }

  source->list = (AppMenuNode *)list_sorted_add(&source->list->node, &menu_node->node,
                                                prv_app_node_comparator, true /* ascending */);
}

////////////////////////////////
// AppInstallManager Callbacks
////////////////////////////////

typedef struct InstallData {
  AppInstallId id;
  AppMenuDataSource *source;
  InstallEventType event_type;
} InstallData;

void prv_alert_data_source_changed(AppMenuDataSource *data_source) {
  if (data_source->callbacks.changed) {
    data_source->callbacks.changed(data_source->callback_context);
  }
}

static void prv_do_app_added(AppMenuDataSource *source, AppInstallId install_id);
static void prv_do_app_removed(AppMenuDataSource *source, AppInstallId install_id);
static void prv_do_app_icon_name_updated(AppMenuDataSource *source, AppInstallId install_id);
static void prv_do_app_db_cleared(AppMenuDataSource *source);

void prv_handle_app_event(void *data) {
  InstallData *install_data = data;
  AppMenuDataSource *source = install_data->source;
  AppInstallId install_id = install_data->id;

  switch (install_data->event_type) {
    case APP_AVAILABLE:
      prv_do_app_added(source, install_id);
      break;
    case APP_REMOVED:
      prv_do_app_removed(source, install_id);
      break;
    case APP_ICON_NAME_UPDATED:
      prv_do_app_icon_name_updated(source, install_id);
      break;
    case APP_DB_CLEARED:
      prv_do_app_db_cleared(source);
      break;
    default:
      break;
  }

  kernel_free(install_data);
}

void prv_send_callback_to_app(AppMenuDataSource *data_source, AppInstallId install_id,
                              InstallEventType event_type) {
  InstallData *install_data = kernel_malloc_check(sizeof(InstallData));
  *install_data = (InstallData) {
    .id = install_id,
    .source = data_source,
    .event_type = event_type,
  };
  process_manager_send_callback_event_to_process(PebbleTask_App,
                                                 prv_handle_app_event,
                                                 install_data);
}

//! Must be run from the app task
static void prv_do_app_added(AppMenuDataSource *source, AppInstallId install_id) {
  AppInstallEntry entry;
  if (!app_install_get_entry_for_install_id(install_id, &entry) ||
      prv_is_app_filtered_out(&entry, source)) {
    return;
  }

  add_app_with_install_id(&entry, source);
  prv_alert_data_source_changed(source);
}

//! Called when an application is installed
static void prv_app_added_callback(const AppInstallId install_id, void *data) {
  AppMenuDataSource *data_source = data;
  prv_send_callback_to_app(data_source, install_id, APP_AVAILABLE);
}

//! Must be run from the app task
static void prv_do_app_removed(AppMenuDataSource *source, AppInstallId install_id) {
  // Don't filter, just always try removing from the list:
  const bool is_removed = remove_app_with_install_id(install_id, source);
  if (is_removed) {
    prv_alert_data_source_changed(source);
  }
}

//! Called when an application is uninstalled
static void prv_app_removed_callback(const AppInstallId install_id, void *data) {
  AppMenuDataSource *data_source = data;
  prv_send_callback_to_app(data_source, install_id, APP_REMOVED);
}

//! Must be run from the app task
static void prv_do_app_icon_name_updated(AppMenuDataSource *source, AppInstallId install_id) {
  AppInstallEntry entry;
  if (!app_install_get_entry_for_install_id(install_id, &entry)) {
    return;
  }

  AppMenuNode *node = prv_find_node_with_install_id(install_id, source);
  if (prv_is_app_filtered_out(&entry, source)) {
    if (node == NULL) {
      // Changed and still excluded:
      return;
    }
    // Changed and is now excluded:
    prv_unload_node(source, node);
  } else {
    // Changed and is now included:
    if (node == NULL) {
      add_app_with_install_id(&entry, source);
    }
  }
  prv_alert_data_source_changed(source);
}

static void prv_app_icon_name_updated_callback(const AppInstallId install_id, void *data) {
  AppMenuDataSource *data_source = data;
  prv_send_callback_to_app(data_source, install_id, APP_ICON_NAME_UPDATED);
}

//! Must be run from the app task
static void prv_do_app_db_cleared(AppMenuDataSource *source) {
  AppMenuNode *iter = source->list;
  while (iter) {
    AppMenuNode *temp = (AppMenuNode *)list_get_next((ListNode *)iter);

    // if the node belonged to the app_db, remove
    if (app_install_id_from_app_db(iter->install_id)) {
      prv_unload_node(source, iter);
    }
    iter = temp;
  }

  prv_alert_data_source_changed(source);
}

static void prv_app_db_cleared_callback(const AppInstallId install_id, void *data) {
  // data is just a pointer to the AppMenuDataSource
  prv_send_callback_to_app((AppMenuDataSource *)data, INSTALL_ID_INVALID, APP_DB_CLEARED);
}

static bool prv_app_enumerate_callback(AppInstallEntry *entry, void *data) {
  if (prv_is_app_filtered_out(entry, (AppMenuDataSource *)data)) {
    return true; // continue
  }
  add_app_with_install_id(entry, data);
  return true; // continue
}

////////////////////
// Add / remove helper functions:

// This function should only be called once per app entry. The icon from the app will either be
// loaded and cached or we will load the default system icon that is set by the client.
static void prv_load_list_item_icon(AppMenuDataSource *source, AppMenuNode *node) {
  // Should only call this function if the icon has not been loaded
  PBL_ASSERTN(node->icon == NULL);

  if (node->icon_resource_id != RESOURCE_ID_INVALID) {
    // If we have some sort of valid resource_id, try loading it
    node->icon = gbitmap_create_with_resource_system(node->app_num, node->icon_resource_id);
  }

  if (!node->icon) {
    // If we failed to load the app's icon or it didn't have one, use the default. This will either
    // be NULL or an actual icon...both are fine. And no need to clip the default icon
    node->icon = source->default_icon;
    return;
  }

  // Clip the icon down if needed
  static const GRect icon_clip = {{0, 0}, {32, 32}};
  grect_clip(&node->icon->bounds, &icon_clip);
}

static void prv_unload_list_item_icon(const AppMenuDataSource *source, AppMenuNode *node) {
  // Don't destroy the default icon here, we'll destroy it later.
  if (node->icon && node->icon != source->default_icon) {
    gbitmap_destroy(node->icon);
    node->icon = NULL;
  }
}

static void prv_load_list_if_needed(AppMenuDataSource *source) {
  if (source->is_list_loaded) {
    return;
  }
  source->is_list_loaded = true;

  PBL_ASSERTN(!source->order_storage);
  source->order_storage = app_order_read_order();

  app_install_enumerate_entries(prv_app_enumerate_callback, source);

  if (source->order_storage != NULL) {
    app_free(source->order_storage);
    source->order_storage = NULL;
  }
}

static void prv_unload_node(const AppMenuDataSource *source, AppMenuNode *node) {
  prv_unload_list_item_icon(source, node);
  list_remove((ListNode*)node, (ListNode**)&source->list, NULL);
  app_free(node->name);
  app_free(node);
}

static void add_app_with_install_id(const AppInstallEntry *entry, AppMenuDataSource *source) {
  if (source->is_list_loaded == false) {
    return;
  }

  AppMenuNode *node = app_malloc_check(sizeof(AppMenuNode));
  *node = (AppMenuNode) {
    .install_id = entry->install_id,
    .app_num = app_install_get_app_icon_bank(entry),
    .icon_resource_id = app_install_entry_get_icon_resource_id(entry),
    .uuid = entry->uuid,
    .color = entry->color,
    .visibility = entry->visibility,
    .sdk_version = entry->sdk_version,
    .record_order = entry->record_order,
  };

  uint8_t len;
  const char *app_name = app_install_get_custom_app_name(node->install_id);
  if (!app_name) {
    app_name = entry->name;
  }

  len = strlen(app_name) + 1;
  node->name = app_malloc_check(len);
  strncpy(node->name, app_name, len);

  prv_sorted_add(source, node);
}

static AppMenuNode * prv_find_node_with_install_id(const AppInstallId install_id,
                                               const AppMenuDataSource * const source) {
  AppMenuNode *node = source->list;
  while (node != NULL) {
    if (node->install_id == install_id) {
      return node;
    } else {
      node = (AppMenuNode*)list_get_next((ListNode*)node);
    }
  }
  return NULL;
}

//! @return True if there was app with install_id was found and removed from the list.
static bool remove_app_with_install_id(const AppInstallId install_id, AppMenuDataSource *source) {
  if (source->is_list_loaded == false) {
    return false;
  }

  AppMenuNode *node = prv_find_node_with_install_id(install_id, source);
  if (node == NULL) {
    return false;
  }

  prv_unload_node(source, node);
  return true;
}

////////////////////
// public interface

void app_menu_data_source_init(AppMenuDataSource *source,
                               const AppMenuDataSourceCallbacks *callbacks,
                               void *callback_context) {
  PBL_ASSERTN(source != NULL);
  *source = (AppMenuDataSource) {
    .callback_context = callback_context,
  };
  if (callbacks) {
    source->callbacks = *callbacks;
  }

  // Register callbacks for app_install_manager updates:
  static const AppInstallCallback s_app_install_callbacks[NUM_INSTALL_EVENT_TYPES] = {
    [APP_AVAILABLE] = prv_app_added_callback,
    [APP_REMOVED] = prv_app_removed_callback,
    [APP_UPGRADED] = prv_app_removed_callback,
    [APP_ICON_NAME_UPDATED] = prv_app_icon_name_updated_callback,
    [APP_DB_CLEARED] = prv_app_db_cleared_callback,
  };
  source->app_install_callback_node = (struct AppInstallCallbackNode) {
    .data = source,
    .callbacks = s_app_install_callbacks,
  };
  app_install_register_callback(&source->app_install_callback_node);
}

void app_menu_data_source_deinit(AppMenuDataSource *source) {
  app_install_deregister_callback(&source->app_install_callback_node);
  // Free the AppMenuNodes:
  AppMenuNode *node = source->list;
  while (node != NULL) {
    AppMenuNode * const next = (AppMenuNode*)list_get_next((ListNode*)node);
    prv_unload_node(source, node);
    node = next;
  }

  if (source->default_icon) {
    gbitmap_destroy(source->default_icon);
  }

  source->callbacks.changed = NULL;
  source->is_list_loaded = false;
}

void app_menu_data_source_enable_icons(AppMenuDataSource *source, uint32_t fallback_icon_id) {
  // should only call this once, and should be passed in a valid resource id.
  PBL_ASSERTN(source->default_icon == NULL && fallback_icon_id != RESOURCE_ID_INVALID);

  source->show_icons = true;
  // The return value will be a valid GBitmap* or NULL (because of an OOM that shouldn't ever happen
  // We will handle both gracefully.
  source->default_icon = gbitmap_create_with_resource_system(SYSTEM_APP, fallback_icon_id);
}

static uint16_t prv_transform_index(AppMenuDataSource *source, uint16_t index) {
  if (source->callbacks.transform_index) {
    return source->callbacks.transform_index(source, index, source->callback_context);
  }
  return index;
}

AppMenuNode* app_menu_data_source_get_node_at_index(AppMenuDataSource *source, uint16_t row_index) {
  prv_load_list_if_needed(source);
  return (AppMenuNode*)list_get_at((ListNode*)source->list,
                                   prv_transform_index(source, row_index));
}

uint16_t app_menu_data_source_get_count(AppMenuDataSource *source) {
  prv_load_list_if_needed(source);
  return list_count((ListNode*)source->list);
}

uint16_t app_menu_data_source_get_index_of_app_with_install_id(AppMenuDataSource *source,
                                                               AppInstallId install_id) {
  prv_load_list_if_needed(source);
  AppMenuNode *node = source->list;
  uint16_t index = 0;
  while (node != NULL) {
    if (node->install_id == install_id) {
      return prv_transform_index(source, index);
    }
    node = (AppMenuNode*)list_get_next((ListNode*)node);
    ++index;
  }
  return MENU_INDEX_NOT_FOUND;
}

GBitmap *app_menu_data_source_get_node_icon(AppMenuDataSource *source, AppMenuNode *node) {
  if (!node->icon && source->show_icons) {
    // If the icon is currently NULL and we should be showing icons, load the icon
    prv_load_list_item_icon(source, node);
  }
  // Will return the icon if it exists, or NULL if one doesn't.
  return node->icon;
}

void app_menu_data_source_draw_row(AppMenuDataSource *source, GContext *ctx, Layer *cell_layer,
                                   MenuIndex *cell_index) {
  AppMenuNode *node = app_menu_data_source_get_node_at_index(source, cell_index->row);
  // Will return an icon or NULL depending on if icons are enabled.
  GBitmap *bitmap = app_menu_data_source_get_node_icon(source, node);
  const GCompOp op = (gbitmap_get_format(bitmap) == GBitmapFormat1Bit) ? GCompOpTint : GCompOpSet;
  graphics_context_set_compositing_mode(ctx, op);
  menu_cell_basic_draw(ctx, cell_layer, node->name, NULL, bitmap);
}
