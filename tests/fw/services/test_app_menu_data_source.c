/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"


#include "applib/ui/menu_layer.h"
#include "flash_region/flash_region.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_menu_data_source.h"
#include "resource/resource.h"
#include "resource/resource_storage.h"
#include "resource/resource_storage_file.h"
#include "pbl/services/system_task.h"
#include "pbl/services/app_cache.h"
#include "pbl/services/blob_db/app_db.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/util/build_id.h"
#include "pbl/util/size.h"
#include "fixtures/load_test_resources.h"

// access it directly just to test things out
#include "shell/system_app_registry_list.auto.h"

#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

// Stub Includes
////////////////////////////////////
#include "stubs_activity.h"
#include "stubs_analytics.h"
#include "stubs_app_custom_icon.h"
#include "stubs_app_fetch_endpoint.h"
#include "stubs_app_manager.h"
#include "stubs_app_state.h"
#include "stubs_bootbits.h"
#include "stubs_build_id.h"
#include "stubs_comm_session.h"
#include "stubs_event_loop.h"
#include "stubs_event_service_client.h"
#include "stubs_events.h"
#include "stubs_fonts.h"
#include "stubs_gbitmap.h"
#include "stubs_graphics.h"
#include "stubs_graphics.h"
#include "stubs_graphics_context.h"
#include "stubs_heap.h"
#include "stubs_hexdump.h"
#include "stubs_i18n.h"
#include "stubs_kino_reel.h"
#include "stubs_logging.h"
#include "stubs_memory_layout.h"
#include "stubs_menu_layer.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_persist.h"
#include "stubs_pin_db.h"
#include "stubs_process_loader.h"
#include "stubs_process_manager.h"
#include "stubs_process_manager.h"
#include "stubs_prompt.h"
#include "stubs_put_bytes.h"
#include "stubs_quick_launch.h"
#include "stubs_rand_ptr.h"
#include "stubs_serial.h"
#include "stubs_shell_prefs.h"
#include "stubs_sleep.h"
#include "stubs_system_task.h"
#include "stubs_task_watchdog.h"
#include "stubs_watchface.h"
#include "stubs_worker_manager.h"

// Fake Includes
////////////////////////////////////
#include "fake_spi_flash.h"

// Test reset function for app_order_storage cached state
extern void app_order_storage_reset_for_tests(void);

const uint32_t g_num_file_resource_stores = 0;
const FileResourceData g_file_resource_stores[] = {};

#define APP_REGISTRY_FIXTURE_PATH "app_registry"

#define BG_COUNTER_APP_NAME "Background Counter"
#define MENU_LAYER_APP_NAME "MenuLayerName"
#define BIG_TIME_APP_NAME "Big Time"

#define BG_COUNTER_APP_ID 1
#define MENU_LAYER_APP_ID 2
#define BIG_TIME_APP_ID 3

// background counter
static const AppDBEntry bg_counter_app = {
  .name = BG_COUNTER_APP_NAME,
  .uuid = {0x1e, 0xb1, 0xd3, 0x9b, 0x56, 0x98, 0x48, 0x44,
           0xb3, 0x94, 0x1f, 0x87, 0xb6, 0xbe, 0xae, 0x67},
  .info_flags = PROCESS_INFO_HAS_WORKER | PROCESS_INFO_STANDARD_APP,
  .icon_resource_id = 0,
  .app_version = {
    .major = 1,
    .minor = 0,
  },
  .sdk_version = {
    .major = 5,
    .minor = 13,
  },
  .app_face_bg_color = {0},
  .template_id = 0,
};

// menu layer
static const AppDBEntry menu_layer_app = {
  .name = MENU_LAYER_APP_NAME,
  .uuid = {0xb8, 0x26, 0x2e, 0x08, 0x57, 0xe9, 0x4e, 0x58,
           0x88, 0x02, 0x45, 0xfd, 0xfe, 0xe0, 0xac, 0x77},
  .info_flags = PROCESS_INFO_STANDARD_APP,
  .icon_resource_id = 0,
  .app_version = {
    .major = 2,
    .minor = 0,
  },
  .sdk_version = {
    .major = 5,
    .minor = 13,
  },
  .app_face_bg_color = {0},
  .template_id = 0,
};

// big time
static const AppDBEntry big_time_app = {
  .name = BIG_TIME_APP_NAME,
  .uuid = {0xaf, 0xcc, 0x68, 0x76, 0x8f, 0x84, 0x44, 0xe0,
           0xbb, 0x8b, 0x02, 0x3f, 0xfb, 0x2d, 0x7c, 0x2c},
  .info_flags = PROCESS_INFO_WATCH_FACE,
  .icon_resource_id = 0,
  .app_version = {
    .major = 6,
    .minor = 0,
  },
  .sdk_version = {
    .major = 5,
    .minor = 17,
  },
  .app_face_bg_color = {0},
  .template_id = 0,
};

AppInstallId bg_counter_app_id;
AppInstallId menu_layer_app_id;
AppInstallId big_time_app_id;

// Fakes
////////////////////////////////////
uint32_t time_get_uptime_seconds(void) {
  return rtc_get_time();
}

// Tests
////////////////////////////////////
static MenuLayer menu_layer;
static AppMenuDataSource data_source;

static bool app_filter_callback(struct AppMenuDataSource *source, AppInstallEntry *entry) {
  if (app_install_entry_is_hidden(entry)) {
    return false;
  }
  if (app_install_entry_is_watchface(entry)) {
    return false; // Only apps
  }
  return true;
}

static bool watchface_filter_callback(struct AppMenuDataSource *source, AppInstallEntry *entry) {
  if (app_install_entry_is_hidden(entry)) {
    return false;
  }
  if (!app_install_entry_is_watchface(entry)) {
    return false; // Only watchfaces
  }
  return true;
}

static bool everything_filter_callback(struct AppMenuDataSource *source, AppInstallEntry *entry) {
  return true;
}

void test_app_menu_data_source__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);

  pfs_init(false);
  pfs_format(false);

  // Reset app_order_storage cached state - file_known_missing persists between tests
  // and would cause app_order_read_order to return NULL even when the file exists
  app_order_storage_reset_for_tests();

  app_install_manager_init();
  app_db_init();
  app_cache_init();

  load_resource_fixture_in_flash(RESOURCES_FIXTURE_PATH, SYSTEM_RESOURCES_FIXTURE_NAME, false);
  resource_init();

  // simulate installing bg_counter_app on flash
  app_db_insert((uint8_t *)&bg_counter_app.uuid, sizeof(Uuid),
                (uint8_t *)&bg_counter_app, sizeof(AppDBEntry));
  bg_counter_app_id = app_db_get_install_id_for_uuid(&bg_counter_app.uuid);
  app_cache_add_entry(bg_counter_app_id, 10701);
  cl_assert_equal_i(BG_COUNTER_APP_ID, bg_counter_app_id);

  // simulate installing menu_layer_app on flash
  app_db_insert((uint8_t *)&menu_layer_app.uuid, sizeof(Uuid),
                (uint8_t *)&menu_layer_app, sizeof(AppDBEntry));
  menu_layer_app_id = app_db_get_install_id_for_uuid(&menu_layer_app.uuid);
  app_cache_add_entry(menu_layer_app_id, 10701);
  cl_assert_equal_i(MENU_LAYER_APP_ID, menu_layer_app_id);

  // simulate installing big_time_app on flash
  app_db_insert((uint8_t *)&big_time_app.uuid, sizeof(Uuid),
                (uint8_t *)&big_time_app, sizeof(AppDBEntry));
  big_time_app_id = app_db_get_install_id_for_uuid(&big_time_app.uuid);
  app_cache_add_entry(big_time_app_id, 10701);
  cl_assert_equal_i(BIG_TIME_APP_ID, big_time_app_id);

  menu_layer_init(&menu_layer, &GRect(0,0,144,76));

  rtc_set_time(100);
}

extern ListNode *s_head_callback_node_list;

void test_app_menu_data_source__cleanup(void) {
  s_head_callback_node_list = NULL;
  app_install_manager_flush_recent_communication_timestamps();
}

/*************************************

 *************************************/

static void prv_menu_layer_reload_data(void *data) {
  cl_assert_equal_p(data, &menu_layer);
  menu_layer_reload_data(&menu_layer);
}

void test_app_menu_data_source__pass_init(void) {
  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = everything_filter_callback,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert(node);
  }
}

void test_app_menu_data_source__check_default_order_apps(void) {
  // settings has to be at the beginning. The app_menu_data_source module enforces it
  static const AppInstallId app_default_order[] = {APP_ID_SETTINGS, APP_ID_MUSIC,
                                                   APP_ID_NOTIFICATIONS, APP_ID_ALARMS,
                                                   APP_ID_WATCHFACES, APP_ID_WORKOUT,
                                                   BG_COUNTER_APP_ID, MENU_LAYER_APP_ID};
  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = app_filter_callback,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);
  cl_assert_equal_i(num_apps, ARRAY_LENGTH(app_default_order));

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, app_default_order[i]);
  }

  app_menu_data_source_deinit(&data_source);
}

static uint16_t prv_reverse_index(AppMenuDataSource *data_source, uint16_t original_index,
                                  void *context) {
  return app_menu_data_source_get_count(data_source) - 1 - original_index;
}

void test_app_menu_data_source__transform_index(void) {
  // settings has to be at the beginning. The app_menu_data_source module enforces it
  static const AppInstallId app_default_order[] = {APP_ID_SETTINGS, APP_ID_MUSIC,
                                                   APP_ID_NOTIFICATIONS, APP_ID_ALARMS,
                                                   APP_ID_WATCHFACES, APP_ID_WORKOUT,
                                                   BG_COUNTER_APP_ID, MENU_LAYER_APP_ID};
  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = app_filter_callback,
    .transform_index = prv_reverse_index,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);
  cl_assert_equal_i(num_apps, ARRAY_LENGTH(app_default_order));

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, app_default_order[num_apps - 1 - i]);
  }

  app_menu_data_source_deinit(&data_source);
}

void test_app_menu_data_source__check_default_order_watchfaces(void) {
  static const AppInstallId watchface_default_order[] = {APP_ID_TICTOC,
                                                         BIG_TIME_APP_ID};
  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = watchface_filter_callback,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);
  cl_assert_equal_i(num_apps, ARRAY_LENGTH(watchface_default_order));

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, watchface_default_order[i]);
  }
  app_menu_data_source_deinit(&data_source);
}

void prv_write_order_to_file(const AppInstallId order[], uint8_t num_entries) {
  uint8_t entries_to_write = num_entries + 1;
  uint16_t file_len = sizeof(uint8_t) + (entries_to_write) * sizeof(AppInstallId);

  pfs_remove("lnc_ord");
  int fd = pfs_open("lnc_ord", OP_FLAG_WRITE, FILE_TYPE_STATIC, file_len);
  pfs_write(fd, &entries_to_write, sizeof(entries_to_write));
  pfs_write(fd, order, sizeof(AppInstallId) * num_entries);
  AppInstallId zero_id = 0;
  pfs_write(fd, &zero_id, sizeof(zero_id));
  pfs_close(fd);
}

void prv_test_new_order_with_filter_callback(const AppInstallId order[], uint8_t num_entries,
                                             AppMenuFilterCallback filter_callback) {
  prv_write_order_to_file(order, num_entries);

  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = filter_callback,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);
  // cl_assert_equal_i(num_apps, num_entries);

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, order[i]);
  }

  app_menu_data_source_deinit(&data_source);
}

void prv_shuffle(AppInstallId *array, uint8_t n) {
  for (uint8_t i = 0; i < n - 1; i++) {
    uint8_t j = i + rand() / (RAND_MAX / (n - i) + 1);
    int t = array[j];
    array[j] = array[i];
    array[i] = t;
  }
}

void test_app_menu_data_source__change_order_apps(void) {
  // settings has to be at the beginning. The app_menu_data_source module enforces it
  AppInstallId app_order[] = {APP_ID_SETTINGS, APP_ID_MUSIC, APP_ID_NOTIFICATIONS, APP_ID_ALARMS,
                              APP_ID_WATCHFACES, APP_ID_WORKOUT, BG_COUNTER_APP_ID,
                              MENU_LAYER_APP_ID};

  uint8_t num_entries = ARRAY_LENGTH(app_order);
  prv_test_new_order_with_filter_callback(app_order, num_entries,
                                          app_filter_callback);
}

void test_app_menu_data_source__change_order_watchfaces(void) {
  AppInstallId watchface_order[] = {BIG_TIME_APP_ID, APP_ID_TICTOC};

  for (int i = 0; i < 10; i++) {
    uint8_t num_entries = ARRAY_LENGTH(watchface_order);
    prv_shuffle(watchface_order, num_entries);
    prv_test_new_order_with_filter_callback(watchface_order, num_entries,
                                            watchface_filter_callback);
  }
}

void test_app_menu_data_source__last_app_not_in_order_file(void) {
  // settings has to be at the beginning. The app_menu_data_source module enforces it
  AppInstallId app_order[] = {APP_ID_SETTINGS, APP_ID_MUSIC, APP_ID_NOTIFICATIONS, APP_ID_ALARMS,
                              APP_ID_WATCHFACES, APP_ID_WORKOUT,
                              BG_COUNTER_APP_ID};

  uint8_t num_entries = ARRAY_LENGTH(app_order);
  prv_write_order_to_file(app_order, num_entries);

  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = app_filter_callback,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);
  cl_assert_equal_i(num_apps, num_entries + 1);

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);

    // MENU_LAYER_APP_ID isn't in file, but it should still be in the list at the end.
    if (i == (num_apps - 1)) {
      cl_assert_equal_i(node->install_id, MENU_LAYER_APP_ID);
    } else {
      cl_assert_equal_i(node->install_id, app_order[i]);
    }
  }

  app_menu_data_source_deinit(&data_source);
}

void test_app_menu_data_source__floating_music_app(void) {
  // settings has to be at the beginning. The app_menu_data_source module enforces it
  // This test will move the music app to the second position
  AppInstallId written_order[] = {APP_ID_SETTINGS, APP_ID_NOTIFICATIONS, APP_ID_ALARMS,
                                  APP_ID_WATCHFACES, APP_ID_WORKOUT, BG_COUNTER_APP_ID,
                                  MENU_LAYER_APP_ID, APP_ID_MUSIC};

  AppInstallId desired_order[] = {APP_ID_MUSIC, APP_ID_SETTINGS, APP_ID_NOTIFICATIONS,
                                  APP_ID_ALARMS, APP_ID_WATCHFACES, APP_ID_WORKOUT,
                                  BG_COUNTER_APP_ID, MENU_LAYER_APP_ID};

  uint8_t num_entries = ARRAY_LENGTH(written_order);
  prv_write_order_to_file(written_order, num_entries);

  app_install_mark_prioritized(APP_ID_MUSIC, true /* can expire */);

  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = app_filter_callback,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, desired_order[i]);
  }

  app_menu_data_source_deinit(&data_source);
}

void test_app_menu_data_source__all_floating_apps(void) {
  // settings has to be at the beginning. The app_menu_data_source module enforces it
  // This test will move the music app to the second position
  AppInstallId written_order[] = {APP_ID_SETTINGS, APP_ID_NOTIFICATIONS, APP_ID_ALARMS,
                                  APP_ID_WATCHFACES, APP_ID_WORKOUT, BG_COUNTER_APP_ID,
                                  MENU_LAYER_APP_ID, APP_ID_MUSIC};

  AppInstallId desired_order[] = {APP_ID_GOLF, APP_ID_WORKOUT, APP_ID_MUSIC,
                                  APP_ID_SETTINGS, APP_ID_NOTIFICATIONS, APP_ID_ALARMS,
                                  APP_ID_WATCHFACES, BG_COUNTER_APP_ID, MENU_LAYER_APP_ID};

  uint8_t num_entries = ARRAY_LENGTH(written_order);
  prv_write_order_to_file(written_order, num_entries);

  app_install_mark_prioritized(APP_ID_MUSIC, true /* can expire */);
  app_install_mark_prioritized(APP_ID_WORKOUT, false /* can expire */);
  app_install_mark_prioritized(APP_ID_GOLF, true /* can expire */);

  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = app_filter_callback,
  }, &menu_layer);
  uint16_t num_apps = app_menu_data_source_get_count(&data_source);

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, desired_order[i]);
  }

  app_menu_data_source_deinit(&data_source);
}

void test_app_menu_data_source__complete_sorted_order(void) {
  // Apps are sorted in the order of Quick Launch only, Override apps, Storage (smallest first),
  // Record (smallest first), and finally Install ID (smallest first). Verify that this is true.
  // This also tests that the Settings app (a special case) respects storage order if it exists
  // in the storage order list.
  AppInstallId storage_order[] = {APP_ID_NOTIFICATIONS, BG_COUNTER_APP_ID, APP_ID_SETTINGS};

  AppInstallId desired_order[] = {
    // Quick Launch only
    APP_ID_QUIET_TIME_TOGGLE,
    // Override apps
    APP_ID_SPORTS,
    APP_ID_GOLF,
    // Storage (smallest first) defined by `storage_order`
    APP_ID_NOTIFICATIONS,
    BG_COUNTER_APP_ID,
    APP_ID_SETTINGS,
    // Record (smallest first) defined by
    // `tests/overrides/fake_app_registry/shell/system_app_registry_list.auto.h`
    APP_ID_TICTOC,
    APP_ID_MUSIC,
    APP_ID_ALARMS,
    APP_ID_WATCHFACES,
    APP_ID_WORKOUT,
    // Install ID (smallest first)
    MENU_LAYER_APP_ID,
    BIG_TIME_APP_ID,
  };

  _Static_assert(MENU_LAYER_APP_ID < BIG_TIME_APP_ID,
                 "MENU_LAYER_APP_ID is unexpectedly >= BIG_TIME_APP_ID.");

  const uint8_t num_entries = ARRAY_LENGTH(storage_order);
  prv_write_order_to_file(storage_order, num_entries);

  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = everything_filter_callback,
  }, &menu_layer);
  const uint16_t num_apps = app_menu_data_source_get_count(&data_source);

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, desired_order[i]);
  }

  app_menu_data_source_deinit(&data_source);
}

void test_app_menu_data_source__settings_app_floats_to_top_if_absent_from_storage_order(void) {
  AppInstallId storage_order[] = {APP_ID_NOTIFICATIONS, BG_COUNTER_APP_ID, APP_ID_MUSIC};

  AppInstallId desired_order[] = {
    // Settings floats above storage entries since it's absent in the storage order
    APP_ID_SETTINGS,
    // Storage (smallest first) defined by `storage_order`
    APP_ID_NOTIFICATIONS,
    BG_COUNTER_APP_ID,
    APP_ID_MUSIC,
    // Record (smallest first) defined by
    // `tests/overrides/fake_app_registry/shell/system_app_registry_list.auto.h`
    APP_ID_ALARMS,
    APP_ID_WATCHFACES,
    APP_ID_WORKOUT,
    // Install ID (smallest first)
    // Note: BIG_TIME_APP_ID is excluded because it's a watchface (PROCESS_INFO_WATCH_FACE)
    // and app_filter_callback filters out watchfaces
    MENU_LAYER_APP_ID,
  };

  const uint8_t num_entries = ARRAY_LENGTH(storage_order);
  prv_write_order_to_file(storage_order, num_entries);

  app_menu_data_source_init(&data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_layer_reload_data,
    .filter = app_filter_callback,
  }, &menu_layer);
  const uint16_t num_apps = app_menu_data_source_get_count(&data_source);

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data_source, i);
    cl_assert_equal_i(node->install_id, desired_order[i]);
  }

  app_menu_data_source_deinit(&data_source);
}

int prv_app_node_comparator(void *app_node_ref, void *new_node_ref);

void test_app_menu_data_source__app_node_comparator_equality_cases(void) {
  // Test handling of storage and record equality cases
  AppMenuNode app_menu_nodes[] = {{
    .install_id = APP_ID_ALARMS,
    .storage_order = 0,
    .record_order = 3,
  }, {
    .install_id = APP_ID_TICTOC,
    .storage_order = 0,
    .record_order = 3,
  }, {
    .install_id = APP_ID_NOTIFICATIONS,
    .storage_order = 1,
    .record_order = 0,
  }, {
    .install_id = APP_ID_SETTINGS,
    .storage_order = 2,
    .record_order = 1,
  }, {
    .install_id = APP_ID_WATCHFACES,
    .storage_order = 0,
    .record_order = 4,
  }, {
    .install_id = APP_ID_WORKOUT,
    .storage_order = 0,
    .record_order = 5,
  }};

  AppInstallId desired_order[] = {
    APP_ID_NOTIFICATIONS,
    APP_ID_SETTINGS,
    APP_ID_TICTOC,
    APP_ID_ALARMS,
    APP_ID_WATCHFACES,
    APP_ID_WORKOUT,
  };

  AppMenuNode *app_list = NULL;
  const uint16_t num_apps = ARRAY_LENGTH(app_menu_nodes);
  for (uint16_t i = 0; i < num_apps; i++) {
    app_list = (AppMenuNode *)list_sorted_add(&app_list->node, &app_menu_nodes[i].node,
                                              prv_app_node_comparator, true /* ascending */);
  }

  for (uint16_t i = 0; i < num_apps; i++) {
    AppMenuNode *node = (AppMenuNode *)list_get_at(&app_list->node, i);
    cl_assert_equal_i(node->install_id, desired_order[i]);
  }
}
