/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/ui/vibes.h"
#include "applib/ui/window_private.h"
#include "apps/system/launcher/default/menu_layer.h"
#include "shell/prefs.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/app_glances/app_glance_service.h"
#include "pbl/services/blob_db/app_glance_db.h"
#include "pbl/util/size.h"

static GContext s_ctx;

// Fakes
/////////////////////

#include "fake_content_indicator.h"
#include "fake_settings_file.h"
#include "fake_spi_flash.h"
#include "fixtures/load_test_resources.h"
#include "pbl/services/timeline/timeline_resources.h"

extern const uint16_t g_timeline_resources[][TimelineResourceSizeCount];
#define TIMELINE_RESOURCE_TEST_FAKE_PNG (9999 | 0x80000000)

//! Add more values to this enum and the array below to add new apps to the launcher in these
//! unit tests.
typedef enum LauncherMenuLayerTestApp {
  LauncherMenuLayerTestApp_Watchfaces,
  LauncherMenuLayerTestApp_LongTitle,
  LauncherMenuLayerTestApp_InteriorApp,
  LauncherMenuLayerTestApp_Travel,
  LauncherMenuLayerTestApp_NoIcon,

  LauncherMenuLayerTestAppCount
} LauncherMenuLayerTestApp;

typedef struct LauncherMenuLayerTestAppNode {
  AppMenuNode node;
  uint32_t bitmap_icon_resource_id;
  uint32_t pdc_icon_resource_id;
  uint32_t bitmap_slice_icon_resource_id;
  uint32_t pdc_slice_icon_resource_id;
} LauncherMenuLayerTestAppNode;

static bool s_use_pdc_icons;

static const LauncherMenuLayerTestAppNode s_fake_app_nodes[LauncherMenuLayerTestAppCount] = {
  [LauncherMenuLayerTestApp_Watchfaces] = {
    .node = {
      .name = "Watchfaces",
      .uuid = (Uuid) {0xc3, 0xcf, 0xda, 0xa9, 0x76, 0x1f, 0x49, 0x89,
                      0x99, 0x4c, 0x30, 0x13, 0xcd, 0xc3, 0xef, 0xb9},
    },
    .bitmap_icon_resource_id = RESOURCE_ID_MENU_LAYER_GENERIC_WATCHFACE_ICON,
    .pdc_icon_resource_id = RESOURCE_ID_ALARM_CLOCK_TINY,
    .bitmap_slice_icon_resource_id = TIMELINE_RESOURCE_TEST_FAKE_PNG,
    .pdc_slice_icon_resource_id = TIMELINE_RESOURCE_BASKETBALL,
  },
  [LauncherMenuLayerTestApp_LongTitle] = {
    .node = {
      .name = "Really really long title",
      .uuid = (Uuid) {0xd4, 0x17, 0x61, 0x3c, 0x43, 0x31, 0x44, 0x90,
                      0xa1, 0x68, 0xf2, 0x46, 0x53, 0xd3, 0x76, 0x3a},
    },
    // these icons are too big and will be replaced with the generic app icon
    .bitmap_icon_resource_id = RESOURCE_ID_SETTINGS_ICON_BLUETOOTH,
    .pdc_icon_resource_id = RESOURCE_ID_AMERICAN_FOOTBALL_SMALL,
    .bitmap_slice_icon_resource_id = TIMELINE_RESOURCE_TEST_FAKE_PNG,
    .pdc_slice_icon_resource_id = TIMELINE_RESOURCE_BASKETBALL,
  },
  [LauncherMenuLayerTestApp_InteriorApp] = {
    .node = {
      .name = "Interior App",
      .uuid = (Uuid) {0x11, 0xcf, 0xac, 0x66, 0x29, 0x9c, 0x4a, 0xa6,
                      0x94, 0x5d, 0xf0, 0x53, 0x6e, 0xd1, 0x4e, 0xe8},
    },
    // these icons are too big and will be replaced with the generic app icon
    .bitmap_icon_resource_id = RESOURCE_ID_SETTINGS_ICON_BLUETOOTH_ALT,
    .pdc_icon_resource_id = RESOURCE_ID_BASEBALL_GAME_SMALL,
    .bitmap_slice_icon_resource_id = TIMELINE_RESOURCE_TEST_FAKE_PNG,
    .pdc_slice_icon_resource_id = TIMELINE_RESOURCE_BASKETBALL,
  },
  [LauncherMenuLayerTestApp_Travel] = {
    .node = {
      .name = "Travel",
      .uuid = (Uuid) {0x27, 0x53, 0xd0, 0xc, 0x65, 0xbb, 0x41, 0x83,
                      0x9c, 0xf1, 0x17, 0x3e, 0x6, 0xdf, 0xda, 0xde},
    },
    .bitmap_icon_resource_id = RESOURCE_ID_SETTINGS_ICON_AIRPLANE,
    .pdc_icon_resource_id = RESOURCE_ID_SCHEDULED_FLIGHT_TINY,
    .bitmap_slice_icon_resource_id = TIMELINE_RESOURCE_TEST_FAKE_PNG,
    .pdc_slice_icon_resource_id = TIMELINE_RESOURCE_BASKETBALL,
  },
  [LauncherMenuLayerTestApp_NoIcon] = {
    .node = {
      .name = "No Icon",
      .uuid = (Uuid) {0x7f, 0x4f, 0xc1, 0x32, 0x32, 0x78, 0x47, 0xec,
                      0x91, 0x64, 0xf1, 0x76, 0xf9, 0xea, 0x1f, 0xc2},
    },
  }
};

AppMenuNode* app_menu_data_source_get_node_at_index(AppMenuDataSource *source,
                                                    uint16_t row_index) {
  cl_assert(source);
  cl_assert(row_index < ARRAY_LENGTH(s_fake_app_nodes));
  const LauncherMenuLayerTestAppNode *test_node = &s_fake_app_nodes[row_index];
  static AppMenuNode node_copy;
  node_copy = test_node->node;
  node_copy.icon_resource_id =
      s_use_pdc_icons ? test_node->pdc_icon_resource_id : test_node->bitmap_icon_resource_id;
  return &node_copy;
}

uint16_t app_menu_data_source_get_count(AppMenuDataSource *source) {
  cl_assert(source);
  return ARRAY_LENGTH(s_fake_app_nodes);
}

static GBitmap s_default_app_icon_bitmap;
void app_menu_data_source_enable_icons(AppMenuDataSource *source, uint32_t fallback_icon_id) {
  cl_assert(source);
  source->show_icons = true;
  gbitmap_deinit(&s_default_app_icon_bitmap);
  gbitmap_init_with_resource_system(&s_default_app_icon_bitmap, SYSTEM_APP, fallback_icon_id);
  source->default_icon = &s_default_app_icon_bitmap;
}

static GBitmap s_app_icon_bitmap;
GBitmap *app_menu_data_source_get_node_icon(AppMenuDataSource *source, AppMenuNode *node) {
  cl_assert(source);
  cl_assert(node);

  if (!node->icon_resource_id) {
    return &s_default_app_icon_bitmap;
  }

  gbitmap_deinit(&s_app_icon_bitmap);
  gbitmap_init_with_resource(&s_app_icon_bitmap, node->icon_resource_id);
  return &s_app_icon_bitmap;
}

//! We use this function in the app glance service to create a key (the install ID) for an app
//! glance cache entry; just fake it by constructing a 32-bit number from the first 4 bytes of the
//! app's UUID
AppInstallId app_install_get_id_for_uuid(const Uuid *uuid) {
  if (!uuid) {
    return INSTALL_ID_INVALID;
  }
  return ((uuid->byte3 << 24) | (uuid->byte2 << 16) | (uuid->byte1 << 8) | uuid->byte0);
}

bool timeline_resources_get_id_system(TimelineResourceId timeline_id, TimelineResourceSize size,
                                      ResAppNum res_app_num, AppResourceInfo *res_info_out) {
  if (timeline_id == TIMELINE_RESOURCE_TEST_FAKE_PNG) {
    // random PNG resource for testing since no timeline resources use PNGs
    res_info_out->res_id = RESOURCE_ID_MUSIC_APP_GLANCE_PLAY;
  } else {
    res_info_out->res_id = g_timeline_resources[timeline_id & 0x7FFFFFFF][size];
  }
  return true;
}

bool timeline_resources_is_system(TimelineResourceId timeline_id) {
  return false;
}

// Stubs
/////////////////////

#include "stubs_app_cache.h"
#include "stubs_alarm.h"
#include "stubs_alerts.h"
#include "stubs_analytics.h"
#include "stubs_app_manager.h"
#include "stubs_app_window_stack.h"
#include "stubs_app_timer.h"
#include "stubs_battery_state_service.h"
#include "stubs_bluetooth_ctl.h"
#include "stubs_bootbits.h"
#include "stubs_click.h"
#include "stubs_clock.h"
#include "stubs_do_not_disturb.h"
#include "stubs_events.h"
#include "stubs_event_service_client.h"
#include "stubs_health_util.h"
#include "stubs_i18n.h"
#include "stubs_kino_player.h"
#include "stubs_layer.h"
#include "stubs_logging.h"
#include "stubs_memory_layout.h"
#include "stubs_music.h"
#include "stubs_mutex.h"
#include "stubs_notification_storage.h"
#include "stubs_passert.h"
#include "stubs_pebble_process_info.h"
#include "stubs_pebble_tasks.h"
#include "stubs_pbl_malloc.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "stubs_session.h"
#include "stubs_sleep.h"
#include "stubs_status_bar_layer.h"
#include "stubs_system_theme.h"
#include "stubs_syscalls.h"
#include "stubs_task_watchdog.h"
#include "stubs_tick.h"
#include "stubs_time.h"
#include "stubs_watchface.h"
#include "stubs_weather_service.h"
#include "stubs_weather_types.h"
#include "stubs_window_manager.h"
#include "stubs_window_stack.h"
#include "stubs_workout_service.h"
#include "stubs_workout_utils.h"

GColor shell_prefs_get_theme_highlight_color(void) {
  return GColorWhite;
}

bool alerts_preferences_get_notification_alternative_design(void) {
  return false;
}

MenuScrollVibeBehavior shell_prefs_get_menu_scroll_vibe_behavior(void) {
  return MenuScrollNoVibe;
}

bool shell_prefs_get_menu_scroll_wrap_around_enable(void) {
  return false;
}

PreferredContentSize system_theme_get_content_size(void) {
  return PreferredContentSizeDefault;
}

void vibes_enqueue_custom_pattern(VibePattern pattern) {}

// We can't include stubs_process_manager.h because it conflicts with the two helper includes below
void process_manager_send_callback_event_to_process(PebbleTask task, void (*callback)(void *),
                                                    void *data) {}

// Helper Functions
/////////////////////

#include "../../../graphics/test_graphics.h"
#include "../../../graphics/util.h"

// Setup and Teardown
////////////////////////////////////

static FrameBuffer *fb = NULL;

GContext *graphics_context_get_current_context(void) {
  return &s_ctx;
}

void test_launcher_menu_layer__initialize(void) {
  // Setup framebuffer and graphics context
  fb = malloc(sizeof(FrameBuffer));
  framebuffer_init(fb, &(GSize) {DISP_COLS, DISP_ROWS});
  test_graphics_context_init(&s_ctx, fb);
  graphics_context_set_antialiased(&s_ctx, true);

  // Setup resources
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(true /* write erase headers */);
  load_resource_fixture_in_flash(RESOURCES_FIXTURE_PATH, SYSTEM_RESOURCES_FIXTURE_NAME,
                                 false /* is_next */);
  resource_init();

  // Setup content indicators buffer
  ContentIndicatorsBuffer *buffer = content_indicator_get_current_buffer();
  content_indicator_init_buffer(buffer);

  // Setup AppGlanceDB
  fake_settings_file_reset();
  app_glance_db_init();

  // Setup AppGlanceService
  app_glance_service_init();

  // Default to showing bitmap icons
  s_use_pdc_icons = false;
}

void app_glance_db_deinit(void);

void test_launcher_menu_layer__cleanup(void) {
  app_glance_db_deinit();
  gbitmap_deinit(&s_app_icon_bitmap);
  gbitmap_deinit(&s_default_app_icon_bitmap);
  free(fb);
}

// Helpers
//////////////////////

//! Declared T_STATIC in launcher_menu_layer.c so we can easily change the launcher's selected index
//! from unit tests without also specifying the y offset for the scroll layer that is required by
//! `launcher_menu_layer_set_selection_state()`.
void prv_launcher_menu_layer_set_selection_index(LauncherMenuLayer *launcher_menu_layer,
                                                 uint16_t index, MenuRowAlign row_align,
                                                 bool animated);

void prv_render_launcher_menu_layer(uint16_t selected_index) {
  AppMenuDataSource data_source = {};
  app_menu_data_source_init(&data_source, NULL, NULL);
  app_menu_data_source_enable_icons(&data_source, RESOURCE_ID_MENU_LAYER_GENERIC_WATCHAPP_ICON);

  LauncherMenuLayer launcher_menu_layer = {};
  launcher_menu_layer_init(&launcher_menu_layer, &data_source);
  const bool animated = false;
  // If we used MenuRowAlignCenter on rect then the test images would show the top and bottom
  // rows being clipped by the edge of the screen
  const MenuRowAlign row_align = PBL_IF_RECT_ELSE(MenuRowAlignTop, MenuRowAlignCenter);
  prv_launcher_menu_layer_set_selection_index(&launcher_menu_layer, selected_index,
                                              row_align, animated);

  layer_render_tree(launcher_menu_layer_get_layer(&launcher_menu_layer), &s_ctx);

  launcher_menu_layer_deinit(&launcher_menu_layer);
  app_menu_data_source_deinit(&data_source);
}

// Tests
//////////////////////

void test_launcher_menu_layer__long_title(void) {
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_LongTitle);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__no_icon(void) {
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_NoIcon);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__interior_app(void) {
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_InteriorApp);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__no_icon_app_with_glance(void) {
  // Insert a glance with a slice for the app that doesn't have a default icon
  const AppGlance glance = (AppGlance) {
      .num_slices = 1,
      .slices = {
        {
          .expiration_time = 1464734484, // (Tue, 31 May 2016 22:41:24 GMT)
          .type = AppGlanceSliceType_IconAndSubtitle,
          .icon_and_subtitle = {
            .icon_resource_id = TIMELINE_RESOURCE_SCHEDULED_FLIGHT,
            .template_string = "Glances baby!",
          },
        },
      },
  };
  cl_assert_equal_i(app_glance_db_insert_glance(
      &s_fake_app_nodes[LauncherMenuLayerTestApp_NoIcon].node.uuid, &glance), S_SUCCESS);

  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_NoIcon);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

static void prv_insert_glances_for_app_selected_and_apps_above_and_below_with_glances_test(void) {
  // Insert glances with 1 slice for the app above the interior app, the interior app, and the app
  // below the interior app so we can see that the subtitle is positioned properly in all 3 cases
  cl_assert(LauncherMenuLayerTestApp_InteriorApp > 0);
  for (LauncherMenuLayerTestApp i = LauncherMenuLayerTestApp_InteriorApp - 1;
       i <= LauncherMenuLayerTestApp_InteriorApp + 1; i++) {
    const LauncherMenuLayerTestAppNode *test_node = &s_fake_app_nodes[i];
    const uint32_t icon_resource_id = s_use_pdc_icons ? test_node->pdc_slice_icon_resource_id :
                                                        test_node->bitmap_slice_icon_resource_id;
    AppGlance glance = (AppGlance) {
      .num_slices = 1,
      .slices = {
        {
          .expiration_time = 1464734484, // (Tue, 31 May 2016 22:41:24 GMT)
          .type = AppGlanceSliceType_IconAndSubtitle,
          .icon_and_subtitle = {
            // Just continue using their default icon, we care more about the subtitle in this test
            .icon_resource_id = icon_resource_id,
          },
        },
      },
    };
    snprintf(glance.slices[0].icon_and_subtitle.template_string,
             sizeof(glance.slices[0].icon_and_subtitle.template_string),
             "%s glance", s_fake_app_nodes[i].node.name);
    cl_assert_equal_i(app_glance_db_insert_glance(&s_fake_app_nodes[i].node.uuid, &glance),
                      S_SUCCESS);
  }
}

void test_launcher_menu_layer__app_selected_and_apps_above_and_below_with_glances(void) {
  prv_insert_glances_for_app_selected_and_apps_above_and_below_with_glances_test();
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_InteriorApp);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__long_title_pdc(void) {
  s_use_pdc_icons = true;
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_LongTitle);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__no_icon_pdc(void) {
  s_use_pdc_icons = true;
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_NoIcon);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__interior_app_pdc(void) {
  s_use_pdc_icons = true;
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_InteriorApp);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__no_icon_app_with_glance_pdc(void) {
  s_use_pdc_icons = true;
  // Insert a glance with a slice for the app that doesn't have a default icon
  const AppGlance glance = (AppGlance) {
    .num_slices = 1,
    .slices = {
      {
        .expiration_time = 1464734484, // (Tue, 31 May 2016 22:41:24 GMT)
        .type = AppGlanceSliceType_IconAndSubtitle,
        .icon_and_subtitle = {
          .icon_resource_id = TIMELINE_RESOURCE_SCHEDULED_FLIGHT,
          .template_string = "Glances baby!",
        },
      },
    },
  };
  cl_assert_equal_i(app_glance_db_insert_glance(
      &s_fake_app_nodes[LauncherMenuLayerTestApp_NoIcon].node.uuid, &glance), S_SUCCESS);

  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_NoIcon);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_launcher_menu_layer__app_selected_and_apps_above_and_below_with_glances_pdc(void) {
  s_use_pdc_icons = true;
  prv_insert_glances_for_app_selected_and_apps_above_and_below_with_glances_test();
  prv_render_launcher_menu_layer(LauncherMenuLayerTestApp_InteriorApp);
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}
