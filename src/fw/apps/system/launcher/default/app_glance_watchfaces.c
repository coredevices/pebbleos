/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_watchfaces.h"

#include "app_glance_structured.h"

#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "shell/normal/watchface.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/struct.h"

typedef struct LauncherAppGlanceWatchfaces {
  char title[APP_NAME_SIZE_BYTES];
  char subtitle[APP_NAME_SIZE_BYTES];
  KinoReel *icon;
} LauncherAppGlanceWatchfaces;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWatchfaces *watchfaces_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(watchfaces_glance, icon, NULL);
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWatchfaces *watchfaces_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(watchfaces_glance, title, NULL);
}

static void prv_watchfaces_glance_subtitle_dynamic_text_node_update(
    PBL_UNUSED GContext *ctx, PBL_UNUSED GTextNode *node, PBL_UNUSED const GRect *box,
    PBL_UNUSED const GTextNodeDrawConfig *config, PBL_UNUSED bool render, char *buffer, size_t buffer_size,
    void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceWatchfaces *watchfaces_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (watchfaces_glance) {
    strncpy(buffer, watchfaces_glance->subtitle, buffer_size);
    buffer[buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  return launcher_app_glance_structured_create_subtitle_text_node(
      structured_glance, prv_watchfaces_glance_subtitle_dynamic_text_node_update);
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWatchfaces *watchfaces_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (watchfaces_glance) {
    kino_reel_destroy(watchfaces_glance->icon);
  }
  app_free(watchfaces_glance);
}

static void prv_update_active_watchface_title(LauncherAppGlanceWatchfaces *watchfaces_glance) {
  const AppInstallId selected_watch_id = watchface_get_default_install_id();

  AppInstallEntry entry;
  if (app_install_get_entry_for_install_id(selected_watch_id, &entry)) {
    const size_t watchfaces_subtitle_size = sizeof(watchfaces_glance->subtitle);
    strncpy(watchfaces_glance->subtitle, entry.name, watchfaces_subtitle_size);
    watchfaces_glance->subtitle[watchfaces_subtitle_size - 1] = '\0';
  }
}

static const LauncherAppGlanceStructuredImpl s_watchfaces_structured_glance_impl = {
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_watchfaces_create(const AppMenuNode *node) {
  PBL_ASSERTN(node);

  LauncherAppGlanceWatchfaces *watchfaces_glance = app_zalloc_check(sizeof(*watchfaces_glance));

  // Copy the name of the Watchfaces app as the title
  const size_t title_size = sizeof(watchfaces_glance->title);
  strncpy(watchfaces_glance->title, node->name, title_size);
  watchfaces_glance->title[title_size - 1] = '\0';

  // Create the icon for the Watchfaces app
  watchfaces_glance->icon = kino_reel_create_with_resource_system(node->app_num,
                                                                  node->icon_resource_id);
  PBL_ASSERTN(watchfaces_glance->icon);

  // Update the active watchface title in the glance's subtitle
  prv_update_active_watchface_title(watchfaces_glance);

  const bool should_consider_slices = false;
  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_watchfaces_structured_glance_impl,
                                            should_consider_slices, watchfaces_glance);
  PBL_ASSERTN(structured_glance);

  return &structured_glance->glance;
}
