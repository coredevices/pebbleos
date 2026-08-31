/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_workout.h"

#include "app_glance_structured.h"

#include "applib/template_string.h"
#include "apps/system/workout/utils.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/health_util.h"
#include "pbl/services/activity/workout_service.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/string.h"
#include "pbl/util/struct.h"

#define MAX_SUBTITLE_BUFFER_SIZE (16)

typedef struct LauncherAppGlanceWorkout {
  char title[APP_NAME_SIZE_BYTES];
  char subtitle[MAX_SUBTITLE_BUFFER_SIZE];
  KinoReel *icon;
  uint32_t icon_resource_id;
  AppTimer *timer;
} LauncherAppGlanceWorkout;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWorkout *workout_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(workout_glance, icon, NULL);
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWorkout *workout_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(workout_glance, title, NULL);
}

static void prv_workout_glance_subtitle_dynamic_text_node_update(
    PBL_UNUSED GContext *ctx, PBL_UNUSED GTextNode *node, PBL_UNUSED const GRect *box,
    PBL_UNUSED const GTextNodeDrawConfig *config, PBL_UNUSED bool render, char *buffer, size_t buffer_size,
    void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceWorkout *workout_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (workout_glance) {
    strncpy(buffer, workout_glance->subtitle, buffer_size);
    buffer[buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  return launcher_app_glance_structured_create_subtitle_text_node(
      structured_glance, prv_workout_glance_subtitle_dynamic_text_node_update);
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceWorkout *workout_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (workout_glance) {
    kino_reel_destroy(workout_glance->icon);
    app_timer_cancel(workout_glance->timer);
  }
  app_free(workout_glance);
}

static bool prv_set_glance_icon(LauncherAppGlanceWorkout *workout_glance,
                                uint32_t new_icon_resource_id) {
  if (workout_glance->icon_resource_id == new_icon_resource_id) {
    // Nothing to do, bail out
    return false;
  }

  // Destroy the existing icon
  kino_reel_destroy(workout_glance->icon);

  // Set the new icon and record its resource ID
  workout_glance->icon = kino_reel_create_with_resource(new_icon_resource_id);
  PBL_ASSERTN(workout_glance->icon);
  workout_glance->icon_resource_id = new_icon_resource_id;

  return true;
}

static uint32_t prv_get_workout_icon_resource_id_for_type(ActivitySessionType type) {
  switch (type) {
    case ActivitySessionType_Open:
      return RESOURCE_ID_WORKOUT_APP_HEART;
    case ActivitySessionType_Walk:
      return RESOURCE_ID_WORKOUT_APP_WALK_TINY;
    case ActivitySessionType_Run:
      return RESOURCE_ID_WORKOUT_APP_RUN_TINY;
    case ActivitySessionType_Sleep:
    case ActivitySessionType_RestfulSleep:
    case ActivitySessionType_Nap:
    case ActivitySessionType_RestfulNap:
    case ActivitySessionTypeCount:
    case ActivitySessionType_None:
      break;
  }

  WTF;
}

static void prv_timer_callback(void *data) {
  LauncherAppGlanceStructured *structured_glance = data;
  LauncherAppGlanceWorkout *workout_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  PBL_ASSERTN(workout_glance);

  ActivitySession automatic_session = {};
  const bool has_automatic_session =
      workout_utils_find_ongoing_activity_session(&automatic_session);

  ActivitySessionType workout_type;
  int32_t workout_duration_s = 0;

  if (workout_service_is_workout_ongoing()) {
    // Manual workout is going on - get the type and duration
    workout_service_get_current_workout_type(&workout_type);
    workout_service_get_current_workout_info(NULL, &workout_duration_s, NULL, NULL, NULL);
  } else if (has_automatic_session) {
    // Automatic workout is going on - get the type and duration
    workout_type = automatic_session.type;
    workout_duration_s = rtc_get_time() - automatic_session.start_utc;
  } else {
    // No workout is going on
    bool glance_changed = false;

    // Set the icon back to default if it isn't already
    if (prv_set_glance_icon(workout_glance, RESOURCE_ID_ACTIVITY_TINY)) {
      glance_changed = true;
    }

    // Clear subtitle if it isn't already
    if (!IS_EMPTY_STRING(workout_glance->subtitle)) {
      memset(workout_glance->subtitle, 0, sizeof(workout_glance->subtitle));
      glance_changed = true;
    }

    // Broadcast to the service that we changed the glance if it was changed
    if (glance_changed) {
      launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
    }

    // Bail since no workout is going on
    return;
  }

  // Set icon for the ongoing workout type
  prv_set_glance_icon(workout_glance, prv_get_workout_icon_resource_id_for_type(workout_type));

  // Zero out the glance's subtitle buffer
  memset(workout_glance->subtitle, 0, sizeof(workout_glance->subtitle));

  // Set subtitle
  health_util_format_hours_minutes_seconds(workout_glance->subtitle,
      sizeof(workout_glance->subtitle), workout_duration_s, true, workout_glance);

  i18n_free_all(workout_glance);

  // Broadcast to the service that we changed the glance
  launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
}

static const LauncherAppGlanceStructuredImpl s_workout_structured_glance_impl = {
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_workout_create(const AppMenuNode *node) {
  PBL_ASSERTN(node);

  LauncherAppGlanceWorkout *workout_glance = app_zalloc_check(sizeof(*workout_glance));

  // Copy the name of the Workout app as the title
  const size_t title_size = sizeof(workout_glance->title);
  strncpy(workout_glance->title, node->name, title_size);
  workout_glance->title[title_size - 1] = '\0';

  const bool should_consider_slices = false;
  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_workout_structured_glance_impl,
                                            should_consider_slices, workout_glance);
  PBL_ASSERTN(structured_glance);

  // Call timer callback and register it to repeat
  prv_timer_callback(structured_glance);

  const uint32_t timer_interval_ms = 1000;
  workout_glance->timer = app_timer_register_repeatable(timer_interval_ms, prv_timer_callback,
                                                        structured_glance, true /* repeating */);

  return &structured_glance->glance;
}
