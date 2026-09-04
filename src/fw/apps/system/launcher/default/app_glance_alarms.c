/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_alarms.h"

#include "app_glance_structured.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/clock.h"
#include "pbl/services/alarms/alarm.h"
#include "pbl/services/timeline/attribute.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/struct.h"

#include <stdio.h>

typedef struct LauncherAppGlanceAlarms {
  char title[APP_NAME_SIZE_BYTES];
  char subtitle[ATTRIBUTE_APP_GLANCE_SUBTITLE_MAX_LEN];
  KinoReel *icon;
  uint32_t icon_resource_id;
  uint32_t default_icon_resource_id;
  EventServiceInfo alarm_clock_event_info;
} LauncherAppGlanceAlarms;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceAlarms *alarms_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(alarms_glance, icon, NULL);
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceAlarms *alarms_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(alarms_glance, title, NULL);
}

static void prv_alarms_glance_subtitle_dynamic_text_node_update(
    PBL_UNUSED GContext *ctx, PBL_UNUSED GTextNode *node, PBL_UNUSED const GRect *box,
    PBL_UNUSED const GTextNodeDrawConfig *config, PBL_UNUSED bool render, char *buffer, size_t buffer_size,
    void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceAlarms *alarms_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (alarms_glance) {
    strncpy(buffer, alarms_glance->subtitle, buffer_size);
    buffer[buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  return launcher_app_glance_structured_create_subtitle_text_node(
      structured_glance, prv_alarms_glance_subtitle_dynamic_text_node_update);
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceAlarms *alarms_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (alarms_glance) {
    event_service_client_unsubscribe(&alarms_glance->alarm_clock_event_info);
    kino_reel_destroy(alarms_glance->icon);
  }
  app_free(alarms_glance);
}

static void prv_set_glance_icon(LauncherAppGlanceAlarms *alarms_glance,
                                uint32_t new_icon_resource_id) {
  if (alarms_glance->icon_resource_id == new_icon_resource_id) {
    // Nothing to do, bail out
    return;
  }

  // Destroy the existing icon
  kino_reel_destroy(alarms_glance->icon);

  // Set the new icon and record its resource ID
  alarms_glance->icon = kino_reel_create_with_resource(new_icon_resource_id);
  PBL_ASSERTN(alarms_glance->icon);
  alarms_glance->icon_resource_id = new_icon_resource_id;
}

//! If alarm is for today, alarm text should look like "5:43 PM" (12 hr) or "17:43" (24 hr)
//! If alarm is not for today, text should look like "Fri, 11:30 PM" (12 hr) or "Fri, 23:30" (24 hr)
//! If no alarms are set, the alarm text should be the empty string ""
static void prv_update_glance_for_next_alarm(LauncherAppGlanceAlarms *alarms_glance) {
  // Start by assuming we'll set the default icon
  uint32_t new_icon_resource_id = alarms_glance->default_icon_resource_id;

  time_t alarm_time_epoch;
  if (!alarm_get_next_enabled_alarm(&alarm_time_epoch)) {
    // Clear the alarm text if there are no alarms set
    alarms_glance->subtitle[0] = '\0';
  } else {
    // If the next alarm is smart, use the smart alarm icon
    if (alarm_is_next_enabled_alarm_smart()) {
      // TODO PBL-39113: Replace this placeholder with a better smart alarm icon for the glance
      new_icon_resource_id = RESOURCE_ID_SMART_ALARM_TINY;
    }

    char time_buffer[TIME_STRING_REQUIRED_LENGTH] = {};
    clock_copy_time_string_timestamp(time_buffer, sizeof(time_buffer), alarm_time_epoch);

    // Determine if the alarm is for today
    const time_t current_time = rtc_get_time();
    const time_t today_midnight = time_util_get_midnight_of(current_time);
    const time_t alarm_midnight = time_util_get_midnight_of(alarm_time_epoch);
    const bool is_alarm_for_today = (alarm_midnight == today_midnight);

    const size_t alarm_subtitle_size = sizeof(alarms_glance->subtitle);

    // Only show the day of the week if the alarm is not for today
    if (!is_alarm_for_today) {
      // Get a string for the abbreviated day of the week in the user's locale
      char day_buffer[TIME_STRING_REQUIRED_LENGTH] = {};
      struct tm alarm_time;
      localtime_r(&alarm_time_epoch, &alarm_time);
      strftime(day_buffer, sizeof(day_buffer), "%a", &alarm_time);

      snprintf(alarms_glance->subtitle, alarm_subtitle_size, "%s, %s", day_buffer, time_buffer);
    } else {
      strncpy(alarms_glance->subtitle, time_buffer, alarm_subtitle_size);
      alarms_glance->subtitle[alarm_subtitle_size - 1] = '\0';
    }
  }

  // Update the icon
  prv_set_glance_icon(alarms_glance, new_icon_resource_id);
}

static void prv_alarm_clock_event_handler(PBL_UNUSED PebbleEvent *event, void *context) {
  LauncherAppGlanceStructured *structured_glance = context;
  LauncherAppGlanceAlarms *alarms_glance =
      launcher_app_glance_structured_get_data(structured_glance);

  prv_update_glance_for_next_alarm(alarms_glance);

  // Broadcast to the service that we changed the glance
  launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
}

static const LauncherAppGlanceStructuredImpl s_alarms_structured_glance_impl = {
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_alarms_create(const AppMenuNode *node) {
  PBL_ASSERTN(node);

  LauncherAppGlanceAlarms *alarms_glance = app_zalloc_check(sizeof(*alarms_glance));

  // Copy the name of the Alarms app as the title
  const size_t title_size = sizeof(alarms_glance->title);
  strncpy(alarms_glance->title, node->name, title_size);
  alarms_glance->title[title_size - 1] = '\0';

  // Save the default app icon resource ID
  alarms_glance->default_icon_resource_id = node->icon_resource_id;

  const bool should_consider_slices = false;
  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_alarms_structured_glance_impl,
                                            should_consider_slices, alarms_glance);
  PBL_ASSERTN(structured_glance);

  // Get the first state of the glance
  prv_update_glance_for_next_alarm(alarms_glance);

  // Subscribe to alarm clock events for updating the glance
  alarms_glance->alarm_clock_event_info = (EventServiceInfo) {
    .type = PEBBLE_ALARM_CLOCK_EVENT,
    .handler = prv_alarm_clock_event_handler,
    .context = structured_glance,
  };
  event_service_client_subscribe(&alarms_glance->alarm_clock_event_info);

  return &structured_glance->glance;
}
