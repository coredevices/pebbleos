/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_notifications.h"

#include "app_glance_structured.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "pbl/services/notifications/notification_storage.h"
#include "pbl/services/timeline/attribute.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/string.h"
#include "pbl/util/struct.h"

typedef struct LauncherAppGlanceNotifications {
  char title[APP_NAME_SIZE_BYTES];
  char subtitle[ATTRIBUTE_APP_GLANCE_SUBTITLE_MAX_LEN];
  KinoReel *icon;
  EventServiceInfo notification_event_info;
} LauncherAppGlanceNotifications;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceNotifications *notifications_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(notifications_glance, icon, NULL);
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceNotifications *notifications_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(notifications_glance, title, NULL);
}

static void prv_notifications_glance_subtitle_dynamic_text_node_update(
    PBL_UNUSED GContext *ctx, PBL_UNUSED GTextNode *node, PBL_UNUSED const GRect *box,
    PBL_UNUSED const GTextNodeDrawConfig *config, PBL_UNUSED bool render, char *buffer, size_t buffer_size,
    void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceNotifications *notifications_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (notifications_glance) {
    strncpy(buffer, notifications_glance->subtitle, buffer_size);
    buffer[buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  return launcher_app_glance_structured_create_subtitle_text_node(
      structured_glance, prv_notifications_glance_subtitle_dynamic_text_node_update);
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceNotifications *notifications_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (notifications_glance) {
    event_service_client_unsubscribe(&notifications_glance->notification_event_info);
    kino_reel_destroy(notifications_glance->icon);
  }
  app_free(notifications_glance);
}

static bool prv_notification_iterator_cb(void *data, SerializedTimelineItemHeader *header_id) {
  Uuid *last_notification_received_id = data;

  // The iterator proceeds from the first notification received to the last notification received,
  // so copy the ID of the current notification and then return true so we iterate until the end.
  // Thus the last ID we save will be the last notification received.
  *last_notification_received_id = header_id->common.id;

  return true;
}

static void prv_update_glance_for_last_notification_received(
    LauncherAppGlanceNotifications *notifications_glance) {
  // Find the ID of the last notification received
  Uuid last_notification_received_id;
  notification_storage_iterate(prv_notification_iterator_cb, &last_notification_received_id);

  TimelineItem notification;
  if (!notification_storage_get(&last_notification_received_id, &notification)) {
    // We couldn't load the notification for some reason; just bail out with the subtitle cleared
    notifications_glance->subtitle[0] = '\0';
    return;
  }

  const char *title = attribute_get_string(&notification.attr_list, AttributeIdTitle, "");
  const char *subtitle = attribute_get_string(&notification.attr_list, AttributeIdSubtitle, "");
  const char *body = attribute_get_string(&notification.attr_list, AttributeIdBody, "");

  // Determine which string we should use in the glance subtitle
  const char *string_to_use_in_glance_subtitle = "";
  if (!IS_EMPTY_STRING(title)) {
    string_to_use_in_glance_subtitle = title;
  } else if (!IS_EMPTY_STRING(subtitle)) {
    // Fallback to the subtitle
    string_to_use_in_glance_subtitle = subtitle;
  } else {
    // Fallback to the body
    string_to_use_in_glance_subtitle = body;
  }

  // Copy the string to the glance
  const size_t glance_subtitle_size = sizeof(notifications_glance->subtitle);
  strncpy(notifications_glance->subtitle, string_to_use_in_glance_subtitle, glance_subtitle_size);
  notifications_glance->subtitle[glance_subtitle_size - 1] = '\0';
}

static void prv_notification_event_handler(PebbleEvent *event, void *context) {
  LauncherAppGlanceStructured *structured_glance = context;
  LauncherAppGlanceNotifications *notifications_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  switch (event->sys_notification.type) {
    case NotificationAdded:
    case NotificationRemoved:
      prv_update_glance_for_last_notification_received(notifications_glance);
      // Broadcast to the service that we changed the glance
      launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
      return;
    case NotificationActedUpon:
    case NotificationActionResult:
      return;
  }
  WTF;
}

static const LauncherAppGlanceStructuredImpl s_notifications_structured_glance_impl = {
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_notifications_create(const AppMenuNode *node) {
  PBL_ASSERTN(node);

  LauncherAppGlanceNotifications *notifications_glance =
      app_zalloc_check(sizeof(*notifications_glance));

  // Copy the name of the Notifications app as the title
  const size_t title_size = sizeof(notifications_glance->title);
  strncpy(notifications_glance->title, node->name, title_size);
  notifications_glance->title[title_size - 1] = '\0';

  // Create the icon for the Notifications app
  notifications_glance->icon = kino_reel_create_with_resource_system(node->app_num,
                                                                     node->icon_resource_id);
  PBL_ASSERTN(notifications_glance->icon);

  const bool should_consider_slices = false;
  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_notifications_structured_glance_impl,
                                            should_consider_slices, notifications_glance);
  PBL_ASSERTN(structured_glance);

  // Get the first state of the glance
  prv_update_glance_for_last_notification_received(notifications_glance);

  // Subscribe to notification events for updating the glance
  notifications_glance->notification_event_info = (EventServiceInfo) {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .handler = prv_notification_event_handler,
    .context = structured_glance,
  };
  event_service_client_subscribe(&notifications_glance->notification_event_info);

  return &structured_glance->glance;
}
