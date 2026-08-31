/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/notifications/notifications.h"

#include "pbl/services/notifications/notification_storage.h"

#include "util/bitset.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"

#include "pbl/services/analytics/analytics.h"
#include "pbl/services/vibes/vibe_intensity.h"

static void prv_notification_migration_iterator_callback(TimelineItem *notification,
    SerializedTimelineItemHeader *header, void *data) {
  header->common.timestamp -= *((int*)data);
  notification->header.timestamp = header->common.timestamp;
}

void notifications_handle_notification_action_result(
    PebbleSysNotificationActionResult *action_result) {
  PebbleEvent launcher_event = {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .sys_notification = {
      .type = NotificationActionResult,
      .action_result = action_result,
    }
  };
  // event loop will free memory of action_result
  event_put(&launcher_event);
}

void notifications_handle_notification_removed(Uuid *notification_id) {
  Uuid *removed_id = kernel_malloc_check(sizeof(Uuid));
  *removed_id = *notification_id;
  PebbleEvent launcher_event = {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .sys_notification = {
      .type = NotificationRemoved,
      .notification_id = removed_id,
    }
  };
  event_put(&launcher_event);
}

void notifications_handle_notification_added(Uuid *notification_id) {
  PebbleEvent launcher_event = {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .sys_notification = {
      .type = NotificationAdded,
      .notification_id = notification_id
    }
  };
  event_put(&launcher_event);
  PBL_ANALYTICS_ADD(notification_received_count, 1);
}

void notifications_handle_notification_acted_upon(Uuid *notification_id) {
  PebbleEvent launcher_event = {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .sys_notification = {
      .type = NotificationActedUpon,
      .notification_id = notification_id
    }
  };
  event_put(&launcher_event);
}

void notifications_migrate_timezone(const int tz_diff) {
  notification_storage_rewrite(prv_notification_migration_iterator_callback, (void*)&tz_diff);
}

void notification_storage_init(void);
void vibe_intensity_init(void);

void notifications_init(void) {
  notification_storage_init();
}

void notifications_add_notification(TimelineItem *notification) {
  notification_storage_store(notification);

  Uuid *uuid = kernel_malloc_check(sizeof(Uuid));
  *uuid = notification->header.id;
  notifications_handle_notification_added(uuid);
}
