/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "ble_hrm_reminder_popup.h"

#include <pbl/drivers/rtc.h>
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/notifications/notifications.h"
#include "pbl/services/timeline/timeline_resources.h"

void ble_hrm_push_reminder_popup(void) {
  AttributeList attr_list = {};

  const char *body = i18n_get("Your heart rate has been shared with an app on your phone for "
                              "several hours. This could affect your battery. Stop sharing now?",
                              &attr_list);
  attribute_list_add_cstring(&attr_list, AttributeIdBody, body);
  attribute_list_add_uint32(&attr_list, AttributeIdIconTiny,
                            TIMELINE_RESOURCE_BLE_HRM_SHARING);
  attribute_list_add_uint8(&attr_list, AttributeIdBgColor, GColorOrangeARGB8);

  AttributeList dismiss_action_attr_list = {};
  attribute_list_add_cstring(&dismiss_action_attr_list, AttributeIdTitle,
                             i18n_get("Dismiss", &attr_list));

  AttributeList stop_action_attr_list = {};
  attribute_list_add_cstring(&stop_action_attr_list, AttributeIdTitle,
                             i18n_get("Stop Sharing Heart Rate", &attr_list));

  TimelineItemActionGroup action_group = {
    .num_actions = 2,
    .actions = (TimelineItemAction[]) {
      {
        .id = 0,
        .type = TimelineItemActionTypeDismiss,
        .attr_list = dismiss_action_attr_list,
      },
      {
        .id = 1,
        .type = TimelineItemActionTypeBLEHRMStopSharing,
        .attr_list = stop_action_attr_list,
      },
    },
  };

  TimelineItem *item = timeline_item_create_with_attributes(rtc_get_time(), 0,
                                                            TimelineItemTypeNotification,
                                                            LayoutIdNotification, &attr_list,
                                                            &action_group);
  i18n_free_all(&attr_list);
  attribute_list_destroy_list(&attr_list);
  attribute_list_destroy_list(&dismiss_action_attr_list);
  attribute_list_destroy_list(&stop_action_attr_list);

  notifications_add_notification(item);

  timeline_item_destroy(item);
}
