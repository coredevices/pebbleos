/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "apps/system/notifications_history.h"

#include "clar.h"
#include "fake_pbl_malloc.h"
#include "pbl/services/timeline/attribute.h"
#include "pbl/services/timeline/timeline.h"

static NotificationHistory s_history;

const char *attribute_get_string(const AttributeList *attr_list, AttributeId id,
                                 char *default_value) {
  for (uint8_t i = 0; i < attr_list->num_attributes; i++) {
    if (attr_list->attributes[i].id == id) {
      return attr_list->attributes[i].cstring;
    }
  }
  return default_value;
}

static Uuid prv_id(uint8_t value) {
  return UuidMake(value, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

static void prv_add_attribute(uint8_t id, time_t timestamp, AttributeId attribute_id,
                              const char *value) {
  static const Uuid s_android_notifications_source = UUID_NOTIFICATIONS_DATA_SOURCE;
  Attribute attribute = {
      .id = attribute_id,
      .cstring = (char *)value,
  };
  TimelineItem item = {
      .header =
          {
              .id = prv_id(id),
              .parent_id = s_android_notifications_source,
              .timestamp = timestamp,
              .type = TimelineItemTypeNotification,
          },
      .attr_list =
          {
              .num_attributes = value ? 1 : 0,
              .attributes = value ? &attribute : NULL,
          },
  };
  notifications_history_add_item(&s_history, &item);
}

static void prv_add(uint8_t id, time_t timestamp, const char *sender) {
  prv_add_attribute(id, timestamp, AttributeIdSender, sender);
}

static void prv_add_ios(uint8_t id, time_t timestamp, const char *sender) {
  Attribute sender_attribute = {
      .id = AttributeIdSender,
      .cstring = (char *)sender,
  };
  TimelineItem item = {
      .header =
          {
              .id = prv_id(id),
              .timestamp = timestamp,
              .type = TimelineItemTypeNotification,
              .ancs_notif = true,
          },
      .attr_list =
          {
              .num_attributes = 1,
              .attributes = &sender_attribute,
          },
  };
  notifications_history_add_item(&s_history, &item);
}

static NotificationHistoryRow *prv_row(uint16_t index) {
  return notifications_history_get_row(&s_history, index);
}

static void prv_assert_id(const Uuid *id, uint8_t expected) {
  Uuid expected_id = prv_id(expected);
  cl_assert(uuid_equal(id, &expected_id));
}

void test_notifications_history__initialize(void) {
  fake_pbl_malloc_clear_tracking();
  notifications_history_init(&s_history, true, 0);
}

void test_notifications_history__cleanup(void) {
  notifications_history_deinit(&s_history);
  fake_pbl_malloc_check_net_allocs();
  fake_pbl_malloc_clear_tracking();
}

void test_notifications_history__same_sender_forms_one_group(void) {
  prv_add(1, 100, "Anna");
  prv_add(2, 200, "Anna");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 1);
  cl_assert(prv_row(0)->is_group);
  cl_assert_equal_i(notifications_history_row_get_count(prv_row(0)), 2);
  cl_assert(notifications_history_row_is_collapsed_group(prv_row(0)));
  cl_assert_equal_s(prv_row(0)->group.sender, "Anna");
  prv_assert_id(notifications_history_row_get_latest_id(prv_row(0)), 2);
}

void test_notifications_history__single_message_is_not_collapsed_group(void) {
  prv_add(1, 100, "Anna");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 1);
  cl_assert(!notifications_history_row_is_collapsed_group(prv_row(0)));
  prv_assert_id(notifications_history_row_get_latest_id(prv_row(0)), 1);
}

void test_notifications_history__different_senders_form_separate_groups(void) {
  prv_add(1, 100, "Anna");
  prv_add(2, 200, "Bob");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 2);
  cl_assert_equal_s(prv_row(0)->group.sender, "Bob");
  cl_assert_equal_s(prv_row(1)->group.sender, "Anna");
  cl_assert(!notifications_history_row_is_collapsed_group(prv_row(0)));
  cl_assert(!notifications_history_row_is_collapsed_group(prv_row(1)));
}

void test_notifications_history__title_is_used_when_sender_is_missing(void) {
  prv_add_attribute(1, 100, AttributeIdTitle, "Anna");
  prv_add_attribute(2, 200, AttributeIdTitle, "Anna");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 1);
  cl_assert_equal_s(prv_row(0)->group.sender, "Anna");
  cl_assert_equal_i(prv_row(0)->group.count, 2);
}

void test_notifications_history__body_is_not_used_as_group_key(void) {
  prv_add_attribute(1, 100, AttributeIdBody, "Same body");
  prv_add_attribute(2, 200, AttributeIdBody, "Same body");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 2);
  cl_assert(!prv_row(0)->is_group);
  cl_assert(!prv_row(1)->is_group);
}

void test_notifications_history__groups_and_members_are_newest_first(void) {
  prv_add(1, 300, "Anna");
  prv_add(2, 100, "Anna");
  prv_add(3, 200, "Anna");
  prv_add(4, 250, "Bob");

  cl_assert_equal_s(prv_row(0)->group.sender, "Anna");
  NotificationHistoryMember *member = prv_row(0)->group.members;
  prv_assert_id(&member->id, 1);
  member = (NotificationHistoryMember *)list_get_next(&member->node);
  prv_assert_id(&member->id, 3);
  member = (NotificationHistoryMember *)list_get_next(&member->node);
  prv_assert_id(&member->id, 2);
  cl_assert_equal_s(prv_row(1)->group.sender, "Bob");
}

void test_notifications_history__equal_timestamps_use_insertion_order(void) {
  prv_add(1, 100, "Anna");
  prv_add(2, 100, "Anna");

  NotificationHistoryMember *member = prv_row(0)->group.members;
  prv_assert_id(&member->id, 2);
  member = (NotificationHistoryMember *)list_get_next(&member->node);
  prv_assert_id(&member->id, 1);
}

void test_notifications_history__missing_sender_and_ios_remain_individual(void) {
  prv_add(1, 100, NULL);
  prv_add(2, 200, "   ");
  prv_add_ios(3, 300, "Anna");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 3);
  cl_assert(!prv_row(0)->is_group);
  cl_assert(!prv_row(1)->is_group);
  cl_assert(!prv_row(2)->is_group);
  prv_assert_id(&prv_row(0)->notification_id, 3);
  prv_assert_id(&prv_row(1)->notification_id, 2);
  prv_assert_id(&prv_row(2)->notification_id, 1);
}

void test_notifications_history__sender_whitespace_is_trimmed_without_case_folding(void) {
  prv_add(1, 100, " Anna ");
  prv_add(2, 200, "Anna");
  prv_add(3, 300, "anna");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 2);
  cl_assert_equal_s(prv_row(0)->group.sender, "anna");
  cl_assert_equal_s(prv_row(1)->group.sender, "Anna");
  cl_assert_equal_i(prv_row(1)->group.count, 2);
}

void test_notifications_history__removing_notifications_updates_and_removes_group(void) {
  prv_add(1, 100, "Anna");
  prv_add(2, 300, "Anna");
  prv_add(3, 200, "Bob");

  Uuid id = prv_id(2);
  cl_assert(notifications_history_remove(&s_history, &id));
  cl_assert_equal_s(prv_row(0)->group.sender, "Bob");
  cl_assert_equal_i(prv_row(1)->group.count, 1);
  cl_assert(!notifications_history_row_is_collapsed_group(prv_row(1)));
  prv_assert_id(notifications_history_row_get_latest_id(prv_row(1)), 1);

  id = prv_id(1);
  cl_assert(notifications_history_remove(&s_history, &id));
  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 1);
  cl_assert_equal_s(prv_row(0)->group.sender, "Bob");
}

void test_notifications_history__mixed_grouped_and_individual_notifications(void) {
  prv_add(1, 100, "Anna");
  prv_add(2, 400, NULL);
  prv_add(3, 300, "Anna");
  prv_add(4, 200, "Bob");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 3);
  prv_assert_id(&prv_row(0)->notification_id, 2);
  cl_assert_equal_s(prv_row(1)->group.sender, "Anna");
  cl_assert_equal_i(prv_row(1)->group.count, 2);
  cl_assert_equal_s(prv_row(2)->group.sender, "Bob");
}

void test_notifications_history__disabled_preserves_storage_iteration_order(void) {
  notifications_history_deinit(&s_history);
  notifications_history_init(&s_history, false, 0);

  CommonTimelineItemHeader first = {
      .id = prv_id(1),
      .timestamp = 300,
  };
  CommonTimelineItemHeader second = {
      .id = prv_id(2),
      .timestamp = 100,
  };
  CommonTimelineItemHeader third = {
      .id = prv_id(3),
      .timestamp = 200,
  };
  notifications_history_add_header(&s_history, &first);
  notifications_history_add_header(&s_history, &second);
  notifications_history_add_header(&s_history, &third);

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 3);
  prv_assert_id(&prv_row(0)->notification_id, 3);
  prv_assert_id(&prv_row(1)->notification_id, 2);
  prv_assert_id(&prv_row(2)->notification_id, 1);
}

void test_notifications_history__only_notifications_within_range_are_grouped(void) {
  notifications_history_deinit(&s_history);
  notifications_history_init(&s_history, true, 1000);

  prv_add(1, 900, "Anna");
  prv_add(2, 1000, "Anna");
  prv_add(3, 1100, "Anna");

  cl_assert_equal_i(notifications_history_get_row_count(&s_history), 2);
  cl_assert(prv_row(0)->is_group);
  cl_assert_equal_i(prv_row(0)->group.count, 2);
  prv_assert_id(notifications_history_row_get_latest_id(prv_row(0)), 3);
  cl_assert(!prv_row(1)->is_group);
  prv_assert_id(&prv_row(1)->notification_id, 1);
}
