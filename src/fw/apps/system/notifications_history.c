/* SPDX-FileCopyrightText: 2026 Aliaksandr Karnilovich */
/* SPDX-License-Identifier: Apache-2.0 */

#include "notifications_history.h"

#include "kernel/pbl_malloc.h"
#include "pbl/services/timeline/attribute.h"
#include "pbl/services/timeline/timeline.h"

#include <ctype.h>
#include <string.h>

typedef struct StringRange {
  const char *start;
  size_t length;
} StringRange;

static int prv_compare_position(time_t timestamp_a, uint32_t sequence_a, time_t timestamp_b,
                                uint32_t sequence_b) {
  if (timestamp_a > timestamp_b) {
    return -1;
  }
  if (timestamp_a < timestamp_b) {
    return 1;
  }
  if (sequence_a > sequence_b) {
    return -1;
  }
  if (sequence_a < sequence_b) {
    return 1;
  }
  return 0;
}

static int prv_row_comparator(void *a, void *b) {
  NotificationHistoryRow *row_a = a;
  NotificationHistoryRow *row_b = b;
  return prv_compare_position(row_a->timestamp, row_a->sequence, row_b->timestamp, row_b->sequence);
}

static int prv_member_comparator(void *a, void *b) {
  NotificationHistoryMember *member_a = a;
  NotificationHistoryMember *member_b = b;
  return prv_compare_position(member_a->timestamp, member_a->sequence, member_b->timestamp,
                              member_b->sequence);
}

static StringRange prv_trimmed_string_range(const char *string) {
  if (!string) {
    return (StringRange){};
  }

  const char *start = string;
  while (*start && isspace((unsigned char)*start)) {
    start++;
  }

  const char *end = start + strlen(start);
  while (end > start && isspace((unsigned char)*(end - 1))) {
    end--;
  }

  return (StringRange){
      .start = start,
      .length = (size_t)(end - start),
  };
}

static bool prv_group_sender_for_item(const TimelineItem *item, StringRange *sender_out) {
  static const Uuid s_android_notifications_source = UUID_NOTIFICATIONS_DATA_SOURCE;

  if (item->header.ancs_notif ||
      !uuid_equal(&item->header.parent_id, &s_android_notifications_source)) {
    return false;
  }

  const char *sender = attribute_get_string(&item->attr_list, AttributeIdSender, NULL);
  if (!sender) {
    sender = attribute_get_string(&item->attr_list, AttributeIdTitle, NULL);
  }

  *sender_out = prv_trimmed_string_range(sender);
  return sender_out->length > 0;
}

static NotificationHistoryRow *prv_find_group(NotificationHistory *history,
                                              const StringRange *sender) {
  NotificationHistoryRow *row = history->rows;
  while (row) {
    if (row->is_group && strlen(row->group.sender) == sender->length &&
        memcmp(row->group.sender, sender->start, sender->length) == 0) {
      return row;
    }
    row = (NotificationHistoryRow *)list_get_next(&row->node);
  }
  return NULL;
}

static void prv_insert_row_sorted(NotificationHistory *history, NotificationHistoryRow *row) {
  history->rows = (NotificationHistoryRow *)list_sorted_add((ListNode *)history->rows, &row->node,
                                                            prv_row_comparator, false);
}

static NotificationHistoryRow *prv_create_individual_row(NotificationHistory *history,
                                                         const CommonTimelineItemHeader *header) {
  NotificationHistoryRow *row = app_malloc_check(sizeof(*row));
  *row = (NotificationHistoryRow){
      .is_group = false,
      .timestamp = header->timestamp,
      .sequence = history->next_sequence++,
      .notification_id = header->id,
  };
  list_init(&row->node);
  return row;
}

static NotificationHistoryMember *prv_create_member(NotificationHistory *history,
                                                    const CommonTimelineItemHeader *header) {
  NotificationHistoryMember *member = app_malloc_check(sizeof(*member));
  *member = (NotificationHistoryMember){
      .id = header->id,
      .timestamp = header->timestamp,
      .sequence = history->next_sequence++,
  };
  list_init(&member->node);
  return member;
}

static NotificationHistoryRow *prv_create_group(NotificationHistory *history,
                                                const StringRange *sender) {
  NotificationHistoryRow *row = app_malloc_check(sizeof(*row));
  *row = (NotificationHistoryRow){
      .is_group = true,
  };
  list_init(&row->node);

  row->group.sender = app_malloc_check(sender->length + 1);
  memcpy(row->group.sender, sender->start, sender->length);
  row->group.sender[sender->length] = '\0';
  return row;
}

static void prv_free_group_members(NotificationHistoryMember *member) {
  while (member) {
    NotificationHistoryMember *next = (NotificationHistoryMember *)list_get_next(&member->node);
    app_free(member);
    member = next;
  }
}

static void prv_free_row(NotificationHistoryRow *row) {
  if (row->is_group) {
    prv_free_group_members(row->group.members);
    app_free(row->group.sender);
  }
  app_free(row);
}

void notifications_history_init(NotificationHistory *history, bool group_by_sender,
                                time_t grouping_cutoff) {
  *history = (NotificationHistory){
      .group_by_sender = group_by_sender,
      .grouping_cutoff = grouping_cutoff,
  };
}

void notifications_history_deinit(NotificationHistory *history) {
  NotificationHistoryRow *row = history->rows;
  while (row) {
    NotificationHistoryRow *next = (NotificationHistoryRow *)list_get_next(&row->node);
    prv_free_row(row);
    row = next;
  }
  history->rows = NULL;
}

void notifications_history_add_header(NotificationHistory *history,
                                      const CommonTimelineItemHeader *header) {
  NotificationHistoryRow *row = prv_create_individual_row(history, header);
  if (history->group_by_sender) {
    prv_insert_row_sorted(history, row);
  } else {
    history->rows = (NotificationHistoryRow *)list_prepend((ListNode *)history->rows, &row->node);
  }
}

void notifications_history_add_item(NotificationHistory *history, const TimelineItem *item) {
  if (!history->group_by_sender || item->header.timestamp < history->grouping_cutoff) {
    notifications_history_add_header(history, &item->header);
    return;
  }

  StringRange sender;
  if (!prv_group_sender_for_item(item, &sender)) {
    notifications_history_add_header(history, &item->header);
    return;
  }

  NotificationHistoryRow *row = prv_find_group(history, &sender);
  if (!row) {
    row = prv_create_group(history, &sender);
  } else {
    list_remove(&row->node, (ListNode **)&history->rows, NULL);
  }

  NotificationHistoryMember *member = prv_create_member(history, &item->header);
  row->group.members = (NotificationHistoryMember *)list_sorted_add(
      (ListNode *)row->group.members, &member->node, prv_member_comparator, false);
  row->group.count++;
  row->timestamp = row->group.members->timestamp;
  row->sequence = row->group.members->sequence;
  prv_insert_row_sorted(history, row);
}

bool notifications_history_remove(NotificationHistory *history, const Uuid *id) {
  NotificationHistoryRow *row = history->rows;
  while (row) {
    if (!row->is_group) {
      if (uuid_equal(&row->notification_id, id)) {
        list_remove(&row->node, (ListNode **)&history->rows, NULL);
        prv_free_row(row);
        return true;
      }
    } else {
      NotificationHistoryMember *member = row->group.members;
      while (member && !uuid_equal(&member->id, id)) {
        member = (NotificationHistoryMember *)list_get_next(&member->node);
      }
      if (member) {
        const bool removed_latest = (member == row->group.members);
        list_remove(&member->node, (ListNode **)&row->group.members, NULL);
        app_free(member);
        row->group.count--;

        if (row->group.count == 0) {
          list_remove(&row->node, (ListNode **)&history->rows, NULL);
          prv_free_row(row);
        } else if (removed_latest) {
          list_remove(&row->node, (ListNode **)&history->rows, NULL);
          row->timestamp = row->group.members->timestamp;
          row->sequence = row->group.members->sequence;
          prv_insert_row_sorted(history, row);
        }
        return true;
      }
    }
    row = (NotificationHistoryRow *)list_get_next(&row->node);
  }
  return false;
}

uint16_t notifications_history_get_row_count(const NotificationHistory *history) {
  return (uint16_t)list_count((ListNode *)history->rows);
}

NotificationHistoryRow *notifications_history_get_row(const NotificationHistory *history,
                                                      uint16_t index) {
  return (NotificationHistoryRow *)list_get_at((ListNode *)history->rows, index);
}

bool notifications_history_row_is_collapsed_group(const NotificationHistoryRow *row) {
  return row->is_group && row->group.count > 1;
}

const Uuid *notifications_history_row_get_latest_id(const NotificationHistoryRow *row) {
  if (row->is_group) {
    return row->group.members ? &row->group.members->id : NULL;
  }
  return &row->notification_id;
}

uint16_t notifications_history_row_get_count(const NotificationHistoryRow *row) {
  return row->is_group ? row->group.count : 1;
}
