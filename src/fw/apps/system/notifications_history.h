/* SPDX-FileCopyrightText: 2026 Aliaksandr Karnilovich */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/timeline/item.h"
#include "pbl/util/list.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct NotificationHistoryMember {
  ListNode node;
  Uuid id;
  time_t timestamp;
  uint32_t sequence;
} NotificationHistoryMember;

typedef struct NotificationHistoryRow {
  ListNode node;
  bool is_group;
  time_t timestamp;
  uint32_t sequence;
  union {
    Uuid notification_id;
    struct {
      char *sender;
      NotificationHistoryMember *members;
      uint16_t count;
    } group;
  };
} NotificationHistoryRow;

typedef struct NotificationHistory {
  NotificationHistoryRow *rows;
  bool group_by_sender;
  time_t grouping_cutoff;
  uint32_t next_sequence;
} NotificationHistory;

void notifications_history_init(NotificationHistory *history, bool group_by_sender,
                                time_t grouping_cutoff);
void notifications_history_deinit(NotificationHistory *history);

void notifications_history_add_header(NotificationHistory *history,
                                      const CommonTimelineItemHeader *header);
void notifications_history_add_item(NotificationHistory *history, const TimelineItem *item);
bool notifications_history_remove(NotificationHistory *history, const Uuid *id);

uint16_t notifications_history_get_row_count(const NotificationHistory *history);
NotificationHistoryRow *notifications_history_get_row(const NotificationHistory *history,
                                                      uint16_t index);
bool notifications_history_row_is_collapsed_group(const NotificationHistoryRow *row);
const Uuid *notifications_history_row_get_latest_id(const NotificationHistoryRow *row);
uint16_t notifications_history_row_get_count(const NotificationHistoryRow *row);
