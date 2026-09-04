/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/peek.h"

#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/timeline/timeline.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>
#include "util/time/time.h"

typedef struct TimelinePeekEventData {
  bool initialized;
  unsigned int show_before_time_s;
} TimelinePeekEventData;

static TimelinePeekEventData s_peek_event_data = {
  .show_before_time_s = TIMELINE_PEEK_DEFAULT_SHOW_BEFORE_TIME_S,
};

typedef struct PeekUpdateContext {
  unsigned int num_peeking;
  uint32_t next_timeout_ms;
  SerializedTimelineItemHeader first_header;
  bool today_has_all_day_event;
  bool today_timed_event_passed;
  bool future_has_event;
} PeekUpdateContext;

static void prv_put_peek_event(PeekUpdateContext *update, TimelineItemId *item_id,
                               TimelinePeekTimeType time_type) {
  TimelineItemId *item_id_copy = NULL;
  if (item_id) {
    item_id_copy = kernel_malloc_check(sizeof(TimelineItemId));
    *item_id_copy = *item_id;
  }
  const bool is_all_day_event_visible =
      (update->today_has_all_day_event && !update->today_timed_event_passed);
  const bool is_first_event = (item_id && uuid_equal(item_id, &update->first_header.common.id));

  PebbleEvent event = {
    .type = PEBBLE_TIMELINE_PEEK_EVENT,
    .timeline_peek = {
      .time_type = time_type,
      .item_id = item_id_copy,
      .num_concurrent = MAX((int)update->num_peeking - 1, 0),
      .is_first_event = (!is_all_day_event_visible && is_first_event),
      .is_future_empty = (!is_all_day_event_visible && !update->future_has_event),
    },
  };
  event_put(&event);
}

static void prv_peek_will_update(void **context) {
  PeekUpdateContext **update = (PeekUpdateContext **)context;
  *update = task_zalloc_check(sizeof(PeekUpdateContext));
  (*update)->first_header.common.id = UUID_INVALID;
}

static void prv_peek_did_update(void **context) {
  PeekUpdateContext **update = (PeekUpdateContext **)context;
  task_free(*update);
}

static bool prv_is_in_peeking_time_window(SerializedTimelineItemHeader *header, time_t now) {
  const unsigned int duration_s = header->common.duration * SECONDS_PER_MINUTE;
  const unsigned int show_duration_after_start_s =
      header->common.persistent ? duration_s :
      MIN(TIMELINE_PEEK_HIDE_AFTER_TIME_S, duration_s);
  // As soon as an event begins, it should peek, hence the show-before time being inclusive
  return timeline_event_starts_within(
      &header->common, now, -show_duration_after_start_s,
      s_peek_event_data.show_before_time_s + 1 /* inclusive */);
}

static bool prv_should_set_first_event(PeekUpdateContext *update,
                                       SerializedTimelineItemHeader *header) {
  // Use the new item if there is no item or it is an earlier item in the future direction
  return ((uuid_is_invalid(&update->first_header.common.id)) ||
           (timeline_item_time_comparator(&header->common, &update->first_header.common,
                                          TimelineIterDirectionFuture) < 0));
}

static bool prv_peek_filter(SerializedTimelineItemHeader *header, void **context) {
  PeekUpdateContext *update = *(PeekUpdateContext **)context;
  const time_t now = rtc_get_time();
  const time_t start = header->common.timestamp;
  const time_t end = start + (header->common.duration * SECONDS_PER_MINUTE);
  if (timeline_event_is_all_day(&header->common)) {
    if (WITHIN(now, start, end)) {
      update->today_has_all_day_event = true;
    }
    return false;
  }
  if (prv_should_set_first_event(update, header)) {
    update->first_header = *header;
  }
  if (timeline_item_should_show(&header->common, TimelineIterDirectionFuture)) {
    update->future_has_event = true;
  }
  if ((now > start) && (time_util_get_midnight_of(now) == time_util_get_midnight_of(start))) {
    update->today_timed_event_passed = true;
  }
  if (now >= end) {
    return false;
  }
  if (header->common.dismissed) {
    // Ignore dismissed events
    return false;
  }
  const bool peeking = prv_is_in_peeking_time_window(header, now);
  if (peeking) {
    update->num_peeking++;
  }
  // Peeking or future event
  return (peeking || (now <= start));
}

static int prv_peek_comparator(SerializedTimelineItemHeader *new_header,
                               SerializedTimelineItemHeader *old_header, void **context) {
  const time_t now = rtc_get_time();
  const bool new_is_peeking = prv_is_in_peeking_time_window(new_header, now);
  const bool old_is_peeking = prv_is_in_peeking_time_window(old_header, now);
  const bool new_is_persistent = new_header->common.persistent;
  const bool old_is_persistent = old_header->common.persistent;
  if (new_is_peeking != old_is_peeking) {
    // Peeking items always take priority
    return (old_is_peeking ? 1 : 0) - (new_is_peeking ? 1 : 0);
  } else if (old_is_peeking && (new_is_persistent != old_is_persistent)) {
    // When both items are peeking, items that are not persistent take priority
    return (new_is_persistent ? 1 : 0) - (old_is_persistent ? 1 : 0);
  } else {
    // When both items are peeking, newer items take priority (larger timestamp first)
    // Otherwise, older items take priority (smaller timestamp first)
    return (old_is_peeking ? 1 : -1) * (old_header->common.timestamp -
                                        new_header->common.timestamp);
  }
}

static uint32_t prv_calc_timeout(CommonTimelineItemHeader *item,
                                 TimelinePeekTimeType *time_type_out) {
  if (item == NULL) {
    goto none;
  }

  TimelinePeekTimeType time_type;
  const time_t now = rtc_get_time();
  const time_t start = item->timestamp;
  const unsigned int duration_m = item->duration;
  const time_t end = start + (duration_m * SECONDS_PER_MINUTE);
  if (now >= end) {
    goto none;
  }

  uint32_t timeout_s;
  if (timeline_event_is_ongoing(now, start, duration_m)) {
    const time_t into = start + TIMELINE_PEEK_HIDE_AFTER_TIME_S;
    const bool short_event = (end < into);
    const bool started_moments_ago = (now < into);
    timeout_s = ((started_moments_ago && !short_event) ? into : end) - now;
    // If it's persistent, it should be shown for the entire duration
    time_type = (started_moments_ago || item->persistent) ?
        TimelinePeekTimeType_ShowStarted : TimelinePeekTimeType_WillEnd;
  } else {
    const time_t before = start - s_peek_event_data.show_before_time_s;
    const bool some_time_next = (now < before);
    timeout_s = (some_time_next ? before : start) - now;
    time_type = some_time_next ? TimelinePeekTimeType_SomeTimeNext :
                                 TimelinePeekTimeType_ShowWillStart;
  }
  if (time_type_out) {
    *time_type_out = time_type;
  }
  return MIN(timeout_s, UINT32_MAX / MS_PER_SECOND) * MS_PER_SECOND;

none:
  if (time_type_out) {
    *time_type_out = TimelinePeekTimeType_None;
  }
  return 0;
}

static int prv_peek_compare_and_save_next_timeout(
    SerializedTimelineItemHeader *new_header, SerializedTimelineItemHeader *old_header,
    void **context) {
  const int rv = prv_peek_comparator(new_header, old_header, context);
  CommonTimelineItemHeader *next_header = &((rv > 0) ? new_header : old_header)->common;
  PeekUpdateContext **update = (PeekUpdateContext **)context;
  const uint32_t next_timeout_ms = prv_calc_timeout(next_header, NULL);
  const uint32_t old_next_timeout_ms = (*update)->next_timeout_ms;
  if (!old_next_timeout_ms || (next_timeout_ms && (next_timeout_ms < old_next_timeout_ms))) {
    (*update)->next_timeout_ms = next_timeout_ms;
  }
  return rv;
}

static uint32_t prv_peek_update(TimelineItem *item, void **context) {
  TimelinePeekTimeType time_type;
  PeekUpdateContext *update = *(PeekUpdateContext **)context;
  const uint32_t timeout_ms = prv_calc_timeout(&item->header, &time_type);
  prv_put_peek_event(update, timeout_ms ? &item->header.id : NULL, time_type);
  const uint32_t next_timeout_ms = update->next_timeout_ms;
  return next_timeout_ms ? MIN(next_timeout_ms, timeout_ms) : timeout_ms;
}

const TimelineEventImpl *timeline_peek_get_event_service(void) {
  static const TimelineEventImpl s_event_impl = {
    .will_update = prv_peek_will_update,
    .filter = prv_peek_filter,
    .comparator = prv_peek_compare_and_save_next_timeout,
    .update = prv_peek_update,
    .did_update = prv_peek_did_update,
  };
  if (!s_peek_event_data.initialized) {
    s_peek_event_data.initialized = true;
    s_peek_event_data.show_before_time_s =
        (timeline_peek_prefs_get_before_time() * SECONDS_PER_MINUTE);
  }
  return &s_event_impl;
}

void timeline_peek_set_show_before_time(unsigned int before_time_s) {
  s_peek_event_data.show_before_time_s = before_time_s;
  timeline_event_refresh();
}
