/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/calendar.h"
#include "pbl/services/timeline/event.h"
#include "pbl/services/timeline/peek.h"

#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "kernel/pebble_tasks.h"
#include "pbl/os/mutex.h"
#include "pbl/services/system_task.h"
#include "pbl/services/blob_db/pin_db.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "system/status_codes.h"
#include "util/time/time.h"

PBL_LOG_MODULE_DECLARE(service_timeline, CONFIG_SERVICE_TIMELINE_LOG_LEVEL);

typedef struct TimelineEventState {
  const TimelineEventImpl *impl;
  SerializedTimelineItemHeader *filter_header;
  void *context;
} TimelineEventState;

static TimelineEventImplGetter s_services[TimelineEventServiceCount] = {
  [TimelineEventService_Calendar] = calendar_get_event_service,
  [TimelineEventService_Peek] = timeline_peek_get_event_service,
};

// This mutex protects all state, but is only used for factory resetting synchronously, therefore
// it should not significantly increase blocking time.
static PebbleMutex *s_mutex;

static TimelineEventState s_states[TimelineEventServiceCount];

static TimerID s_timer;

static bool s_cb_scheduled;

static void prv_update_status(void);

static void prv_update_status_system_task_callback(void *unused) {
  __atomic_clear(&s_cb_scheduled, __ATOMIC_RELAXED);
  prv_update_status();
}

static void prv_update_status_async(void) {
  if (__atomic_test_and_set(&s_cb_scheduled, __ATOMIC_RELAXED)) {
    return; // we already have a cb scheduled
  }

  system_task_add_callback(prv_update_status_system_task_callback, NULL);
}

static void prv_new_timer_callback(void *unused) {
  prv_update_status_async();
}

static uint32_t prv_calc_timeout(const TimelineItem *item) {
  const time_t now = rtc_get_time();
  const time_t start = item->header.timestamp;
  const time_t end = start + (item->header.duration * SECONDS_PER_MINUTE);
  if (now >= end) {
    return 0;
  }
  const uint32_t timeout_s = ((start > now) ? start : end) - now;
  return MIN(timeout_s, UINT32_MAX / MS_PER_SECOND) * MS_PER_SECOND;
}

static void prv_set_timer(unsigned int timeout_ms) {
  if (!timeout_ms) {
    PBL_LOG_ERR("Not setting timer");
  } else if (new_timer_start(s_timer, timeout_ms, prv_new_timer_callback, NULL, 0)) {
    PBL_LOG_DBG("Set timer for %u", timeout_ms);
  } else {
    PBL_LOG_ERR("Could not start timer.");
  }
}

static bool prv_should_use_item(TimelineEventState *state, SerializedTimelineItemHeader *header) {
  // Use the new item if ...
  return ((uuid_is_invalid(&state->filter_header->common.id)) || // There is no old item
          (state->impl->comparator && // Or the comparator chooses the new item
           (state->impl->comparator(header, state->filter_header, &state->context) < 0)) ||
          (header->common.timestamp < state->filter_header->common.timestamp)); // Or it's earlier
}

static bool prv_item_header_filter(SerializedTimelineItemHeader *header, void *unused) {
  bool filter = false;
  for (unsigned int i = 0; i < TimelineEventServiceCount; i++) {
    TimelineEventState *state = &s_states[i];
    // Pass all items through the filter to allow clients to process all item headers
    if (state->impl && state->impl->filter(header, &state->context)) {
      if (prv_should_use_item(state, header)) {
        *state->filter_header = *header;
      }
      filter = true;
    }
  }
  return filter;
}

static void prv_update_status(void) {
  PBL_ASSERT_TASK(PebbleTask_KernelBackground);
  if (!s_mutex) {
    return;
  }
  mutex_lock(s_mutex);
  new_timer_stop(s_timer);
  SerializedTimelineItemHeader *filter_headers =
      kernel_zalloc_check(TimelineEventServiceCount * sizeof(SerializedTimelineItemHeader));

  // Will update
  for (unsigned int i = 0; i < TimelineEventServiceCount; i++) {
    TimelineEventState *state = &s_states[i];
    if (state->impl) {
      state->filter_header = &filter_headers[i];
      state->filter_header->common.id = UUID_INVALID;
      if (state->impl->will_update) {
        state->impl->will_update(&state->context);
      }
    }
  }

  // Filter
  TimelineItem item = {};
  const status_t rv = pin_db_next_item_header(&item, prv_item_header_filter);
  uint32_t timeout_ms = 0;
  if ((rv != S_SUCCESS) && (rv != S_NO_MORE_ITEMS)) {
    // A failure occurred. Call the update functions with a NULL item
    PBL_LOG_ERR("Failed to find next event.");
  } else if (rv != S_NO_MORE_ITEMS) {
    // Calculate the timeout before the item buffer is re-used
    timeout_ms = prv_calc_timeout(&item);
  }

  // Update
  for (unsigned int i = 0; i < TimelineEventServiceCount; i++) {
    TimelineEventState *state = &s_states[i];
    if (!state->impl) {
      continue;
    }
    const bool has_item = !uuid_is_invalid(&state->filter_header->common.id);
    if (has_item) {
      timeline_item_deserialize_header(&item, state->filter_header);
    }
    const uint32_t other_timeout_ms =
        state->impl->update(has_item ? &item : NULL, &state->context);
    if (other_timeout_ms) {
      timeout_ms = timeout_ms ? MIN(timeout_ms, other_timeout_ms) : other_timeout_ms;
    }
  }

  // Did update
  for (int i = 0; i < TimelineEventServiceCount; i++) {
    TimelineEventState *state = &s_states[i];
    if (state->impl && state->impl->did_update) {
      state->impl->did_update(&state->context);
    }
  }

  prv_set_timer(timeout_ms);
  kernel_free(filter_headers);
  mutex_unlock(s_mutex);
}

static void prv_init(void *PBL_UNUSED data) {
  s_mutex = mutex_create();
  mutex_lock(s_mutex);

  for (unsigned int i = 0; i < TimelineEventServiceCount; i++) {
    TimelineEventState *state = &s_states[i];
    state->impl = s_services[i]();
  }

  s_timer = new_timer_create();

  mutex_unlock(s_mutex);
  prv_update_status();
}

void timeline_event_init(void) {
  system_task_add_callback(prv_init, NULL);
}

void timeline_event_deinit(void) {
  mutex_lock(s_mutex);

  new_timer_delete(s_timer);
  s_timer = TIMER_INVALID_ID;

  mutex_unlock(s_mutex);
  mutex_destroy(s_mutex);
  s_mutex = NULL;
}

void timeline_event_handle_blobdb_event(void) {
  prv_update_status_async();
}

void timeline_event_refresh(void) {
  prv_update_status_async();
}

bool timeline_event_is_all_day(CommonTimelineItemHeader *common) {
  return (common->all_day ||
          (common->duration >= MINUTES_PER_DAY)); // Include >= 24 hour events. See PBL-23584
}

bool timeline_event_is_ongoing(time_t now, time_t event_start, int event_duration_m) {
  return ((event_start <= now) && ((event_start + (SECONDS_PER_MINUTE * event_duration_m)) > now));
}

bool timeline_event_starts_within(CommonTimelineItemHeader *common, time_t now,
                                  int delta_start_s, int delta_end_s) {
  return ((common->type == TimelineItemTypePin) && // Ignore non-pins
          (((delta_start_s == TIMELINE_EVENT_DELTA_INFINITE) || // Any past event or
            (common->timestamp > (now + delta_start_s))) && // Begins after range start and
           ((delta_end_s == TIMELINE_EVENT_DELTA_INFINITE) || // Any future event or
            (common->timestamp < (now + delta_end_s))))); // Begins before range end
}
