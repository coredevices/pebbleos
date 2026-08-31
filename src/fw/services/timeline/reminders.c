/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/reminders.h"

#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/system_task.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/blob_db/reminder_db.h"
#include "pbl/services/timeline/item.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DECLARE(service_timeline, CONFIG_SERVICE_TIMELINE_LOG_LEVEL);

#define INVALID_SNOOZE_DELAY 0
#define HALF_SNOOZE_END_MARK 30 // Seconds
#define CONSTANT_SNOOZE_DELAY (10 * SECONDS_PER_MINUTE) // Seconds
#define CONSTANT_SNOOZE_END_MARK (48 * MINUTES_PER_HOUR * SECONDS_PER_MINUTE) // Seconds

static RegularTimerInfo s_reminder_timer;
static bool s_reminder_armed;
static time_t s_next_reminder_timestamp;
static ReminderId s_next_reminder_id;

bool reminders_mark_has_reminded(ReminderId *reminder_id);

static void prv_put_reminder_event(ReminderId *reminder_id, ReminderEventType type) {
  Uuid *removed_id = kernel_malloc(sizeof(Uuid));
  if (!removed_id) {
    return;
  }

  *removed_id = *reminder_id;
  PebbleEvent event = {
    .type = PEBBLE_REMINDER_EVENT,
    .reminder = {
      .type = type,
      .reminder_id = removed_id,
    }
  };
  event_put(&event);
}

void reminders_handle_reminder_updated(const Uuid *reminder_id) {
  prv_put_reminder_event((ReminderId *)reminder_id, ReminderUpdated);
}

void reminders_handle_reminder_removed(const Uuid *reminder_id) {
  prv_put_reminder_event((ReminderId *)reminder_id, ReminderRemoved);
}

static void prv_trigger_reminder_system_task_callback(void *data) {
  ReminderId *item_id = (ReminderId *)data;

  // Mark that we are about to display the reminder
  if (!reminders_mark_has_reminded(item_id)) {
    return;
  }

  prv_put_reminder_event(item_id, ReminderTriggered);
  reminders_update_timer();
}

// Polls once per second against the RTC. Same pattern cron uses internally,
// avoids FreeRTOS-tick drift relative to wall-clock.
static void prv_timer_callback(void *data) {
  if (!s_reminder_armed) {
    return;
  }
  if (s_next_reminder_timestamp > rtc_get_time()) {
    return;
  }
  if (system_task_add_callback(prv_trigger_reminder_system_task_callback,
                               &s_next_reminder_id)) {
    s_reminder_armed = false;
  }
}

static status_t prv_set_timer(Reminder *item) {
  s_next_reminder_id = item->header.id;
  s_next_reminder_timestamp = item->header.timestamp;
  s_reminder_armed = true;
  PBL_LOG_DBG("Set reminder for %ld", s_next_reminder_timestamp);
  return S_SUCCESS;
}

status_t reminders_update_timer(void) {
  PBL_LOG_DBG("Attempting to update timer.");
  s_reminder_armed = false;

  TimelineItem item = {{{0}}};
  status_t rv = reminder_db_next_item_header(&item);
  if (rv == S_NO_MORE_ITEMS) {
    PBL_LOG_DBG("No more reminders to add to queue.");
    return S_SUCCESS;
  } else if (rv) {
    return rv;
  }

  return prv_set_timer(&item);
}

status_t reminders_insert(Reminder *reminder) {
  status_t rv = reminder_db_insert_item(reminder);
  return rv;
}

status_t reminders_init(void) {
  if (s_reminder_timer.cb == NULL) {
    s_reminder_timer.cb = prv_timer_callback;
    regular_timer_add_seconds_callback(&s_reminder_timer);
  }
  return reminders_update_timer();
}

status_t reminders_delete(ReminderId *reminder_id) {
  return reminder_db_delete_item(reminder_id, true /* send_event */);
}

T_STATIC uint32_t prv_calculate_snooze_delay(TimelineItem *item) {
  time_t current_time_utc = rtc_get_time();
  time_t reminder_time_utc = item->header.timestamp;
  if (current_time_utc <= reminder_time_utc) {
    return INVALID_SNOOZE_DELAY;
  }

  uint32_t snooze_delay;

  // Get parent pin
  const TimelineItemId *parent_id = &item->header.parent_id;
  TimelineItem parent_item;
  status_t status = pin_db_get(parent_id, &parent_item);
  if (status != S_SUCCESS) {
    return INVALID_SNOOZE_DELAY;
  }

  // Snooze logic:
  // If current_time is more than HALF_SNOOZE_END_MARK before event_time, snooze for half the
  //   remaining time until the event.
  // If current_time is less than HALF_SNOOZE_END_MARK before event_time, and not more than
  //   CONSTANT_SNOOZE_END_MARK after event_time, snooze for CONSTANT_SNOOZE_DELAY.
  // If current_time is more than CONSTANT_SNOOZE_END_MARK after event_time, don't snooze.
  time_t event_time_utc = parent_item.header.timestamp;
  if (event_time_utc > current_time_utc &&
      event_time_utc - current_time_utc > HALF_SNOOZE_END_MARK) {
    // Half-time snooze
    snooze_delay = (event_time_utc - reminder_time_utc) / 2;
  } else if (current_time_utc > event_time_utc &&
             current_time_utc - event_time_utc > CONSTANT_SNOOZE_END_MARK) {
    // Stop snoozing
    snooze_delay = INVALID_SNOOZE_DELAY;
  } else {
    // Constant-time snooze
    snooze_delay = CONSTANT_SNOOZE_DELAY;
  }

  timeline_item_free_allocated_buffer(&parent_item);
  return snooze_delay;
}

bool reminders_can_snooze(Reminder *reminder) {
  return (prv_calculate_snooze_delay((TimelineItem *)reminder) > 0);
}

status_t reminders_snooze(Reminder *reminder) {
  uint32_t snooze_delay = prv_calculate_snooze_delay((TimelineItem *)reminder);
  if (snooze_delay == 0) {
    return E_INVALID_OPERATION;
  }

  // Modify reminder timestamp
  TimelineItem *item = (TimelineItem*) reminder;
  item->header.timestamp = rtc_get_time() + (time_t) snooze_delay;

  // Unset the reminded status
  item->header.reminded = false;

  // Reinsert the reminder
  return reminders_insert(reminder);
}

// only used for tests
RegularTimerInfo *get_reminder_timer(void) {
  return &s_reminder_timer;
}

bool get_reminder_armed(void) {
  return s_reminder_armed;
}

time_t get_reminder_timestamp(void) {
  return s_next_reminder_timestamp;
}

ReminderId *get_reminder_id(void) {
  return &s_next_reminder_id;
}

bool reminders_mark_has_reminded(ReminderId *reminder_id) {
  status_t rv = reminder_db_set_status_bits(reminder_id, TimelineItemStatusReminded);
  return (rv == S_SUCCESS);
}
