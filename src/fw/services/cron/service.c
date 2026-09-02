/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/cron.h"
#include <pebbleos/cron.h>

#include <pbl/drivers/rtc.h>
#include "pbl/kernel/mutex.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"

PBL_LOG_MODULE_DEFINE(service_cron, CONFIG_SERVICE_CRON_LOG_LEVEL);

//! Don't let users modify the list while callbacks are occurring.
static PBL_MUTEX_DEFINE(s_list_mutex);

// List of jobs sorted from soonest to farthest.
static ListNode *s_scheduled_jobs;

static void prv_timer_callback(void* data);

//! One-shot timer armed for the next job's execute time. Re-armed after every
//! list mutation and every firing; stopped when no jobs are scheduled. Capped
//! so that long sleeps re-check at least hourly, which keeps any tick-clock
//! drift across sleep bounded.
#define CRON_MAX_ARM_INTERVAL_S (60 * 60)
static TimerID s_wakeup_timer = TIMER_INVALID_ID;

//! Arm (or stop) the wakeup timer for the head job. s_list_mutex must be held.
static void prv_arm_wakeup(void) {
  if (s_scheduled_jobs == NULL) {
    new_timer_stop(s_wakeup_timer);
    return;
  }
  time_t now;
  uint16_t milliseconds;
  rtc_get_time_ms(&now, &milliseconds);
  const time_t execute_time = ((CronJob *)s_scheduled_jobs)->cached_execute_time;
  int32_t delta_s = (execute_time > now) ? (int32_t)(execute_time - now) : 0;
  delta_s = MIN(delta_s, CRON_MAX_ARM_INTERVAL_S);
  uint32_t timeout_ms = (uint32_t)delta_s * 1000U;
  if (timeout_ms > milliseconds) {
    timeout_ms -= milliseconds;
  }
  new_timer_start(s_wakeup_timer, timeout_ms, prv_timer_callback, NULL, 0 /*flags*/);
}

// -------------------------------------------------------------------------------------------
static bool prv_is_scheduled(CronJob *job) {
  // Assumes mutex lock is already taken
  return list_contains(s_scheduled_jobs, &job->list_node);
}

static int prv_sort(void *a, void *b) {
  CronJob *job_a = (CronJob*)a;
  CronJob *job_b = (CronJob*)b;
  return job_b->cached_execute_time - job_a->cached_execute_time;
}

// -------------------------------------------------------------------------------------------
static void prv_timer_callback(void* data) {
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);
  while (s_scheduled_jobs != NULL &&
         ((CronJob*)s_scheduled_jobs)->cached_execute_time <= rtc_get_time()) {
    CronJob *job = (CronJob*)s_scheduled_jobs;
    // Remove the job from the list, it's done.
    s_scheduled_jobs = list_pop_head(s_scheduled_jobs);

    // Release the mutex while we execute the callback
    pbl_mutex_unlock(&s_list_mutex);
    job->cb(job, job->cb_data);
    pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);
  }
  prv_arm_wakeup();
  pbl_mutex_unlock(&s_list_mutex);
}

// --------------------------------------------------------------------------------------------
void cron_service_handle_clock_change(PebbleSetTimeEvent *set_time_info) {
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);

  const bool must_recalc = set_time_info->gmt_offset_delta != 0 || set_time_info->dst_changed;
  // Because it's ABS, it'll be unsigned. This makes the compiler behave.
  const uint32_t change_diff = ABS(set_time_info->utc_time_delta);
  // Need to re-build the list somewhere else
  ListNode *newlist = NULL;
  while (s_scheduled_jobs != NULL) {
    CronJob* job = (CronJob*)s_scheduled_jobs;
    s_scheduled_jobs = list_pop_head(s_scheduled_jobs);
    // Re-calculate the execute time.
    // See the notes in the API header on how this works.
    if (must_recalc || change_diff >= job->clock_change_tolerance) {
      job->cached_execute_time = cron_job_get_execute_time(job);
    }
    PBL_LOG_DBG("Cron job rescheduled for %ld", job->cached_execute_time);

    newlist = list_sorted_add(newlist, &job->list_node, prv_sort, true);
  }
  // Then move it back to the static
  s_scheduled_jobs = newlist;

  pbl_mutex_unlock(&s_list_mutex);

  // We want to run any tasks we've skipped over.
  prv_timer_callback(NULL);
}

// --------------------------------------------------------------------------------------------
void cron_service_init(void) {
  s_scheduled_jobs = NULL;

  if (s_wakeup_timer == TIMER_INVALID_ID) {
    s_wakeup_timer = new_timer_create();
  }
}

// -------------------------------------------------------------------------------------------
time_t cron_job_schedule(CronJob *job) {
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);

  const time_t now = rtc_get_time();
  // Always update the execution time.
  job->cached_execute_time = cron_job_get_execute_time_from_epoch(job, now);
  // If not scheduled yet, schedule it.
  if (!prv_is_scheduled(job)) {
    s_scheduled_jobs = list_sorted_add(s_scheduled_jobs, &job->list_node, prv_sort, true);
  }
  PBL_LOG_DBG("Cron job scheduled for %ld (%+ld)", job->cached_execute_time,
          (job->cached_execute_time - now));

  prv_arm_wakeup();
  pbl_mutex_unlock(&s_list_mutex);

  return job->cached_execute_time;
}

// ------------------------------------------------------------------------------------------
time_t cron_job_schedule_after(CronJob *job, CronJob *new_job) {
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);

  // can't schedule an already scheduled job
  PBL_ASSERTN(!prv_is_scheduled(new_job));
  // can't schedule after an unscheduled job
  PBL_ASSERTN(prv_is_scheduled(job));

  // copy schedule info from existing job
  CronJob temp_job = *job;
  list_init(&temp_job.list_node);
  temp_job.cb = new_job->cb;
  temp_job.cb_data = new_job->cb_data;
  *new_job = temp_job;

  // insert after in the list, which guarantees it gets executed after
  list_insert_after(&job->list_node, &new_job->list_node);
  PBL_LOG_DBG("Cron job scheduled for %ld", job->cached_execute_time);

  prv_arm_wakeup();
  pbl_mutex_unlock(&s_list_mutex);

  return job->cached_execute_time;
}

// ------------------------------------------------------------------------------------------
bool cron_job_is_scheduled(CronJob *job) {
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);
  bool rv = prv_is_scheduled(job);
  pbl_mutex_unlock(&s_list_mutex);

  return (rv);
}

// ------------------------------------------------------------------------------------------
bool cron_job_unschedule(CronJob *job) {
  bool removed = false;
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);

  if (prv_is_scheduled(job)) {
    list_remove(&job->list_node, &s_scheduled_jobs, NULL);
    removed = true;
    prv_arm_wakeup();
  }

  pbl_mutex_unlock(&s_list_mutex);
  return removed;
}


// ---------------------------------------------------------------------------------------
// For Testing:

void cron_clear_all_jobs(void) {
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);

  // Iterate over all the jobs to remove them all.
  for (ListNode* iter = s_scheduled_jobs; iter != NULL; ) {
    CronJob* job = (CronJob*)iter;
    iter = list_get_next(iter);
    // Remove the job from the list.
    list_remove(&job->list_node, NULL, NULL);
  }
  s_scheduled_jobs = NULL;
  prv_arm_wakeup();

  pbl_mutex_unlock(&s_list_mutex);
}

void cron_service_deinit(void) {
  cron_clear_all_jobs();

  new_timer_stop(s_wakeup_timer);
}

uint32_t cron_service_get_job_count(void) {
  uint32_t count = 0;
  pbl_mutex_lock(&s_list_mutex, PBL_FOREVER);
  count = list_count(s_scheduled_jobs);
  pbl_mutex_unlock(&s_list_mutex);
  return count;
}

void cron_service_wakeup(void) {
  prv_timer_callback(NULL);
}

// ---------------------------------------------------------------------------------------
// The brains.
typedef enum {
  CronAssignMode_LocalEpoch, // 'any' uses local epoch's value
  CronAssignMode_Zero, // 'any' uses 0
} CronAssignMode;

// Indices for the access arrays
#define CRON_INDEX_YEAR 0
#define CRON_INDEX_MONTH 1
#define CRON_INDEX_DAY 2
#define CRON_INDEX_HOUR 3
#define CRON_INDEX_MIN 4
#define CRON_INDEX_SEC 5
#define CRON_INDEX_COUNT 6

#define CRON_GENERIC_ANY (-1)
#define CRON_YEAR_ANY (-1)
#define CRON_SECOND_ANY (-1)

// If the 'working' time is ahead of local epoch, we return 1. If behind, we return -1.
// Otherwise, return 0.
static int prv_future_past_direction(int **dest_arr, const int *curr_arr) {
  // Iterate from highest order to lowest.
  for (int i = 0; i < CRON_INDEX_COUNT; i++) {
    if (*(dest_arr[i]) > curr_arr[i]) {
      // In future
      return 1;
    } else if (*(dest_arr[i]) < curr_arr[i]) {
      // In past
      return -1;
    }
  }
  return 0;
}

// Increase the day in `cron_tm` to fit into the wday set in `cron`.
// This doesn't take mday into account because that's way too hard and we won't need it.
static bool prv_adjust_for_wday_spec(const CronJob *cron, struct tm *cron_tm) {
  // If we're allowing any wday, we're not adjusting.
  if (cron->wday == WDAY_ANY || cron->wday == 0) {
    return false;
  }

  // Keep track of whether we've adjusted or not.
  bool adjusted = false;

  // We need to update cron_tm's tm_wday for proper checking.
  cron_tm->tm_mday += 1; // Adjustment because struct tm has mday 1-indexed for whatever reason
  mktime(cron_tm);
  cron_tm->tm_mday -= 1;
  // We have 1 week to find a fitting date
  for (int l = 0; l < DAYS_PER_WEEK; l++) {
    if (cron->wday & (1 << cron_tm->tm_wday)) {
      break;
    }
    // Advance the day.
    cron_tm->tm_mday++;
    cron_tm->tm_wday = (cron_tm->tm_wday + 1) % DAYS_PER_WEEK;
    adjusted = true;
  }
  return adjusted;
}

static time_t prv_get_execute_time_from_epoch(const CronJob *job, time_t local_epoch) {
  struct tm current_tm;
  // We work off of each element, so we need a struct tm.
  localtime_r(&local_epoch, &current_tm);

  // Adjust to be zero-indexed
  current_tm.tm_mday -= 1;

  // If the job isn't allowed to fire instantly, we're going to force the current second to be
  // 1, and the destination second to be 0. This works because it means we cannot use the current
  // time as-is, but it will not influence the other fields more than necessary.
  if (!job->may_be_instant) {
    current_tm.tm_sec = 1;
  }
  // Cron tm is based on the current tm
  struct tm cron_tm = current_tm;
  // Don't listen to this stuff (yet)
  cron_tm.tm_gmtoff = 0;
  cron_tm.tm_isdst = 0;

  // Access everything as arrays because it's way easier that way.
  int *dest_arr[CRON_INDEX_COUNT] = {
    &cron_tm.tm_year,
    &cron_tm.tm_mon,
    &cron_tm.tm_mday,
    &cron_tm.tm_hour,
    &cron_tm.tm_min,
    &cron_tm.tm_sec,
  };
  const int curr_arr[CRON_INDEX_COUNT] = {
    current_tm.tm_year,
    current_tm.tm_mon,
    current_tm.tm_mday,
    current_tm.tm_hour,
    current_tm.tm_min,
    current_tm.tm_sec,
  };
  const int spec_arr[CRON_INDEX_COUNT] = {
    CRON_YEAR_ANY, // year should always default
    job->month,
    job->mday,
    job->hour,
    job->minute,
    // If can be instant, second should default.
    // If it can't, use 0 because it's less than 1.
    job->may_be_instant ? CRON_SECOND_ANY : 0,
  };

/*
This is where the actual date finding is done. Essentially, we start with setting the result to
the local epoch, and modify from there.

We iterate over the fields from most significant to least significant. The reasoning for this is
that we will only know how to properly adjust a less significant field based on the value of the
more significant fields.

When a field in the spec is marked as ANY (-1), we need to decide what to put in the result:
 - If all values so far are still the same as the local epoch, we will use the local epoch's
   value.
 - Otherwise, the value stored will be 0, because the result is in the future, so a value of 0
   will definitely be the soonest time that matches.

Now, if the result is behind the local epoch, we step through higher order fields for a field
that was not specified. When we find one, we increase the value by 1. Since this is a higher
order field, this is guaranteed to put the result ahead of the local epoch.
*/

  // 'any' assignment defaults to using the local epoch's values.
  CronAssignMode assign_mode = CronAssignMode_LocalEpoch;
  // Iterate over all the fields
  for (int i = CRON_INDEX_YEAR; i < CRON_INDEX_COUNT; i++) {
    // If the spec had an 'any':
    if (spec_arr[i] <= CRON_GENERIC_ANY) {
      switch (assign_mode) {
        case CronAssignMode_LocalEpoch:
          // value will be the local epoch's value, we don't need to change anything
          break;
        case CronAssignMode_Zero:
          // value will be set to 0
          *(dest_arr[i]) = 0;
          break;
      }
    } else { // Otherwise, use the spec's value.
      *(dest_arr[i]) = spec_arr[i];
    }

    if (assign_mode == CronAssignMode_LocalEpoch) {
      // If we haven't started adjusting things yet, we need to do checking.

      const int direction = prv_future_past_direction(dest_arr, curr_arr);
      if (direction < 0) {
        // If the target is _behind_ the current time, we need to increase a higher order field.

        // Step from next highest all the way up to a year. We adjust the least significant field
        // that is more significant than the current field, and is unspec'd.
        for (int l = i - 1; l >= CRON_INDEX_YEAR; l--) {
          // If the field isn't set in the spec, increasing it by 1 will put us back in the
          // future.
          if (spec_arr[l] <= CRON_GENERIC_ANY) {
            *(dest_arr[l]) += 1;
            break;
          }
        }
      }
      if (direction != 0) {
        // The target is now ahead of the current time, the rest of the unspec'd fields
        // should be 0.
        assign_mode = CronAssignMode_Zero;
      }
    }
  }

  // Increase the day until we fit into the `wday` spec.
  if (prv_adjust_for_wday_spec(job, &cron_tm)) {
    // If the day has been adjusted, we need to re-set hour+minute+second.
    // Since we are definitely in the future on an adjustment, fields with 'any' should be
    // set to 0, otherwise set to the spec value.
    cron_tm.tm_hour = MAX(job->hour, 0);
    cron_tm.tm_min = MAX(job->minute, 0);
    // Second is always 0 when we're in the future.
    cron_tm.tm_sec = 0;
  }

  // Adjust back to 1-indexed
  cron_tm.tm_mday += 1;

  // Decide the DSTny (Adjust for DST transitions)
  cron_tm.tm_gmtoff = time_get_gmtoffset();
  cron_tm.tm_isdst = 0; // We'll do the DST adjust ourselves
  time_t t = mktime(&cron_tm);

  // Apply offset seconds
  t += job->offset_seconds;

  if (time_get_isdst(t)) {
    t -= time_get_dstoffset();
    if (!time_get_isdst(t)) {
      // We're in the hole where DST starts.
      // We want holed alarms to fire instantly, so set time to DST start time.
      t = time_get_dst_start();
    }
  }
  // We could be in the overlap where DST ends, but we don't actually care about it.
  // Why, you ask? This gives us the 'first' matching time if we ignore it.
  // So 1:30 will give us the first 1:30, not the second one.
  // Yes it's arbitrary. Yes it's confusing. But that's timekeeping and DST for you.

  return t;
}

time_t cron_job_get_execute_time_from_epoch(const CronJob *job, time_t local_epoch) {
  time_t t = prv_get_execute_time_from_epoch(job, local_epoch);

  if (job->offset_seconds != 0) {
    time_t t_last = t;
    time_t offset_epoch = local_epoch;
    while (true) {
      const time_t t_delta = (t - local_epoch) * ((job->offset_seconds > 0) ? 1 : -1);
      if ((job->may_be_instant ? (t_delta <= 0) : (t_delta < 0))) {
        break;
      }
      // Offset seconds is positive => Applying a positive offset seconds could result in a trigger
      // time after the nearest trigger time, find and check the previous time.
      // Offset seconds is negative => Applying a negative offset seconds resulted in a time before
      // local_epoch, calculate the next time.
      t_last = t;
      offset_epoch -= job->offset_seconds;
      const time_t rv = prv_get_execute_time_from_epoch(job, offset_epoch);
      t = rv < local_epoch ? t : rv;
      if (job->offset_seconds > 0 && t == t_last) {
        break;
      }
    }
  }

  return t;
}


time_t cron_job_get_execute_time(const CronJob *job) {
  return cron_job_get_execute_time_from_epoch(job, rtc_get_time());
}
