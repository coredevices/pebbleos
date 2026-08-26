/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <time.h>

#include "pbl/util/list.h"

//! @file cron.h
//! Wall-clock based timer system. Designed for use in things such as alarms, calendar events, etc.
//! Properly handles DST, etc.

typedef struct CronJob CronJob;

typedef void (*CronJobCallback)(CronJob *job, void* data);

//! Matches any possible value.
#define CRON_MINUTE_ANY (-1)
#define CRON_HOUR_ANY (-1)
#define CRON_MDAY_ANY (-1)
#define CRON_MONTH_ANY (-1)

#define WDAY_SUNDAY (1 << 0)
#define WDAY_MONDAY (1 << 1)
#define WDAY_TUESDAY (1 << 2)
#define WDAY_WEDNESDAY (1 << 3)
#define WDAY_THURSDAY (1 << 4)
#define WDAY_FRIDAY (1 << 5)
#define WDAY_SATURDAY (1 << 6)

#define WDAY_WEEKDAYS (WDAY_MONDAY | WDAY_TUESDAY | WDAY_WEDNESDAY | WDAY_THURSDAY | WDAY_FRIDAY)
#define WDAY_WEEKENDS (WDAY_SUNDAY | WDAY_SATURDAY)
#define WDAY_ANY (WDAY_WEEKENDS | WDAY_WEEKDAYS)

struct CronJob {
  //! internal, no touchy
  ListNode list_node;

  //! Cached execution timestamp in UTC.
  //! This is set by `cron_job_schedule`, and is required to never be changed once the job has been
  //! added.
  time_t cached_execute_time;

  //! Callback that is called when the job fires.
  CronJobCallback cb;
  void* cb_data;

  //! Occasionally, the system gets a clock change event for various reasons:
  //!  - User changed time-zones or a DST transition happened
  //!  - User changed the time
  //!  - Phone sent the current time and was different from ours, so we took theirs.
  //! In the first case, the cron job's execute time will always be recalculated.
  //! In the other two, we see if the time difference from the old time is >= this.
  //! If it is, then we'll recalculate. Otherwise, we leave the calculated time alone.
  //! In this way, 0 will always recalculate, and UINT32_MAX will never recalculate.
  //!
  //! Recalculating would essentially mean that a job that was "skipped over" will not fire until
  //! the next match. If recalculation is not done, but the job was skipped over, it will fire
  //! instantly.
  //!
  //! This value is specified in seconds.
  uint32_t clock_change_tolerance;

  int8_t minute; //!< 0-59, or CRON_MINUTE_ANY
  int8_t hour; //!< 0-23, or CRON_HOUR_ANY
  int8_t mday; //!< 0-30, or CRON_MDAY_ANY
  int8_t month; //!< 0-11, or CRON_MONTH_ANY

  //! Seconds to offset the cron execution time applied after regular cron job time calculation.
  //! For example, a cron scheduled for Monday at 0:15 with an offset of negative 30min will fire
  //! on Sunday at 23:45.
  int32_t offset_seconds;

  union {
    uint8_t flags;

    struct {
      //! This should be any combination of WDAY_*. If zero, acts like WDAY_ANY.
      uint8_t wday : 7;

      //! If this flag is set, the resulting execution time may be equal to the local epoch.
      //! Having it set could be used for some event that must happen at the specified time even if
      //! that time is right now.
      bool may_be_instant : 1;
    };
  };
};

//! Add a cron job. This will make the service hold a reference to the specified job, so it must
//! not leave scope or be destroyed until it is unscheduled.
//! The job only gets scheduled once. For re-scheduling, you can call this on the job again.
//! @param job pointer to the CronJob struct to be scheduled.
//! @returns time_t for when the job is destined to go off.
time_t cron_job_schedule(CronJob *job);

//! Schedule a cron job to run after another cron job.
//! This will make the service hold a reference to the new job, so it must
//! not leave scope or be destroyed until it is unscheduled.
//! @param job pointer to the CronJob after which we want our job to run. job must be scheduled.
//! @param new_job pointer to the CronJob struct to be scheduled. new_job must be unscheduled.
//! @returns time_t for when the job is destined to go off.
//! @note This API makes no guarantee that the two jobs will be scheduled back to back,
//! only that new_job will have the same scheduled time as job and that it will trigger
//! strictly after job.
time_t cron_job_schedule_after(CronJob *new_job, CronJob *job);

//! Remove a scheduled cron job.
//! @param job pointer to the CronJob struct to be unscheduled.
//! @return true if the job was successfully removed (false may indicate no job was
//!  scheduled at all or the cb is currently executing)
bool cron_job_unschedule(CronJob *job);

//! Check if a cron job is scheduled.
//! @param job pointer to the CronJob struct to be checked for being scheduled.
//! @returns true if scheduled or pending deletion, false otherwise
bool cron_job_is_scheduled(CronJob *job);

//! Calculate cron job's destined execution time, from the current time.
//! @param job pointer to the CronJob struct to get the execution time for.
//! @returns time_t for when the job is destined to go off.
time_t cron_job_get_execute_time(const CronJob *job);

//! Calculate cron job's destined execution time if it were scheduled at the given time.
//! @param job pointer to the CronJob struct to get the execution time for.
//! @param local_epoch the epoch for getting the job's execution time.
//! @returns time_t for when the job is destined to go off.
time_t cron_job_get_execute_time_from_epoch(const CronJob *job, time_t local_epoch);

//! Schedules a cron job to fire at the destined execution time calculated from a specific epoch.
//! @params job pointer to the CronJob struct to be scheduled.
//! @params from_epoch the epoch from which to compute the next execution time.
//! @returns time_t for when the job is scheduled to go off.
time_t cron_job_schedule_from_epoch(CronJob *job, time_t from_epoch);
