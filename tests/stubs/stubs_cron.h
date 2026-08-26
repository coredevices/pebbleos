/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pebbleos/cron.h>

time_t cron_job_get_execute_time(const CronJob *job) {
  return 0;
}

time_t cron_job_get_execute_time_from_epoch(const CronJob *job, time_t local_epoch) {
  return 0;
}

time_t cron_job_schedule(CronJob *job) {
  return 0;
}

time_t cron_job_schedule_from_epoch(CronJob *job, time_t from_epoch) {
  return 0;
}

bool cron_job_unschedule(CronJob *job) {
  return true;
}
