/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/types.h"
#include <pbl/drivers/rtc.h>
#include "syscall/syscall_internal.h"
#include "time.h"
#include "pbl/util/math.h"
#include <string.h>

// timezone abbreviation
static char s_timezone_abbr[TZ_LEN] = { 0 }; // longest timezone abbreviation is 5 char + null
static int32_t s_timezone_gmtoffset = 0;
static int32_t s_dst_adjust = SECONDS_PER_HOUR;
static time_t s_dst_start = 0;
static time_t s_dst_end = 0;

static const uint8_t s_mon_lengths[2][MONTHS_PER_YEAR] = {
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

static const uint16_t s_year_lengths[2] = {
  365,
  366
};

int32_t time_get_gmtoffset(void) {
  return s_timezone_gmtoffset;
}

bool time_get_isdst(time_t utc_time) {
  // do we have any DST set for the timezone we are in
  if ((s_dst_start == 0) || (s_dst_end == 0)) {
    return false;
  }

  return ((s_dst_start <= utc_time) && (utc_time < s_dst_end));
}

int time_will_transition_dst(time_t prev, time_t next) {
  if (time_get_isdst(prev) == time_get_isdst(next)) {
    return 0;
  } else if (time_get_isdst(prev)) {
    return time_get_dstoffset();
  } else {
    return -time_get_dstoffset();
  }
}

int32_t time_get_dstoffset(void) {
  return s_dst_adjust;
}

time_t time_get_dst_start(void) {
  return s_dst_start;
}

time_t time_get_dst_end(void) {
  return s_dst_end;
}

DEFINE_SYSCALL(time_t, sys_time_utc_to_local, time_t t) {
  return time_utc_to_local(t);
}

time_t time_utc_to_local(time_t utc_time) {
  utc_time += time_get_isdst(utc_time) ? s_dst_adjust : 0;
  utc_time += s_timezone_gmtoffset;
  return utc_time;
}

time_t time_local_to_utc(time_t local_time) {
  // Note that there is 1 hour a year where it is impossible to undo the DST offset based solely
  // on local time. For example, if the clock goes backward by 1 hour at 2am, then all times
  // between 1am and 2am will appear twice, and there is no way to tell which of the two
  // intervals we are being passed.
  local_time -= s_timezone_gmtoffset;
  local_time -= time_get_isdst(local_time - s_dst_adjust) ? s_dst_adjust : 0;
  return local_time;
}

void time_get_timezone_abbr(char *out_buf, time_t utc_time) {
  if (!out_buf) {
    return;
  }
  strncpy(out_buf, s_timezone_abbr, TZ_LEN);
  out_buf[TZ_LEN - 1] = 0;

  // Timezones with daylight savings, update modifier with current dst char
  // ie. P*T is PDT for daylight savings, PST for non-daylight savings
  char *tz_zone_dst_char = memchr(out_buf, '*', TZ_LEN);
  if (tz_zone_dst_char) {
    *tz_zone_dst_char = (time_get_isdst(utc_time)) ? 'D' : 'S';
    // Workaround for UK Winter, Greenwich Mean Time; UK Summer, British Summer Time
    if (!strncmp(out_buf, "BDT", TZ_LEN)) {
      strncpy(out_buf, "BST", TZ_LEN);
    } else if (!strncmp(out_buf, "BST", TZ_LEN)) {
      strncpy(out_buf, "GMT", TZ_LEN);
    }
  }
}

// converts time_t to struct tm for localtime and gmtime
struct tm *time_to_tm(const time_t * tim_p, struct tm *res, bool utc_mode) {
  time_t local_time;
  time_t utc_time = *tim_p;
  if (utc_mode) {
    res->tm_gmtoff = 0;
    res->tm_isdst = 0;
    strncpy(res->tm_zone, "UTC", TZ_LEN);
    res->tm_zone[TZ_LEN - 1] = '\0';
    local_time = utc_time;
  } else {
    res->tm_isdst = time_get_isdst(utc_time);
    res->tm_gmtoff = time_get_gmtoffset() + (res->tm_isdst ? s_dst_adjust : 0);
    time_get_timezone_abbr(res->tm_zone, utc_time);
    local_time = utc_time + res->tm_gmtoff;
  }

  int32_t days = local_time / SECONDS_PER_DAY;
  int32_t rem = local_time % SECONDS_PER_DAY;
  while (rem < 0) {
    rem += SECONDS_PER_DAY;
    --days;
  }

  while (rem >= SECONDS_PER_DAY) {
    rem -= SECONDS_PER_DAY;
    ++days;
  }

  /* compute hour, min, and sec */
  res->tm_hour = (int) (rem / SECONDS_PER_HOUR);
  rem %= SECONDS_PER_HOUR;
  res->tm_min = (int) (rem / SECONDS_PER_MINUTE);
  res->tm_sec = (int) (rem % SECONDS_PER_MINUTE);

  /* compute day of week */
  if ((res->tm_wday = ((EPOCH_WDAY + days) % DAYS_PER_WEEK)) < 0) {
    res->tm_wday += DAYS_PER_WEEK;
  }

  /* compute year & day of year */
  int y = EPOCH_YEAR;
  int yleap;
  if (days >= 0) {
    for (;;) {
      yleap = YEAR_IS_LEAP(y);
      if (days < s_year_lengths[yleap]) {
        break;
      }
      y++;
      days -= s_year_lengths[yleap];
    }
  } else {
    do {
      --y;
      yleap = YEAR_IS_LEAP(y);
      days += s_year_lengths[yleap];
    } while (days < 0);
  }

  res->tm_year = y - TM_YEAR_ORIGIN;
  res->tm_yday = days;
  const uint8_t *ip = s_mon_lengths[yleap];
  for (res->tm_mon = 0; days >= ip[res->tm_mon]; ++res->tm_mon) {
    days -= ip[res->tm_mon];
  }
  res->tm_mday = days + 1;

  return res;
}

struct tm *gmtime_r(const time_t *timep, struct tm *result) {
  return time_to_tm(timep, result, true);
}

struct tm *localtime_r(const time_t *timep, struct tm *result) {
  return time_to_tm(timep, result, false);
}

void time_util_split_seconds_into_parts(uint32_t seconds,
      uint32_t *day_part, uint32_t *hour_part, uint32_t *minute_part, uint32_t *second_part) {
  *day_part = seconds / SECONDS_PER_DAY;
  seconds -= *day_part * SECONDS_PER_DAY;

  *hour_part = seconds / SECONDS_PER_HOUR;
  seconds -= *hour_part * SECONDS_PER_HOUR;

  *minute_part = seconds / SECONDS_PER_MINUTE;
  seconds -= *minute_part * SECONDS_PER_MINUTE;

  *second_part = seconds;
}

int time_util_get_num_hours(int hours, bool is24h) {
  return is24h ? hours : (hours + 12 - 1) % 12 + 1;
}

int time_util_get_seconds_until_daily_time(struct tm *time, int hour, int minute) {
  int hour_diff = hour - time->tm_hour;

  if (hour < time->tm_hour || (hour == time->tm_hour && minute <= time->tm_min)) {
    // It's past the mark, skip to tomorrow.
    hour_diff += HOURS_PER_DAY;
  }

  int minutes_diff = (hour_diff * MINUTES_PER_HOUR) + (minute - time->tm_min);
  return (minutes_diff * SECONDS_PER_MINUTE) - (time->tm_sec);
}

void time_util_update_timezone(const TimezoneInfo *tz_info) {
  strncpy(s_timezone_abbr, tz_info->tm_zone, sizeof(tz_info->tm_zone) + 0);
  s_timezone_abbr[TZ_LEN - 1] = '\0';
  s_timezone_gmtoffset = tz_info->tm_gmtoff;
  s_dst_start = tz_info->dst_start;
  s_dst_end = tz_info->dst_end;
  // Lord Howe Island has a half-hour DST
  if (tz_info->dst_id == DSTID_LORDHOWE) {
    s_dst_adjust = SECONDS_PER_HOUR / 2;
  } else {
    s_dst_adjust = SECONDS_PER_HOUR;
  }
}

time_t time_util_get_midnight_of(time_t ts) {
  struct tm tm;
  localtime_r(&ts, &tm);
  tm.tm_hour = 0;
  tm.tm_min = 0;
  tm.tm_sec = 0;
  return mktime(&tm);
}

bool time_util_range_spans_day(time_t start, time_t end, time_t start_of_day) {
  return (start <= start_of_day && end >= (start_of_day + SECONDS_PER_DAY));
}

time_t time_utc_to_local_using_offset(time_t utc_time, int16_t utc_offset_min) {
  if (utc_offset_min < 0) {
    return utc_time - (time_t) ABS(utc_offset_min) * SECONDS_PER_MINUTE;
  } else {
    return utc_time + (time_t) utc_offset_min * SECONDS_PER_MINUTE;
  }
}

time_t time_local_to_utc_using_offset(time_t local_time, int16_t utc_offset_min) {
  if (utc_offset_min < 0) {
    return local_time + (time_t) ABS(utc_offset_min) * SECONDS_PER_MINUTE;
  } else {
    return local_time - (time_t) utc_offset_min * SECONDS_PER_MINUTE;
  }
}

time_t time_util_utc_to_local_offset(void) {
  time_t now = rtc_get_time();
  return (time_utc_to_local(now) - now);
}

// ---------------------------------------------------------------------------------------
DayInWeek time_util_get_day_in_week(time_t utc_sec) {
  struct tm local_tm;
  localtime_r(&utc_sec, &local_tm);
  return local_tm.tm_wday;
}

// ---------------------------------------------------------------------------------------
uint16_t time_util_get_day(time_t utc_sec) {
  // Convert to local seconds
  time_t local_sec = time_utc_to_local(utc_sec);

  // Figure out the day index
  return (local_sec / SECONDS_PER_DAY);
}

// ---------------------------------------------------------------------------------------
int time_util_get_minute_of_day(time_t utc_sec) {
  struct tm local_tm;
  localtime_r(&utc_sec, &local_tm);
  return (local_tm.tm_hour * MINUTES_PER_HOUR) + local_tm.tm_min;
}

// ---------------------------------------------------------------------------------------
int time_util_minute_of_day_adjust(int minute, int delta) {
  minute += delta;
  if (minute < 0) {
    minute += MINUTES_PER_DAY;
  } else if (minute >= MINUTES_PER_DAY) {
    minute -= MINUTES_PER_DAY;
  }
  return minute;
}

// ---------------------------------------------------------------------------------------
time_t time_start_of_today(void) {
  time_t now = rtc_get_time();
  return time_util_get_midnight_of(now);
}

DEFINE_SYSCALL(time_t, sys_time_start_of_today, void) {
  return time_start_of_today();
}

// ---------------------------------------------------------------------------------------
uint32_t time_get_uptime_seconds(void) {
  RtcTicks ticks = rtc_get_ticks();
  return ticks / PBL_TICK_HZ;
}
