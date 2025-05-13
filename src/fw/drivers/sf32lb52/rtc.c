/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>

#include "drivers/rtc.h"
#include "drivers/rtc_private.h"
#include "system/passert.h"
#include "util/time/time.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bf0_hal_rtc.h"

// The RTC clock, CLK_RTC, can be configured to use the LXT32 (32.768 kHz) or
// LRC10 (9.8 kHz). The prescaler values need to be set such that the CLK1S
// event runs at 1 Hz. The formula that relates prescaler values with the
// clock frequency is as follows:
//
//   F(CLK1S) = CLK_RTC / (DIV_A_INT + DIV_A_FRAC / 2^14) / DIV_B
#define DIV_A_INT 128
#define DIV_A_FRAC 0
#define DIV_B 256

static RTC_HandleTypeDef RTC_Handler = {
    .Instance = (RTC_TypeDef*)RTC_BASE,
    .Init =
        {
            .HourFormat = RTC_HOURFORMAT_24,
            .DivAInt = DIV_A_INT,
            .DivAFrac = DIV_A_FRAC,
            .DivB = DIV_B,
        },
};

void rtc_init(void) {
  HAL_StatusTypeDef ret;

  ret = HAL_PMU_LXTReady();
  PBL_ASSERTN(ret == HAL_OK);

  ret = HAL_RTC_Init(&RTC_Handler, RTC_INIT_NORMAL);
  PBL_ASSERTN(ret == HAL_OK);
}

void rtc_init_timers(void) {}

static RtcTicks get_ticks(void) {
  static TickType_t s_last_freertos_tick_count = 0;
  static RtcTicks s_coarse_ticks = 0;

  bool ints_enabled = mcu_state_are_interrupts_enabled();
  if (ints_enabled) {
    __disable_irq();
  }

  TickType_t freertos_tick_count = xTaskGetTickCount();
  if (freertos_tick_count < s_last_freertos_tick_count) {
    TickType_t rollover_amount = -1;
    s_coarse_ticks += rollover_amount;
  }

  s_last_freertos_tick_count = freertos_tick_count;
  RtcTicks ret_value = freertos_tick_count + s_coarse_ticks;

  if (ints_enabled) {
    __enable_irq();
  }

  return ret_value;
}

void rtc_set_time(time_t time) {
  struct tm t;
  gmtime_r(&time, &t);

  PBL_ASSERTN(!rtc_sanitize_struct_tm(&t));

  RTC_TimeTypeDef rtc_time_struct = {.Hours = t.tm_hour, .Minutes = t.tm_min, .Seconds = t.tm_sec};

  RTC_DateTypeDef rtc_date_struct = {
      .Month = t.tm_mon + 1,
      .Date = t.tm_mday,
      .Year = t.tm_year % 100,
  };

  HAL_RTC_SetTime(&RTC_Handler, &rtc_time_struct, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&RTC_Handler, &rtc_date_struct, RTC_FORMAT_BIN);
}

void rtc_get_time_ms(time_t* out_seconds, uint16_t* out_ms) {
  RTC_DateTypeDef rtc_date;
  RTC_TimeTypeDef rtc_time;

  HAL_RTC_GetTime(&RTC_Handler, &rtc_time, RTC_FORMAT_BIN);
  while (HAL_RTC_GetDate(&RTC_Handler, &rtc_date, RTC_FORMAT_BIN) == HAL_ERROR) {
    // HAL_ERROR is returned if a rollover occurs, so just keep trying
    HAL_RTC_GetTime(&RTC_Handler, &rtc_time, RTC_FORMAT_BIN);
  };

  struct tm current_time = {
      .tm_sec = rtc_time.Seconds,
      .tm_min = rtc_time.Minutes,
      .tm_hour = rtc_time.Hours,
      .tm_mday = rtc_date.Date,
      .tm_mon = rtc_date.Month - 1,
      .tm_year = rtc_date.Year + 100,
      .tm_wday = rtc_date.WeekDay,
      .tm_yday = 0,
      .tm_isdst = 0,
  };

  *out_seconds = mktime(&current_time);
  *out_ms = (uint16_t)((rtc_time.SubSeconds * 1000) / DIV_B);
}

time_t rtc_get_time(void) {
  time_t seconds;
  uint16_t ms;

  rtc_get_time_ms(&seconds, &ms);

  return seconds;
}

RtcTicks rtc_get_ticks(void) { return get_ticks(); }

void rtc_alarm_init(void) {}

void rtc_alarm_set(RtcTicks num_ticks) {}

RtcTicks rtc_alarm_get_elapsed_ticks(void) { return 0; }

bool rtc_alarm_is_initialized(void) { return true; }

bool rtc_sanitize_struct_tm(struct tm* t) {
  // These values come from time_t (which suffers from the 2038 problem) and our hardware which
  // only stores a 2 digit year, so we only represent values after 2000.

  // Remember tm_year is years since 1900.
  if (t->tm_year < 100) {
    // Bump it up to the year 2000 to work with our hardware.
    t->tm_year = 100;
    return true;
  } else if (t->tm_year > 137) {
    t->tm_year = 137;
    return true;
  }
  return false;
}

bool rtc_sanitize_time_t(time_t* t) {
  struct tm time_struct;
  gmtime_r(t, &time_struct);

  const bool result = rtc_sanitize_struct_tm(&time_struct);
  *t = mktime(&time_struct);

  return result;
}

void rtc_get_time_tm(struct tm* time_tm) {
  time_t t = rtc_get_time();
  localtime_r(&t, time_tm);
}

const char* rtc_get_time_string(char* buffer) { return time_t_to_string(buffer, rtc_get_time()); }

const char* time_t_to_string(char* buffer, time_t t) {
  struct tm time;
  localtime_r(&t, &time);

  strftime(buffer, TIME_STRING_BUFFER_SIZE, "%c", &time);

  return buffer;
}

//! We attempt to save registers by placing both the timezone abbreviation
//! timezone index and the daylight_savingtime into the same register set
void rtc_set_timezone(TimezoneInfo* tzinfo) {}

void rtc_get_timezone(TimezoneInfo* tzinfo) {}

void rtc_timezone_clear(void) {}

uint16_t rtc_get_timezone_id(void) { return 0; }

bool rtc_is_timezone_set(void) { return 0; }

void rtc_enable_backup_regs(void) {}

void rtc_calibrate_frequency(uint32_t frequency) {}
