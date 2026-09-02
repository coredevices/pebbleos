/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/sched.h"
#include "pbl/kernel/types.h"
#include <stdint.h>

#include "board/board.h"
#include <pbl/drivers/flash.h>
#include <pbl/drivers/rtc.h>
#include "flash_region/flash_region.h"
#include "kernel/events.h"
#include "pbl/mcu/interrupts.h"
#include "system/passert.h"
#include "util/time/time.h"
#include <pbl/logging/logging.h>
#include "pbl/services/new_timer/new_timer.h"

#include "bf0_hal_rtc.h"

PBL_LOG_MODULE_DEFINE(driver_rtc_sf32lb, CONFIG_DRIVER_RTC_LOG_LEVEL);

// The RTC clock, CLK_RTC, can be configured to use the LXT32 (32.768 kHz) or
// LRC10 (9.8 kHz). The prescaler values need to be set such that the CLK1S
// event runs at 1 Hz. The formula that relates prescaler values with the
// clock frequency is as follows:
//
//   F(CLK1S) = CLK_RTC / (DIV_A_INT + DIV_A_FRAC / 2^14) / DIV_B

#ifndef SF32LB52_USE_LXT
#define DIV_A_INT 38U
#define DIV_A_FRAC 4608U
#define DIV_B 256U

// Default RC10K cycles value on 48MHz clock
#define RC10K_DEFAULT_CYCLES 1200000UL

// The deviation limit of the current calibration value of RC10K relative to the average
// calibration value.
#define MAX_DELTA_BETWEEN_RTC_AVE (HAL_RC_CAL_GetLPCycle() / 2)

// Calibration period in milliseconds
#define RTC_CAL_PERIOD_MS 60000

// Maximum reasonable correction in seconds. If the calculated correction exceeds this,
// something is wrong and we should reset calibration state instead of applying it.
// 60 seconds is generous - normal drift should be milliseconds per calibration cycle.
#define MAX_REASONABLE_CORRECTION_SECS 60

static TimerID s_rtc_cal_timer;

// Calibration state - must be reset when RTC time is set externally
static uint32_t s_rtc_cycle_count_init = 0;
static double s_rtc_a = 0.0;
static double s_delta_total = 0.0;

static void prv_reset_calibration_state(void) {
  s_rtc_cycle_count_init = 0;
  s_rtc_a = 0.0;
  s_delta_total = 0.0;
}

// Forward declaration - defined later in file
static void prv_rtc_set_time_no_cal_reset(time_t time);
#else
#define DIV_A_INT 128U
#define DIV_A_FRAC 0U
#define DIV_B 256U
#endif

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

static bool s_initialized = false;

#ifndef SF32LB52_USE_LXT
static uint32_t prv_rtc_get_lpcycle() {
  uint32_t value;

  value = HAL_RC_CAL_get_average_cycle_on_48M();
  if (value == 0UL) {
    value = RC10K_DEFAULT_CYCLES;
  }

  HAL_Set_backup(RTC_BACKUP_LPCYCLE, value);

  return value;
}

void prv_rtc_rc10_calculate_div(RTC_HandleTypeDef* hdl, uint32_t value) {
  hdl->Init.DivB = RC10K_SUB_SEC_DIVB;

  // 1 seconds has total 1/(x/(48*8))/256=1.5M/x cycles, times 2^14 for DIVA
  uint32_t divider = RTC_Handler.Init.DivB * value;
  value = (48000000ULL * HAL_RC_CAL_GetLPCycle() * (1 << 14) + (divider >> 1)) / divider;
  hdl->Init.DivAInt = (uint32_t)(value >> 14);
  hdl->Init.DivAFrac = (uint32_t)(value & ((1 << 14) - 1));
}

static void prv_rtc_reconfig() {
  uint32_t cur_ave;
  HAL_StatusTypeDef ret;
  cur_ave = prv_rtc_get_lpcycle();
  prv_rtc_rc10_calculate_div(&RTC_Handler, cur_ave);

  ret = HAL_RTC_Init(&RTC_Handler, RTC_INIT_REINIT);
  PBL_ASSERTN(ret == HAL_OK);
}

static void prv_rtc_cal_timer_cb(void* data) {
  if (s_rtc_cycle_count_init == 0) {
    uint16_t sub;
    time_t t;

    prv_rtc_reconfig();
    // Get initial lpcycle, RTC is running based on it.
    s_rtc_cycle_count_init = HAL_Get_backup(RTC_BACKUP_LPCYCLE);
    s_delta_total = 0.0;
    rtc_get_time_ms(&t, &sub);
    s_rtc_a = 1.0 * t + ((float)(1.0 * sub)) / RC10K_SUB_SEC_DIVB;
  } else {
    uint16_t sub2 = 0;
    double rtc_cal = 0.0;
    double delta = 0.0;
    time_t t2;
    uint32_t cur_ave;
    uint32_t ref_cycle;
    double rtc_b;

    rtc_get_time_ms(&t2, &sub2);
    cur_ave = HAL_RC_CAL_get_average_cycle_on_48M();
    ref_cycle = cur_ave;
    rtc_b = 1.0 * t2 + ((double)(1.0 * sub2)) / RC10K_SUB_SEC_DIVB;

    // Delta time between s_rtc_a to rtc_b, in seconds.
    delta = rtc_b - s_rtc_a;
    // Calculate accurate rtc_b
    rtc_cal = delta * ref_cycle / s_rtc_cycle_count_init + s_rtc_a;
    // Detla time of accurrate rtc_b and current rtc_b
    delta = rtc_cal - rtc_b;

    // Accumulate error
    s_delta_total += delta;

    // Sanity check: if accumulated error is unreasonably large, the RTC time was likely
    // changed externally. Reset calibration state instead of applying a bogus correction.
    if (s_delta_total > MAX_REASONABLE_CORRECTION_SECS ||
        s_delta_total < -MAX_REASONABLE_CORRECTION_SECS) {
      PBL_LOG_WRN("RTC calibration: delta_sum=%d exceeds max, resetting calibration state",
              (int)(s_delta_total * 1000));
      prv_reset_calibration_state();
      return;
    }

    if (s_delta_total > 1.0 || s_delta_total < -1.0) {
      // Accurate time
      rtc_cal = s_delta_total + rtc_b;
      // Apply integral part difference.
      prv_rtc_set_time_no_cal_reset((uint32_t)rtc_cal);
      // Continue with subseconds
      s_delta_total = rtc_cal - (uint32_t)rtc_cal;
      // Next interval start time
      s_rtc_a = (uint32_t)rtc_cal;
      if ((cur_ave > s_rtc_cycle_count_init &&
           (cur_ave - s_rtc_cycle_count_init) > MAX_DELTA_BETWEEN_RTC_AVE) ||
          (cur_ave < s_rtc_cycle_count_init &&
           (s_rtc_cycle_count_init - cur_ave) > MAX_DELTA_BETWEEN_RTC_AVE)) {
        prv_rtc_reconfig();
        s_rtc_cycle_count_init = HAL_Get_backup(RTC_BACKUP_LPCYCLE);
      }
    } else {
      // Next interval start time
      s_rtc_a = rtc_b;
    }

    PBL_LOG_DBG("origin: f=%dHz,cycle=%d avr: f=%dHz cycle_ave=%d delta=%d, delta_sum=%d\n",
            (int)(48000000ULL * HAL_RC_CAL_GetLPCycle() / s_rtc_cycle_count_init),
            (int)s_rtc_cycle_count_init,
            (int)(48000000ULL * HAL_RC_CAL_GetLPCycle() / ref_cycle),
            (int)ref_cycle,
            (int)(delta * 1000), (int)(s_delta_total * 1000));
  }
}
#endif

void rtc_init(void) {
#ifdef SF32LB52_USE_LXT
  HAL_StatusTypeDef ret;

  ret = HAL_PMU_LXTReady();
  PBL_ASSERTN(ret == HAL_OK);

  ret = HAL_RTC_Init(&RTC_Handler, RTC_INIT_NORMAL);
  PBL_ASSERTN(ret == HAL_OK);
#else
  prv_rtc_reconfig();
#endif

  s_initialized = true;
}

void rtc_init_timers(void) {}

static RtcTicks get_ticks(void) {
  static pbl_tick_t s_last_freertos_tick_count = 0;
  static RtcTicks s_coarse_ticks = 0;

  bool ints_enabled = mcu_state_are_interrupts_enabled();
  if (ints_enabled) {
    __disable_irq();
  }

  pbl_tick_t freertos_tick_count = pbl_uptime_ticks();
  if (freertos_tick_count < s_last_freertos_tick_count) {
    pbl_tick_t rollover_amount = -1;
    s_coarse_ticks += rollover_amount;
  }

  s_last_freertos_tick_count = freertos_tick_count;
  RtcTicks ret_value = freertos_tick_count + s_coarse_ticks;

  if (ints_enabled) {
    __enable_irq();
  }

  return ret_value;
}

// Internal function to set RTC time without resetting calibration state.
// Used by the calibration code when applying corrections.
static void prv_rtc_set_time_no_cal_reset(time_t time) {
  // Capture old time before changing it to send proper event
  time_t old_time = rtc_get_time();

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

  // Send clock change event to notify system components (e.g., DND timer scheduler)
  // This ensures long-duration timers are properly rescheduled after calibration adjustments
  int32_t time_delta = (int32_t)(time - old_time);
  if (time_delta != 0) {
    PebbleEvent e = {
      .type = PEBBLE_SET_TIME_EVENT,
      .set_time_info = {
        .utc_time_delta = time_delta,
        .gmt_offset_delta = 0,
        .dst_changed = false,
      }
    };
    event_put(&e);
  }
}

void rtc_set_time(time_t time) {
#ifndef SF32LB52_USE_LXT
  // Reset calibration state when time is set externally to prevent
  // the calibration algorithm from computing bogus corrections based on
  // stale reference times.
  prv_reset_calibration_state();
#endif
  prv_rtc_set_time_no_cal_reset(time);
}

void rtc_get_time_ms(time_t* out_seconds, uint16_t* out_ms) {
  RTC_DateTypeDef rtc_date;
  RTC_TimeTypeDef rtc_time;

  if (s_initialized) {
    while((RTC_Handler.Instance->ISR & RTC_ISR_RSF) == (uint32_t)RESET) {
      // Wait for RTC registers to synchronize
    }
  }

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

RtcTicks rtc_get_ticks(void) {
  return get_ticks();
}

void rtc_alarm_init(void) {}

void rtc_alarm_set(RtcTicks num_ticks) {}

RtcTicks rtc_alarm_get_elapsed_ticks(void) {
  return 0;
}

bool rtc_alarm_is_initialized(void) {
  return true;
}

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

const char* rtc_get_time_string(char* buffer) {
  return time_t_to_string(buffer, rtc_get_time());
}

const char* time_t_to_string(char* buffer, time_t t) {
  struct tm time;
  localtime_r(&t, &time);

  strftime(buffer, TIME_STRING_BUFFER_SIZE, "%c", &time);

  return buffer;
}

//! We store timezone info in the flash TZINFO region

//! Versioned storage structure for timezone info in flash
//! This allows for future migrations and avoids struct alignment issues
typedef struct __attribute__((packed)) {
  uint8_t version;            // Version number for future migrations
  char tm_zone[TZ_LEN - 1];   // Up to 5 character timezone abbreviation
  uint8_t dst_id;             // Daylight savings time zone index
  int16_t timezone_id;        // Olson index of timezone
  int32_t tm_gmtoff;          // GMT time offset
  time_t dst_start;           // Timestamp of start of DST period (0 if none)
  time_t dst_end;             // Timestamp of end of DST period (0 if none)
} TzinfoFlashStorage;

#define TZINFO_VERSION 1

void rtc_set_timezone(TimezoneInfo* tzinfo) {
  _Static_assert(sizeof(TzinfoFlashStorage) <= SUBSECTOR_SIZE_BYTES,
      "TzinfoFlashStorage must fit in TZINFO flash region (4KB)");

  // Copy to versioned buffer
  TzinfoFlashStorage storage = {
    .version = TZINFO_VERSION,
    .dst_id = tzinfo->dst_id,
    .timezone_id = tzinfo->timezone_id,
    .tm_gmtoff = tzinfo->tm_gmtoff,
    .dst_start = tzinfo->dst_start,
    .dst_end = tzinfo->dst_end,
  };
  memcpy(storage.tm_zone, tzinfo->tm_zone, TZ_LEN - 1);

  flash_erase_subsector_blocking(FLASH_REGION_TZINFO_BEGIN);
  flash_write_bytes((const uint8_t*)&storage, FLASH_REGION_TZINFO_BEGIN, sizeof(TzinfoFlashStorage));
}

void rtc_get_timezone(TimezoneInfo* tzinfo) {
  TzinfoFlashStorage storage;

  flash_read_bytes((uint8_t*)&storage, FLASH_REGION_TZINFO_BEGIN, sizeof(TzinfoFlashStorage));

  if (storage.version != TZINFO_VERSION) {
    // Future versions can handle migrations here
    // For now, treat invalid version as unset
    memset(tzinfo, 0, sizeof(TimezoneInfo));
    return;
  }

  memcpy(tzinfo->tm_zone, storage.tm_zone, TZ_LEN - 1);
  tzinfo->dst_id = storage.dst_id;
  tzinfo->timezone_id = storage.timezone_id;
  tzinfo->tm_gmtoff = storage.tm_gmtoff;
  tzinfo->dst_start = storage.dst_start;
  tzinfo->dst_end = storage.dst_end;
}

void rtc_timezone_clear(void) {
  flash_erase_subsector_blocking(FLASH_REGION_TZINFO_BEGIN);
}

uint16_t rtc_get_timezone_id(void) {
  TimezoneInfo tzinfo;

  rtc_get_timezone(&tzinfo);

  return (uint16_t)tzinfo.timezone_id;
}

bool rtc_is_timezone_set(void) {
  uint8_t version;

  flash_read_bytes((uint8_t*)&version, FLASH_REGION_TZINFO_BEGIN, sizeof(version));

  return version == TZINFO_VERSION;
}

void rtc_enable_backup_regs(void) {}

void rtc_calibrate_frequency(uint32_t frequency) {
#ifndef SF32LB52_USE_LXT
  prv_rtc_cal_timer_cb(NULL);

  s_rtc_cal_timer = new_timer_create();
  PBL_ASSERTN(s_rtc_cal_timer != TIMER_INVALID_ID);

  bool success = new_timer_start(s_rtc_cal_timer, RTC_CAL_PERIOD_MS,
                                 prv_rtc_cal_timer_cb, NULL,
                                 TIMER_START_FLAG_REPEATING);
  PBL_ASSERTN(success);
#endif
}
