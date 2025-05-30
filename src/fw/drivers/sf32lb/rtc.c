#include "drivers/rtc.h"
#include "drivers/rtc_private.h"
#include "drivers/stm32f2/rtc_calibration.h"

#include "console/dbgserial.h"

#include "drivers/exti.h"
#include "drivers/periph_config.h"
#include "drivers/watchdog.h"

#include "kernel/util/stop.h"
#include "mcu/interrupts.h"

#include "services/common/regular_timer.h"

#include "system/bootbits.h"
#include "system/logging.h"
#include "system/passert.h"
#include "system/reset.h"

#include "util/time/time.h"

#define SF32LB_COMPATIBLE
#include <mcu.h>

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static RTC_HandleTypeDef RTC_Handler = {
  .Instance = (RTC_TypeDef *) RTC_BASE
};

typedef uint32_t RtcIntervalTicks;

static const unsigned int LSE_FREQUENCY_HZ = 32768;
#define SECONDS_IN_A_DAY (60 * 60 * 24)
#define TICKS_IN_AN_INTERVAL SECONDS_IN_A_DAY

//! This variable is a UNIX timestamp of what the current wall clock time was at tick s_time_tick_base.
static time_t s_time_base = 0;
//! This variable is the tick where the wall clock time was equal to s_time_base. If you subtract this variable
//! from the current tick count, you'll get the number of ticks that have elapsed since s_time_base, which will
//! allow you to calculate the current wall clock time. Note that this value may be negative on startup, see
//! restore_rtc_time_state
static int64_t s_time_tick_base = 0;

//! The value of the RTC registers last time we checked them.
static RtcIntervalTicks s_last_ticks = 0;
//! This value is added to the current value of the RTC ticks to get the number
//! of ticks since system start. Incremented whenever we detect a rollover.
static RtcTicks s_coarse_ticks = 1;

//! The time that we last set the alarm at. See rtc_alarm_set and rtc_alarm_get_elapsed_ticks.
static RtcTicks s_alarm_set_time = 0;

static bool s_tick_alarm_initialized = false;

static void save_rtc_time_state(RtcIntervalTicks current_rtc_ticks);

//! Our RTC tick counter actually overflows once every 86 seconds. If we don't call rtc_get_ticks() every 86 seconds,
//! the counter may roll over multiple times, causing our clock to appear to have gaps. This repeating callback allows
//! us to make sure this doesn't happen.
static void rtc_resync_timer_callback() {
  rtc_get_ticks();
}

static uint8_t BcdToByte(uint8_t Value) {
  const uint8_t tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}

static RtcIntervalTicks get_rtc_interval_ticks(void) {
  uint32_t time_register = RTC_Handler.Instance->TR;

  const uint8_t hours = BcdToByte((time_register & (RTC_TR_HT | RTC_TR_HU)) >> 16);
  const uint8_t minutes = BcdToByte((time_register & (RTC_TR_MNT | RTC_TR_MNU)) >> 8);
  const uint8_t seconds = BcdToByte(time_register & (RTC_TR_ST | RTC_TR_SU));

  return (((hours * 60) + minutes) * 60) + seconds;
}

static RtcIntervalTicks elapsed_ticks(RtcIntervalTicks before, RtcIntervalTicks after) {
  int32_t result = after - before;
  if (result < 0) {
    result = (TICKS_IN_AN_INTERVAL - before) + after;
  }
  return result;
}

static void restore_rtc_time_state(void) {
  // Recover the previously set time from the RTC backup registers.
  RtcIntervalTicks last_save_time_ticks = HAL_Get_backup(CURRENT_INTERVAL_TICKS_REGISTER);
  time_t last_save_time = HAL_Get_backup(CURRENT_TIME_REGISTER);

  RtcIntervalTicks current_ticks = get_rtc_interval_ticks();
  const int32_t ticks_since_last_save = elapsed_ticks(last_save_time_ticks, current_ticks);
  s_time_base = last_save_time + (ticks_since_last_save / RTC_TICKS_HZ);

  s_time_tick_base = -(((int64_t)current_ticks) % RTC_TICKS_HZ);

#ifdef VERBOSE_LOGGING
  char buffer[TIME_STRING_BUFFER_SIZE];
  PBL_LOG_VERBOSE("Restore RTC: saved: %"PRIu32" diff: %"PRIu32, last_save_time_ticks, ticks_since_last_save);
  PBL_LOG_VERBOSE("Restore RTC: saved_time: %s raw: %lu", time_t_to_string(buffer, last_save_time), last_save_time);
  PBL_LOG_VERBOSE("Restore RTC: current time: %s", time_t_to_string(buffer, s_time_base));
  PBL_LOG_VERBOSE("Restore RTC: s_time_tick_base: %"PRId64, s_time_tick_base);
#endif
}

static time_t ticks_to_time(RtcTicks ticks) {
  return s_time_base + ((ticks - s_time_tick_base) / RTC_TICKS_HZ);
}

static RtcIntervalTicks get_last_save_time_ticks(void) {
  return HAL_Get_backup(CURRENT_INTERVAL_TICKS_REGISTER);
}

static void save_rtc_time_state_exact(RtcIntervalTicks current_rtc_ticks, time_t time) {
  HAL_Set_backup(CURRENT_TIME_REGISTER, time);
  HAL_Set_backup(CURRENT_INTERVAL_TICKS_REGISTER, current_rtc_ticks);

  // Dbgserial instead of PBL_LOG to avoid infinite recursion due to PBL_LOG wanting to know
  // the current ticks.
  //char buffer[128];
  //dbgserial_putstr_fmt(buffer, 128, "Saving RTC state: ticks: %"PRIu32" time: %s raw: %lu", current_rtc_ticks, time_t_to_string(time), time);
  //itoa(time, buffer, sizeof(buffer));
  //dbgserial_putstr(buffer);
  //dbgserial_putstr("Done");
}

static void save_rtc_time_state(RtcIntervalTicks current_rtc_ticks) {
  // Floor it to the latest second
  const RtcIntervalTicks current_rtc_ticks_at_second = (current_rtc_ticks / RTC_TICKS_HZ) * RTC_TICKS_HZ;

  save_rtc_time_state_exact(current_rtc_ticks_at_second, ticks_to_time(s_coarse_ticks + current_rtc_ticks));
}

static void initialize_fast_mode_state(void) {
  RtcIntervalTicks before_ticks = get_rtc_interval_ticks();

  // Set the RTC to value 0 so we start from scratch nicely
  RTC_TimeTypeDef rtc_time = {
    .Seconds = 0,
    .Minutes = 0,
    .Hours = 0
  };
  // RTC_TimeStructInit(&rtc_time);
  HAL_RTC_SetTime(&RTC_Handler, &rtc_time, RTC_FORMAT_BIN);

  // Reset the last ticks counter so we don't rollover prematurely.
  // This value will be set to non-zero if anyone asked for the tick count
  // before this point.
  s_last_ticks = 0;

  // Refresh the saved time so it's more current.
  save_rtc_time_state_exact(TICKS_IN_AN_INTERVAL - (RTC_TICKS_HZ - (before_ticks % RTC_TICKS_HZ)), ticks_to_time(s_coarse_ticks));
  //save_rtc_time_state(0);
}

void rtc_init(void) {
  periph_config_acquire_lock();
  rtc_enable_backup_regs();
#ifdef LXT_DISABLE
#error "RTC LXT is disabled, but RTC init requires it to be enabled"
#endif
#ifdef SF32LB52X
  HAL_PMU_EnableXTAL32();
  if (HAL_PMU_LXTReady() != HAL_OK)
#else
  if (HAL_RTC_LXT_ENABLED() && HAL_PMU_LXTReady() != HAL_OK)
#endif
  {
    PBL_LOG(LOG_LEVEL_ALWAYS, "RTC init fail");
    WTF;
  }
  HAL_RTC_ENABLE_LXT();
  
  RTC_Handler.Init.DivAInt = 0x80;
  RTC_Handler.Init.DivAFrac = 0x0;
  RTC_Handler.Init.DivB = 0x100;
  uint32_t wakesrc = RTC_INIT_NORMAL;
  if (HAL_RTC_Init(&RTC_Handler, wakesrc) != HAL_OK)
  {
    WTF;
  }

  periph_config_release_lock();

  restore_rtc_time_state();
  initialize_fast_mode_state();

#ifdef PBL_LOG_ENABLED
  char buffer[TIME_STRING_BUFFER_SIZE];
  PBL_LOG(LOG_LEVEL_DEBUG, "Current time is <%s>", rtc_get_time_string(buffer));
#endif
}

void rtc_init_timers(void) {
  static RegularTimerInfo rtc_sync_timer = { .list_node = { 0, 0 }, .cb = rtc_resync_timer_callback};
  regular_timer_add_minutes_callback(&rtc_sync_timer);
}

//! How frequently we save the time state to the backup registers in ticks.
#define SAVE_TIME_FREQUENCY (30 * RTC_TICKS_HZ)

static void check_and_handle_rollover(RtcIntervalTicks rtc_ticks) {
  bool save_needed = false;

  const RtcIntervalTicks last_ticks = s_last_ticks;
  s_last_ticks = rtc_ticks;

  if (rtc_ticks < last_ticks) {
    // We've wrapped. Add on the number of seconds in a day to the base number.
    s_coarse_ticks += TICKS_IN_AN_INTERVAL;

    save_needed = true;
  } else if (elapsed_ticks(get_last_save_time_ticks(), rtc_ticks) > SAVE_TIME_FREQUENCY) {
    // If we didn't do this, we would have an edge case where if the watch reset
    // immediately before rollover and then rolled over before we booted again,
    // we wouldn't be able to detect the rollover and we'd think the saved state
    // is very fresh, when really it's over an interval old. By saving multiple
    // times an interval this is still possible to happen, but it's much less likely.
    // We would need to be shutdown for (SECONDS_IN_A_DAY - SAVE_TIME_FREQUENCY) ticks
    // for this to happen.
    save_needed = true;
  }


  if (save_needed) {
    save_rtc_time_state(rtc_ticks);
  }
}

static RtcTicks get_ticks(void) {

  // Prevent this from being interrupted
  bool ints_enabled = mcu_state_are_interrupts_enabled();
  if (ints_enabled) {
    __disable_irq();
  }

  RtcTicks rtc_interval_ticks = get_rtc_interval_ticks();
  check_and_handle_rollover(rtc_interval_ticks);

  if (ints_enabled) {
    __enable_irq();
  }

  return s_coarse_ticks + rtc_interval_ticks;
}

void rtc_set_time(time_t time) {
#ifdef PBL_LOG_ENABLED
  char buffer[TIME_STRING_BUFFER_SIZE];
  PBL_LOG(LOG_LEVEL_INFO, "Setting time to %lu <%s>", time, time_t_to_string(buffer, time));
#endif

  s_time_base = time;
  s_time_tick_base = get_ticks();

  save_rtc_time_state(s_time_tick_base - s_coarse_ticks);
}

time_t rtc_get_time(void) {
  return ticks_to_time(get_ticks());
}

void rtc_get_time_ms(time_t* out_seconds, uint16_t* out_ms) {
  RtcTicks ticks = get_ticks();

  RtcTicks ticks_since_time_base = (ticks - s_time_tick_base);
  *out_seconds = s_time_base + (ticks_since_time_base / RTC_TICKS_HZ);

  RtcTicks ticks_this_second = ticks_since_time_base % RTC_TICKS_HZ;
  *out_ms = (ticks_this_second * 1000) / RTC_TICKS_HZ;
}

RtcTicks rtc_get_ticks(void) {
  return get_ticks();
}


void rtc_alarm_init(void) {
  // RTC_ITConfig(RTC_IT_ALRA, DISABLE);
  // RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

  // RTC_ClearITPendingBit(RTC_IT_ALRA);

  // exti_configure_other(ExtiLineOther_RTCAlarm, ExtiTrigger_Rising);
  // exti_enable_other(ExtiLineOther_RTCAlarm);

  // s_tick_alarm_initialized = true;
}

void rtc_alarm_set(RtcTicks num_ticks) {
  // PBL_ASSERTN(s_tick_alarm_initialized);

  // RTC_ITConfig(RTC_IT_ALRA, DISABLE);
  // RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

  // RTC_AlarmTypeDef alarm_config;
  // RTC_AlarmStructInit(&alarm_config);
  // alarm_config.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;

  // s_alarm_set_time = rtc_get_ticks();

  // RtcTicks alarm_expiry_time = s_alarm_set_time + num_ticks;

  // uint32_t days, hours, minutes, seconds;
  // time_util_split_seconds_into_parts(alarm_expiry_time, &days, &hours, &minutes, &seconds);

  // (void) days; // Don't care about days.
  // alarm_config.RTC_AlarmTime.RTC_Hours = hours;
  // alarm_config.RTC_AlarmTime.RTC_Minutes = minutes;
  // alarm_config.RTC_AlarmTime.RTC_Seconds = seconds;

  // RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm_config);

  // RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  // RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

  // RTC_ClearFlag(RTC_FLAG_ALRAF);
  // EXTI_ClearITPendingBit(EXTI_Line17);
  // RTC_ClearITPendingBit(RTC_IT_ALRA);
}

RtcTicks rtc_alarm_get_elapsed_ticks(void) {
  return rtc_get_ticks() - s_alarm_set_time;
}

bool rtc_alarm_is_initialized(void) {
  return s_tick_alarm_initialized;
}


//! Handler for the RTC alarm interrupt. We don't actually have to do anything in this handler,
//! just the interrupt firing is enough to bring us out of stop mode.
// void RTC_Alarm_IRQHandler(void) {
//   if (RTC_GetITStatus(RTC_IT_ALRA) != RESET) {
//     RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

//     RTC_ClearITPendingBit(RTC_IT_ALRA);
//     EXTI_ClearITPendingBit(EXTI_Line17);
//   }
// }

bool rtc_sanitize_struct_tm(struct tm *t) {
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

bool rtc_sanitize_time_t(time_t *t) {
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

//! We attempt to save registers by placing both the timezone abbreviation
//! timezone index and the daylight_savingtime into the same register set
void rtc_set_timezone(TimezoneInfo *tzinfo) {
}


void rtc_get_timezone(TimezoneInfo *tzinfo) {
}

void rtc_timezone_clear(void) {
}

uint16_t rtc_get_timezone_id(void) {
  return 0;
}

bool rtc_is_timezone_set(void) {
  return 0;
}

void rtc_enable_backup_regs(void) {
}

void rtc_calibrate_frequency(uint32_t frequency) {
}
