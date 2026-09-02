/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "pbl/kernel/types.h"
#include <pbl/drivers/rtc.h>

#include <pbl/drivers/exti.h>
#include <pbl/drivers/task_watchdog.h>

#include "pbl/mcu/interrupts.h"

#include "pbl/services/regular_timer.h"

#include <pbl/logging/logging.h>
#include "system/passert.h"

#include "util/time/time.h"

#include <hal/nrf_rtc.h>

#include <inttypes.h>

PBL_LOG_MODULE_DEFINE(driver_rtc_nrf5, CONFIG_DRIVER_RTC_LOG_LEVEL);

//! The type of a raw reading from the RTC (masked to 0xFFFFFF).
typedef uint32_t RtcIntervalTicks;

//! How frequently we save the time state to the backup registers in ticks.
#define SAVE_TIME_FREQUENCY (30 * RTC_TICKS_HZ)
static RtcIntervalTicks prv_get_last_save_time_ticks(void);
static void prv_save_rtc_time_state(RtcIntervalTicks current_rtc_ticks);

#define TICKS_IN_AN_INTERVAL (RTC_COUNTER_COUNTER_Msk + 1)

static RtcIntervalTicks prv_elapsed_ticks(RtcIntervalTicks before, RtcIntervalTicks after) {
  int32_t result = after - before;
  if (result < 0) {
    result = (TICKS_IN_AN_INTERVAL - before) + after;
  }
  return result;
}

static RtcIntervalTicks prv_get_rtc_interval_ticks(void) {
  return nrf_rtc_counter_get(BOARD_RTC_INST);
}

/***
 * Logic associated with keeping raw coarse / fine RTC ticks -- the
 * monotonic RtcTicks counter.
 */

//! The value of the RTC registers last time we checked them.
static uint32_t s_last_ticks = 0;
//! This value is added to the current value of the RTC ticks to get the number
//! of ticks since system start. Incremented whenever we detect a rollover.
static RtcTicks s_coarse_ticks = 1;

static bool s_did_init_rtc = false;

//! did we boot with a full reset that brought RTC ticks to 0?
static bool s_had_amnesia_on_boot = false;

static void prv_check_and_handle_rollover(RtcIntervalTicks rtc_ticks) {
  bool save_needed = false;

  const RtcIntervalTicks last_ticks = s_last_ticks;
  s_last_ticks = rtc_ticks;

  if (rtc_ticks < last_ticks) {
    // We've wrapped.  Add on the RTC wrap length to the base number.  On
    // nRF5, this is 0xFFFFFF; that's only 4.5 hours (at 1.024 kHz),
    // compared to STM32's available SECONDS_IN_A_DAY.  Sucks for us; oh
    // well.
    
    s_coarse_ticks += TICKS_IN_AN_INTERVAL;

    save_needed = true;
  } else if (prv_elapsed_ticks(prv_get_last_save_time_ticks(), rtc_ticks) > SAVE_TIME_FREQUENCY) {
    // If we didn't do this, we would have an edge case where if the watch reset
    // immediately before rollover and then rolled over before we booted again,
    // we wouldn't be able to detect the rollover and we'd think the saved state
    // is very fresh, when really it's over an interval old. By saving multiple
    // times an interval this is still possible to happen, but it's much less likely.
    // We would need to be shutdown for (RTC_COUNTER_COUNTER_Msk - SAVE_TIME_FREQUENCY) ticks
    // for this to happen.
    save_needed = true;
  }

  if (save_needed) {
    prv_save_rtc_time_state(rtc_ticks);
  }
}

RtcTicks rtc_get_ticks(void) {
  // Prevent this from being interrupted
  bool ints_enabled = mcu_state_are_interrupts_enabled();
  if (ints_enabled) {
    __disable_irq();
  }

  RtcTicks rtc_interval_ticks = prv_get_rtc_interval_ticks();
  prv_check_and_handle_rollover(rtc_interval_ticks);

  if (ints_enabled) {
    __enable_irq();
  }

  return s_coarse_ticks + rtc_interval_ticks;
}

/***
 * Logic associated with converting extended RTC ticks to wall clock time.
 */

//! This variable is a UNIX timestamp of what the current wall clock time was at tick s_time_tick_base.
static time_t s_time_base = 0;
//! This variable is the tick where the wall clock time was equal to s_time_base. If you subtract this variable
//! from the current tick count, you'll get the number of ticks that have elapsed since s_time_base, which will
//! allow you to calculate the current wall clock time. Note that this value may be negative on startup, see
//! prv_restore_rtc_time_state
static int64_t s_time_tick_base = 0;

static time_t prv_ticks_to_time(RtcTicks ticks) {
  return s_time_base + ((ticks - s_time_tick_base) / RTC_TICKS_HZ);
}

void rtc_set_time(time_t time) {
  s_time_base = time;
  s_time_tick_base = rtc_get_ticks();

  prv_save_rtc_time_state(s_time_tick_base - s_coarse_ticks);
}

time_t rtc_get_time(void) {
  return prv_ticks_to_time(rtc_get_ticks());
}

void rtc_get_time_ms(time_t* out_seconds, uint16_t* out_ms) {
  RtcTicks ticks = rtc_get_ticks();

  RtcTicks ticks_since_time_base = (ticks - s_time_tick_base);
  *out_seconds = s_time_base + (ticks_since_time_base / RTC_TICKS_HZ);

  RtcTicks ticks_this_second = ticks_since_time_base % RTC_TICKS_HZ;
  *out_ms = (ticks_this_second * 1000) / RTC_TICKS_HZ;
}

/***
 * Logic associated with saving the RTC-tick-to-wallclock conversion factor
 * to retained-RAM.
 */

static void prv_restore_rtc_time_state(void) {
  // Recover the previously set time from the RTC backup registers.
  RtcIntervalTicks last_save_time_ticks = retained_read(CURRENT_INTERVAL_TICKS_REGISTER);
  time_t last_save_time = retained_read(CURRENT_TIME_REGISTER);

  if (s_had_amnesia_on_boot) {
    /* We have no idea what time it might be.  The closest we got is the
     * last time we saved.  */
    s_time_base = last_save_time;
    s_time_tick_base = 0;
  } else {
    RtcIntervalTicks current_ticks = prv_get_rtc_interval_ticks();
    const int32_t ticks_since_last_save = prv_elapsed_ticks(last_save_time_ticks * RTC_TICKS_HZ, current_ticks);
    s_time_base = last_save_time + (ticks_since_last_save / RTC_TICKS_HZ);
    s_time_tick_base = -(((int64_t)current_ticks) % RTC_TICKS_HZ);
  }
}

static RtcIntervalTicks prv_get_last_save_time_ticks(void) {
  return retained_read(CURRENT_INTERVAL_TICKS_REGISTER);
}

static void prv_save_rtc_time_state_exact(RtcIntervalTicks current_rtc_ticks, time_t time) {
  retained_write(CURRENT_TIME_REGISTER, time);
  retained_write(CURRENT_INTERVAL_TICKS_REGISTER, current_rtc_ticks);
}

static void prv_save_rtc_time_state(RtcIntervalTicks current_rtc_ticks) {
  if (!s_did_init_rtc) {
    return;
  }

  // Floor it to the latest second
  const RtcIntervalTicks current_rtc_ticks_at_second = (current_rtc_ticks / RTC_TICKS_HZ) * RTC_TICKS_HZ;

  prv_save_rtc_time_state_exact(current_rtc_ticks_at_second, prv_ticks_to_time(s_coarse_ticks + current_rtc_ticks));
}

/*** Logic that ought be refactored into rtc_common, were it not stm32-only. ***/

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

const char *rtc_get_time_string(char *buffer) {
  return time_t_to_string(buffer, rtc_get_time());
}

const char *time_t_to_string(char *buffer, time_t t) {
  struct tm time;
  localtime_r(&t, &time);

  strftime(buffer, TIME_STRING_BUFFER_SIZE, "%c", &time);

  return buffer;
}

//! We attempt to save registers by placing both the timezone abbreviation
//! timezone index and the daylight_savingtime into the same register set
void rtc_set_timezone(TimezoneInfo *tzinfo) {
  uint32_t *raw = (uint32_t*)tzinfo;
  _Static_assert(sizeof(TimezoneInfo) <= 5 * sizeof(uint32_t),
      "RTC Set Timezone invalid data size");

  retained_write(RTC_TIMEZONE_ABBR_START, raw[0]);
  retained_write(RTC_TIMEZONE_ABBR_END_TZID_DSTID, raw[1]);
  retained_write(RTC_TIMEZONE_GMTOFFSET, raw[2]);
  retained_write(RTC_TIMEZONE_DST_START, raw[3]);
  retained_write(RTC_TIMEZONE_DST_END, raw[4]);
}

void rtc_get_timezone(TimezoneInfo *tzinfo) {
  uint32_t *raw = (uint32_t*)tzinfo;

  raw[0] = retained_read(RTC_TIMEZONE_ABBR_START);
  raw[1] = retained_read(RTC_TIMEZONE_ABBR_END_TZID_DSTID);
  raw[2] = retained_read(RTC_TIMEZONE_GMTOFFSET);
  raw[3] = retained_read(RTC_TIMEZONE_DST_START);
  raw[4] = retained_read(RTC_TIMEZONE_DST_END);
}

void rtc_timezone_clear(void) {
  retained_write(RTC_TIMEZONE_ABBR_START, 0);
  retained_write(RTC_TIMEZONE_ABBR_END_TZID_DSTID, 0);
  retained_write(RTC_TIMEZONE_GMTOFFSET, 0);
  retained_write(RTC_TIMEZONE_DST_START, 0);
  retained_write(RTC_TIMEZONE_DST_END, 0);
}

uint16_t rtc_get_timezone_id(void) {
  return ((retained_read(RTC_TIMEZONE_ABBR_END_TZID_DSTID) >> 16) & 0xFFFF);
}

bool rtc_is_timezone_set(void) {
  // True if the timezone abbreviation has been set (including UNK for unknown)
  return (retained_read(RTC_TIMEZONE_ABBR_START) != 0);
}

void rtc_enable_backup_regs(void) {
  /* we always use retained ram for this, so no problem */
}

void rtc_calibrate_frequency(uint32_t frequency) {
  /* On nRF5, there is no way to calibrate the RTC.  That crystal had better
   * be accurate!
   */
}

void rtc_init(void) {
  /* May have been initialized out of sequence by FreeRTOS startup -- if so,
   * no worries.  */
  if (s_did_init_rtc) {
    return;
  }

#ifndef NRF_RTC_FREQ_TO_PRESCALER
#define NRF_RTC_FREQ_TO_PRESCALER RTC_FREQ_TO_PRESCALER
#endif
  if (prv_get_rtc_interval_ticks() == 0) {
    s_had_amnesia_on_boot = true;
    PBL_LOG_INFO("RTC appears to have been reset :( hope you have your phone connected");
  }

  nrf_rtc_prescaler_set(BOARD_RTC_INST, NRF_RTC_FREQ_TO_PRESCALER(RTC_TICKS_HZ));

  nrf_rtc_event_disable(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_1);
  nrf_rtc_event_clear(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_1);
  nrf_rtc_event_enable(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_1);
  nrf_rtc_int_enable(BOARD_RTC_INST, NRF_RTC_INT_COMPARE1_MASK);
  nrf_rtc_cc_set(BOARD_RTC_INST, 1, (RTC_TICKS_HZ / (1000 / TASK_WATCHDOG_FEED_PERIOD_MS)) - 1);

  nrf_rtc_task_trigger(BOARD_RTC_INST, NRF_RTC_TASK_START);

  prv_restore_rtc_time_state();
  s_did_init_rtc = true;

  NVIC_SetPriority(BOARD_RTC_IRQN, PBL_IRQ_PRIO_KERNEL);
  NVIC_EnableIRQ(BOARD_RTC_IRQN);

#if TEST_RTC_FREQ
  // FIXME: can be removed after FIRM-121 is fixed
  extern void board_early_init();
  board_early_init();
  for (int i = 0; i < 100; i++) {
    uint32_t ctr0 = nrf_rtc_counter_get(BOARD_RTC_INST);
    while (nrf_rtc_counter_get(BOARD_RTC_INST) == ctr0)
      ;
    ctr0 = nrf_rtc_counter_get(BOARD_RTC_INST) + 100;
    uint32_t iters = 0;
    while (nrf_rtc_counter_get(BOARD_RTC_INST) != ctr0)
      iters++;
    PBL_LOG_INFO("RTC: 100 RTC ticks took %"PRIu32" iters", iters);
  }
#endif
}

void rtc_enable_synthetic_systick(void) {
  // Now that the RTC is awake, we can switch from SysTick to RTC interrupt
  // ticks.  We need to do this so that we actually get ticks in wfi, since
  // nRF5 stops SysTick in sleep.
  _Static_assert(RTC_TICKS_HZ == PBL_TICK_HZ);
  if (!s_did_init_rtc) {
    rtc_init();
  }

  nrf_rtc_event_enable(BOARD_RTC_INST, NRF_RTC_EVENT_TICK);
  nrf_rtc_int_enable(BOARD_RTC_INST, NRF_RTC_INT_TICK_MASK);
}

void rtc_systick_pause(void) {
  // We don't want the fine-grained interrupts at 100Hz when we're in stop
  // mode -- we have a timer set for that, after all.
  nrf_rtc_event_disable(BOARD_RTC_INST, NRF_RTC_EVENT_TICK);
  nrf_rtc_int_disable(BOARD_RTC_INST, NRF_RTC_INT_TICK_MASK);
}

void rtc_systick_resume(void) {
  nrf_rtc_event_clear(BOARD_RTC_INST, NRF_RTC_EVENT_TICK);
  nrf_rtc_event_enable(BOARD_RTC_INST, NRF_RTC_EVENT_TICK);
  nrf_rtc_int_enable(BOARD_RTC_INST, NRF_RTC_INT_TICK_MASK);
}

//! Our RTC tick counter can overflow if nobody asks about it.  This
//! repeating callback allows us to make sure this doesn't happen.
static void prv_rtc_resync_timer_callback() {
  rtc_get_ticks();
}

void rtc_init_timers(void) {
  static RegularTimerInfo rtc_sync_timer = { .list_node = { 0, 0 }, .cb = prv_rtc_resync_timer_callback};
  regular_timer_add_minutes_callback(&rtc_sync_timer);
}

/*** Logic to control RTC wakeup timer. ***/

static RtcTicks s_alarm_set_time = 0;
static RtcTicks s_alarm_expiry_time = 0;
static bool s_tick_alarm_initialized = false;

void rtc_alarm_init(void) {
  nrf_rtc_event_disable(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_0);
  nrf_rtc_int_disable(BOARD_RTC_INST, NRF_RTC_INT_COMPARE0_MASK);
  s_tick_alarm_initialized = true;
}

void rtc_alarm_set(RtcTicks num_ticks) {
  PBL_ASSERTN(s_tick_alarm_initialized);
  
  nrf_rtc_event_disable(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_0);
  nrf_rtc_event_clear(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_0);
  
  s_alarm_set_time = rtc_get_ticks();
  s_alarm_expiry_time = s_alarm_set_time + num_ticks - 1;
  
  /* We're bounded by the regular_timer_add_minutes_callback for the
   * rtc_alarm_set, so we're not going to wrap around more than once -- one
   * minute is always less than 4.5 hours.
   */
  nrf_rtc_cc_set(BOARD_RTC_INST, 0, s_alarm_expiry_time & RTC_COUNTER_COUNTER_Msk);
  
  nrf_rtc_event_enable(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_0);
  nrf_rtc_int_enable(BOARD_RTC_INST, NRF_RTC_INT_COMPARE0_MASK);
}

RtcTicks rtc_alarm_get_elapsed_ticks(void) {
  return rtc_get_ticks() - s_alarm_set_time;
}

bool rtc_alarm_is_initialized(void) {
  return s_tick_alarm_initialized;
}

//! Handler for the RTC alarm interrupt.  We don't actually have to do
//! anything in that handler, just the interrupt firing is enough to bring
//! us out of stop mode.  But once we switch into RTC-provided-systick mode,
//! we *do* need to call into systick!
void rtc_irq_handler(void) {
  if (nrf_rtc_event_check(BOARD_RTC_INST, NRF_RTC_EVENT_TICK)) {
    extern void SysTick_Handler();

    nrf_rtc_event_clear(BOARD_RTC_INST, NRF_RTC_EVENT_TICK);
    SysTick_Handler();
  }

  if (nrf_rtc_event_check(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_0)) {
    nrf_rtc_event_disable(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_0);
    nrf_rtc_event_clear(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_0);
    nrf_rtc_int_disable(BOARD_RTC_INST, NRF_RTC_INT_COMPARE0_MASK);
  }

  if (nrf_rtc_event_check(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_1)) {
    task_watchdog_feed();

    nrf_rtc_event_clear(BOARD_RTC_INST, NRF_RTC_EVENT_COMPARE_1);

    uint32_t next_feed_ticks = nrf_rtc_counter_get(BOARD_RTC_INST) +
                               (RTC_TICKS_HZ / (1000 / TASK_WATCHDOG_FEED_PERIOD_MS));
    nrf_rtc_cc_set(BOARD_RTC_INST, 1, next_feed_ticks & RTC_COUNTER_COUNTER_Msk);
  }

  NVIC_ClearPendingIRQ(BOARD_RTC_IRQN);
}
