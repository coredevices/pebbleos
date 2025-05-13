/*
 * Copyright 2024 Google LLC
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

#include "drivers/task_watchdog.h"

#include "drivers/periph_config.h"
#include "drivers/watchdog.h"
#include "kernel/core_dump.h"
#include "kernel/event_loop.h"
#include "kernel/pebble_tasks.h"
#include "os/mutex.h"
#include "process_management/app_manager.h"
#include "services/common/analytics/analytics.h"
#include "services/common/new_timer/new_timer.h"
#include "services/common/system_task.h"
#include "system/bootbits.h"
#include "system/die.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/size.h"

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF5_COMPATIBLE
#define SF32LB52_COMPATIBLE
#include <mcu.h>

#include "FreeRTOS.h"
#include "task.h"

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#ifdef NO_WATCHDOG
#include "debug/setup.h"
#endif

#if MICRO_FAMILY_NRF5
#include <hal/nrf_rtc.h>
#endif

#define APP_THROTTLE_TIME_MS 300

// These bits get set by calls to task_watchdog_bit_set and checked and cleared periodically by our watchdog feed
static PebbleTaskBitset s_watchdog_bits = 0;

#define DEFAULT_TASK_WATCHDOG_MASK ( 1 << PebbleTask_NewTimers )
static PebbleTaskBitset s_watchdog_mask = DEFAULT_TASK_WATCHDOG_MASK;

_Static_assert(sizeof(s_watchdog_bits) == sizeof(s_watchdog_mask),
               "The task watchdog bitset has a different size than the "
               "task watchdog mask");

// The App Throttle Timer
static TimerID s_throttle_timer_id = TIMER_INVALID_ID;

// How often we want the interrupt to fire
#define TIMER_INTERRUPT_HZ  2
// The frequency to run the peripheral at
#if MICRO_FAMILY_NRF5
#define TIMER_CLOCK_HZ 32768
#else
#define TIMER_CLOCK_HZ 32000
#endif
// The number of timer ticks that should elapse before the timer interrupt fires
#define TIME_PERIOD  (TIMER_CLOCK_HZ / TIMER_INTERRUPT_HZ)

// How many ticks have elapsed since we fed the HW watchdog
static uint8_t s_ticks_since_successful_feed = 0;

// We use this interrupt vector for our lower priority interrupts
#if MICRO_FAMILY_NRF5
#define WATCHDOG_FREERTOS_IRQn        QDEC_IRQn
#define WATCHDOG_FREERTOS_IRQHandler  QDEC_IRQHandler
#else
#define WATCHDOG_FREERTOS_IRQn        CAN2_SCE_IRQn
#define WATCHDOG_FREERTOS_IRQHandler  CAN2_SCE_IRQHandler
#endif

static void prv_task_watchdog_feed(void);

static void prv_log_stuck_timer_task(RebootReason *reboot_reason) {
  void* current_cb = new_timer_debug_get_current_callback();

  if (!current_cb) {
    PBL_LOG_SYNC(LOG_LEVEL_WARNING, "No timer in progress.");
    return;
  }

  PBL_LOG_SYNC(LOG_LEVEL_WARNING, "Timer callback %p", current_cb);
  reboot_reason->watchdog.stuck_task_callback = (uint32_t)current_cb;
}

static void prv_log_stuck_system_task(RebootReason *reboot_reason) {
  void *current_cb = system_task_get_current_callback();

  if (!current_cb) {
    PBL_LOG_SYNC(LOG_LEVEL_WARNING, "No system task callback in progress.");
    return;
  }

  PBL_LOG_SYNC(LOG_LEVEL_WARNING, "System task callback: %p", current_cb);
  reboot_reason->watchdog.stuck_task_callback = (uint32_t)current_cb;
}

static void prv_log_stuck_task(RebootReason *reboot_reason, PebbleTask task) {
  TaskHandle_t *task_handle = pebble_task_get_handle_for_task(task);
  void *current_lr = (void*) ulTaskDebugGetStackedLR(task_handle);
  void *current_pc = (void*) ulTaskDebugGetStackedPC(task_handle);

  PBL_LOG_SYNC(LOG_LEVEL_WARNING, "Task <%s> stuck: LR: %p PC: %p", pebble_task_get_name(task), current_lr, current_pc);
  reboot_reason->watchdog.stuck_task_pc = (uint32_t)current_pc;
  reboot_reason->watchdog.stuck_task_lr = (uint32_t)current_lr;
}

static void prv_log_failed_message(RebootReason *reboot_reason) {
  PBL_LOG_SYNC(LOG_LEVEL_WARNING,
      "Watchdog feed failed, last feed %dms ago, current status 0x%"PRIx16" mask 0x%"PRIx16,
      (s_ticks_since_successful_feed * 1000) / TIMER_INTERRUPT_HZ,
      s_watchdog_bits, s_watchdog_mask);

  // Log about the tasks in reverse priority order. If we have multiple tasks stuck, this might just be because the
  // highest priority of the stuck tasks is preventing the other tasks from getting scheduled. This way, the most
  // suspicious task will get logged about last and will have it's values stored in the RTC backup registers.
  // We'll have to remember to update this list whenever we add additional tasks to the mask. For now this is all
  // the ones that the task_watchdog service watches over.
  const PebbleTask tasks_in_reverse_priority[] = {
    PebbleTask_KernelBackground,
    PebbleTask_KernelMain,
    PebbleTask_PULSE,
    PebbleTask_NewTimers
  };

  for (unsigned int i = 0; i < ARRAY_LENGTH(tasks_in_reverse_priority); ++i) {
    const uint8_t task_index = tasks_in_reverse_priority[i];
    const PebbleTaskBitset task_mask = (1 << task_index);
    if ((s_watchdog_mask & task_mask) && !(s_watchdog_bits & task_mask)) {
      prv_log_stuck_task(reboot_reason, task_index);

      if (task_index == PebbleTask_NewTimers) {
        prv_log_stuck_timer_task(reboot_reason);
      } else if (task_index == PebbleTask_KernelBackground) {
        prv_log_stuck_system_task(reboot_reason);
      }
    }
  }
}

// -------------------------------------------------------------------------------------------------
// The Timer ISR. This runs at super high priority (higher than configMAX_SYSCALL_INTERRUPT_PRIORITY), so
// it is not safe to call ANY FreeRTOS functions from here.
#if MICRO_FAMILY_NRF5
void RTC2_IRQHandler(void) {
  nrf_rtc_event_clear(NRF_RTC2, NRF_RTC_EVENT_COMPARE_0);
  nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_CLEAR);
  nrf_rtc_int_enable(NRF_RTC2, NRF_RTC_INT_COMPARE0_MASK);
  nrf_rtc_event_enable(NRF_RTC2, NRF_RTC_EVENT_COMPARE_0);

  s_ticks_since_successful_feed++;
  prv_task_watchdog_feed();
}
#elif MICRO_FAMILY_SF32LB52
// TODO(SF32LB52): Add implementation
#else
void TIM2_IRQHandler(void) {
  // Workaround M3 bug that causes interrupt to fire twice:
  // https://my.st.com/public/Faq/Lists/faqlst/DispForm.aspx?ID=143
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  s_ticks_since_successful_feed++;
  prv_task_watchdog_feed();
}
#endif

static void prv_app_task_throttle_end(void *data) {
  vTaskPrioritySet(pebble_task_get_handle_for_task(PebbleTask_App),
      APP_TASK_PRIORITY | portPRIVILEGE_BIT);
  PBL_LOG(LOG_LEVEL_DEBUG, "Ending App Throttling");
}

static void prv_app_task_throttle_start(void) {
  static char last_throttled_task[configMAX_TASK_NAME_LEN];
  const char *curr_task = pebble_task_get_name(PebbleTask_App);

  // if an app results in system throttling, log it at the INFO level at least
  // once to aid in debug
  if (strcmp(last_throttled_task, curr_task) != 0) {
    strcpy(last_throttled_task, curr_task);
    PBL_LOG(LOG_LEVEL_INFO, "Starting App Throttling for %s", curr_task);
  } else {
    PBL_LOG(LOG_LEVEL_DEBUG, "Starting App Throttling for %s", curr_task);
  }

  analytics_inc(ANALYTICS_DEVICE_METRIC_APP_THROTTLED_COUNT, AnalyticsClient_System);
  vTaskPrioritySet(pebble_task_get_handle_for_task(PebbleTask_App),
      tskIDLE_PRIORITY | portPRIVILEGE_BIT);
}

static void prv_system_task_starved_callback(void *data) {
  if (system_task_is_ready_to_run() || (system_task_get_current_callback() != NULL)) {
    // check if system task is ready to go or is already running a callback.
    // If it's ready to run, we definitely want to throttle the app task.
    // Or, if it's blocked in a callback, there's a chance it could be waiting for a mutex held by
    // the background worker and the worker won't be able to release it until we throttle the app
    // to give the worker some time.
    prv_app_task_throttle_start();
    // throttle the app task for APP_THROTTLE_TIME_MS to give the system task some runtime
    new_timer_start(s_throttle_timer_id, APP_THROTTLE_TIME_MS, prv_app_task_throttle_end, NULL, 0);
  }
}

// -------------------------------------------------------------------------------------------------
// This is a lower priority interrupt (at configMAX_SYSCALL_INTERRUPT_PRIORITY) that we trigger
// when we need to perform logging.
void WATCHDOG_FREERTOS_IRQHandler(void) {
  PBL_LOG(LOG_LEVEL_DEBUG, "WD: low priority ISR");

  // Are we rebooting because of watch dog?
  RebootReason reason;
  reboot_reason_get(&reason);
  if (reason.code == RebootReasonCode_Watchdog) {
    // Check if system task is the one triggering the watchdog
    PebbleTaskBitset new_mask =
        s_watchdog_mask & ~(1 << PebbleTask_KernelBackground);
    if ((new_mask & s_watchdog_bits) == new_mask) {
      // Put system task callback using from ISR variant
      PebbleEvent event = {
        .type = PEBBLE_CALLBACK_EVENT,
        .callback = {
          .callback = prv_system_task_starved_callback,
          .data = NULL,
        },
      };
      event_put_isr(&event);
    }
    prv_log_failed_message(&reason);

    // Re-write the reason including the stuck task info collected by prv_log_failed_message()
    reboot_reason_clear();
    reboot_reason_set(&reason);

    // If getting reset by the watchdog timer is imminent (it will reset the
    // CPU if not fed at least once every 7 seconds), then just coredump now
    if (s_ticks_since_successful_feed >= (6 * TIMER_INTERRUPT_HZ)) {
#if defined(NO_WATCHDOG)
      PBL_LOG(LOG_LEVEL_DEBUG,
              "Would have coredumped if built with watchdogs ... enabling lowpowerdebug!");
      enable_mcu_debugging();
#else
      reset_due_to_software_failure();
#endif
    }


  } else if (reason.code == 0) {
    PBL_LOG_SYNC(LOG_LEVEL_WARNING, "Recovered from task watchdog stall.");
  }
}

// ============================================================================================================
// Public functions

// -------------------------------------------------------------------------------------------------
// Setup a very high priority interrupt to fire periodically. This ISR will call task_watchdog_feed()
// which resets the watchdog timer if it detects that none of our watchable tasks are stuck.
void task_watchdog_init(void) {
#if MICRO_FAMILY_NRF5
  // We use RTC2 as the WDT kicker; RTC1 is used by the OS RTC
  nrf_rtc_prescaler_set(NRF_RTC2, NRF_RTC_FREQ_TO_PRESCALER(TIMER_CLOCK_HZ));

  // trigger compare interrupt at appropriate time
  nrf_rtc_cc_set(NRF_RTC2, 0, TIME_PERIOD);
  nrf_rtc_event_clear(NRF_RTC2, NRF_RTC_EVENT_COMPARE_0);
  nrf_rtc_int_enable(NRF_RTC2, NRF_RTC_INT_COMPARE0_MASK);
  nrf_rtc_event_enable(NRF_RTC2, NRF_RTC_EVENT_COMPARE_0);

  NVIC_SetPriority(RTC2_IRQn, TASK_WATCHDOG_PRIORITY);
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);

  nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);
#elif MICRO_FAMILY_SF32LB52
// TODO(SF32LB52): Add implementation
#else
  // The timer is on ABP1 which is clocked by PCLK1
  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq(&clocks);
  uint32_t timer_clock = clocks.PCLK1_Frequency; // Hz

  uint32_t prescale = RCC->CFGR & RCC_CFGR_PPRE1;
  if (prescale != RCC_CFGR_PPRE1_DIV1) {
    // per the stm32 'clock tree' diagram, if the prescaler for APBx is not 1, then
    // the timer clock is at double the APBx frequency
    timer_clock *= 2;
  }

  // Enable the timer clock
  periph_config_enable(TIM2, RCC_APB1Periph_TIM2);

  // Setup timer 6 to generate very high priority interrupts
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TASK_WATCHDOG_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Setup timer 2 for periodic interrupts at TIMER_INTERRUPT_HZ
  TIM_TimeBaseInitTypeDef  tim_config;
  TIM_TimeBaseStructInit(&tim_config);

  // Clock frequency to run the timer at
  uint32_t prescaler = timer_clock / TIMER_CLOCK_HZ;

  // period & prescaler values are 16 bits, check for configuration errors
  PBL_ASSERTN(TIME_PERIOD <= UINT16_MAX && prescaler <= UINT16_MAX);

  tim_config.TIM_Period = TIME_PERIOD;
  tim_config.TIM_Prescaler = prescaler;
  tim_config.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &tim_config);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
#endif

  // Setup another unused interrupt vector to handle our low priority interrupts. When we need to do higher
  // level functions (like PBL_LOG), we trigger this lower-priority interrupt to fire. Since it runs at
  // configMAX_SYSCALL_INTERRUPT_PRIORITY or lower, it can at least call FreeRTOS ISR functions.
#if MICRO_FAMILY_NRF5
  NVIC_SetPriority(WATCHDOG_FREERTOS_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(WATCHDOG_FREERTOS_IRQn);
#elif MICRO_FAMILY_SF32LB52
  // TODO(SF32LB52): Add implementation
#else
  NVIC_InitStructure.NVIC_IRQChannel = WATCHDOG_FREERTOS_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_EnableIRQ(WATCHDOG_FREERTOS_IRQn);
#endif

  // create the app throttling timer
  s_throttle_timer_id = new_timer_create();
}

static void task_watchdog_disable_interrupt() {
#if MICRO_FAMILY_NRF5
  NVIC_DisableIRQ(RTC2_IRQn);
#elif MICRO_FAMILY_SF32LB52
// TODO(SF32LB52): Add implementation
#else
  NVIC_DisableIRQ(TIM2_IRQn);
#endif
  taskENTER_CRITICAL();
}

static void task_watchdog_enable_interrupt() {
  taskEXIT_CRITICAL();
#if MICRO_FAMILY_NRF5
  NVIC_EnableIRQ(RTC2_IRQn);
#elif MICRO_FAMILY_SF32LB52
// TODO(SF32LB52): Add implementation
#else
  NVIC_EnableIRQ(TIM2_IRQn);
#endif
}

void task_watchdog_bit_set_all(void) {
  task_watchdog_disable_interrupt();
  s_watchdog_bits |= s_watchdog_mask;
  task_watchdog_enable_interrupt();
}

void task_watchdog_bit_set(PebbleTask task) {
  task_watchdog_disable_interrupt();
  s_watchdog_bits |= (1 << task);
  task_watchdog_enable_interrupt();
}

bool task_watchdog_mask_get(PebbleTask task) {
  task_watchdog_disable_interrupt();
  bool result = (s_watchdog_mask & (1 << task));
  task_watchdog_enable_interrupt();
  return result;
}

void task_watchdog_mask_set(PebbleTask task) {
  task_watchdog_disable_interrupt();
  s_watchdog_mask |= (1 << task);
  task_watchdog_enable_interrupt();
}

void task_watchdog_mask_clear(PebbleTask task) {
  task_watchdog_disable_interrupt();
  s_watchdog_mask &= ~(1 << task);
  task_watchdog_enable_interrupt();
}

void task_watchdog_step_elapsed_time_ms(uint32_t elapsed_ms) {
  // nRF5 has the RTC running during sleep, and needs no help here
#if !MICRO_FAMILY_NRF5 && !MICRO_FAMILY_SF32LB52
  uint32_t timer_ticks = (elapsed_ms * TIMER_CLOCK_HZ) / 1000;
  timer_ticks += TIM2->CNT;

  uint8_t timer_ticks_elapsed = timer_ticks / TIME_PERIOD;
  if (timer_ticks_elapsed > 0) {
    // we don't want the interrupt to fire while we are editing the feed count
    TIM_Cmd(TIM2, DISABLE);
    s_ticks_since_successful_feed += timer_ticks_elapsed;
    TIM_Cmd(TIM2, ENABLE);
  }

  TIM2->CNT = timer_ticks % TIME_PERIOD;
#endif
  prv_task_watchdog_feed();
}

#define WATCHDOG_WARN_TICK_CNT      (5 * TIMER_INTERRUPT_HZ)         /* 5s */
#define WATCHDOG_COREDUMP_TICK_CNT  ((65 * TIMER_INTERRUPT_HZ) / 10) /* 6.5 s */

//! Test to see if all the bits are set. If so, feed the hardware watchdog.
//! Note: Should only ever be called upon exit from stop mode and from our
//! high priority software watchdog timer. To actually prevent a particular
//! task from triggering a watchdog you can call task_watchdog_bit_set to feed it
static void prv_task_watchdog_feed(void) {
  // NOTE! This function runs from a timer interrupt setup by the watchdog_feed_timer driver that is at a priority
  // higher than configMAX_SYSCALL_INTERRUPT_PRIORITY. This means you can't call ANY FreeRTOS functions.
  // Careful what you put here.

  // We do want to log watchdog actions, since it's really important for debugging watchdog stalls either on
  // bigboards through serial or using flash logging. To accomplish this trigger a lower priority interrupt to fire,
  // which is at or below configMAX_SYSCALL_INTERRUPT_PRIORITY and make our logging calls from there.

  static int s_last_warning_message_tick_time = 0; //!< Used to rate limit the warning message
  if ((s_watchdog_bits & s_watchdog_mask) == s_watchdog_mask) {
    // All tasks have checked in, feed the actual watchdog and clear any state.

    s_watchdog_bits = 0;
    watchdog_feed();
    s_ticks_since_successful_feed = 0;

    if (s_last_warning_message_tick_time) {
      // We logged a warning message, clear this state as we apparently recoved.

      reboot_reason_clear();
      // Trigger our lower priority interrupt to fire. If it fires when reboot reason is not RebootReasonCode_Watchdog,
      //  it simply logs a message that the we recovered from a watchdog stall
#if !MICRO_FAMILY_SF32LB52
      NVIC_SetPendingIRQ(WATCHDOG_FREERTOS_IRQn);
#endif

      s_last_warning_message_tick_time = 0;
    }

#if defined(TARGET_QEMU)
    // Investigating PBL-29422
    extern volatile int g_qemu_num_skipped_ticks;
    g_qemu_num_skipped_ticks = 0;
#endif // defined(TARGET_QEMU)
  }

  // If we haven't fed the watchdog in the last 5 seconds and we haven't
  // spammed the log in the last 1/2 second, set the reboot reason - we are
  // about to go down...

  if (s_ticks_since_successful_feed >= WATCHDOG_WARN_TICK_CNT &&
      ((s_ticks_since_successful_feed - s_last_warning_message_tick_time) > 0)) {

    // FIXME PBL-39328: Truncate s_watchdog_bits and s_watchdog mask
    // to eight bits each.
    RebootReason reboot_reason = {
      .code = RebootReasonCode_Watchdog,
      .data8 = { (uint8_t)s_watchdog_bits, (uint8_t)s_watchdog_mask }
    };
    reboot_reason_set(&reboot_reason);

    // Trigger our lower priority interrupt to fire. When it sees
    //  RebootReasonCode_Watchdog in the reboot reason, it logs information
    //  about the stuck task
#if !MICRO_FAMILY_SF32LB52
    NVIC_SetPendingIRQ(WATCHDOG_FREERTOS_IRQn);
#endif
    // If the low priority interrupt hasn't reset us by the time 6.5 seconds
    // rolls around (it will issue the reset if executed at least 6 seconds
    // after s_last_successful_feed_time), it likely means that we are stuck in
    // an ISR or low priority interrupts are disabled, so coredump now
    if (s_ticks_since_successful_feed >= WATCHDOG_COREDUMP_TICK_CNT) {
#if defined(NO_WATCHDOG)
      dbgserial_putstr("Would have coredumped if built with watchdogs ... enabling lowpowerdebug!");
      enable_mcu_debugging();
#else
      reset_due_to_software_failure();
#endif
    }
    s_last_warning_message_tick_time = s_ticks_since_successful_feed;
  }
}
