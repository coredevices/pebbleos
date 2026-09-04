/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/task_watchdog.h>

#include <pbl/drivers/watchdog.h>
#include "kernel/event_loop.h"
#include "kernel/pebble_tasks.h"
#include "process_management/app_manager.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include "system/die.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"

#include "FreeRTOS.h"
#include "task.h"

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#ifdef CONFIG_SOC_NRF52
#include <hal/nrf_rtc.h>
#endif

#ifdef CONFIG_SOC_SF32LB52
#include <bf0_hal.h>
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
#define TIMER_INTERRUPT_HZ  (1000 / TASK_WATCHDOG_FEED_PERIOD_MS)

// How many ticks have elapsed since we fed the HW watchdog
static uint8_t s_ticks_since_successful_feed = 0;

// Pause state: number of ticks remaining in pause
static uint32_t s_pause_ticks_remaining = 0;

// We use this interrupt vector for our lower priority interrupts
#ifdef CONFIG_SOC_NRF52
#define WATCHDOG_FREERTOS_IRQn        QDEC_IRQn
#define WATCHDOG_FREERTOS_IRQHandler  QDEC_IRQHandler
#elif defined(CONFIG_SOC_SF32LB52)
#define WATCHDOG_FREERTOS_IRQn        USART5_IRQn
#define WATCHDOG_FREERTOS_IRQHandler  USART5_IRQHandler
#elif defined(CONFIG_QEMU)
#define WATCHDOG_FREERTOS_IRQn        WATCHDOG_IRQn
#define WATCHDOG_FREERTOS_IRQHandler  WATCHDOG_IRQHandler
#else
#define WATCHDOG_FREERTOS_IRQn        CAN2_SCE_IRQn
#define WATCHDOG_FREERTOS_IRQHandler  CAN2_SCE_IRQHandler
#endif

static void prv_task_watchdog_feed(void);

static void prv_log_stuck_timer_task(RebootReason *reboot_reason) {
  void* current_cb = new_timer_debug_get_current_callback();

  if (!current_cb) {
    PBL_LOG_SYNC_WRN("No timer in progress.");
    return;
  }

  PBL_LOG_SYNC_WRN("Timer callback %p", current_cb);
  reboot_reason->watchdog.stuck_task_callback = (uint32_t)current_cb;
}

static void prv_log_stuck_system_task(RebootReason *reboot_reason) {
  void *current_cb = system_task_get_current_callback();

  if (!current_cb) {
    PBL_LOG_SYNC_WRN("No system task callback in progress.");
    return;
  }

  PBL_LOG_SYNC_WRN("System task callback: %p", current_cb);
  reboot_reason->watchdog.stuck_task_callback = (uint32_t)current_cb;
}

static void prv_log_stuck_task(RebootReason *reboot_reason, PebbleTask task) {
  TaskHandle_t *task_handle = pebble_task_get_handle_for_task(task);
  void *current_lr = (void*) ulTaskDebugGetStackedLR(task_handle);
  void *current_pc = (void*) ulTaskDebugGetStackedPC(task_handle);

  PBL_LOG_SYNC_WRN("Task <%s> stuck: LR: %p PC: %p", pebble_task_get_name(task), current_lr, current_pc);
  reboot_reason->watchdog.stuck_task_pc = (uint32_t)current_pc;
  reboot_reason->watchdog.stuck_task_lr = (uint32_t)current_lr;
}

#ifndef CONFIG_NO_WATCHDOG
// TCB-only variant of prv_log_failed_message; no logging or FreeRTOS calls
// so it's safe from above configMAX_SYSCALL_INTERRUPT_PRIORITY.
static void prv_capture_stuck_task_info(RebootReason *reboot_reason) {
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
      TaskHandle_t *task_handle = pebble_task_get_handle_for_task(task_index);
      reboot_reason->watchdog.stuck_task_pc = (uint32_t)ulTaskDebugGetStackedPC(task_handle);
      reboot_reason->watchdog.stuck_task_lr = (uint32_t)ulTaskDebugGetStackedLR(task_handle);
    }
  }
}
#endif

static void prv_log_failed_message(RebootReason *reboot_reason) {
  PBL_LOG_SYNC_WRN("Watchdog feed failed, last feed %dms ago, current status 0x%"PRIx16" mask 0x%"PRIx16,
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

static void prv_app_task_throttle_end(void *data) {
  vTaskPrioritySet(pebble_task_get_handle_for_task(PebbleTask_App),
      APP_TASK_PRIORITY | portPRIVILEGE_BIT);
  PBL_LOG_DBG("Ending App Throttling");
}

static void prv_app_task_throttle_start(void) {
  static char last_throttled_task[configMAX_TASK_NAME_LEN];
  const char *curr_task = pebble_task_get_name(PebbleTask_App);

  // if an app results in system throttling, log it at the INFO level at least
  // once to aid in debug
  if (strcmp(last_throttled_task, curr_task) != 0) {
    strcpy(last_throttled_task, curr_task);
    PBL_LOG_WRN("Starting App Throttling for %s", curr_task);
  } else {
    PBL_LOG_DBG("Starting App Throttling for %s", curr_task);
  }

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
  PBL_LOG_DBG("WD: low priority ISR");

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
#if !defined(CONFIG_NO_WATCHDOG)
      reset_due_to_software_failure();
#endif
    }


  } else if (reason.code == 0) {
    PBL_LOG_SYNC_WRN("Recovered from task watchdog stall.");
  }
}

// ============================================================================================================
// Public functions

// -------------------------------------------------------------------------------------------------
// Setup a very high priority interrupt to fire periodically. This ISR will call task_watchdog_feed()
// which resets the watchdog timer if it detects that none of our watchable tasks are stuck.
void task_watchdog_init(void) {
  // Setup another unused interrupt vector to handle our low priority interrupts. When we need to do higher
  // level functions (like PBL_LOG), we trigger this lower-priority interrupt to fire. Since it runs at
  // configMAX_SYSCALL_INTERRUPT_PRIORITY or lower, it can at least call FreeRTOS ISR functions.
  NVIC_SetPriority(WATCHDOG_FREERTOS_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(WATCHDOG_FREERTOS_IRQn);

  // create the app throttling timer
  s_throttle_timer_id = new_timer_create();
}

void task_watchdog_feed(void) {
  s_ticks_since_successful_feed++;
  prv_task_watchdog_feed();
}

static void task_watchdog_disable_interrupt() {
  taskENTER_CRITICAL();
}

static void task_watchdog_enable_interrupt() {
  taskEXIT_CRITICAL();
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

void task_watchdog_pause(unsigned int seconds) {
  task_watchdog_disable_interrupt();
  s_pause_ticks_remaining = seconds * TIMER_INTERRUPT_HZ;
  task_watchdog_enable_interrupt();
}

void task_watchdog_resume(void) {
  task_watchdog_disable_interrupt();
  s_pause_ticks_remaining = 0;
  task_watchdog_enable_interrupt();
}

void task_watchdog_step_elapsed_time_ms(uint32_t elapsed_ms) {
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

  // Handle pause state
  if (s_pause_ticks_remaining > 0) {
    s_pause_ticks_remaining--;
    watchdog_feed();
    s_ticks_since_successful_feed = 0;
    return;
  }

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
      NVIC_SetPendingIRQ(WATCHDOG_FREERTOS_IRQn);

      s_last_warning_message_tick_time = 0;
    }
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
    NVIC_SetPendingIRQ(WATCHDOG_FREERTOS_IRQn);
    // If the low priority interrupt hasn't reset us by the time 6.5 seconds
    // rolls around (it will issue the reset if executed at least 6 seconds
    // after s_last_successful_feed_time), it likely means that we are stuck in
    // an ISR or low priority interrupts are disabled, so coredump now
    if (s_ticks_since_successful_feed >= WATCHDOG_COREDUMP_TICK_CNT) {
#if !defined(CONFIG_NO_WATCHDOG)
      // Low-pri handler didn't run; capture stuck_task_pc/lr ourselves so
      // the coredump has a real PC to fingerprint on.
      prv_capture_stuck_task_info(&reboot_reason);
      reboot_reason_clear();
      reboot_reason_set(&reboot_reason);
      reset_due_to_software_failure();
#endif
    }
    s_last_warning_message_tick_time = s_ticks_since_successful_feed;
  }
}
