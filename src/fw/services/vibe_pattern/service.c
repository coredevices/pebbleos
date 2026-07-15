/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/vibe_pattern.h"

#include "drivers/accel.h"
#include "drivers/vibe.h"
#include "drivers/battery.h"
#include "drivers/rtc.h"

#include "kernel/pebble_tasks.h"

#include "pbl/util/list.h"
#include "pbl/util/math.h"

#include "pbl/os/mutex.h"

#include "pbl/services/analytics/analytics.h"
#include "pbl/services/accel_manager.h"
#include "pbl/services/new_timer/new_timer.h"
#include "kernel/events.h"

#include "kernel/pbl_malloc.h"
#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"
#include "system/logging.h"
#include "system/passert.h"

#include <inttypes.h>
#include <stddef.h>

PBL_LOG_MODULE_DEFINE(service_vibe_pattern, CONFIG_SERVICE_VIBE_PATTERN_LOG_LEVEL);

// Pattern lifecycle logs are DBG by default, elevated to INFO when the
// Vibe Log Info debugging toggle is on so field captures include them.
#if !defined(CONFIG_RECOVERY_FW)
extern bool shell_prefs_get_vibe_log_info_enabled(void);
#define VIBE_PATTERN_LOG(fmt, ...)                    \
  do {                                                \
    if (shell_prefs_get_vibe_log_info_enabled()) {    \
      PBL_LOG_INFO(fmt, ##__VA_ARGS__);               \
    } else {                                          \
      PBL_LOG_DBG(fmt, ##__VA_ARGS__);                \
    }                                                 \
  } while (0)
#else
#define VIBE_PATTERN_LOG(fmt, ...) PBL_LOG_DBG(fmt, ##__VA_ARGS__)
#endif

typedef struct {
  ListNode list_node;
  uint64_t time_start;
  uint64_t time_end;
} VibeHistory;

// The maximum history we need to keep is based on the maximum time between accel samples (the
// lowest sampling rate) in milliseconds and the maximum number of accel samples per update.
#define MAX_HISTORY_MS (accel_get_max_num_samples() * 1000 / ACCEL_MINIMUM_SAMPLING_RATE)
#define END_NOT_SET 0
#define HISTORY_CLEAR_ALL 0

static PebbleMutex *s_vibe_history_mutex = NULL;
static VibeHistory *s_vibe_history = NULL;
static bool s_vibe_history_enabled = false;
static bool s_vibe_service_enabled = true;

DEFINE_SYSCALL(bool, sys_vibe_history_was_vibrating, uint64_t time_search) {
  bool rc = false;

  PBL_ASSERTN(s_vibe_history_mutex);
  mutex_lock(s_vibe_history_mutex);
  VibeHistory *node = s_vibe_history;
  while (node) {
    if (node->time_end == END_NOT_SET && time_search >= node->time_start) {
      rc = true;
      break;
    }
    if (time_search >= node->time_start && time_search <= node->time_end) {
      rc = true;
      break;
    }
    node = (VibeHistory*)list_get_next((ListNode*)node);
  }
  mutex_unlock(s_vibe_history_mutex);
  return rc;
}

// @param cutoff The time to cut off the list at. 0 means to clear the list.
static void prv_vibe_history_clear(uint64_t cutoff) {
  PBL_ASSERTN(s_vibe_history_mutex);
  mutex_assert_held_by_curr_task(s_vibe_history_mutex, true);
  while (s_vibe_history && s_vibe_history->time_end != END_NOT_SET) {
    VibeHistory *vibe = s_vibe_history;
    if (cutoff != HISTORY_CLEAR_ALL && vibe->time_end >= cutoff) {
      break;
    }
    s_vibe_history = (VibeHistory*)list_get_next((ListNode*)vibe);
    kernel_free(vibe);
  }
}

DEFINE_SYSCALL(void, sys_vibe_history_start_collecting, void) {
  s_vibe_history_enabled = true;
}

DEFINE_SYSCALL(void, sys_vibe_history_stop_collecting, void) {
  s_vibe_history_enabled = false;
  mutex_lock(s_vibe_history_mutex);
  prv_vibe_history_clear(HISTORY_CLEAR_ALL);
  mutex_unlock(s_vibe_history_mutex);
}

static void prv_vibe_history_start_event(void) {
  if (!s_vibe_history_enabled) {
    return;
  }
  VibeHistory *vibe = kernel_malloc(sizeof(VibeHistory));
  if (vibe == NULL) {
    s_vibe_history_enabled = false;
    return;
  }
  list_init((ListNode*)vibe);
  time_t s;
  uint16_t ms;
  rtc_get_time_ms(&s, &ms);
  vibe->time_start = ((uint64_t)s) * 1000 + ms;
  vibe->time_end = END_NOT_SET;

  PBL_ASSERTN(s_vibe_history_mutex);
  mutex_lock(s_vibe_history_mutex);
  if (s_vibe_history == NULL) {
    s_vibe_history = vibe;
  } else {
    list_append((ListNode*)s_vibe_history, (ListNode*)vibe);
  }
  prv_vibe_history_clear(vibe->time_start - MAX_HISTORY_MS);
  mutex_unlock(s_vibe_history_mutex);
}

// Ends the last vibration event
static void prv_vibe_history_end_event(void) {
  if (!s_vibe_history_enabled) {
    return;
  }
  if (!s_vibe_history) {
    // Possible that it was enabled while the watch was vibrating
    return;
  }

  time_t s;
  uint16_t ms;
  rtc_get_time_ms(&s, &ms);

  mutex_lock(s_vibe_history_mutex);
  VibeHistory *vibe = (VibeHistory*)list_get_tail((ListNode*)s_vibe_history);
  if (vibe->time_end == END_NOT_SET) {
    vibe->time_end = ((uint64_t)s) * 1000 + ms;
  }
  mutex_unlock(s_vibe_history_mutex);
}

typedef struct {
  ListNode list_node;
  uint32_t duration_ms;
  int32_t  strength;
} VibePatternStep;

static const uint32_t MAX_VIBE_DURATION_MS = 10000;

static int s_pattern_timer = TIMER_INVALID_ID;
static bool s_pattern_in_progress = false;
// Source that started the active pattern; VibePatternOwner_Other when none is live.
static VibePatternOwner s_pattern_owner = VibePatternOwner_Other;
// Owner to assign to the next started pattern, set by vibe_pattern_set_owner();
// consumed (reset to _Other) when the pattern starts.
static VibePatternOwner s_pending_owner = VibePatternOwner_Other;
// Reference points for drift-free step scheduling. The pattern's t=0 is anchored
// to s_pattern_start_ticks; s_pattern_deadline_ms is the (start-relative)
// completion time of the step the timer is currently waiting on. Each step's
// timeout is computed against an absolute deadline so per-callback scheduling
// jitter does not accumulate across long patterns (the Reveille score, for
// example, expands to ~180 chained steps).
static RtcTicks s_pattern_start_ticks = 0;
static uint64_t s_pattern_deadline_ms = 0;
// s_vibe_strength is the current vibration strength setting of the motor
static int32_t s_vibe_strength = VIBE_STRENGTH_OFF;
// s_vibe_strength_default is the vibrations trength of the motor used when one is not specified
// explicitly, and can be changed in the notification vibration strength setting.
static int32_t s_vibe_strength_default = VIBE_STRENGTH_MAX;

static PebbleMutex *s_vibe_pattern_mutex = NULL;
static VibePatternStep *s_vibe_queue_head = NULL;

//! Analytics: Track time-weighted average strength
static uint64_t s_strength_time_product_sum; // Sum of (strength_pct × time_ms)
static RtcTicks s_last_strength_sample_ticks; // Timestamp of last sample
static uint8_t s_last_sampled_strength_pct; // Last strength percentage sampled
static uint32_t s_total_vibe_on_time_ms; // Total vibe on time tracked internally

void vibes_init() {
  s_vibe_history_mutex = mutex_create();
  s_vibe_pattern_mutex = mutex_create();
  s_pattern_in_progress = false;
  s_pattern_timer = new_timer_create();

  // Initialize strength analytics tracking
  s_strength_time_product_sum = 0;
  s_last_strength_sample_ticks = 0;
  s_last_sampled_strength_pct = 0;
  s_total_vibe_on_time_ms = 0;
}

static void prv_update_strength_analytics(uint8_t new_strength_pct) {
  RtcTicks now_ticks = rtc_get_ticks();

  // Calculate time delta in ms since last sample
  if (s_last_strength_sample_ticks > 0) {
    uint32_t time_delta_ms = ((now_ticks - s_last_strength_sample_ticks) * 1000) / RTC_TICKS_HZ;

    // Accumulate strength × time for weighted average calculation
    // Use last sampled strength for the period that just elapsed
    s_strength_time_product_sum += (uint64_t)s_last_sampled_strength_pct * time_delta_ms;

    // Track total on-time when strength is above zero
    if (s_last_sampled_strength_pct > 0) {
      s_total_vibe_on_time_ms += time_delta_ms;
    }
  }

  // Update tracking variables
  s_last_strength_sample_ticks = now_ticks;
  s_last_sampled_strength_pct = new_strength_pct;
}

//! Turn the vibe motor on or off.
//!
//! This function should be used instead of vibe_ctl so that the vibe
//! history is kept in sync with the vibe state.
//! The caller must be holding s_vibe_pattern_mutex
static void prv_vibes_set_vibe_strength(int32_t new_strength) {
  mutex_assert_held_by_curr_task(s_vibe_pattern_mutex, true);
  if (!s_vibe_service_enabled) {
    PBL_ASSERTN(s_vibe_strength == VIBE_STRENGTH_OFF);
    return;
  }

  // Track strength for analytics (convert to percentage: -100..100 -> 0..100%)
  // Take absolute value since we care about vibration intensity, not direction
  uint8_t new_strength_pct = (ABS(new_strength) * 100) / VIBE_STRENGTH_MAX;
  prv_update_strength_analytics(new_strength_pct);

  if (new_strength != VIBE_STRENGTH_OFF) {
    vibe_set_strength(new_strength);
    vibe_ctl(true /* on */);
    if (s_vibe_strength == VIBE_STRENGTH_OFF) {
      // Transitioning from off to on
      PBL_ANALYTICS_TIMER_START(vibrator_on_time_ms);
      prv_vibe_history_start_event();
    }
  } else {
    vibe_ctl(false /* on */);
    if (s_vibe_strength != VIBE_STRENGTH_OFF) {
      // Transitioning from on to off
      PBL_ANALYTICS_TIMER_STOP(vibrator_on_time_ms);
      prv_vibe_history_end_event();
    }
  }
  s_vibe_strength = new_strength;
}

void vibe_service_set_enabled(bool enable) {
  mutex_lock(s_vibe_pattern_mutex);
  if (enable != s_vibe_service_enabled) {
    // ensure that the vibe is off before disabling it. No op if enabling it
    prv_vibes_set_vibe_strength(VIBE_STRENGTH_OFF);
    s_vibe_service_enabled = enable;
  }
  mutex_unlock(s_vibe_pattern_mutex);
}

//! Extend the pattern's absolute deadline by step_duration_ms and return the
//! timer duration to schedule for the next step, compensating for any
//! callback-scheduling latency that has elapsed since the pattern started.
//! Caller must hold s_vibe_pattern_mutex.
static uint32_t prv_next_timeout_ms(uint32_t step_duration_ms) {
  s_pattern_deadline_ms += step_duration_ms;
  const RtcTicks elapsed_ticks = rtc_get_ticks() - s_pattern_start_ticks;
  const uint64_t elapsed_ms = (elapsed_ticks * 1000) / RTC_TICKS_HZ;
  if (s_pattern_deadline_ms <= elapsed_ms) {
    // Already late — fire as soon as the timer service can. The deadline
    // accumulator preserves the planned schedule so subsequent steps still
    // line up against the original anchor.
    return 1;
  }
  return (uint32_t)(s_pattern_deadline_ms - elapsed_ms);
}

static void prv_timer_callback(void* data) {
  if (s_vibe_queue_head == NULL) {
    PBL_LOG_ERR("Tried to handle a vibe event with a null vibe queue");
    return;
  }

  mutex_lock(s_vibe_pattern_mutex);

  // remove the event I've finished
  VibePatternStep *removed_node = s_vibe_queue_head;
  s_vibe_queue_head = (VibePatternStep*)list_pop_head((ListNode*)s_vibe_queue_head);
  kernel_free(removed_node);

  if (s_vibe_queue_head != NULL) {
    // move to the next step
    prv_vibes_set_vibe_strength(s_vibe_queue_head->strength);
    const uint32_t next_ms = prv_next_timeout_ms(s_vibe_queue_head->duration_ms);
    bool success = new_timer_start(s_pattern_timer, next_ms,
                                   prv_timer_callback, NULL, 0 /*flags*/);
    PBL_ASSERTN(success);
  } else {
    // I'm done with the active pattern
    // make sure it's off
    prv_vibes_set_vibe_strength(VIBE_STRENGTH_OFF);
    s_pattern_in_progress = false;
    s_pattern_owner = VibePatternOwner_Other;
    VIBE_PATTERN_LOG("vibe_pattern: pattern complete");
  }

  mutex_unlock(s_vibe_pattern_mutex);
}

int32_t vibes_get_vibe_strength(void) {
  return s_vibe_strength;
}

int32_t vibes_get_default_vibe_strength(void) {
  return s_vibe_strength_default;
}

void vibes_set_default_vibe_strength(int32_t vibe_strength_default) {
  s_vibe_strength_default = vibe_strength_default;
}

DEFINE_SYSCALL(int32_t, sys_vibe_get_vibe_strength, void) {
  return vibes_get_vibe_strength();
}

bool prv_vibe_pattern_enqueue_step_raw(uint32_t duration_ms, int32_t strength) {
  mutex_lock(s_vibe_pattern_mutex);

  if (s_pattern_in_progress) {
    mutex_unlock(s_vibe_pattern_mutex);
    return false;
  }

  VibePatternStep *step = kernel_malloc(sizeof(VibePatternStep));
  if (step == NULL) {
    PBL_LOG_ERR("Couldn't malloc for a vibe step");
    mutex_unlock(s_vibe_pattern_mutex);
    return false;
  }

  list_init((ListNode*)step);
  step->duration_ms = MIN(duration_ms, MAX_VIBE_DURATION_MS);
  step->strength = strength;

  if (s_vibe_queue_head == NULL) {
    // Fresh queue: take the explicitly-set owner, else default by task.
    if (s_pending_owner != VibePatternOwner_Other) {
      s_pattern_owner = s_pending_owner;
    } else {
      s_pattern_owner = (pebble_task_get_current() == PebbleTask_App)
                            ? VibePatternOwner_App : VibePatternOwner_Other;
    }
    s_pending_owner = VibePatternOwner_Other;
    s_vibe_queue_head = step;
  } else {
    list_append((ListNode*)s_vibe_queue_head, (ListNode*)step);
  }

  mutex_unlock(s_vibe_pattern_mutex);

  return true;
}

DEFINE_SYSCALL(bool, sys_vibe_pattern_enqueue_step_raw, uint32_t duration_ms, int32_t strength) {
  return prv_vibe_pattern_enqueue_step_raw(duration_ms, strength);
}

DEFINE_SYSCALL(bool, sys_vibe_pattern_enqueue_step, uint32_t duration_ms, bool on) {
  return prv_vibe_pattern_enqueue_step_raw(duration_ms, on ? s_vibe_strength_default
                                                           : VIBE_STRENGTH_OFF);
}

DEFINE_SYSCALL(void, sys_vibe_pattern_trigger_start, void) {
  mutex_lock(s_vibe_pattern_mutex);
  if (s_vibe_queue_head == NULL || s_pattern_in_progress) {
    // either no vibes queued or I've already started
    mutex_unlock(s_vibe_pattern_mutex);
    return;
  }

#if !defined(CONFIG_RECOVERY_FW)
  {
    unsigned int step_count = 0;
    uint32_t total_duration_ms = 0;
    VibePatternStep *step = s_vibe_queue_head;
    while (step) {
      step_count++;
      total_duration_ms += step->duration_ms;
      step = (VibePatternStep *)list_get_next((ListNode *)step);
    }
    VIBE_PATTERN_LOG("vibe_pattern: trigger_start, %u steps, %" PRIu32
                     "ms total, strength=%" PRId32,
                     step_count, total_duration_ms, s_vibe_queue_head->strength);
  }
#endif

  s_pattern_start_ticks = rtc_get_ticks();
  s_pattern_deadline_ms = 0;
  prv_vibes_set_vibe_strength(s_vibe_queue_head->strength);
  s_pattern_in_progress = true;
  const uint32_t first_ms = prv_next_timeout_ms(s_vibe_queue_head->duration_ms);
  bool success = new_timer_start(s_pattern_timer, first_ms,
                                 prv_timer_callback, NULL, 0 /*flags*/);
  PBL_ASSERTN(success);
  mutex_unlock(s_vibe_pattern_mutex);
}

// Stop the active pattern and drop queued steps. Caller holds s_vibe_pattern_mutex.
static void prv_clear_pattern_locked(void) {
  mutex_assert_held_by_curr_task(s_vibe_pattern_mutex, true);
  new_timer_stop(s_pattern_timer);
  unsigned int dropped_steps = 0;
  while (s_vibe_queue_head) {
    VibePatternStep *removed_node = s_vibe_queue_head;
    s_vibe_queue_head = (VibePatternStep*)list_pop_head((ListNode*)s_vibe_queue_head);
    kernel_free(removed_node);
    dropped_steps++;
  }
  // Log whether a pattern was still live and whether the motor was on: a
  // clear that finds the motor on with no active pattern is a wedged vibe.
  VIBE_PATTERN_LOG("vibe_pattern: clear, in_progress=%d, strength=%" PRId32
                   ", %u steps dropped",
                   s_pattern_in_progress, s_vibe_strength, dropped_steps);
  prv_vibes_set_vibe_strength(VIBE_STRENGTH_OFF);
  s_pattern_in_progress = false;
  s_pattern_owner = VibePatternOwner_Other;
}

DEFINE_SYSCALL(void, sys_vibe_pattern_clear, void) {
  mutex_lock(s_vibe_pattern_mutex);
  prv_clear_pattern_locked();
  mutex_unlock(s_vibe_pattern_mutex);
}

void vibe_pattern_set_owner(VibePatternOwner owner) {
  mutex_lock(s_vibe_pattern_mutex);
  s_pending_owner = owner;
  mutex_unlock(s_vibe_pattern_mutex);
}

// Clear the active pattern only if `owner` started it; no-op otherwise.
void vibe_pattern_clear_for_owner(VibePatternOwner owner) {
  mutex_lock(s_vibe_pattern_mutex);
  if (s_pattern_owner == owner) {
    prv_clear_pattern_locked();
  }
  mutex_unlock(s_vibe_pattern_mutex);
}

void pbl_analytics_external_collect_vibe_stats(void) {
  mutex_lock(s_vibe_pattern_mutex);

  // Capture one final sample to account for time since last strength change
  uint8_t current_strength_pct = (ABS(s_vibe_strength) * 100) / VIBE_STRENGTH_MAX;
  prv_update_strength_analytics(current_strength_pct);

  // Calculate time-weighted average strength using internally tracked on-time
  uint32_t avg_strength_pct = 0;
  if (s_total_vibe_on_time_ms > 0) {
    // Calculate weighted average: sum(strength × time) / total_time
    avg_strength_pct = s_strength_time_product_sum / s_total_vibe_on_time_ms;
  }

  PBL_ANALYTICS_SET_UNSIGNED(vibrator_avg_strength_pct, avg_strength_pct);

  // Reset accumulators for next period
  s_strength_time_product_sum = 0;
  s_total_vibe_on_time_ms = 0;
  s_last_strength_sample_ticks = rtc_get_ticks();

  mutex_unlock(s_vibe_pattern_mutex);
}
