/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/touch/touch.h"
#include "pbl/services/touch/touch_event.h"
#include "pbl/services/touch/touch_session.h"

#include <pbl/drivers/display/display.h>
#include <pbl/drivers/touch/touch_sensor.h>
#include "kernel/event_loop.h"
#include "kernel/events.h"
#include "kernel/pebble_tasks.h"
#include "pbl/services/event_service.h"
#include "pbl/services/analytics/analytics.h"
#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "pbl/os/mutex.h"
#include "system/passert.h"

PBL_LOG_MODULE_DEFINE(service_touch, CONFIG_SERVICE_TOUCH_LOG_LEVEL);

static TouchState s_touch_state = TouchState_FingerUp;
static int16_t s_last_x;
static int16_t s_last_y;

static PebbleMutex *s_touch_mutex;

static uint8_t s_subscriber_count = 0;
//! Bitmask by PebbleTask of tasks with a live raw-slot subscription
//! (touch_service_subscribe). Tracked explicitly because the nav twins'
//! system-slot handlers share the same per-task event-service subscription, so
//! raw subscribers cannot be derived from s_subscriber_count.
static uint8_t s_raw_subscriber_tasks = 0;
static bool s_backlight_subscribed = false;
static bool s_system_hold_subscribed = false;
static bool s_nav_enabled = false;
static bool s_app_nav_active = false;
static bool s_globally_enabled = true;
static bool s_rotated = false;
//! Set while an injected gesture owns the sensor, between its touchdown and liftoff. Physical
//! samples are ignored for its duration so the synthetic path cannot be corrupted mid-swipe.
static bool s_injecting = false;

static void prv_apply_rotation(int16_t *x, int16_t *y) {
  if (s_rotated) {
    *x = (DISP_COLS - 1) - *x;
    *y = (DISP_ROWS - 1) - *y;
  }
}

static void prv_add_subscriber_cb(PebbleTask task) {
  mutex_lock(s_touch_mutex);
  // Honor the global kill switch: when touch is globally disabled, track the
  // subscriber count but don't power up the sensor.
  if (++s_subscriber_count == 1 && s_globally_enabled) {
    touch_sensor_set_enabled(true);
  }
  PBL_LOG_DBG("Touch: subscriber added, count=%" PRIu8, s_subscriber_count);
  mutex_unlock(s_touch_mutex);
}

static void prv_remove_subscriber_cb(PebbleTask task) {
  mutex_lock(s_touch_mutex);
  PBL_ASSERTN(s_subscriber_count > 0);
  if (--s_subscriber_count == 0 && s_globally_enabled) {
    touch_sensor_set_enabled(false);
  }
  // An app whose shared touch subscription disappears (task exit, or both
  // slots emptied) cannot have a live raw handler or nav dispatcher anymore;
  // drop its raw-slot mark and the app-nav-active flag so a crashed app cannot
  // leak backlight-follow behavior.
  if (task == PebbleTask_App) {
    s_raw_subscriber_tasks &= (uint8_t)~(1u << task);
    s_app_nav_active = false;
  }
  PBL_LOG_DBG("Touch: subscriber removed, count=%" PRIu8, s_subscriber_count);
  mutex_unlock(s_touch_mutex);
}

void touch_init(void) {
  s_touch_mutex = mutex_create();

  event_service_init(PEBBLE_TOUCH_EVENT, &prv_add_subscriber_cb,
      &prv_remove_subscriber_cb);
  event_service_init(PEBBLE_GESTURE_EVENT, &prv_add_subscriber_cb,
      &prv_remove_subscriber_cb);
}

bool touch_nav_enabled(void) {
  mutex_lock(s_touch_mutex);
  const bool enabled = s_nav_enabled;
  mutex_unlock(s_touch_mutex);
  return enabled;
}

void touch_set_nav_enabled(bool enabled) {
  mutex_lock(s_touch_mutex);
  s_nav_enabled = enabled;
  mutex_unlock(s_touch_mutex);
}

bool touch_app_nav_active(void) {
  mutex_lock(s_touch_mutex);
  const bool active = s_app_nav_active;
  mutex_unlock(s_touch_mutex);
  return active;
}

void touch_set_app_nav_active(bool active) {
  mutex_lock(s_touch_mutex);
  s_app_nav_active = active;
  mutex_unlock(s_touch_mutex);
}

bool touch_has_app_subscribers(void) {
  mutex_lock(s_touch_mutex);
  // Only explicit raw-slot subscriptions (touch_service_subscribe) count as
  // app subscribers. The event-service count cannot be used: the nav twins'
  // system-slot handlers share the same per-task subscription and would read
  // as apps here, re-enabling wake-on-every-touch with menu gestures off.
  const bool has_apps = (s_raw_subscriber_tasks != 0);
  mutex_unlock(s_touch_mutex);
  return has_apps;
}

void touch_service_set_globally_enabled(bool enabled) {
  mutex_lock(s_touch_mutex);
  if (s_globally_enabled == enabled) {
    mutex_unlock(s_touch_mutex);
    return;
  }
  s_globally_enabled = enabled;
  const bool sensor_enabled = enabled && (s_subscriber_count > 0);
  mutex_unlock(s_touch_mutex);

  touch_sensor_set_enabled(sensor_enabled);
  if (!enabled) {
    // A finger down when touch is torn down never gets a Liftoff otherwise, so
    // the backlight hold counter stays pinned. Synthesize one before the reset
    // zeroes the last coordinates.
    touch_release_active();
    // Avoid delivering stale position on re-enable.
    touch_reset();
  }
}

bool touch_service_is_globally_enabled(void) {
  mutex_lock(s_touch_mutex);
  const bool enabled = s_globally_enabled;
  mutex_unlock(s_touch_mutex);
  return enabled;
}

DEFINE_SYSCALL(bool, sys_touch_service_is_enabled, void) {
  return touch_service_is_globally_enabled();
}

DEFINE_SYSCALL(bool, sys_touch_nav_enabled, void) {
  return touch_nav_enabled();
}

DEFINE_SYSCALL(bool, sys_touch_app_nav_active, void) {
  return touch_app_nav_active();
}

DEFINE_SYSCALL(void, sys_touch_set_raw_subscribed, bool subscribed) {
  const PebbleTask task = pebble_task_get_current();
  mutex_lock(s_touch_mutex);
  if (subscribed) {
    s_raw_subscriber_tasks |= (uint8_t)(1u << task);
  } else {
    s_raw_subscriber_tasks &= (uint8_t)~(1u << task);
  }
  mutex_unlock(s_touch_mutex);
}

DEFINE_SYSCALL(void, sys_touch_reset, void) {
  touch_reset();
}

DEFINE_SYSCALL(void, sys_touch_release_active, void) {
  touch_release_active();
}

void touch_set_backlight_enabled(bool enabled) {
  mutex_lock(s_touch_mutex);
  if (enabled && !s_backlight_subscribed) {
    s_backlight_subscribed = true;
    mutex_unlock(s_touch_mutex);
    prv_add_subscriber_cb(PebbleTask_KernelMain);
    return;
  } else if (!enabled && s_backlight_subscribed) {
    s_backlight_subscribed = false;
    mutex_unlock(s_touch_mutex);
    prv_remove_subscriber_cb(PebbleTask_KernelMain);
    return;
  }
  mutex_unlock(s_touch_mutex);
}

void touch_set_system_hold(bool held) {
  // Permanent sensor hold for the nav feature: hold the sensor directly via the
  // subscriber refcount (no event-service subscription). Taken when the master
  // nav pref turns on, released when it turns off.
  mutex_lock(s_touch_mutex);
  if (held && !s_system_hold_subscribed) {
    s_system_hold_subscribed = true;
    mutex_unlock(s_touch_mutex);
    prv_add_subscriber_cb(PebbleTask_KernelMain);
    return;
  } else if (!held && s_system_hold_subscribed) {
    s_system_hold_subscribed = false;
    mutex_unlock(s_touch_mutex);
    prv_remove_subscriber_cb(PebbleTask_KernelMain);
    return;
  }
  mutex_unlock(s_touch_mutex);
}

static void prv_put_touch_event(TouchEventType type, int16_t x, int16_t y) {
  PebbleEvent e = {
    .type = PEBBLE_TOUCH_EVENT,
    .touch = {
      .event = {
        .type = type,
        .x = x,
        .y = y,
      },
    },
  };
  event_put(&e);
}

static void prv_put_gesture_event(GestureEventType gesture, int16_t x, int16_t y) {
  PebbleEvent e = {
    .type = PEBBLE_GESTURE_EVENT,
    .gesture = {
      .event = {
        .type = gesture,
        .x = x,
        .y = y,
      },
    },
  };
  event_put(&e);
}

static void prv_arm_session_cb(void *unused) {
  // touch_session is KernelMain-only, and injection runs off a timer callback.
  touch_session_arm(TouchSessionArmSource_Injected);
}

//! Applies one sample's state transition and queues the matching events.
//! Caller must hold s_touch_mutex. The events are queued with the lock held so a concurrent
//! sample cannot interleave between a transition and the event it produced.
static void prv_apply_update(TouchState touch_state, int16_t x, int16_t y, bool injected) {
  if (s_touch_state != touch_state) {
    s_touch_state = touch_state;
    s_last_x = x;
    s_last_y = y;

    if (touch_state == TouchState_FingerDown) {
      if (!injected) {
        // Synthetic contact must not read as user engagement.
        PBL_ANALYTICS_ADD(touch_event_count, 1);
      }
      PBL_LOG_DBG("Touch: Touchdown @ (%" PRId16 ", %" PRId16 ")", x, y);
      prv_put_touch_event(TouchEvent_Touchdown, x, y);
    } else {
      PBL_LOG_DBG("Touch: Liftoff");
      prv_put_touch_event(TouchEvent_Liftoff, x, y);
    }
    return;
  }

  if (touch_state == TouchState_FingerDown && (x != s_last_x || y != s_last_y)) {
    s_last_x = x;
    s_last_y = y;
    PBL_LOG_DBG("Touch: Position Update @ (%" PRId16 ", %" PRId16 ")", x, y);
    prv_put_touch_event(TouchEvent_PositionUpdate, x, y);
  }
}

void touch_handle_update(TouchState touch_state, int16_t x, int16_t y) {
  mutex_lock(s_touch_mutex);
  if (!s_globally_enabled || s_injecting) {
    mutex_unlock(s_touch_mutex);
    return;
  }
  prv_apply_rotation(&x, &y);
  prv_apply_update(touch_state, x, y, false /* injected */);
  mutex_unlock(s_touch_mutex);
}

bool touch_handle_injected_update(TouchInjectPhase phase, int16_t x, int16_t y) {
  mutex_lock(s_touch_mutex);
  if (!s_globally_enabled) {
    mutex_unlock(s_touch_mutex);
    return false;
  }
  if (phase == TouchInjectPhase_Begin) {
    // Whoever puts a finger down first owns the sensor until the gesture ends.
    if (s_injecting || (s_touch_state == TouchState_FingerDown)) {
      mutex_unlock(s_touch_mutex);
      return false;
    }
    s_injecting = true;
    // Arm before the touchdown is queued: both land on KernelMain in order, so the gate sees an
    // armed session rather than dropping the gesture as unarmed idle-watchface contact.
    launcher_task_add_callback(prv_arm_session_cb, NULL);
  } else if (!s_injecting) {
    // The gesture lost the sensor (reset, or touch switched off and back on). Refusing here is
    // what lets the caller abort, rather than having this sample taken as a new touchdown from
    // the middle of its path.
    mutex_unlock(s_touch_mutex);
    return false;
  } else if (phase == TouchInjectPhase_End) {
    s_injecting = false;
  }

  // Injected coordinates are already the ones the UI observes, so no rotation is applied. The
  // sample is applied under the same lock hold as the arbitration above: dropping the lock in
  // between would let a physical sample or a reset interleave and garble the event stream.
  const TouchState touch_state =
      (phase == TouchInjectPhase_End) ? TouchState_FingerUp : TouchState_FingerDown;
  prv_apply_update(touch_state, x, y, true /* injected */);
  mutex_unlock(s_touch_mutex);
  return true;
}

bool touch_injection_is_available(void) {
  mutex_lock(s_touch_mutex);
  const bool available =
      s_globally_enabled && (s_injecting || (s_touch_state != TouchState_FingerDown));
  mutex_unlock(s_touch_mutex);
  return available;
}

void touch_handle_gesture(TouchGesture gesture, int16_t x, int16_t y) {
  mutex_lock(s_touch_mutex);

  if (!s_globally_enabled) {
    mutex_unlock(s_touch_mutex);
    return;
  }

  // Drivers report gestures alongside the raw samples they are derived from, so a real finger
  // landing mid-injection would otherwise reach subscribers even though its samples do not.
  if (s_injecting) {
    mutex_unlock(s_touch_mutex);
    return;
  }

  prv_apply_rotation(&x, &y);

  PBL_LOG_DBG("Gesture: %d @ (%" PRId16 ", %" PRId16 ")", gesture, x, y);

  switch (gesture) {
    case TouchGesture_Tap:
      PBL_ANALYTICS_ADD(gesture_tap_count, 1);
      prv_put_gesture_event(GestureEvent_Tap, x, y);
      break;
    case TouchGesture_DoubleTap:
      PBL_ANALYTICS_ADD(gesture_double_tap_count, 1);
      prv_put_gesture_event(GestureEvent_DoubleTap, x, y);
      break;
    default:
      break;
  }

  mutex_unlock(s_touch_mutex);
}

void touch_reset(void) {
  mutex_lock(s_touch_mutex);
  s_touch_state = TouchState_FingerUp;
  s_last_x = 0;
  s_last_y = 0;
  // Ownership ends with the finger it belonged to; leaving it set would block physical touch for
  // good, since the injected liftoff that would clear it can no longer arrive.
  s_injecting = false;
  mutex_unlock(s_touch_mutex);
}

void touch_release_active(void) {
  mutex_lock(s_touch_mutex);
  const bool was_down = (s_touch_state == TouchState_FingerDown);
  const int16_t x = s_last_x;
  const int16_t y = s_last_y;
  s_touch_state = TouchState_FingerUp;
  // The gesture is over however it was owned; see touch_reset().
  s_injecting = false;
  mutex_unlock(s_touch_mutex);

  if (was_down) {
    PBL_LOG_DBG("Touch: synthetic Liftoff @ (%" PRId16 ", %" PRId16 ")", x, y);
    prv_put_touch_event(TouchEvent_Liftoff, x, y);
  }
}

static bool s_wake_gate_latch;

void touch_wake_gate_stamp(TouchEvent *event, TouchWakeGateResult gate) {
  if (event->type == TouchEvent_Touchdown) {
    s_wake_gate_latch = gate.latch;
  }
  event->non_navigational = s_wake_gate_latch;
}

void touch_set_rotated(bool rotated) {
  mutex_lock(s_touch_mutex);
  s_rotated = rotated;
  mutex_unlock(s_touch_mutex);
}
