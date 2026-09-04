/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/light.h"

#include "board/board.h"
#include <pbl/drivers/ambient_light.h>
#include <pbl/drivers/backlight.h>
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
#include <pbl/drivers/backlight.h>
#endif
#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "kernel/low_power.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/util/math.h"
#include "services/light/als_screen_compensation.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "pbl/os/mutex.h"

#include "FreeRTOS.h"

PBL_LOG_MODULE_DEFINE(service_light, CONFIG_SERVICE_LIGHT_LOG_LEVEL);

typedef enum {
  LIGHT_STATE_ON = 1,           // backlight on, no timeouts
  LIGHT_STATE_ON_TIMED = 2,     // backlight on, will start fading after a period
  LIGHT_STATE_ON_FADING = 3,    // backlight in the process of fading out
  LIGHT_STATE_OFF = 4,          // backlight off; idle state
} BacklightState;

// the time duration of a fade out from full intensity
const uint32_t LIGHT_FADE_TIME_MS = 500;
// upper bound on fade-out steps
#define LIGHT_FADE_MAX_STEPS 20U
const uint8_t LIGHT_FADE_STEPS = LIGHT_FADE_MAX_STEPS;

/*
 *              ^
 *              |
 *     LIGHT_ON |            +---------------------------------+
 *              |           /                                   \
 *              |          /                                     \
 *              |         /                                       \
 *              |        /                                         \
 *              |       /                                           \
 *  LIGHT_ON/2  |      /+                                           +\
 *              |     / |                                           | \
 *              |    /  |                                           |  \
 *              |   /   |                                           |   \
 *              |  /    |                                           |    \
 *              | /     |                                           |     \
 *              |/      |                                           |      \
 *    LIGHT_OFF +-------|-------------------------------------------|--------->
 *                      |                                           |
 *                      |<----------------------------------------->|
 *                          Integrate over this range for the mean
 */

//! The current state of the backlight (example: ON/ON_TIMED/ON_FADING).
static BacklightState s_light_state;

//! The brightness of the display in a range between 0 and 100
static uint8_t s_current_brightness;

//! Timer to count down from the LIGHT_STATE_ON_TIMED state.
static TimerID s_timer_id;

//! Refcount of the number of buttons that are currently pushed
static int s_num_buttons_down;

//! The current app is forcing the light on and off, don't muck with it.
static bool s_user_controlled_state;

//! True while a touch is holding the backlight on. Lets app teardown release a
//! touch whose liftoff was never delivered. KernelMain-only.
static bool s_touch_holding;

#ifdef CONFIG_BACKLIGHT_HAS_COLOR
//! The app's requested backlight tint. Valid only when s_app_rgb_override_valid
//! is true; otherwise the LED uses the user default (white).
static uint32_t s_app_rgb_override;
static bool s_app_rgb_override_valid;

//! Count of active modal preempts (notifications, etc.) that temporarily
//! mask any app RGB override. The app's override stays stored; only while
//! this refcount is non-zero is the LED driven to the default color.
//! When the refcount returns to zero, the override (if any) is re-applied.
static uint8_t s_color_preempt_refcount;
#endif

//! For temporary disabling backlight (ie: low power mode)
static bool s_backlight_allowed = false;

//! Descending fade-out ladder: intensities that each land on a distinct
//! hardware backlight level (built once when the fade begins)
static uint8_t s_fade_levels[LIGHT_FADE_MAX_STEPS];
static uint8_t s_fade_level_count = 0;
static uint8_t s_fade_level_idx = 0;

//! Dwell time per fade rung, paced so a fade from full intensity takes
//! LIGHT_FADE_TIME_MS
static uint32_t s_fade_step_ms = 0;

//! Mutex to guard all the above state. We have a pattern of taking the lock in the public functions and assuming
//! it's already taken in the prv_ functions.
static PebbleMutex *s_mutex;

//! Analytics: Track time-weighted average intensity
static uint64_t s_intensity_time_product_sum; // Sum of (intensity_pct × time_ms)
static RtcTicks s_last_intensity_sample_ticks; // Timestamp of last sample
static uint8_t s_last_sampled_intensity_pct; // Last intensity percentage sampled
static uint32_t s_total_on_time_ms; // Total backlight on time tracked internally

//! Short-lived cache so back-to-back ALS consumers in the same wake path
//! (prv_light_allowed → prv_backlight_get_intensity, plus a button release
//! that follows the press within the TTL) skip the ~200 ms I2C poll.
static uint32_t s_als_cached_level;
static RtcTicks s_als_cached_ticks;  // 0 = invalid
#define ALS_CACHE_TTL_TICKS (RTC_TICKS_HZ)  // 1 second

//! Event-gated continuous ALS:
//!
//! Wake-related entry points call prv_als_prime_for_interaction(), which (if
//! not already primed) takes a single ambient_light_prime() refcount and
//! starts a release timer for ALS_PRIME_HOLDOFF_MS. Each subsequent wake-y
//! call rearms the timer. When it eventually fires, we drop the prime so the
//! sensor goes back to idle.
//!
//! While primed, the W1160 runs in continuous mode: ALS reads collapse to a
//! plain register read once one IT has elapsed, and the cache hits more often
//! because reads are cheap enough that callers don't gate on them. The
//! holdoff is sized to cover typical multi-wake patterns ("raise wrist, lower,
//! raise again a few seconds later") so we keep sampling across them rather
//! than re-paying the first-IT wait per wake.
static TimerID s_als_prime_release_timer_id;
static bool s_als_primed;
#define ALS_PRIME_HOLDOFF_MS (5000)

#if defined(CONFIG_DYNAMIC_BACKLIGHT) && !defined(CONFIG_RECOVERY_FW)
//! Lux level at which the dynamic backlight ramp reaches the user's max
//! intensity, per mode. Deliberately decoupled from the dark threshold that
//! gates backlight wakes: ramps topping out past it just stay dimmer.
static uint32_t prv_dynamic_mode_full_lux(BacklightDynamicMode mode) {
  switch (mode) {
    case BacklightDynamicMode_Bright:
      return 125;
    case BacklightDynamicMode_Dim:
      return 500;
    case BacklightDynamicMode_Standard:
    default:
      return 250;
  }
}

//! Intensity floor at 0 lux, per mode. Distinct floors keep the modes
//! visually distinguishable in a dark room, where the ramp contributes
//! nothing.
static uint8_t prv_dynamic_mode_floor_intensity(BacklightDynamicMode mode) {
  switch (mode) {
    case BacklightDynamicMode_Bright:
      return 30;
    case BacklightDynamicMode_Dim:
      return 10;
    case BacklightDynamicMode_Standard:
    default:
      return 20;
  }
}
#endif

static void prv_change_state(BacklightState new_state);

//! Timer callback: holdoff expired, drop the prime so the W1160 stops
//! integrating in the background. Runs on the new_timer task.
static void prv_als_prime_release_callback(void *data) {
  mutex_lock(s_mutex);
  if (s_als_primed) {
    s_als_primed = false;
    ambient_light_release();
  }
  mutex_unlock(s_mutex);
}

//! Open or extend an "interaction window" during which the W1160 is held in
//! continuous-sampling mode. Call from any wake-event-y entry point before
//! consulting ALS. Caller must hold s_mutex.
static void prv_als_prime_for_interaction(void) {
  if (!s_als_primed) {
    s_als_primed = true;
    ambient_light_prime();
  }
  new_timer_start(s_als_prime_release_timer_id, ALS_PRIME_HOLDOFF_MS,
                  prv_als_prime_release_callback, NULL, 0 /* flags */);
}

static uint32_t prv_get_als_level(void) {
  RtcTicks now = rtc_get_ticks();
  const bool cache_valid =
      s_als_cached_ticks != 0 &&
      (s_current_brightness > 0 || (now - s_als_cached_ticks) < ALS_CACHE_TTL_TICKS);
  if (cache_valid) {
    return s_als_cached_level;
  }
  uint32_t level = ambient_light_get_light_level();
#if defined(CONFIG_ALS_SCREEN_COMPENSATION) && !defined(CONFIG_RECOVERY_FW)
  // The ALS sits under the display; correct the reading for the transmittance
  // of the pixels in front of the sensor. Done once at ingest so every consumer
  // sees a screen-corrected value, and the cache below throttles the
  // framebuffer scan to at most once per TTL.
  level = als_compensation_correct(level);
#endif
  // Convert to lux (identity on boards without coefficients) so thresholds
  // and the backlight ramp operate in device-independent units.
  level = ambient_light_level_to_lux(level);
  s_als_cached_level = level;
  s_als_cached_ticks = now;
  return s_als_cached_level;
}

uint32_t light_get_ambient_lux(void) {
  return prv_get_als_level();
}

static bool prv_als_is_light(void) {
  return prv_get_als_level() > ambient_light_get_dark_threshold();
}

static void light_timer_callback(void *data) {
  mutex_lock(s_mutex);
  prv_change_state(LIGHT_STATE_ON_FADING);
  mutex_unlock(s_mutex);
}

static uint8_t prv_backlight_get_intensity(void) {
  // low_power_mode backlight intensity (25% of max brightness)
  const uint8_t backlight_low_power_intensity = 25;
  
  if (low_power_is_active()) {
    return backlight_low_power_intensity;
  }
  
#if defined(CONFIG_DYNAMIC_BACKLIGHT) && !defined(CONFIG_RECOVERY_FW)
  // Dynamic backlight: linear ramp from the mode's floor intensity at 0 lux up
  // to 100% at the mode's full-brightness lux level, then clamped to user_max.
  // This keeps the slope independent of the user's brightness preference, so a
  // user who caps their max at e.g. 60% still hits that cap partway up the ALS
  // range rather than only at the brightest end. prv_light_allowed()
  // independently rejects wakes above the dark threshold; paths that bypass it
  // (app-driven force-on, ambient-sensor pref off) sensibly land at user_max
  // here.
  const BacklightDynamicMode mode = backlight_get_dynamic_mode();
  if (mode != BacklightDynamicMode_Off) {
    const uint8_t floor_intensity = prv_dynamic_mode_floor_intensity(mode);
    const uint8_t user_max = backlight_get_intensity();
    const uint32_t als = prv_get_als_level();
    const uint32_t full_lux = prv_dynamic_mode_full_lux(mode);

    if (user_max <= floor_intensity) {
      return user_max;
    }
    if (als >= full_lux) {
      return user_max;
    }
    const uint32_t ramped = floor_intensity + ((100 - floor_intensity) * als) / full_lux;
    return (ramped > user_max) ? user_max : (uint8_t)ramped;
  }
#endif
  
  return backlight_get_intensity();
}

static void prv_update_intensity_analytics(uint8_t new_intensity_pct) {
  RtcTicks now_ticks = rtc_get_ticks();

  // Calculate time delta in ms since last sample
  if (s_last_intensity_sample_ticks > 0) {
    uint32_t time_delta_ms = ((now_ticks - s_last_intensity_sample_ticks) * 1000) / RTC_TICKS_HZ;

    // Accumulate intensity × time for weighted average calculation
    // Use last sampled intensity for the period that just elapsed
    s_intensity_time_product_sum += (uint64_t)s_last_sampled_intensity_pct * time_delta_ms;

    // Track total on-time when intensity is above zero
    if (s_last_sampled_intensity_pct > 0) {
      s_total_on_time_ms += time_delta_ms;
    }
  }

  // Update tracking variables
  s_last_intensity_sample_ticks = now_ticks;
  s_last_sampled_intensity_pct = new_intensity_pct;
}

#ifdef CONFIG_BACKLIGHT_HAS_COLOR
//! LED color to drive when no app has set an override. Backed by the
//! user's stored backlight-color preference, defaulting to BACKLIGHT_COLOR_WARM_WHITE.
static void prv_apply_rgb_color(void) {
  const bool preempted = (s_color_preempt_refcount > 0);
  const uint32_t color = (preempted || !s_app_rgb_override_valid)
                             ? backlight_get_default_color()
                             : s_app_rgb_override;
  backlight_set_color(color);
}
#endif

static void prv_change_brightness(uint8_t new_brightness) {
  // Scale the 0-100% to the maximum value allowed in hardware
  uint8_t scaled_brightness =
      DIVIDE_CEIL(new_brightness * (uint16_t)BOARD_CONFIG.backlight_on_percent, 100U);

  // Bleed-through gate around backlight 0↔on edges: while the LED is
  // illuminating the cover glass, the W1160 photodiode would latch
  // contaminated readings even though our cache pins to the pre-on value.
  // Suspending stops integration entirely, so the chip preserves the last
  // clean DATA_ALS until we resume. No-op while not primed (driver
  // composes suspend with the prime refcount).
  const bool turning_on = (s_current_brightness == 0U) && (new_brightness > 0U);
  const bool turning_off = (s_current_brightness > 0U) && (new_brightness == 0U);

  if (new_brightness == 0U) {
    PBL_ANALYTICS_TIMER_STOP(backlight_on_time_ms);
  } else {
    PBL_ANALYTICS_TIMER_START(backlight_on_time_ms);
  }

  prv_update_intensity_analytics(scaled_brightness);

  if (turning_on) {
    ambient_light_suspend();
  }
  backlight_set_brightness(scaled_brightness);
  if (turning_off) {
    ambient_light_resume();
  }
  s_current_brightness = new_brightness;

#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  // backlight_set_brightness re-applies the last RGB color at the new
  // intensity; follow up with the app override (or default white) so the
  // LED reflects the current color request for this state change.
  prv_apply_rgb_color();
#endif
}

//! Hardware backlight level a given intensity would produce.
static uint8_t prv_hw_level(uint8_t intensity) {
  return backlight_get_level(
      DIVIDE_CEIL(intensity * (uint16_t)BOARD_CONFIG.backlight_on_percent, 100U));
}

//! Build the descending ladder of intensities below `from` that each produce
//! a distinct hardware backlight level. On boards with continuous control
//! this is the classic LIGHT_FADE_STEPS-tick ramp; on quantized backlights
//! (e.g. the AW9364E's 16 codes) redundant steps collapse so every rung is a
//! visible change. `levels` must hold LIGHT_FADE_MAX_STEPS entries.
static uint8_t prv_build_fade_ladder(uint8_t from, uint8_t *levels) {
  if (from == 0U) {
    return 0U;
  }

  const uint8_t step = DIVIDE_CEIL(from, LIGHT_FADE_MAX_STEPS);
  uint8_t count = 0;
  uint8_t prev_hw = prv_hw_level(from);

  for (int16_t candidate = (int16_t)from - step; candidate > 0; candidate -= step) {
    const uint8_t hw = prv_hw_level((uint8_t)candidate);
    if (hw != prev_hw) {
      levels[count++] = (uint8_t)candidate;
      prev_hw = hw;
    }
  }
  return count;
}

static void prv_change_state(BacklightState new_state) {
  BacklightState old_state = s_light_state;
  s_light_state = new_state;

  // Calculate the new brightness and reset any timers based on our state.
  uint8_t new_brightness = 0;

  switch (new_state) {
    case LIGHT_STATE_ON:
      new_brightness = prv_backlight_get_intensity();
      new_timer_stop(s_timer_id);
      break;
    case LIGHT_STATE_ON_TIMED:
      new_brightness = prv_backlight_get_intensity();

      // Schedule the timer to move us from the ON_TIMED state to the ON_FADING state
      new_timer_start(s_timer_id, backlight_get_timeout_ms(),
                      light_timer_callback, NULL, 0 /* flags */);
      break;
    case LIGHT_STATE_ON_FADING:
      // Build the fade ladder only when we first enter fading state. Pacing
      // is normalized to a full-intensity fade, so fades from dimmer levels
      // walk fewer rungs at the same rate and finish sooner.
      if (old_state != LIGHT_STATE_ON_FADING) {
        uint8_t scratch[LIGHT_FADE_MAX_STEPS];
        s_fade_step_ms = LIGHT_FADE_TIME_MS / (prv_build_fade_ladder(100U, scratch) + 1U);
        s_fade_level_count = prv_build_fade_ladder(s_current_brightness, s_fade_levels);
        s_fade_level_idx = 0;
      }

      if (s_fade_level_idx >= s_fade_level_count) {
        new_brightness = 0;
        s_light_state = LIGHT_STATE_OFF;
      } else {
        new_brightness = s_fade_levels[s_fade_level_idx++];

        // Reschedule the timer so we step down the brightness again.
        new_timer_start(s_timer_id, s_fade_step_ms, light_timer_callback, NULL, 0 /* flags */);
      }
      break;
    case LIGHT_STATE_OFF:
      new_brightness = 0;
      new_timer_stop(s_timer_id);
      break;
  }

  if (s_current_brightness != new_brightness) {
    prv_change_brightness(new_brightness);
  } else if (new_state == LIGHT_STATE_ON || new_state == LIGHT_STATE_ON_TIMED) {
    // A user-initiated wake with unchanged brightness would otherwise
    // short-circuit. Re-apply the driver state so the hardware is brought back
    // in sync if it has drifted from what the firmware believes.
    backlight_refresh();
  }

  // Notify subscribers when the backlight transitions between on and off.
  // Treat any non-OFF state as "on" so apps see a single edge per wake.
  const bool was_on = (old_state != LIGHT_STATE_OFF);
  const bool is_on = (s_light_state != LIGHT_STATE_OFF);
  if (was_on != is_on) {
    PebbleEvent event = {
      .type = PEBBLE_BACKLIGHT_EVENT,
      .backlight = {
        .is_on = is_on,
      },
    };
    event_put(&event);
  }
}

static bool prv_light_allowed(void) {
  if (!s_backlight_allowed) {
    return false;
  }
  
  if (backlight_is_enabled()) {
    if (backlight_is_ambient_sensor_enabled()) {
      // If the light is off and it's bright outside, don't allow the light to turn on
      // (we don't need it!). Grab the mutex here so that the timer state machine doesn't change
      // the light brightness while we're checking the ambient light levels.
      bool allowed = !((s_current_brightness == 0) && prv_als_is_light());
      return allowed;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

void light_init(void) {
  s_light_state = LIGHT_STATE_OFF;
  s_current_brightness = 0;
  s_num_buttons_down = 0;
  s_user_controlled_state = false;
  s_touch_holding = false;
  s_fade_level_count = 0;
  s_fade_level_idx = 0;
  s_mutex = mutex_create();

  // Initialize intensity analytics tracking
  s_intensity_time_product_sum = 0;
  s_last_intensity_sample_ticks = 0;
  s_last_sampled_intensity_pct = 0;
  s_total_on_time_ms = 0;

  s_als_cached_level = 0;
  s_als_cached_ticks = 0;

  // Create the ALS release timer before the fade timer so existing tests that
  // pluck the most-recently-created idle timer (test_light.c:149) still pick
  // up s_timer_id.
  s_als_prime_release_timer_id = new_timer_create();
  s_als_primed = false;
  s_timer_id = new_timer_create();
}

void light_button_pressed(void) {
  mutex_lock(s_mutex);

  s_num_buttons_down++;
  if (s_num_buttons_down > 4) {
    PBL_LOG_ERR("More buttons were pressed than have been released.");
    s_num_buttons_down = 0;
  }

  prv_als_prime_for_interaction();

  // set the state to be on; releasing buttons will start the timer counting down
  if (prv_light_allowed()) {
    prv_change_state(LIGHT_STATE_ON);
  }

  mutex_unlock(s_mutex);
}

void light_button_released(void) {
  mutex_lock(s_mutex);

  s_num_buttons_down--;
  if (s_num_buttons_down < 0) {
    PBL_LOG_ERR("More buttons were released than have been pressed.");
    s_num_buttons_down = 0;
  }

  if (s_num_buttons_down == 0 &&
      s_light_state == LIGHT_STATE_ON &&
      !s_user_controlled_state) {
    // no more buttons pressed: wait for a bit and then start the fade-out timer
    prv_change_state(LIGHT_STATE_ON_TIMED);
  }

  mutex_unlock(s_mutex);
}

void light_touch_down(void) {
  // Mirror a touch onto the button refcount, coalescing repeats to one ref.
  if (s_touch_holding) {
    return;
  }
  s_touch_holding = true;
  light_button_pressed();
}

void light_touch_up(void) {
  if (!s_touch_holding) {
    return;
  }
  s_touch_holding = false;
  light_button_released();
}

void light_enable_interaction(void) {
  mutex_lock(s_mutex);

  //if some buttons are held or light_enable is asserted, do nothing
  if (s_num_buttons_down > 0 || s_light_state == LIGHT_STATE_ON) {
    mutex_unlock(s_mutex);
    return;
  }

  prv_als_prime_for_interaction();

  if (prv_light_allowed()) {
    prv_change_state(LIGHT_STATE_ON_TIMED);
  }

  mutex_unlock(s_mutex);
}

void light_enable(bool enable) {
  mutex_lock(s_mutex);

  // This function is a bit of a black sheep - it dives in and messes with the normal
  // flow of the state machine.
  // We don't actually use it, but it is now documented and used in the SDK, so
  // I am reluctant to chop it out.

  s_user_controlled_state = enable;

  if (enable) {
    prv_change_state(LIGHT_STATE_ON);
  } else if (s_num_buttons_down == 0) {
    // reset the state if someone calls light_enable(false);
    // (unless there are buttons pressed, then leave the backlight on)
    prv_change_state(LIGHT_STATE_OFF);
  }

  mutex_unlock(s_mutex);
}

void light_enable_respect_settings(bool enable) {
  mutex_lock(s_mutex);

  s_user_controlled_state = enable;

  if (enable) {
    prv_als_prime_for_interaction();
    if (prv_light_allowed()) {
      prv_change_state(LIGHT_STATE_ON);
    }
  } else if (s_num_buttons_down == 0) {
    prv_change_state(LIGHT_STATE_OFF);
  }

  mutex_unlock(s_mutex);
}

void light_reset_user_controlled(void) {
  // Release a touch that never saw its liftoff (app exited mid-press) so the
  // button refcount can't leak. Call before locking; light_touch_up locks.
  light_touch_up();

  mutex_lock(s_mutex);

  // http://www.youtube.com/watch?v=6t_KgE6Yuqg
  if (s_user_controlled_state) {
    s_user_controlled_state = false;

    if (s_num_buttons_down == 0) {
      prv_change_state(LIGHT_STATE_OFF);
    }
  }

  mutex_unlock(s_mutex);
}

void light_set_color_rgb888(uint32_t rgb) {
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  mutex_lock(s_mutex);
  s_app_rgb_override = rgb & 0x00FFFFFF;
  s_app_rgb_override_valid = true;
  if (s_light_state != LIGHT_STATE_OFF) {
    prv_apply_rgb_color();
  }
  mutex_unlock(s_mutex);
#else
  (void)rgb;
#endif
}

void light_set_system_color(void) {
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  mutex_lock(s_mutex);
  if (s_app_rgb_override_valid) {
    s_app_rgb_override_valid = false;
    if (s_light_state != LIGHT_STATE_OFF) {
      prv_apply_rgb_color();
    }
  }
  mutex_unlock(s_mutex);
#endif
}

void light_system_color_request(void) {
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  mutex_lock(s_mutex);
  if (s_color_preempt_refcount < UINT8_MAX) {
    s_color_preempt_refcount++;
  }
  if (s_color_preempt_refcount == 1 && s_light_state != LIGHT_STATE_OFF) {
    prv_apply_rgb_color();
  }
  mutex_unlock(s_mutex);
#endif
}

void light_system_color_release(void) {
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  mutex_lock(s_mutex);
  if (s_color_preempt_refcount > 0) {
    s_color_preempt_refcount--;
    if (s_color_preempt_refcount == 0 && s_light_state != LIGHT_STATE_OFF) {
      prv_apply_rgb_color();
    }
  }
  mutex_unlock(s_mutex);
#endif
}

static void prv_light_reset_to_timed_mode(void) {
  mutex_lock(s_mutex);

  if (s_user_controlled_state) {
    s_user_controlled_state = false;
    prv_als_prime_for_interaction();
    if (prv_light_allowed()) {
      prv_change_state(LIGHT_STATE_ON_TIMED);
    }
  }

  mutex_unlock(s_mutex);
}

void light_toggle_enabled(void) {
  mutex_lock(s_mutex);

  // Toggling the setting is the user's only escape hatch if some path left
  // s_user_controlled_state stuck on. Clear it here so a subsequent button
  // release will actually start the auto-off timer.
  s_user_controlled_state = false;

  backlight_set_enabled(!backlight_is_enabled());
  if (prv_light_allowed()) {
    prv_change_state(LIGHT_STATE_ON_TIMED);
  } else {
    prv_change_state(LIGHT_STATE_OFF);
  }
  mutex_unlock(s_mutex);
}

void light_toggle_ambient_sensor_enabled(void) {
  mutex_lock(s_mutex);
  s_user_controlled_state = false;
  backlight_set_ambient_sensor_enabled(!backlight_is_ambient_sensor_enabled());
  if (prv_light_allowed() && !prv_als_is_light()) {
    prv_change_state(LIGHT_STATE_ON_TIMED);
  } else {
    prv_change_state(LIGHT_STATE_OFF);
    // FIXME: PBL-24793 There is an edge case of when the backlight has timed off
    // or you're toggling it from no ambient (always light on buttons) to ambient,
    // you will see it turn on and immediately off if its bright out
  }
  mutex_unlock(s_mutex);
}

#ifdef CONFIG_DYNAMIC_BACKLIGHT
void light_set_dynamic_mode(BacklightDynamicMode mode) {
  mutex_lock(s_mutex);
  backlight_set_dynamic_mode(mode);
  // Briefly turn the light on so the user sees the new mode's brightness.
  if (prv_light_allowed()) {
    prv_change_state(LIGHT_STATE_ON_TIMED);
  }
  mutex_unlock(s_mutex);
}
#endif

void light_allow(bool allowed) {
  if (s_backlight_allowed && !allowed) {
    prv_change_state(LIGHT_STATE_OFF);
  }
  s_backlight_allowed = allowed;
}

DEFINE_SYSCALL(bool, sys_light_is_on, void) {
  return light_is_on();
}

DEFINE_SYSCALL(void, sys_light_enable_interaction, void) {
  light_enable_interaction();
}

DEFINE_SYSCALL(void, sys_light_enable, bool enable) {
  light_enable(enable);
}

DEFINE_SYSCALL(void, sys_light_enable_respect_settings, bool enable) {
  light_enable_respect_settings(enable);
}

DEFINE_SYSCALL(void, sys_light_reset_to_timed_mode, void) {
  prv_light_reset_to_timed_mode();
}

DEFINE_SYSCALL(void, sys_light_set_color_rgb888, uint32_t rgb) {
  light_set_color_rgb888(rgb);
}

DEFINE_SYSCALL(void, sys_light_set_system_color, void) {
  light_set_system_color();
}

extern BacklightBehaviour backlight_get_behaviour(void);

uint8_t light_get_current_brightness_percent(void) {
  return s_current_brightness;
}

bool light_is_on(void) {
  return s_light_state != LIGHT_STATE_OFF;
}

void pbl_analytics_external_collect_backlight_stats(void) {
  mutex_lock(s_mutex);

  // Capture one final sample to account for time since last brightness change
  prv_update_intensity_analytics(s_current_brightness);

  // Calculate time-weighted average intensity using internally tracked on-time
  uint32_t avg_intensity_pct = 0;
  if (s_total_on_time_ms > 0) {
    // Calculate weighted average: sum(intensity × time) / total_time
    avg_intensity_pct = s_intensity_time_product_sum / s_total_on_time_ms;
  }

  PBL_ANALYTICS_SET_UNSIGNED(backlight_avg_intensity_pct, avg_intensity_pct);

  // Reset accumulators for next period
  s_intensity_time_product_sum = 0;
  s_total_on_time_ms = 0;
  s_last_intensity_sample_ticks = rtc_get_ticks();

  mutex_unlock(s_mutex);
}
