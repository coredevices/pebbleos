/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "kernel/ui/modals/modal_manager.h"
#include "pbl/services/touch/touch_session.h"

#include <stdbool.h>

#include "fake_rtc.h"

// Stubs
#include "stubs_logging.h"

// The session's environmental overrides; each test drives them directly.
static bool s_watchface_running;
bool app_manager_is_watchface_running(void) {
  return s_watchface_running;
}

static bool s_modal_enabled;
static ModalProperty s_modal_properties;
bool modal_manager_get_enabled(void) {
  return s_modal_enabled;
}
ModalProperty modal_manager_get_properties(void) {
  return s_modal_properties;
}

static bool s_light_on;
bool light_is_on(void) {
  return s_light_on;
}

// The session deadline follows the backlight timeout pref, sampled per arm.
static uint32_t s_backlight_timeout_ms;
uint32_t backlight_get_timeout_ms(void) {
  return s_backlight_timeout_ms;
}

#define MS_TO_TICKS(ms) (((ms) * (RtcTicks)RTC_TICKS_HZ) / 1000)
#define TIMEOUT_TICKS MS_TO_TICKS(s_backlight_timeout_ms)

// setup and teardown
void test_touch_session__initialize(void) {
  fake_rtc_init(1000 /* initial_ticks */, 0 /* initial_time */);
  // Guarded posture: idle watchface, no modal, backlight off.
  s_watchface_running = true;
  s_modal_enabled = false;
  s_modal_properties = ModalPropertyDefault;
  s_light_on = false;
  s_backlight_timeout_ms = 3000;
  touch_session_reset();
}

void test_touch_session__cleanup(void) {
}

// tests
void test_touch_session__inactive_by_default(void) {
  cl_assert(!touch_session_is_active());
}

void test_touch_session__arm_activates_until_deadline(void) {
  touch_session_arm(TouchSessionArmSource_WakeGesture);
  cl_assert(touch_session_is_active());

  fake_rtc_increment_ticks(TIMEOUT_TICKS - 1);
  cl_assert(touch_session_is_active());

  fake_rtc_increment_ticks(1);
  cl_assert(!touch_session_is_active());
}

void test_touch_session__extend_pushes_deadline(void) {
  touch_session_arm(TouchSessionArmSource_Button);

  fake_rtc_increment_ticks(TIMEOUT_TICKS - 1);
  touch_session_extend();

  // Past the original deadline, still inside the extended one.
  fake_rtc_increment_ticks(TIMEOUT_TICKS - 1);
  cl_assert(touch_session_is_active());

  fake_rtc_increment_ticks(1);
  cl_assert(!touch_session_is_active());
}

void test_touch_session__extend_while_inactive_is_a_noop(void) {
  // Gated contact must not keep the session alive, let alone open it.
  touch_session_extend();
  cl_assert(!touch_session_is_active());

  // Same after an armed session has expired.
  touch_session_arm(TouchSessionArmSource_WakeGesture);
  fake_rtc_increment_ticks(TIMEOUT_TICKS);
  touch_session_extend();
  cl_assert(!touch_session_is_active());
}

void test_touch_session__non_watchface_is_active(void) {
  s_watchface_running = false;
  cl_assert(touch_session_is_active());
}

void test_touch_session__focused_modal_is_active(void) {
  s_modal_enabled = true;
  s_modal_properties = 0;  // focused
  cl_assert(touch_session_is_active());
}

void test_touch_session__unfocused_modal_stays_guarded(void) {
  s_modal_enabled = true;
  s_modal_properties = ModalProperty_Unfocused;
  cl_assert(!touch_session_is_active());
}

void test_touch_session__disabled_modal_stays_guarded(void) {
  // Properties say focused, but the modal stack is disabled.
  s_modal_enabled = false;
  s_modal_properties = 0;
  cl_assert(!touch_session_is_active());
}

void test_touch_session__lit_backlight_is_active(void) {
  s_light_on = true;
  cl_assert(touch_session_is_active());
}

void test_touch_session__extend_in_menu_arms_deadline(void) {
  // Touching inside a menu (non-watchface => active) arms the deadline, so
  // touch stays live briefly after backing out to the watchface.
  s_watchface_running = false;
  touch_session_extend();

  s_watchface_running = true;
  cl_assert(touch_session_is_active());

  fake_rtc_increment_ticks(TIMEOUT_TICKS);
  cl_assert(!touch_session_is_active());
}

void test_touch_session__reset_closes_deadline(void) {
  touch_session_arm(TouchSessionArmSource_WakeGesture);
  cl_assert(touch_session_is_active());

  touch_session_reset();
  cl_assert(!touch_session_is_active());
}

void test_touch_session__deadline_follows_backlight_timeout_pref(void) {
  // The pref is sampled when the session is armed, not cached at init.
  s_backlight_timeout_ms = 10000;
  touch_session_arm(TouchSessionArmSource_WakeGesture);

  fake_rtc_increment_ticks(MS_TO_TICKS(10000) - 1);
  cl_assert(touch_session_is_active());
  fake_rtc_increment_ticks(1);
  cl_assert(!touch_session_is_active());

  // A shortened pref applies from the next arm.
  s_backlight_timeout_ms = 1000;
  touch_session_arm(TouchSessionArmSource_Button);
  fake_rtc_increment_ticks(MS_TO_TICKS(1000));
  cl_assert(!touch_session_is_active());
}

void test_touch_session__reset_leaves_overrides(void) {
  touch_session_arm(TouchSessionArmSource_Button);
  touch_session_reset();

  // Environmental overrides are not the deadline's business.
  s_watchface_running = false;
  cl_assert(touch_session_is_active());
}
