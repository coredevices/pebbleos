/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "shell/normal/button_lock.h"

#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "kernel/events.h"
#include "kernel/ui/modals/modal_manager.h"

#include "clar.h"

// Stubs
///////////////////////////////////////////////////////////////////////////////
#include "stubs_logging.h"
#include "stubs_passert.h"

#include "fake_new_timer.h"

// Fakes
///////////////////////////////////////////////////////////////////////////////

static uint32_t s_pref_hold_ms;

uint32_t shell_prefs_get_button_lock_hold_ms(void) {
  return s_pref_hold_ms;
}

static bool s_watchface_running;

bool app_manager_is_watchface_running(void) {
  return s_watchface_running;
}

static int s_num_cancel_force_quit_calls;

void launcher_cancel_force_quit(void) {
  s_num_cancel_force_quit_calls++;
}

static int s_num_watchface_reset_calls;

void watchface_reset_click_manager(void) {
  s_num_watchface_reset_calls++;
}

static CallbackEventCallback s_kernel_cb;
static void *s_kernel_cb_data;

void launcher_task_add_callback(CallbackEventCallback callback, void *data) {
  s_kernel_cb = callback;
  s_kernel_cb_data = data;
}

static bool s_touch_enabled = true;
static bool s_touch_pref_enabled = true;

void touch_service_set_globally_enabled(bool enabled) {
  s_touch_enabled = enabled;
}

bool touch_is_globally_enabled(void) {
  return s_touch_pref_enabled;
}

static int s_num_short_pulses;
static int s_num_double_pulses;

void vibes_short_pulse(void) {
  s_num_short_pulses++;
}

void vibes_double_pulse(void) {
  s_num_double_pulses++;
}

static int s_num_dialogs_created;
static int s_num_dialogs_popped;
static SimpleDialog s_dialog_storage;
static DialogCallbacks s_dialog_callbacks;
static const char *s_last_dialog_text;

SimpleDialog *simple_dialog_create(const char *dialog_name) {
  s_num_dialogs_created++;
  return &s_dialog_storage;
}

Dialog *simple_dialog_get_dialog(SimpleDialog *simple_dialog) {
  return &simple_dialog->dialog;
}

void dialog_set_text(Dialog *dialog, const char *text) {
  s_last_dialog_text = text;
}

void dialog_set_timeout(Dialog *dialog, uint32_t timeout) {}

void dialog_set_callbacks(Dialog *dialog, const DialogCallbacks *callbacks,
                          void *callback_context) {
  s_dialog_callbacks = *callbacks;
}

void simple_dialog_push(SimpleDialog *simple_dialog, WindowStack *window_stack) {}

void dialog_pop(Dialog *dialog) {
  s_num_dialogs_popped++;
}

WindowStack *modal_manager_get_window_stack(ModalPriority priority) {
  return NULL;
}

const char *i18n_get(const char *string, const void *owner) {
  return string;
}

void i18n_free(const char *string, const void *owner) {}

// Helpers
///////////////////////////////////////////////////////////////////////////////

static bool prv_press(ButtonId id) {
  PebbleEvent e = {
      .type = PEBBLE_BUTTON_DOWN_EVENT,
      .button.button_id = id,
  };
  return button_lock_handle_button_event(&e);
}

static bool prv_release(ButtonId id) {
  PebbleEvent e = {
      .type = PEBBLE_BUTTON_UP_EVENT,
      .button.button_id = id,
  };
  return button_lock_handle_button_event(&e);
}

static void prv_invoke_kernel_cb(void) {
  cl_assert(s_kernel_cb != NULL);
  CallbackEventCallback cb = s_kernel_cb;
  s_kernel_cb = NULL;
  cb(s_kernel_cb_data);
}

//! Hold the combo, fire the hold timer and run the posted KernelMain callback.
static void prv_toggle_lock(void) {
  prv_press(BUTTON_ID_BACK);
  prv_press(BUTTON_ID_DOWN);
  stub_new_timer_invoke(1 /* num_to_invoke */);
  prv_invoke_kernel_cb();
  prv_release(BUTTON_ID_BACK);
  prv_release(BUTTON_ID_DOWN);
}

// Tests
///////////////////////////////////////////////////////////////////////////////

void test_button_lock__initialize(void) {
  static bool s_timer_created;
  if (!s_timer_created) {
    button_lock_init();
    s_timer_created = true;
  }

  s_pref_hold_ms = 2000;
  s_watchface_running = true;

  // Unwind state a previous test may have left behind.
  for (ButtonId id = 0; id < NUM_BUTTONS; id++) {
    prv_release(id);
  }
  if (button_lock_is_locked()) {
    prv_toggle_lock();
  }

  s_num_cancel_force_quit_calls = 0;
  s_num_watchface_reset_calls = 0;
  s_kernel_cb = NULL;
  s_touch_enabled = true;
  s_touch_pref_enabled = true;
  s_num_short_pulses = 0;
  s_num_double_pulses = 0;
  s_num_dialogs_created = 0;
  s_num_dialogs_popped = 0;
  s_last_dialog_text = NULL;
}

void test_button_lock__cleanup(void) {}

void test_button_lock__pref_disabled_is_inert(void) {
  s_pref_hold_ms = 0;
  const int num_timer_starts_before = s_num_new_timer_start_calls;

  cl_assert(!prv_press(BUTTON_ID_BACK));
  cl_assert(!prv_press(BUTTON_ID_DOWN));
  cl_assert_equal_i(s_num_new_timer_start_calls, num_timer_starts_before);
  cl_assert(!prv_release(BUTTON_ID_BACK));
  cl_assert(!prv_release(BUTTON_ID_DOWN));
  cl_assert(!button_lock_is_locked());
}

void test_button_lock__lock_engages_after_hold(void) {
  cl_assert(!prv_press(BUTTON_ID_BACK));
  cl_assert(prv_press(BUTTON_ID_DOWN));

  cl_assert_equal_i(s_num_cancel_force_quit_calls, 1);
  cl_assert_equal_i(s_num_watchface_reset_calls, 1);
  TimerID timer = stub_new_timer_get_next();
  cl_assert(stub_new_timer_is_scheduled(timer));
  cl_assert_equal_i(stub_new_timer_timeout(timer), 2000);

  stub_new_timer_invoke(1);
  prv_invoke_kernel_cb();

  cl_assert(button_lock_is_locked());
  cl_assert_equal_i(s_num_short_pulses, 1);
  cl_assert(!s_touch_enabled);
  cl_assert_equal_s(s_last_dialog_text, "Buttons Locked");

  // The first button's DOWN was delivered, so its UP must be too.
  cl_assert(!prv_release(BUTTON_ID_BACK));
  cl_assert(prv_release(BUTTON_ID_DOWN));
}

void test_button_lock__configurable_hold_duration(void) {
  s_pref_hold_ms = 5000;

  prv_press(BUTTON_ID_BACK);
  prv_press(BUTTON_ID_DOWN);
  cl_assert_equal_i(stub_new_timer_timeout(stub_new_timer_get_next()), 5000);
  prv_release(BUTTON_ID_BACK);
  prv_release(BUTTON_ID_DOWN);
}

void test_button_lock__release_before_timeout_aborts(void) {
  prv_press(BUTTON_ID_BACK);
  prv_press(BUTTON_ID_DOWN);
  TimerID timer = stub_new_timer_get_next();

  cl_assert(!prv_release(BUTTON_ID_BACK));
  cl_assert(!stub_new_timer_is_scheduled(timer));
  // The second button's DOWN was swallowed, so its UP must be too.
  cl_assert(prv_release(BUTTON_ID_DOWN));
  cl_assert(!button_lock_is_locked());
}

void test_button_lock__third_button_cancels_pending(void) {
  prv_press(BUTTON_ID_BACK);
  prv_press(BUTTON_ID_DOWN);
  TimerID timer = stub_new_timer_get_next();

  cl_assert(!prv_press(BUTTON_ID_SELECT));
  cl_assert(!stub_new_timer_is_scheduled(timer));

  prv_release(BUTTON_ID_SELECT);
  prv_release(BUTTON_ID_BACK);
  prv_release(BUTTON_ID_DOWN);
  cl_assert(!button_lock_is_locked());
}

void test_button_lock__locked_swallows_input_and_hints(void) {
  prv_toggle_lock();
  cl_assert(button_lock_is_locked());

  s_num_dialogs_created = 0;
  cl_assert(prv_press(BUTTON_ID_SELECT));
  cl_assert(prv_release(BUTTON_ID_SELECT));
  cl_assert_equal_i(s_num_dialogs_created, 1);
  cl_assert_equal_s(s_last_dialog_text, "Hold Back + Down to unlock");

  // Hint popup is not re-created while still on screen.
  cl_assert(prv_press(BUTTON_ID_UP));
  cl_assert(prv_release(BUTTON_ID_UP));
  cl_assert_equal_i(s_num_dialogs_created, 1);

  // Once it unloaded, another press shows it again.
  s_dialog_callbacks.unload(NULL);
  cl_assert(prv_press(BUTTON_ID_UP));
  cl_assert(prv_release(BUTTON_ID_UP));
  cl_assert_equal_i(s_num_dialogs_created, 2);
}

void test_button_lock__unlock_restores_touch_pref(void) {
  prv_toggle_lock();
  cl_assert(button_lock_is_locked());
  cl_assert(!s_touch_enabled);

  s_touch_pref_enabled = false;
  prv_toggle_lock();
  cl_assert(!button_lock_is_locked());
  cl_assert_equal_i(s_num_double_pulses, 1);
  // Touch comes back to the persisted pref, not blindly on.
  cl_assert(!s_touch_enabled);

  prv_toggle_lock();
  s_touch_pref_enabled = true;
  prv_toggle_lock();
  cl_assert(s_touch_enabled);
}

void test_button_lock__continuous_hold_toggles_once(void) {
  prv_press(BUTTON_ID_BACK);
  prv_press(BUTTON_ID_DOWN);
  stub_new_timer_invoke(1);
  prv_invoke_kernel_cb();
  cl_assert(button_lock_is_locked());

  // Still holding: the timer must not be re-armed.
  cl_assert(!stub_new_timer_is_scheduled(stub_new_timer_get_next()));

  prv_release(BUTTON_ID_BACK);
  prv_release(BUTTON_ID_DOWN);
  cl_assert(button_lock_is_locked());
}

void test_button_lock__no_watchface_reset_in_app(void) {
  s_watchface_running = false;

  prv_press(BUTTON_ID_BACK);
  prv_press(BUTTON_ID_DOWN);
  cl_assert_equal_i(s_num_watchface_reset_calls, 0);
  cl_assert_equal_i(s_num_cancel_force_quit_calls, 1);
  prv_release(BUTTON_ID_BACK);
  prv_release(BUTTON_ID_DOWN);
}

void test_button_lock__timer_fire_after_release_race(void) {
  prv_press(BUTTON_ID_BACK);
  prv_press(BUTTON_ID_DOWN);

  // Timer fires, but the combo is released before KernelMain runs the
  // posted callback: the toggle must not happen.
  stub_new_timer_invoke(1);
  prv_release(BUTTON_ID_BACK);
  prv_release(BUTTON_ID_DOWN);
  prv_invoke_kernel_cb();

  cl_assert(!button_lock_is_locked());
}

void test_button_lock__unlock_while_hint_visible_pops_it(void) {
  prv_toggle_lock();

  s_num_dialogs_created = 0;
  prv_press(BUTTON_ID_SELECT);
  prv_release(BUTTON_ID_SELECT);
  cl_assert_equal_i(s_num_dialogs_created, 1);

  s_num_dialogs_popped = 0;
  prv_toggle_lock();
  cl_assert(!button_lock_is_locked());
  cl_assert_equal_i(s_num_dialogs_popped, 1);
  cl_assert_equal_s(s_last_dialog_text, "Buttons Unlocked");
}
