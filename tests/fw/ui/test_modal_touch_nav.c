/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// Kernel/modal touch-context wiring: as modals gain/lose focus (and across modal<->app
// transitions) the kernel recognizer manager follows the focus, and the kernel touch dispatcher is
// gated on a focused modal so app gestures never tick the kernel twin.

#include "applib/ui/app_window_stack.h"
#include "applib/ui/window.h"
#include "applib/ui/window_manager.h"
#include "applib/ui/window_private.h"
#include "applib/ui/window_stack.h"
#include "applib/ui/window_stack_private.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/touch_nav.h"
#include "applib/touch_service_private.h"
#include "kernel/ui/modals/modal_manager.h"
#include "pbl/services/touch/touch_event.h"

#include "applib/connection_service_private.h"
#include "applib/battery_state_service_private.h"
#include "applib/tick_timer_service_private.h"

#include "clar.h"

// Stubs
////////////////////////////////////

#include "stubs_sem.h"
#include "stubs_accel_service.h"
#include "stubs_app_state.h"
#include "stubs_app_timer.h"
#include "stubs_ble_app_support.h"
#include "stubs_event_service_client.h"
#include "stubs_fonts.h"
#include "stubs_freertos.h"
#include "stubs_gbitmap.h"
#include "stubs_graphics.h"
#include "stubs_graphics_context.h"
#include "stubs_heap.h"
#include "stubs_logging.h"
#include "stubs_new_timer.h"
#include "stubs_passert.h"
#include "stubs_persist.h"
#include "stubs_plugin_service.h"
#include "stubs_print.h"
#include "stubs_process_manager.h"
#include "stubs_prompt.h"
#include "stubs_queue.h"
#include "stubs_resources.h"
#include "stubs_syscalls.h"
#include "stubs_unobstructed_area.h"

// Fakes
////////////////////////////////////

#include "fake_events.h"
#include "fake_pbl_malloc.h"
#include "fake_pebble_tasks.h"
#include "fake_animation.h"
#include "fake_rtc.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Static Variables
////////////////////////////////////

static Window *s_last_click_configured_window;
static bool s_app_idle = false;
static bool s_nav_enabled = true;

// Captured kernel touch-slot handler (the gated modal dispatcher subscribed via
// modal_touch_nav_subscribe).
static TouchServiceHandler s_kernel_handler;
static void *s_kernel_ctx;

// The app twin of the recognizer manager; window_get_recognizer_manager routes app windows here.
static RecognizerManager s_app_recognizer_manager;

// Overrides
////////////////////////////////////

void battery_state_service_state_init(BatteryStateServiceState *state) {}
void connection_service_state_init(ConnectionServiceState *state) {}
void tick_timer_service_state_init(TickTimerServiceState *state) {}
void framebuffer_clear(FrameBuffer *f) {}

void launcher_task_add_callback(void (*callback)(void *data), void *data) {
  callback(data);
}

void app_idle_timeout_pause(void) { s_app_idle = true; }
void app_idle_timeout_resume(void) { s_app_idle = false; }
void app_idle_timeout_refresh(void) {}

bool app_install_id_from_app_db(AppInstallId id) { return false; }
void framebuffer_dirty_all(FrameBuffer *f) {}
void framebuffer_mark_dirty_rect(FrameBuffer *f, GRect rect) {}
bool layer_is_status_bar_layer(Layer *layer) { return false; }
void status_bar_layer_render(GContext *ctx, const GRect *bounds, void *config) {}

GDrawState graphics_context_get_drawing_state(GContext *ctx) {
  GDrawState state;
  memset(&state, 0, sizeof(GDrawState));
  return state;
}
void graphics_context_set_drawing_state(GContext *ctx, GDrawState draw_state) {}

bool compositor_is_animating(void) { return false; }
void *compositor_modal_transition_to_modal_get(bool dest) { return NULL; }
void compositor_modal_render_ready(void) {}
void compositor_transition_cancel(void) {}
void compositor_transition(const CompositorTransition *type) {}

bool sys_app_is_watchface(void) { return false; }

void click_manager_init(ClickManager *click_manager) {}
void click_manager_clear(ClickManager *click_manager) {}
void click_manager_reset(ClickManager *click_manager) {}
void watchface_reset_click_manager(void) {}

Animation *window_transition_default_pop_create_animation(WindowTransitioningContext *context) {
  window_transition_context_disappear(context);
  window_transition_context_appear(context);
  return animation_create();
}
const WindowTransitionImplementation window_transition_default_pop_implementation = {
  .create_animation = window_transition_default_pop_create_animation,
};
const WindowTransitionImplementation *window_transition_get_default_pop_implementation(void) {
  return &window_transition_default_pop_implementation;
}

Animation *window_transition_default_push_create_animation(WindowTransitioningContext *context) {
  window_transition_context_disappear(context);
  window_transition_context_appear(context);
  return animation_create();
}
const WindowTransitionImplementation window_transition_default_push_implementation = {
  .create_animation = window_transition_default_push_create_animation,
};
const WindowTransitionImplementation *window_transition_get_default_push_implementation(void) {
  return &window_transition_default_push_implementation;
}

Animation *window_transition_none_create_animation(WindowTransitioningContext *context) {
  window_transition_context_disappear(context);
  window_transition_context_appear(context);
  return animation_create();
}
const WindowTransitionImplementation g_window_transition_none_implementation = {
  .create_animation = window_transition_none_create_animation,
};

void app_click_config_setup_with_window(ClickManager *click_manager, struct Window *window) {
  s_last_click_configured_window = window;
}

// Touch-nav collaborators for the kernel twin.
bool sys_touch_nav_enabled(void) { return s_nav_enabled; }
bool sys_touch_app_nav_active(void) { return false; }

void touch_service_set_system_handler(TouchServiceHandler handler, void *context) {
  s_kernel_handler = handler;
  s_kernel_ctx = context;
}

RecognizerManager *app_state_get_recognizer_manager(void) {
  return &s_app_recognizer_manager;
}

// Helpers
////////////////////////////////////

static void prv_unload_destroy(Window *window) {
  window_destroy(window);
}

static Window *prv_make_window(void) {
  Window *window = window_create();
  window_set_window_handlers(window, &(WindowHandlers){ .unload = prv_unload_destroy });
  return window;
}

// Drive the captured kernel touch-slot handler with a single Touchdown. A non_navigational
// Touchdown is used so the observable is the cheap `gated` counter (routing is not exercised): when
// the gate lets the dispatcher run, the counter ticks; when it gates out, it does not.
static void prv_kernel_touchdown(void) {
  const TouchEvent e = {
    .type = TouchEvent_Touchdown,
    .x = 50,
    .y = 90,
    .non_navigational = true,
  };
  cl_assert(s_kernel_handler != NULL);
  s_kernel_handler(&e, s_kernel_ctx);
}

static RecognizerManager *prv_kernel_manager(void) {
  return modal_manager_get_recognizer_manager();
}

// Setup and Teardown
////////////////////////////////////

void test_modal_touch_nav__initialize(void) {
  fake_rtc_init(0, 0);
  s_last_click_configured_window = NULL;
  s_app_idle = false;
  s_nav_enabled = true;
  s_kernel_handler = NULL;
  s_kernel_ctx = NULL;
  recognizer_manager_init(&s_app_recognizer_manager);

  WindowStack *stack = app_state_get_window_stack();
  *stack = (WindowStack){};

  modal_manager_reset();

  stub_pebble_tasks_set_current(PebbleTask_KernelMain);
}

void test_modal_touch_nav__cleanup(void) {
  stub_pebble_tasks_set_current(PebbleTask_App);
  app_window_stack_pop_all(false);

  stub_pebble_tasks_set_current(PebbleTask_KernelMain);
  modal_manager_pop_all();
  modal_manager_event_loop_upkeep();

  fake_animation_cleanup();

  cl_assert_equal_i(fake_pbl_malloc_num_net_allocs(), 0);
}

// Tests
////////////////////////////////////

// Criterion 1: a focused modal's gestures reach the kernel (modal) manager, never the app manager.
// The two managers are distinct instances, and window routing keeps app vs modal windows apart.
void test_modal_touch_nav__app_and_modal_managers_are_separate(void) {
  cl_assert(app_state_get_recognizer_manager() != modal_manager_get_recognizer_manager());

  Window *app_window = prv_make_window();
  stub_pebble_tasks_set_current(PebbleTask_App);
  window_stack_push(app_state_get_window_stack(), app_window, false);
  cl_assert_equal_p(window_get_recognizer_manager(app_window),
                    app_state_get_recognizer_manager());

  stub_pebble_tasks_set_current(PebbleTask_KernelMain);
  Window *modal_window = prv_make_window();
  window_stack_push(modal_manager_get_window_stack(ModalPriorityGeneric), modal_window, false);
  cl_assert_equal_p(window_get_recognizer_manager(modal_window),
                    modal_manager_get_recognizer_manager());
}

// Criterion 5: without a focused modal the kernel dispatcher does not run its pipeline (kernel
// counters do not grow on app gestures); with a focused modal it does.
void test_modal_touch_nav__gate_requires_focused_modal(void) {
  modal_touch_nav_subscribe();
  TouchNavState *state = modal_manager_get_touch_nav_state();

  // No modal on screen -> not focused -> the kernel dispatcher is gated out.
  prv_kernel_touchdown();
  cl_assert_equal_i(state->counters.gated, 0);

  // A focusable modal is now focused -> the dispatcher runs.
  Window *modal_window = prv_make_window();
  window_stack_push(modal_manager_get_window_stack(ModalPriorityGeneric), modal_window, false);
  modal_manager_event_loop_upkeep();
  prv_kernel_touchdown();
  cl_assert_equal_i(state->counters.gated, 1);
}

// Criterion 2: a modal pushed over another modal migrates the kernel manager to the top modal, and
// closing it returns focus (and the manager) to the lower modal. The return path is driven purely
// by the modal-manager upkeep since the lower modal's own window stack sees no transition.
void test_modal_touch_nav__migrates_over_modal_and_back(void) {
  RecognizerManager *manager = prv_kernel_manager();
  WindowStack *lower = modal_manager_get_window_stack(ModalPriorityGeneric);
  WindowStack *upper = modal_manager_get_window_stack(ModalPriorityAlert);

  Window *low_window = prv_make_window();
  Window *high_window = prv_make_window();

  window_stack_push(lower, low_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, low_window);

  window_stack_push(upper, high_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, high_window);

  // Close the top modal: focus (and the manager) returns to the lower modal.
  window_stack_remove(high_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, low_window);
}

static void prv_noop_click_config(void *context) {}

// Same migration, but with a click config provider on both modals so is_click_configured == 1,
// matching real modals. The loss/gain focus-transition branches key off that flag, so this variant
// actually exercises the cross-stack coordination the plain test above cannot reach.
void test_modal_touch_nav__migrates_over_modal_and_back_configured(void) {
  RecognizerManager *manager = prv_kernel_manager();
  WindowStack *lower = modal_manager_get_window_stack(ModalPriorityGeneric);
  WindowStack *upper = modal_manager_get_window_stack(ModalPriorityAlert);

  Window *low_window = prv_make_window();
  Window *high_window = prv_make_window();
  window_set_click_config_provider(low_window, prv_noop_click_config);
  window_set_click_config_provider(high_window, prv_noop_click_config);

  window_stack_push(lower, low_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, low_window);

  window_stack_push(upper, high_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, high_window);

  window_stack_remove(high_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, low_window);
}

// Criterion 4: after the last focusable modal leaves, the kernel manager is unbound and reset.
void test_modal_touch_nav__reset_after_last_modal(void) {
  RecognizerManager *manager = prv_kernel_manager();
  WindowStack *stack = modal_manager_get_window_stack(ModalPriorityGeneric);

  Window *modal_window = prv_make_window();
  window_stack_push(stack, modal_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, modal_window);

  // Dirty the manager to prove the modal->app transition actually resets it.
  manager->state = RecognizerManagerState_RecognizersActive;

  window_stack_remove(modal_window, false);
  modal_manager_event_loop_upkeep();
  cl_assert_equal_p(manager->window, NULL);
  cl_assert_equal_i(manager->state, RecognizerManagerState_WaitForTouchdown);
}

// Criterion 3: an unfocusable modal (e.g. Timeline Peek) never binds the kernel manager and never
// opens the dispatcher gate, so it cannot steal touch.
void test_modal_touch_nav__unfocusable_does_not_grab(void) {
  RecognizerManager *manager = prv_kernel_manager();
  TouchNavState *state = modal_manager_get_touch_nav_state();
  modal_touch_nav_subscribe();

  Window *peek = prv_make_window();
  window_set_focusable(peek, false);
  window_stack_push(modal_manager_get_window_stack(ModalPriorityGeneric), peek, false);
  modal_manager_event_loop_upkeep();

  // The unfocusable modal did not bind the manager...
  cl_assert_equal_p(manager->window, NULL);

  // ...and the dispatcher stays gated (no focused modal), so a gesture ticks nothing.
  prv_kernel_touchdown();
  cl_assert_equal_i(state->counters.gated, 0);
}
