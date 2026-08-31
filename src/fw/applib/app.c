/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app.h"

#include "applib/ui/app_window_stack.h"
#include "applib/ui/window_stack.h"
#include "applib/ui/window_private.h"
#include "pbl/mcu/fpu.h"
#include "process_state/app_state/app_state.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/profiler.h"

static void prv_render_app(void) {
  WindowStack *stack = app_state_get_window_stack();
  GContext *ctx = app_state_get_graphics_context();
  if (!window_stack_is_animating(stack)) {
    SYS_PROFILER_NODE_START(render_app);
    window_render(app_window_stack_get_top_window(), ctx);
    SYS_PROFILER_NODE_STOP(render_app);
  } else {
    // TODO: PBL-17645 render container layer instead of the two windows
    WindowTransitioningContext *transition_context = &stack->transition_context;

    if (transition_context->implementation->render) {
      transition_context->implementation->render(transition_context, ctx);
    }
  }

  *app_state_get_framebuffer_render_pending() = true;

  PebbleEvent event = {
    .type = PEBBLE_RENDER_READY_EVENT,
  };
  sys_send_pebble_event_to_kernel(&event);
}

static bool prv_window_is_render_scheduled(Window *window) {
  return window && window->is_render_scheduled;
}

static bool prv_app_is_render_scheduled() {
  Window *top_window = app_window_stack_get_top_window();
  if (prv_window_is_render_scheduled(top_window)) {
    return true;
  }

  WindowStack *stack = app_state_get_window_stack();
  if (!window_stack_is_animating(stack)) {
    return false;
  }

  WindowTransitioningContext *transition_ctx = &stack->transition_context;
  return prv_window_is_render_scheduled(transition_ctx->window_from) ||
         prv_window_is_render_scheduled(transition_ctx->window_to);
}

void app_request_render(void) {
  Window *window = app_window_stack_get_top_window();
  if (window) {
    window_schedule_render(window);
  }
}

//! Tasks that have to be done in between each event.
static NOINLINE void event_loop_upkeep(void) {

  // Check to see if the most recent event caused us to pop our final window. If that's the case, we need to
  // kill ourselves.
  if (app_window_stack_count() == 0) {
    PBL_LOG_DBG("No more windows, killing current app");

    PebbleEvent event = { .type = PEBBLE_PROCESS_KILL_EVENT, .kill = { .gracefully = true, .task=PebbleTask_App } };
    sys_send_pebble_event_to_kernel(&event);
    return;
  }

  // Check to see if handling the previous event requires us to rerender ourselves.
  if (prv_app_is_render_scheduled() &&
      !*app_state_get_framebuffer_render_pending()) {
    prv_render_app();
  }
}

static void prv_app_will_focus_handler(PebbleEvent *e, void *context) {
  Window *window = app_window_stack_get_top_window();
  click_manager_reset(app_state_get_click_manager());
  if (e->app_focus.in_focus) {
    if (window) {
      window_became_input_focus(window);
      // Do not call 'appear' handler on window displacing modal window
      window_set_on_screen(window, true, false);
      window_render(window, app_state_get_graphics_context());
    }
  } else if (window) {
    window_lost_input_focus(window);
    // Do not call 'disappear' handler on window displaced by modal window
    window_set_on_screen(window, false, false);
  }
}

static void prv_app_button_down_handler(PebbleEvent *e, void *context) {
  WindowStack *app_window_stack = app_state_get_window_stack();
  if (window_stack_is_animating(app_window_stack)) {
    return;
  }

  if (e->button.button_id == BUTTON_ID_BACK &&
      !app_window_stack_get_top_window()->overrides_back_button) {
    // a transition of NULL means we will use the stored pop transition for this stack item
    window_stack_pop_with_transition(app_window_stack, NULL /* transition */);
    return;
  }

  click_recognizer_handle_button_down(
      &app_state_get_click_manager()->recognizers[e->button.button_id]);
}

static void prv_app_button_up_handler(PebbleEvent *e, void *context) {
  if (window_stack_is_animating(app_state_get_window_stack())) {
    return;
  }

  click_recognizer_handle_button_up(
      &app_state_get_click_manager()->recognizers[e->button.button_id]);
}

// this handler is called via the legacy2_status_bar_change_event and
// will update the status bar once a minute for non-fullscreen legacy2 apps
static void prv_legacy2_status_bar_handler(PebbleEvent *e, void *context) {
  Window *window = app_window_stack_get_top_window();
  // only force render if we're not fullscreen
  if (!window->is_fullscreen) {
    // a little logic to only force update when the minute changes
    ApplibInternalEventsInfo *events_info =
        app_state_get_applib_internal_events_info();
    struct tm currtime;
    sys_localtime_r(&e->clock_tick.tick_time, &currtime);
    const int minute_of_day = (currtime.tm_hour * 60) + currtime.tm_min;
    if (events_info->minute_of_last_legacy2_statusbar_change != minute_of_day) {
      events_info->minute_of_last_legacy2_statusbar_change = minute_of_day;
      window_schedule_render(window);
    }
  }
}

static void prv_legacy2_status_bar_timer_subscribe(void) {
  // we only need this tick event if we are a legacy2 app
  if (process_manager_compiled_with_legacy2_sdk()) {
    ApplibInternalEventsInfo *events_info =
        app_state_get_applib_internal_events_info();
    // Initialize the state for the status bar handler.
    events_info->minute_of_last_legacy2_statusbar_change = -1;
    events_info->legacy2_status_bar_change_event = (EventServiceInfo) {
      .type = PEBBLE_TICK_EVENT,
      .handler = prv_legacy2_status_bar_handler,
    };
    event_service_client_subscribe(
        &events_info->legacy2_status_bar_change_event);
  }
  // NOTE: We could be super fancy and register and unregister when the fullscreen
  // status changes, but it's probably not worth it as we'll be waking up once a
  // minute anyway to update the face itself and it will happen as part of the same interval
}

static void prv_legacy2_status_bar_timer_unsubscribe(void) {
  // we should only unsubscribe if we subscribed in the first place
  if (process_manager_compiled_with_legacy2_sdk()) {
    ApplibInternalEventsInfo *events_info =
        app_state_get_applib_internal_events_info();
    event_service_client_unsubscribe(
        &events_info->legacy2_status_bar_change_event);
  }
}

static void prv_app_callback_handler(PebbleEvent *e) {
  e->callback.callback(e->callback.data);
}

static NOINLINE void prv_handle_deinit_event(void) {
  ApplibInternalEventsInfo *events_info =
      app_state_get_applib_internal_events_info();
  event_service_client_unsubscribe(&events_info->will_focus_event);
  event_service_client_unsubscribe(&events_info->button_down_event);
  event_service_client_unsubscribe(&events_info->button_up_event);
  prv_legacy2_status_bar_timer_unsubscribe(); // a no-op on sdk3+ applications
  WindowStack *app_window_stack = app_state_get_window_stack();
  window_stack_lock_push(app_window_stack);
  window_stack_pop_all(app_window_stack, false);
  window_stack_unlock_push(app_window_stack);
}

// Get the app_id for the current app or worker
// @return INSTALL_ID_INVALID if unsuccessful
AppInstallId app_get_app_id(void) {
  // Only support from app or workers
  PebbleTask task = pebble_task_get_current();
  if ((task != PebbleTask_App) && (task != PebbleTask_Worker)) {
    APP_LOG(APP_LOG_LEVEL_ERROR, "Only supported from app or worker tasks");
    return INSTALL_ID_INVALID;
  }

  // Get the app id
  if (task == PebbleTask_App) {
    return sys_app_manager_get_current_app_id();
  } else {
    return sys_worker_manager_get_current_worker_id();
  }
}

void app_event_loop_common(void) {
  // Register our event handlers before we do anything else. Registering for an event requires
  // an event being sent to the kernel and therefore should be done before any other events are
  // generated by us to ensure we don't miss out on anything.
  ApplibInternalEventsInfo *events_info =
      app_state_get_applib_internal_events_info();
  events_info->will_focus_event = (EventServiceInfo) {
    .type = PEBBLE_APP_WILL_CHANGE_FOCUS_EVENT,
    .handler = prv_app_will_focus_handler,
  };
  events_info->button_down_event = (EventServiceInfo) {
    .type = PEBBLE_BUTTON_DOWN_EVENT,
    .handler = prv_app_button_down_handler,
  };
  events_info->button_up_event = (EventServiceInfo) {
    .type = PEBBLE_BUTTON_UP_EVENT,
    .handler = prv_app_button_up_handler,
  };
  event_service_client_subscribe(&events_info->will_focus_event);
  event_service_client_subscribe(&events_info->button_down_event);
  event_service_client_subscribe(&events_info->button_up_event);
  prv_legacy2_status_bar_timer_subscribe(); // a no-op on sdk3+ applications

  event_loop_upkeep();

  // Event loop:
  while (1) {
    PebbleEvent event;

    sys_get_pebble_event(&event);

    if (event.type == PEBBLE_PROCESS_DEINIT_EVENT) {
      prv_handle_deinit_event();
      // We're done here. Return the app's main function.
      event_cleanup(&event);
      return;
    } else if (event.type == PEBBLE_CALLBACK_EVENT) {
      prv_app_callback_handler(&event);
    } else if (event.type == PEBBLE_RENDER_REQUEST_EVENT) {
      app_request_render();
    } else if (event.type == PEBBLE_RENDER_FINISHED_EVENT) {
      *app_state_get_framebuffer_render_pending() = false;
    } else {
      event_service_client_handle_event(&event);
    }

    mcu_fpu_cleanup();
    event_cleanup(&event);

    event_loop_upkeep();
  }
}

void app_event_loop(void) {
  app_event_loop_common();
}
