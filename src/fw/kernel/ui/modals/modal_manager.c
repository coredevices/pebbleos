/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "modal_manager.h"

#include "applib/ui/app_window_click_glue.h"
#include "applib/ui/click_internal.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/touch_nav.h"
#include "applib/touch_service.h"
#include "applib/touch_service_private.h"
#include "applib/ui/window.h"
#include "applib/ui/window_private.h"
#include "applib/ui/window_stack.h"
#include "applib/ui/window_stack_animation.h"
#include "applib/ui/window_stack_private.h"
#include "console/prompt.h"
#include "kernel/events.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/compositor/compositor_transitions.h"
#include "shell/normal/app_idle_timeout.h"
#include "shell/normal/watchface.h"
#include "system/passert.h"
#include "system/profiler.h"

#include "FreeRTOS.h"
#include "semphr.h"

typedef struct ModalContext {
  WindowStack window_stack;
} ModalContext;

typedef struct UpdateContext {
  ModalPriority highest_idx;
  ModalProperty properties;
} UpdateContext;

static void prv_update_modal_stacks(UpdateContext *context);

// Static State
///////////////////
static ModalContext s_modal_window_stacks[NumModalPriorities];

static ClickManager s_modal_window_click_manager;

#ifdef CONFIG_TOUCH
// Kernel twin of the per-task touch-nav bridge state (the app twin lives in AppState).
static RecognizerList s_modal_recognizer_list;
static RecognizerManager s_modal_recognizer_manager;
static TouchNavState s_modal_touch_nav_state;
#endif

static ModalPriority s_modal_min_priority = ModalPriorityMin;

// Used to help us keep track various modal properties in aggregate, such as existence.
// Initialize the default to being equivalent to having no modals.
static ModalProperty s_current_modal_properties = ModalPropertyDefault;

// Used to decide the compositor transition after a modal is already removed from the stack
static ModalPriority s_last_highest_modal_priority = ModalPriorityInvalid;

// Private API
////////////////////
static bool prv_has_visible_window(ModalContext *context, void *unused) {
  const bool empty = (context->window_stack.list_head == NULL);
  const bool filtered_out = (context < &s_modal_window_stacks[s_modal_min_priority]);
  return (!empty && !filtered_out);
}

static bool prv_has_transition_window(ModalContext *context) {
  Window *window = window_stack_get_top_window(&context->window_stack);
  return (window && (context > &s_modal_window_stacks[ModalPriorityDiscreet]));
}

static bool prv_has_opaque_window(ModalContext *context) {
  Window *window = window_stack_get_top_window(&context->window_stack);
  return (window && !window->is_transparent);
}

static bool prv_has_focusable_window(ModalContext *context) {
  Window *window = window_stack_get_top_window(&context->window_stack);
  return (window && !window->is_unfocusable);
}

static bool prv_has_visible_focusable_window(ModalContext *context, void *unused) {
  return (prv_has_visible_window(context, NULL) && prv_has_focusable_window(context));
}

static void prv_send_will_focus_event(bool in_focus) {
  static bool s_focus_lost = true;
  if (s_focus_lost == in_focus) {
    return;
  }

  s_focus_lost = in_focus;

  PebbleEvent event = {
    .type = PEBBLE_APP_WILL_CHANGE_FOCUS_EVENT,
    .app_focus = {
      .in_focus = in_focus,
    }
  };
  event_put(&event);
}

#ifdef CONFIG_TOUCH
static Window *prv_get_visible_focused_window(void);

// Touch-nav bridge effects for the kernel (modal) task.
static bool prv_modal_touch_nav_is_animating(void *ctx) {
  Window *window = prv_get_visible_focused_window();
  return window && window_stack_is_animating(window->parent_window_stack);
}

static bool prv_modal_touch_nav_top_overrides_back(void *ctx) {
  Window *window = prv_get_visible_focused_window();
  return window && window->overrides_back_button;
}

static bool prv_modal_touch_nav_top_tap_requires_action_bar(void *ctx) {
  Window *window = prv_get_visible_focused_window();
  return window && window->touch_tap_requires_action_bar;
}

static bool prv_modal_touch_nav_top_bridge_disabled(void *ctx) {
  Window *window = prv_get_visible_focused_window();
  // No focused modal window: keep the modal twin inert. The button path is
  // likewise gated on a focused modal (and asserts a window), so report
  // bridge-disabled here -> route resolves to None -> the whole set fails and
  // nothing is emitted, instead of emitting into an unowned modal click manager.
  if (!window) {
    return true;
  }
  return window->touch_bridge_disabled;
}

static void prv_modal_touch_nav_pop_top(void *ctx) {
  Window *window = prv_get_visible_focused_window();
  if (window) {
    window_stack_remove(window, true /* animated */);
  }
}

static void prv_modal_touch_nav_emit_button(void *ctx, ButtonId button) {
  ClickManager *cm = modal_manager_get_click_manager();
  click_recognizer_handle_button_down(&cm->recognizers[button]);
  click_recognizer_handle_button_up(&cm->recognizers[button]);
}

static void prv_modal_touch_nav_idle_refresh(void *ctx) {
  app_idle_timeout_refresh();
}

static const TouchNavOps s_modal_touch_nav_ops = {
  .is_animating = prv_modal_touch_nav_is_animating,
  .top_overrides_back = prv_modal_touch_nav_top_overrides_back,
  .top_tap_requires_action_bar = prv_modal_touch_nav_top_tap_requires_action_bar,
  .top_bridge_disabled = prv_modal_touch_nav_top_bridge_disabled,
  .pop_top = prv_modal_touch_nav_pop_top,
  .emit_button = prv_modal_touch_nav_emit_button,
  .idle_refresh = prv_modal_touch_nav_idle_refresh,
};

RecognizerManager *modal_manager_get_recognizer_manager(void) {
  return &s_modal_recognizer_manager;
}

TouchNavState *modal_manager_get_touch_nav_state(void) {
  return &s_modal_touch_nav_state;
}

// The focused-modal predicate: enabled and at least one focusable modal on top. This is the same
// predicate the button path uses in the kernel event loop to decide whether to route input to the
// modal twin instead of the app.
static bool prv_modal_is_focused(void) {
  return modal_manager_get_enabled() &&
         !(modal_manager_get_properties() & ModalProperty_Unfocused);
}

// Point the kernel recognizer manager at \a window (NULL to unbind), dropping any in-flight gesture.
// Mirrors window_became_input_focus / window_lost_input_focus for the app manager. Used for the
// cross-stack modal focus changes that bypass the per-stack window transitions.
static void prv_modal_recognizer_focus(Window *window) {
  recognizer_manager_cancel_and_reset(&s_modal_recognizer_manager);
  recognizer_manager_set_window(&s_modal_recognizer_manager, window);
}

// Kernel touch-slot handler. Gate the nav pipeline on a focused modal: without one the app twin
// owns the gesture, and running the kernel manager too would double every counter/action for a
// single gesture (an unfocusable modal such as Timeline Peek never passes this gate either).
static void prv_modal_touch_nav_dispatch(const TouchEvent *touch_event, void *context) {
  if (!prv_modal_is_focused()) {
    return;
  }
  touch_nav_dispatch(touch_event, context);
}

void modal_touch_nav_subscribe(void) {
  // Runs on KernelMain. touch_service_set_system_handler routes the kernel touch slot to the gated
  // nav dispatcher for the modal twin.
  touch_service_set_system_handler(prv_modal_touch_nav_dispatch, &s_modal_touch_nav_state);
}

void modal_touch_nav_unsubscribe(void) {
  recognizer_manager_cancel_and_reset(&s_modal_recognizer_manager);
  touch_service_set_system_handler(NULL, NULL);
}
#endif

// Public API
////////////////////
void modal_manager_init(void) {
  // Don't touch s_modal_window_stacks or s_modal_min_priority, it's valid for someone to have
  // disabled modals using modal_manager_set_enabled before we've initialized and we should
  // honour that setting.

  click_manager_init(&s_modal_window_click_manager);

#ifdef CONFIG_TOUCH
  recognizer_list_init(&s_modal_recognizer_list);
  recognizer_manager_init(&s_modal_recognizer_manager);
  s_modal_recognizer_manager.global_list = &s_modal_recognizer_list;
  touch_nav_state_init(&s_modal_touch_nav_state, &s_modal_recognizer_manager,
                       &s_modal_touch_nav_ops);
#endif
}

void modal_manager_set_min_priority(ModalPriority priority) {
  s_modal_min_priority = priority;
  for (int i = 0; i < priority; i++) {
    window_stack_lock_push(&s_modal_window_stacks[i].window_stack);
  }
  for (int i = priority; i < NumModalPriorities; ++i) {
    window_stack_unlock_push(&s_modal_window_stacks[i].window_stack);
  }
}

bool modal_manager_get_enabled(void) {
  return s_modal_min_priority < ModalPriorityMax;
}

ClickManager *modal_manager_get_click_manager(void) {
  return &s_modal_window_click_manager;
}

static WindowStack *prv_find_window_stack(ModalContextFilterCallback callback, void *data) {
  for (ModalPriority idx = NumModalPriorities - 1; idx >= ModalPriorityMin; idx--) {
    if (callback(&s_modal_window_stacks[idx], data)) {
      return &s_modal_window_stacks[idx].window_stack;
    }
  }
  return NULL;
}

WindowStack *modal_manager_find_window_stack(ModalContextFilterCallback filter_cb, void *ctx) {
  return prv_find_window_stack(filter_cb, ctx);
}

WindowStack *modal_manager_get_window_stack(ModalPriority priority) {
  PBL_ASSERTN((priority > ModalPriorityInvalid) && (priority < NumModalPriorities));
  ModalContext *context = &s_modal_window_stacks[priority];
  return &context->window_stack;
}

Window *modal_manager_get_top_window(void) {
  WindowStack *stack = prv_find_window_stack(prv_has_visible_window, NULL);
  return window_stack_get_top_window(stack);
}

static void prv_pop_stacks_in_range(ModalPriority low, ModalPriority high) {
  // Discreet modals are transparent and unfocusable, they are not meant to be popped when
  // requesting opaque focusable modals to pop.
  for (ModalPriority priority = MAX(low, ModalPriorityDiscreet + 1); priority <= high;
       priority++) {
    ModalContext *m_context = &s_modal_window_stacks[priority];
    window_stack_pop_all(&m_context->window_stack, true /* animated */);
  }
}

void modal_manager_pop_all(void) {
  prv_pop_stacks_in_range(ModalPriorityMin, NumModalPriorities - 1);
}

void modal_manager_pop_all_below_priority(ModalPriority priority) {
  prv_pop_stacks_in_range(ModalPriorityMin, priority - 1);
}

static const CompositorTransition *prv_get_compositor_transition(bool modal_is_destination) {
  bool is_top_discreet;
  if (modal_is_destination) {
    Window *window =
        window_stack_get_top_window(&s_modal_window_stacks[ModalPriorityDiscreet].window_stack);
    is_top_discreet = (window && (window == modal_manager_get_top_window()));
  } else {
    is_top_discreet = (s_last_highest_modal_priority == ModalPriorityDiscreet);
  }
  return is_top_discreet ? NULL : compositor_modal_transition_to_modal_get(modal_is_destination);
}

static void prv_handle_app_to_modal_transition_visible(void) {
  // The last event resulted in a modal window being pushed where we didn't have any before.
  // Start the animation!
  compositor_transition(prv_get_compositor_transition(true /* modal_is_destination */));
}

static void prv_handle_modal_to_app_transition_visible(void) {
  compositor_transition(prv_get_compositor_transition(false /* modal_is_destination */));
}

static void prv_handle_app_to_modal_transition_hidden_and_unfocused(void) {
#if !defined(CONFIG_RECOVERY_FW) && !defined(CONFIG_SHELL_SDK)
  app_idle_timeout_pause();
#endif
}

static void prv_handle_modal_to_app_transition_hidden_and_unfocused(void) {
#if !defined(CONFIG_RECOVERY_FW) && !defined(CONFIG_SHELL_SDK)
  app_idle_timeout_resume();
#endif
}

static void prv_handle_app_to_modal_transition_focus(void) {
#if !defined(CONFIG_RECOVERY_FW) && !defined(CONFIG_SHELL_SDK)
  watchface_reset_click_manager();
#endif

  // Let the underlying window know it has lost focus if this is the first modal
  // window to show up.
  prv_send_will_focus_event(false /* in_focus */);
}

static void prv_handle_modal_to_app_transition_focus(void) {
  // There are no more modal windows, so we need to cleanup the modal window state.
  click_manager_clear(modal_manager_get_click_manager());

#ifdef CONFIG_TOUCH
  // The last focusable modal is gone; unbind the kernel recognizer manager so no gesture state or
  // active window survives into the app-focused period.
  prv_modal_recognizer_focus(NULL);
#endif

  prv_send_will_focus_event(true /* in_focus */);
}

void modal_manager_event_loop_upkeep(void) {
  if (!modal_manager_get_enabled()) {
    return;
  }

  UpdateContext update;
  prv_update_modal_stacks(&update);
  const ModalProperty last_properties = s_current_modal_properties;
  s_current_modal_properties = update.properties;

  const bool is_modal_transitionable = (update.properties & ModalProperty_CompositorTransitions);
  const bool was_modal_transitionable = (last_properties & ModalProperty_CompositorTransitions);
  if (!was_modal_transitionable && is_modal_transitionable) {
    // We now have a window visible when we didn't have one before, start the transition.
    prv_handle_app_to_modal_transition_visible();
  } else if (was_modal_transitionable && !is_modal_transitionable) {
    // This event resulted in our last visible modal window being popped, let's transition away.
    prv_handle_modal_to_app_transition_visible();
  }

  const bool is_modal_unfocused = (update.properties & ModalProperty_Unfocused);
  const bool was_modal_unfocused = (last_properties & ModalProperty_Unfocused);
  if (was_modal_unfocused && !is_modal_unfocused) {
    // We now have a modal window focused when we didn't have one before, start the transition.
    prv_handle_app_to_modal_transition_focus();
  } else if (!was_modal_unfocused && is_modal_unfocused) {
    // This event resulted in our last focusable modal window being popped, let's transition away.
    prv_handle_modal_to_app_transition_focus();
  }

  const bool is_app_hidden_and_unfocused =
      (!(update.properties & ModalProperty_Transparent) && !is_modal_unfocused);
  const bool was_app_hidden_and_unfocused =
      (!(last_properties & ModalProperty_Transparent) && !was_modal_unfocused);
  if (!was_app_hidden_and_unfocused && is_app_hidden_and_unfocused) {
    // The app is now obstructed by an opaque modal and lost focus to a modal, idle.
    prv_handle_app_to_modal_transition_hidden_and_unfocused();
  } else if (was_app_hidden_and_unfocused && !is_app_hidden_and_unfocused) {
    // The app now either is obstructed only by transparent modals or gained focus, resume.
    prv_handle_modal_to_app_transition_hidden_and_unfocused();
  }

  // We have modal windows and we should render them, either because they asked to or because
  // they recently became the top window in their respective modal stacks and haven't noticed yet.
  // See the handling for off screen windows in prv_render_modal_stack.
  if (update.properties & ModalProperty_RenderRequested) {
    compositor_modal_render_ready();
  }

  s_last_highest_modal_priority = update.highest_idx;
}

typedef struct IterContext {
  Window *current_top_window;
  ModalPriority current_idx;
  ModalPriority first_visible_idx;
  ModalPriority first_transition_idx;
  ModalPriority first_focus_idx;
  ModalPriority first_opaque_idx;
} IterContext;

typedef bool (*ModalContextIterCallback)(ModalContext *modal, IterContext *iter, void *data);

static void prv_each_modal_stack(ModalContextIterCallback callback, void *data) {
  IterContext iter = {
    .first_visible_idx = ModalPriorityInvalid,
    .first_transition_idx = ModalPriorityInvalid,
    .first_focus_idx = ModalPriorityInvalid,
    .first_opaque_idx = ModalPriorityInvalid,
  };
  for (ModalPriority idx = NumModalPriorities - 1; idx >= ModalPriorityMin; idx--) {
    ModalContext *context = &s_modal_window_stacks[idx];
    if (!prv_has_visible_window(context, NULL)) {
      continue;
    } else if (iter.first_visible_idx == ModalPriorityInvalid) {
      iter.first_visible_idx = idx;
    }
    if (prv_has_transition_window(context) && (iter.first_transition_idx == ModalPriorityInvalid)) {
      iter.first_transition_idx = idx;
    }
    if (prv_has_focusable_window(context) && (iter.first_focus_idx == ModalPriorityInvalid)) {
      iter.first_focus_idx = idx;
    }
    if (prv_has_opaque_window(context) && (iter.first_opaque_idx == ModalPriorityInvalid)) {
      iter.first_opaque_idx = idx;
    }
  }
  if (iter.first_visible_idx != ModalPriorityInvalid) {
    for (ModalPriority idx = ModalPriorityMin; idx < NumModalPriorities; idx++) {
      ModalContext *modal = &s_modal_window_stacks[idx];
      WindowStack *stack = &modal->window_stack;
      iter.current_idx = idx;
      iter.current_top_window = window_stack_get_top_window(stack);
      if (!iter.current_top_window) {
        continue;
      }
      const bool should_continue = callback(modal, &iter, data);
      if (!should_continue) {
        break;
      }
    }
  }
}

static bool prv_update_modal_stack_callback(ModalContext *modal, IterContext *iter, void *data) {
  UpdateContext *ctx = data;
  Window *window = iter->current_top_window;

  // Handle window state changes
  const bool is_visible = (iter->current_idx >= iter->first_opaque_idx);
  if (!window->on_screen && is_visible) {
    // We've been exposed by a higher priority modal window stack emptying out, become on screen
    // now.
    window_set_on_screen(window, true /* new on screen */, true /* call handlers */);
  }

  // Setting on-screen can configure the click, but if this is a window below a transparent
  // window that just disappeared, it was already on screen and may need its click configured.
  const bool is_focused = (iter->current_idx == iter->first_focus_idx);
  if (!window->is_click_configured && is_focused) {
    // Input is now exposed by a higher priority modal window stack emptying out, gain input
    window_setup_click_config_provider(window);
#ifdef CONFIG_TOUCH
    // Migrate the kernel recognizer manager onto the newly focused modal. A modal-over-modal focus
    // change happens across priority stacks and bypasses the per-stack window transitions, so this
    // is where the manager follows the focus.
    prv_modal_recognizer_focus(window);
#endif
  } else if (window->is_click_configured && !is_focused) {
    // A different modal window now has focus
    window->is_click_configured = false;
#ifdef CONFIG_TOUCH
    // This modal just lost focus to a higher priority one; unbind the kernel manager so its active
    // layer and any in-flight gesture do not leak into the newly focused modal.
    prv_modal_recognizer_focus(NULL);
#endif
  }

  // Set the last highest visible modal priority
  if (is_visible) {
    ctx->highest_idx = iter->current_idx;
  }

  // Update properties based on state changes
  // If this callback was called, there exists a modal
  ctx->properties |= ModalProperty_Exists;

  if (iter->current_idx > ModalPriorityDiscreet) {
    // There is a modal window that has a compositor transition
    ctx->properties |= ModalProperty_CompositorTransitions;
  }

  if (is_visible) {
    if (!window->is_transparent) {
      // There is a visible opaque window, remove the transparent property
      ctx->properties &= ~ModalProperty_Transparent;
    }
    if (window->is_render_scheduled) {
      // There is a visible window that will render, add the render requested property
      ctx->properties |= ModalProperty_RenderRequested;
    }
  }

  if (is_focused) {
    // There is a modal with focus, remove the unfocused property
    ctx->properties &= ~ModalProperty_Unfocused;
  }

  return true;
}

static bool prv_render_modal_stack_callback(ModalContext *modal, IterContext *iter, void *data) {
  if (iter->current_idx < iter->first_opaque_idx) {
    return true;
  }

  GContext *ctx = data;
  WindowStack *stack = &modal->window_stack;
  Window *window = iter->current_top_window;

  if (window_stack_is_animating(stack) &&
      stack->transition_context.implementation &&
      stack->transition_context.implementation->render) {
    // a lot of safety guards to make sure the transition can do render by its own
    WindowTransitioningContext *const transition_context = &stack->transition_context;
    transition_context->implementation->render(transition_context, ctx);
  } else {
    PROFILER_NODE_START(render_modal);
    window_render(window, ctx);
    PROFILER_NODE_STOP(render_modal);
  }

  return true;
}

static void prv_update_modal_stacks(UpdateContext *context) {
  context->highest_idx = ModalPriorityInvalid;
  context->properties = ModalPropertyDefault;
  prv_each_modal_stack(prv_update_modal_stack_callback, context);
}


ModalProperty modal_manager_get_properties(void) {
  return modal_manager_get_enabled() ? s_current_modal_properties : ModalPropertyDefault;
}

void modal_manager_render(GContext *ctx) {
  PBL_ASSERTN(ctx);
  prv_each_modal_stack(prv_render_modal_stack_callback, ctx);
}

typedef struct VisibleContext {
  Window *window;
  bool visible;
} VisibleContext;

static bool prv_is_window_visible_callback(ModalContext *modal, IterContext *iter, void *data) {
  if (iter->current_idx < iter->first_opaque_idx) {
    return true;
  }

  VisibleContext *ctx = data;
  ctx->visible = (ctx->window == iter->current_top_window);
  return !ctx->visible;
}

bool modal_manager_is_window_visible(Window *window) {
  VisibleContext context = { .window = window };
  prv_each_modal_stack(prv_is_window_visible_callback, &context);
  return context.visible;
}

typedef struct FocusedContext {
  Window *window;
  bool focused;
} FocusedContext;

static bool prv_is_window_focused_callback(ModalContext *modal, IterContext *iter, void *data) {
  FocusedContext *ctx = data;
  ctx->focused = ((iter->current_top_window == ctx->window) &&
                  (iter->current_idx == iter->first_focus_idx));
  return !ctx->focused;
}

bool modal_manager_is_window_focused(Window *window) {
  FocusedContext context = { .window = window };
  prv_each_modal_stack(prv_is_window_focused_callback, &context);
  return context.focused;
}

static Window *prv_get_visible_focused_window(void) {
  WindowStack *stack = prv_find_window_stack(prv_has_visible_focusable_window, NULL);
  return window_stack_get_top_window(stack);
}

void modal_manager_handle_button_event(PebbleEvent *event) {
  ClickManager *click_manager = modal_manager_get_click_manager();
  switch (event->type) {
    case PEBBLE_BUTTON_DOWN_EVENT: {
      // If we get a button event, it must also be for the top modal window.
      Window *window = prv_get_visible_focused_window();
      // Ensure that this function isn't being called when a modal window
      // is not present.
      PBL_ASSERTN(window);
      ButtonId id = event->button.button_id;
      if (id == BUTTON_ID_BACK && !window->overrides_back_button) {
        window_stack_remove(window, true /* animated */);
      } else {
        click_recognizer_handle_button_down(&click_manager->recognizers[id]);
      }
      break;
    }
    case PEBBLE_BUTTON_UP_EVENT: {
      ButtonId id = event->button.button_id;
      click_recognizer_handle_button_up(&click_manager->recognizers[id]);
      break;
    }
    default:
      PBL_CROAK("Invalid event type: %u", event->type);
  }
}

void modal_window_push(Window *window, ModalPriority priority, bool animated) {
  // Note: We do not have to adjust the `animated` argument to consider whether
  // this window is higher priority than the current visible window stack, as
  // this is taken care of by the transition context handlers in `window_stack.c`
  window_stack_push(modal_manager_get_window_stack(priority), window, animated);
}

// Commands
////////////////////////////

typedef struct WindowStackInfoContext {
  SemaphoreHandle_t interlock;
  WindowStackDump *dumps[NumModalPriorities];
  size_t counts[NumModalPriorities];
  bool disabled;
} WindowStackInfoContext;

static void prv_modal_window_stack_info_cb(void *ctx) {
  WindowStackInfoContext *info = ctx;
  if (modal_manager_get_enabled()) {
    for (ModalPriority priority = 0;
         priority < NumModalPriorities;
         ++priority) {
      WindowStack *window_stack = modal_manager_get_window_stack(priority);
      info->counts[priority] = window_stack_dump(window_stack,
                                                 &info->dumps[priority]);
    }
  } else {
    info->disabled = true;
  }
  xSemaphoreGive(info->interlock);
}

void command_modal_stack_info(void) {
  WindowStackInfoContext info = {
    .interlock = xSemaphoreCreateBinary(),
  };
  if (!info.interlock) {
    prompt_send_response("Couldn't allocate semaphore for modal stack");
    return;
  }

  launcher_task_add_callback(prv_modal_window_stack_info_cb, &info);
  xSemaphoreTake(info.interlock, portMAX_DELAY);
  vSemaphoreDelete(info.interlock);

  prompt_send_response("Modal Stack, top to bottom:");

  char buffer[128];
  for (ModalPriority priority = NumModalPriorities - 1;
       priority > ModalPriorityInvalid;
       --priority) {
    prompt_send_response_fmt(buffer, sizeof(buffer), "Priority: %d (%zu)",
                             priority, info.counts[priority]);
    if (info.counts[priority] > 0 && !info.dumps[priority]) {
      prompt_send_response("Couldn't allocate buffers for modal stack data");
    } else {
      for (size_t i = 0; i < info.counts[priority]; ++i) {
        prompt_send_response_fmt(buffer, sizeof(buffer), "window %p <%s>",
                                 info.dumps[priority][i].addr,
                                 info.dumps[priority][i].name);
      }
    }
    kernel_free(info.dumps[priority]);
  }
}

void modal_manager_reset(void) {
  for (ModalPriority idx = ModalPriorityInvalid + 1; idx < NumModalPriorities; idx++) {
    memset(&s_modal_window_stacks[idx], 0, sizeof(ModalContext));
  }

  s_modal_min_priority = ModalPriorityDiscreet;

  modal_manager_init();
}
