/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "window_stack.h"

#include "app_window_click_glue.h"
#include "recognizer/recognizer_manager.h"
#include "window_manager.h"
#include "window_private.h"
#include "window_stack_animation.h"
#include "window_stack_private.h"

#include "applib/applib_malloc.auto.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/compositor/compositor.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/struct.h"

// Private API
////////////////////////////////////

static bool prv_filter_window_item_for_window(ListNode *node, void *data) {
  WindowStackItem *item = (WindowStackItem *)node;
  return item->window == data;
}

static WindowStackItem *prv_next_item(WindowStackItem *item) {
  return (WindowStackItem *)list_get_next(&item->list_node);
}

static WindowStackItem *prv_find_window_stack_item_for_window(WindowStack *window_stack,
                                                              Window *window) {
  if (!window_stack) {
    // Window can't be on a NULL window stack.
    return NULL;
  }

  WindowStackItem *item = (WindowStackItem *)list_find(window_stack->list_head,
      prv_filter_window_item_for_window, window);

  return item;
}

static void prv_set_new_window_on_screen(Window *appeared_window) {
  GRect bounds = window_calc_frame(window_get_fullscreen(appeared_window));

  Layer *root_layer = window_get_root_layer(appeared_window);
  layer_set_bounds(root_layer, &bounds);

  window_set_on_screen(appeared_window, true /* new window */, true /* call handlers */);

  click_manager_clear(window_manager_get_window_click_manager(appeared_window));
}

static void prv_unload_removed_windows(WindowStack *window_stack) {
  // Copy the removed windows list into a local array and then call the unload callback on
  // each removed window.
  WindowStackItem *items_to_unload[WINDOW_STACK_ITEMS_MAX];
  unsigned int num_items = 0;
  while (window_stack->removed_list_head) {
    if (num_items >= WINDOW_STACK_ITEMS_MAX) {
      break;
    }
    WindowStackItem *removed_item = (WindowStackItem *)window_stack->removed_list_head;
    items_to_unload[num_items++] = removed_item;
    window_stack->removed_list_head = list_pop_head(window_stack->removed_list_head);
  }

  WindowTransitioningContext *context = &window_stack->transition_context;

  for (unsigned int i = 0; i < num_items; ++i) {
    // The update routine for the transition_to animation relies on the
    // window_to being present. If we are unloading the window we should really
    // unschedule the animation so we don't touch free'd memory!
    //
    // For now, rely on our animation transition routines having checks for
    // NULL windows since our animation subsystem can't cope with these
    // unschedules in some cases (See PBL-25460 for more details)
    if (context->window_to == items_to_unload[i]->window) {
      context->window_to = NULL;
    }

    if (context->window_from == items_to_unload[i]->window) {
      context->window_from = NULL;
    }

    // Reset the recognizer manager before the window is destroyed so it drops any dangling pointer
    window_lost_input_focus(items_to_unload[i]->window);

    window_unload(items_to_unload[i]->window);
    applib_free(items_to_unload[i]);
  }
}

static void prv_transition_to(Window *window_from, Window *window_to,
                              const WindowTransitionImplementation *transition) {
  PBL_ASSERTN(window_to && transition);
  WindowStack *window_stack = window_to->parent_window_stack;
  WindowTransitioningContext *context = &window_stack->transition_context;

  if (context->animation) {
    // If we currently have an animation, run it to completion immediately before starting
    // another transition.
    // For 2.x apps, just unschedule the animation because there is no equivalent call for
    // animation_set_elapsed (which is not available to 2.x apps)
    if (process_manager_compiled_with_legacy2_sdk()) {
      animation_unschedule(context->animation);
      applib_free(context->animation);
    } else {
      animation_set_elapsed(context->animation,
                            animation_get_duration(context->animation, true, true));
    }
  }

  *context = (WindowTransitioningContext) {
    .window_to = window_to,
    .window_to_last_x = INT16_MAX,
    .window_from = window_from,
    .window_from_last_x = INT16_MAX,
    .implementation = transition,
  };

  // TODO: PBL-17806 in future, store frames and config values as well
  if (transition->create_animation) {
    context->animation = transition->create_animation(context);
  }
  // If we haven't set an animation, either because create_animation() was NULL or it returned
  // NULL, use g_window_transition_none_implementation
  if (!context->animation) {
    context->animation = g_window_transition_none_implementation.create_animation(context);
  }
  PBL_ASSERTN(context->animation);

  // TODO: PBL-17645 setup container view
  animation_schedule(context->animation);
  // TODO: PBL-17645 cleanup container view in animation.stopped
}

static void prv_next_inserter(WindowStackItem *stack_item) {
  WindowStack *window_stack = stack_item->window->parent_window_stack;
  WindowStackItem *prev_item = (WindowStackItem *)window_stack->list_head;
  // Insert after the found previous item:
  list_insert_after(&prev_item->list_node, &stack_item->list_node);
}

static void prv_push_inserter(WindowStackItem *stack_item) {
  WindowStack *window_stack = stack_item->window->parent_window_stack;
  WindowStackItem *prev_item = (WindowStackItem *)(window_stack->list_head);

  // Insert before the found previous item, provided that it exists.
  ListNode *list_node_closest_to_head = &stack_item->list_node;
  list_init(list_node_closest_to_head);

  if (prev_item) {
    ListNode *prev_list_node = &prev_item->list_node;
    list_node_closest_to_head = list_insert_before(prev_list_node, list_node_closest_to_head);
  }

  // Update the reference to list head in case the window that was inserted is
  // the new list head.
  window_stack->list_head = list_get_head(list_node_closest_to_head);
}

static void prv_insert_with_function(WindowStack *window_stack_to, Window *window,
                                     void(*inserter)(WindowStackItem *),
                                     const WindowTransitionImplementation *transition_insert,
                                     const WindowTransitionImplementation *transition_pop) {
  PBL_ASSERTN(window_stack_to);
  WindowStack *window_stack_from = window->parent_window_stack;
  Window *window_from = window_manager_get_top_window();

  // Assign the new stack for the window
  window->parent_window_stack = window_stack_to;

  if (!window_from) {
    // We do not animate the first window, but instead let the compositor animate.
    transition_insert = &g_window_transition_none_implementation;
    transition_pop = &g_window_transition_none_implementation;
  }

  // This is a backwards compatibility hack for legacy2 applications.  By default, watchface
  // windows are not fullscreen until they're pushed onto the window stack, whereas legacy2
  // watchfaces assume they are already full screen before being pushed.
  bool is_app_window = window_manager_is_app_window(window);
  if (is_app_window && sys_app_is_watchface()) {
    window_set_fullscreen(window, true);
  }

  // If on the list of removed items for a window stack, remove from the removed items list
  // as we want to add it back to the window stack.
  if (window_stack_from) {
    ListNode *node = list_find(window_stack_from->removed_list_head,
        prv_filter_window_item_for_window, window);
    if (node != NULL) {
      list_remove(node, &window_stack_from->removed_list_head, NULL);
    }
  }

  WindowStackItem *item = NULL;
  if (window_stack_contains_window(window_stack_from, window)) {
    // If the item is on the list of window items, then we remove it as we're going to
    // re-insert it.
    item = prv_find_window_stack_item_for_window(window_stack_from, window);
    list_remove(&item->list_node, &window_stack_from->list_head, NULL);
  } else {
    // If the item is not yet on the window stack's list, then we allocate space for it
    // on the heap.
    item = applib_type_malloc(WindowStackItem);

    *item = (WindowStackItem) {
      .window = window,
      .pop_transition_implementation = transition_pop,
    };
  }

  inserter(item);

  if (window_from == NULL || !window_manager_is_window_visible(window_from)) {
    prv_transition_to(window_from, window, transition_insert);
  }

  PBL_LOG_DBG("(+) %s=%p <%s>", is_app_window ? "window" : "modal window",
      window, window_get_debug_name(window));
}

static Window *prv_remove_item(WindowStackItem *pop_item,
                               const WindowTransitionImplementation *transition) {
  PBL_ASSERTN(pop_item->window);
  WindowStack *window_stack = pop_item->window->parent_window_stack;
  // Do a transition from element that needs to be removed only if it was on the
  // top of the visible window stack.
  Window *window_from = NULL;

  // If this window is currently transitioning and it is a modal window
  if ((window_stack->transition_context.window_to == pop_item->window) &&
      !window_manager_is_app_window(pop_item->window)) {
    compositor_transition_cancel();
  }

  if (window_manager_is_window_visible(pop_item->window)) {
    window_from = pop_item->window;
    // If no transition is explicitly provided, use the one specified when pushed.
    transition = transition ?: pop_item->pop_transition_implementation;
    PBL_ASSERTN(transition);
  } else {
    // We don't intentionally clean up the .pop_transition of a previous element if
    // a client actively messes with the window stack; they need to take care of this
    // in any potential custom transition.  The default transitions cannot handle this.
    window_from = NULL;
  }

  // Remove the item from the window stack
  list_remove(&pop_item->list_node, &window_stack->list_head, NULL);

  WindowStackItem *stack_item = (WindowStackItem *)window_stack->list_head;
  Window *window_to = stack_item ? stack_item->window : NULL;

  // Add the removed item to the 'removed' list
  window_stack->removed_list_head = list_insert_before(window_stack->removed_list_head,
      &pop_item->list_node);

  // Store the window here, as we're potentially free'ing the item later on.
  Window *pop_item_window = pop_item->window;
  bool is_app_window = window_manager_is_app_window(pop_item_window);
  PBL_LOG_DBG("(-) %s=%p <%s>", is_app_window ? "window" : "modal window",
      pop_item_window, window_get_debug_name(pop_item_window));

  // Only animate if the window was previously at the top of the stack and there's a
  // window we can transition to.
  if (window_from && window_to) {
    prv_transition_to(window_from, window_to, transition);
  } else {
    // We don't fire a transition in this case, but to ensure that all window-callbacks
    // will still be called and the click handler is managed correctly, we call the
    // appropriate helper functions manually with a fake transitioning context.
    WindowTransitioningContext ctx = {
      .window_from = pop_item_window,
    };

    window_transition_context_disappear(&ctx);
  }

  return pop_item_window;
}

// Public API
////////////////////////////////////

Window *window_stack_get_top_window(WindowStack *window_stack) {
  WindowStackItem *item = (WindowStackItem *)NULL_SAFE_FIELD_ACCESS(window_stack, list_head, NULL);
  return NULL_SAFE_FIELD_ACCESS(item, window, NULL);
}

void window_stack_push(WindowStack *window_stack, Window *window, bool animated) {
  const WindowTransitionImplementation *transition_insert =
      animated ? window_transition_get_default_push_implementation() :
                 &g_window_transition_none_implementation;
  const WindowTransitionImplementation *transition_pop =
      animated ? window_transition_get_default_pop_implementation() :
                 &g_window_transition_none_implementation;

  window_stack_push_with_transition(window_stack, window, transition_insert, transition_pop);
}

void window_stack_push_with_transition(WindowStack *window_stack, Window *window,
                                       const WindowTransitionImplementation *push_transition,
                                       const WindowTransitionImplementation *pop_transition) {
  if (window_stack->lock_push) {
    return;
  }

  PBL_ASSERTN(push_transition && pop_transition);
  prv_insert_with_function(window_stack, window, prv_push_inserter, push_transition,
                           pop_transition);
}

void window_stack_insert_next(WindowStack *window_stack, Window *window) {
  if (window_stack->lock_push) {
    return;
  }

  const WindowTransitionImplementation *transition_to = &g_window_transition_none_implementation;
  const WindowTransitionImplementation *transition_pop =
      window_transition_get_default_pop_implementation();
  prv_insert_with_function(window_stack, window, prv_next_inserter, transition_to, transition_pop);
}

Window *window_stack_pop(WindowStack *window_stack, bool animated) {
  // Transition NULL will default to the registered pop_transition to the stack item
  const WindowTransitionImplementation *transition =
      animated ? NULL : &g_window_transition_none_implementation;
  return window_stack_pop_with_transition(window_stack, transition);
}

Window *window_stack_pop_with_transition(WindowStack *window_stack,
                                         const WindowTransitionImplementation *transition) {
  if (window_stack->list_head == NULL) {
    PBL_LOG_DBG("Nothing to pop.");
    return NULL;
  }

  WindowStackItem *pop_item = (WindowStackItem *)window_stack->list_head;
  Window *window = NULL;
  if (pop_item) {
    window = prv_remove_item(pop_item, transition);
  }
  return window;
}

void window_stack_pop_all(WindowStack *window_stack, const bool animated) {
  if (window_stack->list_head == NULL) {
    return;
  }

  WindowStackItem *top_item = (WindowStackItem *)window_stack->list_head;
  WindowStackItem *next_item;
  do {
    // We manually remove each item to ensure that we do not call prv_remove_unloaded_windows
    // until we have cleaned up the list.  This prevents us from running into the issue where
    // an unload handler pushes a window onto the stack, but that window is subsequently
    // popped from the stack on another iteration of the while loop.
    next_item = prv_next_item(top_item);
    if (next_item == NULL) {
      break;
    }
    list_remove(&next_item->list_node, &window_stack->list_head, NULL);

    window_stack->removed_list_head = list_insert_before(window_stack->removed_list_head,
                                                         &next_item->list_node);

    window_set_on_screen(next_item->window, false /* not new */, true /* call handlers */);
  } while (true);

  window_stack_pop(window_stack, animated);
}

bool window_stack_remove(Window *window, bool animated) {
  if (window == NULL) {
    return false;
  }

  WindowStackItem *item = prv_find_window_stack_item_for_window(window->parent_window_stack,
      window);
  if (item == NULL) {
    return false;
  }

  const WindowTransitionImplementation *transition =
      animated ? window_transition_get_default_pop_implementation() :
                 &g_window_transition_none_implementation;

  window = prv_remove_item(item, transition);
  return window != NULL;
}

bool window_stack_remove_with_transition(Window *window,
                                         const WindowTransitionImplementation *transition) {
  if (window == NULL) {
    return false;
  }

  WindowStack *stack = window->parent_window_stack;
  WindowStackItem *item = prv_find_window_stack_item_for_window(stack, window);
  if (item == NULL) {
    return false;
  }
  window = prv_remove_item(item, transition);
  return window != NULL;
}

bool window_stack_contains_window(WindowStack *window_stack, Window *window) {
  return (prv_find_window_stack_item_for_window(window_stack, window) != NULL);
}

uint32_t window_stack_count(WindowStack *window_stack) {
  return list_count(window_stack->list_head);
}

// Stack status
////////////////////////////////////

void window_stack_lock_push(WindowStack *window_stack) {
  window_stack->lock_push = true;
}

void window_stack_unlock_push(WindowStack *window_stack) {
  window_stack->lock_push = false;
}

bool window_stack_is_animating(WindowStack *window_stack) {
  return window_stack && animation_is_scheduled(window_stack->transition_context.animation);
}

bool window_stack_is_animating_with_fixed_status_bar(WindowStack *window_stack) {
  WindowTransitioningContext *context = &window_stack->transition_context;
  return window_stack_is_animating(window_stack) &&
      window_has_status_bar(context->window_from) &&
      window_has_status_bar(context->window_to);
}

// Transitioning Context Functions
////////////////////////////////////

// @note: This function prototypes exist in window_stack_private.h

bool window_transition_context_has_legacy_window_to(WindowStack *stack, Window *window) {
  return (stack->transition_context.window_to == window) &&
      process_manager_compiled_with_legacy2_sdk();
}

void window_transition_context_disappear(WindowTransitioningContext *context) {
  Window *window_from = context->window_from;
  if (!window_from || window_manager_is_window_visible(window_from)) {
    PBL_LOG_DBG("No windows to unload from stack.");
    context->window_from = NULL;
    return;
  }

  // Remove window reference from context to prevent future calls to it (e.g. "is dirty?")
  context->window_from = NULL;

  if (!window_manager_is_window_visible(window_from)) {
    window_set_on_screen(window_from, false /* not new */, true /* call handlers */);
  }

  prv_unload_removed_windows(window_from->parent_window_stack);
}

void window_transition_context_appear(WindowTransitioningContext *context) {
  Window *window_to = context->window_to;
  if (!window_manager_is_window_visible(window_to)) {
    return;
  }

  prv_set_new_window_on_screen(window_to);

  ClickManager *click_manager = window_manager_get_window_click_manager(window_to);
  if (window_manager_is_window_focused(window_to)) {
    // TODO: PBL-37477 Window Stack directly calls click config
    // Either this or app_click_config_setup_with_window should be calling
    // window_setup_click_config_provider instead
    app_click_config_setup_with_window(click_manager, window_to);

#ifdef CONFIG_TOUCH
    if (window_stack_is_animating(window_to->parent_window_stack)) {
      // The appear hook fires at the start of the transition and there is no end-of-transition
      // hook. Drop any in-flight gesture but keep the window bound so navigation still works when
      // the animation settles.
      RecognizerManager *manager = window_get_recognizer_manager(window_to);
      if (manager) {
        recognizer_manager_set_window(manager, window_to);
        recognizer_manager_cancel_and_reset(manager);
      }
    } else {
      window_became_input_focus(window_to);
    }
#endif
  }
}

// Debug and Test Functions
/////////////////////////////

size_t window_stack_dump(WindowStack *stack, WindowStackDump **dump) {
  *dump = NULL;
  size_t count = window_stack_count(stack);
  size_t idx = 0;
  if (count > 0) {
    *dump = kernel_calloc(count, sizeof(WindowStackDump));
    if (*dump) {
      WindowStackItem *item = (WindowStackItem *)stack->list_head;
      while (item) {
        (*dump)[idx++] = (WindowStackDump) {
          .addr = item->window,
          .name = window_get_debug_name(item->window),
        };
        item = (WindowStackItem *)list_get_next(&item->list_node);
      }
      PBL_ASSERTN(idx == count);
    }
  }
  return count;
}
