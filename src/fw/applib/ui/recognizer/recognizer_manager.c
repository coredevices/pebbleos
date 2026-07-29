/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "recognizer.h"
#include "recognizer_list.h"
#include "recognizer_manager.h"
#include "recognizer_private.h"

#include "applib/applib_malloc.auto.h"
#include "applib/ui/layer.h"
#include "applib/ui/window.h"
#include "pbl/services/touch/touch_event.h"
#include "system/passert.h"
#include "pbl/util/list.h"

#include <stddef.h>

T_STATIC bool prv_process_all_recognizers(RecognizerManager *manager,
                                          RecognizerListIteratorCb iter_cb, void *context) {

  // Process the task-global recognizers first (NULL list is a no-op)
  if (!recognizer_list_iterate(manager->global_list, iter_cb, context)) {
    return false;
  }

  // Then process the recognizers attached to the window pointed to by the manager.
  // Note: This is kind of weird because we could just request the top window of the app, but I want
  // to keep the decision making of when to cancel recognizers and change the window that the
  // manager points to (when windows are pushed and popped) outside this module
  if (!recognizer_list_iterate(window_get_recognizer_list(manager->window), iter_cb, context)) {
    return false;
  }

  Layer *root = window_get_root_layer(manager->window);
  Layer *layer = manager->active_layer;
  // layers attached to window are attached to it's root layer, so don't process recognizers on the
  // root layer
  while (layer && (layer != root)) {
    if (!recognizer_list_iterate(layer_get_recognizer_list(layer), iter_cb, context)) {
      return false;
    }
    layer = layer->parent;
  }
  return true;
}

typedef struct ProcessTouchCtx {
  Recognizer *triggered;
  const TouchEvent *touch_event;
} ProcessTouchCtx;

T_STATIC bool prv_dispatch_touch_event(Recognizer *recognizer, void *context) {
  ProcessTouchCtx *ctx = context;
  // Skip recognizer if it has already failed, been cancelled or completed
  if (!recognizer_is_active(recognizer)) {
    return true;
  }
  // If there is another recognizer that has started recognizing a gesture, then skip this
  // recognizer, unless it has been configured to operate simultaneously with the recognizer
  // that has started recognizing a gesture.
  if (ctx->triggered && !recognizer_should_evaluate_simultaneously(recognizer, ctx->triggered)) {
    return true;
  }

  recognizer_handle_touch_event(recognizer, ctx->touch_event);
  RecognizerState state = recognizer_get_state(recognizer);
  if (!ctx->triggered &&
      ((state == RecognizerState_Completed) ||
       (state == RecognizerState_Started) ||
       (state == RecognizerState_Updated))) {
    ctx->triggered = recognizer;
  }

  return true;
}

static Recognizer *prv_dispatch_touch_event_to_all_recognizers(RecognizerManager *manager,
                                                               const TouchEvent *touch_event) {
  ProcessTouchCtx ctx = {
    .touch_event = touch_event,
  };
  prv_process_all_recognizers(manager, prv_dispatch_touch_event, &ctx);
  return ctx.triggered;
}

typedef struct FailRecognizerCtx {
  Recognizer *triggered;
  bool recognizers_active;
} FailRecognizerCtx;

T_STATIC bool prv_fail_recognizer(Recognizer *recognizer, void *context) {
  FailRecognizerCtx *ctx = context;
  if ((recognizer == ctx->triggered) ||
      !recognizer_is_active(recognizer)) {
    return true;
  }
  if (ctx->triggered && !recognizer_should_evaluate_simultaneously(recognizer, ctx->triggered)) {
    recognizer_set_failed(recognizer);
  }
  // Accumulate: ANY still-active recognizer (e.g. a simultaneous one earlier in the list) must
  // keep the manager from resetting, not just the last one examined.
  ctx->recognizers_active = ctx->recognizers_active || recognizer_is_active(recognizer);
  return true;
}

static bool prv_fail_other_recognizers(RecognizerManager *manager) {
  FailRecognizerCtx ctx = {
    .triggered = manager->triggered,
  };
  prv_process_all_recognizers(manager, prv_fail_recognizer, &ctx);
  return ctx.recognizers_active;
}

static bool prv_is_active_and_triggered(Recognizer *recognizer, void *context) {
  Recognizer **triggered = context;
  if (recognizer_has_triggered(recognizer) && recognizer_is_active(recognizer)) {
    *triggered = recognizer;
    return false;
  }
  return true;
}

static Recognizer *prv_any_recognizers_active_triggered(RecognizerManager *manager) {
  Recognizer *triggered = NULL;
  prv_process_all_recognizers(manager, prv_is_active_and_triggered, &triggered);
  return triggered;
}

static void prv_process_layer_tree_recognizers(RecognizerManager *manager, Layer *top_layer,
                                               Layer *bottom_layer,
                                               RecognizerListIteratorCb iter_cb) {
  Layer *root = window_get_root_layer(manager->window);
  Layer *curr = bottom_layer;
  // Traverse the layer's ancestors and cancel all (unless one of the ancestors is the new active
  // layer - then stop)
  while (curr && (curr != top_layer) && (curr != root)) {
    recognizer_list_iterate(layer_get_recognizer_list(curr), iter_cb, manager);
    curr = curr->parent;
  }
}

static void prv_set_triggered(RecognizerManager *manager, Recognizer *triggered) {
  manager->triggered = triggered;
  if (triggered) {
    manager->state = RecognizerManagerState_RecognizersTriggered;
  }
}

static bool prv_cancel_or_fail_recognizer(Recognizer *recognizer, void *context) {
  RecognizerManager *manager = context;
  if (manager && (manager->triggered == recognizer)) {
    prv_set_triggered(manager, NULL);
  }
  if (recognizer_get_state(recognizer) == RecognizerState_Possible) {
    recognizer_set_failed(recognizer);
  } else {
    recognizer_cancel(recognizer);
  }
  return true;
}

static void prv_cancel_all_recognizers(RecognizerManager *manager) {
  prv_process_all_recognizers(manager, prv_cancel_or_fail_recognizer, NULL);
}

static bool prv_reset_recognizer(Recognizer *recognizer, void *context) {
  recognizer_reset(recognizer);
  return true;
}

static void prv_reset_layer_tree_recognizers(RecognizerManager *manager, Layer *top_layer,
                                                Layer *bottom_layer) {
  prv_process_layer_tree_recognizers(manager, top_layer, bottom_layer, prv_reset_recognizer);
}

static void prv_reset_all_recognizers(RecognizerManager *manager) {
  prv_process_all_recognizers(manager, prv_reset_recognizer, NULL);
}

static void prv_reset(RecognizerManager *manager) {
  prv_reset_all_recognizers(manager);
  prv_set_triggered(manager, NULL);
  manager->state = RecognizerManagerState_WaitForTouchdown;
  manager->active_layer = NULL;
}

static void prv_fail_then_reset_if_no_active_recognizers(RecognizerManager *manager) {
  bool other_recognizers_active = prv_fail_other_recognizers(manager);

  // Reset if all recognizers are complete or failed
  if (!recognizer_is_active(manager->triggered) && !other_recognizers_active) {
    prv_reset(manager);
  }
}

static void prv_cleanup_state_change(RecognizerManager *manager, Recognizer *triggered) {
  if (triggered) {
    prv_set_triggered(manager, triggered);
  }
  prv_fail_then_reset_if_no_active_recognizers(manager);
  if (manager->state == RecognizerManagerState_RecognizersTriggered) {
    prv_set_triggered(manager, prv_any_recognizers_active_triggered(manager));
  }
}

void recognizer_manager_handle_touch_event(const TouchEvent *touch_event, void *context) {
  RecognizerManager *manager = context;

  if (touch_event->type == TouchEvent_Touchdown) {
    Layer *root = manager->window ? window_get_root_layer(manager->window) : NULL;
    GPoint touch_pos = GPoint(touch_event->x, touch_event->y);
    Layer *new_active_layer = root ? layer_find_layer_containing_point(root, &touch_pos) : NULL;
    if (new_active_layer == root) {
      new_active_layer = NULL;
    }

    if (manager->state == RecognizerManagerState_WaitForTouchdown) {
      manager->state = RecognizerManagerState_RecognizersActive;
      manager->active_layer = new_active_layer;
    } else {
      // A new touchdown arrived mid-gesture. Reset first to complete the cancelled-snap of any
      // stuck recognizer, then decide whether to start fresh.
      prv_reset(manager);
      if (touch_event->non_navigational) {
        // The touch must not drive navigation, so stay idle.
        return;
      }
      manager->state = RecognizerManagerState_RecognizersActive;
      manager->active_layer = new_active_layer;
      prv_reset_layer_tree_recognizers(manager, NULL, new_active_layer);
    }
  }

  if (manager->state != RecognizerManagerState_WaitForTouchdown) {
    Recognizer *triggered = prv_dispatch_touch_event_to_all_recognizers(manager, touch_event);
    prv_cleanup_state_change(manager, triggered);
  }
}

void recognizer_manager_init(RecognizerManager *manager) {
  PBL_ASSERTN(manager);
  *manager = (RecognizerManager) {
    .state = RecognizerManagerState_WaitForTouchdown
  };
}

void recognizer_manager_set_window(RecognizerManager *manager, Window *window) {
  PBL_ASSERTN(manager);
  manager->window = window;
}

void recognizer_manager_cancel_touches(RecognizerManager *manager) {
  PBL_ASSERTN(manager);
  prv_cancel_all_recognizers(manager);
}

void recognizer_manager_reset(RecognizerManager *manager) {
  PBL_ASSERTN(manager);
  prv_reset_all_recognizers(manager);
}

void recognizer_manager_cancel_and_reset(RecognizerManager *manager) {
  PBL_ASSERTN(manager);
  prv_reset(manager);
}

void recognizer_manager_register_recognizer(RecognizerManager *manager, Recognizer *recognizer) {
  // A layer may be attached before it is added to a window, so a NULL manager is a soft no-op
  if (!manager) {
    return;
  }
  PBL_ASSERTN(recognizer);

  if (recognizer->manager == manager) {
    // Already registered with this manager
    return;
  }

  recognizer_reset(recognizer);
  if (manager->triggered) {
    // Set recognizer to failed state so that it is only evaluated after all recognizers are reset
    recognizer_set_failed(recognizer);
  }
  recognizer_set_manager(recognizer, manager);
}

void recognizer_manager_deregister_recognizer(RecognizerManager *manager, Recognizer *recognizer) {
  // A layer may be detached after its window is gone, so a NULL manager is a soft no-op
  if (!manager) {
    return;
  }
  PBL_ASSERTN(recognizer);

  if (recognizer->manager != manager) {
    // Registered with a different manager
    return;
  }

  recognizer_cancel(recognizer);
  Recognizer *triggered = recognizer_has_triggered(recognizer) ? recognizer : NULL;
  prv_cleanup_state_change(manager, triggered);

  recognizer_reset(recognizer);
  recognizer_set_manager(recognizer, NULL);
}

void recognizer_manager_handle_state_change(RecognizerManager *manager, Recognizer *changed) {
  if (!manager || !changed) {
    return;
  }
  PBL_ASSERTN(recognizer_get_manager(changed) == manager);

  Recognizer *triggered = recognizer_has_triggered(changed) ? changed : NULL;
  prv_cleanup_state_change(manager, triggered);
}
