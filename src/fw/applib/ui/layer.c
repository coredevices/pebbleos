/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "layer.h"
#include "layer_private.h"

#include "applib/app_logging.h"
#include "applib/applib_malloc.auto.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/window_private.h"
#include "applib/unobstructed_area_service_private.h"
#include "kernel/kernel_applib_state.h"
#include "kernel/pebble_tasks.h"
#include "process_management/process_manager.h"
#include "process_state/app_state/app_state.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"

#include <string.h>

void layer_init(Layer *layer, const GRect *frame) {
  *layer = (Layer){};
  layer->frame = *frame;
  layer->bounds = (GRect){{0, 0}, frame->size};
  layer->clips = true;
}

Layer* layer_create(GRect frame) {
  Layer* layer = applib_type_malloc(Layer);
  if (layer) {
    layer_init(layer, &frame);
  }
  return layer;
}

Layer* layer_create_with_data(GRect frame, size_t data_size) {
  Layer* layer = applib_malloc(applib_type_size(Layer) + data_size);
  if (layer) {
    layer_init(layer, &frame);
    layer->has_data = true;

    DataLayer *data_layer = (DataLayer*) layer;
    memset(data_layer->data, 0, data_size);
  }
  return layer;
}

#ifdef CONFIG_TOUCH
static bool prv_destroy_recognizer(Recognizer *recognizer, void *context) {
  Layer *layer = context;
  layer_detach_recognizer(layer, recognizer);
  recognizer_destroy(recognizer);
  return true;
}

//! The recognizer manager latches the touched layer for the whole gesture and walks its parent
//! chain on every touch event. A layer destroyed mid-gesture (an app rebuilding UI from a timer
//! or message callback with a finger down) would leave that latch dangling, so if the dying layer
//! is the active layer or one of its ancestors, cancel the gesture and reset the manager.
static void prv_invalidate_manager_active_layer(Layer *layer) {
  if (!layer->window) {
    return;
  }
  RecognizerManager *manager = window_get_recognizer_manager(layer->window);
  if (!manager) {
    return;
  }
  for (Layer *active = manager->active_layer; active; active = active->parent) {
    if (active == layer) {
      recognizer_manager_cancel_and_reset(manager);
      return;
    }
  }
}
#endif

void layer_deinit(Layer *layer) {
  if (!layer) {
    return;
  }
#ifdef CONFIG_TOUCH
  // Must run while layer->window is still set (layer_remove_from_parent clears it).
  prv_invalidate_manager_active_layer(layer);
#endif
  layer_remove_from_parent(layer);

#ifdef CONFIG_TOUCH
  // Destroy all attached recognizers
  recognizer_list_iterate(&layer->recognizer_list, prv_destroy_recognizer, layer);
#endif
}

void layer_destroy(Layer* layer) {
  if (layer == NULL) {
    return;
  }
  layer_deinit(layer);
  applib_free(layer);
}

void layer_mark_dirty(Layer *layer) {
  if (layer->property_changed_proc) {
    layer->property_changed_proc(layer);
  }
  if (layer->window) {
    window_schedule_render(layer->window);
  }
}

static bool layer_process_tree_level(Layer *node, void *ctx, LayerIteratorFunc iterator_func);

static bool layer_property_changed_tree_node(Layer *node, void *ctx) {
  if (node) {
    if (node->property_changed_proc) {
      node->property_changed_proc(node);
    }
  }
  return true;
}

static bool layer_process_tree_level(Layer *node, void *ctx, LayerIteratorFunc iterator_func) {
  while (node) {
    if (!iterator_func(node, ctx)) {
      return false;
    };
    if (!layer_process_tree_level(node->first_child, ctx, iterator_func)) {
      return false;
    };

    node = node->next_sibling;
  }
  return true;
}

void layer_process_tree(Layer *node, void *ctx, LayerIteratorFunc iterator_func) {
  layer_process_tree_level(node, ctx, iterator_func);
}

inline static Layer __attribute__((always_inline)) *prv_layer_tree_traverse_next(Layer *stack[],
    int const stack_size, uint8_t *current_depth,
    const bool descend) {
  const Layer *top_of_stack = stack[*current_depth];

  // goto first child
  if (descend && top_of_stack->first_child) {
    if (*current_depth < stack_size-1) {
      return stack[++(*current_depth)] = top_of_stack->first_child;
    } else {
      PBL_LOG_WRN("layer stack exceeded (%d). Will skip rendering.", stack_size);
    }
  }

  // no children, try next sibling
  if (top_of_stack->next_sibling) {
    return stack[*current_depth] = top_of_stack->next_sibling;
  }

  // there are no more siblings
  // continue with siblings of parents/grandparents
  while (*current_depth > 0) {
    (*current_depth)--;
    const Layer *sibling = stack[*current_depth]->next_sibling;
    if (sibling) {
      return stack[*current_depth] = (Layer*)sibling;
    }
  }

  // no more siblings on root level of stack
  return NULL;
}

Layer *__layer_tree_traverse_next__test_accessor(Layer *stack[],
    int const max_depth, uint8_t *current_depth, const bool descend) {
  return prv_layer_tree_traverse_next(stack, max_depth, current_depth, descend);
}

void layer_render_tree(Layer *node, GContext *ctx) {
  // NOTE: make sure to restore ctx->draw_state before leaving this function
  const GDrawState root_draw_state = ctx->draw_state;
  uint8_t current_depth = 0;

  // We render our layout tree using a stack as opposed to using recursion to optimize for task
  // stack usage. We can't allocate this stack on the stack anymore without blowing our stack
  // up when doing a few common operations. We don't want to allocate this on the app heap as we
  // didn't before and that would cause less RAM to be available to apps after a firmware upgrade.
  Layer **stack;
  if (pebble_task_get_current() == PebbleTask_App) {
    stack = app_state_get_layer_tree_stack();
  } else {
    stack = kernel_applib_get_layer_tree_stack();
  }
  stack[0] = node;

  while (node) {
    bool descend = false;
    if (node->hidden) {
      goto node_hidden_do_not_descend;
    }
    // prepare draw_state for the current layer
    // it will not be stored and restored but recalculated from the root
    // for every layer
    for (unsigned int level = 0; level <= current_depth; level++) {
      const Layer *levels_layer = stack[level];
      if (levels_layer->clips) {
        const GRect levels_layer_frame_in_ctx_space = {
            .origin = {
                // drawing_box is expected to be setup as the bounds of the parent:
                .x = ctx->draw_state.drawing_box.origin.x + levels_layer->frame.origin.x,
                .y = ctx->draw_state.drawing_box.origin.y + levels_layer->frame.origin.y,
            },
            .size = levels_layer->frame.size,
        };
        grect_clip(&ctx->draw_state.clip_box, &levels_layer_frame_in_ctx_space);
      }

      // translate the drawing_box to the bounds of the layer:
      ctx->draw_state.drawing_box.origin.x +=
          levels_layer->frame.origin.x + levels_layer->bounds.origin.x;
      ctx->draw_state.drawing_box.origin.y +=
          levels_layer->frame.origin.y + levels_layer->bounds.origin.y;
      ctx->draw_state.drawing_box.size = levels_layer->bounds.size;
    }

    if (!grect_is_empty(&ctx->draw_state.clip_box)) {
      // call the current node's render procedure
      if (node->update_proc) {
        node->update_proc(node, ctx);
      }

      // if client has forgotten to release frame buffer
      if (ctx->lock) {
        graphics_release_frame_buffer(ctx, &ctx->dest_bitmap);
        APP_LOG(APP_LOG_LEVEL_WARNING,
            "Frame buffer was not released. "
            "Make sure to call graphics_release_frame_buffer before leaving update_proc.");
      }
      descend = true;
    }

node_hidden_do_not_descend:
    node = prv_layer_tree_traverse_next(stack, LAYER_TREE_STACK_SIZE, &current_depth, descend);

    ctx->draw_state = root_draw_state;
  }
}

void layer_property_changed_tree(Layer *node) {
  layer_process_tree(node, NULL, layer_property_changed_tree_node);
}

void layer_set_update_proc(Layer *layer, LayerUpdateProc update_proc) {
  PBL_ASSERTN(layer != NULL);
  layer->update_proc = update_proc;
}

void layer_set_frame(Layer *layer, const GRect *frame) {
  if (grect_equal(frame, &layer->frame)) {
    return;
  }
  const bool bounds_in_sync = gpoint_equal(&layer->bounds.origin, &GPointZero) &&
                              gsize_equal(&layer->bounds.size, &layer->frame.size);

  layer->frame = *frame;

  if (bounds_in_sync && !process_manager_compiled_with_legacy2_sdk()) {
    layer->bounds = (GRect){.size = layer->frame.size};
  } else {
    // Legacy 2.x behavior needed for ScrollLayer

    // Grow the bounds if it doesn't cover the area that the frame is showing.
    // This is not a necessity, but supposedly a handy thing.
    const int16_t visible_width = layer->bounds.size.w + layer->bounds.origin.x;
    const int16_t visible_height = layer->bounds.size.h + layer->bounds.origin.y;
    if (frame->size.w > visible_width ||
        frame->size.h > visible_height) {
      layer->bounds.size.w += MAX(frame->size.w - visible_width, 0);
      layer->bounds.size.h += MAX(frame->size.h - visible_height, 0);
    }
  }

  layer_mark_dirty(layer);
}

void layer_set_frame_by_value(Layer *layer, GRect frame) {
  layer_set_frame(layer, &frame);
}

void layer_get_frame(const Layer *layer, GRect *frame) {
  *frame = layer->frame;
}

GRect layer_get_frame_by_value(const Layer *layer) {
  GRect frame;
  layer_get_frame(layer, &frame);
  return frame;
}

void layer_set_bounds(Layer *layer, const GRect *bounds) {
  if (grect_equal(bounds, &layer->bounds)) {
    return;
  }
  layer->bounds = *bounds;
  layer_mark_dirty(layer);
}

void layer_set_bounds_by_value(Layer *layer, GRect bounds) {
  layer_set_bounds(layer, &bounds);
}

void layer_get_bounds(const Layer *layer, GRect *bounds) {
  *bounds = layer->bounds;
}

GRect layer_get_bounds_by_value(const Layer *layer) {
  GRect bounds;
  layer_get_bounds(layer, &bounds);
  return bounds;
}

void layer_get_unobstructed_bounds(const Layer *layer, GRect *bounds_out) {
  PBL_ASSERT_TASK(PebbleTask_App);
  if (!layer || !bounds_out) {
    return;
  }
  GRect area;
  unobstructed_area_service_get_area(app_state_get_unobstructed_area_state(), &area);
  // Convert the area from screen coordinates to layer coordinates
  gpoint_sub_eq(&area.origin, layer_convert_point_to_screen(layer->parent, GPointZero));
  layer_get_bounds(layer, bounds_out);
  grect_clip(bounds_out, &area);
}

GRect layer_get_unobstructed_bounds_by_value(const Layer *layer) {
  GRect bounds;
  layer_get_unobstructed_bounds(layer, &bounds);
  return bounds;
}

#ifdef CONFIG_TOUCH
static bool prv_register_recognizer_cb(Recognizer *recognizer, void *context) {
  recognizer_manager_register_recognizer(context, recognizer);
  return true;
}
#endif

//! Sets the window on the layer and on all of its children
static void layer_set_window(Layer *layer, Window *window) {
  layer->window = window;
#ifdef CONFIG_TOUCH
  // Register recognizers that were attached while the layer had no window (NULL manager is a no-op)
  recognizer_list_iterate(&layer->recognizer_list, prv_register_recognizer_cb,
                          window_get_recognizer_manager(window));
#endif
  Layer *child = layer->first_child;
  while (child) {
    layer_set_window(child, window);
    child = child->next_sibling;
  }
}

struct Window *layer_get_window(const Layer *layer) {
  if (layer) {
    return layer->window;
  }
  return NULL;
}

void layer_remove_from_parent(Layer *child) {
  if (!child || child->parent == NULL) {
    return;
  }
#ifdef CONFIG_TOUCH
  // Must run while child->window is still set (cleared below). A subtree that leaves the
  // tree mid-gesture must drop the recognizer manager's active-layer latch: the layers may
  // be freed right after removal without a layer_deinit (e.g. the swap-layer layout path),
  // leaving the latch dangling.
  prv_invalidate_manager_active_layer(child);
#endif
  if (child->parent->window) {
    window_schedule_render(child->parent->window);
  }
  Layer *node = child->parent->first_child;
  if (node == child) {
    child->parent->first_child = node->next_sibling;
  } else {
    while (node->next_sibling != child) {
      node = node->next_sibling;
    }
    node->next_sibling = child->next_sibling;
  }
  child->parent = NULL;
  layer_set_window(child, NULL);
  child->next_sibling = NULL;
}

void layer_remove_child_layers(Layer *parent) {
  Layer *child = parent->first_child;
  while (child) {
    // Get the reference to the next now; layer_remove_from_parent will unlink them.
    Layer *next_sibling = child->next_sibling;
    layer_remove_from_parent(child);
    child = next_sibling;
  };
}

void layer_add_child(Layer *parent, Layer *child) {
  PBL_ASSERTN(parent != NULL);
  PBL_ASSERTN(child != NULL);
  if (child->parent) {
    layer_remove_from_parent(child);
  }
  PBL_ASSERTN(child->next_sibling == NULL);
  child->parent = parent;
  layer_set_window(child, parent->window);
  if (child->window) {
    window_schedule_render(child->window);
  }
  Layer *sibling = parent->first_child;
  if (sibling == NULL) {
    parent->first_child = child;
    return;
  }
  for (;;) {
    // Prevent setting the child to point to itself, causing infinite loop the next time this is
    // called
    if (sibling == child) {
      PBL_LOG_DBG("Layer has already been added to this parent!");
      return;
    }

    if (!sibling->next_sibling) {
      break;
    }
    sibling = sibling->next_sibling;
  }
  sibling->next_sibling = child;
}

// Below means higher up in the hierarchy so it gets drawn earlier,
// and as a result the one below gets occluded by what's draw on top of it.
void layer_insert_below_sibling(Layer *layer_to_insert, Layer *below_layer) {
  if (below_layer->parent == NULL) {
    return;
  }
  if (layer_to_insert->parent) {
    layer_remove_from_parent(layer_to_insert);
  }
  PBL_ASSERTN(layer_to_insert->next_sibling == NULL);
  layer_to_insert->parent = below_layer->parent;
  layer_set_window(layer_to_insert, below_layer->window);
  if (layer_to_insert->window) {
    window_schedule_render(layer_to_insert->window);
  }
  Layer *prev_sibling = below_layer->parent->first_child;
  if (below_layer == prev_sibling) {
    below_layer->parent->first_child = layer_to_insert;
  } else {
    while (prev_sibling->next_sibling != below_layer) {
      prev_sibling = prev_sibling->next_sibling;
    }
    prev_sibling->next_sibling = layer_to_insert;
  }
  layer_to_insert->next_sibling = below_layer;
}

// Above means lower down in the hierarchy so it gets drawn later,
// and as a result the drawn on top of what's below it.
void layer_insert_above_sibling(Layer *layer_to_insert, Layer *above_layer) {
  if (above_layer->parent == NULL) {
    return;
  }
  if (layer_to_insert->parent) {
    layer_remove_from_parent(layer_to_insert);
  }
  PBL_ASSERTN(layer_to_insert->next_sibling == NULL);
  layer_to_insert->parent = above_layer->parent;
  layer_set_window(layer_to_insert, above_layer->window);
  if (layer_to_insert->window) {
    window_schedule_render(layer_to_insert->window);
  }
  Layer *old_next_sibling = above_layer->next_sibling;
  above_layer->next_sibling = layer_to_insert;
  layer_to_insert->next_sibling = old_next_sibling;
}

void layer_set_hidden(Layer *layer, bool hidden) {
  if (hidden == layer->hidden) {
    return;
  }
  layer->hidden = hidden;
  if (layer->parent) {
    layer_mark_dirty(layer->parent);
  }
}

bool layer_get_hidden(const Layer *layer) {
  return layer->hidden;
}

void layer_set_clips(Layer *layer, bool clips) {
  if (clips == layer->clips) {
    return;
  }
  layer->clips = clips;
  layer_mark_dirty(layer);
}

bool layer_get_clips(const Layer *layer) {
  return layer->clips;
}

void* layer_get_data(const Layer *layer) {
  if (!layer->has_data) {
    PBL_LOG_ERR("Layer was not allocated with a data region.");
    return NULL;
  }
  return ((DataLayer *)layer)->data;
}

// TODO: PBL-25368 cover the following "convert coordinates to screen space" functions with tests

GPoint layer_convert_point_to_screen(const Layer *layer, GPoint point) {
  while (layer) {
    // don't consider window's root layer's frame/bounds
    // and no, we don't need to check for l->window != NULL as &l->window->layer is just an offset
    // (an offset of 0 to be precise)
    if (&layer->window->layer == layer) {
      break;
    }
    // follow how the drawing_box is computed to obtain the global frame
    // see \ref layer_render_tree
    point.x += layer->frame.origin.x + layer->bounds.origin.x;
    point.y += layer->frame.origin.y + layer->bounds.origin.y;
    layer = layer->parent;
  }

  return point;
}

GRect layer_convert_rect_to_screen(const Layer *layer, GRect rect) {
  return (GRect){
    .origin = layer_convert_point_to_screen(layer, rect.origin),
    .size = rect.size,
  };
}

void layer_get_global_frame(const Layer *layer, GRect *global_frame_out) {
  *global_frame_out = (GRect) {
    .origin = layer_convert_point_to_screen(layer, GPointZero),
    .size = layer->frame.size,
  };
}

bool layer_contains_point(const Layer *layer, const GPoint *point) {
  if (!layer || !point) {
    return false;
  }
#ifdef CONFIG_TOUCH
  if (layer->contains_point_override) {
    return layer->contains_point_override(layer, point);
  }
#endif
  return grect_contains_point(&layer->frame, point);
}

void layer_set_contains_point_override(Layer *layer, LayerContainsPointOverride override) {
  if (!layer) {
    return;
  }
#ifdef CONFIG_TOUCH
  layer->contains_point_override = override;
#endif
}

typedef struct LayerContainsPointIterCtx {
  const Layer *layer;
  GPoint pos;
} LayerTouchIteratorCtx;

// Recursively search the layer tree for a layer that fulfills the following criteria:
//   - contains the specified point
//   - is the last sibling added to the parent layer, if more than one sibling contains the point
//   - does not have any children that also contain the point
// This function returns true to indicate that the search should continue, and false to indicate
// that a layer has been found and that the search should stop
static bool prv_find_layer_containing_point(const Layer *node, LayerTouchIteratorCtx *iter_ctx) {
  while (node) {
    if (layer_contains_point(node, &iter_ctx->pos)) {
      iter_ctx->layer = node;
      if (!node->first_child && !node->next_sibling) {
        return false;
      }

      iter_ctx->pos = gpoint_sub(iter_ctx->pos, node->bounds.origin);
      if (!prv_find_layer_containing_point(node->first_child, iter_ctx)) {
        return false;
      };
      iter_ctx->pos = gpoint_add(iter_ctx->pos, node->bounds.origin);
    }

    node = node->next_sibling;
  }
  return true;
}

MOCKABLE Layer *layer_find_layer_containing_point(const Layer *node, const GPoint *point) {
  if (!node || !point) {
    return NULL;
  }
  LayerTouchIteratorCtx iter_ctx = {
    .pos = *point,
  };
  gpoint_sub(iter_ctx.pos, node->frame.origin);
  prv_find_layer_containing_point(node, &iter_ctx);
  return (Layer *)iter_ctx.layer;
}

void layer_attach_recognizer(Layer *layer, Recognizer *recognizer) {
#ifdef CONFIG_TOUCH
  if (!layer || !recognizer) {
    return;
  }
  recognizer_manager_register_recognizer(window_get_recognizer_manager(layer_get_window(layer)),
                                         recognizer);
  recognizer_add_to_list(recognizer, &layer->recognizer_list);
#endif
}

void layer_detach_recognizer(Layer *layer, Recognizer *recognizer) {
#ifdef CONFIG_TOUCH
  if (!layer || !recognizer) {
    return;
  }
  recognizer_remove_from_list(recognizer, &layer->recognizer_list);
  recognizer_manager_deregister_recognizer(window_get_recognizer_manager(layer_get_window(layer)),
                                           recognizer);
#endif
}

RecognizerList *layer_get_recognizer_list(const Layer *layer) {
#ifdef CONFIG_TOUCH
  if (!layer) {
    return NULL;
  }
  return (RecognizerList *)&layer->recognizer_list;
#else
  return NULL;
#endif
}

bool layer_is_descendant(const Layer *layer, const Layer *potential_ancestor) {
  if (!layer || !potential_ancestor) {
    return false;
  }
  Layer *parent = layer->parent;
  while (parent) {
    if (parent == potential_ancestor) {
      return true;
    }
    parent = parent->parent;
  }
  return false;
}
