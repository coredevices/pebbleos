/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pin_window.h"

#include "applib/ui/action_button.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/modals/modal_manager.h"

void timeline_pin_window_set_item(TimelinePinWindow *pin_window, TimelineItem *item,
                                  time_t current_day) {
  timeline_layout_init_info(&pin_window->info, item, current_day);
  timeline_item_layer_set_item(&pin_window->item_detail_layer, item, &pin_window->info);
}

static Animation *prv_create_bounds_origin_animation(
    TimelinePinWindow *pin_window, GPoint *from_origin, GPoint *to_origin) {
  Animation *animation = property_animation_get_animation(
      property_animation_create_bounds_origin(&pin_window->layer, from_origin, to_origin));
  animation_set_duration(animation, TIMELINE_CARD_TRANSITION_MS / 2);
  return animation;
}

static void prv_pin_window_load(Window *window) {
  TimelinePinWindow *pin_window = window_get_user_data(window);
  Layer *window_layer = window_get_root_layer(window);
  layer_init(&pin_window->layer, &window_layer->bounds);
  layer_add_child(window_layer, &pin_window->layer);

  TimelineItemLayer *item_layer = &pin_window->item_detail_layer;
  timeline_item_layer_set_click_config_onto_window(item_layer, window);
  layer_add_child(&pin_window->layer, &item_layer->layer);

  layer_init(&pin_window->action_button_layer, &window_layer->bounds);
  layer_set_clips(&pin_window->action_button_layer, false);
  pin_window->action_button_layer.update_proc = action_button_update_proc;
  layer_add_child(&pin_window->layer, &pin_window->action_button_layer);

  StatusBarLayer *status_layer = &pin_window->status_layer;
  status_bar_layer_init(status_layer);
  status_bar_layer_set_separator_mode(status_layer, StatusBarLayerSeparatorModeNone);
  layer_add_child(&pin_window->layer, &status_layer->layer);

  const LayoutColors *colors =
      layout_get_colors((LayoutLayer *)pin_window->item_detail_layer.timeline_layout);
  status_bar_layer_set_colors(status_layer, colors->bg_color, colors->primary_color);
  window_set_background_color(window, colors->bg_color);

  // bounce back from the right
  GPoint from_origin = { -TIMELINE_CARD_MARGIN, 0 };
  Animation *animation = prv_create_bounds_origin_animation(pin_window, &from_origin, NULL);
  animation_schedule(animation);
}

static void prv_pin_window_unload(Window *window) {
  TimelinePinWindow *pin_window = window_get_user_data(window);
  timeline_item_layer_deinit(&pin_window->item_detail_layer);
  status_bar_layer_deinit(&pin_window->status_layer);
  layer_deinit(&pin_window->action_button_layer);
  layer_deinit(&pin_window->layer);
}

static void prv_pop_animation_stopped(Animation *animation, bool finished, void *context) {
  TimelinePinWindow *pin_window = animation_get_context(animation);
  prv_pin_window_unload(&pin_window->window);
}

void timeline_pin_window_pop(TimelinePinWindow *pin_window) {
  Window *window = &pin_window->window;

  // delay window unload until the end of the animation
  window_set_window_handlers(window, &(WindowHandlers) {});
  window_stack_remove(window, false);

  // animate the pop by using the new top most window
  Window *other_window = app_window_stack_get_top_window();
  layer_add_child(&other_window->layer, &pin_window->layer);

  // animate the card layout to the right
  GPoint to_origin = { pin_window->layer.bounds.size.w, 0 };
  Animation *animation = prv_create_bounds_origin_animation(pin_window, NULL, &to_origin);
  animation_set_custom_interpolation(animation, interpolate_moook);
  animation_set_handlers(animation, (AnimationHandlers) {
    .stopped = prv_pop_animation_stopped,
  }, pin_window);
  animation_schedule(animation);

  pin_window->pop_animation = animation;
}

void timeline_pin_window_init(TimelinePinWindow *pin_window, TimelineItem *item,
                              time_t current_day) {
  if (pin_window->pop_animation) {
    animation_unschedule(pin_window->pop_animation);
  }

  Window *window = &pin_window->window;
  window_init(window, WINDOW_NAME("Pin"));
  window_set_user_data(window, pin_window);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_pin_window_load,
    .unload = prv_pin_window_unload,
  });

  TimelineItemLayer *layer = &pin_window->item_detail_layer;
  GRect frame = window->layer.bounds;
  frame.origin.y += STATUS_BAR_LAYER_HEIGHT;
  frame.size.h -= STATUS_BAR_LAYER_HEIGHT;
  timeline_item_layer_init(layer, &frame);
  timeline_pin_window_set_item(pin_window, item, current_day);

  const LayoutColors *colors = layout_get_colors((LayoutLayer *)layer->timeline_layout);
  window_set_background_color(window, colors->bg_color);
}

static void prv_pin_window_unload_modal(Window *window) {
  TimelinePinWindow *pin_window = (TimelinePinWindow *)window;
  event_service_client_unsubscribe(&pin_window->blobdb_event_info);

  timeline_item_destroy(pin_window->item_detail_layer.item);
  prv_pin_window_unload(window);
  kernel_free(window);
}

static void prv_blobdb_event_handler(PebbleEvent *event, void *context) {
  TimelinePinWindow *pin_window = context;
  PebbleBlobDBEvent *blobdb_event = &event->blob_db;
  if (blobdb_event->db_id != BlobDBIdPins) {
    // we only care about pins
    return;
  }

  BlobDBEventType type = blobdb_event->type;
  Uuid *id = (Uuid *)blobdb_event->key;
  if (type == BlobDBEventTypeDelete) {
    if (uuid_equal(id, &pin_window->item_detail_layer.item->header.id)) {
      window_stack_remove((Window *)pin_window, true /* animated */);
    }
  }
}

void timeline_pin_window_push_modal(TimelineItem *item) {
  TimelinePinWindow *pin_window = kernel_zalloc_check(sizeof(TimelinePinWindow));
  timeline_pin_window_init(pin_window, item, time_util_get_midnight_of(rtc_get_time()));
  window_set_window_handlers((Window *)pin_window, &(WindowHandlers) {
    .load = prv_pin_window_load,
    .unload = prv_pin_window_unload_modal,
  });

  // Subscribe to pin removal events (handled by timeline app when not modal)
  pin_window->blobdb_event_info = (EventServiceInfo) {
    .type = PEBBLE_BLOBDB_EVENT,
    .handler = prv_blobdb_event_handler,
    .context = pin_window,
  };
  event_service_client_subscribe(&pin_window->blobdb_event_info);

  const bool animated = true;
  modal_window_push((Window *)pin_window, ModalPriorityNotification, animated);
}
