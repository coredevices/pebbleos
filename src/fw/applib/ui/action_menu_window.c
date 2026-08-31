/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "action_menu_window.h"
#include "action_menu_window_private.h"

#include "applib/applib_malloc.auto.h"
#include "applib/ui/status_bar_layer.h"
#include "applib/ui/window.h"
#include "applib/ui/window_stack.h"
#include "process_state/app_state/app_state.h"

#define ACTION_MENU_DEFAULT_BACKGROUND_COLOR GColorWhite

static const int IN_OUT_ANIMATION_DURATION = 200;

static void prv_invoke_will_close(ActionMenu *action_menu) {
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  if (data->config.will_close) {
    data->config.will_close(action_menu,
                            data->performed_item,
                            data->config.context);
  }
}

static void prv_invoke_did_close(ActionMenu *action_menu) {
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  if (data->config.did_close) {
    data->config.did_close(action_menu,
                           data->performed_item,
                           data->config.context);
  }
}

static void prv_action_window_push(WindowStack *window_stack, ActionMenu *action_menu,
                                   bool animated) {
  window_stack_push(window_stack, &action_menu->window, animated);
}

static void prv_action_window_pop(bool animated, ActionMenu *action_menu) {
  prv_invoke_will_close(action_menu);
  window_stack_remove(&action_menu->window, animated);
}

static void prv_action_window_insert_below(ActionMenu *action_menu, Window *window) {
  window_stack_insert_next(action_menu->window.parent_window_stack, window);
}

static void prv_remove_window(Window *window) {
  window_stack_remove(window, false /* animated */);
}

static void prv_action_callback(const ActionMenuItem *item, void *context);
static void prv_action_menu_layer_selection_changed(const ActionMenuItem *item, void *context);

static void prv_invoke_level_selection_changed(ActionMenuData *data, const ActionMenuItem *item) {
  const ActionMenuLevel *cur_level = data->view_model.cur_level;
  if (cur_level->selection_changed) {
    cur_level->selection_changed(item, data->config.context);
  }
}

static void prv_set_action_menu_layer_callbacks(ActionMenuData *data,
                                                bool enable_selection_changed) {
  action_menu_layer_set_callbacks(&data->action_menu_layer, (ActionMenuLayerCallbacks) {
    .select = prv_action_callback,
    .selection_changed = enable_selection_changed ? prv_action_menu_layer_selection_changed : NULL,
  }, data);
}

static void prv_view_model_did_change(ActionMenuData *data) {
  ActionMenuViewModel *vm = &data->view_model;
  const ActionMenuLevel *cur_level = vm->cur_level;
  GRect frame = grect_inset(data->action_menu.window.layer.frame, vm->menu_insets);
  layer_set_frame(&data->action_menu_layer.layer, &frame);
  prv_set_action_menu_layer_callbacks(data, false);
  if (cur_level->display_mode == ActionMenuLevelDisplayModeThin) {
    action_menu_layer_set_items(&data->action_menu_layer, NULL, 0, 0, 0);
    action_menu_layer_set_short_items(&data->action_menu_layer,
                                      cur_level->items,
                                      cur_level->num_items,
                                      cur_level->default_selected_item);
  } else {
    action_menu_layer_set_short_items(&data->action_menu_layer, NULL, 0, 0);
    action_menu_layer_set_items(&data->action_menu_layer, cur_level->items, cur_level->num_items,
        cur_level->default_selected_item, cur_level->separator_index);
  }
  prv_set_action_menu_layer_callbacks(data, true);
  action_menu_layer_notify_selection_changed(&data->action_menu_layer);
  crumbs_layer_set_level(&data->crumbs_layer, vm->num_dots);
}

static void prv_next_level_anim_stopped(Animation *anim, bool finished, void *context) {
  // update the view model
  AnimationContext *anim_ctx = (AnimationContext*) context;
  ActionMenuData *data = window_get_user_data((Window*) anim_ctx->window);
  if (!data || !finished) {
    // We could have gotten cleaned up in the middle of an animation, bail
    applib_free(anim_ctx);
    return;
  }

  if (data->view_model.cur_level->parent_level == anim_ctx->next_level) {
    --data->view_model.num_dots;
  } else {
    ++data->view_model.num_dots;
  }
  data->view_model.cur_level = anim_ctx->next_level;

  // update the view
  prv_view_model_did_change(data);
  applib_free(anim_ctx);
}

static GEdgeInsets prv_action_menu_insets(Window *window) {
  const int crumbs_width = crumbs_layer_width();
  return (GEdgeInsets) {
      .top    = PBL_IF_RECT_ELSE(0, STATUS_BAR_LAYER_HEIGHT),
      .right  = PBL_IF_RECT_ELSE(0, crumbs_width),
      .bottom = PBL_IF_RECT_ELSE(0, STATUS_BAR_LAYER_HEIGHT),
      .left   = crumbs_width,
  };
}

static Animation* prv_create_content_in_animation(ActionMenuData *data,
    const ActionMenuLevel *level) {
  // animate the ease in of the new level
  const GRect window_frame = data->action_menu.window.layer.frame;
  const GEdgeInsets insets = prv_action_menu_insets(&data->action_menu.window);
  GRect stop = grect_inset(window_frame, insets);
  GRect start = stop;
  start.origin.x -= crumbs_layer_width();
  PropertyAnimation *prop_anim =
      property_animation_create_layer_frame((Layer *)&data->action_menu_layer,
                                            &start,
                                            &stop);
  Animation *content_in = property_animation_get_animation(prop_anim);
  animation_set_duration(content_in, IN_OUT_ANIMATION_DURATION);

  // animate the dots
  Animation *crumbs_anim = crumbs_layer_get_animation(&data->crumbs_layer);
  animation_set_duration(crumbs_anim, IN_OUT_ANIMATION_DURATION);
  // combine the two
  Animation *spawn_anim = animation_spawn_create(content_in, crumbs_anim, NULL);
  return spawn_anim;
}

static Animation* prv_create_content_out_animation(ActionMenuData *data,
    const ActionMenuLevel *level) {
  // animate the ease out of the current level
  GRect *start = &data->action_menu_layer.layer.frame;
  GRect stop = *start;
  stop.origin.x = crumbs_layer_width() - start->size.w;
  PropertyAnimation *prop_anim =
      property_animation_create_layer_frame((Layer *)&data->action_menu_layer, start, &stop);
  Animation *content_out = property_animation_get_animation(prop_anim);
  animation_set_duration(content_out, IN_OUT_ANIMATION_DURATION);
  AnimationHandlers anim_handlers = {
    .started = NULL,
    .stopped = prv_next_level_anim_stopped,
  };

  AnimationContext *anim_ctx = applib_type_malloc(AnimationContext);
  *anim_ctx  = (AnimationContext) {
    .window = &data->action_menu.window,
    .next_level = level,
  };
  animation_set_handlers(content_out, anim_handlers, anim_ctx);

  return content_out;
}

static void prv_set_level(ActionMenuData *data, const ActionMenuLevel *level) {
  if (animation_is_scheduled(data->level_change_anim)) {
    // We are already animating.
    return;
  }

  prv_invoke_level_selection_changed(data, NULL);

  Animation *content_out = prv_create_content_out_animation(data, level);
  Animation *content_in = prv_create_content_in_animation(data, level);

  data->level_change_anim = animation_sequence_create(content_out, content_in, NULL);
  animation_schedule(data->level_change_anim);
}

static void prv_action_callback(const ActionMenuItem *item, void *context) {
  ActionMenuData *data = context;
  ActionMenu *action_menu = &data->action_menu;
  if (item->is_leaf && item->perform_action) {
    item->perform_action(action_menu, item, data->config.context);
    data->performed_item = item;
    if (!data->frozen) {
      prv_action_window_pop(true /*animated*/, action_menu);
    }
  } else if (item->next_level) {
    prv_set_level(data, item->next_level);
  }
}

static void prv_action_menu_layer_selection_changed(const ActionMenuItem *item, void *context) {
  prv_invoke_level_selection_changed((ActionMenuData *)context, item);
}

static void prv_back_click_handler(ClickRecognizerRef recognizer, void *context) {
  ActionMenuData *data = context;

  if (animation_is_scheduled(data->level_change_anim)) {
    animation_set_elapsed(data->level_change_anim,
                          animation_get_duration(data->level_change_anim, true, true));
  }

  ActionMenuLevel *parent_level = data->view_model.cur_level->parent_level;
  if (parent_level) {
    prv_set_level(data, parent_level);
  } else {
    prv_action_window_pop(true, &data->action_menu);
  }
}

static void prv_click_config_provider(void *context) {
  ActionMenuData *data = context;
  action_menu_layer_click_config_provider(&data->action_menu_layer);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_back_click_handler);
  window_set_click_context(BUTTON_ID_BACK, data);
}

static void prv_action_window_load(Window *window) {
  ActionMenuData *data = window_get_user_data(window);
  // Init action menu layer
  ActionMenuLayer *action_menu_layer = &data->action_menu_layer;
  action_menu_layer_init(action_menu_layer, &GRectZero);
  prv_set_action_menu_layer_callbacks(data, true);
  action_menu_layer_set_align(action_menu_layer, data->config.align);
  // Init crumbs layer
  CrumbsLayer *crumbs_layer = &data->crumbs_layer;
  GRect frame = window_get_root_layer(window)->frame;
#if PBL_RECT
  // on round display, the layer fills a full circle
  // here (on rect) it's just a small vertical stripe on the left
  frame.size.w = crumbs_layer_width();
#endif
  crumbs_layer_init(&data->crumbs_layer, &frame, data->config.colors.background,
                    data->config.colors.foreground);
  // Add them to the tree
  layer_add_child(window_get_root_layer(window), (Layer *)action_menu_layer);
  layer_add_child(window_get_root_layer(window), (Layer *)crumbs_layer);
  // Click config
  window_set_click_config_provider_with_context(window, prv_click_config_provider, data);
  // Init the view model
  data->view_model = (ActionMenuViewModel) {
    .cur_level = data->config.root_level,
    .menu_insets = prv_action_menu_insets(window),
    .num_dots = 1,
  };
  prv_view_model_did_change(data);
}

static void prv_action_window_unload(Window *window) {
  ActionMenuData *data = window_get_user_data(window);

  // call did close callback so user can cleanup
  prv_invoke_did_close((ActionMenu *)window);

  // cleanup
  animation_unschedule(data->level_change_anim);
  action_menu_layer_deinit(&data->action_menu_layer);
  crumbs_layer_deinit(&data->crumbs_layer);
  applib_free(data);
}

static void prv_dummy_click_config(void *data) {
}

ActionMenuLevel *action_menu_get_root_level(ActionMenu *action_menu) {
  if (!action_menu) return NULL;
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  return (ActionMenuLevel *)data->config.root_level;
}

void *action_menu_get_context(ActionMenu *action_menu) {
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  return data->config.context;
}

void action_menu_freeze(ActionMenu *action_menu) {
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  window_set_click_config_provider(&action_menu->window, prv_dummy_click_config);
  data->frozen = true;
}

void action_menu_unfreeze(ActionMenu *action_menu) {
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  window_set_click_config_provider_with_context(&action_menu->window,
                                                prv_click_config_provider, data);
  data->frozen = false;
}

bool action_menu_is_frozen(ActionMenu *action_menu) {
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  return data->frozen;
}

void action_menu_close(ActionMenu *action_menu, bool animated) {
  prv_action_window_pop(animated, action_menu);
}

void action_menu_set_result_window(ActionMenu *action_menu, Window *result_window) {
  if (!action_menu) return;

  // remove existing result window
  ActionMenuData *data = window_get_user_data(&action_menu->window);
  if (data->result_window) {
    prv_remove_window(data->result_window);
  }

  // insert new result window
  if (result_window) {
    prv_action_window_insert_below(action_menu, result_window);
  }

  data->result_window = result_window;
}


void action_menu_set_align(ActionMenuConfig *config, ActionMenuAlign align) {
  if (!config) {
    return;
  }
  config->align = align;
}

ActionMenu *action_menu_open(WindowStack *window_stack, ActionMenuConfig *config) {
  ActionMenuData *data = applib_type_zalloc(ActionMenuData);
  data->config = *config;
#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 8
  // Apply defaults if client didn't assign foreground/background colors
  if (gcolor_is_invisible(data->config.colors.background)) {
    data->config.colors.background = ACTION_MENU_DEFAULT_BACKGROUND_COLOR;
  }
  if (gcolor_is_invisible((data->config.colors.foreground))) {
    data->config.colors.foreground = gcolor_legible_over(data->config.colors.background);
  }
#else
  data->config.colors.background = GColorLightGray;
  data->config.colors.foreground = GColorBlack;
#endif

  Window *window = &data->action_menu.window;
  window_init(window, WINDOW_NAME("Action Menu"));
  window_set_user_data(window, data);
  window_set_fullscreen(window, true);
  window_set_background_color(window, GColorBlack);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_action_window_load,
    .unload = prv_action_window_unload,
  });

  prv_action_window_push(window_stack, &data->action_menu, true /* animated */);

  return &data->action_menu;
}

ActionMenu *app_action_menu_open(ActionMenuConfig *config) {
  return action_menu_open(app_state_get_window_stack(), config);
}
