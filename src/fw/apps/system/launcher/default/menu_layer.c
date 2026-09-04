/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "menu_layer.h"

#include "app_glance_service.h"
#include "menu_layer_private.h"

#include "applib/graphics/gtypes.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/content_indicator.h"
#include "resource/resource_ids.auto.h"
#include "system/passert.h"
#include "shell/prefs.h"
#include "pbl/util/attributes.h"

#define LAUNCHER_MENU_LAYER_CONTENT_INDICATOR_LAYER_HEIGHT (32)
#define LAUNCHER_MENU_LAYER_GENERIC_APP_ICON (RESOURCE_ID_MENU_LAYER_GENERIC_WATCHAPP_ICON)

////////////////////////////
// Misc. callbacks/helpers

static void prv_launch_app_cb(void *data) {
  const AppInstallId app_install_id_to_launch = (AppInstallId)data;
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = app_install_id_to_launch,
    .common.reason = APP_LAUNCH_USER,
    .common.button = BUTTON_ID_SELECT,
  });
}

static void prv_launcher_menu_layer_mark_dirty(LauncherMenuLayer *launcher_menu_layer) {
  if (launcher_menu_layer) {
    layer_mark_dirty(menu_layer_get_layer(&launcher_menu_layer->menu_layer));
  }
}

//////////////////////////////////////
// LauncherAppGlanceService handlers

static void prv_glance_changed(void *context) {
  LauncherMenuLayer *launcher_menu_layer = context;
  prv_launcher_menu_layer_mark_dirty(launcher_menu_layer);
}

////////////////////////
// MenuLayer callbacks

static void prv_menu_layer_select(PBL_UNUSED MenuLayer *menu_layer, MenuIndex *cell_index,
                                  void *context) {
  LauncherMenuLayer *launcher_menu_layer = context;
  AppMenuDataSource *data_source = launcher_menu_layer->data_source;
  if (!data_source) {
    return;
  }

  Window *window = layer_get_window(launcher_menu_layer_get_layer(launcher_menu_layer));
  if (!window) {
    return;
  }
  // Disable all clicking on the window so the user can't scroll anymore
  window_set_click_config_provider(window, NULL);

  // Capture what app we should launch - we'll actually launch it as part of an app task callback
  // we register in our .draw_row callback so that we don't launch the app until after we finish
  // rendering the last frame of the menu layer; we need to do this because some clients (like the
  // normal firmware app launcher) rely on the display reflecting the final state of the launcher
  // when we launch an app (e.g. for compositor transition animations)
  AppMenuNode *node = app_menu_data_source_get_node_at_index(data_source, cell_index->row);
  PBL_ASSERTN(node);
  launcher_menu_layer->app_to_launch_after_next_render = node->install_id;

  // Now kick off a render of the last frame of the menu layer; note that any menu layer scroll or
  // selection animation has already been advanced to completion by the menu layer before it called
  // this select click handler
  prv_launcher_menu_layer_mark_dirty(launcher_menu_layer);
}

static uint16_t prv_menu_layer_get_num_rows(PBL_UNUSED MenuLayer *menu_layer,
                                            PBL_UNUSED uint16_t section_index, void *context) {
  LauncherMenuLayer *launcher_menu_layer = context;
  AppMenuDataSource *data_source = launcher_menu_layer->data_source;
  return data_source ? app_menu_data_source_get_count(data_source) : (uint16_t)0;
}

static void prv_menu_layer_draw_row(GContext* ctx, const Layer *cell_layer, MenuIndex *cell_index,
                                    void *context) {
  LauncherMenuLayer *launcher_menu_layer = context;
  AppMenuDataSource *data_source = launcher_menu_layer->data_source;
  if (!data_source) {
    return;
  }

  AppMenuNode *node = app_menu_data_source_get_node_at_index(data_source, cell_index->row);

  const GRect *cell_layer_bounds = &cell_layer->bounds;
  const bool is_highlighted = menu_cell_layer_is_highlighted(cell_layer);

  // Get global frame for continuous arc effect during scroll animations
  // Include the animation offset for smooth animation during center-focused scroll
  GRect global_frame;
  layer_get_global_frame((Layer *)cell_layer, &global_frame);
  const int16_t animation_offset = launcher_menu_layer->menu_layer.animation.cell_content_origin_offset_y;
  const int16_t screen_center_y = global_frame.origin.y + animation_offset + (global_frame.size.h / 2);

  launcher_app_glance_service_draw_glance_for_app_node(&launcher_menu_layer->glance_service,
                                                       ctx, cell_layer_bounds, is_highlighted,
                                                       screen_center_y, node);

  // If we should launch an app after this render, push a callback to do that on the app task
  if (launcher_menu_layer->app_to_launch_after_next_render != INSTALL_ID_INVALID) {
    const AppInstallId app_to_launch_install_id =
        launcher_menu_layer->app_to_launch_after_next_render;
    // Resetting this here in combination with disabling user input in the select click handler
    // (the only place that sets this field) ensures we only do this once
    launcher_menu_layer->app_to_launch_after_next_render = INSTALL_ID_INVALID;
    process_manager_send_callback_event_to_process(PebbleTask_App, prv_launch_app_cb,
                                                   (void *)(uintptr_t)app_to_launch_install_id);
  }
}

static int16_t prv_menu_layer_get_cell_height(PBL_UNUSED MenuLayer *menu_layer,
                                              PBL_UNUSED MenuIndex *cell_index, PBL_UNUSED void *context) {
#if PBL_RECT
  return LAUNCHER_MENU_LAYER_CELL_RECT_CELL_HEIGHT;
#elif PBL_ROUND
  return menu_layer_is_index_selected(menu_layer, cell_index) ?
      LAUNCHER_MENU_LAYER_CELL_ROUND_FOCUSED_CELL_HEIGHT :
      LAUNCHER_MENU_LAYER_CELL_ROUND_UNFOCUSED_CELL_HEIGHT;
#else
#error "Unknown display shape type"
#endif
}

static void prv_play_glance_for_row(LauncherMenuLayer *launcher_menu_layer, uint16_t row) {
  if (!launcher_menu_layer || !launcher_menu_layer->selection_animations_enabled) {
    return;
  }

  // Get the app menu node for the glance that is about to be selected
  AppMenuDataSource *data_source = launcher_menu_layer->data_source;
  AppMenuNode *node = app_menu_data_source_get_node_at_index(data_source, row);

  // Instruct the launcher app glance service to play the glance for the node
  launcher_app_glance_service_play_glance_for_app_node(&launcher_menu_layer->glance_service, node);
}

static void prv_menu_layer_selection_will_change(MenuLayer *PBL_UNUSED menu_layer, MenuIndex *new_index,
                                                 MenuIndex PBL_UNUSED old_index, void *context) {
  LauncherMenuLayer *launcher_menu_layer = context;
  prv_play_glance_for_row(launcher_menu_layer, new_index->row);
}

T_STATIC void prv_launcher_menu_layer_set_selection_index(LauncherMenuLayer *launcher_menu_layer,
                                                          uint16_t index, MenuRowAlign row_align,
                                                          bool animated) {
  if (!launcher_menu_layer || !launcher_menu_layer->data_source) {
    return;
  }

  const MenuIndex new_selected_menu_index = MenuIndex(0, index);
  menu_layer_set_selected_index(&launcher_menu_layer->menu_layer, new_selected_menu_index,
                                row_align, animated);
  prv_play_glance_for_row(launcher_menu_layer, index);
}

////////////////////////
// Public API

void launcher_menu_layer_init(LauncherMenuLayer *launcher_menu_layer,
                              AppMenuDataSource *data_source) {
  if (!launcher_menu_layer) {
    return;
  }

  // We force the launcher menu layer to be the size of the display so that the calculation of
  // LAUNCHER_MENU_LAYER_NUM_VISIBLE_ROWS in launcher_menu_layer_private.h is valid
  const GRect frame = DISP_FRAME;

  launcher_menu_layer->title_font = fonts_get_system_font(LAUNCHER_MENU_LAYER_TITLE_FONT);
  launcher_menu_layer->subtitle_font = fonts_get_system_font(LAUNCHER_MENU_LAYER_SUBTITLE_FONT);

  Layer *container_layer = &launcher_menu_layer->container_layer;
  layer_init(container_layer, &frame);

  launcher_menu_layer->data_source = data_source;

  GRect menu_layer_frame = frame;
#if PBL_ROUND
  const int top_bottom_inset =
      (frame.size.h - LAUNCHER_MENU_LAYER_CELL_ROUND_FOCUSED_CELL_HEIGHT -
          (2 * LAUNCHER_MENU_LAYER_NUM_UNFOCUSED_ROWS_PER_SIDE *
           LAUNCHER_MENU_LAYER_CELL_ROUND_UNFOCUSED_CELL_HEIGHT)) / 2;
  const GEdgeInsets menu_layer_frame_insets = GEdgeInsets(top_bottom_inset, 0);
  menu_layer_frame = grect_inset(menu_layer_frame, menu_layer_frame_insets);
#endif

  MenuLayer *menu_layer = &launcher_menu_layer->menu_layer;
  menu_layer_init(menu_layer, &menu_layer_frame);
  GColor highlight_bg = shell_prefs_get_theme_highlight_color();
  menu_layer_set_highlight_colors(menu_layer,
                                  highlight_bg,
                                  gcolor_legible_over(highlight_bg));
  menu_layer_pad_bottom_enable(menu_layer, false);
  menu_layer_set_callbacks(menu_layer, launcher_menu_layer, &(MenuLayerCallbacks) {
    .get_num_rows = prv_menu_layer_get_num_rows,
    .draw_row = prv_menu_layer_draw_row,
    .select_click = prv_menu_layer_select,
    .get_cell_height = prv_menu_layer_get_cell_height,
    .selection_will_change = prv_menu_layer_selection_will_change,
  });
  menu_layer_set_scroll_wrap_around(menu_layer, shell_prefs_get_menu_scroll_wrap_around_enable());
  menu_layer_set_scroll_vibe_on_wrap(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
  menu_layer_set_scroll_vibe_on_blocked(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);

  // Only setup the content indicator on round
#if PBL_ROUND
  const GSize arrow_layer_frame_size = GSize(frame.size.w,
                                             LAUNCHER_MENU_LAYER_CONTENT_INDICATOR_LAYER_HEIGHT);
  const GRect up_arrow_layer_frame = (GRect) {
    .size = arrow_layer_frame_size,
  };
  Layer *up_arrow_layer = &launcher_menu_layer->up_arrow_layer;
  layer_init(up_arrow_layer, &up_arrow_layer_frame);
  layer_add_child(container_layer, up_arrow_layer);

  const int16_t down_arrow_layer_frame_origin_y =
      (int16_t)(frame.size.h - LAUNCHER_MENU_LAYER_CONTENT_INDICATOR_LAYER_HEIGHT);

  const GRect down_arrow_layer_frame =
      grect_inset(frame, GEdgeInsets(down_arrow_layer_frame_origin_y, 0, 0, 0));
  Layer *down_arrow_layer = &launcher_menu_layer->down_arrow_layer;
  layer_init(down_arrow_layer, &down_arrow_layer_frame);
  layer_add_child(container_layer, down_arrow_layer);

  ContentIndicator *content_indicator =
      scroll_layer_get_content_indicator(&menu_layer->scroll_layer);
  ContentIndicatorConfig content_indicator_config = (ContentIndicatorConfig) {
    .layer = up_arrow_layer,
    .colors.background = GColorWhite,
    .colors.foreground = GColorDarkGray,
  };
  content_indicator_configure_direction(content_indicator, ContentIndicatorDirectionUp,
                                        &content_indicator_config);
  content_indicator_config.layer = down_arrow_layer;
  content_indicator_configure_direction(content_indicator, ContentIndicatorDirectionDown,
                                        &content_indicator_config);
#endif

  // Wait to add the menu layer until after we might have added the content indicators because
  // the indicator arrows only get positioned properly if their layers overlap with the menu layer's
  // edges
  layer_add_child(container_layer, menu_layer_get_layer(menu_layer));

  launcher_app_glance_service_init(&launcher_menu_layer->glance_service,
                                   LAUNCHER_MENU_LAYER_GENERIC_APP_ICON);
  const LauncherAppGlanceServiceHandlers glance_handlers = (LauncherAppGlanceServiceHandlers) {
    .glance_changed = prv_glance_changed,
  };
  launcher_app_glance_service_set_handlers(&launcher_menu_layer->glance_service,
                                           &glance_handlers, launcher_menu_layer);

  // Select the visually first item from the top
  const uint16_t first_index = 0;
  const bool animated = false;
  prv_launcher_menu_layer_set_selection_index(launcher_menu_layer, first_index, MenuRowAlignBottom,
                                              animated);
}

Layer *launcher_menu_layer_get_layer(LauncherMenuLayer *launcher_menu_layer) {
  if (!launcher_menu_layer) {
    return NULL;
  }

  return &launcher_menu_layer->container_layer;
}

void launcher_menu_layer_set_click_config_onto_window(LauncherMenuLayer *launcher_menu_layer,
                                                      Window *window) {
  if (!launcher_menu_layer || !window) {
    return;
  }

  menu_layer_set_click_config_onto_window(&launcher_menu_layer->menu_layer, window);
}

void launcher_menu_layer_reload_data(LauncherMenuLayer *launcher_menu_layer) {
  if (!launcher_menu_layer) {
    return;
  }

  menu_layer_reload_data(&launcher_menu_layer->menu_layer);
}

void launcher_menu_layer_set_selection_state(LauncherMenuLayer *launcher_menu_layer,
                                             const LauncherMenuLayerSelectionState *new_state) {
  if (!launcher_menu_layer || !launcher_menu_layer->data_source || !new_state) {
    return;
  }

  const bool animated = false;

  prv_launcher_menu_layer_set_selection_index(launcher_menu_layer, new_state->row_index,
                                              MenuRowAlignNone, animated);

  const GPoint new_scroll_offset = GPoint(0, new_state->scroll_offset_y);
  scroll_layer_set_content_offset(&launcher_menu_layer->menu_layer.scroll_layer, new_scroll_offset,
                                  animated);
}

void launcher_menu_layer_get_selection_vertical_range(const LauncherMenuLayer *launcher_menu_layer,
                                                      GRangeVertical *vertical_range_out) {
  if (!launcher_menu_layer || !vertical_range_out) {
    return;
  }

  GRect selection_global_rect;
  layer_get_global_frame(&launcher_menu_layer->menu_layer.inverter.layer, &selection_global_rect);

  *vertical_range_out = (GRangeVertical) {
    .origin_y = selection_global_rect.origin.y,
    .size_h = selection_global_rect.size.h,
  };
}

void launcher_menu_layer_get_selection_state(const LauncherMenuLayer *launcher_menu_layer,
                                             LauncherMenuLayerSelectionState *state_out) {
  if (!launcher_menu_layer || !launcher_menu_layer->data_source || !state_out) {
    return;
  }

  const MenuLayer *menu_layer = &launcher_menu_layer->menu_layer;
  const ScrollLayer *scroll_layer = &menu_layer->scroll_layer;

  *state_out = (LauncherMenuLayerSelectionState) {
    .row_index = menu_layer_get_selected_index(menu_layer).row,
    // This cast is required because this ScrollLayer function's argument isn't const
    .scroll_offset_y = scroll_layer_get_content_offset((ScrollLayer *)scroll_layer).y,
  };
}

void launcher_menu_layer_set_selection_animations_enabled(LauncherMenuLayer *launcher_menu_layer,
                                                          bool enabled) {
  if (!launcher_menu_layer) {
    return;
  }
  launcher_menu_layer->selection_animations_enabled = enabled;
  if (enabled) {
    const MenuIndex selected_index =
        menu_layer_get_selected_index(&launcher_menu_layer->menu_layer);
    prv_play_glance_for_row(launcher_menu_layer, selected_index.row);
  } else {
    launcher_app_glance_service_rewind_current_glance(&launcher_menu_layer->glance_service);
  }
}

void launcher_menu_layer_deinit(LauncherMenuLayer *launcher_menu_layer) {
  if (!launcher_menu_layer) {
    return;
  }

  launcher_app_glance_service_deinit(&launcher_menu_layer->glance_service);
  menu_layer_deinit(&launcher_menu_layer->menu_layer);

#if PBL_ROUND
  layer_deinit(&launcher_menu_layer->up_arrow_layer);
  layer_deinit(&launcher_menu_layer->down_arrow_layer);
#endif
  layer_deinit(&launcher_menu_layer->container_layer);
}
