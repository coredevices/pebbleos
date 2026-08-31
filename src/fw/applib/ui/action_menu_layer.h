/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "action_menu_window.h"
#include "layer.h"
#include "menu_layer.h"

#include "applib/graphics/graphics.h"
#include "applib/ui/animation.h"
#include "applib/ui/window_private.h"
#include "system/passert.h"

typedef void (*ActionMenuLayerCallback)(const ActionMenuItem *item, void *context);

typedef struct {
  ActionMenuLayerCallback select;
  ActionMenuLayerCallback selection_changed;
} ActionMenuLayerCallbacks;

typedef struct {
  ActionMenuAlign align;
  GFont font;
  int16_t *item_heights;
} ActionMenuLayoutCache;

typedef struct {
  struct Animation *animation;

  int16_t top_offset_y;
  int16_t bottom_offset_y;
  int16_t current_offset_y;

  GBitmap fade_top;
  GBitmap fade_bottom;
} ActionMenuItemAnimation;

typedef struct {
  Layer layer;
  MenuLayer menu_layer;
  int selected_index;
  unsigned separator_index;
  ActionMenuLayerCallbacks callbacks;

  const ActionMenuItem* items;
  int num_items;

  //! @internal
  ActionMenuLayoutCache layout_cache;
  //! @internal
  ActionMenuItemAnimation item_animation;

  const ActionMenuItem* short_items;
  int num_short_items;
  void *context;
} ActionMenuLayer;

ActionMenuLayer *action_menu_layer_create(GRect frame);

void action_menu_layer_set_callback(ActionMenuLayer *aml,
                                    ActionMenuLayerCallback cb,
                                    void *context);

void action_menu_layer_set_callbacks(ActionMenuLayer *aml,
                                     ActionMenuLayerCallbacks callbacks,
                                     void *context);

void action_menu_layer_notify_selection_changed(ActionMenuLayer *aml);

void action_menu_layer_set_align(ActionMenuLayer *aml,
                                 ActionMenuAlign align);

void action_menu_layer_set_items(ActionMenuLayer *aml,
                                 const ActionMenuItem *items,
                                 int num_items,
                                 unsigned default_selected_item,
                                 unsigned separator_index);

void action_menu_layer_click_config_provider(ActionMenuLayer *aml);

void action_menu_layer_destroy(ActionMenuLayer *aml);

void action_menu_layer_set_short_items(ActionMenuLayer *aml,
                                       const ActionMenuItem *items,
                                       int num_items,
                                       unsigned default_selected_item);

void action_menu_layer_init(ActionMenuLayer *aml, const GRect *frame);

void action_menu_layer_deinit(ActionMenuLayer *aml);
