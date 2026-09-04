/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "action_menu_hierarchy.h"
#include "action_menu_window_private.h"

#include "applib/applib_malloc.auto.h"

// Item
/////////////////////////////////
char *action_menu_item_get_label(const ActionMenuItem *item) {
  if (item == NULL) {
    return NULL;
  }
  return (char *)item->label;
}

void *action_menu_item_get_action_data(const ActionMenuItem *item) {
  if (item == NULL || !item->is_leaf) {
    return NULL;
  }
  return (void *)item->action_data;
}

// Level
/////////////////////////////////

ActionMenuLevel *action_menu_level_create(uint16_t max_items) {
  // TODO add applib-malloc padding
  ActionMenuLevel *level = applib_malloc(applib_type_size(ActionMenuLevel) +
                                         max_items * applib_type_size(ActionMenuItem));
  if (!level) return NULL;
  *level = (ActionMenuLevel){
    .max_items = max_items,
    .display_mode = ActionMenuLevelDisplayModeWide,
  };

  return level;
}

void action_menu_level_set_display_mode(ActionMenuLevel *level,
                                        ActionMenuLevelDisplayMode display_mode) {
  if (!level) return;
  level->display_mode = display_mode;
}

ActionMenuItem *action_menu_level_add_action(ActionMenuLevel *level,
                             const char *label,
                             ActionMenuPerformActionCb cb,
                             void *action_data) {
  if (!level || !label || !cb ||
      (level->num_items >= level->max_items)) {
    return NULL;
  }

  ActionMenuItem *item = &level->items[level->num_items];
  *item = (ActionMenuItem) {
    .label = label,
    .perform_action = cb,
    .action_data = action_data,
  };
  ++level->num_items;
  return item;
}

ActionMenuItem *action_menu_level_add_child(ActionMenuLevel *level,
                                 ActionMenuLevel *child,
                                 const char *label) {
  if (!level || !child || !label ||
      (level->num_items >= level->max_items)) {
    return NULL;
  }

  child->parent_level = level;

  ActionMenuItem *item = &level->items[level->num_items];
  *item = (ActionMenuItem) {
    .label = label,
    .next_level = child,
  };
  ++level->num_items;

  return item;
}

// Hierarchy
/////////////////////////////////

static void prv_cleanup_helper(const ActionMenuLevel *level,
                               ActionMenuEachItemCb each_cb,
                               void *context) {
  for (int i = 0; i < level->num_items; ++i) {
    const ActionMenuItem *item = &level->items[i];
    if (!item->is_leaf && item->next_level) {
      prv_cleanup_helper(item->next_level, each_cb, context);
    }
    if (each_cb) {
      each_cb(item, context);
    }
  }

  applib_free((void *)level);
}

void action_menu_hierarchy_destroy(const ActionMenuLevel *root,
                                   ActionMenuEachItemCb each_cb,
                                   void *context) {
  if (root) {
    prv_cleanup_helper(root, each_cb, context);
  }
}
