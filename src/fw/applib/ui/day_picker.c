/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/ui/day_picker.h"

#include "applib/pbl_std/timelocal.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/menu_layer.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/i18n/i18n.h"
#include "resource/resource_ids.auto.h"
#include "shell/prefs.h"
#include "system/passert.h"

#include <string.h>

#define DAY_PICKER_CELL_HEIGHT PBL_IF_RECT_ELSE(menu_cell_small_cell_height(), \
                                                   menu_cell_basic_cell_height())

typedef struct {
  Window window;
  MenuLayer menu_layer;
  DayPickerResult initial;
  DayPickerCallback callback;
  void *callback_context;
  GColor highlight_color;
  bool allow_once;
} DayPickerData;

typedef struct {
  Window window;
  MenuLayer menu_layer;
  bool scheduled_days[DAYS_PER_WEEK];
  GBitmap deselected_icon;
  GBitmap selected_icon;
  GBitmap checkmark_icon;
  uint32_t current_checkmark_icon_resource_id;
  bool show_check_something_first_text;
  bool was_completed;
  DayPickerCallback callback;
  void *callback_context;
  GColor highlight_color;
} CustomDayPickerData;

static const char *prv_kind_strings[DayPickerKindNumItems] = {
  [DayPickerKindEveryday] = "Every Day",
  [DayPickerKindWeekdays] = "Weekdays",
  [DayPickerKindWeekends] = "Weekends",
  [DayPickerKindCustom] = "Custom",
  [DayPickerKindJustOnce] = "Just Once",
};

const char *day_picker_kind_get_string(DayPickerKind kind) {
  if (kind >= DayPickerKindNumItems) {
    return "";
  }
  return i18n_noop(prv_kind_strings[kind]);
}

static DayPickerKind prv_row_to_kind(bool allow_once, int row) {
  if (allow_once) {
    if (row == 0) {
      return DayPickerKindJustOnce;
    }
    int kind_row = row - 1;
    if (kind_row >= 0 && kind_row < (DayPickerKindNumItems - 1)) {
      return (DayPickerKind)kind_row;
    }
    return DayPickerKindEveryday;
  }
  if (row >= 0 && row < (DayPickerKindNumItems - 1)) {
    return (DayPickerKind)row;
  }
  return DayPickerKindEveryday;
}

static int prv_kind_to_row(bool allow_once, DayPickerKind kind) {
  if (allow_once) {
    if (kind == DayPickerKindJustOnce) {
      return 0;
    }
    return (int)kind + 1;
  }
  if (kind == DayPickerKindJustOnce) {
    return 0;
  }
  return (int)kind;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Day Picker (kind selection)

static uint16_t prv_day_picker_get_num_sections(struct MenuLayer *menu_layer,
                                                 void *callback_context) {
  return 1;
}

static uint16_t prv_day_picker_get_num_rows(struct MenuLayer *menu_layer,
                                             uint16_t section_index,
                                             void *callback_context) {
  DayPickerData *data = (DayPickerData *)callback_context;
  return (DayPickerKindNumItems - 1) + (data->allow_once ? 1 : 0);
}

static int16_t prv_day_picker_get_cell_height(struct MenuLayer *menu_layer,
                                               MenuIndex *cell_index,
                                               void *callback_context) {
  return DAY_PICKER_CELL_HEIGHT;
}

static void prv_day_picker_draw_row(GContext *ctx, const Layer *cell_layer,
                                    MenuIndex *cell_index, void *callback_context) {
  DayPickerData *data = (DayPickerData *)callback_context;
  DayPickerKind kind = prv_row_to_kind(data->allow_once, cell_index->row);
  const char *cell_text = i18n_get(day_picker_kind_get_string(kind), &data->window);
  menu_cell_basic_draw(ctx, cell_layer, cell_text, NULL, NULL);
}

static void prv_day_picker_handle_selection(MenuLayer *menu_layer, MenuIndex *cell_index,
                                            void *callback_context) {
  DayPickerData *data = (DayPickerData *)callback_context;
  DayPickerKind kind = prv_row_to_kind(data->allow_once, cell_index->row);
  DayPickerResult result = {
    .kind = kind,
  };
  memset(result.custom_days, 0, sizeof(result.custom_days));

  if (kind == DayPickerKindCustom) {
    bool initial_days[DAYS_PER_WEEK] = {false};
    if (data->initial.kind == DayPickerKindCustom) {
      memcpy(initial_days, data->initial.custom_days, sizeof(initial_days));
    }
    custom_day_picker_push(initial_days, data->callback, data->callback_context,
                          data->highlight_color);
    app_window_stack_remove(&data->window, true);
    return;
  }

  if (data->callback) {
    data->callback(result, data->callback_context);
  }
  app_window_stack_remove(&data->window, true);
}

static void prv_day_picker_window_unload(Window *window) {
  DayPickerData *data = (DayPickerData *)window_get_user_data(window);
  menu_layer_deinit(&data->menu_layer);
  i18n_free_all(&data->window);
  task_free(data);
}

void day_picker_push(DayPickerConfig config, DayPickerCallback callback,
                     void *context) {
  DayPickerData *data = task_malloc_check(sizeof(DayPickerData));
  *data = (DayPickerData){
    .initial = config.initial,
    .callback = callback,
    .callback_context = context,
    .highlight_color = config.highlight_color,
    .allow_once = config.allow_once,
  };

  window_init(&data->window, WINDOW_NAME("Day Picker"));
  window_set_user_data(&data->window, data);
  data->window.window_handlers.unload = prv_day_picker_window_unload;

  GRect bounds = data->window.layer.bounds;
#if PBL_ROUND
  bounds = grect_inset_internal(bounds, 0, STATUS_BAR_LAYER_HEIGHT);
#endif
  menu_layer_init(&data->menu_layer, &bounds);
  menu_layer_set_callbacks(&data->menu_layer, data, &(MenuLayerCallbacks){
    .get_num_sections = prv_day_picker_get_num_sections,
    .get_num_rows = prv_day_picker_get_num_rows,
    .get_cell_height = prv_day_picker_get_cell_height,
    .draw_row = prv_day_picker_draw_row,
    .select_click = prv_day_picker_handle_selection,
  });
  menu_layer_set_highlight_colors(&data->menu_layer, config.highlight_color, GColorWhite);
  menu_layer_set_click_config_onto_window(&data->menu_layer, &data->window);
  menu_layer_set_scroll_wrap_around(&data->menu_layer,
                                    shell_prefs_get_menu_scroll_wrap_around_enable());
  menu_layer_set_scroll_vibe_on_wrap(&data->menu_layer,
                                     shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
  menu_layer_set_scroll_vibe_on_blocked(&data->menu_layer,
                                        shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);
  layer_add_child(&data->window.layer, menu_layer_get_layer(&data->menu_layer));

  uint16_t selected_row = (uint16_t)prv_kind_to_row(config.allow_once, config.initial.kind);
  menu_layer_set_selected_index(&data->menu_layer,
                                (MenuIndex){.row = selected_row},
                                MenuRowAlignCenter, false);

  app_window_stack_push(&data->window, true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Custom Day Picker (day-of-week toggle)

static bool prv_is_custom_day_scheduled(CustomDayPickerData *data) {
  for (unsigned int i = 0; i < sizeof(data->scheduled_days); i++) {
    if (data->scheduled_days[i]) {
      return true;
    }
  }
  return false;
}

static uint16_t prv_custom_day_picker_get_num_sections(struct MenuLayer *menu_layer,
                                                        void *callback_context) {
  return 1;
}

static uint16_t prv_custom_day_picker_get_num_rows(struct MenuLayer *menu_layer,
                                                    uint16_t section_index, void *callback_context) {
  return DAYS_PER_WEEK + 1;
}

static int16_t prv_custom_day_picker_get_cell_height(struct MenuLayer *menu_layer,
                                                      MenuIndex *cell_index,
                                                      void *callback_context) {
  return DAY_PICKER_CELL_HEIGHT;
}

static void prv_custom_day_picker_draw_row(GContext *ctx, const Layer *cell_layer,
                                            MenuIndex *cell_index, void *callback_context) {
  CustomDayPickerData *data = (CustomDayPickerData *)callback_context;
  GBitmap *ptr_bitmap;

  if (cell_index->row == 0) {
    GRect box;
    uint32_t new_resource_id = RESOURCE_ID_CHECKMARK_ICON_BLACK;

    if (!prv_is_custom_day_scheduled(data)) {
      if (menu_cell_layer_is_highlighted(cell_layer)) {
        if (data->show_check_something_first_text) {
          box.size = GSize(cell_layer->bounds.size.w, DAY_PICKER_CELL_HEIGHT);
          box.origin = GPoint(0, 4);
          graphics_draw_text(ctx, i18n_get("Check something first.", &data->window),
                             fonts_get_system_font(FONT_KEY_GOTHIC_18), box,
                             GTextOverflowModeFill, GTextAlignmentCenter, NULL);
          return;
        } else {
          new_resource_id = RESOURCE_ID_CHECKMARK_ICON_DOTTED;
        }
      }
    }

    if (new_resource_id != data->current_checkmark_icon_resource_id) {
      data->current_checkmark_icon_resource_id = new_resource_id;
      gbitmap_deinit(&data->checkmark_icon);
      gbitmap_init_with_resource(&data->checkmark_icon, data->current_checkmark_icon_resource_id);
    }

    box.origin = GPoint((((cell_layer->bounds.size.w) / 2) - ((data->checkmark_icon.bounds.size.w) / 2)),
                        (((cell_layer->bounds.size.h) / 2) - ((data->checkmark_icon.bounds.size.h) / 2)));
    box.size = data->checkmark_icon.bounds.size;
    graphics_context_set_compositing_mode(ctx, GCompOpTint);
    graphics_draw_bitmap_in_rect(ctx, &data->checkmark_icon, &box);
  } else {
    const char *cell_text;
    uint16_t day_index = cell_index->row % DAYS_PER_WEEK;
    const struct lc_time_T *time_locale = time_locale_get();
    cell_text = i18n_get(time_locale->weekday[day_index], &data->window);

    if (data->scheduled_days[(cell_index->row) % DAYS_PER_WEEK]) {
      ptr_bitmap = &data->selected_icon;
    } else {
      ptr_bitmap = &data->deselected_icon;
    }
    graphics_context_set_compositing_mode(ctx, GCompOpTint);
    menu_cell_basic_draw_icon_right(ctx, cell_layer, cell_text, NULL, ptr_bitmap);
  }
}

static void prv_custom_day_picker_handle_selection(MenuLayer *menu_layer, MenuIndex *cell_index,
                                                    void *callback_context) {
  CustomDayPickerData *data = (CustomDayPickerData *)callback_context;

  if (cell_index->row == 0) {
    if (!prv_is_custom_day_scheduled(data)) {
      data->show_check_something_first_text = true;
      layer_mark_dirty(menu_layer_get_layer(menu_layer));
    } else {
      data->was_completed = true;
      DayPickerResult result = {
        .kind = DayPickerKindCustom,
      };
      memcpy(result.custom_days, data->scheduled_days, sizeof(result.custom_days));
      if (data->callback) {
        data->callback(result, data->callback_context);
      }
      app_window_stack_pop(true);
    }
  } else {
    uint16_t day_of_week = (cell_index->row) % DAYS_PER_WEEK;
    data->scheduled_days[day_of_week] = !data->scheduled_days[day_of_week];
    layer_mark_dirty(menu_layer_get_layer(menu_layer));
  }
}

static void prv_custom_day_picker_selection_changed(MenuLayer *menu_layer, MenuIndex new_index,
                                                     MenuIndex old_index, void *callback_context) {
  CustomDayPickerData *data = (CustomDayPickerData *)callback_context;
  if (old_index.row == 0) {
    data->show_check_something_first_text = false;
  }
}

static void prv_custom_day_picker_window_unload(Window *window) {
  CustomDayPickerData *data = (CustomDayPickerData *)window_get_user_data(window);
  menu_layer_deinit(&data->menu_layer);
  gbitmap_deinit(&data->selected_icon);
  gbitmap_deinit(&data->deselected_icon);
  gbitmap_deinit(&data->checkmark_icon);
  i18n_free_all(&data->window);
  task_free(data);
}

void custom_day_picker_push(bool initial_days[DAYS_PER_WEEK],
                            DayPickerCallback callback, void *context,
                            GColor highlight_color) {
  CustomDayPickerData *data = task_malloc_check(sizeof(CustomDayPickerData));
  *data = (CustomDayPickerData){
    .callback = callback,
    .callback_context = context,
    .highlight_color = highlight_color,
    .was_completed = false,
    .show_check_something_first_text = false,
  };
  memcpy(data->scheduled_days, initial_days, sizeof(data->scheduled_days));

  window_init(&data->window, WINDOW_NAME("Custom Day Picker"));
  window_set_user_data(&data->window, data);
  data->window.window_handlers.unload = prv_custom_day_picker_window_unload;

  GRect bounds = data->window.layer.bounds;
#if PBL_ROUND
  bounds = grect_inset_internal(bounds, 0, STATUS_BAR_LAYER_HEIGHT);
#endif
  menu_layer_init(&data->menu_layer, &bounds);
  menu_layer_set_callbacks(&data->menu_layer, data, &(MenuLayerCallbacks){
    .get_num_sections = prv_custom_day_picker_get_num_sections,
    .get_num_rows = prv_custom_day_picker_get_num_rows,
    .get_cell_height = prv_custom_day_picker_get_cell_height,
    .draw_row = prv_custom_day_picker_draw_row,
    .select_click = prv_custom_day_picker_handle_selection,
    .selection_changed = prv_custom_day_picker_selection_changed,
  });
  menu_layer_set_highlight_colors(&data->menu_layer, highlight_color, GColorWhite);
  menu_layer_set_click_config_onto_window(&data->menu_layer, &data->window);
  menu_layer_set_scroll_wrap_around(&data->menu_layer,
                                     shell_prefs_get_menu_scroll_wrap_around_enable());
  menu_layer_set_scroll_vibe_on_wrap(&data->menu_layer,
                                      shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
  menu_layer_set_scroll_vibe_on_blocked(&data->menu_layer,
                                        shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);
  layer_add_child(&data->window.layer, menu_layer_get_layer(&data->menu_layer));

  gbitmap_init_with_resource(&data->selected_icon, RESOURCE_ID_CHECKBOX_ICON_CHECKED);
  gbitmap_init_with_resource(&data->deselected_icon, RESOURCE_ID_CHECKBOX_ICON_UNCHECKED);
  gbitmap_init_with_resource(&data->checkmark_icon, RESOURCE_ID_CHECKMARK_ICON_BLACK);
  data->current_checkmark_icon_resource_id = RESOURCE_ID_CHECKMARK_ICON_BLACK;

  app_window_stack_push(&data->window, true);
}