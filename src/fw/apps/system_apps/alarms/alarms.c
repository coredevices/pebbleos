/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "alarms.h"

#include "alarm_detail.h"
#include "alarm_editor.h"

#include "applib/app.h"
#include "applib/graphics/gtypes.h"
#include "applib/preferred_content_size.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "services/common/clock.h"
#include "services/common/i18n/i18n.h"
#include "services/normal/alarms/alarm.h"
#include "services/normal/timeline/timeline.h"
#include "shell/system_theme.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/string.h"
#include "util/time/time.h"
#include "util/trig.h"

#include <stdio.h>
#include <string.h>

// Alarms app versions
// 0: Initial version or never opened
// 1: Smart alarms
#define CURRENT_ALARMS_APP_VERSION 1

typedef struct {
  ListNode node;
  AlarmId id;
  AlarmInfo info;
  bool scheduled_days[DAYS_PER_WEEK];
} AlarmNode;

typedef struct AlarmsAppData {
  Window window;
  MenuLayer menu_layer;
  StatusBarLayer status_layer;

  GBitmap plus_icon;
#if CAPABILITY_HAS_HEALTH_TRACKING
  GBitmap smart_alarm_icon;
#endif

  AlarmNode *alarm_list_head;
  MenuIndex selected_index;
  bool show_limit_reached_text;
  bool can_schedule_alarm;
  uint32_t current_plus_icon_resource_id;

  EventServiceInfo alarm_event_info;
} AlarmsAppData;


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Alarm list functions

static int prv_alarm_comparator(void *a, void *b) {
  AlarmNode *alarm_a = (AlarmNode*) a;
  AlarmNode *alarm_b = (AlarmNode*) b;

  // Sort by alarm time, with 12:00AM being the starting point
  if (alarm_a->info.hour > alarm_b->info.hour) {
    return true;
  }
  if (alarm_a->info.hour == alarm_b->info.hour && alarm_a->info.minute > alarm_b->info.minute) {
    return true;
  }
  return false;
}

static void prv_clear_alarm_list(AlarmsAppData* data) {
  while (data->alarm_list_head) {
    AlarmNode* old_head = data->alarm_list_head;
    data->alarm_list_head = (AlarmNode*) list_pop_head((ListNode *) old_head);
    task_free(old_head);
    old_head = NULL;
  }
}

static void prv_add_alarm_to_list(AlarmId id, const AlarmInfo *info, void *callback_context) {
  AlarmsAppData *data = (AlarmsAppData *)callback_context;

  AlarmNode *new_node = task_malloc_check(sizeof(AlarmNode));
  list_init((ListNode*) new_node);
  new_node->id = id;
  new_node->info = *info;
  memcpy(&new_node->scheduled_days, info->scheduled_days, sizeof(new_node->scheduled_days));
  new_node->info.scheduled_days = &new_node->scheduled_days;
  data->alarm_list_head = (AlarmNode *)list_sorted_add((ListNode *)data->alarm_list_head,
                                                       (ListNode *)new_node,
                                                       prv_alarm_comparator,
                                                       false);
}

static void prv_update_alarm_list(AlarmsAppData *data) {
  prv_clear_alarm_list(data);

  alarm_for_each(prv_add_alarm_to_list, data);
  data->can_schedule_alarm = alarm_can_schedule();
}

static bool prv_are_alarms_scheduled(AlarmsAppData *data) {
  return (list_count((ListNode*) data->alarm_list_head) > 0);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! General helper functions

static void prv_show_deleted_dialog(void) {
  SimpleDialog *simple_dialog = simple_dialog_create("AlarmDelete");
  Dialog *dialog = simple_dialog_get_dialog(simple_dialog);
  const char *delete_text = i18n_noop("Alarm Deleted");
  dialog_set_text(dialog, i18n_get(delete_text, dialog));
  i18n_free(delete_text, dialog);
  dialog_set_icon(dialog, RESOURCE_ID_RESULT_SHREDDED_LARGE);
  dialog_set_background_color(dialog, ALARMS_APP_HIGHLIGHT_COLOR);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_DEFAULT);
  app_simple_dialog_push(simple_dialog);
}

static int prv_get_list_idx_of_alarm_id(AlarmsAppData* data, AlarmId id) {
  if (id == ALARM_INVALID_ID) {
    return 0;
  }
  AlarmNode *cur_node = data->alarm_list_head;
  // Starting at one because of the "+" cell
  int list_idx = 1;
  while (cur_node) {
    if (cur_node->id == id) {
      return list_idx;
    }
    list_idx++;
    cur_node = (AlarmNode* )list_get_next((ListNode*) cur_node);
  }
  return 0;
}

static void prv_update_menu_layer(AlarmsAppData* data, AlarmId select_alarm) {
    MenuIndex selected_menu_index = {0, prv_get_list_idx_of_alarm_id(data, select_alarm)};
    menu_layer_reload_data(&data->menu_layer);
    menu_layer_set_selected_index(&data->menu_layer, selected_menu_index,
                                  MenuRowAlignCenter, false);
}

static void prv_handle_alarm_editor_complete(AlarmEditorResult result, AlarmId id,
                                             void *callback_context) {
  AlarmsAppData *data = (AlarmsAppData *)callback_context;
  if (result == CANCELLED && !prv_are_alarms_scheduled(data)) {
    // In the case the user had no alarms set, and didn't finish creating one.
    // We want to exit the app without showing an empty alarm list
    app_window_stack_remove(&data->window, true);
  } else if (result == DELETED) {
    prv_update_alarm_list(data);
    if (!prv_are_alarms_scheduled(data)) {
      // The user deleted their last alarm, show a dialog and setup the create new alarm screen.
      // We don't want to show an empty alarm list
      prv_show_deleted_dialog();
      Window *editor = alarm_editor_create_new_alarm(prv_handle_alarm_editor_complete, data);
      app_window_stack_insert_next(editor);
    } else {
      prv_update_menu_layer(data, ALARM_INVALID_ID);
    }
  } else {  // Created / Edited
    prv_update_alarm_list(data);
    prv_update_menu_layer(data, id);
  }
}

static void prv_handle_alarm_event(PebbleEvent *e, void *callback_context) {
  AlarmsAppData *data = (AlarmsAppData *)callback_context;
  prv_update_alarm_list(data);
}

static void prv_create_new_alarm(AlarmsAppData* data) {
  Window *editor = alarm_editor_create_new_alarm(prv_handle_alarm_editor_complete, data);
  app_window_stack_push(editor, true);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Menu Layer Callbacks

static bool prv_is_add_alarm_cell(MenuIndex *cell_index) {
  return cell_index->row == 0;
}

static uint16_t prv_alarm_list_get_num_sections_callback(struct MenuLayer *menu_layer,
                                                         void *callback_context) {
  return 1;
}

static uint16_t prv_alarm_list_get_num_rows_callback(struct MenuLayer *menu_layer,
                                                     uint16_t section_index,
                                                     void *callback_context) {
  AlarmsAppData *data = (AlarmsAppData *)callback_context;
  // Number of alarms + the add alarm header
  return list_count((ListNode*) data->alarm_list_head) + 1;
}

static int16_t prv_alarm_list_get_cell_height_callback(struct MenuLayer *menu_layer,
                                                       MenuIndex *cell_index,
                                                       void *callback_context) {
  return menu_cell_basic_cell_height();
}

static void prv_alarm_list_draw_row_callback(GContext *ctx, const Layer *cell_layer,
                                             MenuIndex *cell_index, void *callback_context) {
  AlarmsAppData *data = (AlarmsAppData *)callback_context;

  if (prv_is_add_alarm_cell(cell_index)) {

    GRect box;
    uint32_t new_bitmap_resource = RESOURCE_ID_PLUS_ICON_BLACK;

    if (!data->can_schedule_alarm) { // alarm limit reached
      if (menu_cell_layer_is_highlighted(cell_layer)) {
        if (data->show_limit_reached_text) {
          // Trying to add a new alarm when list is already full
          const GFont font =
              system_theme_get_font_for_default_size(TextStyleFont_MenuCellSubtitle);

          box = GRect(0, 0, cell_layer->bounds.size.w, fonts_get_font_height(font));

          const char *text = i18n_get("Limit reached.", data);
          box.size = graphics_text_layout_get_max_used_size(ctx, text, font, box,
                                                            GTextOverflowModeTrailingEllipsis,
                                                            GTextAlignmentCenter, NULL);
          grect_align(&box, &cell_layer->bounds, GAlignCenter, true /* clip */);
          box.origin.y -= fonts_get_font_cap_offset(font);

          graphics_draw_text(ctx, text, font, box,
                             GTextOverflowModeFill, GTextAlignmentCenter, NULL);
          return;
        } else { // "add alarm" cell highlighted
          new_bitmap_resource = RESOURCE_ID_PLUS_ICON_DOTTED;
        }
      } else { // "add alarm" cell not highlighted
        // Have to manually override the tint color as we're using a color that differs
        // from the ones the MenuLayer uses.
        graphics_context_set_tint_color(ctx, GColorLightGray);
      }
    }

    if (new_bitmap_resource != data->current_plus_icon_resource_id) {
      // Change the icon to the dotted one
      data->current_plus_icon_resource_id = new_bitmap_resource;
      gbitmap_deinit(&data->plus_icon);
      gbitmap_init_with_resource(&data->plus_icon, data->current_plus_icon_resource_id);
    }


    box.origin = GPoint((cell_layer->bounds.size.w - data->plus_icon.bounds.size.w) / 2,
                        (cell_layer->bounds.size.h - data->plus_icon.bounds.size.h) / 2);
    box.size = data->plus_icon.bounds.size;
    graphics_context_set_compositing_mode(ctx, GCompOpTint);
    graphics_draw_bitmap_in_rect(ctx, &data->plus_icon, &box);

    return;
  }

  AlarmNode *node = (AlarmNode*) list_get_at((ListNode*)data->alarm_list_head, cell_index->row - 1);

  // Format 1: 10:34 AM
  // Format 2: 14:56
  char alarm_time_text[9];
  clock_format_time(alarm_time_text, sizeof(alarm_time_text),
                    node->info.hour, node->info.minute, true);
  const char *enabled = node->info.enabled ? i18n_get("ON", data) : i18n_get("OFF", data);

  graphics_context_set_compositing_mode(ctx, GCompOpTint);
  // If the alarm is not smart, use the icon as spacing but don't render it.
  // Otherwise if the alarm is smart draw according to the menu highlight.
  graphics_context_set_tint_color(ctx, !node->info.is_smart ? GColorClear :
                                       (cell_layer->is_highlighted ? GColorWhite : GColorBlack));

  char alarm_day_text[32] = {0};
  MenuCellLayerConfig config = {
    .title = alarm_time_text,
    .value = enabled,
#if CAPABILITY_HAS_HEALTH_TRACKING
    .icon = &data->smart_alarm_icon,
    .icon_align = MenuCellLayerIconAlign_TopLeft,
    .icon_box_model = &(GBoxModel) { .offset = { 0, 5 }, .margin = { 6, 0 } },
    .icon_form_fit = true,
    .horizontal_inset = PBL_IF_ROUND_ELSE(-6, 0),
#endif
    .overflow_mode = GTextOverflowModeTrailingEllipsis,
  };
  if (node->info.kind != ALARM_KIND_CUSTOM) {
    const bool all_caps = false;
    config.subtitle = i18n_get(alarm_get_string_for_kind(node->info.kind, all_caps), data);
  } else {
    alarm_get_string_for_custom(node->scheduled_days, alarm_day_text);
    config.subtitle = alarm_day_text;
  }
  menu_cell_layer_draw(ctx, cell_layer, &config);
}

static void prv_alarm_list_select_callback(MenuLayer *menu_layer, MenuIndex *cell_index,
                                           void *callback_context) {
  AlarmsAppData *data = (AlarmsAppData *)callback_context;
  if (prv_is_add_alarm_cell(cell_index)) {
    if (!data->can_schedule_alarm) {
      data->show_limit_reached_text = true;
      layer_mark_dirty(menu_layer_get_layer(&data->menu_layer));
    } else {
      prv_create_new_alarm(data);
    }
    return;
  }

  // Minus 1 because of the "add alarm" cell
  AlarmNode *node =
      (AlarmNode *)list_get_at((ListNode *)data->alarm_list_head, cell_index->row - 1);
  alarm_detail_window_push(node->id, &node->info, prv_handle_alarm_editor_complete, data);
}

static void prv_alarm_list_selection_changed_callback(MenuLayer *menu_layer, MenuIndex new_index,
                                                      MenuIndex old_index, void *callback_context) {
  AlarmsAppData *data = (AlarmsAppData *)callback_context;
  if (prv_is_add_alarm_cell(&old_index)) {
    data->show_limit_reached_text = false;
  }
}

#if CAPABILITY_HAS_HEALTH_TRACKING
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Smart Alarm first use dialog

typedef struct FirstUseDialog {
  ExpandableDialog dialog;
  AlarmsAppData *data;
} FirstUseDialog;

static void prv_alarms_app_opened_click_handler(ClickRecognizerRef recognizer, void *context) {
  ExpandableDialog *expandable_dialog = context;
  expandable_dialog_pop(expandable_dialog);
}

static void prv_push_alarms_app_opened_dialog(AlarmsAppData *data) {
  const char *first_use_text = i18n_get(
      "Let us wake you in your lightest sleep so you're fully refreshed! "
      "Smart Alarm wakes you up to 30min before your alarm.", data);
  const char *header = i18n_get("Smart Alarm", data);
  ExpandableDialog *expandable_dialog = expandable_dialog_create_with_params(
      header, RESOURCE_ID_SMART_ALARM_TINY, first_use_text,
      GColorBlack, GColorWhite, NULL, RESOURCE_ID_ACTION_BAR_ICON_CHECK,
      prv_alarms_app_opened_click_handler);

  expandable_dialog_set_action_bar_background_color(expandable_dialog, ALARMS_APP_HIGHLIGHT_COLOR);
  expandable_dialog_set_header(expandable_dialog, header);
#if defined(PBL_ROUND)
  expandable_dialog_set_header_font(expandable_dialog,
                                    fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
#endif

  // Show immediately since this is the first window and there is already a compositor animation
  app_window_stack_push(&expandable_dialog->dialog.window, false /* animated */);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
//! App boilerplate

static void prv_handle_init(void) {
  AlarmsAppData *data = app_malloc_check(sizeof(*data));
  *data = (AlarmsAppData) {{ .user_data = NULL }};

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Alarms"));
  window_set_user_data(window, data);

  data->alarm_list_head = NULL;
  // Alarm list must be updated before menu layer is initialized
  prv_update_alarm_list(data);

  const GRect bounds = grect_inset(data->window.layer.bounds,
                                   GEdgeInsets(STATUS_BAR_LAYER_HEIGHT, 0,
                                               PBL_IF_ROUND_ELSE(STATUS_BAR_LAYER_HEIGHT, 0), 0));
  menu_layer_init(&data->menu_layer, &bounds);
  menu_layer_set_callbacks(&data->menu_layer, data, &(MenuLayerCallbacks) {
    .get_num_sections = prv_alarm_list_get_num_sections_callback,
    .get_num_rows = prv_alarm_list_get_num_rows_callback,
    .get_cell_height = prv_alarm_list_get_cell_height_callback,
    .draw_row = prv_alarm_list_draw_row_callback,
    .select_click = prv_alarm_list_select_callback,
    .selection_changed = prv_alarm_list_selection_changed_callback
  });

  menu_layer_set_highlight_colors(&data->menu_layer, ALARMS_APP_HIGHLIGHT_COLOR, GColorWhite);
  menu_layer_set_click_config_onto_window(&data->menu_layer, &data->window);
  layer_add_child(&data->window.layer, menu_layer_get_layer(&data->menu_layer));

  status_bar_layer_init(&data->status_layer);
  status_bar_layer_set_colors(&data->status_layer, PBL_IF_COLOR_ELSE(GColorWhite, GColorBlack),
                              PBL_IF_COLOR_ELSE(GColorBlack, GColorWhite));
  status_bar_layer_set_separator_mode(&data->status_layer, StatusBarLayerSeparatorModeNone);
  layer_add_child(&data->window.layer, status_bar_layer_get_layer(&data->status_layer));

#if CAPABILITY_HAS_HEALTH_TRACKING
  gbitmap_init_with_resource(&data->smart_alarm_icon, RESOURCE_ID_SMART_ALARM_ICON_BLACK);
#endif
  gbitmap_init_with_resource(&data->plus_icon, RESOURCE_ID_PLUS_ICON_BLACK);
  data->current_plus_icon_resource_id = RESOURCE_ID_PLUS_ICON_BLACK;

  app_state_set_user_data(data);

  data->alarm_event_info = (EventServiceInfo) {
    .type = PEBBLE_ALARM_CLOCK_EVENT,
    .handler = prv_handle_alarm_event,
    .context = data,
  };
  event_service_client_subscribe(&data->alarm_event_info);

  if (prv_are_alarms_scheduled(data)) {
    int list_idx = 1; // Default to first alarm entry in list
    if (app_launch_reason() == APP_LAUNCH_TIMELINE_ACTION) {
      AlarmId alarm_id = (AlarmId)app_launch_get_args();
      list_idx = prv_get_list_idx_of_alarm_id(data, alarm_id);
      if (list_idx == 0) {
        list_idx = 1; // Default to first alarm if idx not found
      }
    }

    app_window_stack_push(&data->window, true);
    menu_layer_set_selected_index(&data->menu_layer, MenuIndex(0, list_idx),
                                  MenuRowAlignCenter, false);
  } else {
    Window *editor = alarm_editor_create_new_alarm(prv_handle_alarm_editor_complete, data);
    app_window_stack_push(editor, true);
    app_window_stack_insert_next(&data->window);
  }

#if CAPABILITY_HAS_HEALTH_TRACKING
  uint32_t version = alarm_prefs_get_alarms_app_opened();
  if (version == 0) {
    prv_push_alarms_app_opened_dialog(data);
  }
  alarm_prefs_set_alarms_app_opened(CURRENT_ALARMS_APP_VERSION);
#endif
}

static void prv_handle_deinit(void) {
  AlarmsAppData *data = app_state_get_user_data();
  status_bar_layer_deinit(&data->status_layer);
  menu_layer_deinit(&data->menu_layer);
  i18n_free_all(data);
  prv_clear_alarm_list(data);
  event_service_client_unsubscribe(&data->alarm_event_info);
  app_free(data);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();

  prv_handle_deinit();
}

const PebbleProcessMd* alarms_app_get_info() {
  static const PebbleProcessMdSystem s_alarms_app_info = {
    .common = {
      .main_func = s_main,
      .uuid = UUID_ALARMS_DATA_SOURCE,
    },
    .name = i18n_noop("Alarms"),
#if CAPABILITY_HAS_APP_GLANCES
    .icon_resource_id = RESOURCE_ID_ALARM_CLOCK_TINY,
#elif PLATFORM_TINTIN
    .icon_resource_id = RESOURCE_ID_MENU_LAYER_ALARMS_APP_ICON,
#endif
  };
  return (const PebbleProcessMd*) &s_alarms_app_info;
}
