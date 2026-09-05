/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "notifications.h"
#include "notifications_history.h"

#include <time.h>

#include "applib/app.h"
#include "applib/app_exit_reason.h"
#include "applib/preferred_content_size.h"
#include "applib/fonts/fonts.h"
#include "applib/graphics/gdraw_command_image.h"
#include "applib/graphics/gdraw_command_list.h"
#include "applib/ui/dialogs/actionable_dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/menu_cell_layer.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/system_icons.h"
#include "popups/notifications/notification_window.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/drivers/rtc.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/clock.h"
#include "pbl/services/notifications/alerts_preferences_private.h"
#include "pbl/services/notifications/notification_storage.h"
#include "pbl/services/timeline/notification_layout.h"
#include "shell/prefs.h"
#include "shell/system_theme.h"
#include "system/passert.h"
#include "pbl/util/list.h"
#include "pbl/util/string.h"
#include "util/time/time.h"

typedef struct LoadedNotificationNode {
  ListNode node;
  TimelineItem notification;
  GDrawCommandImage *icon;
  bool icon_is_default;
} LoadedNotificationNode;

typedef struct NotificationGroupWindow NotificationGroupWindow;

typedef struct NotificationsData {
  Window window;
  MenuLayer menu_layer;
  TextLayer text_layer;
  NotificationHistory history;
  LoadedNotificationNode *loaded_notification_list;
  EventServiceInfo notification_event_info;
  ActionableDialog *actionable_dialog;
  char *group_title;
  size_t group_title_size;
  NotificationGroupWindow *group_window;
#if PBL_ROUND
  StatusBarLayer status_bar_layer;
#endif
} NotificationsData;

struct NotificationGroupWindow {
  Window window;
  MenuLayer menu_layer;
  NotificationsData *notifications_data;
  Uuid *notification_ids;
  uint16_t count;
  char *sender;
  char time_buffer[16];
};

static NotificationsData *s_data = NULL;

static const unsigned int MAX_ACTIVE_NOTIFICATIONS = 6;

static bool prv_loaded_notification_list_filter_cb(ListNode *node, void *data) {
  LoadedNotificationNode *loaded_notification = (LoadedNotificationNode *)node;
  Uuid *id = data;
  return uuid_equal(&loaded_notification->notification.header.id, id);
}

static LoadedNotificationNode *prv_find_loaded_notification(LoadedNotificationNode *list,
                                                            Uuid *id) {
  return (LoadedNotificationNode *)list_find((ListNode *)list,
                                             prv_loaded_notification_list_filter_cb,
                                             id);
}

static bool prv_notif_iterator_callback(void *data, SerializedTimelineItemHeader *header) {
  NotificationsData *notifications_data = data;
  notifications_history_add_header(&notifications_data->history, &header->common);
  return true;
}

static bool prv_notif_item_iterator_callback(void *data, const CommonTimelineItemHeader *header,
                                             const TimelineItem *item) {
  NotificationsData *notifications_data = data;
  if (item) {
    notifications_history_add_item(&notifications_data->history, item);
  } else {
    notifications_history_add_header(&notifications_data->history, header);
  }
  return true;
}

static void prv_load_notification_storage(NotificationsData *data) {
  if (data->history.group_by_sender) {
    notification_storage_iterate_items_after(data->history.grouping_cutoff,
                                             prv_notif_item_iterator_callback, data);
  } else {
    notification_storage_iterate(&prv_notif_iterator_callback, data);
  }
}

static void prv_unload_loaded_notification(LoadedNotificationNode *loaded_notif) {
  timeline_item_free_allocated_buffer(&loaded_notif->notification);
  gdraw_command_image_destroy(loaded_notif->icon);
  app_free(loaded_notif);
}

static NOINLINE LoadedNotificationNode *prv_loaded_notification_list_load_item(
    LoadedNotificationNode **loaded_list, const Uuid *id) {
  if (id == NULL) {
    return NULL;
  }

  LoadedNotificationNode *loaded_node = prv_find_loaded_notification(*loaded_list, (Uuid *)id);
  if (loaded_node) {
    return loaded_node;
  }

  // unload old notifications
  if (list_count((ListNode *)*loaded_list) >= MAX_ACTIVE_NOTIFICATIONS) {
    LoadedNotificationNode *old_node = (LoadedNotificationNode*) list_get_tail(
        (ListNode*) *loaded_list);
    list_remove((ListNode*) old_node, (ListNode**) loaded_list, NULL);
    prv_unload_loaded_notification(old_node);
  }

  // load the notification
  TimelineItem notification;
  if (!notification_storage_get((Uuid *)id, &notification)) {
    return NULL;
  }

  // track the loaded notification
  loaded_node = app_malloc_check(sizeof(LoadedNotificationNode));

  list_init((ListNode*) loaded_node);
  loaded_node->notification = notification;

  TimelineResourceId timeline_res_id = attribute_get_uint32(&notification.attr_list,
                                                            AttributeIdIconTiny,
                                                            NOTIF_FALLBACK_ICON);

  // Read the associated pin's app id
  TimelineItem pin;
  if (timeline_resources_is_system(timeline_res_id) ||
      pin_db_read_item_header(&pin, &notification.header.parent_id) != S_SUCCESS) {
    pin.header.parent_id = (Uuid)UUID_INVALID;
  }

  TimelineResourceInfo timeline_res = {
    .res_id = timeline_res_id,
    .app_id = &pin.header.parent_id,
    .fallback_id = NOTIF_FALLBACK_ICON
  };
  AppResourceInfo icon_res_info;
  timeline_resources_get_id(&timeline_res, TimelineResourceSizeTiny, &icon_res_info);
  loaded_node->icon = gdraw_command_image_create_with_resource_system(icon_res_info.res_app_num,
                                                                      icon_res_info.res_id);
  loaded_node->icon_is_default = (timeline_res_id == NOTIF_FALLBACK_ICON) ||
                                 (timeline_res_id == TIMELINE_RESOURCE_NOTIFICATION_GENERIC);

  *loaded_list = (LoadedNotificationNode*) list_prepend((ListNode*) *loaded_list,
      (ListNode*)loaded_node);

  return loaded_node;
}

static void prv_loaded_notification_list_deinit(LoadedNotificationNode *loaded_list) {
  while (loaded_list) {
    LoadedNotificationNode *node = loaded_list;
    loaded_list = (LoadedNotificationNode*) list_pop_head((ListNode*) loaded_list);
    prv_unload_loaded_notification(node);
  }
}

static void prv_notifications_history_init(NotificationsData *data) {
  const NotificationGroupingRange range = alerts_preferences_get_notification_grouping_range();
  time_t cutoff = 0;
  time_t window = 0;

  if (range == NotificationGroupingRange_OneDay) {
    window = SECONDS_PER_DAY;
  } else if (range == NotificationGroupingRange_OneWeek) {
    window = 7 * SECONDS_PER_DAY;
  }

  if (window > 0) {
    const time_t now = rtc_get_time();
    cutoff = (now > window) ? now - window : 0;
  }

  notifications_history_init(&data->history, range != NotificationGroupingRange_Never, cutoff);
}

static bool prv_push_single_notification_window(const Uuid *id) {
  notification_window_init_history(false);
  if (notification_window_is_modal()) {
    return false;
  }

  notification_window_add_notification_by_id((Uuid *)id);
  notification_window_show();
  notification_window_focus_notification((Uuid *)id, false);
  return true;
}

static uint16_t prv_group_window_get_num_rows(MenuLayer *menu_layer, uint16_t section_index,
                                              void *context) {
  NotificationGroupWindow *group_window = context;
  return group_window->count;
}

static int16_t prv_group_window_get_header_height(MenuLayer *menu_layer, uint16_t section_index,
                                                  void *context) {
  return MENU_CELL_BASIC_HEADER_HEIGHT;
}

static int16_t prv_group_window_get_cell_height(MenuLayer *menu_layer, MenuIndex *cell_index,
                                                void *context) {
  return menu_cell_basic_cell_height();
}

static void prv_group_window_draw_header(GContext *ctx, const Layer *cell_layer,
                                         uint16_t section_index, void *context) {
  NotificationGroupWindow *group_window = context;
  menu_cell_basic_header_draw(ctx, cell_layer, group_window->sender);
}

static void prv_group_window_draw_row(GContext *ctx, const Layer *cell_layer, MenuIndex *cell_index,
                                      void *context) {
  NotificationGroupWindow *group_window = context;
  if (cell_index->row >= group_window->count) {
    return;
  }

  LoadedNotificationNode *loaded_node = prv_loaded_notification_list_load_item(
      &group_window->notifications_data->loaded_notification_list,
      &group_window->notification_ids[cell_index->row]);
  if (!loaded_node) {
    return;
  }

  TimelineItem *notification = &loaded_node->notification;
  const char *message = attribute_get_string(&notification->attr_list, AttributeIdBody, "");
  if (IS_EMPTY_STRING(message)) {
    message = attribute_get_string(&notification->attr_list, AttributeIdSubtitle, "");
  }
  if (IS_EMPTY_STRING(message)) {
    message = attribute_get_string(&notification->attr_list, AttributeIdTitle, "[Empty]");
  }

  clock_copy_time_string_timestamp(group_window->time_buffer, sizeof(group_window->time_buffer),
                                   notification->header.timestamp);
  menu_cell_basic_draw(ctx, cell_layer, message, group_window->time_buffer, NULL);
}

static void prv_group_window_select(MenuLayer *menu_layer, MenuIndex *cell_index, void *context) {
  NotificationGroupWindow *group_window = context;
  if (cell_index->row < group_window->count) {
    prv_push_single_notification_window(&group_window->notification_ids[cell_index->row]);
  }
}

static void prv_group_window_load(Window *window) {
  NotificationGroupWindow *group_window = window_get_user_data(window);
  MenuLayer *menu_layer = &group_window->menu_layer;
  menu_layer_init(menu_layer, &window->layer.bounds);
  menu_layer_set_callbacks(menu_layer, group_window,
                           &(MenuLayerCallbacks){
                               .get_num_rows = prv_group_window_get_num_rows,
                               .get_header_height = prv_group_window_get_header_height,
                               .get_cell_height = prv_group_window_get_cell_height,
                               .draw_header = prv_group_window_draw_header,
                               .draw_row = prv_group_window_draw_row,
                               .select_click = prv_group_window_select,
                           });
  menu_layer_set_normal_colors(menu_layer, GColorWhite, GColorBlack);
  menu_layer_set_highlight_colors(
      menu_layer, PBL_IF_COLOR_ELSE(DEFAULT_NOTIFICATION_COLOR, GColorBlack), GColorWhite);
  menu_layer_set_click_config_onto_window(menu_layer, window);
  menu_layer_set_scroll_wrap_around(menu_layer, false);
  layer_add_child(&window->layer, menu_layer_get_layer(menu_layer));
  menu_layer_set_selected_index(menu_layer, MenuIndex(0, 0), MenuRowAlignTop, false);
}

static void prv_group_window_unload(Window *window) {
  NotificationGroupWindow *group_window = window_get_user_data(window);
  menu_layer_deinit(&group_window->menu_layer);
  group_window->notifications_data->group_window = NULL;
  app_free(group_window->notification_ids);
  app_free(group_window->sender);
  app_free(group_window);
}

static void prv_push_group_window(NotificationsData *data, const NotificationHistoryRow *row) {
  NotificationGroupWindow *group_window = app_zalloc_check(sizeof(*group_window));
  group_window->notifications_data = data;
  group_window->count = row->group.count;
  const size_t sender_size = strlen(row->group.sender) + 1;
  group_window->sender = app_malloc_check(sender_size);
  memcpy(group_window->sender, row->group.sender, sender_size);
  group_window->notification_ids = app_malloc_check(sizeof(Uuid) * group_window->count);

  NotificationHistoryMember *member = row->group.members;
  for (uint16_t i = 0; i < group_window->count; i++) {
    group_window->notification_ids[i] = member->id;
    member = (NotificationHistoryMember *)list_get_next(&member->node);
  }

  window_init(&group_window->window, WINDOW_NAME("Notification Group"));
  window_set_user_data(&group_window->window, group_window);
  window_set_window_handlers(&group_window->window, &(WindowHandlers){
                                                        .load = prv_group_window_load,
                                                        .unload = prv_group_window_unload,
                                                    });
  data->group_window = group_window;
  app_window_stack_push(&group_window->window, true);
}

// Return true if successful
static bool prv_push_notification_window(NotificationsData *data,
                                         NotificationHistoryRow *selected_row) {
  const bool has_collapsed_groups = notifications_history_has_collapsed_groups(&data->history);
  notification_window_init_history(!has_collapsed_groups);

  // Bail if a notification came in ahead of us and created a modal window
  // before we had a chance to react to the select button event.
  if (notification_window_is_modal()) {
    return false;
  }

  if (has_collapsed_groups) {
    notification_window_add_notification_by_id(
        (Uuid *)notifications_history_row_get_latest_id(selected_row));
  } else {
    NotificationHistoryRow *row =
        (NotificationHistoryRow *)list_get_tail(&data->history.rows->node);
    while (row) {
      notification_window_add_notification_by_id(
          (Uuid *)notifications_history_row_get_latest_id(row));
      row = (NotificationHistoryRow *)list_get_prev(&row->node);
    }
  }

  notification_window_show();
  return true;
}

///////////////////
// Confirm Dialog

static void prv_dialog_unloaded(void *context) {
  NotificationsData *data = context;
  data->actionable_dialog = NULL;
}

static void prv_confirmed_handler(ClickRecognizerRef recognizer, void *context) {
  NotificationsData *data = context;
  notification_storage_reset_and_init();
  prv_loaded_notification_list_deinit(data->loaded_notification_list);
  data->loaded_notification_list = NULL;
  notifications_history_deinit(&data->history);
  prv_notifications_history_init(data);
  prv_load_notification_storage(data);
  actionable_dialog_pop(data->actionable_dialog);

  // Create and display DONE dialog
  SimpleDialog *confirmation_dialog = simple_dialog_create("Notifications Cleared");
  Dialog *dialog = simple_dialog_get_dialog(confirmation_dialog);
  dialog_set_text(dialog, i18n_get("Done", data));
  dialog_set_icon(dialog, RESOURCE_ID_RESULT_SHREDDED_LARGE);
  static const uint32_t DIALOG_TIMEOUT = 2000;
  dialog_set_timeout(dialog, DIALOG_TIMEOUT);

  // Set the app exit reason so we will go to the watchface upon exit
  app_exit_reason_set(APP_EXIT_ACTION_PERFORMED_SUCCESSFULLY);

  // Pop all windows so we'll soon exit the app
  app_window_stack_pop_all(true /* animated */);

  // Immediately push this result dialog so it's the last thing we see before exiting
  app_simple_dialog_push(confirmation_dialog);
}


static void prv_dialog_click_config(void *context) {
  NotificationsData *data = app_state_get_user_data();
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_confirmed_handler);
  window_set_click_context(BUTTON_ID_SELECT, data);
}

static void prv_settings_clear_history_window_push(NotificationsData *data) {
  ActionableDialog *actionable_dialog = actionable_dialog_create("Clear Notifications");
  actionable_dialog_set_click_config_provider(actionable_dialog, prv_dialog_click_config);
  actionable_dialog_set_action_bar_type(actionable_dialog, DialogActionBarConfirm, NULL);
  Dialog *dialog = actionable_dialog_get_dialog(actionable_dialog);
  dialog_set_text(dialog, i18n_get("Clear history?", data));
  TimelineResourceInfo timeline_res = {
    .res_id = TIMELINE_RESOURCE_GENERIC_QUESTION,
  };
  AppResourceInfo icon_res_info;
  timeline_resources_get_id(&timeline_res, TimelineResourceSizeLarge, &icon_res_info);
  dialog_set_icon(dialog, icon_res_info.res_id);
  dialog_set_icon_animate_direction(dialog, DialogIconAnimationFromRight);
  dialog_set_callbacks(dialog, &(DialogCallbacks) {
    .unload = prv_dialog_unloaded,
  }, data);
  app_actionable_dialog_push(actionable_dialog);
  data->actionable_dialog = actionable_dialog;
}

#if PBL_BW
static GColor prv_invert_bw_color(GColor color) {
  if (gcolor_equal(color, GColorBlack)) {
    return GColorWhite;
  } else if (gcolor_equal(color, GColorWhite)) {
    return GColorBlack;
  }
  return color;
}

static void prv_invert_pdc_colors(GDrawCommandProcessor *processor,
                                  GDrawCommand *processed_command,
                                  size_t processed_command_max_size,
                                  const GDrawCommandList* list,
                                  const GDrawCommand *command) {
  gdraw_command_set_stroke_color(processed_command,
      prv_invert_bw_color(gdraw_command_get_stroke_color((GDrawCommand *)command)));
  gdraw_command_set_fill_color(processed_command,
      prv_invert_bw_color(gdraw_command_get_fill_color((GDrawCommand *)command)));
}

static void prv_draw_pdc_bw_inverted(GContext *ctx, GDrawCommandImage *image, GPoint offset) {
  GDrawCommandProcessor processor = {
    .command = prv_invert_pdc_colors,
  };
  gdraw_command_image_draw_processed(ctx, image, offset, &processor);
}
#endif // PBL_BW

//////////////
// MenuLayer callbacks

#if PBL_RECT
static void prv_draw_notification_cell_rect(GContext *ctx, const Layer *cell_layer,
                                            const char *title, const char *subtitle,
                                            GDrawCommandImage *icon) {
  const GRect cell_layer_bounds = cell_layer->bounds;
  const GSize icon_size = gdraw_command_image_get_bounds_size(icon);
  const int16_t icon_left_margin = menu_cell_basic_horizontal_inset();
  if (icon) {
    void (*draw_func)(GContext *, GDrawCommandImage *, GPoint) = gdraw_command_image_draw;
#if PBL_BW
    if (menu_cell_layer_is_highlighted(cell_layer)) {
      draw_func = prv_draw_pdc_bw_inverted;
    }
#endif

    // Inset the draw box from the left to leave some margin on the icon's left side
    GRect box = cell_layer_bounds;
    box.origin.x += icon_left_margin;

    // Align the icon to the left of the draw box, centered vertically
    GRect icon_rect = (GRect) { .size = gdraw_command_image_get_bounds_size(icon) };
    grect_align(&icon_rect, &box, GAlignLeft, false /* clip */);

    draw_func(ctx, icon, icon_rect.origin);
  }

  // Temporarily inset the cell layer's bounds from the left so the text doesn't draw over any
  // icon on the left
  Layer *mutable_cell_layer = (Layer *)cell_layer;
  const int text_left_margin =
      icon_left_margin + MAX(icon_size.w, ATTRIBUTE_ICON_TINY_SIZE_PX);
  mutable_cell_layer->bounds = grect_inset(cell_layer_bounds,
                                           GEdgeInsets(0, 5, 0, text_left_margin));

  const GFont title_font = system_theme_get_font_for_default_size(TextStyleFont_MenuCellTitle);
  const GFont subtitle_font = system_theme_get_font_for_default_size(TextStyleFont_Caption);
  menu_cell_basic_draw_custom(ctx, cell_layer, title_font, title, NULL /* value_font */,
                              NULL /* value */, subtitle_font, subtitle, NULL /* icon */,
                              false /* icon_on_right */, GTextOverflowModeTrailingEllipsis);

  // Restore the cell layer's bounds
  mutable_cell_layer->bounds = cell_layer_bounds;
}
#endif

//! outer_box is passed as a pointer to save stack space
static int16_t prv_draw_centered_text_line_in(GContext *ctx, GFont font, const GRect *outer_box,
                                            const char *text, GAlign align) {
  if (!text) {
    return 0;
  }

  GRect text_box = *outer_box;
  text_box.size.h = fonts_get_font_height(font);
  grect_align(&text_box, outer_box, align, true);

  graphics_draw_text(ctx, text, font, text_box, GTextOverflowModeTrailingEllipsis,
                     GTextAlignmentCenter, NULL);

  return text_box.size.h;
}

//! box is passed as a pointer to save stack space
//! after this call, box will point to the GRect where
//! the notification title was drawn
void prv_draw_notification_cell_round(GContext *ctx, const Layer *cell_layer, GRect *box,
                                      GFont const title_font, const char *title,
                                      GFont const subtitle_font, const char *subtitle,
                                      GDrawCommandImage *icon) {

  if (icon) {
    GRect icon_rect = (GRect){.size = gdraw_command_image_get_bounds_size(icon)};

    grect_align(&icon_rect, box, GAlignTop, true);
    icon_rect.origin.y += 4;

    gdraw_command_image_draw(ctx, icon, icon_rect.origin);

    // more box by icon + some margin
    const int16_t icon_space = icon_rect.origin.y + icon_rect.size.h - 12;

    // manually inset to save stack space, instead of using grect_inset
    box->origin.y += icon_space;
    box->size.h -= icon_space;
  }

  // hack: compensate for text placement inside a rect
  box->origin.y -= 4;

  if (subtitle) {
    box->size.h -= prv_draw_centered_text_line_in(ctx, subtitle_font, box, subtitle,
                                                         GAlignBottom);
  }

  if (title) {
    prv_draw_centered_text_line_in(ctx, title_font, box, title, GAlignCenter);
  }
}

#if PBL_ROUND
static void prv_draw_notification_cell_round_selected(GContext *ctx, const Layer *cell_layer,
                                                      const char *title, const char *subtitle,
                                                      GDrawCommandImage *icon) {
  // as measured from the design specs
  const int inset = 8;
  GRect frame = cell_layer->bounds;
  // manually inset the frame to save stack space, instead of using grect_inset
  frame.origin.x += inset;
  frame.origin.y += inset;
  frame.size.h -= inset * 2;
  frame.size.w -= inset * 2;
  const GFont title_font = system_theme_get_font_for_default_size(TextStyleFont_MenuCellTitle);
  const GFont subtitle_font =
      system_theme_get_font_for_default_size(TextStyleFont_MenuCellSubtitle);
  prv_draw_notification_cell_round(ctx, cell_layer, &frame, title_font, title, subtitle_font,
                                   subtitle, icon);
}

static void prv_draw_notification_cell_round_unselected(GContext *ctx, const Layer *cell_layer,
                                                        const char *title, const char *subtitle,
                                                        GDrawCommandImage *icon) {
  // as measured from the design specs
  const int horizontal_inset = MENU_CELL_ROUND_UNFOCUSED_HORIZONTAL_INSET;
  const int top_inset = 2;
  GRect frame = cell_layer->bounds;
  // manually inset the frame to save stack space, instead of using grect_inset
  frame.origin.x += horizontal_inset;
  frame.size.w -= horizontal_inset * 2;
  frame.origin.y += top_inset;
  frame.size.h -= top_inset;
  // Using TextStyleFont_Header here is a little bit of a hack to achieve Gothic 18 Bold on
  // Spalding's default content size (medium) while still being a little robust for any future round
  // watches that have a default content size larger than medium
  const GFont font = system_theme_get_font_for_default_size(TextStyleFont_Header);
  prv_draw_notification_cell_round(ctx, cell_layer, &frame, font, title, NULL, NULL, NULL);
}
#endif

static void prv_select_callback(MenuLayer *menu_layer, MenuIndex *cell_index,
                                void *data) {
  NotificationsData *notifications_data = data;

  if (notifications_data->history.rows && (cell_index->row == 0)) {
    // Clear All button selected
    prv_settings_clear_history_window_push(notifications_data);
    return;
  }

  // shift index since the first one is hard coded to Clear
  int16_t notif_idx = cell_index->row - 1;

  NotificationHistoryRow *row =
      notifications_history_get_row(&notifications_data->history, notif_idx);
  if (!row) {
    return;
  }

  if (notifications_history_row_is_collapsed_group(row)) {
    prv_push_group_window(notifications_data, row);
    return;
  }

  bool success = prv_push_notification_window(notifications_data, row);
  if (!success) {
    // Bail if a notification came in ahead of us and created a modal window
    // before we had a chance to react to the select button event.
    return;
  }
  const bool animated = false;
  notification_window_focus_notification((Uuid *)notifications_history_row_get_latest_id(row),
                                         animated);
}

static uint16_t prv_get_num_rows_callback(struct MenuLayer *menu_layer, uint16_t section_index,
                                          void *data) {
  NotificationsData *notifications_data = data;
  NotificationHistoryRow *row = notifications_data->history.rows;
  // There's no notifications, don't draw anything
  if (!row) {
    return 0;
  }

  // add one for the CLEAR ALL at the top
  return notifications_history_get_row_count(&notifications_data->history) + 1;
}

static int16_t prv_get_cell_height(struct MenuLayer *menu_layer, MenuIndex *cell_index,
                                    void *data) {
#if PBL_ROUND
  MenuIndex selected_index = menu_layer_get_selected_index(menu_layer);
  bool is_selected = menu_index_compare(cell_index, &selected_index) == 0;
  if (is_selected) {
    return MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT;
  }
#if PBL_DISPLAY_HEIGHT >= 200
  // Larger round displays fit two unfocused rows on each side of the focused row
  return ((DISP_ROWS - STATUS_BAR_LAYER_HEIGHT * 2) - MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT) / 4;
#endif
#endif
  const PreferredContentSize runtime_platform_content_size =
      system_theme_get_default_content_size_for_runtime_platform();
  return ((int16_t[NumPreferredContentSizes]) {
    //! @note this is the same as Medium until Small is designed
    [PreferredContentSizeSmall] = PBL_IF_RECT_ELSE(46, MENU_CELL_ROUND_UNFOCUSED_SHORT_CELL_HEIGHT),
    [PreferredContentSizeMedium] = PBL_IF_RECT_ELSE(46,
                                                    MENU_CELL_ROUND_UNFOCUSED_SHORT_CELL_HEIGHT),
    [PreferredContentSizeLarge] = menu_cell_basic_cell_height(),
    //! @note this is the same as Large until ExtraLarge is designed
    [PreferredContentSizeExtraLarge] = menu_cell_basic_cell_height(),
  })[runtime_platform_content_size];
}

static const char *prv_get_group_title(NotificationsData *data, const NotificationHistoryRow *row) {
  const char *sender = row->group.sender;
  /// Notification sender followed by the number of grouped notifications
  const char *format = i18n_get("%s (%u)", data);
  const int title_length = snprintf(NULL, 0, format, sender, (unsigned int)row->group.count);
  if (title_length < 0) {
    return sender;
  }

  const size_t required_size = (size_t)title_length + 1;
  if (required_size > data->group_title_size) {
    char *group_title = app_realloc(data->group_title, required_size);
    if (!group_title) {
      return sender;
    }
    data->group_title = group_title;
    data->group_title_size = required_size;
  }

  snprintf(data->group_title, data->group_title_size, format, sender,
           (unsigned int)row->group.count);
  return data->group_title;
}

static void prv_draw_row_callback(GContext *ctx, const Layer *cell_layer, MenuIndex *cell_index,
                                  void *data) {
  NotificationsData *notifications_data = data;

  void (*draw_cell)(GContext *, const Layer *, const char *, const char *, GDrawCommandImage *) =
    PBL_IF_RECT_ELSE(prv_draw_notification_cell_rect, prv_draw_notification_cell_round_selected);
#if PBL_ROUND
  // on round: just draw the title for anything but the focused row
  if (!menu_layer_is_index_selected(&s_data->menu_layer, cell_index)) {
    draw_cell = prv_draw_notification_cell_round_unselected;
  }
#endif

  bool first_row = (cell_index->row == 0);
  // Test if there are any notifications in the list.
  if (first_row) {
    // Draw "Clear all" box and exit
#if PBL_ROUND
    draw_cell(ctx, cell_layer, i18n_get("Clear All", data), NULL, NULL);
#else
    const GFont font = system_theme_get_font_for_default_size(TextStyleFont_MenuCellTitle);
    GRect box = cell_layer->bounds;
    box.origin.y += (box.size.h - fonts_get_font_height(font)) / 2 - fonts_get_font_cap_offset(font);

    graphics_draw_text(ctx, i18n_get("Clear All", data), font, box,
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
#endif
    return;
  }

  // shift index since the first one is hard coded to Clear
  const int16_t notif_idx = cell_index->row - 1;

  NotificationHistoryRow *row =
      notifications_history_get_row(&notifications_data->history, notif_idx);
  if (!row) {
    return;
  }

  LoadedNotificationNode *loaded_node = prv_loaded_notification_list_load_item(
      &notifications_data->loaded_notification_list, notifications_history_row_get_latest_id(row));
  if (!loaded_node) {
    return;
  }

  TimelineItem *notification = &loaded_node->notification;
  const char *title = attribute_get_string(&notification->attr_list, AttributeIdTitle, "");
  const char *subtitle = attribute_get_string(&notification->attr_list, AttributeIdSubtitle, "");
  const char *app_name = attribute_get_string(&notification->attr_list, AttributeIdAppName, "");
  const char *body = attribute_get_string(&notification->attr_list, AttributeIdBody, "");

  // We show the app name if we don't have a custom icon, otherwise we use the title
  if (!IS_EMPTY_STRING(app_name) && loaded_node->icon_is_default) {
    title = app_name;
  }

  if (!IS_EMPTY_STRING(title) && !IS_EMPTY_STRING(subtitle)) {
    // we got a title & subtitle, we're done
  } else if (IS_EMPTY_STRING(title) && IS_EMPTY_STRING(subtitle)) {
    // we got neither, use the body
    if (IS_EMPTY_STRING(body)) {
      // we're screwed... empty message
      title = "[Empty]";
    } else {
      // try to show as much content as possible in title + subtitle
      title = body;
      subtitle = strchr(body, '\n'); // NULL handled gracefully downstream
    }
  } else if (IS_EMPTY_STRING(title)) {
    // no title, but yes subtitle.
    title = subtitle;
    subtitle = body;
  } else if (IS_EMPTY_STRING(subtitle)) {
    // no subtitle, but yes title
    subtitle = body;
  } else {
    WTF;
  }

  if (notifications_history_row_is_collapsed_group(row)) {
    title = prv_get_group_title(notifications_data, row);
    subtitle = !IS_EMPTY_STRING(body) ? body : subtitle;
  }

  draw_cell(ctx, cell_layer, title, subtitle, loaded_node->icon);
}

// Display the appropriate layer
static void prv_update_text_layer_visibility(NotificationsData *data) {
  NotificationHistoryRow *row = data->history.rows;

  // Toggle which layer is visible
  if (row == NULL) {
    layer_set_hidden((Layer *) &data->menu_layer, true);
    layer_set_hidden((Layer *) &data->text_layer, false);
  } else {
    layer_set_hidden((Layer *) &data->menu_layer, false);
    layer_set_hidden((Layer *) &data->text_layer, true);
  }
}

static void prv_group_window_remove_notification(NotificationsData *data, const Uuid *id) {
  NotificationGroupWindow *group_window = data->group_window;
  if (!group_window) {
    return;
  }

  for (uint16_t i = 0; i < group_window->count; i++) {
    if (!uuid_equal(&group_window->notification_ids[i], id)) {
      continue;
    }

    group_window->count--;
    memmove(&group_window->notification_ids[i], &group_window->notification_ids[i + 1],
            sizeof(Uuid) * (group_window->count - i));
    if (group_window->count <= 1) {
      app_window_stack_remove(&group_window->window, false);
      return;
    }

    menu_layer_reload_data(&group_window->menu_layer);
    const uint16_t selected_row = (i < group_window->count) ? i : group_window->count - 1;
    menu_layer_set_selected_index(&group_window->menu_layer, MenuIndex(0, selected_row),
                                  MenuRowAlignCenter, false);
    return;
  }
}

static void prv_group_window_add_notification(NotificationsData *data, const Uuid *id) {
  NotificationGroupWindow *group_window = data->group_window;
  if (!group_window) {
    return;
  }

  NotificationHistoryRow *row = data->history.rows;
  while (row) {
    if (row->is_group && strcmp(row->group.sender, group_window->sender) == 0) {
      NotificationHistoryMember *member = row->group.members;
      uint16_t member_index = 0;
      while (member) {
        if (uuid_equal(&member->id, id)) {
          Uuid *notification_ids =
              app_realloc(group_window->notification_ids, sizeof(Uuid) * (group_window->count + 1));
          if (!notification_ids) {
            return;
          }

          group_window->notification_ids = notification_ids;
          memmove(&group_window->notification_ids[member_index + 1],
                  &group_window->notification_ids[member_index],
                  sizeof(Uuid) * (group_window->count - member_index));
          group_window->notification_ids[member_index] = *id;
          group_window->count++;
          menu_layer_reload_data(&group_window->menu_layer);
          menu_layer_set_selected_index(&group_window->menu_layer, MenuIndex(0, member_index),
                                        MenuRowAlignCenter, false);
          return;
        }
        member = (NotificationHistoryMember *)list_get_next(&member->node);
        member_index++;
      }
    }
    row = (NotificationHistoryRow *)list_get_next(&row->node);
  }
}

static void prv_handle_notification_removed(Uuid *id) {
  prv_group_window_remove_notification(s_data, id);
  notifications_history_remove(&s_data->history, id);
  app_notification_window_remove_notification_by_id(id);
}

static void prv_handle_notification_acted_upon(Uuid *id) {
  prv_group_window_remove_notification(s_data, id);
  notifications_history_remove(&s_data->history, id);
  app_notification_window_remove_notification_by_id(id);
}

static void prv_handle_notification_added(Uuid *id) {
  TimelineItem notification;
  if (!notification_storage_get(id, &notification)) {
    return;
  }

  notifications_history_add_item(&s_data->history, &notification);
  prv_group_window_add_notification(s_data, id);
  timeline_item_free_allocated_buffer(&notification);

  if (!notifications_history_has_collapsed_groups(&s_data->history)) {
    app_notification_window_add_new_notification_by_id(id);
  }
}

static void prv_handle_notification(PebbleEvent *e, void *context) {
  if (e->type == PEBBLE_SYS_NOTIFICATION_EVENT) {
    Uuid *id = e->sys_notification.notification_id;
    switch(e->sys_notification.type) {
      case NotificationAdded:
        prv_handle_notification_added(id);
        break;
      case NotificationRemoved:
        prv_handle_notification_removed(id);
        break;
      case NotificationActedUpon:
        prv_handle_notification_acted_upon(id);
        break;
      case NotificationActionResult: {
        PebbleSysNotificationActionResult *action_result = e->sys_notification.action_result;
        if (action_result &&
            (action_result->type == ActionResultTypeSuccess ||
             action_result->type == ActionResultTypeSuccessANCSDismiss)) {
          prv_group_window_remove_notification(s_data, &action_result->id);
          notifications_history_remove(&s_data->history, &action_result->id);
          app_notification_window_remove_notification_by_id(&action_result->id);
        }
        break;
      }
      default:
        break;
        // Not implemented
    }
    menu_layer_reload_data(&s_data->menu_layer);
    prv_update_text_layer_visibility(s_data);
  }
  // we don't handle reminders within the notifications app
}

///////////////////
// Window callbacks

static void prv_window_appear(Window *window) {
  NotificationsData *data = window_get_user_data(window);

  prv_update_text_layer_visibility(data);
}

static void prv_window_disappear(Window *window) {
  NotificationsData *data = window_get_user_data(window);
  prv_loaded_notification_list_deinit(data->loaded_notification_list);
  data->loaded_notification_list = NULL;
}

static void prv_window_load(Window *window) {
  NotificationsData *data = window_get_user_data(window);
  MenuLayer *menu_layer = &data->menu_layer;
  const GRect menu_layer_frame = PBL_IF_RECT_ELSE(
      window->layer.bounds, grect_inset_internal(window->layer.bounds, 0, STATUS_BAR_LAYER_HEIGHT));
  menu_layer_init(menu_layer, &menu_layer_frame);
  menu_layer_set_callbacks(menu_layer, data, &(MenuLayerCallbacks) {
      .get_num_rows = prv_get_num_rows_callback,
      .draw_row = prv_draw_row_callback,
      .get_cell_height = prv_get_cell_height,
      .select_click = prv_select_callback,
  });

  menu_layer_set_normal_colors(menu_layer, GColorWhite, GColorBlack);
  menu_layer_set_highlight_colors(menu_layer,
                                  PBL_IF_COLOR_ELSE(DEFAULT_NOTIFICATION_COLOR, GColorBlack),
                                  GColorWhite);

  menu_layer_set_click_config_onto_window(menu_layer, window);
  menu_layer_set_scroll_wrap_around(menu_layer, shell_prefs_get_menu_scroll_wrap_around_enable());
  menu_layer_set_scroll_vibe_on_wrap(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
  menu_layer_set_scroll_vibe_on_blocked(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);
  layer_add_child(&window->layer, menu_layer_get_layer(menu_layer));

  TextLayer *text_layer = &data->text_layer;
  const int16_t horizontal_margin = 5;
  const GFont font = system_theme_get_font_for_default_size(TextStyleFont_MenuCellTitle);
  // configure text layer to be vertically aligned (15 is hacking around our poor fonts)
  text_layer_init_with_parameters(text_layer,
                                  &GRect(horizontal_margin, window->layer.bounds.size.h / 2 - 15,
                                         window->layer.bounds.size.w - horizontal_margin,
                                         window->layer.bounds.size.h / 2),
                                  i18n_get("No notifications", data), font, GColorBlack,
                                  GColorWhite, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, text_layer_get_layer(text_layer));

#if PBL_ROUND
  GColor bg_color = GColorClear;
  GColor fg_color = GColorBlack;

  StatusBarLayer *status_bar = &data->status_bar_layer;
  status_bar_layer_init(status_bar);
  status_bar_layer_set_colors(status_bar, bg_color, fg_color);
  layer_add_child(&window->layer, &status_bar->layer);
#endif

  menu_layer_set_selected_index(menu_layer, MenuIndex(0, 1),
                                PBL_IF_RECT_ELSE(MenuRowAlignNone, MenuRowAlignCenter), false);
}

static void prv_push_window(NotificationsData *data) {
  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Notifications"));
  window_set_user_data(window, data);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_window_load,
    .appear = prv_window_appear,
    .disappear = prv_window_disappear,
  });

  const bool animated = true;
  app_window_stack_push(window, animated);
}

////////////////////
// App boilerplate

static void prv_handle_init(void) {
  NotificationsData *data = s_data = app_zalloc_check(sizeof(NotificationsData));

  app_state_set_user_data(data);

  data->notification_event_info = (EventServiceInfo) {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .handler = prv_handle_notification,
  };
  event_service_client_subscribe(&data->notification_event_info);
  prv_notifications_history_init(data);
  prv_load_notification_storage(data);

  prv_push_window(data);
}

static void prv_handle_deinit(void) {
  NotificationsData *data = app_state_get_user_data();
#if PBL_ROUND
  status_bar_layer_deinit(&data->status_bar_layer);
#endif
  menu_layer_deinit(&data->menu_layer);
  event_service_client_unsubscribe(&data->notification_event_info);
  prv_loaded_notification_list_deinit(data->loaded_notification_list);
  notifications_history_deinit(&data->history);
  app_free(data->group_title);

  i18n_free_all(data);
  app_free(data);
  s_data = NULL;
}

static void prv_s_main(void) {
  prv_handle_init();

  app_event_loop();

  prv_handle_deinit();
}

// Launch the existing clear-history confirmation flow without opening the list UI.
static void prv_clear_history_handle_init(void) {
  // Reuse the existing dialog callbacks, which expect NotificationsData storage.
  NotificationsData *data = app_zalloc_check(sizeof(NotificationsData));

  app_state_set_user_data(data);
  prv_settings_clear_history_window_push(data);
}

static void prv_clear_history_handle_deinit(void) {
  NotificationsData *data = app_state_get_user_data();

  i18n_free_all(data);
  app_free(data);
}

static void prv_clear_history_main(void) {
  prv_clear_history_handle_init();

  app_event_loop();

  prv_clear_history_handle_deinit();
}


const PebbleProcessMd* notifications_app_get_info() {
  static const PebbleProcessMdSystem s_app_md = {
    .common = {
      .main_func = prv_s_main,
      // UUID: b2cae818-10f8-46df-ad2b-98ad2254a3c1
      .uuid = {0xb2, 0xca, 0xe8, 0x18, 0x10, 0xf8, 0x46, 0xdf,
               0xad, 0x2b, 0x98, 0xad, 0x22, 0x54, 0xa3, 0xc1},
    },
    .name = i18n_noop("Notifications"),
    .icon_resource_id = RESOURCE_ID_NOTIFICATIONS_APP_GLANCE,
  };
  return (const PebbleProcessMd*) &s_app_md;
}

const PebbleProcessMd *notifications_clear_history_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_md = {
    .common = {
      .main_func = prv_clear_history_main,
      .uuid = NOTIFICATIONS_CLEAR_HISTORY_UUID,
      .visibility = ProcessVisibilityQuickLaunch,
    },
    .name = i18n_noop("Clear Notification History"),
  };
  return (const PebbleProcessMd *) &s_app_md;
}
