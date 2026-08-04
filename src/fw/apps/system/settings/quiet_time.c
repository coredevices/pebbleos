/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "quiet_time.h"
#include "menu.h"
#include "window.h"

#include "applib/ui/action_menu_window_private.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/menu_layer.h"
#include "applib/ui/time_range_selection_window.h"
#include "kernel/pbl_malloc.h"
#include "popups/health_tracking_ui.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/notifications/alerts_private.h"
#include "pbl/services/notifications/do_not_disturb.h"
#include "pbl/services/notifications/alerts_preferences.h"
#include "pbl/services/notifications/alerts_preferences_private.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"
#include "shell/prefs.h"

#include <stdio.h>

typedef struct {
  SettingsCallbacks callbacks;
} SettingsQuietTimeData;

typedef struct {
  SettingsCallbacks callbacks;
  char *action_menu_text;
  TimeRangeSelectionWindowData schedule_window;
  ActionMenuConfig action_menu;
} SettingsQuietTimeScheduleData;

#ifdef CONFIG_TOUCH
typedef struct {
  SettingsCallbacks callbacks;
} SettingsQuietTimeBacklightData;
#endif

enum QuietTimeItem {
  QuietTimeItemManual,
  QuietTimeItemUntilWake,
  QuietTimeItemSchedule,
  QuietTimeItemInterruptions,
  QuietTimeItemNotifications,
#ifdef CONFIG_TOUCH
  QuietTimeItemBacklight,
#else
  QuietTimeItemMotionBacklight,
#endif
#ifdef CONFIG_SPEAKER
  QuietTimeItemMuteSpeaker,
#endif
  QuietTimeItem_Count,
};

enum QuietTimeScheduleItem {
  QuietTimeScheduleItemCalendarAware,
  QuietTimeScheduleItemSleepAware,
  QuietTimeScheduleItemWeekday,
  QuietTimeScheduleItemWeekend,
  QuietTimeScheduleItem_Count,
};

#ifdef CONFIG_TOUCH
enum QuietTimeBacklightItem {
  QuietTimeBacklightItemMotion,
  QuietTimeBacklightItemTouch,
  QuietTimeBacklightItem_Count,
};
#endif

static const AlertMask s_dnd_mask_cycle[] = {
  AlertMaskAllOff,
  AlertMaskPhoneCalls,
};

static AlertMask prv_cycle_dnd_mask(void) {
  AlertMask mask = alerts_get_dnd_mask();
  int index = 0;
  for (size_t i = 0; i < ARRAY_LENGTH(s_dnd_mask_cycle); i++) {
    if (s_dnd_mask_cycle[i] == mask) {
      index = i;
      break;
    }
  }
  mask = s_dnd_mask_cycle[(index + 1) % ARRAY_LENGTH(s_dnd_mask_cycle)];
  alerts_set_dnd_mask(mask);
  return mask;
}

static const char *prv_get_dnd_mask_subtitle(void *i18n_key) {
  const char *title = NULL;
  switch (alerts_get_dnd_mask()) {
    case AlertMaskAllOff:
      title = i18n_get("Quiet All Notifications", i18n_key);
      break;
    case AlertMaskPhoneCalls:
      title = i18n_get("Allow Phone Calls", i18n_key);
      break;
    default:
      title = "???";
      break;
  }
  return title;
}


static void prv_get_dnd_time(DoNotDisturbScheduleType type, char *time_string, const uint8_t len) {
  DoNotDisturbSchedule schedule;
  do_not_disturb_get_schedule(type, &schedule);

  clock_format_time(time_string, len, schedule.from_hour, schedule.from_minute, true);
  strcat(time_string, " - ");
  uint8_t current_length = strnlen(time_string, len);
  char *buffer = time_string + current_length;
  clock_format_time(buffer, len - current_length, schedule.to_hour, schedule.to_minute, true);
}

///////////////////////////////
// DND Action Menu Window
///////////////////////////////

enum {
  DNDMenuItemDisable = 0,
  DNDMenuItemChangeSchedule,
  DNDMenuItem_Count
};

static void prv_toggle_scheduled_dnd(ActionMenu *action_menu,
                                     const ActionMenuItem *item,
                                     void *context) {
  do_not_disturb_toggle_scheduled((DoNotDisturbScheduleType) item->action_data);
}

static void prv_complete_schedule(TimeRangeSelectionWindowData *schedule_window, void *data) {
  DoNotDisturbScheduleType type = (DoNotDisturbScheduleType) data;
  DoNotDisturbSchedule schedule = {
    .from_hour = schedule_window->from.hour,
    .from_minute = schedule_window->from.minute,
    .to_hour = schedule_window->to.hour,
    .to_minute = schedule_window->to.minute,
  };

  if (schedule.from_hour == schedule.to_hour && schedule.from_minute == schedule.to_minute) {
    if ((schedule.to_minute = (schedule.to_minute + 1) % 60) == 0) {
      schedule.to_hour = (schedule.to_hour + 1) % 24;
    }
  }

  do_not_disturb_set_schedule(type, &schedule);

  const bool animated = true;
  window_stack_remove(&schedule_window->window, animated);
}

static void prv_time_range_select_window_push(DoNotDisturbScheduleType type,
                                              SettingsQuietTimeScheduleData *data) {
  DoNotDisturbSchedule schedule;
  do_not_disturb_get_schedule(type, &schedule);
  TimeRangeSelectionWindowData *schedule_window = &data->schedule_window;
  time_range_selection_window_init(schedule_window, GColorCobaltBlue,
                                   prv_complete_schedule, (void*)(uintptr_t) type);

  schedule_window->from.hour = schedule.from_hour;
  schedule_window->from.minute = schedule.from_minute;
  schedule_window->to.hour = schedule.to_hour;
  schedule_window->to.minute = schedule.to_minute;
  app_window_stack_push(&schedule_window->window, true);
}

static void prv_dnd_set_schedule(ActionMenu *action_menu,
                                    const ActionMenuItem *item,
                                    void *context) {
  DoNotDisturbScheduleType type = (DoNotDisturbScheduleType) item->action_data;
  do_not_disturb_set_schedule_enabled(type, true);
  prv_time_range_select_window_push(type, context);
}

static void prv_scheduled_dnd_menu_cleanup(ActionMenu *action_menu,
                                 const ActionMenuItem *item,
                                 void *context) {
  ActionMenuLevel *root_level = action_menu_get_root_level(action_menu);
  SettingsQuietTimeScheduleData *data = context;
  time_range_selection_window_deinit(&data->schedule_window);
  app_free(data->action_menu_text);
  i18n_free_all(&data->action_menu);
  task_free(root_level);
}

static void prv_scheduled_dnd_menu_push(DoNotDisturbScheduleType type,
                                        SettingsQuietTimeScheduleData *data) {
  data->action_menu = (ActionMenuConfig) {
    .context = data,
    .colors.background = shell_prefs_get_theme_highlight_color(),
    .did_close = prv_scheduled_dnd_menu_cleanup,
  };

  ActionMenuLevel *level =
      task_malloc_check(sizeof(ActionMenuLevel) + DNDMenuItem_Count * sizeof(ActionMenuItem));
  *level = (ActionMenuLevel) {
    .num_items = DNDMenuItem_Count,
    .display_mode = ActionMenuLevelDisplayModeWide,
  };
  const uint8_t text_max_size = 30;
  const uint8_t buffer_size = text_max_size + 22;
  data->action_menu_text = app_malloc_check(buffer_size);
  if (do_not_disturb_is_schedule_enabled(type)) {
    strncpy(data->action_menu_text, i18n_get("Disable", &data->action_menu), buffer_size);
  } else {
    strncpy(data->action_menu_text, i18n_get("Enable", &data->action_menu), text_max_size);
    strcat(data->action_menu_text, " (");
    uint8_t current_length = strnlen(data->action_menu_text, buffer_size);
    char *buffer = data->action_menu_text + current_length;
    prv_get_dnd_time(type, buffer, buffer_size - current_length);
    strcat(data->action_menu_text, ")");
  }

  level->items[DNDMenuItemDisable] = (ActionMenuItem) {
    .label = data->action_menu_text,
    .perform_action = prv_toggle_scheduled_dnd,
  };

  level->items[DNDMenuItemChangeSchedule] = (ActionMenuItem) {
    .label = i18n_get("Change Schedule", &data->action_menu),
    .perform_action = prv_dnd_set_schedule,
  };

  if (type == WeekdaySchedule) {
    level->items[DNDMenuItemDisable].action_data = (void*)(uintptr_t) WeekdaySchedule;
    level->items[DNDMenuItemChangeSchedule].action_data = (void*)(uintptr_t) WeekdaySchedule;
  } else {
    level->items[DNDMenuItemDisable].action_data = (void*)(uintptr_t) WeekendSchedule;
    level->items[DNDMenuItemChangeSchedule].action_data = (void*)(uintptr_t) WeekendSchedule;
  }

  data->action_menu.root_level = level;
  app_action_menu_open(&data->action_menu);
}

///////////////////////////////
// Schedule sub-menu
///////////////////////////////

static void prv_schedule_deinit_cb(SettingsCallbacks *context) {
  SettingsQuietTimeScheduleData *data = (SettingsQuietTimeScheduleData *) context;
  i18n_free_all(data);
  app_free(data);
}

static void prv_schedule_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                                     const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsQuietTimeScheduleData *data = (SettingsQuietTimeScheduleData *) context;
  const char *title = NULL;
  char *subtitle = NULL;
  const uint8_t buffer_length = 80;
  subtitle = app_malloc_check(buffer_length);

  switch (row) {
    case QuietTimeScheduleItemCalendarAware:
      title = i18n_get("Calendar Aware", data);
      strncpy(subtitle, do_not_disturb_is_smart_dnd_enabled() ?
                i18n_ctx_get("QuietTime", "Enabled", data) :
                i18n_ctx_get("QuietTime", "Disabled", data), buffer_length);
      break;
    case QuietTimeScheduleItemSleepAware:
      title = i18n_get("Sleep Aware", data);
      strncpy(subtitle, do_not_disturb_is_sleep_dnd_enabled() ?
                i18n_ctx_get("QuietTime", "Enabled", data) :
                i18n_ctx_get("QuietTime", "Disabled", data), buffer_length);
      break;
    case QuietTimeScheduleItemWeekday:
      title = i18n_get("Weekdays", data);
      if (do_not_disturb_is_schedule_enabled(WeekdaySchedule)) {
        prv_get_dnd_time(WeekdaySchedule, subtitle, buffer_length);
      } else {
        strncpy(subtitle, i18n_ctx_get("QuietTime", "Disabled", data), buffer_length);
      }
      break;
    case QuietTimeScheduleItemWeekend:
      title = i18n_get("Weekends", data);
      if (do_not_disturb_is_schedule_enabled(WeekendSchedule)) {
        prv_get_dnd_time(WeekendSchedule, subtitle, buffer_length);
      } else {
        strncpy(subtitle, i18n_ctx_get("QuietTime", "Disabled", data), buffer_length);
      }
      break;
    default:
        WTF;
  }
  menu_cell_basic_draw(ctx, cell_layer, title, subtitle, NULL);
  app_free(subtitle);
}

static void prv_schedule_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  SettingsQuietTimeScheduleData *data = (SettingsQuietTimeScheduleData *) context;

  switch (row) {
    case QuietTimeScheduleItemCalendarAware:
      do_not_disturb_toggle_smart_dnd();
      break;
    case QuietTimeScheduleItemSleepAware:
      if (!do_not_disturb_is_sleep_dnd_enabled() &&
          !activity_prefs_tracking_is_enabled()) {
        health_tracking_ui_feature_show_disabled();
        break;
      }
      do_not_disturb_toggle_sleep_dnd();
      break;
    case QuietTimeScheduleItemWeekday:
      prv_scheduled_dnd_menu_push(WeekdaySchedule, data);
      break;
    case QuietTimeScheduleItemWeekend:
      prv_scheduled_dnd_menu_push(WeekendSchedule, data);
      break;
    default:
        WTF;
  }
  settings_menu_reload_data(SettingsMenuItemQuietTime);
}

static uint16_t prv_schedule_num_rows_cb(SettingsCallbacks *context) {
  return QuietTimeScheduleItem_Count;
}

static void prv_schedule_submenu_push(void) {
  SettingsQuietTimeScheduleData *data = app_zalloc_check(sizeof(*data));

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_schedule_deinit_cb,
    .draw_row = prv_schedule_draw_row_cb,
    .select_click = prv_schedule_select_click_cb,
    .num_rows = prv_schedule_num_rows_cb,
  };

  Window *window = settings_window_create_with_title(SettingsMenuItemQuietTime,
                                                     i18n_noop("Schedule"), &data->callbacks);
  app_window_stack_push(window, true /* animated */);
}

static const char *prv_get_dnd_notifications_subtitle(void *i18n_key) {
  if (alerts_preferences_dnd_get_show_notifications() == DndNotificationModeHide) {
    return i18n_get("Off", i18n_key);
  }
  if (alerts_preferences_dnd_get_auto_dismiss()) {
    return i18n_get("On - Auto Dismiss", i18n_key);
  }
  return i18n_get("On - Persistent", i18n_key);
}

static void prv_cycle_dnd_notifications(void) {
  if (alerts_preferences_dnd_get_show_notifications() == DndNotificationModeHide) {
    alerts_preferences_dnd_set_show_notifications(DndNotificationModeShow);
    alerts_preferences_dnd_set_auto_dismiss(false);
  } else if (!alerts_preferences_dnd_get_auto_dismiss()) {
    alerts_preferences_dnd_set_auto_dismiss(true);
  } else {
    alerts_preferences_dnd_set_show_notifications(DndNotificationModeHide);
    alerts_preferences_dnd_set_auto_dismiss(false);
  }
}

///////////////////////////////
// Backlight sub-menu (touch boards)
///////////////////////////////

#ifdef CONFIG_TOUCH
static void prv_backlight_deinit_cb(SettingsCallbacks *context) {
  SettingsQuietTimeBacklightData *data = (SettingsQuietTimeBacklightData *) context;
  i18n_free_all(data);
  app_free(data);
}

static void prv_backlight_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                                      const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsQuietTimeBacklightData *data = (SettingsQuietTimeBacklightData *) context;
  const char *title = NULL;
  const char *subtitle = NULL;
  switch (row) {
    case QuietTimeBacklightItemMotion:
      title = i18n_noop("Motion");
      subtitle = alerts_preferences_dnd_get_motion_backlight() ? i18n_noop("On") : i18n_noop("Off");
      break;
    case QuietTimeBacklightItemTouch:
      title = i18n_noop("Touch");
      subtitle = alerts_preferences_dnd_get_touch_backlight() ? i18n_noop("On") : i18n_noop("Off");
      break;
    default:
        WTF;
  }
  menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, data), i18n_get(subtitle, data), NULL);
}

static void prv_backlight_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  switch (row) {
    case QuietTimeBacklightItemMotion:
      alerts_preferences_dnd_set_motion_backlight(!alerts_preferences_dnd_get_motion_backlight());
      break;
    case QuietTimeBacklightItemTouch:
      alerts_preferences_dnd_set_touch_backlight(!alerts_preferences_dnd_get_touch_backlight());
      break;
    default:
        WTF;
  }
  settings_menu_reload_data(SettingsMenuItemQuietTime);
}

static uint16_t prv_backlight_num_rows_cb(SettingsCallbacks *context) {
  return QuietTimeBacklightItem_Count;
}

static void prv_backlight_submenu_push(void) {
  SettingsQuietTimeBacklightData *data = app_zalloc_check(sizeof(*data));

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_backlight_deinit_cb,
    .draw_row = prv_backlight_draw_row_cb,
    .select_click = prv_backlight_select_click_cb,
    .num_rows = prv_backlight_num_rows_cb,
  };

  Window *window = settings_window_create_with_title(SettingsMenuItemQuietTime,
                                                     i18n_noop("Backlight"), &data->callbacks);
  app_window_stack_push(window, true /* animated */);
}
#endif  // CONFIG_TOUCH

///////////////////////////////
// Top-level Quiet Time menu
///////////////////////////////

static void prv_deinit_cb(SettingsCallbacks *context) {
  SettingsQuietTimeData *data = (SettingsQuietTimeData *) context;
  i18n_free_all(data);
  app_free(data);
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                            const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsQuietTimeData *data = (SettingsQuietTimeData *) context;
  const char *title = NULL;
  const char *subtitle = NULL;

  switch (row) {
    case QuietTimeItemManual:
      title = i18n_get("Manual", data);
      subtitle = do_not_disturb_is_manually_enabled() ?
                     i18n_get("On", data) : i18n_get("Off", data);
      break;
    case QuietTimeItemUntilWake:
      title = i18n_get("Until Wake", data);
      subtitle = do_not_disturb_is_until_wake_enabled() ?
                     i18n_get("On", data) : i18n_get("Off", data);
      break;
    case QuietTimeItemSchedule:
      title = i18n_get("Schedule", data);
      break;
    case QuietTimeItemInterruptions:
      title = i18n_get("Interruptions", data);
      subtitle = prv_get_dnd_mask_subtitle(data);
      break;
    case QuietTimeItemNotifications:
      title = i18n_get("Notifications", data);
      subtitle = prv_get_dnd_notifications_subtitle(data);
      break;
#ifdef CONFIG_TOUCH
    case QuietTimeItemBacklight:
      title = i18n_get("Backlight", data);
      break;
#else
    case QuietTimeItemMotionBacklight:
      title = i18n_get("Motion Backlight", data);
      subtitle = alerts_preferences_dnd_get_motion_backlight() ?
                     i18n_get("On", data) : i18n_get("Off", data);
      break;
#endif
#ifdef CONFIG_SPEAKER
    case QuietTimeItemMuteSpeaker:
      title = i18n_get("Mute Speaker", data);
      subtitle = alerts_preferences_dnd_get_mute_speaker() ?
                     i18n_get("On", data) : i18n_get("Off", data);
      break;
#endif
    default:
        WTF;
  }
  menu_cell_basic_draw(ctx, cell_layer, title, subtitle, NULL);
}

static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  switch (row) {
    case QuietTimeItemManual:
      do_not_disturb_toggle_manually_enabled(ManualDNDFirstUseSourceSettingsMenu);
      break;
    case QuietTimeItemUntilWake:
      if (!do_not_disturb_is_until_wake_enabled() &&
          !activity_prefs_tracking_is_enabled()) {
        health_tracking_ui_feature_show_disabled();
        break;
      }
      do_not_disturb_set_until_wake_enabled(!do_not_disturb_is_until_wake_enabled());
      break;
    case QuietTimeItemSchedule:
      prv_schedule_submenu_push();
      break;
    case QuietTimeItemInterruptions:
      prv_cycle_dnd_mask();
      break;
    case QuietTimeItemNotifications:
      prv_cycle_dnd_notifications();
      break;
#ifdef CONFIG_TOUCH
    case QuietTimeItemBacklight:
      prv_backlight_submenu_push();
      break;
#else
    case QuietTimeItemMotionBacklight:
      alerts_preferences_dnd_set_motion_backlight(!alerts_preferences_dnd_get_motion_backlight());
      break;
#endif
#ifdef CONFIG_SPEAKER
    case QuietTimeItemMuteSpeaker:
      alerts_preferences_dnd_set_mute_speaker(!alerts_preferences_dnd_get_mute_speaker());
      break;
#endif
    default:
        WTF;
  }
  settings_menu_reload_data(SettingsMenuItemQuietTime);
}

static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
  return QuietTimeItem_Count;
}

static Window *prv_init(void) {
  SettingsQuietTimeData* data = app_zalloc_check(sizeof(*data));

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_deinit_cb,
    .draw_row = prv_draw_row_cb,
    .select_click = prv_select_click_cb,
    .num_rows = prv_num_rows_cb,
  };

  return settings_window_create(SettingsMenuItemQuietTime, &data->callbacks);
}

const SettingsModuleMetadata *settings_quiet_time_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    .name = i18n_noop("Quiet Time"),
    .init = prv_init,
  };

  return &s_module_info;
}
