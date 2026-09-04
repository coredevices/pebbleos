/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "alarm_editor.h"

#include "applib/pbl_std/timelocal.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/day_picker.h"
#include "applib/ui/time_selection_window.h"
#include "applib/ui/ui.h"
#include "apps/system/settings/option_menu.h"
#include "kernel/pbl_malloc.h"
#include "popups/health_tracking_ui.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/alarms/alarm.h"
#include "shell/prefs.h"
#include "system/passert.h"
#include "pbl/util/size.h"

#include <string.h>

typedef struct {
  OptionMenu *alarm_type_menu;
  AlarmType alarm_type;

  TimeSelectionWindowData time_picker_window;
  bool time_picker_was_completed;

  AlarmEditorCompleteCallback complete_callback;
  void *callback_context;

  AlarmId alarm_id;
  int alarm_hour;
  int alarm_minute;
  AlarmKind alarm_kind;
  bool creating_alarm;
} AlarmEditorData;

static void prv_remove_windows(AlarmEditorData *data) {
  if (app_window_stack_contains_window(&data->time_picker_window.window)) {
    app_window_stack_remove(&data->time_picker_window.window, false);
  }
  if (data->alarm_type_menu && app_window_stack_contains_window(&data->alarm_type_menu->window)) {
    app_window_stack_remove(&data->alarm_type_menu->window, false);
  }
}

static void prv_call_complete_cancelled_if_no_alarm(AlarmEditorData *data) {
  if (data->alarm_id == ALARM_INVALID_ID && data->complete_callback) {
    data->complete_callback(CANCELLED, data->alarm_id, data->callback_context);
  }
}

static AlarmKind prv_day_picker_kind_to_alarm_kind(DayPickerKind kind) {
  switch (kind) {
    case DayPickerKindWeekdays:
      return ALARM_KIND_WEEKDAYS;
    case DayPickerKindWeekends:
      return ALARM_KIND_WEEKENDS;
    case DayPickerKindEveryday:
      return ALARM_KIND_EVERYDAY;
    case DayPickerKindCustom:
      return ALARM_KIND_CUSTOM;
    case DayPickerKindJustOnce:
      return ALARM_KIND_JUST_ONCE;
    default:
      return ALARM_KIND_EVERYDAY;
  }
}

static DayPickerKind prv_alarm_kind_to_day_picker_kind(AlarmKind kind) {
  switch (kind) {
    case ALARM_KIND_WEEKDAYS:
      return DayPickerKindWeekdays;
    case ALARM_KIND_WEEKENDS:
      return DayPickerKindWeekends;
    case ALARM_KIND_EVERYDAY:
      return DayPickerKindEveryday;
    case ALARM_KIND_CUSTOM:
      return DayPickerKindCustom;
    case ALARM_KIND_JUST_ONCE:
      return DayPickerKindJustOnce;
    default:
      return DayPickerKindEveryday;
  }
}

static void prv_day_picker_callback(DayPickerResult result, void *context) {
  AlarmEditorData *data = (AlarmEditorData *)context;

  data->alarm_kind = prv_day_picker_kind_to_alarm_kind(result.kind);

  if (data->creating_alarm) {
    if (data->alarm_kind == ALARM_KIND_CUSTOM) {
      const AlarmInfo info = {
        .hour = data->alarm_hour,
        .minute = data->alarm_minute,
        .kind = ALARM_KIND_CUSTOM,
        .scheduled_days = &result.custom_days,
        .is_smart = (data->alarm_type == AlarmType_Smart),
        .vibrate_enabled = true,
#ifdef CONFIG_SPEAKER
        .sound_enabled = false,
        .tone = AlarmTone_Reveille,
#endif
      };
      data->alarm_id = alarm_create(&info);
    } else {
      const AlarmInfo info = {
        .hour = data->alarm_hour,
        .minute = data->alarm_minute,
        .kind = data->alarm_kind,
        .is_smart = (data->alarm_type == AlarmType_Smart),
        .vibrate_enabled = true,
#ifdef CONFIG_SPEAKER
        .sound_enabled = false,
        .tone = AlarmTone_Reveille,
#endif
      };
      data->alarm_id = alarm_create(&info);
    }
    data->complete_callback(CREATED, data->alarm_id, data->callback_context);
    time_selection_window_deinit(&data->time_picker_window);
    prv_remove_windows(data);
    i18n_free_all(data);
    task_free(data);
  } else {
    if (data->alarm_kind == ALARM_KIND_CUSTOM) {
      alarm_set_custom(data->alarm_id, result.custom_days);
    } else {
      alarm_set_kind(data->alarm_id, data->alarm_kind);
    }
    data->complete_callback(EDITED, data->alarm_id, data->callback_context);
    i18n_free_all(data);
    task_free(data);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Time Picker

static void prv_time_picker_window_unload(Window *window) {
  AlarmEditorData *data = (AlarmEditorData *)window_get_user_data(window);
  if (data->creating_alarm) {
    return;
  }

  time_selection_window_deinit(&data->time_picker_window);

  if (data->time_picker_was_completed) {
    data->complete_callback(EDITED, data->alarm_id, data->callback_context);
  }
  i18n_free_all(data);
  task_free(data);
}

static void prv_time_picker_window_appear(Window *window) {
  AlarmEditorData *data = (AlarmEditorData *)window_get_user_data(window);
  const bool is_smart = (data->alarm_type == AlarmType_Smart);
  const char *label = (!data->creating_alarm ? i18n_noop("Change Time") :
                       is_smart ? i18n_noop("New Smart Alarm") : i18n_noop("New Alarm"));
  const char *range_text = PBL_IF_RECT_ELSE(i18n_noop("Wake up between"),
                                             i18n_noop("Wake up interval"));
  const TimeSelectionWindowConfig config = {
    .label = i18n_get(label, data),
    .range = {
      .update = true,
      .text = is_smart ? i18n_get(range_text, data) : NULL,
      .duration_m = SMART_ALARM_RANGE_S / SECONDS_PER_MINUTE,
      .enabled = is_smart,
    },
  };
  time_selection_window_configure(&data->time_picker_window, &config);
  data->time_picker_window.selection_layer.selected_cell_idx = 0;
}

static void prv_time_picker_complete(TimeSelectionWindowData *time_picker_window, void *cb_data) {
  AlarmEditorData *data = (AlarmEditorData *) cb_data;
  data->time_picker_was_completed = true;
  data->alarm_hour = time_picker_window->time_data.hour;
  data->alarm_minute = time_picker_window->time_data.minute;

  if (data->creating_alarm) {
    DayPickerResult initial = {
      .kind = DayPickerKindJustOnce,
    };
    memset(initial.custom_days, 0, sizeof(initial.custom_days));
    DayPickerConfig config = {
      .initial = initial,
      .highlight_color = ALARMS_APP_HIGHLIGHT_COLOR,
      .allow_once = true,
    };
    day_picker_push(config, prv_day_picker_callback, data);
  } else {
    alarm_set_time(data->alarm_id, data->alarm_hour, data->alarm_minute);
    app_window_stack_remove(&time_picker_window->window, true);
  }
}

static void prv_setup_time_picker_window(AlarmEditorData *data) {
  const TimeSelectionWindowConfig config = {
    .color = ALARMS_APP_HIGHLIGHT_COLOR,
    .callback = {
      .update = true,
      .complete = prv_time_picker_complete,
      .context = data,
    },
  };
  time_selection_window_init(&data->time_picker_window, &config);
  window_set_user_data(&data->time_picker_window.window, data);
  data->time_picker_window.window.window_handlers.unload = prv_time_picker_window_unload;
  data->time_picker_window.window.window_handlers.appear = prv_time_picker_window_appear;

  if (data->creating_alarm) {
    time_selection_window_set_to_current_time(&data->time_picker_window);
  } else {
    int hour, minute;
    alarm_get_hours_minutes(data->alarm_id, &hour, &minute);
    data->time_picker_window.time_data.hour = hour;
    data->time_picker_window.time_data.minute = minute;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Type Picker

static void prv_type_menu_unload(OptionMenu *option_menu, void *context) {
  AlarmEditorData *data = settings_option_menu_get_context(context);
  prv_call_complete_cancelled_if_no_alarm(data);
  data->alarm_type_menu = NULL;
}

static void prv_type_menu_select(OptionMenu *option_menu, int selection, void *context) {
  AlarmEditorData *data = settings_option_menu_get_context(context);
  data->alarm_type = selection;

  if (selection == AlarmType_Smart && !activity_prefs_tracking_is_enabled()) {
    health_tracking_ui_feature_show_disabled();
    return;
  }

  if (data->creating_alarm) {
    app_window_stack_push(&data->time_picker_window.window, true);
  } else {
    alarm_set_smart(data->alarm_id, (data->alarm_type == AlarmType_Smart));
    app_window_stack_remove(&option_menu->window, true);
  }
}

static void prv_setup_type_menu_window(AlarmEditorData *data) {
  const OptionMenuCallbacks callbacks = {
    .select = prv_type_menu_select,
    .unload = prv_type_menu_unload,
  };
  static const char *s_type_labels[AlarmTypeCount] = {
    [AlarmType_Basic] = i18n_noop("Basic Alarm"),
    [AlarmType_Smart] = i18n_noop("Smart Alarm"),
  };
  const char *title = i18n_get("New Alarm", data);
  OptionMenu *option_menu = settings_option_menu_create(
      title, OptionMenuContentType_Default, 0, &callbacks, ARRAY_LENGTH(s_type_labels),
      false /* icons_enabled */, s_type_labels, data);
  PBL_ASSERTN(option_menu);
  data->alarm_type_menu = option_menu;
  option_menu_set_highlight_colors(option_menu, ALARMS_APP_HIGHLIGHT_COLOR, GColorWhite);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Public API

Window* alarm_editor_create_new_alarm(AlarmEditorCompleteCallback complete_callback,
                                      void *callback_context) {
  AlarmEditorData* data = task_malloc_check(sizeof(AlarmEditorData));
  *data = (AlarmEditorData) {
    .alarm_id = ALARM_INVALID_ID,
    .complete_callback = complete_callback,
    .callback_context = callback_context,
    .creating_alarm = true,
  };

  prv_setup_time_picker_window(data);
  prv_setup_type_menu_window(data);
  return &data->alarm_type_menu->window;
}

void alarm_editor_update_alarm_time(AlarmId alarm_id, AlarmType alarm_type,
                                    AlarmEditorCompleteCallback complete_callback,
                                    void *callback_context) {
  AlarmEditorData* data = task_malloc_check(sizeof(AlarmEditorData));
  *data = (AlarmEditorData) {
    .alarm_id = alarm_id,
    .alarm_type = alarm_type,
    .complete_callback = complete_callback,
    .callback_context = callback_context,
  };

  prv_setup_time_picker_window(data);

  app_window_stack_push(&data->time_picker_window.window, true);
}

void alarm_editor_update_alarm_days(AlarmId alarm_id, AlarmEditorCompleteCallback complete_callback,
                                    void *callback_context) {
  AlarmEditorData* data = task_malloc_check(sizeof(AlarmEditorData));
  *data = (AlarmEditorData) {
    .alarm_id = alarm_id,
    .complete_callback = complete_callback,
    .callback_context = callback_context,
  };
  alarm_get_kind(alarm_id, &data->alarm_kind);

  DayPickerResult initial;
  initial.kind = prv_alarm_kind_to_day_picker_kind(data->alarm_kind);
  if (data->alarm_kind == ALARM_KIND_CUSTOM) {
    alarm_get_custom_days(alarm_id, initial.custom_days);
  } else {
    memset(initial.custom_days, 0, sizeof(initial.custom_days));
  }

  DayPickerConfig config = {
    .initial = initial,
    .highlight_color = ALARMS_APP_HIGHLIGHT_COLOR,
    .allow_once = true,
  };
  day_picker_push(config, prv_day_picker_callback, data);
}
