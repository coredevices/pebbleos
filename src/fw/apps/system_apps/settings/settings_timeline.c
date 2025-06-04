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

#include "settings_timeline.h"
#include "settings_option_menu.h"
#include "settings_window.h"

#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/menu_layer.h"
#include "kernel/pbl_malloc.h"
#include "services/common/i18n/i18n.h"
#include "shell/prefs.h"
#include "system/passert.h"
#include "util/size.h"

#if CAPABILITY_HAS_TIMELINE_PEEK
typedef enum TimelineSettingsVersion {
  //! Initial version or never opened
  TimelineSettingsVersion_InitialVersion = 0,
  //! 4.0 UX with Timeline Quick View (code named Peek)
  TimelineSettingsVersion_UX4WithQuickView = 1,

  TimelineSettingsVersionCount,
  //! TimelineSettingsVersion is an increasing version number. TimelineSettingsVersionCurrent must
  //! not decrement. This should ensure that the current version is always the latest.
  TimelineSettingsVersionCurrent = TimelineSettingsVersionCount - 1,
} TimelineSettingsVersion;

typedef struct SettingsTimelinePeekData {
  SettingsCallbacks callbacks;
  GFont info_font;
} SettingsTimelinePeekData;

typedef enum TimelinePeekMenuIndex {
  TimelinePeekMenuIndex_PastOnUp,
  TimelinePeekMenuIndex_Toggle,
  TimelinePeekMenuIndex_Timing,

  TimelinePeekMenuIndexCount,
  TimelinePeekMenuIndexEnabledCount = TimelinePeekMenuIndexCount,
  TimelinePeekMenuIndexDisabledCount = (TimelinePeekMenuIndex_Toggle + 1),
} TimelinePeekMenuIndex;

typedef enum PeekBeforeTimingMenuIndex {
  PeekBeforeTimingMenuIndex_StartTime,
  PeekBeforeTimingMenuIndex_5Min,
  PeekBeforeTimingMenuIndex_10Min,
  PeekBeforeTimingMenuIndex_15Min,
  PeekBeforeTimingMenuIndex_30Min,

  PeekBeforeTimingMenuIndexCount,
  PeekBeforeTimingMenuIndexDefault = PeekBeforeTimingMenuIndex_10Min,
} PeekBeforeTimingMenuIndex;

static const char *s_before_time_strings[PeekBeforeTimingMenuIndexCount] = {
  /// Shows up in the Timeline settings as a "Timing" subtitle and submenu option.
  i18n_noop("Start Time"),
  /// Shows up in the Timeline settings as a "Timing" subtitle and submenu option.
  i18n_noop("5 Min Before"),
  /// Shows up in the Timeline settings as a "Timing" subtitle and submenu option.
  i18n_noop("10 Min Before"),
  /// Shows up in the Timeline settings as a "Timing" subtitle and submenu option.
  i18n_noop("15 Min Before"),
  /// Shows up in the Timeline settings as a "Timing" subtitle and submenu option.
  i18n_noop("30 Min Before"),
};

static uint16_t s_before_time_values[PeekBeforeTimingMenuIndexCount] = {
  0, 5, 10, 15, 30,
};

static PeekBeforeTimingMenuIndex prv_before_time_min_to_index(unsigned int before_time_m) {
  if (before_time_m == 0) {
    return PeekBeforeTimingMenuIndex_StartTime;
  } else if (before_time_m <= 5) {
    return PeekBeforeTimingMenuIndex_5Min;
  } else if (before_time_m <= 10) {
    return PeekBeforeTimingMenuIndex_10Min;
  } else if (before_time_m <= 15) {
    return PeekBeforeTimingMenuIndex_15Min;
  } else if (before_time_m <= 30) {
    return PeekBeforeTimingMenuIndex_30Min;
  }
  return PeekBeforeTimingMenuIndexDefault;
}

static void prv_before_time_menu_select(OptionMenu *option_menu, int selection, void *context) {
  timeline_peek_prefs_set_before_time(s_before_time_values[selection]);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_push_before_time_menu(SettingsTimelinePeekData *data) {
  /// Shows up in the Timeline settings as the title for the "Timing" submenu window.
  const char *title = i18n_noop("Timing");
  const int selected = prv_before_time_min_to_index(timeline_peek_prefs_get_before_time());
  const OptionMenuCallbacks callbacks = {
    .select = prv_before_time_menu_select,
  };
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, selected, &callbacks,
      ARRAY_LENGTH(s_before_time_strings), true /* icons_enabled */, s_before_time_strings, data);
}

static void prv_deinit_cb(SettingsCallbacks *context) {
  i18n_free_all(context);
  app_free(context);
}

static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
  return timeline_peek_prefs_get_enabled() ? TimelinePeekMenuIndexEnabledCount :
                                             TimelinePeekMenuIndexDisabledCount;
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                            const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsTimelinePeekData *data = (SettingsTimelinePeekData *)context;
  const char *title = NULL;
  const char *subtitle = NULL;

  switch ((TimelinePeekMenuIndex)row) {
    case TimelinePeekMenuIndex_Toggle:
      /// Shows up in the Timeline settings as a toggle-able "Quick View" item.
      title = i18n_noop("Quick View");
      /// Shows up in the Timeline settings as the status under the "Quick View" toggle.
      subtitle = timeline_peek_prefs_get_enabled() ? i18n_noop("On") :
      /// Shows up in the Timeline settings as the status under the "Quick View" toggle.
                                                     i18n_noop("Off");
      break;
    case TimelinePeekMenuIndex_Timing:
      /// Shows up in the Timeline settings as the title for the menu item that controls the
      /// timing for when to begin showing the peek for an event.
      title = i18n_noop("Timing");
      subtitle = s_before_time_strings[
          prv_before_time_min_to_index(timeline_peek_prefs_get_before_time())];
      break;
    case TimelinePeekMenuIndex_PastOnUp:
      /// Shows up in the Timeline settings as the title for the menu item that controls if
      /// the up button launches Health or Timeline Past.
      title = i18n_noop("Past on Up Button");
      subtitle = timeline_prefs_get_past_on_up() ? i18n_noop("On") :
                                                   i18n_noop("Off");
      break;
    case TimelinePeekMenuIndexCount:
      break;
  }

  PBL_ASSERTN(title);
  menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, data), i18n_get(subtitle, data), NULL);
}

static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  SettingsTimelinePeekData *data = (SettingsTimelinePeekData *)context;
  switch ((TimelinePeekMenuIndex)row) {
    case TimelinePeekMenuIndex_Toggle:
      timeline_peek_prefs_set_enabled(!timeline_peek_prefs_get_enabled());
      goto done;
    case TimelinePeekMenuIndex_Timing:
      prv_push_before_time_menu(data);
      goto done;
    case TimelinePeekMenuIndex_PastOnUp:
      timeline_prefs_set_past_on_up(!timeline_prefs_get_past_on_up());
      goto done;
    case TimelinePeekMenuIndexCount:
      break;
  }
  WTF;
done:
  settings_menu_reload_data(SettingsMenuItemTimeline);
}

static Window *prv_create_settings_window(void) {
  SettingsTimelinePeekData *data = app_malloc_check(sizeof(*data));

  *data = (SettingsTimelinePeekData) {
    .callbacks = {
      .deinit = prv_deinit_cb,
      .draw_row = prv_draw_row_cb,
      .select_click = prv_select_click_cb,
      .num_rows = prv_num_rows_cb,
    },
    .info_font = fonts_get_system_font(FONT_KEY_GOTHIC_18),
  };

  return settings_window_create(SettingsMenuItemTimeline, &data->callbacks);
}

static void prv_push_settings_window(ClickRecognizerRef recognizer, void *context) {
  PBL_ASSERTN(context);
  expandable_dialog_pop(context);
  Window *window = prv_create_settings_window();
  app_window_stack_push(window, true /* animated */);
}

static Window *prv_create_first_use_dialog(void) {
  const void *i18n_owner = prv_create_first_use_dialog; // Use this function as the i18n owner
  /// Title for the Timeline Quick View first use dialog.
  const char *header = i18n_get("Quick View", i18n_owner);
  /// Help text for the Timeline Quick View first use dialog.
  const char *text = i18n_get("Appears on your watchface when an event is about to start.",
                              i18n_owner);
  ExpandableDialog *expandable_dialog = expandable_dialog_create_with_params(
      WINDOW_NAME("Timeline Quick View First Use"), RESOURCE_ID_SUNNY_DAY_TINY, text, GColorBlack,
      PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite), NULL, RESOURCE_ID_ACTION_BAR_ICON_CHECK,
      prv_push_settings_window);
  expandable_dialog_set_header(expandable_dialog, header);
#if PBL_ROUND
  expandable_dialog_set_header_font(expandable_dialog,
                                    fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
#endif
  i18n_free_all(i18n_owner);
  return &expandable_dialog->dialog.window;
}

static Window *prv_init(void) {
  const uint32_t version = timeline_prefs_get_settings_opened();
  timeline_prefs_set_settings_opened(TimelineSettingsVersionCurrent);
  if (version == TimelineSettingsVersion_InitialVersion) {
    return prv_create_first_use_dialog();
  } else {
    return prv_create_settings_window();
  }
}

const SettingsModuleMetadata *settings_timeline_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    .name = i18n_noop("Timeline"),
    .init = prv_init,
  };

  return &s_module_info;
}
#endif // CAPABILITY_HAS_TIMELINE_PEEK
