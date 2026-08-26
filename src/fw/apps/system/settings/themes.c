/* SPDX-FileCopyrightText: 2025 Elad Dvash */
/* SPDX-License-Identifier: Apache-2.0 */

#include "themes.h"
#include "menu.h"
#include "option_menu.h"
#include "window.h"

#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/menu_layer.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/i18n/i18n.h"
#include "shell/prefs.h"
#include "system/passert.h"
#include "applib/ui/time_range_selection_window.h"
#include "pbl/services/clock.h"
#include "pbl/util/size.h"

#ifdef CONFIG_THEMING

#define DEFAULT_THEME_HIGHLIGHT_COLOR GColorVividCerulean

typedef struct SettingsThemesData {
  SettingsCallbacks callbacks;
  TimeRangeSelectionWindowData schedule_window;
} SettingsThemesData;

typedef struct ColorDefinition {
  const char *name;
  const GColor color;
} ColorDefinition;

static const ColorDefinition s_color_definitions[11] = {
  {"Default", GColorClear},
  {"Red", GColorSunsetOrange},
  {"Orange", GColorChromeYellow},
  {"Yellow", GColorYellow},
  {"Green", GColorGreen},
  {"Cyan", GColorCyan},
  {"Light Blue", GColorVividCerulean},
  {"Royal Blue", GColorVeryLightBlue},
  {"Purple", GColorLavenderIndigo},
  {"Magenta", GColorMagenta},
  {"Pink", GColorBrilliantRose},
};
static const char* color_names[ARRAY_LENGTH(s_color_definitions)];
static bool color_names_initialized = false;

static const char** prv_get_color_names(bool short_list) {
  if (!color_names_initialized) {
    for (size_t i = 0; i < ARRAY_LENGTH(s_color_definitions); i++) {
      color_names[i] = (char*)s_color_definitions[i].name;
    }
    color_names_initialized = true;
  }
  return color_names;
}




static int prv_color_to_index(GColor color, GColor default_color) {
  if (color.argb == GColorClear.argb || color.argb == default_color.argb) {
    return 0;
  }
  for (size_t i = 0; i < ARRAY_LENGTH(s_color_definitions); i++) {
    GColor selected_color = s_color_definitions[i].color;
    if ((uint8_t)(color.argb) == (uint8_t)(selected_color.argb)) {
      return i;
    }
  }
  return -1;
}


/////////////////////////////
// Unified Accent Color Settings
/////////////////////////////

static void prv_color_menu_select(OptionMenu *option_menu, int selection, void *context) {
  GColor color;
  if (selection == 0) {
    /* Default option selected -> restore default color. */
    color = DEFAULT_THEME_HIGHLIGHT_COLOR;
  } else {
    color = s_color_definitions[selection].color;
  }

  /* Set the theme highlight color */
  shell_prefs_set_theme_highlight_color(color);

  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_option_menu_selection_will_change(OptionMenu *option_menu,
                                                   uint16_t new_row,
                                                   uint16_t old_row,
                                                   void *context) {
  if (new_row == old_row) {
    return;
  }
  GColor color = s_color_definitions[new_row].color;
  if (color.argb != GColorClear.argb) {
    option_menu_set_highlight_colors(option_menu, color, gcolor_legible_over(color));
  } else {
    option_menu_set_highlight_colors(option_menu, DEFAULT_THEME_HIGHLIGHT_COLOR, gcolor_legible_over(DEFAULT_THEME_HIGHLIGHT_COLOR));
  }
}

static OptionMenu *prv_push_color_menu(void) {
  const char *title = i18n_noop("Accent Color");
  int selected = prv_color_to_index(shell_prefs_get_theme_highlight_color(), DEFAULT_THEME_HIGHLIGHT_COLOR);
  const char** color_names = prv_get_color_names(false);
  const OptionMenuCallbacks callbacks = {
    .select = prv_color_menu_select,
    .selection_will_change = prv_option_menu_selection_will_change,
  };
  if (selected < 0) {
    // Invalid color stored - fall back to default instead of crashing
    // This can happen if an invalid color was synced from the phone
    PBL_LOG_WRN("Invalid menu color, using default");
    selected = 0;
  }
  OptionMenu * const option_menu = settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, selected, &callbacks,
      ARRAY_LENGTH(s_color_definitions), true /* icons_enabled */, color_names, NULL);

  if (option_menu) {
    if (selected == 0) {
      option_menu_set_highlight_colors(option_menu, DEFAULT_THEME_HIGHLIGHT_COLOR,
                                       gcolor_legible_over(DEFAULT_THEME_HIGHLIGHT_COLOR));
    } else {
      option_menu_set_highlight_colors(option_menu, s_color_definitions[selected].color,
                                       gcolor_legible_over(s_color_definitions[selected].color));
    }
  }

  return option_menu;
}

static const char * const s_dark_mode_labels[] = {
  [DarkModeOff] = i18n_noop("Off"),
  [DarkModeOn] = i18n_noop("On"),
  [DarkModeAmbient] = i18n_noop("Ambient"),
  [DarkModeScheduled] = i18n_noop("Schedule"),
};

static void prv_dark_mode_schedule_window_push(SettingsThemesData *data);

static void prv_dark_mode_menu_select(OptionMenu *option_menu, int selection, void *context) {
  SettingsThemesData *data = settings_option_menu_get_context(context);
  const DarkMode mode = (DarkMode)selection;
  shell_prefs_set_dark_mode(mode);
  if (mode == DarkModeScheduled) {
    app_window_stack_remove(&option_menu->window, false /* animated */);
    prv_dark_mode_schedule_window_push(data);
  } else {
    app_window_stack_remove(&option_menu->window, true /* animated */);
  }
  settings_menu_reload_data(SettingsMenuItemThemes);
  settings_menu_mark_dirty(SettingsMenuItemThemes);
}

static void prv_push_dark_mode_menu(SettingsThemesData *data) {
  const int index = (int)shell_prefs_get_dark_mode();
  const OptionMenuCallbacks callbacks = {
    .select = prv_dark_mode_menu_select,
  };
  const char *title = PBL_IF_RECT_ELSE(i18n_noop("DARK MODE"), i18n_noop("Dark Mode"));
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_dark_mode_labels), true /* icons_enabled */,
      (const char **)s_dark_mode_labels, data);
}

static void prv_complete_dark_mode_schedule(TimeRangeSelectionWindowData *schedule_window, void *data) {
  DarkModeSchedule schedule = {
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
  shell_prefs_set_dark_mode_schedule(&schedule);
  settings_menu_reload_data(SettingsMenuItemThemes);
  settings_menu_mark_dirty(SettingsMenuItemThemes);
  const bool animated = true;
  app_window_stack_remove(&schedule_window->window, animated);
}

static void prv_dark_mode_schedule_window_push(SettingsThemesData *data) {
  DarkModeSchedule schedule;
  shell_prefs_get_dark_mode_schedule(&schedule);
  TimeRangeSelectionWindowData *schedule_window = &data->schedule_window;
  time_range_selection_window_init(schedule_window, shell_prefs_get_theme_highlight_color(),
                                   prv_complete_dark_mode_schedule, data);
  schedule_window->from.hour = schedule.from_hour;
  schedule_window->from.minute = schedule.from_minute;
  schedule_window->to.hour = schedule.to_hour;
  schedule_window->to.minute = schedule.to_minute;
  app_window_stack_push(&schedule_window->window, true);
}

static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  SettingsThemesData *data = (SettingsThemesData *)context;
  const bool has_schedule = (shell_prefs_get_dark_mode() == DarkModeScheduled);
  if (row == 0) {
    prv_push_dark_mode_menu(data);
  } else if (has_schedule && row == 1) {
    prv_dark_mode_schedule_window_push(data);
  } else {
    prv_push_color_menu();
  }
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                             const Layer *cell_layer, uint16_t row, bool selected) {
  const char *title = NULL;
  const char *subtitle = NULL;
  char time_buf[32];
  const bool has_schedule = (shell_prefs_get_dark_mode() == DarkModeScheduled);
  if (row == 0) {
    title = i18n_noop("Dark Mode");
    subtitle = s_dark_mode_labels[shell_prefs_get_dark_mode()];
  } else if (has_schedule && row == 1) {
    title = i18n_noop("Schedule Time");
    DarkModeSchedule schedule;
    shell_prefs_get_dark_mode_schedule(&schedule);
    clock_format_time(time_buf, sizeof(time_buf), schedule.from_hour, schedule.from_minute, true);
    strcat(time_buf, " - ");
    const size_t len = strlen(time_buf);
    clock_format_time(time_buf + len, sizeof(time_buf) - len, schedule.to_hour, schedule.to_minute, true);
    subtitle = time_buf;
  } else {
    int idx = prv_color_to_index(shell_prefs_get_theme_highlight_color(),
                                  DEFAULT_THEME_HIGHLIGHT_COLOR);
    title = i18n_noop("Accent Color");
    subtitle = s_color_definitions[idx < 0 ? 0 : idx].name;
  }
  menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, context),
                       i18n_get(subtitle, context), NULL);
}

static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
  return (shell_prefs_get_dark_mode() == DarkModeScheduled) ? 3 : 2;
}

static void prv_deinit_cb(SettingsCallbacks *context) {
  SettingsThemesData *data = (SettingsThemesData *)context;
  time_range_selection_window_deinit(&data->schedule_window);
  i18n_free_all(context);
  app_free(context);
}

#endif // CONFIG_THEMING

static Window *prv_create_color_menu(void) {
#ifdef CONFIG_THEMING
  SettingsThemesData *data = app_malloc_check(sizeof(*data));
  *data = (SettingsThemesData){
    .callbacks = {
      .deinit = prv_deinit_cb,
      .draw_row = prv_draw_row_cb,
      .select_click = prv_select_click_cb,
      .num_rows = prv_num_rows_cb,
    },
  };
  return settings_window_create(SettingsMenuItemThemes, &data->callbacks);
#else
  WTF;
  return NULL;
#endif
}

static Window *prv_init(void) {
  return prv_create_color_menu();
}


const SettingsModuleMetadata *settings_themes_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    /// Title of the Themes Settings submenu in Settings
    .name = i18n_noop("Themes"),
    .init = prv_init,
  };

  return &s_module_info;
}
