/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "menu.h"
#include "option_menu.h"
#include "time.h"
#include "window.h"

#include "applib/app.h"
#include "applib/ui/action_menu_window_private.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/date_selection_window.h"
#include "applib/ui/option_menu_window.h"
#include "applib/ui/time_selection_window.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "util/date.h"
#include "util/time/time.h"
#include "pbl/util/string.h"

#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/timezone_database.h"
#include "shell/prefs.h"

#include <stdio.h>
#include <time.h>

// 9 (TZ) continents: Africa, America, Antarctica, Asia, Atlantic, Australia,
// Europe, Indian, Pacific
#define NUM_CONTINENTS 9

typedef struct {
  SettingsCallbacks callbacks;

  MenuLayer menu_layer;

  int hour;
  bool is_morning;

  // Timezone data
  uint16_t region_count;
  uint16_t continent_selected;
  uint16_t continent_start[NUM_CONTINENTS + 1]; //!< First region id for the continent
  uint16_t continent_end[NUM_CONTINENTS]; //!< Last+1 region id for the continent

  const char **continent_names;
  const char **region_names;
  char *region_names_buffer;

  ActionMenuConfig action_menu;

  Window *continent_window;

  // Manual time / date picker windows
  TimeSelectionWindowData time_picker;
  DateSelectionWindowData date_picker;
} SettingsTimeData;

typedef enum {
  TimeRow_TimeSource,
  TimeRow_SetTime,
  TimeRow_SetDate,
  TimeRow_Format,
  TimeRow_TimezoneSource,
  TimeRow_Timezone,
  TimeRowNum,
} TimeRow;

//! Set Time and Set Date are hidden when the time source is automatic;
//! Timezone is hidden when the timezone source is automatic.
static bool prv_row_visible(TimeRow row) {
  switch (row) {
    case TimeRow_SetTime:
    case TimeRow_SetDate:
      return clock_time_source_is_manual();
    case TimeRow_Timezone:
      return clock_timezone_source_is_manual();
    default:
      return true;
  }
}

//! Map a visible row index to its TimeRow enum value.
static TimeRow prv_row_for_index(uint16_t index) {
  for (TimeRow row = 0; row < TimeRowNum; row++) {
    if (prv_row_visible(row) && (index-- == 0)) {
      return row;
    }
  }
  return TimeRowNum;
}

static uint16_t prv_visible_row_count(void) {
  uint16_t count = 0;
  for (TimeRow row = 0; row < TimeRowNum; row++) {
    if (prv_row_visible(row)) {
      count++;
    }
  }
  return count;
}


// Timezone Window Setup
////////////////////////////

static void prv_format_region_name(char *region_name) {
  const size_t string_length = strnlen(region_name, TIMEZONE_NAME_LENGTH);
  for (size_t i = 0; i < string_length; i++) {
    if (region_name[i] == '_') {
      region_name[i] = ' ';
    }
  }
}

//! Initialize the continent and region names for the timezone windows
static void prv_init_continent_and_region_names(SettingsTimeData *data) {
  const uint16_t region_count = data->region_count = timezone_database_get_region_count();
  char * const region_names_buffer = app_zalloc_check(region_count * TIMEZONE_NAME_LENGTH);
  char *cursor = region_names_buffer;
  char *last_cursor = cursor;
  const char **continent_names = app_zalloc_check(NUM_CONTINENTS * sizeof(char *));
  const char **region_names = app_zalloc_check(region_count * sizeof(char *));
  uint16_t continent_index = 0;
  // Iterate through the region IDs to sort out the region IDs into continents.
  // We have sorted the region IDs by name, so each continent _will_ be separate from each other.
  data->continent_start[continent_index] = 0;
  for (uint16_t i = 0; i < region_count; i++, cursor += TIMEZONE_NAME_LENGTH) {
    timezone_database_load_region_name(i, cursor);
    prv_format_region_name(cursor);
    // Split 'Continent/City' into the two parts
    const int sep_pos = strcspn(cursor, "/");
    cursor[sep_pos] = '\0';
    // Store pointer to region name
    region_names[i] = cursor + sep_pos + 1;
    // If the continent is the same as the last entry, keep going as-is
    if (strcmp(cursor, last_cursor) == 0) {
      data->continent_end[continent_index] = i + 1;
      continue;
    }
    // If the new continent is filtered out, don't create a new continent or update the last
    // continent pointer.
    if (strcmp(cursor, "Etc") == 0) { // Filter out 'Etc' fake-continent
      continue;
    }
    // Set pointer for the continent name
    continent_names[continent_index] = last_cursor;
    // Set the new continent name
    last_cursor = cursor;
    // Set the starting region ID for the new continent.
    data->continent_start[++continent_index] = i;
  }
  // Save the last continent name
  continent_names[continent_index] = last_cursor;

  data->region_names_buffer = region_names_buffer;
  data->region_names = region_names;
  data->continent_names = continent_names;
}

static char *prv_get_timezone_title(void) {
  /// Title of the menu for changing the watch's timezone.
  return i18n_noop("Timezone");
}

// Timezone Region Menu
/////////////////////////

static void prv_region_menu_select(OptionMenu *option_menu, int selection, void *context) {
  SettingsTimeData *data = ((SettingsOptionMenuData *)context)->context;
  const uint16_t region_id = data->continent_start[data->continent_selected] + selection;
  clock_set_timezone_by_region_id(region_id);

  const bool continent_animated = false;
  app_window_stack_remove(data->continent_window, continent_animated);
  const bool region_animated = true;
  app_window_stack_remove(&option_menu->window, region_animated);
}

static void prv_region_menu_push(SettingsTimeData *data) {
  const char *title = prv_get_timezone_title();
  const OptionMenuCallbacks callbacks = {
    .select = prv_region_menu_select,
  };
  const int start_index = data->continent_start[data->continent_selected];
  const int end_index = data->continent_end[data->continent_selected];
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, OPTION_MENU_CHOICE_NONE, &callbacks,
      end_index - start_index, true /* icons_enabled */, &data->region_names[start_index], data);
}

// Timezone Continent Menu
/////////////////////////

static void prv_continent_menu_select(OptionMenu *option_menu, int selection, void *context) {
  SettingsTimeData *data = ((SettingsOptionMenuData *)context)->context;
  data->continent_selected = selection;
  prv_region_menu_push(data);
}

static void prv_continent_menu_push(SettingsTimeData *data) {
  const char *title = prv_get_timezone_title();
  const OptionMenuCallbacks callbacks =  {
    .select = prv_continent_menu_select,
  };
  OptionMenu * const continent_menu = settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, OPTION_MENU_CHOICE_NONE, &callbacks, NUM_CONTINENTS,
      false /* icons_enabled */, data->continent_names, data);
  data->continent_window = &continent_menu->window;
}

// 24h Switch
/////////////////////////

static void prv_cycle_clock_style(void) {
  clock_set_24h_style(!clock_is_24h_style());
}

static void prv_cycle_clock_time_source(void) {
  const bool was_manual = clock_time_source_is_manual();
  clock_set_manual_time_source(!was_manual);

  if (was_manual) {
    // Switching from manual to automatic: ask the phone to send its current time
    clock_request_time_from_phone();
  }
}

static void prv_cycle_clock_timezone_source(void) {
  clock_set_manual_timezone_source(!clock_timezone_source_is_manual());

  if (!clock_timezone_source_is_manual()) {
    clock_set_timezone_by_region_id(shell_prefs_get_automatic_timezone_id());
  }
}

// Set Time
/////////////////////////

static void prv_time_picker_complete(TimeSelectionWindowData *picker, void *context) {
  // Build a new UTC timestamp: keep today's local date, update hours/minutes.
  // Seconds are reset to zero when manually setting the time.
  struct tm now;
  clock_get_time_tm(&now);
  now.tm_hour = picker->time_data.hour;
  now.tm_min = picker->time_data.minute;
  now.tm_sec = 0;
  // mktime interprets struct tm as local time and returns a UTC epoch
  time_t new_utc = mktime(&now);
  clock_set_time(new_utc);

  const bool animated = true;
  app_window_stack_remove(&picker->window, animated);
}

static void prv_time_picker_push(SettingsTimeData *data) {
  TimeSelectionWindowData *picker = &data->time_picker;

  // Deinit any previous state before re-initializing (safe on zeroed struct)
  time_selection_window_deinit(picker);

  const TimeSelectionWindowConfig config = {
    .label = i18n_noop("Set Time"),
    .color = PBL_IF_COLOR_ELSE(GColorJaegerGreen, GColorBlack),
    .range = { .update = true, .enabled = false },
    .callback = {
      .update = true,
      .complete = prv_time_picker_complete,
      .context = data,
    },
  };
  time_selection_window_init(picker, &config);
  time_selection_window_set_to_current_time(picker);

  app_window_stack_push(&picker->window, true /* animated */);
}

// Set Date
/////////////////////////

static void prv_date_picker_complete(DateSelectionWindowData *picker, void *context) {
  // Build a new UTC timestamp: keep today's local time-of-day (hour/min/sec), update year/month/day
  struct tm now;
  clock_get_time_tm(&now);
  now.tm_year = picker->date.year;
  now.tm_mon = picker->date.month;
  now.tm_mday = picker->date.day;
  // mktime normalises any out-of-range fields and converts local → UTC
  time_t new_utc = mktime(&now);
  clock_set_time(new_utc);

  const bool animated = true;
  app_window_stack_remove(&picker->window, animated);
}

static void prv_date_picker_push(SettingsTimeData *data) {
  DateSelectionWindowData *picker = &data->date_picker;

  // Deinit any previous state before re-initializing (safe on zeroed struct)
  date_selection_window_deinit(picker);

  date_selection_window_init(picker, i18n_noop("Set Date"),
                             PBL_IF_COLOR_ELSE(GColorJaegerGreen, GColorBlack),
                             prv_date_picker_complete, data);
  date_selection_window_set_to_current_date(picker);

  app_window_stack_push(&picker->window, true /* animated */);
}

// Date & Time Menu
////////////////////////////
static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  SettingsTimeData *data = (SettingsTimeData*) context;
  switch (prv_row_for_index(row)) {
    case TimeRow_TimeSource:
      // Toggle automatic / manual time
      prv_cycle_clock_time_source();
      settings_menu_reload_data(SettingsMenuItemDateTime);
      return;
    case TimeRow_SetTime:
      prv_time_picker_push(data);
      return;  // mark dirty happens via window unload callback
    case TimeRow_SetDate:
      prv_date_picker_push(data);
      return;  // mark dirty happens via window unload callback
    case TimeRow_Format:
      // Set Time Display
      prv_cycle_clock_style();
      break;
    case TimeRow_TimezoneSource:
      // Toggle automatic / manual timezone; the Timezone row is shown/hidden
      prv_cycle_clock_timezone_source();
      settings_menu_reload_data(SettingsMenuItemDateTime);
      return;
    case TimeRow_Timezone:
      // Set Timezone Region; row is hidden while source is automatic
      if (!clock_timezone_source_is_manual()) {
        return;
      }
      prv_continent_menu_push(data);
      break;
    default:
      break;
  }
  settings_menu_mark_dirty(SettingsMenuItemDateTime);
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                            const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsTimeData *data = (SettingsTimeData*) context;

  const char *title = NULL;
  const char *subtitle = NULL;
  char current_timezone_region[TIMEZONE_NAME_LENGTH];
  char tz_source_buf[TIMEZONE_NAME_LENGTH + 32];
  char time_buf[TIME_STRING_TIME_LENGTH];
  char date_buf[16];

  switch (prv_row_for_index(row)) {
    case TimeRow_TimeSource: {
      title = i18n_noop("Time Source");
      subtitle = clock_time_source_is_manual() ? i18n_noop("Manual") :
                                                 i18n_noop("Automatic");
      break;
    }
    case TimeRow_SetTime: {
      title = i18n_noop("Set Time");
      struct tm local_now;
      clock_get_time_tm(&local_now);
      clock_format_time(time_buf, sizeof(time_buf),
                        local_now.tm_hour, local_now.tm_min, true);
      subtitle = time_buf;
      break;
    }
    case TimeRow_SetDate: {
      title = i18n_noop("Set Date");
      struct tm local_now;
      clock_get_time_tm(&local_now);
      strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", &local_now);
      subtitle = date_buf;
      break;
    }
    case TimeRow_Format: {
      title = i18n_noop("Time Format");
      subtitle = clock_is_24h_style() ? i18n_noop("24h") : i18n_noop("12h");
      break;
    }
    case TimeRow_TimezoneSource: {
      title = i18n_noop("Timezone Source");
      if (clock_timezone_source_is_manual()) {
        subtitle = i18n_noop("Manual");
      } else if (clock_is_timezone_set()) {
        // Show the active timezone since the Timezone row is hidden
        clock_get_timezone_region(current_timezone_region, TIMEZONE_NAME_LENGTH);
        snprintf(tz_source_buf, sizeof(tz_source_buf), "%s (%s)",
                 i18n_get("Automatic", data), current_timezone_region);
        subtitle = tz_source_buf;
      } else {
        subtitle = i18n_noop("Automatic");
      }
      break;
    }
    case TimeRow_Timezone: {
      title = i18n_noop("Timezone");
      clock_get_timezone_region(current_timezone_region, TIMEZONE_NAME_LENGTH);
      subtitle = current_timezone_region;
      break;
    }
    default:
      break;
  }

  menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, data), i18n_get(subtitle, data), NULL);
}

static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
  return prv_visible_row_count();
}

static void prv_deinit_cb(SettingsCallbacks *context) {
  SettingsTimeData *data = (SettingsTimeData*) context;
  i18n_free_all(data);
  app_free(data->continent_names);
  app_free(data->region_names);
  app_free(data->region_names_buffer);
  time_selection_window_deinit(&data->time_picker);
  date_selection_window_deinit(&data->date_picker);
  app_free(data);
}

static Window *prv_init(void) {
  SettingsTimeData *data = app_malloc_check(sizeof(*data));
  *data = (SettingsTimeData){};

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_deinit_cb,
    .draw_row = prv_draw_row_cb,
    .select_click = prv_select_click_cb,
    .num_rows = prv_num_rows_cb,
  };

  prv_init_continent_and_region_names(data);

  return settings_window_create(SettingsMenuItemDateTime, &data->callbacks);
}

const SettingsModuleMetadata *settings_time_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    .name = i18n_noop("Date & Time"),
    .init = prv_init,
  };

  return &s_module_info;
}
