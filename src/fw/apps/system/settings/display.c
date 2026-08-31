/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "display.h"
#include "menu.h"
#include "notifications_private.h"
#include "option_menu.h"
#include "window.h"

#include "applib/ui/ui.h"
#include <pbl/drivers/ambient_light.h>
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/light.h"
#include "shell/prefs.h"
#include "shell/system_theme.h"
#include "system/passert.h"
#include "pbl/util/size.h"

#include <stdbool.h>
#include <stdio.h>

// Forward decl so the parent menu can push the Backlight submenu.
static void prv_backlight_submenu_push(void);

typedef struct SettingsDisplayData {
  SettingsCallbacks callbacks;
} SettingsDisplayData;

typedef struct SettingsBacklightData {
  SettingsCallbacks callbacks;
  char als_value_buffer[16];  // Buffer for ALS value display
  char backlight_percent_buffer[16];  // Buffer for backlight percentage display
  AppTimer *update_timer;  // Timer for live updating debug values
  bool als_primed;  // Whether we currently hold an ambient_light_prime() ref
} SettingsBacklightData;

static const char *s_language_labels[] = {
  [ShellLanguageInstalledPack] = i18n_noop("Custom"),
  [ShellLanguageEnglish] = "English",
  [ShellLanguageCatalan] = "Català",
  [ShellLanguageGerman] = "Deutsch",
  [ShellLanguageSpanish] = "Español",
  [ShellLanguageFrench] = "Français",
  [ShellLanguageItalian] = "Italiano",
  [ShellLanguageDutch] = "Nederlands",
  [ShellLanguagePortuguese] = "Português",
  [ShellLanguagePolish] = "Polski",
};

static void prv_language_menu_select(OptionMenu *option_menu, int selection, void *context) {
  shell_prefs_set_language((ShellLanguage)selection);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_language_menu_push(SettingsDisplayData *data) {
  const OptionMenuCallbacks callbacks = {
    .select = prv_language_menu_select,
  };
  settings_option_menu_push(i18n_noop("Language"), OptionMenuContentType_SingleLine,
                            shell_prefs_get_language(), &callbacks,
                            ARRAY_LENGTH(s_language_labels), false /* icons_enabled */,
                            s_language_labels, data);
}

// Text Size
/////////////////////////////

static const char *s_text_size_names[] = {
  [SettingsContentSize_Small]   = i18n_noop("Smaller"),
  [SettingsContentSize_Default] = i18n_ctx_noop("TextSize", "Default"),
  [SettingsContentSize_Large]   = i18n_noop("Larger"),
};

static void prv_text_size_menu_select(OptionMenu *option_menu, int selection, void *context) {
  system_theme_set_content_size(settings_content_size_to_preferred_size(selection));
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_text_size_menu_push(SettingsDisplayData *data) {
  const OptionMenuCallbacks callbacks = {
    .select = prv_text_size_menu_select,
  };
  /// The option in the Settings app for choosing the text size of the system UI.
  const char *title = i18n_noop("Text Size");
  const SettingsContentSize index =
      settings_content_size_from_preferred_size(system_theme_get_content_size());
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks, SettingsContentSizeCount,
      true /* icons_enabled */, s_text_size_names, data);
}

// Intensity Settings
/////////////////////////////

static const uint32_t s_intensity_values[] = { 10, 25, 50, 100 };

static const char *s_intensity_labels[] = {
    i18n_noop("Low"),
    i18n_noop("Medium"),
    i18n_noop("High"),
    i18n_noop("Blinding")
};

#define BACKLIGHT_SCALE_GRANULARITY 5
// Normalize the result from light get brightness as it sometimes
// will round down/up by a %
static uint8_t prv_get_scaled_brightness(void) {
  return BACKLIGHT_SCALE_GRANULARITY
         * ((backlight_get_intensity() + BACKLIGHT_SCALE_GRANULARITY - 1)
            / BACKLIGHT_SCALE_GRANULARITY);
}

static int prv_intensity_get_selection_index() {
  const uint8_t intensity = prv_get_scaled_brightness();

  // FIXME: PBL-22272 ... We will return idx 0 if someone has an old value for
  // one of the intensity options
  for (int i = 0; i < (int)ARRAY_LENGTH(s_intensity_values); i++) {
    if (s_intensity_values[i] == intensity) {
      return i;
    }
  }
  return 0;
}

static void prv_intensity_menu_select(OptionMenu *option_menu, int selection, void *context) {
  backlight_set_intensity(s_intensity_values[selection]);
  app_window_stack_remove(&option_menu->window, true /*animated*/);
}

static void prv_intensity_menu_push(SettingsBacklightData *data) {
  const int index = prv_intensity_get_selection_index();
  const OptionMenuCallbacks callbacks = {
    .select = prv_intensity_menu_select,
  };
  const char *title = PBL_IF_RECT_ELSE(i18n_noop("INTENSITY"), i18n_noop("Intensity"));
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks, ARRAY_LENGTH(s_intensity_labels),
      true /* icons_enabled */, s_intensity_labels, data);
}

#ifdef CONFIG_ORIENTATION_MANAGER
// Orientation Settings
/////////////////////////////
static const char *s_display_orientation_labels[] = {
    i18n_ctx_noop("Orientation", "Default"),
    i18n_noop("Left-Handed"),
};

static int prv_display_orientation_get_selection_index() {
  return display_orientation_is_left() ? 1 : 0;
}

static void prv_display_orientation_menu_select(OptionMenu *option_menu, int selection,
                                                void *context) {
  if (prv_display_orientation_get_selection_index() == selection) {
    // No change
    app_window_stack_remove(&option_menu->window, true /* animated */);
    return;
  }

  display_orientation_set_left(!display_orientation_is_left());
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_display_orientation_menu_push(SettingsDisplayData *data) {
  const int index = prv_display_orientation_get_selection_index();
  const OptionMenuCallbacks callbacks = {
      .select = prv_display_orientation_menu_select,
  };

  const char *title = i18n_noop("Orientation");
  settings_option_menu_push(title, OptionMenuContentType_SingleLine, index, &callbacks,
                            ARRAY_LENGTH(s_display_orientation_labels), true,
                            s_display_orientation_labels, data);
}
#endif

// Timeout Settings
/////////////////////////////

static const uint32_t s_timeout_values[] = { 3000, 5000, 8000 };

static const char *s_timeout_labels[] = {
  i18n_noop("3 Seconds"),
  i18n_noop("5 Seconds"),
  i18n_noop("8 Seconds")
};

static int prv_timeout_get_selection_index() {
  uint32_t timeout_ms = backlight_get_timeout_ms();
  for (size_t i = 0; i < ARRAY_LENGTH(s_timeout_values); i++) {
    if (s_timeout_values[i] == timeout_ms) {
      return i;
    }
  }
  return 0;
}

static void prv_timeout_menu_select(OptionMenu *option_menu, int selection, void *context) {
  backlight_set_timeout_ms(s_timeout_values[selection]);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_timeout_menu_push(SettingsBacklightData *data) {
  int index = prv_timeout_get_selection_index();
  const OptionMenuCallbacks callbacks = {
    .select = prv_timeout_menu_select,
  };
  const char *title = PBL_IF_RECT_ELSE(i18n_noop("TIMEOUT"), i18n_noop("Timeout"));
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks, ARRAY_LENGTH(s_timeout_labels),
      true /* icons_enabled */, s_timeout_labels, data);
}

// Touch Wake Settings
/////////////////////////////
#ifdef CONFIG_TOUCH
static const char *s_touch_wake_labels[] = {
    [BacklightTouchWake_DoubleTap] = i18n_noop("Double Tap"),
    [BacklightTouchWake_Tap] = i18n_noop("Tap"),
    [BacklightTouchWake_Off] = i18n_ctx_noop("TouchWake", "Off"),
};

static void prv_touch_wake_menu_select(OptionMenu *option_menu, int selection, void *context) {
  backlight_set_touch_wake((BacklightTouchWake)selection);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_touch_wake_menu_push(SettingsBacklightData *data) {
  const int index = (int)backlight_get_touch_wake();
  const OptionMenuCallbacks callbacks = {
    .select = prv_touch_wake_menu_select,
  };
  const char *title = PBL_IF_RECT_ELSE(i18n_noop("WAKE ON TOUCH"), i18n_noop("Wake on touch"));
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_touch_wake_labels), true /* icons_enabled */, s_touch_wake_labels, data);
}
#endif

// Dynamic Backlight Settings
/////////////////////////////
#ifdef CONFIG_DYNAMIC_BACKLIGHT
static const char *s_dynamic_mode_labels[BacklightDynamicModeCount] = {
    [BacklightDynamicMode_Off] = i18n_ctx_noop("DynBacklight", "Off"),
    [BacklightDynamicMode_Bright] = i18n_noop("Bright"),
    [BacklightDynamicMode_Standard] = i18n_noop("Standard"),
    [BacklightDynamicMode_Dim] = i18n_noop("Dim"),
};

static void prv_dynamic_mode_menu_select(OptionMenu *option_menu, int selection, void *context) {
  light_set_dynamic_mode((BacklightDynamicMode)selection);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_dynamic_mode_menu_push(SettingsBacklightData *data) {
  const int index = (int)backlight_get_dynamic_mode();
  const OptionMenuCallbacks callbacks = {
    .select = prv_dynamic_mode_menu_select,
  };
  const char *title = PBL_IF_RECT_ELSE(i18n_noop("DYNAMIC BACKLIGHT"),
                                       i18n_noop("Dynamic Backlight"));
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_dynamic_mode_labels), true /* icons_enabled */, s_dynamic_mode_labels, data);
}
#endif

// Backlight Preset Settings
/////////////////////////////
static const char *s_backlight_preset_labels[BacklightPresetCount] = {
    [BacklightPreset_MaxBrightness] = i18n_noop("Max Brightness"),
    [BacklightPreset_Standard] = i18n_noop("Standard"),
    [BacklightPreset_BatterySaver] = i18n_noop("Battery Saver"),
    [BacklightPreset_Advanced] = i18n_noop("Advanced"),
};

static void prv_preset_menu_select(OptionMenu *option_menu, int selection, void *context) {
  backlight_set_preset((BacklightPreset)selection);
  if (selection != BacklightPreset_Advanced) {
    // Briefly turn the light on so the user sees the new preset's brightness.
    light_enable_interaction();
  }
  // Selecting (or leaving) Advanced adds/removes the Backlight Settings row.
  settings_menu_reload_data(SettingsMenuItemDisplay);
  settings_menu_mark_dirty(SettingsMenuItemDisplay);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_preset_menu_push(SettingsDisplayData *data) {
  const int index = (int)backlight_get_preset();
  const OptionMenuCallbacks callbacks = {
    .select = prv_preset_menu_select,
  };
  const char *title = PBL_IF_RECT_ELSE(i18n_noop("BACKLIGHT"), i18n_noop("Backlight"));
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_backlight_preset_labels), true /* icons_enabled */,
      s_backlight_preset_labels, data);
}

// Legacy App Mode Settings (Obelix only)
/////////////////////////////
#ifdef CONFIG_APP_SCALING
static const char *s_legacy_app_mode_labels[] = {
    i18n_noop("Centered"),
    i18n_noop("Scaled (Nearest)"),
    i18n_noop("Scaled (Bilinear)")
};

static void prv_legacy_app_mode_menu_select(OptionMenu *option_menu, int selection, void *context) {
  shell_prefs_set_legacy_app_render_mode((LegacyAppRenderMode)selection);
  app_window_stack_remove(&option_menu->window, true /*animated*/);
}

static void prv_legacy_app_mode_menu_push(SettingsDisplayData *data) {
  const int index = (int)shell_prefs_get_legacy_app_render_mode();
  const OptionMenuCallbacks callbacks = {
    .select = prv_legacy_app_mode_menu_select,
  };
  const char *title = i18n_noop("Legacy App Display");
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_legacy_app_mode_labels),
      false /* icons_enabled */, s_legacy_app_mode_labels, data);
}
#endif

// Backlight submenu
//////////////////////////////////////////////////////////////////////////////

enum SettingsBacklightItem {
  SettingsBacklightMode,
  SettingsBacklightAmbientSensor,
#ifdef CONFIG_DYNAMIC_BACKLIGHT
  SettingsBacklightDynamicIntensity,
#endif
  SettingsBacklightIntensity,
  SettingsBacklightMotionWake,
#ifdef CONFIG_TOUCH
  SettingsBacklightTouchWake,
#endif
  SettingsBacklightTimeout,
  NumSettingsBacklightItems
};

static bool prv_backlight_item_is_visible(uint16_t item) {
  switch (item) {
    case SettingsBacklightMode:
    case SettingsBacklightTimeout:
      // Always shown, even when the backlight is off.
      return true;
#ifdef CONFIG_TOUCH
    // The wake-on-touch row is only relevant when global touch is enabled.
    // It gets hidden dynamically (not just gated at compile time) so users
    // don't see a dangling backlight option that can't do anything.
    case SettingsBacklightTouchWake:
      return backlight_is_enabled() && touch_is_globally_enabled();
#endif
    default:
      return backlight_is_enabled();
  }
}

static uint16_t prv_backlight_item_from_row(uint16_t row) {
  uint16_t visible_row = 0;
  for (uint16_t item = 0; item < NumSettingsBacklightItems; item++) {
    if (!prv_backlight_item_is_visible(item)) {
      continue;
    }
    if (visible_row == row) {
      return item;
    }
    visible_row++;
  }
  WTF;
}

static void prv_backlight_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  SettingsBacklightData *data = (SettingsBacklightData*)context;
  switch (prv_backlight_item_from_row(row)) {
    case SettingsBacklightMode:
      light_toggle_enabled();
      break;
    case SettingsBacklightMotionWake:
      backlight_set_motion_enabled(!backlight_is_motion_enabled());
      break;
#ifdef CONFIG_TOUCH
    case SettingsBacklightTouchWake:
      prv_touch_wake_menu_push(data);
      break;
#endif
    case SettingsBacklightAmbientSensor:
      light_toggle_ambient_sensor_enabled();
      break;
#ifdef CONFIG_DYNAMIC_BACKLIGHT
    case SettingsBacklightDynamicIntensity:
      prv_dynamic_mode_menu_push(data);
      break;
#endif
    case SettingsBacklightIntensity:
      prv_intensity_menu_push(data);
      break;
    case SettingsBacklightTimeout:
      prv_timeout_menu_push(data);
      break;
    default:
      WTF;
  }
  settings_menu_reload_data(SettingsMenuItemDisplay);
  settings_menu_mark_dirty(SettingsMenuItemDisplay);
}

static void prv_backlight_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                                      const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsBacklightData *data = (SettingsBacklightData*) context;
  const char *title = NULL;
  const char *subtitle = NULL;
  switch (prv_backlight_item_from_row(row)) {
    case SettingsBacklightMode:
      title = i18n_noop("Backlight");
      if (backlight_is_enabled()) {
#ifdef CONFIG_DYNAMIC_BACKLIGHT
        if (backlight_is_dynamic_intensity_enabled()) {
          uint8_t current_percent = light_get_current_brightness_percent();
          snprintf(data->backlight_percent_buffer, sizeof(data->backlight_percent_buffer),
                   i18n_get("On - %d%%", data), (int)current_percent);
          subtitle = data->backlight_percent_buffer;
        } else {
          subtitle = i18n_ctx_noop("DeviceState", "On");
        }
#else
        subtitle = i18n_ctx_noop("DeviceState", "On");
#endif
      } else {
        subtitle = i18n_ctx_noop("DeviceState", "Off");
      }
      break;
    case SettingsBacklightMotionWake:
      title = i18n_noop("Wake on motion");
      subtitle = backlight_is_motion_enabled() ? i18n_noop("On") : i18n_noop("Off");
      break;
#ifdef CONFIG_TOUCH
    case SettingsBacklightTouchWake:
      title = i18n_noop("Wake on touch");
      subtitle = s_touch_wake_labels[backlight_get_touch_wake()];
      break;
#endif
    case SettingsBacklightAmbientSensor:
      title = i18n_noop("Ambient Sensor");
      if (backlight_is_ambient_sensor_enabled()) {
        uint32_t als_value = light_get_ambient_lux();
        snprintf(data->als_value_buffer, sizeof(data->als_value_buffer),
                 i18n_get("On (%d)", data), (int)als_value);
        subtitle = data->als_value_buffer;
      } else {
        subtitle = i18n_ctx_noop("DeviceState", "Off");
      }
      break;
#ifdef CONFIG_DYNAMIC_BACKLIGHT
    case SettingsBacklightDynamicIntensity:
      title = i18n_noop("Dynamic Backlight");
      subtitle = s_dynamic_mode_labels[backlight_get_dynamic_mode()];
      break;
#endif
    case SettingsBacklightIntensity:
#ifdef CONFIG_DYNAMIC_BACKLIGHT
      title = backlight_is_dynamic_intensity_enabled() ? i18n_noop("Max Intensity")
                                                       : i18n_noop("Intensity");
#else
      title = i18n_noop("Intensity");
#endif
      subtitle = s_intensity_labels[prv_intensity_get_selection_index()];
      break;
    case SettingsBacklightTimeout:
      title = i18n_noop("Timeout");
      subtitle = s_timeout_labels[prv_timeout_get_selection_index()];
      break;
    default:
      WTF;
  }
  menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, data), i18n_get(subtitle, data), NULL);
}

static uint16_t prv_backlight_num_rows_cb(SettingsCallbacks *context) {
  uint16_t rows = 0;
  for (uint16_t item = 0; item < NumSettingsBacklightItems; item++) {
    if (prv_backlight_item_is_visible(item)) {
      rows++;
    }
  }
  return rows;
}

// Timer refresh while the Backlight submenu is visible — lets us live-update
// the ALS reading and the dynamic-backlight percentage subtitle.
#define UPDATE_INTERVAL_MS 500
static void prv_backlight_update_timer_cb(void *context) {
  SettingsBacklightData *data = (SettingsBacklightData*)context;
  settings_menu_mark_dirty(SettingsMenuItemDisplay);
  data->update_timer = app_timer_register(UPDATE_INTERVAL_MS, prv_backlight_update_timer_cb, data);
}

static void prv_backlight_appear_cb(SettingsCallbacks *context) {
  SettingsBacklightData *data = (SettingsBacklightData*)context;
  if (!data->update_timer) {
    data->update_timer = app_timer_register(UPDATE_INTERVAL_MS,
                                            prv_backlight_update_timer_cb, data);
  }
  // Hold the ALS in continuous mode while this submenu is visible so the
  // 500 ms refresh tick doesn't pay a full integration time per read.
  if (!data->als_primed) {
    ambient_light_prime();
    data->als_primed = true;
  }
}

static void prv_backlight_hide_cb(SettingsCallbacks *context) {
  SettingsBacklightData *data = (SettingsBacklightData*)context;
  if (data->update_timer) {
    app_timer_cancel(data->update_timer);
    data->update_timer = NULL;
  }
  if (data->als_primed) {
    ambient_light_release();
    data->als_primed = false;
  }
}

static void prv_backlight_deinit_cb(SettingsCallbacks *context) {
  SettingsBacklightData *data = (SettingsBacklightData*) context;
  if (data->update_timer) {
    app_timer_cancel(data->update_timer);
    data->update_timer = NULL;
  }
  if (data->als_primed) {
    ambient_light_release();
    data->als_primed = false;
  }
  i18n_free_all(data);
  app_free(data);
}

static void prv_backlight_submenu_push(void) {
  SettingsBacklightData *data = app_malloc_check(sizeof(*data));
  *data = (SettingsBacklightData){};

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_backlight_deinit_cb,
    .draw_row = prv_backlight_draw_row_cb,
    .select_click = prv_backlight_select_click_cb,
    .num_rows = prv_backlight_num_rows_cb,
    .appear = prv_backlight_appear_cb,
    .hide = prv_backlight_hide_cb,
  };

  const char *title = i18n_noop("Backlight Settings");
  Window *window = settings_window_create_with_title(SettingsMenuItemDisplay,
                                                     title, &data->callbacks);
  app_window_stack_push(window, true /* animated */);
}

// Display top-level menu
//////////////////////////////////////////////////////////////////////////////

enum SettingsDisplayItem {
  SettingsDisplayBacklight,
  SettingsDisplayBacklightSettings,
#ifdef CONFIG_TOUCH
  SettingsDisplayTouchNav,
  SettingsDisplayTouchNavMenus,
#endif
#ifdef CONFIG_ORIENTATION_MANAGER
  SettingsDisplayOrientation,
#endif
  SettingsDisplayTextSize,
  SettingsDisplayLanguage,
#ifdef CONFIG_APP_SCALING
  SettingsDisplayLegacyAppMode,
#endif
  NumSettingsDisplayItems
};

static bool prv_display_item_is_visible(uint16_t item) {
  // The full backlight submenu only shows for the Advanced backlight mode;
  // the presets cover everything it contains.
  if (item == SettingsDisplayBacklightSettings) {
    return backlight_get_preset() == BacklightPreset_Advanced;
  }
#ifdef CONFIG_TOUCH
  // The Touch Navigation row (native menu gestures) only has an effect while the master Touch
  // switch is on.
  if (item == SettingsDisplayTouchNavMenus) {
    return touch_is_globally_enabled();
  }
#endif
  return true;
}

static uint16_t prv_display_item_from_row(uint16_t row) {
  uint16_t visible_row = 0;
  for (uint16_t item = 0; item < NumSettingsDisplayItems; item++) {
    if (!prv_display_item_is_visible(item)) {
      continue;
    }
    if (visible_row == row) {
      return item;
    }
    visible_row++;
  }
  WTF;
}

static void prv_display_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  switch (prv_display_item_from_row(row)) {
    case SettingsDisplayBacklight:
      prv_preset_menu_push((SettingsDisplayData *)context);
      break;
    case SettingsDisplayBacklightSettings:
      prv_backlight_submenu_push();
      break;
#ifdef CONFIG_TOUCH
    case SettingsDisplayTouchNav:
      // The master "Touch" row IS the global touch kill switch:
      // off = sensor down, events dropped, nothing touch-driven anywhere.
      touch_set_globally_enabled(!touch_is_globally_enabled());
      break;
    case SettingsDisplayTouchNavMenus:
      touch_set_navigation_menu_enabled(!touch_navigation_menu_is_enabled());
      break;
#endif
#ifdef CONFIG_ORIENTATION_MANAGER
    case SettingsDisplayOrientation:
      prv_display_orientation_menu_push((SettingsDisplayData*)context);
      break;
#endif
    case SettingsDisplayTextSize:
      prv_text_size_menu_push((SettingsDisplayData *)context);
      break;
    case SettingsDisplayLanguage:
      prv_language_menu_push((SettingsDisplayData *)context);
      break;
#ifdef CONFIG_APP_SCALING
    case SettingsDisplayLegacyAppMode:
      prv_legacy_app_mode_menu_push((SettingsDisplayData*)context);
      break;
#endif
    default:
      WTF;
  }
  settings_menu_reload_data(SettingsMenuItemDisplay);
  settings_menu_mark_dirty(SettingsMenuItemDisplay);
}

static void prv_display_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                                    const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsDisplayData *data = (SettingsDisplayData*) context;
  const char *title = NULL;
  const char *subtitle = NULL;
  switch (prv_display_item_from_row(row)) {
    case SettingsDisplayBacklight:
      title = i18n_noop("Backlight");
      subtitle = backlight_is_enabled() ? s_backlight_preset_labels[backlight_get_preset()]
                                        : i18n_ctx_noop("DeviceState", "Off");
      break;
    case SettingsDisplayBacklightSettings:
      title = i18n_noop("Backlight Settings");
      break;
#ifdef CONFIG_TOUCH
    case SettingsDisplayTouchNav:
      title = i18n_noop("Touch");
      subtitle = touch_is_globally_enabled() ? i18n_noop("On") : i18n_noop("Off");
      break;
    case SettingsDisplayTouchNavMenus:
      title = i18n_noop("Touch Navigation");
      subtitle = touch_navigation_menu_is_enabled() ? i18n_noop("On") : i18n_noop("Off");
      break;
#endif
#ifdef CONFIG_ORIENTATION_MANAGER
    case SettingsDisplayOrientation:
      title = i18n_noop("Orientation");
      subtitle = s_display_orientation_labels[prv_display_orientation_get_selection_index()];
      break;
#endif
    case SettingsDisplayTextSize: {
      /// String within Settings->Display that describes the text font size
      title = i18n_noop("Text Size");
      const SettingsContentSize index =
          settings_content_size_from_preferred_size(system_theme_get_content_size());
      subtitle = (index < SettingsContentSizeCount) ? s_text_size_names[index] : "";
      break;
    }
    case SettingsDisplayLanguage:
      title = i18n_noop("Language");
      subtitle = i18n_get_lang_name();
      break;
#ifdef CONFIG_APP_SCALING
    case SettingsDisplayLegacyAppMode:
      title = i18n_noop("Legacy Apps");
      subtitle = s_legacy_app_mode_labels[shell_prefs_get_legacy_app_render_mode()];
      break;
#endif
    default:
      WTF;
  }
  menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, data), i18n_get(subtitle, data), NULL);
}

static uint16_t prv_display_num_rows_cb(SettingsCallbacks *context) {
  uint16_t rows = 0;
  for (uint16_t item = 0; item < NumSettingsDisplayItems; item++) {
    if (prv_display_item_is_visible(item)) {
      rows++;
    }
  }
  return rows;
}

static void prv_display_deinit_cb(SettingsCallbacks *context) {
  SettingsDisplayData *data = (SettingsDisplayData*) context;
  i18n_free_all(data);
  app_free(data);
}

static Window *prv_init(void) {
  SettingsDisplayData *data = app_malloc_check(sizeof(*data));
  *data = (SettingsDisplayData){};

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_display_deinit_cb,
    .draw_row = prv_display_draw_row_cb,
    .select_click = prv_display_select_click_cb,
    .num_rows = prv_display_num_rows_cb,
  };

  return settings_window_create(SettingsMenuItemDisplay, &data->callbacks);
}

const SettingsModuleMetadata *settings_display_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    .name = i18n_noop("Display"),
    .init = prv_init,
  };

  return &s_module_info;
}
