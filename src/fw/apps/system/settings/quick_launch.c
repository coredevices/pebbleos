/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! This file displays the main Quick Launch menu that is found in our settings menu
//! It allows the feature to be enabled or for an action or app to be set
//! The list of targets that the user can choose from is found in quick_launch_app_menu.c
//! This file is also responsible for saving / storing the uuid of each quichlaunch app as well as
//! whether or not the quicklaunch app is enabled.

#include "menu.h"
#include "option_menu.h"
#include "quick_launch.h"
#include "quick_launch_app_menu.h"
#include "quick_launch_setup_menu.h"
#include "window.h"

#include "applib/app.h"
#include "applib/app_launch_button.h"
#include "applib/app_launch_reason.h"
#include "applib/ui/window_stack.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_menu_data_source.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "shell/normal/quick_launch.h"
#include "system/passert.h"
#include "system/status_codes.h"

#define NUM_ROWS (NUM_BUTTONS + 2)  // 4 hold buttons + 2 tap buttons (up and down)

typedef enum {
  ROW_TAP_UP = 0,
  ROW_TAP_DOWN,
  ROW_HOLD_UP,
  ROW_HOLD_SELECT,
  ROW_HOLD_DOWN,
  ROW_HOLD_BACK,
} QuickLaunchRow;

typedef enum {
  CATEGORY_ROW_ACTIONS = 0,
  CATEGORY_ROW_APPS,

  CategoryRowCount,
} QuickLaunchCategoryRow;

typedef struct QuickLaunchData {
  SettingsCallbacks callbacks;
  char app_names[NUM_ROWS][APP_NAME_SIZE_BYTES];
  //! The button whose binding the pushed sub-menus are editing.
  ButtonId button;
  bool is_tap;
} QuickLaunchData;

static const char *s_row_titles[NUM_ROWS] = {
  /// Shown in Quick Launch Settings as the title of the tap up button option.
  [ROW_TAP_UP]       = i18n_noop("Tap Up"),
  /// Shown in Quick Launch Settings as the title of the tap down button option.
  [ROW_TAP_DOWN]     = i18n_noop("Tap Down"),
  /// Shown in Quick Launch Settings as the title of the hold up button quick launch option.
  [ROW_HOLD_UP]      = i18n_noop("Hold Up"),
  /// Shown in Quick Launch Settings as the title of the hold center button quick launch option.
  [ROW_HOLD_SELECT]  = i18n_noop("Hold Center"),
  /// Shown in Quick Launch Settings as the title of the hold down button quick launch option.
  [ROW_HOLD_DOWN]    = i18n_noop("Hold Down"),
  /// Shown in Quick Launch Settings as the title of the hold back button quick launch option.
  [ROW_HOLD_BACK]    = i18n_noop("Hold Back"),
};

static const char *s_category_row_titles[CategoryRowCount] = {
  /// Shown in Quick Launch Settings as the title of the list of actions.
  [CATEGORY_ROW_ACTIONS] = i18n_noop("Actions"),
  /// Shown in Quick Launch Settings as the title of the list of apps.
  [CATEGORY_ROW_APPS]    = i18n_noop("Apps"),
};

static void prv_get_subtitle_string(AppInstallId app_id, QuickLaunchData *data,
                                    char *buffer, uint8_t buf_len) {
  if (app_id == INSTALL_ID_INVALID) {
    /// Shown in Quick Launch Settings when no action or app is bound to the button.
    i18n_get_with_buffer("Unassigned", buffer, buf_len);
    return;
  } else {
    AppInstallEntry entry;
    if (app_install_get_entry_for_install_id(app_id, &entry)) {
      strncpy(buffer, entry.name, buf_len);
      buffer[buf_len - 1] = '\0';
      return;
    }
  }
  // if failed both, set as empty string
  buffer[0] = '\0';
}

// Filter List Callbacks
////////////////////////
static void prv_deinit_cb(SettingsCallbacks *context) {
  QuickLaunchData *data = (QuickLaunchData *) context;
  i18n_free_all(data);
  app_free(data);
}

static void prv_update_app_names(QuickLaunchData *data) {
  // Tap buttons
  prv_get_subtitle_string(quick_launch_single_click_get_app(BUTTON_ID_UP), data,
                          data->app_names[ROW_TAP_UP], APP_NAME_SIZE_BYTES);
  prv_get_subtitle_string(quick_launch_single_click_get_app(BUTTON_ID_DOWN), data,
                          data->app_names[ROW_TAP_DOWN], APP_NAME_SIZE_BYTES);
  
  // Hold buttons
  prv_get_subtitle_string(quick_launch_get_app(BUTTON_ID_UP), data,
                          data->app_names[ROW_HOLD_UP], APP_NAME_SIZE_BYTES);
  prv_get_subtitle_string(quick_launch_get_app(BUTTON_ID_SELECT), data,
                          data->app_names[ROW_HOLD_SELECT], APP_NAME_SIZE_BYTES);
  prv_get_subtitle_string(quick_launch_get_app(BUTTON_ID_DOWN), data,
                          data->app_names[ROW_HOLD_DOWN], APP_NAME_SIZE_BYTES);
  prv_get_subtitle_string(quick_launch_get_app(BUTTON_ID_BACK), data,
                          data->app_names[ROW_HOLD_BACK], APP_NAME_SIZE_BYTES);
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                            const Layer *cell_layer, uint16_t row, bool selected) {
  QuickLaunchData *data = (QuickLaunchData *)context;
  PBL_ASSERTN(row < NUM_ROWS);
  const char *title = i18n_get(s_row_titles[row], data);
  char *subtitle_buf = data->app_names[row];
  menu_cell_basic_draw(ctx, cell_layer, title, subtitle_buf, NULL);
}

static uint16_t prv_get_initial_selection_cb(SettingsCallbacks *context) {
  // If launched by quick launch, select the row of the button pressed, otherwise default to 0
  if (app_launch_reason() == APP_LAUNCH_QUICK_LAUNCH) {
    ButtonId button = app_launch_button();
    // Map button to hold row (quick launch is always hold/long press)
    switch (button) {
      case BUTTON_ID_UP:     return ROW_HOLD_UP;
      case BUTTON_ID_SELECT: return ROW_HOLD_SELECT;
      case BUTTON_ID_DOWN:   return ROW_HOLD_DOWN;
      case BUTTON_ID_BACK:   return ROW_HOLD_BACK;
      default: break;
    }
  }
  return 0;
}

//! The row to start on, which is the category the button is currently bound to.
static QuickLaunchCategoryRow prv_get_category_row(QuickLaunchData *data) {
  const AppInstallId app_id = data->is_tap ? quick_launch_single_click_get_app(data->button)
                                           : quick_launch_get_app(data->button);
  AppInstallEntry entry;
  if ((app_id == INSTALL_ID_INVALID) || !app_install_get_entry_for_install_id(app_id, &entry)) {
    return CATEGORY_ROW_ACTIONS;
  }
  return app_install_entry_is_quick_launch_visible_only(&entry) ? CATEGORY_ROW_ACTIONS
                                                               : CATEGORY_ROW_APPS;
}

static void prv_category_menu_select(OptionMenu *option_menu, int selection, void *context) {
  QuickLaunchData *data = settings_option_menu_get_context(context);
  const QuickLaunchMenuCategory category = (selection == CATEGORY_ROW_ACTIONS) ?
      QuickLaunchMenuCategoryActions : QuickLaunchMenuCategoryApps;
  quick_launch_app_menu_window_push(data->button, data->is_tap, category, &option_menu->window);
}

static void prv_category_menu_push(QuickLaunchData *data) {
  const OptionMenuCallbacks callbacks = {
    .select = prv_category_menu_select,
  };
  settings_option_menu_push(i18n_noop("Quick Launch"), OptionMenuContentType_SingleLine,
                            prv_get_category_row(data), &callbacks, CategoryRowCount,
                            false /* icons_enabled */, s_category_row_titles, data);
}

static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  PBL_ASSERTN(row < NUM_ROWS);

  ButtonId button;
  bool is_tap;

  switch (row) {
    case ROW_TAP_UP:
      button = BUTTON_ID_UP;
      is_tap = true;
      break;
    case ROW_TAP_DOWN:
      button = BUTTON_ID_DOWN;
      is_tap = true;
      break;
    case ROW_HOLD_UP:
      button = BUTTON_ID_UP;
      is_tap = false;
      break;
    case ROW_HOLD_SELECT:
      button = BUTTON_ID_SELECT;
      is_tap = false;
      break;
    case ROW_HOLD_DOWN:
      button = BUTTON_ID_DOWN;
      is_tap = false;
      break;
    case ROW_HOLD_BACK:
      button = BUTTON_ID_BACK;
      is_tap = false;
      break;
    default:
      return;
  }

  QuickLaunchData *data = (QuickLaunchData *)context;
  data->button = button;
  data->is_tap = is_tap;
  prv_category_menu_push(data);
}

static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
  return NUM_ROWS;
}

static void prv_appear(SettingsCallbacks *context) {
  QuickLaunchData *data = (QuickLaunchData *)context;
  prv_update_app_names(data);
}

static Window *prv_init(void) {
  QuickLaunchData *data = app_malloc_check(sizeof(*data));
  *data = (QuickLaunchData){};

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_deinit_cb,
    .draw_row = prv_draw_row_cb,
    .get_initial_selection = prv_get_initial_selection_cb,
    .select_click = prv_select_click_cb,
    .num_rows = prv_num_rows_cb,
    .appear = prv_appear,
  };

  return settings_window_create(SettingsMenuItemQuickLaunch, &data->callbacks);
}

const SettingsModuleMetadata *settings_quick_launch_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    /// Title of the Quick Launch Settings submenu in Settings
    .name = i18n_noop("Quick Launch"),
    .init = prv_init,
  };

  return &s_module_info;
}
