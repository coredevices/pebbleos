/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! This file is responsible for displaying the initial Quick Launch setup menu.
//! If a user long presses up or down from a watchface and has previously not
//! set up an application to launch for that long press direction or has not disabled
//! the Quick Launch feature, then this will act as a mini-setup guide for the feature.
//! Once an application is set up to launch for that menu press direction, this should
//! never appear again.

#include "quick_launch_setup_menu.h"

#include "applib/app.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "system/passert.h"

typedef enum QuickLaunchSetupVersion {
  //! Initial version or never opened
  QuickLaunchSetupVersion_InitialVersion = 0,
  //! 4.0 UX with Toggle Apps
  QuickLaunchSetupVersion_UX4WithToggleApps = 1,

  QuickLaunchSetupVersionCount,
  //! QuickLaunchSetupVersion is an increasing version number. QuickLaunchSetupVersionCurrent must
  //! not decrement. This should ensure that the current version is always the latest.
  QuickLaunchSetupVersionCurrent = QuickLaunchSetupVersionCount - 1,
} QuickLaunchSetupVersion;

static void prv_push_settings_menu(void) {
  settings_menu_push(SettingsMenuItemQuickLaunch);
}

static void prv_handle_quick_launch_confirm(ClickRecognizerRef recognizer, void *context) {
  PBL_ASSERTN(context);
  expandable_dialog_pop(context);
  prv_push_settings_menu();
}

static void prv_push_first_use_dialog(void) {
  const void *i18n_owner = prv_push_first_use_dialog; // Use this function as the i18n owner
  /// Title for the Quick Launch first use dialog.
  const char *header = i18n_get("Quick Launch", i18n_owner);
  /// Help text for the Quick Launch first use dialog.
  const char *text = i18n_get("Open favorite apps quickly with a long button press from your "
                              "watchface.", i18n_owner);
  ExpandableDialog *expandable_dialog = expandable_dialog_create_with_params(
      WINDOW_NAME("Quick Launch First Use"), RESOURCE_ID_SUNNY_DAY_TINY, text, GColorBlack,
      GColorWhite, NULL, RESOURCE_ID_ACTION_BAR_ICON_CHECK, prv_handle_quick_launch_confirm);
  expandable_dialog_set_header(expandable_dialog, header);
#if PBL_ROUND
  expandable_dialog_set_header_font(expandable_dialog,
                                    fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
#endif
  i18n_free_all(i18n_owner);
  app_expandable_dialog_push(expandable_dialog);
}

static void prv_init(void) {
  const uint32_t version = quick_launch_get_quick_launch_setup_opened();
  quick_launch_set_quick_launch_setup_opened(QuickLaunchSetupVersionCurrent);
  if (version == QuickLaunchSetupVersion_InitialVersion) {
    prv_push_first_use_dialog();
  } else {
    prv_push_settings_menu();
  }
}

static void prv_main(void) {
  prv_init();
  app_event_loop();
}

const PebbleProcessMd* quick_launch_setup_get_app_info(void) {
  static const PebbleProcessMdSystem s_quick_launch_setup_app = {
    .common = {
      .visibility = ProcessVisibilityHidden,
      .main_func = prv_main,
      // UUID: 07e0d9cb-8957-4bf7-9d42-aaaaaaaaaaaa
      .uuid = {0x07, 0xe0, 0xd9, 0xcb, 0x89, 0x57, 0x4b, 0xf7,
               0x9d, 0x42, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa},
    },
    .name = "Quick Launch",
  };
  return (const PebbleProcessMd*) &s_quick_launch_setup_app;
}
