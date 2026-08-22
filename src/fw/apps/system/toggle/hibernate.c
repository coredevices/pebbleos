/* SPDX-FileCopyrightText: 2025 The PebbleOS Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "hibernate.h"

#include "applib/app.h"
#include "applib/ui/dialogs/actionable_dialog.h"
#include "applib/ui/dialogs/dialog.h"
#include "kernel/event_loop.h"

#ifdef CONFIG_HIBERNATE

#include "process_management/app_manager.h"
#include "pbl/services/i18n/i18n.h"
#include "resource/resource_ids.auto.h"
#include "shell/normal/battery_ui.h"

static void prv_do_shutdown(void *data) {
  battery_ui_handle_mcu_shutdown();
}

static void prv_shutdown_confirm_cb(ClickRecognizerRef recognizer, void *context) {
  actionable_dialog_pop((ActionableDialog *)context);
  launcher_task_add_callback(prv_do_shutdown, NULL);
}

static void prv_shutdown_back_cb(ClickRecognizerRef recognizer, void *context) {
  actionable_dialog_pop((ActionableDialog *)context);
}

static void prv_shutdown_click_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_shutdown_confirm_cb);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_shutdown_back_cb);
}

static void prv_main(void) {
  ActionableDialog *a_dialog = actionable_dialog_create("Hibernate");
  Dialog *dialog = actionable_dialog_get_dialog(a_dialog);

  actionable_dialog_set_action_bar_type(a_dialog, DialogActionBarConfirm, NULL);
  actionable_dialog_set_click_config_provider(a_dialog, prv_shutdown_click_provider);

  dialog_set_text_color(dialog, GColorWhite);
  dialog_set_background_color(dialog, GColorCobaltBlue);
  /// Confirmation message shown in the Hibernate Quick Launch dialog.
  dialog_set_text(dialog, i18n_get("Do you want to hibernate?", a_dialog));
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_QUESTION_LARGE);

  i18n_free_all(a_dialog);

  app_actionable_dialog_push(a_dialog);
  app_event_loop();
}

const PebbleProcessMd *hibernate_toggle_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &prv_main,
      .uuid = HIBERNATE_TOGGLE_UUID,
      .visibility = ProcessVisibilityQuickLaunch,
    },
    /// Name of the Hibernate Quick Launch option shown in Quick Launch settings.
    .name = i18n_noop("Hibernate"),
  };
  return &s_app_info.common;
}
#endif
