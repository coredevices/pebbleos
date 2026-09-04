/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "health_tracking_ui.h"

#include "applib/ui/dialogs/expandable_dialog.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/modals/modal_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/light.h"

typedef struct HealthTrackingUIData {
  uint32_t res_id;
  const char *text;
  bool show_action_bar;
} HealthTrackingUIData;

static AppInstallId  s_last_app_id;

// ---------------------------------------------------------------------------
static WindowStack *prv_get_window_stack(void) {
  return modal_manager_get_window_stack(ModalPriorityAlert);
}

// ---------------------------------------------------------------------------
static void prv_select_handler(ClickRecognizerRef recognizer, void *context) {
  ExpandableDialog *expandable_dialog = context;
  expandable_dialog_pop(expandable_dialog);
}

// ---------------------------------------------------------------------------
static void prv_push_enable_in_mobile_dialog(void *context) {
  HealthTrackingUIData *data = context;
  ExpandableDialog *expandable_dialog = expandable_dialog_create("Health Disabled");
  Dialog *dialog = expandable_dialog_get_dialog(expandable_dialog);

  // Set the base dialog properties
  dialog_set_text(dialog, i18n_get(data->text, dialog));
  // i18n_free(data->text, dialog);
  dialog_set_icon(dialog, data->res_id);
  dialog_set_vibe(dialog, false);

  // Set the expandable dialog properties
  expandable_dialog_show_action_bar(expandable_dialog, data->show_action_bar);
  if (data->show_action_bar) {
    expandable_dialog_set_select_action(expandable_dialog, RESOURCE_ID_ACTION_BAR_ICON_CHECK,
                                        prv_select_handler);
  }

  expandable_dialog_push(expandable_dialog, prv_get_window_stack());

  light_enable_interaction();

  kernel_free(data);
}

// ---------------------------------------------------------------------------
void health_tracking_ui_show_message(uint32_t res_id, const char *text, bool show_action_bar) {
  HealthTrackingUIData *data = kernel_malloc(sizeof(HealthTrackingUIData));
  *data = (HealthTrackingUIData){
    .res_id = res_id,
    .text = text,
    .show_action_bar = show_action_bar,
  };

  launcher_task_add_callback(prv_push_enable_in_mobile_dialog, data);
}

// ---------------------------------------------------------------------------
void health_tracking_ui_feature_show_disabled(void) {
  /// Feature requires health dialog
  static const char *msg =
      i18n_noop("This feature requires Pebble Health to work. Enable Health in the Pebble"
                " mobile app to continue.");

  health_tracking_ui_show_message(RESOURCE_ID_GENERIC_WARNING_TINY, msg, false);
}

// ---------------------------------------------------------------------------
void health_tracking_ui_register_app_launch(AppInstallId app_id) {
  s_last_app_id = 0;
}
