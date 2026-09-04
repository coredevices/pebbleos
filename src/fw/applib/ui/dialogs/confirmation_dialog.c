/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "confirmation_dialog.h"

#include "applib/graphics/gtypes.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/dialogs/actionable_dialog.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource_ids.auto.h"
#include "system/passert.h"

struct ConfirmationDialog {
  ActionableDialog action_dialog;
  ActionBarLayer action_bar;
  GBitmap confirm_icon;
  GBitmap decline_icon;
};

ConfirmationDialog *confirmation_dialog_create(const char *dialog_name) {
  // Note: This isn't a memory leak as the ConfirmationDialog has the action dialog as its
  // first member, so when we call init and pass the ConfirmationDialog as the argument, when
  // it frees the associated data, it actually frees the ConfirmationDialog.
  ConfirmationDialog *confirmation_dialog = task_zalloc_check(sizeof(ConfirmationDialog));
  if (!gbitmap_init_with_resource(&confirmation_dialog->confirm_icon,
                                  RESOURCE_ID_ACTION_BAR_ICON_CHECK)) {
    task_free(confirmation_dialog);
    return NULL;
  }

  if (!gbitmap_init_with_resource(&confirmation_dialog->decline_icon,
                                  RESOURCE_ID_ACTION_BAR_ICON_X)) {
    gbitmap_deinit(&confirmation_dialog->confirm_icon);
    task_free(confirmation_dialog);
    return NULL;
  }

  // In order to create a custom ActionDialog type, we need to create an action bar
  ActionBarLayer *action_bar = &confirmation_dialog->action_bar;
  action_bar_layer_init(action_bar);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_UP, &confirmation_dialog->confirm_icon);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_DOWN, &confirmation_dialog->decline_icon);
  action_bar_layer_set_background_color(action_bar, GColorBlack);
  action_bar_layer_set_context(action_bar, confirmation_dialog);

  // Create the underlying actionable dialog as a custom type
  ActionableDialog *action_dialog = &confirmation_dialog->action_dialog;
  actionable_dialog_init(action_dialog, dialog_name);
  actionable_dialog_set_action_bar_type(action_dialog, DialogActionBarCustom, action_bar);

  return confirmation_dialog;
}

Dialog *confirmation_dialog_get_dialog(ConfirmationDialog *confirmation_dialog) {
  if (confirmation_dialog == NULL) {
    return NULL;
  }
  return actionable_dialog_get_dialog(&confirmation_dialog->action_dialog);
}

ActionBarLayer *confirmation_dialog_get_action_bar(ConfirmationDialog *confirmation_dialog) {
  if (confirmation_dialog == NULL) {
    return NULL;
  }
  return &confirmation_dialog->action_bar;
}

void confirmation_dialog_set_click_config_provider(ConfirmationDialog *confirmation_dialog,
                                                   ClickConfigProvider click_config_provider) {
  if (confirmation_dialog == NULL) {
    return;
  }
  ActionBarLayer *action_bar = &confirmation_dialog->action_bar;
  action_bar_layer_set_click_config_provider(action_bar, click_config_provider);
}

void confirmation_dialog_push(ConfirmationDialog *confirmation_dialog, WindowStack *window_stack) {
  actionable_dialog_push(&confirmation_dialog->action_dialog, window_stack);
}

void app_confirmation_dialog_push(ConfirmationDialog *confirmation_dialog) {
  app_actionable_dialog_push(&confirmation_dialog->action_dialog);
}

void confirmation_dialog_pop(ConfirmationDialog *confirmation_dialog) {
  if (confirmation_dialog == NULL) {
    return;
  }

  action_bar_layer_remove_from_window(&confirmation_dialog->action_bar);
  action_bar_layer_deinit(&confirmation_dialog->action_bar);

  gbitmap_deinit(&confirmation_dialog->confirm_icon);
  gbitmap_deinit(&confirmation_dialog->decline_icon);

  actionable_dialog_pop(&confirmation_dialog->action_dialog);
}
