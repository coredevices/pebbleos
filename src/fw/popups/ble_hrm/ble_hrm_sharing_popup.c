/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/ble_hrm.h"

#include "applib/graphics/gcolor_definitions.h"
#include "applib/ui/dialogs/actionable_dialog.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "kernel/ui/modals/modal_manager.h"
#include "applib/ui/vibes.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"

#include <pbl/util/size.h>

#define BLE_HRM_CONFIRMATION_TIMEOUT_MS (2000)

static void prv_respond(bool is_granted, ActionableDialog *actionable_dialog) {
  BLEHRMSharingRequest *sharing_request = actionable_dialog->dialog.callback_context;
  ble_hrm_handle_sharing_request_response(is_granted, sharing_request);

  actionable_dialog_pop(actionable_dialog);

  if (is_granted) {
    SimpleDialog *simple_dialog = simple_dialog_create("Sharing");
    Dialog *dialog = simple_dialog_get_dialog(simple_dialog);

    const char *msg = i18n_get("Sharing Heart Rate", dialog);
    dialog_set_text(dialog, msg);
    dialog_set_icon(dialog, RESOURCE_ID_BLE_HRM_SHARED);
    dialog_set_timeout(dialog, BLE_HRM_CONFIRMATION_TIMEOUT_MS);
    simple_dialog_set_icon_animated(simple_dialog, false);
    i18n_free(msg, dialog);
    simple_dialog_push(simple_dialog, modal_manager_get_window_stack(ModalPriorityGeneric));
  }
}

static void prv_confirm_cb(ClickRecognizerRef recognizer, void *context) {
  prv_respond(true /* is_granted */, (ActionableDialog *)context);
}

static void prv_back_cb(ClickRecognizerRef recognizer, void *context) {
  prv_respond(false /* is_granted */, (ActionableDialog *)context);
}

static void prv_shutdown_click_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_confirm_cb);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_back_cb);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_back_cb);
}

void ble_hrm_push_sharing_request_window(BLEHRMSharingRequest *sharing_request) {
  ActionableDialog *a_dialog = actionable_dialog_create("HRM Sharing");
  Dialog *dialog = actionable_dialog_get_dialog(a_dialog);
  dialog->callback_context = sharing_request;

  actionable_dialog_set_action_bar_type(a_dialog, DialogActionBarConfirmDecline, NULL);
  actionable_dialog_set_click_config_provider(a_dialog, prv_shutdown_click_provider);

  dialog_set_text_color(dialog, GColorWhite);
  dialog_set_background_color(dialog, GColorCobaltBlue);

  dialog_set_icon(dialog, RESOURCE_ID_BLE_HRM_SHARE_REQUEST_LARGE);

  dialog_set_text(dialog, i18n_get("Share heart rate?", a_dialog));
  i18n_free_all(a_dialog);

  actionable_dialog_push(a_dialog, modal_manager_get_window_stack(ModalPriorityGeneric));

  const uint32_t heart_beat_durations[] = { 100, 100, 150, 600, 100, 100, 150 };
  VibePattern heart_beat_pattern = {
    .durations = heart_beat_durations,
    .num_segments = ARRAY_LENGTH(heart_beat_durations),
  };
  vibes_enqueue_custom_pattern(heart_beat_pattern);
}
