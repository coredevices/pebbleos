/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bt_conn_dialog.h"

#include "applib/applib_malloc.auto.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/dialog_private.h"
#include "applib/ui/window.h"
#include "kernel/events.h"
#include "kernel/pebble_tasks.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "syscall/syscall.h"
#include "system/passert.h"

static void prv_handle_comm_session_event(PebbleEvent *e, void *context) {
  BtConnDialog *bt_dialog = context;
  if (!e->bluetooth.comm_session_event.is_system) {
    return;
  }

  if (e->bluetooth.comm_session_event.is_open) {
    if (bt_dialog->connected_handler) {
      bt_dialog->connected_handler(true, bt_dialog->context);
    }
    // handler to NULL so it won't be called again during the unload
    bt_dialog->connected_handler = NULL;
    dialog_pop(&bt_dialog->dialog.dialog);
  }
}

static void prv_bt_dialog_unload(void *context) {
  BtConnDialog *bt_dialog = context;
  event_service_client_unsubscribe(&bt_dialog->pebble_app_event_sub);
  if (bt_dialog->connected_handler) {
    bt_dialog->connected_handler(false, bt_dialog->context);
  }
  if (bt_dialog->owns_buffer) {
    applib_free(bt_dialog->text_buffer);
  }
}

void bt_conn_dialog_push(BtConnDialog *bt_dialog, BtConnDialogResultHandler handler,
                         void *context) {
  if (!bt_dialog) {
    bt_dialog = bt_conn_dialog_create();
    if (!bt_dialog) {
      return;
    }
  }
  bt_dialog->connected_handler = handler;
  bt_dialog->context = context;

  bt_dialog->pebble_app_event_sub = (EventServiceInfo) {
    .type = PEBBLE_COMM_SESSION_EVENT,
    .handler = prv_handle_comm_session_event,
    .context = bt_dialog
  };
  event_service_client_subscribe(&bt_dialog->pebble_app_event_sub);

  WindowStack *window_stack = NULL;
  if (pebble_task_get_current() == PebbleTask_App) {
    window_stack = app_state_get_window_stack();
  } else {
    // Bluetooth disconnection events are always displayed at maximum priority.
    window_stack = modal_manager_get_window_stack(ModalPriorityCritical);
  }
  simple_dialog_push(&bt_dialog->dialog, window_stack);
}

BtConnDialog *bt_conn_dialog_create(void) {
  BtConnDialog *bt_dialog = applib_malloc(sizeof(BtConnDialog));
  bt_conn_dialog_init(bt_dialog, NULL, 0);
  return bt_dialog;
}

void bt_conn_dialog_init(BtConnDialog *bt_dialog, char *text_buffer, size_t buffer_size) {
  memset(bt_dialog, 0, sizeof(BtConnDialog));

  simple_dialog_init(&bt_dialog->dialog, "Bluetooth Disconnected");
  Dialog *dialog = &bt_dialog->dialog.dialog;

  size_t len = sys_i18n_get_length("Check bluetooth connection");
  if (text_buffer) {
    PBL_ASSERTN(len < buffer_size);
    bt_dialog->text_buffer = text_buffer;
    bt_dialog->owns_buffer = false;
  } else {
    buffer_size = len + 1;
    bt_dialog->text_buffer = applib_malloc(buffer_size);
    bt_dialog->owns_buffer = true;
  }

  sys_i18n_get_with_buffer("Check bluetooth connection", bt_dialog->text_buffer, buffer_size);
  dialog_set_text(dialog, bt_dialog->text_buffer);
  dialog_set_icon(dialog, RESOURCE_ID_WATCH_DISCONNECTED_LARGE);
  dialog_show_status_bar_layer(dialog, true);
  dialog_set_callbacks(dialog, &(DialogCallbacks) {
    .unload = prv_bt_dialog_unload
  }, bt_dialog);
}
