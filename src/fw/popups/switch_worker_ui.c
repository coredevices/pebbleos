/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "switch_worker_ui.h"

#include <stdio.h>

#include "applib/ui/action_bar_layer.h"
#include "applib/ui/dialogs/confirmation_dialog.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "process_management/process_manager.h"
#include "process_management/worker_manager.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/app_cache.h"

typedef struct {
  AppInstallId new_worker_id;
  bool set_as_default;
  ConfirmationDialog *confirmation_dialog;
} SwitchWorkerUIArgs;

static bool s_is_on_screen = false;


static void prv_click_confirm_decline_callback(ClickRecognizerRef recognizer, void *context) {
  // TODO: Currently set_as_default does nothing.  This will be corrected later on to allow
  // launching of a worker while an app is open, then returning to the default worker after
  // the application has been exited.  The likely UI flow would prompt the user to set the
  // worker as the default (if the flag is false) after they've confirmed enabling activity
  // tracking using the launch application, to which they can decline.
  SwitchWorkerUIArgs *args = (SwitchWorkerUIArgs *) context;
  ConfirmationDialog *confirmation_dialog = args->confirmation_dialog;

  confirmation_dialog_pop(confirmation_dialog);

  bool selection_confirmed = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_UP);
  if (selection_confirmed) {
    if (!app_cache_entry_exists(args->new_worker_id)) {
      // If an app cache entry does not exist for the new worker, then we will have to
      // fetch the application.  Since this will prompt the user to confirm activity tracking
      // for the worker because the previous worker is still running, we have to kill the
      // previous worker here.
      process_manager_put_kill_process_event(PebbleTask_Worker, true /* graceful */);
    }
    worker_manager_set_default_install_id(args->new_worker_id);
    worker_manager_put_launch_worker_event(args->new_worker_id);
  }

  s_is_on_screen = false;
  task_free(args);
}

static void prv_click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_click_confirm_decline_callback);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_click_confirm_decline_callback);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_click_confirm_decline_callback);
}

void switch_worker_confirm(AppInstallId new_worker_id, bool set_as_default,
                           WindowStack *window_stack) {
  AppInstallId cur_worker_id = worker_manager_get_current_worker_id();

  if (s_is_on_screen == true) {
    // If we already have a window up, let that one finish.  This prevents apps that
    // spam worker launch from displaying multiple confirmation dialogs on-top of
    // one another.
    return;
  }

  if (cur_worker_id == INSTALL_ID_INVALID) {
    // If there is no worker running, we can simply launch the new one without confirming
    worker_manager_put_launch_worker_event(new_worker_id);
    return;
  } else if (cur_worker_id == new_worker_id) {
    // Or if the new one is already running, then there is nothing to do
    return;
  }

  AppInstallEntry old_entry;
  if (!app_install_get_entry_for_install_id(cur_worker_id, &old_entry)) {
    return;
  }

  AppInstallEntry new_entry;
  if (!app_install_get_entry_for_install_id(new_worker_id, &new_entry)) {
    return;
  }

  s_is_on_screen = true;

  ConfirmationDialog *confirmation_dialog = confirmation_dialog_create("Background App");
  Dialog *dialog = confirmation_dialog_get_dialog(confirmation_dialog);
  const char *fmt = i18n_get("Run %s instead of %s as the background app?", confirmation_dialog);
  char *msg_buffer = task_zalloc_check(DIALOG_MAX_MESSAGE_LEN);
  sniprintf(msg_buffer, DIALOG_MAX_MESSAGE_LEN, fmt, new_entry.name, old_entry.name);

  confirmation_dialog_set_click_config_provider(confirmation_dialog, prv_click_config_provider);

  dialog_set_background_color(dialog, GColorCobaltBlue);
  dialog_set_text_color(dialog, GColorWhite);
  dialog_set_text(dialog, msg_buffer);

  task_free(msg_buffer);
  i18n_free_all(confirmation_dialog);

  SwitchWorkerUIArgs *args = task_malloc_check(sizeof(SwitchWorkerUIArgs));
  *args = (SwitchWorkerUIArgs) {
      .new_worker_id = new_worker_id,
      .set_as_default = set_as_default,
      .confirmation_dialog = confirmation_dialog,
  };

  // Set our arguments to be passed as the context to the confirmation dialog action bar
  ActionBarLayer *action_bar = confirmation_dialog_get_action_bar(confirmation_dialog);
  action_bar_layer_set_context(action_bar, args);

  confirmation_dialog_push(confirmation_dialog, window_stack);
}
