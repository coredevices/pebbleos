/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app_worker.h"
#include "kernel/event_loop.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_management/worker_manager.h"
#include "process_management/app_manager.h"
#include "popups/switch_worker_ui.h"
#include "syscall/syscall_internal.h"

// ---------------------------------------------------------------------------------------------------------------
// Determine if the worker for the current app is running
DEFINE_SYSCALL(bool, sys_app_worker_is_running, void) {
  const PebbleProcessMd *md = worker_manager_get_current_worker_md();
  if (md == NULL) {
    return false;
  }
  return (uuid_equal(&md->uuid, &app_manager_get_current_app_md()->uuid));
}


// ---------------------------------------------------------------------------------------------------------------
// Display the confirmation dialog for switching into the worker
static void prv_switch_worker(void *data) {
  AppInstallId install_id = (AppInstallId)data;

  WindowStack *window_stack = modal_manager_get_window_stack(ModalPriorityGeneric);
  switch_worker_confirm(install_id, false /* do not set as default */, window_stack);
}

// Launch the worker for the current app
DEFINE_SYSCALL(AppWorkerResult, sys_app_worker_launch, void) {
  AppInstallId install_id = app_manager_get_task_context()->install_id;

  // Make sure there is a worker for this app
  if (!app_manager_get_task_context()->app_md->has_worker) {
    return APP_WORKER_RESULT_NO_WORKER;
  }

  // Is there a worker already running?
  const PebbleProcessMd *md = worker_manager_get_current_worker_md();
  if (md != NULL) {
    const PebbleProcessMd *app_process = app_manager_get_current_app_md();
    if (uuid_equal(&md->uuid, &app_process->uuid)) {
      return APP_WORKER_RESULT_ALREADY_RUNNING;
    }

    // We have to get confirmation first that it is OK to launch the new worker
    launcher_task_add_callback(prv_switch_worker, (void *) install_id);
    return APP_WORKER_RESULT_ASKING_CONFIRMATION;
  }

  worker_manager_put_launch_worker_event(install_id);
  worker_manager_set_default_install_id(install_id);
  return APP_WORKER_RESULT_SUCCESS;
}


// ---------------------------------------------------------------------------------------------------------------
// Kill the worker for the current app
DEFINE_SYSCALL(AppWorkerResult, sys_app_worker_kill, void) {

  const PebbleProcessMd *md = worker_manager_get_current_worker_md();
  if (md == NULL) {
    return APP_WORKER_RESULT_NOT_RUNNING;
  }
  if (!uuid_equal(&md->uuid, &app_manager_get_current_app_md()->uuid)) {
    return APP_WORKER_RESULT_DIFFERENT_APP;
  }

  process_manager_put_kill_process_event(PebbleTask_Worker, true /*graceful*/);
  return APP_WORKER_RESULT_SUCCESS;
}


// ---------------------------------------------------------------------------------------------------------------
// Launch the app for the current worker
DEFINE_SYSCALL(void, sys_launch_app_for_worker, void) {
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = worker_manager_get_task_context()->install_id,
    .common.reason = APP_LAUNCH_WORKER,
  });
}

