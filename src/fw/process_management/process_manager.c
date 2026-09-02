/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "process_manager.h"
#include "app_install_manager.h"
#include "app_manager.h"
#include "worker_manager.h"

#include "applib/app_logging.h"
#include "applib/accel_service_private.h"
#include "applib/platform.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"

#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"

#include "pebble_process_md.h"
#include "kernel/pebble_tasks.h"
#include "pbl/kernel/types.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/animation_service.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/event_service.h"
#include "pbl/services/hrm/hrm_manager.h"
#include "pbl/services/app_cache.h"
#include "pbl/services/blob_db/app_db.h"
#include "pbl/services/data_logging/data_logging_service.h"
#include "pbl/services/persist.h"
#include "pbl/services/voice/voice.h"
#include "shell/normal/watchface.h"

#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include "kernel/pbl_malloc.h"
#include "kernel/ui/modals/modal_manager.h"
#include "pbl/util/heap.h"

#include "syscall/syscall_internal.h"

#include "apps/system/app_fetch_ui.h"

#include "pbl/kernel/debug.h"

static TimerID s_deinit_timer_id = TIMER_INVALID_ID;

// -------------------------------------------------------------------------------------------
static ProcessContext *prv_get_context_for_task(PebbleTask task) {
  if (task == PebbleTask_App) {
    return app_manager_get_task_context();
  } else {
    PBL_ASSERTN(task == PebbleTask_Worker);
    return worker_manager_get_task_context();
  }
}

// -------------------------------------------------------------------------------------------
static ProcessContext *prv_get_context(void) {
  return prv_get_context_for_task(pebble_task_get_current());
}

// --------------------------------------------------------------------------------------------------
// This timer callback gets called if the process doesn't finish it's deinit within the required timeout (currently
// 3 seconds).
static void prv_graceful_close_timer_callback(void* data) {
  PBL_LOG_DBG("deinit timeout expired, killing app forcefully");
  PebbleTask task = (PebbleTask)data;

  process_manager_put_kill_process_event(task, false /*gracefully*/);
}

// ---------------------------------------------------------------------------------------------
static bool prv_force_stop_task_if_unprivileged(ProcessContext *context) {
  pbl_thread_suspend(context->task_handle);

  struct pbl_thread_saved_regs regs;
  pbl_thread_saved_regs(context->task_handle, &regs);
  if ((regs.control & 0x1) == 0) {
    // We're priviledged, it's not safe to just kill the app task.
    pbl_thread_resume(context->task_handle);
    return false;
  }

  context->safe_to_kill = true;
  return true;
}

// --------------------------------------------------------------------------------------------------
static void prv_force_close_timer_callback(void* data) {
  PebbleTask task = (PebbleTask)data;
  ProcessContext *context = prv_get_context_for_task(task);

  if (!prv_force_stop_task_if_unprivileged(context)) {
    PBL_CROAK("task stuck inside privileged code!");
  }
  process_manager_put_kill_process_event(task, false /*graceful*/);
}

// ---------------------------------------------------------------------------------------------
EXTERNALLY_VISIBLE void process_manager_handle_syscall_exit(void) {
  PebbleTask task = pebble_task_get_current();
  ProcessContext *context = prv_get_context_for_task(task);

  if (context->closing_state == ProcessRunState_ForceClosing) {
    PBL_LOG_DBG("Hit syscall exit trap!");
    context->safe_to_kill = true;
    process_manager_put_kill_process_event(task, false);

    pbl_thread_suspend(NULL);
  }
}

// ---------------------------------------------------------------------------------------------
void process_manager_init(void) {
  s_deinit_timer_id = new_timer_create();
}

// -----------------------------------------------------------------------------------------------------------
void process_manager_put_kill_process_event(PebbleTask task, bool gracefully) {
  PebbleEvent event = {
    .type = PEBBLE_PROCESS_KILL_EVENT,
    .kill = {
      .gracefully = gracefully,
      .task = task,
    },
  };

  // When we have decided to exit the app,
  // it doesn't need to process any queued accel data
  // or other services before exiting, so clear the to_process_event_queue
  ProcessContext *context = prv_get_context_for_task(task);
  if (context->to_process_event_queue) {
    event_queue_cleanup_and_reset(context->to_process_event_queue);
  }

  // Since the app is about to exit, make sure the next (only)
  // message on the from app queue is the PEBBLE_APP_KILL_EVENT
  // to expedite exiting the application
  event_reset_from_process_queue(task);

  event_put_from_process(task, &event);
}

// ---------------------------------------------------------------------------------------------
//! Init the context variables for a task.
void process_manager_init_context(ProcessContext* context,
                                  const PebbleProcessMd *app_md, const void *args) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  PBL_ASSERTN(context->task_handle == NULL);
  PBL_ASSERTN(context->to_process_event_queue == NULL);

  context->app_md = app_md;

  AppInstallId install_id = app_install_get_id_for_uuid(&app_md->uuid);
  context->install_id = install_id;

  // we are safe to kill until the app main starts
  context->safe_to_kill = true;
  context->closing_state = ProcessRunState_Running;
  context->args = args;
  context->user_data = 0;

  // get app launch reason and wakeup_info
  context->launch_reason = app_manager_get_launch_reason();
  context->launch_button = app_manager_get_launch_button();
  context->wakeup_info = app_manager_get_app_wakeup_state();

  // set the default exit reason
  context->exit_reason = APP_EXIT_NOT_SPECIFIED;
}

#if !defined(CONFIG_RECOVERY_FW)
bool process_manager_check_SDK_compatible(const AppInstallId id) {
  AppInstallEntry entry;
  if (!app_install_get_entry_for_install_id(id, &entry)) {
    return false;
  }

  if (app_install_entry_is_SDK_compatible(&entry)) {
    return true;
  }

  PBL_LOG_WRN("App requires support for SDK version (%"PRIu8".%"PRIu8"), "
                             "we only support version (%"PRIu8".%"PRIu8").",
          entry.sdk_version.major, entry.sdk_version.minor,
          (uint8_t) PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR,
          (uint8_t) PROCESS_INFO_CURRENT_SDK_VERSION_MINOR);

  ExpandableDialog *expandable_dialog = expandable_dialog_create("Incompatible SDK");
  Dialog *dialog = expandable_dialog_get_dialog(expandable_dialog);
  const char *error_text = i18n_noop("This app requires a newer version of the Pebble firmware.");
  dialog_set_text(dialog, i18n_get(error_text, expandable_dialog));
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_WARNING_SMALL);
  i18n_free(error_text, expandable_dialog);

  if (pebble_task_get_current() == PebbleTask_KernelMain) {
    WindowStack *window_stack = modal_manager_get_window_stack(ModalPriorityAlert);
    expandable_dialog_push(expandable_dialog, window_stack);
  } else {
    app_expandable_dialog_push(expandable_dialog);
  }

  return false;
}

static bool prv_needs_fetch(AppInstallId id, const PebbleProcessMd **md, bool is_worker) {
  PBL_ASSERTN(md);

  if (!app_cache_entry_exists(id)) {
    PBL_LOG_DBG("Cache entry did not exist on launch attempt");
    return true;
  }

  *md = app_install_get_md(id, is_worker);

  // The cache is keyed on the install ID alone, and IDs are recycled. An entry orphaned by a
  // previous install leaves a stale binary at this ID, whose metadata comes from its own header --
  // so without this check we would silently launch the wrong app. Drop it and fetch the real one.
  AppDBEntry entry;
  if (*md && (app_db_get_app_entry_for_install_id(id, &entry) == S_SUCCESS) &&
      !uuid_equal(&(*md)->uuid, &entry.uuid)) {
    char cached_uuid[UUID_STRING_BUFFER_LENGTH];
    char expected_uuid[UUID_STRING_BUFFER_LENGTH];
    uuid_to_string(&(*md)->uuid, cached_uuid);
    uuid_to_string(&entry.uuid, expected_uuid);
    PBL_LOG_WRN("Stale app cache entry for id %" PRId32 ": cached %s, expected %s. Refetching.",
                id, cached_uuid, expected_uuid);
    app_install_release_md(*md);
    *md = NULL;
    app_cache_remove_entry(id);
    return true;
  }

  return false;
}

#endif

void process_manager_launch_process(const ProcessLaunchConfig *config) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);
  const AppInstallId id = config->id;
  const bool is_worker = config->worker;

  if (id == INSTALL_ID_INVALID) {
    PBL_LOG_DBG("Invalid ID");
    return;
  }

  const PebbleProcessMd *md = NULL;
#if !defined(CONFIG_RECOVERY_FW)
  if (app_install_id_from_app_db(id)) {
      if (!process_manager_check_SDK_compatible(id)) {
        return;
      }

    // This is a third party flash 3.0 app install
    if (prv_needs_fetch(id, &md, is_worker)) {
      PBL_LOG_DBG("Cache entry did not exist on launch attempt");

      // Freed in app_fetch_ui.c
      AppFetchUIArgs *fetch_args = kernel_malloc_check(sizeof(AppFetchUIArgs));
      *fetch_args = (AppFetchUIArgs){};
      fetch_args->common = config->common;
      fetch_args->app_id = id;
      fetch_args->forcefully = config->forcefully;

      // if the data is wakeup info, then copy out that information.
      if ((config->common.reason == APP_LAUNCH_WAKEUP) && (config->common.args != NULL)) {
        fetch_args->wakeup_info = *(WakeupInfo *)config->common.args;
        fetch_args->common.args = &fetch_args->wakeup_info;
      }

      PebbleEvent e = {
        .type = PEBBLE_APP_FETCH_REQUEST_EVENT,
        .app_fetch_request = {
          .id = id,
          .with_ui = true,
          .fetch_args = fetch_args,
        },
      };
      event_put(&e);
      return;
    } else {
      // tell the app cache that we are launching this application.
      app_cache_app_launched(id);
    }
  }
#endif
  // we either came here if PRF or if we didn't start a fetch
  // md is either already initialized, or we took a code path that didn't try
  if (!md) {
    md = app_install_get_md(id, is_worker);
  }

  if (!md) {
    PBL_LOG_ERR("Tried to launch non-existant app!");
    return;
  }

#if !defined(CONFIG_RECOVERY_FW)
  if (!is_worker) {
    // Check if the app ram size is valid in order to determine if its SDK version is supported.
    if (!app_manager_is_app_supported(md)) {
      PBL_LOG_WRN("Tried to launch an app with an unsupported SDK version.");
      AppInstallEntry entry;
      if (!app_install_get_entry_for_install_id(id, &entry)) {
        // can't retrieve app install entry for id
        PBL_LOG_ERR("Failed to get entry for id %"PRId32, id);
      } else if (app_install_entry_is_watchface(&entry)) {
        // If the watchface is for an unsupported SDK version, we need to switch the default
        // watchface back to tictoc. Otherwise, we will be stuck in the launcher forever.
        watchface_set_default_install_id(INSTALL_ID_INVALID);
        watchface_launch_default(NULL);
      }

      // Not going to launch this, release the allocated memory
      app_install_release_md(md);

      return;
    }

    // RockyJS is no longer supported. Show a dialog to the user and, if it's a watchface,
    // revert to the default watchface so the user isn't stuck in the launcher.
    if (md->is_rocky_app) {
      PBL_LOG_WRN("Tried to launch an unsupported RockyJS app.");
      AppInstallEntry entry;
      if (app_install_get_entry_for_install_id(id, &entry) &&
          app_install_entry_is_watchface(&entry)) {
        watchface_set_default_install_id(INSTALL_ID_INVALID);
        watchface_launch_default(NULL);
      }

      ExpandableDialog *expandable_dialog = expandable_dialog_create("Unsupported App");
      Dialog *dialog = expandable_dialog_get_dialog(expandable_dialog);
      const char *error_text =
          i18n_noop("This app uses RockyJS which is no longer supported.");
      dialog_set_text(dialog, i18n_get(error_text, expandable_dialog));
      dialog_set_icon(dialog, RESOURCE_ID_GENERIC_WARNING_SMALL);
      i18n_free(error_text, expandable_dialog);

      WindowStack *window_stack = modal_manager_get_window_stack(ModalPriorityAlert);
      expandable_dialog_push(expandable_dialog, window_stack);

      app_install_release_md(md);
      return;
    }
  }
#endif

  if (is_worker) {
    worker_manager_launch_new_worker_with_args(md, NULL);
  } else {
    app_manager_launch_new_app(&(AppLaunchConfig) {
      .md = md,
      .common = config->common,
      .forcefully = config->forcefully,
    });
  }
}

// ---------------------------------------------------------------------------------------------
//! This method returns true if the process is safe to kill (it has exited out of it's main function). If the
//! the process is not already safe to kill, it will "prod" it to exit, set a timer, and return false.
//!
//! The app manager and worker manager MUST call this before they call the code to kill the task and clean it up
//! (most of that work is done by process_manager_process_cleanup()). If it returns false, they should abort the
//! current process exit operation and wait for another KILL event to get posted.
//!
//! If the task does eventually fall through it's main function, the exit handling code will set the safe to kill
//! boolean and post another KILL event to the KernelMain which will result in this method being called again, and
//! this time it will see the safe to kill is set and return true
//!
//! If the task does not exit by itself before the timer expires, then the timer will post another KILL event
//! with graceful set to false. This will result in this method being alled again with gracefully = false. When
//! we see this, we just try and make sure the app is not stuck in privilege code. If it's not, we return true
//! and allow the caller to kill the task.
//!
//! If however, the task is in privilege mode, we tell the syscall machinery to set the safe to kill boolean as
//! soon as the current syscall returns and set another timer. Once that timer expires, if the task is no longer
//! in privilege mode we post another KILL event (graceful = false). If the task is still in privilege mode then,
//! we croak.
bool process_manager_make_process_safe_to_kill(PebbleTask task, bool gracefully) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);
  ProcessContext *context = prv_get_context_for_task(task);

  // If already safe to kill, we're done
  if (context->safe_to_kill) {
#if !defined(CONFIG_RECOVERY_FW) && !defined(CONFIG_SHELL_SDK)
    // Stop per-watchface time tracking if this was a watchface
    if (context->app_md->process_type == ProcessTypeWatchface) {
      PBL_ANALYTICS_TIMER_STOP(watchface_time_ms);
    }
#endif

    return true;
  }

  PBL_LOG_DBG("make %s process safe to kill: state %u", pebble_task_get_name(task),
                              context->closing_state);

  if (gracefully) {
    if (context->closing_state == ProcessRunState_Running) {
      context->closing_state = ProcessRunState_GracefullyClosing;

      PBL_LOG_DBG("Attempting to gracefully deinit %s", pebble_task_get_name(task));

      // Send deinit event to app:
      PebbleEvent deinit_event = {
        .type = PEBBLE_PROCESS_DEINIT_EVENT,
      };
      process_manager_send_event_to_process(task, &deinit_event);

      // Set a timer to forcefully close the app in 3 seconds if it doesn't respond by then. The app can respond
      // within 3 seconds by posting a PEBBLE_APP_KILL_EVENT (graceful=true), which will result in
      // app_manager_close_current_app() being called, which in turn calls this method with graceful = true.
      bool success = new_timer_start(s_deinit_timer_id, 3 * 1000, prv_graceful_close_timer_callback, (void*)task,
                        0 /*flags*/);
      PBL_ASSERTN(success);
    }
    // Else we're already in the gracefully closing state, just let the timer run out or the
    // app to mark itself as safe_to_kill.
  } else {
    PBL_LOG_DBG("Check if we can force stop the %s task", pebble_task_get_name(task));

    if (prv_force_stop_task_if_unprivileged(context)) {
      PBL_LOG_DBG("Got it");

#if !defined(CONFIG_RECOVERY_FW) && !defined(CONFIG_SHELL_SDK)
      // Stop per-watchface time tracking if this was a watchface
      if (context->app_md->process_type == ProcessTypeWatchface) {
        PBL_ANALYTICS_TIMER_STOP(watchface_time_ms);
      }
#endif

      return true;
    }

    // Non-graceful close
    if (context->closing_state == ProcessRunState_Running ||
        context->closing_state == ProcessRunState_GracefullyClosing) {
      // Right before a syscall drops privilege, it calls
      // process_manager_force_close_syscall_exit_trap to check whether
      // it is about to return control to a misbehaving app. This
      // function checks the process context's closing state and makes
      // the process safe to kill if its state is set to ForceClosing.
      // All we have to do is set the state and wait.
      context->closing_state = ProcessRunState_ForceClosing;
      PBL_LOG_DBG("task is privileged, setting the syscall exit trap");

      // Send a DEINIT event to wake up the app if it's blocked waiting for events
      // (e.g., in sys_get_pebble_event). This allows the syscall to return and
      // trigger process_manager_handle_syscall_exit() which will mark the process
      // as safe to kill.
      PBL_LOG_DBG("Sending DEINIT event to wake %s from syscall",
              pebble_task_get_name(task));
      PebbleEvent deinit_event = {
        .type = PEBBLE_PROCESS_DEINIT_EVENT,
      };
      process_manager_send_event_to_process(task, &deinit_event);

      bool success = new_timer_start(s_deinit_timer_id, 3 * 1000, prv_force_close_timer_callback, (void*)task,
              0 /*flags*/);
      PBL_ASSERTN(success);
    }
  }
  return false;
}

// -----------------------------------------------------------------------------------------------------------
// This is designed to be called from the task itself, in privilege mode, after it exits. It is called from
// app_task_exit for app tasks and worker_task_exit from worker tasks
NORETURN process_manager_task_exit(void) {
  PebbleTask task = pebble_task_get_current();
  ProcessContext *context = prv_get_context_for_task(task);

  // If this is not a system app, output its heap usage stats.
  if (context->app_md->process_storage == ProcessStorageFlash) {
    const Heap *heap;
    if (task == PebbleTask_App) {
      heap = app_state_get_heap();
    } else if (task == PebbleTask_Worker) {
      heap = worker_state_get_heap();
    } else {
      WTF;
    }

    // FIXME: We cast heap_size's size_t result to int because for some reason our printf doesn't
    // like the %zd formatter
    APP_LOG(APP_LOG_LEVEL_INFO, "Heap Usage for %s: Total Size <%dB> Used <%uB> Still allocated <%uB>",
        pebble_task_get_name(task), (int) heap_size(heap), heap->high_water_mark, heap->current_size);
  }

  // Let the task manager know we're done cleaning up.
  context->safe_to_kill = true;

  // Tell the task manager that we want to be killed. This may be redundant if we're responding to a DEINIT
  // message, but just in case we're exiting on our own (someone found the sys_exit syscall and called in when
  // we weren't expecting it?) we should let the app manager know.
  process_manager_put_kill_process_event(task, true);

  // Better to die in our sleep ...
  pbl_thread_suspend(NULL /* self */);

  // We don't expect someone to resume us.
  PBL_CROAK("");

}

// ---------------------------------------------------------------------------------------------
// Get the args for the current process
const void *process_manager_get_current_process_args(void) {
  return prv_get_context()->args;
}

// ---------------------------------------------------------------------------------------------
// Setup the system services required for this process. Called by app_manager and worker_manager
// right before we launch the task for the new process.
void process_manager_process_setup(PebbleTask task) {
  ProcessContext *context = prv_get_context_for_task(task);
  persist_service_client_open(&context->app_md->uuid);
}

// ---------------------------------------------------------------------------------------------
//! Kills the process, giving it no chance to clean things up or exit gracefully. The proces must already be in a
//! state where it's safe to exit, so the caller must call process_manager_make_process_safe_to_kill() first and only
//! call this method if process_manager_make_process_safe_to_kill() returns true;
void process_manager_process_cleanup(PebbleTask task) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  ProcessContext *context = prv_get_context_for_task(task);
  PBL_ASSERTN(context->safe_to_kill);

  PBL_LOG_DBG("%s is getting cleaned up", pebble_task_get_name(task));

  // Shutdown services that may be running. Do this before we destory the task and clear the queue
  // just in case other services are still in flight.
  accel_service_cleanup_task_session(task);
  animation_service_cleanup(task);
  persist_service_client_close(&context->app_md->uuid);
  event_reset_from_process_queue(task);
  evented_timer_clear_process_timers(task);
  event_service_clear_process_subscriptions(task);

#ifdef CONFIG_HRM
  hrm_manager_process_cleanup(task, context->install_id);
#endif

#ifndef CONFIG_RECOVERY_FW
#ifdef CONFIG_MIC
  voice_kill_app_session(task);
#endif
  dls_inactivate_sessions(task);

  if (task == PebbleTask_App) {
  }
#endif // CONFIG_RECOVERY_FW

  // Unregister the task
  pebble_task_unregister(task);

  new_timer_stop(s_deinit_timer_id);

  if (context->task_handle) {
    pbl_thread_abort(context->task_handle);
    context->task_handle = NULL;
  }

  // cleanup memory that was used to store the Md, but only if it isn't a system application
  app_install_release_md(context->app_md);

  // Clear the old app metadata
  context->app_md = 0;
  context->install_id = INSTALL_ID_INVALID;

  if (context->to_process_event_queue) {
    event_queue_cleanup_and_reset(context->to_process_event_queue);
  }
  context->to_process_event_queue = NULL;
}

// -----------------------------------------------------------------------------------------------------------
void process_manager_close_process(PebbleTask task, bool gracefully) {
  if (task == PebbleTask_App) {
    // This will tell the app manager to switch to the last registered app.
    app_manager_close_current_app(gracefully);

  } else if (task == PebbleTask_Worker) {
    worker_manager_close_current_worker(gracefully);

  } else {
    WTF;
  }
}

// ----------------------------------------------------------------------------------------------
bool process_manager_send_event_to_process(PebbleTask task, PebbleEvent* e) {
  ProcessContext *context = prv_get_context_for_task(task);

  if (context->to_process_event_queue == 0) {
    PBL_LOG_WRN("Dropped app event! Type: %u", e->type);
    return false;
  }

  // Put on app's own queue:
  if (pbl_msgq_put(context->to_process_event_queue, e, PBL_MSEC(1000)) != 0) {
    PBL_LOG_ERR("Failed to send event %u to app! Closing it!", e->type);
    // We could be called from a timer task callback, so post a kill event rather than call
    //  process_manager_close_process directly.
    process_manager_put_kill_process_event(task, false);
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------------------------
uint32_t process_manager_process_events_waiting(PebbleTask task) {
  ProcessContext *context = prv_get_context_for_task(task);

  if (context->to_process_event_queue == 0) {
    PBL_LOG_WRN("no event queue");
    return 0;
  }

  return pbl_msgq_num_used(context->to_process_event_queue);
}

// ----------------------------------------------------------------------------------------------
void process_manager_send_callback_event_to_process(PebbleTask task, void (*callback)(void *data), void *data) {

  PBL_ASSERTN(callback != NULL);
  PebbleEvent event = {
    .type = PEBBLE_CALLBACK_EVENT,
    .callback = {
      .callback = callback,
      .data = data,
    },
  };
  process_manager_send_event_to_process(task, &event);
}

// ----------------------------------------------------------------------------------------------
void *process_manager_address_to_offset(PebbleTask task, void *system_address) {
  ProcessContext *context = prv_get_context_for_task(task);
  if (system_address >= context->load_start &&
      system_address < context->load_end) {
    return (void*)((uintptr_t) system_address - (uintptr_t)context->load_start);
  }
  // Not in app space:
  return system_address;
}

// ----------------------------------------------------------------------------------------------

extern char __APP_RAM__[];
extern char __APP_RAM_end__[];
extern char __WORKER_RAM__[];
extern char __WORKER_RAM_end__[];

bool process_manager_is_address_in_region(PebbleTask task, const void *address,
                                          const void *lower_bound) {
  void *ram_start = NULL, *ram_end = NULL;
  if (task == PebbleTask_App) {
    ram_start = __APP_RAM__;
    ram_end = __APP_RAM_end__;
  } else if (task == PebbleTask_Worker) {
    ram_start = __WORKER_RAM__;
    ram_end = __WORKER_RAM_end__;
  } else {
    WTF;
  }

  // check for vulnerability: lower_bound outside of task's region
  PBL_ASSERTN(lower_bound >= ram_start);

  return (address >= lower_bound && address < ram_end);
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(void, sys_get_pebble_event, PebbleEvent *event) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(event, sizeof(*event));
  }

  pbl_msgq_get(prv_get_context()->to_process_event_queue, event, PBL_FOREVER);
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(AppLaunchReason, sys_process_get_launch_reason, void) {
  return prv_get_context()->launch_reason;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(ButtonId, sys_process_get_launch_button, void) {
  return prv_get_context()->launch_button;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(uint32_t, sys_process_get_launch_args, void) {
  if (sys_process_get_launch_reason() != APP_LAUNCH_TIMELINE_ACTION) {
    return 0;
  } else {
    return (uint32_t) process_manager_get_current_process_args();
  }
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(AppQuickLaunchAction, sys_process_get_quick_launch_action, void) {
  if (sys_process_get_launch_reason() != APP_LAUNCH_QUICK_LAUNCH) {
    return APP_QUICK_LAUNCH_ACTION_NONE;
  }
  return (AppQuickLaunchAction)(uintptr_t) process_manager_get_current_process_args();
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(AppExitReason, sys_process_get_exit_reason, void) {
  return prv_get_context()->exit_reason;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(void, sys_process_set_exit_reason, AppExitReason exit_reason) {
  // Just return if exit_reason is invalid
  if (exit_reason >= NUM_EXIT_REASONS) {
    return;
  }
  prv_get_context()->exit_reason = exit_reason;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(void, sys_process_get_wakeup_info, WakeupInfo *info) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(info, sizeof(*info));
  }
  *info = prv_get_context()->wakeup_info;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(const PebbleProcessMd*, sys_process_manager_get_current_process_md, void) {
  return prv_get_context()->app_md;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(bool, sys_process_manager_get_current_process_uuid, Uuid *uuid_out) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(uuid_out, sizeof(*uuid_out));
  }

  const PebbleProcessMd* app_md = prv_get_context()->app_md;
  if (!app_md) {
    return false;
  }
  *uuid_out = app_md->uuid;
  return true;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(AppInstallId, sys_process_manager_get_current_process_id, void) {
  return prv_get_context()->install_id;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(bool, process_manager_compiled_with_legacy2_sdk, void) {
  PebbleTask task = pebble_task_get_current();
  if (task != PebbleTask_App && task != PebbleTask_Worker) {
    return false;
  }

  const ProcessAppSDKType sdk_type =
      process_metadata_get_app_sdk_type(sys_process_manager_get_current_process_md());

  return sdk_type == ProcessAppSDKType_Legacy2x;
}

// -------------------------------------------------------------------------------------------
DEFINE_SYSCALL(bool, process_manager_compiled_with_legacy3_sdk, void) {
  PebbleTask task = pebble_task_get_current();
  if (task != PebbleTask_App && task != PebbleTask_Worker) {
    return false;
  }

  const ProcessAppSDKType sdk_type =
      process_metadata_get_app_sdk_type(sys_process_manager_get_current_process_md());

  return sdk_type == ProcessAppSDKType_Legacy3x;
}

DEFINE_SYSCALL(Version, sys_get_current_process_sdk_version, void) {
  return process_metadata_get_sdk_version(sys_process_manager_get_current_process_md());
}

DEFINE_SYSCALL(PlatformType, process_manager_current_platform, void) {
  PebbleTask task = pebble_task_get_current();
  if (task != PebbleTask_App && task != PebbleTask_Worker) {
    return PBL_PLATFORM_TYPE_CURRENT;
  }

  const PebbleProcessMd *const md = sys_process_manager_get_current_process_md();
  return process_metadata_get_app_sdk_platform(md);
}
