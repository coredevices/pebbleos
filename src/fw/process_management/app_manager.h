/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/types.h"
#include "launch_config.h"
#include "process_manager.h"

#include "kernel/events.h"
#include "kernel/pebble_tasks.h"
#include "resource/resource.h"
#include "pbl/services/compositor/compositor.h"

#include <stdbool.h>

#define APP_TASK_PRIORITY (PBL_PRIO_IDLE + 2)

typedef enum {
  AppTaskCtxIdxLauncher = 0,
  AppTaskCtxIdxApp,
  AppTaskCtxIdxCount,
  AppTaskCtxIdxInvalid = -1,
} AppTaskCtxIdx;

typedef struct AppLaunchConfig {
  LaunchConfigCommon common;
  const PebbleProcessMd *md;
  bool restart; //!< Allows the current app to be restarted
  bool forcefully; //!< Causes the current app to be forcefully closed
} AppLaunchConfig;

typedef struct AppLaunchEventConfig {
  LaunchConfigCommon common;
  AppInstallId id;
} AppLaunchEventConfig;


// App management functions
void app_manager_init(void);

bool app_manager_is_initialized(void);

void app_manager_start_first_app(void);

bool app_manager_is_first_app_launched(void);


//! Start up a new application with the given metadata. This will kill the currently running app.
//! May only be called from the KernelMain task
bool app_manager_launch_new_app(const AppLaunchConfig *config);

//! Handles the PebbleEvent of type PEBBLE_APP_FETCH_REQUEST_EVENT, for example, by launching the
//! fetched app.
void app_manager_handle_app_fetch_request_event(const PebbleAppFetchRequestEvent *const evt);

//! Close the currently running app. This will cause the next app in the
//! system app state machine to automatically get launched.
//! May only be called from the KernelMain task
void app_manager_close_current_app(bool gracefully);

//! Stop the current app, and bring up the launcher.
void app_manager_force_quit_to_launcher(void);

//! Sets a minimum run level for app launches that interrupt the current running app. Any app below the specified
//! will be ignored when trying to launch.
//! The minimum run level will be reset to the incoming app whenever an app is launched. This function just allows
//! an app to modify the minimum level between app changes.
void app_manager_set_minimum_run_level(ProcessAppRunLevel run_level);

//! Gets the wakeup info from the PebbleEvent sent to app_manager
//! to provide newly launched app access to AppLaunchEvent data
WakeupInfo app_manager_get_app_wakeup_state(void);

//! @return Whether or not we support an app execution environment for this app.
bool app_manager_is_app_supported(const PebbleProcessMd *md);

// Send App Management Events To The Launcher
///////////////////////////////////////////////////////////////////////////////

//! Adds a PebbleLaunchAppEvent to the event queue
void app_manager_put_launch_app_event(const AppLaunchEventConfig *config);

// Getters For App Management State
///////////////////////////////////////////////////////////////////////////////

const PebbleProcessMd* app_manager_get_current_app_md(void);

AppInstallId app_manager_get_current_app_id(void);

const PebbleProcessMd* app_manager_get_current_worker(void);

ProcessContext* app_manager_get_task_context(void);

bool app_manager_is_watchface_running(void);

ResAppNum app_manager_get_current_resource_num(void);

AppLaunchReason app_manager_get_launch_reason(void);

ButtonId app_manager_get_launch_button(void);

void app_manager_get_framebuffer_size(GSize *size);


//! Exit the application. Do some cleanup to make sure things close nicely.
//! Called from the app task
NORETURN app_task_exit(void);

