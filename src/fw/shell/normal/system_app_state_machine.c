/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "shell/system_app_state_machine.h"

#include "apps/core/panic_window.h"
#include "apps/system/battery_critical.h"
#include "apps/system_app_ids.h"
#include "apps/system/launcher/launcher.h"
#include "apps/watch/low_power/face.h"
#include "shell/normal/watchface.h"
#include "kernel/low_power.h"
#include "kernel/panic.h"
#include "pbl/services/battery/battery_monitor.h"
#include <pbl/logging/logging.h>
#include "process_management/app_manager.h"

//! @file system_app_state_machine.c
//!
//! This file implements our app to app flow that makes up our normal shell. It defines
//! which app first runs at start up and what app should be launched to replace the current
//! app if the current app wants to close.
//!
//! The logic for which app should replace closing apps is a little tricky. Apps can be launched
//! in various ways, either due to direct user interaction (selecting an app in the launcher) or
//! through the phone app using pebble protocol (for example, a new app being installed or a
//! companion app launching its watchapp in response to an event). What we want to happen is
//! the user can then close that app and end up in a rough approximation of where they came from.
//!
//! The way we implement this is by having two apps that make up roots of the graph. If you're
//! in the launcher and you launch an app, closing that app will return to the launcher. If you
//! attempt to nest further (you launch an app from the launcher and that app in turn launches
//! another app), closing any app will still return you to the launcher. This is done to prevent
//! the stack from growing too deep and having to exit a ton of apps to get back to where you want.
//! The watchface is also a root (closing an app that launched while you were in a watchface
//! will return to you to the watchface). Finally, closing the launcher will return you to the
//! watchface, and closing the watchface (either by pressing select or the watchface crashing)
//! should take you to the launcher.
//!
//! Launching any watchface for any reason will put you in the "root watchface" state.
//!
//! Below is a pretty ASCII picture to describe the states we can be in. What happens when you
//! close an app is illustrated with the arrow with the X.
//!
//! +---------------------+----+     +-------------------------+-----+
//! | Remote Launched App |    |     |  Remote Launched App    |     |
//! +---------------+-----+ <--+     |  Launcher Launched App  | <---+
//!                 X                +---------------+---------+
//!      ^          |                                X
//!      |          v                       ^        |
//!      |                                  |        v
//! +----+----------------+ +X-----> +------+------------------+
//! |  Watchface          |          |     Launcher            |
//! +---------------------+ <-----X+ +-------------------------+
//!

//! As per the above block comment, are we currently rooted in the watchface stack or the
//! launcher stack?
static bool s_rooted_in_watchface = false;

const PebbleProcessMd* system_app_state_machine_system_start(void) {
  // start critical battery app when necessary
  if (battery_monitor_critical_lockout()) {
    return battery_critical_get_app_info();
  }

  if (low_power_is_active()) {
    return low_power_face_get_app_info();
  }

  if (launcher_panic_get_current_error() != 0) {
    return panic_app_get_app_info();
  }

  return launcher_menu_app_get_app_info();
}

//! @return True if the currently running app is an installed watchface
static bool prv_current_app_is_watchface(void) {
  return app_install_is_watchface(app_manager_get_current_app_id());
}

AppInstallId system_app_state_machine_get_last_registered_app(void) {
  // If we're rooted in the watchface but we're not the watchface itself, or the launcher
  // is closing, we should launch the watchface.
  if ((s_rooted_in_watchface && !prv_current_app_is_watchface())
      || (app_manager_get_current_app_md() == launcher_menu_app_get_app_info())) {
    return watchface_get_default_install_id();
  }

  return APP_ID_LAUNCHER_MENU;
}

const PebbleProcessMd* system_app_state_machine_get_default_app(void) {
  return launcher_menu_app_get_app_info();
}

void system_app_state_machine_register_app_launch(AppInstallId app_id) {
  if (app_id == APP_ID_LAUNCHER_MENU) {
    s_rooted_in_watchface = false;
  } else if (app_install_is_watchface(app_id)) {
    s_rooted_in_watchface = true;
  }

  // Other app launches don't modify our root so just ignore them.
}

void system_app_state_machine_panic(void) {
  if (app_manager_is_initialized()) {
    app_manager_launch_new_app(&(AppLaunchConfig) {
      .md = panic_app_get_app_info(),
    });
  }

  // Else, just wait for the app_manager to initialize to show the panic app using
  // system_app_state_machine_system_start().
}

