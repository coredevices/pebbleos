/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "shell/shell.h"

#include "apps/system_app_ids.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_install_types.h"
#include "process_management/app_manager.h"
#include "pbl/services/compositor/compositor_transitions.h"
#include "shell/prefs.h"

#define HEALTH_SHUTTER_COLOR PBL_IF_COLOR_ELSE(GColorBlack, GColorWhite)
#define ACTION_SHUTTER_COLOR PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite)

static const CompositorTransition *prv_get_watchface_compositor_animation(
    CompositorTransitionDirection direction) {
  return PBL_IF_RECT_ELSE(compositor_shutter_transition_get(direction,
                                                            shell_prefs_get_theme_highlight_color()),
                          compositor_port_hole_transition_app_get(direction));
}

static const CompositorTransition *prv_get_health_compositor_animation(
    CompositorTransitionDirection direction) {
  return PBL_IF_RECT_ELSE(compositor_shutter_transition_get(direction, HEALTH_SHUTTER_COLOR),
                          compositor_port_hole_transition_app_get(direction));
}

// Health card navigation is mirrored when quick launched with Down (the app then sits below
// the watchface), so its open/close animations must be mirrored too.
static bool prv_health_launch_is_reversed(AppLaunchReason reason, ButtonId button,
                                          const void *args) {
  const AppQuickLaunchAction action = (AppQuickLaunchAction)(uintptr_t)args;
  return (reason == APP_LAUNCH_QUICK_LAUNCH) && (button == BUTTON_ID_DOWN) &&
         ((action == APP_QUICK_LAUNCH_ACTION_TAP) || (action == APP_QUICK_LAUNCH_ACTION_HOLD));
}

static const CompositorTransition *prv_health_open_transition(const LaunchConfigCommon *config) {
  if (prv_health_launch_is_reversed(config->reason, config->button, config->args)) {
    // Down quick launch: health enters from the bottom, pushing the watchface up
    return PBL_IF_RECT_ELSE(
        compositor_slide_transition_app_get(true /* slide_up */),
        compositor_port_hole_transition_app_get(CompositorTransitionDirectionDown));
  }
  return prv_get_health_compositor_animation(CompositorTransitionDirectionUp);
}

static CompositorTransitionDirection prv_health_close_direction(void) {
  const ProcessContext *context = app_manager_get_task_context();
  return prv_health_launch_is_reversed(context->launch_reason, context->launch_button,
                                       context->args) ?
      CompositorTransitionDirectionUp : CompositorTransitionDirectionDown;
}

static const CompositorTransition *prv_get_action_compositor_animation(
    CompositorTransitionDirection direction) {
  return PBL_IF_RECT_ELSE(compositor_shutter_transition_get(direction, ACTION_SHUTTER_COLOR),
                          NULL);
}

const CompositorTransition *shell_get_watchface_compositor_animation(
    bool watchface_is_destination) {
  const CompositorTransitionDirection direction = watchface_is_destination ?
                CompositorTransitionDirectionLeft : CompositorTransitionDirectionRight;
  return prv_get_watchface_compositor_animation(direction);
}

static const CompositorTransition *prv_app_launcher_transition_animation(
    CompositorTransitionDirection direction) {
#if PBL_RECT
  const bool app_is_destination = (direction == CompositorTransitionDirectionRight);
  return compositor_launcher_app_transition_get(app_is_destination);
#else
  return compositor_port_hole_transition_app_get(direction);
#endif
}

const CompositorTransition *shell_get_close_compositor_animation(AppInstallId current_app_id,
                                                                 AppInstallId next_app_id) {
  const CompositorTransition *res = NULL;
  AppInstallEntry *app_entry = kernel_zalloc_check(sizeof(AppInstallEntry));

  if (app_install_get_entry_for_install_id(next_app_id, app_entry) &&
      app_install_entry_is_watchface(app_entry)) {
    if (current_app_id == APP_ID_LAUNCHER_MENU) {
      res = prv_get_watchface_compositor_animation(CompositorTransitionDirectionLeft);
      goto done;
    } else if (current_app_id == APP_ID_HEALTH_APP) {
      res = prv_get_health_compositor_animation(prv_health_close_direction());
      goto done;
    } else {
      res = prv_get_action_compositor_animation(CompositorTransitionDirectionLeft);
      goto done;
    }
  }

  if (next_app_id == APP_ID_LAUNCHER_MENU) {
    res = prv_app_launcher_transition_animation(CompositorTransitionDirectionLeft);
    goto done;
  }

  // If we get here, we don't use a compositor animation for the transition

done:
  kernel_free(app_entry);
  return res;
}

const CompositorTransition *shell_get_open_compositor_animation(
    AppInstallId current_app_id, AppInstallId next_app_id, const LaunchConfigCommon *config) {
  const CompositorTransition *res = NULL;
  AppInstallEntry *app_entry = kernel_zalloc_check(sizeof(AppInstallEntry));

  if (app_install_get_entry_for_install_id(current_app_id, app_entry)) {
    if (app_install_entry_is_watchface(app_entry)) {
      if (next_app_id == APP_ID_LAUNCHER_MENU) {
        res = prv_get_watchface_compositor_animation(CompositorTransitionDirectionRight);
        goto done;
      } else if (next_app_id == APP_ID_HEALTH_APP) {
        res = prv_health_open_transition(config);
        goto done;
      }
    } else if ((current_app_id == APP_ID_HEALTH_APP) &&
               app_install_get_entry_for_install_id(next_app_id, app_entry) &&
               app_install_entry_is_watchface(app_entry)) {
      res = prv_get_health_compositor_animation(prv_health_close_direction());
      goto done;
    }
  }

  if (current_app_id == APP_ID_LAUNCHER_MENU) {
    res = prv_app_launcher_transition_animation(CompositorTransitionDirectionRight);
    goto done;
  }

  // If we get here, we don't use a compositor animation for the transition

done:
  kernel_free(app_entry);
  return res;
}
