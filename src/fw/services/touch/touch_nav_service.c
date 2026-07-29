/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/touch/touch_nav_service.h"

#ifdef CONFIG_TOUCH

#include "applib/graphics/gtypes.h"
#include "applib/ui/recognizer/touch_nav.h"
#include "kernel/event_loop.h"
#include "kernel/pebble_tasks.h"
#include "kernel/ui/modals/modal_manager.h"
#include "pbl/services/touch/touch.h"
#include "process_management/process_manager.h"
#include "process_state/app_state/app_state.h"
#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"

// --- Action-bar snapshot publish (applib -> current task's nav state) --------------------------

//! Resolve the touch-nav state of the task that owns the calling ActionBarLayer. Apps publish into
//! their own nav twin; kernel modals into the modal twin. Other tasks have no bar.
static TouchNavState *prv_current_touch_nav_state(void) {
  switch (pebble_task_get_current()) {
    case PebbleTask_App:
      return app_state_get_touch_nav_state();
    case PebbleTask_KernelMain:
      return modal_manager_get_touch_nav_state();
    default:
      return NULL;
  }
}

DEFINE_SYSCALL(void, sys_touch_set_action_bar, const GRect *frame, uint8_t icon_mask) {
  if (PRIVILEGE_WAS_ELEVATED && frame) {
    syscall_assert_userspace_buffer(frame, sizeof(*frame));
  }
  TouchNavState *state = prv_current_touch_nav_state();
  if (!state) {
    return;
  }
  // Copy the caller's frame out of (possibly user) memory before storing it.
  GRect local_frame;
  if (frame) {
    local_frame = *frame;
  }
  touch_nav_set_action_bar(state, frame ? &local_frame : NULL, icon_mask);
}

// --- App opt-in (applib app_touch_navigation_enable -> app-task participation) -----------------

DEFINE_SYSCALL(void, sys_app_touch_navigation_enable, bool enable) {
  // App-facing opt-in: only the app task has a per-app participation flag / nav twin to reconcile.
  if (pebble_task_get_current() != PebbleTask_App) {
    return;
  }
  app_touch_nav_set_participating(enable);
}

// --- KernelMain-bound effects -----------------------------------------------------------------

static void prv_kernel_subscribe_cb(void *unused) {
  modal_touch_nav_subscribe();
  // Take the permanent sensor hold here on KernelMain so the sensor stays powered while nav is on.
  touch_set_system_hold(true);
}

static void prv_kernel_unsubscribe_cb(void *unused) {
  modal_touch_nav_unsubscribe();
}

static void prv_kernel_release_hold_cb(void *unused) {
  // Release the permanent sensor hold on KernelMain. Queued AFTER prv_kernel_unsubscribe_cb so the
  // hold outlives the kernel unsubscribe (disable step 5 runs after step 3, not before it).
  touch_set_system_hold(false);
}

// --- App-task-bound effects -------------------------------------------------------------------

static void prv_app_subscribe_cb(void *unused) {
  app_touch_nav_subscribe();
}

static void prv_app_resync_cb(void *unused) {
  // Re-evaluate rather than blindly unsubscribe: an app that explicitly opted in follows the
  // master pref alone, so turning only the Touch Navigation sub-pref off must not cut it off.
  app_touch_nav_resync();
}

// --- Concrete transaction ops ------------------------------------------------------------------

static void prv_persist(void *ctx, bool enable) {
  // The pref file write is done by the shell pref system before we run; flip the runtime gate.
  touch_set_nav_enabled(enable);
}

static void prv_kernel_subscribe(void *ctx) {
  launcher_task_add_callback(prv_kernel_subscribe_cb, NULL);
}

static void prv_take_system_hold(void *ctx) {
  // Folded into the KernelMain callback above; nothing to do synchronously.
}

static void prv_synthesize_liftoff(void *ctx) {
  // Narrow contract: unwinds the backlight hold and driver state only (does not reach managers).
  touch_release_active();
}

static void prv_kernel_cancel_reset_unsub(void *ctx) {
  launcher_task_add_callback(prv_kernel_unsubscribe_cb, NULL);
}

static void prv_app_unsubscribe(void *ctx) {
  process_manager_send_callback_event_to_process(PebbleTask_App, prv_app_resync_cb, NULL);
}

static void prv_release_system_hold(void *ctx) {
  // Defer to KernelMain instead of releasing synchronously on the calling task: steps 3 (kernel
  // cancel_and_reset + unsubscribe) are already queued on KernelMain, so scheduling the release
  // after them makes the actual async order 2->3->5 rather than releasing the hold before 3-4 run.
  launcher_task_add_callback(prv_kernel_release_hold_cb, NULL);
}

static const TouchNavTxnOps s_txn_ops = {
  .persist = prv_persist,
  .kernel_subscribe = prv_kernel_subscribe,
  .take_system_hold = prv_take_system_hold,
  .synthesize_liftoff = prv_synthesize_liftoff,
  .kernel_cancel_reset_unsub = prv_kernel_cancel_reset_unsub,
  .app_unsubscribe = prv_app_unsubscribe,
  .release_system_hold = prv_release_system_hold,
};

void touch_nav_set_enabled(bool enable) {
  touch_nav_transaction_apply(&s_txn_ops, enable);
  if (enable) {
    // Also pick up any already-running app so nav starts working without a relaunch.
    process_manager_send_callback_event_to_process(PebbleTask_App, prv_app_subscribe_cb, NULL);
  }
}

void touch_nav_master_changed(void) {
  // The master pref flipped without changing the effective system state (the Touch Navigation
  // sub-pref is off): only an opted-in running app is affected, so re-evaluate its twin.
  process_manager_send_callback_event_to_process(PebbleTask_App, prv_app_resync_cb, NULL);
}

#endif  // CONFIG_TOUCH
