/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "button_lock.h"

#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/dialog_private.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/vibes.h"
#include "kernel/event_loop.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_management/app_manager.h"
#include "shell/normal/watchface.h"
#include "shell/prefs.h"
#include "system/passert.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/i18n/i18n.h"
#include <pbl/logging/logging.h>

#ifdef CONFIG_TOUCH
#include "pbl/services/touch/touch.h"
#endif

#define BUTTON_LOCK_COMBO ((1 << BUTTON_ID_BACK) | (1 << BUTTON_ID_DOWN))
#define BUTTON_LOCK_POPUP_TIMEOUT_MS (1800)

static TimerID s_combo_timer = TIMER_INVALID_ID;
static uint8_t s_buttons_held;
//! Deliver a button UP iff its DOWN was delivered, so click recognizers in
//! the app/watchface never see an unbalanced press.
static uint8_t s_downs_delivered;
static bool s_combo_pending;
//! Set once a hold toggled the lock; blocks re-triggering until all buttons
//! are released, so a continuous hold toggles exactly once.
static bool s_combo_consumed;
static bool s_toggle_cancelled;
static bool s_locked;
static SimpleDialog *s_hint_dialog;

static void prv_hint_dialog_unload(void *context) {
  s_hint_dialog = NULL;
}

static const DialogCallbacks s_hint_dialog_callbacks = {
    .unload = prv_hint_dialog_unload,
};

static SimpleDialog *prv_push_popup(const char *text, const DialogCallbacks *callbacks) {
  SimpleDialog *simple_dialog = simple_dialog_create("ButtonLock");
  Dialog *dialog = simple_dialog_get_dialog(simple_dialog);
  const char *msg = i18n_get(text, dialog);
  dialog_set_text(dialog, msg);
  dialog_set_timeout(dialog, BUTTON_LOCK_POPUP_TIMEOUT_MS);
  if (callbacks) {
    dialog_set_callbacks(dialog, callbacks, NULL);
  }
  i18n_free(msg, dialog);
  simple_dialog_push(simple_dialog, modal_manager_get_window_stack(ModalPriorityGeneric));
  return simple_dialog;
}

static void prv_show_hint_popup(void) {
  if (s_hint_dialog) {
    return;
  }
  s_hint_dialog = prv_push_popup(i18n_noop("Hold Back + Down to unlock"), &s_hint_dialog_callbacks);
}

static void prv_pop_hint_popup(void) {
  if (!s_hint_dialog) {
    return;
  }
  dialog_pop(simple_dialog_get_dialog(s_hint_dialog));
  s_hint_dialog = NULL;
}

//! KernelMain callback posted by the combo hold timer.
static void prv_toggle_lock_cb(void *data) {
  if (s_toggle_cancelled || !s_combo_pending) {
    return;
  }
  s_combo_consumed = true;
  s_locked = !s_locked;
  PBL_LOG_DBG("Button lock %s", s_locked ? "engaged" : "released");

#ifdef CONFIG_TOUCH
  if (s_locked) {
    touch_service_set_globally_enabled(false);
  } else {
    // touch_is_globally_enabled() is the persisted user pref, not the runtime
    // switch flipped above, so this restores the user's touch setting.
    touch_service_set_globally_enabled(touch_is_globally_enabled());
  }
#endif

  if (s_locked) {
    vibes_short_pulse();
  } else {
    vibes_double_pulse();
  }

  prv_pop_hint_popup();
  prv_push_popup(s_locked ? i18n_noop("Buttons Locked") : i18n_noop("Buttons Unlocked"), NULL);
}

//! Runs on the NewTimer thread; just bounce to KernelMain.
static void prv_combo_timer_cb(void *data) {
  launcher_task_add_callback(prv_toggle_lock_cb, NULL);
}

void button_lock_init(void) {
  s_combo_timer = new_timer_create();
}

bool button_lock_is_locked(void) {
  return s_locked;
}

bool button_lock_handle_button_event(PebbleEvent *e) {
  const ButtonId button_id = e->button.button_id;
  const bool is_down = (e->type == PEBBLE_BUTTON_DOWN_EVENT);

  if (is_down) {
    s_buttons_held |= (1 << button_id);
  } else {
    s_buttons_held &= ~(1 << button_id);
  }
  if (s_buttons_held == 0) {
    s_combo_consumed = false;
  }

  const bool combo_held = (s_buttons_held == BUTTON_LOCK_COMBO) && !s_combo_consumed &&
                          (shell_prefs_get_button_lock_hold_ms() != 0);

  if (combo_held && !s_combo_pending) {
    s_combo_pending = true;
    s_toggle_cancelled = false;
    launcher_cancel_force_quit();
    if (!s_locked && app_manager_is_watchface_running()) {
      // Kill the first combo button's armed quick launch long click.
      watchface_reset_click_manager();
    }
    PBL_ASSERTN(new_timer_start(s_combo_timer, shell_prefs_get_button_lock_hold_ms(),
                                prv_combo_timer_cb, NULL, 0 /* flags */));
    // Swallow the combo-completing DOWN: the first combo button's DOWN was already delivered
    // (its click fires like with the quick launch combos), the second button must stay
    // invisible so no click recognizer arms for it. Its UP is swallowed via s_downs_delivered.
    return true;
  }
  if (!combo_held && s_combo_pending) {
    s_combo_pending = false;
    s_toggle_cancelled = true;
    new_timer_stop(s_combo_timer);
  }

  if (is_down) {
    if (s_locked || s_combo_pending) {
      if (s_locked && !s_combo_pending) {
        prv_show_hint_popup();
      }
      return true;
    }
    s_downs_delivered |= (1 << button_id);
    return false;
  }

  const bool deliver = (s_downs_delivered & (1 << button_id));
  s_downs_delivered &= ~(1 << button_id);
  return !deliver;
}
