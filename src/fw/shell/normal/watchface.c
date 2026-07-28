/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "watchface.h"

#include "apps/system_app_ids.h"
#include "apps/system/launcher/launcher.h"
#include "apps/system/settings/quick_launch.h"
#include "apps/system/settings/quick_launch_app_menu.h"
#include "apps/system/settings/quick_launch_setup_menu.h"
#include "apps/system/timeline/timeline.h"
#include "apps/watch/low_power/face.h"
#include "kernel/event_loop.h"
#include "kernel/low_power.h"
#include "kernel/ui/modals/modal_manager.h"
#include "popups/timeline/peek.h"
#include "process_management/app_manager.h"
#include "process_management/pebble_process_md.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/compositor/compositor_transitions.h"
#include "applib/app_timer.h"
#include "applib/app_launch_reason.h"
#include "applib/ui/click_internal.h"
#ifdef CONFIG_TOUCH
#include "applib/touch_service.h"
#include "apps/system/watchfaces.h"
#include "pbl/services/touch/touch.h"
#include <pbl/util/math.h>
#endif
#include "pbl/services/notifications/do_not_disturb.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#ifdef CONFIG_ORIENTATION_MANAGER
#include "shell/prefs.h"
#endif 

#define QUICK_LAUNCH_HOLD_MS (400)
#define BIT_SET (1)
#define BIT_CLEAR (0)
#define COMBO_BACK_UP_BUTTONS ((BIT_SET << BUTTON_ID_BACK) | (BIT_SET << BUTTON_ID_UP))
#define COMBO_UP_DOWN_BUTTONS ((BIT_SET << BUTTON_ID_UP) | (BIT_SET << BUTTON_ID_DOWN))
#ifdef CONFIG_ORIENTATION_MANAGER
#define COMBO_BACK_UP_FLIPPED_BUTTONS ((BIT_SET << BUTTON_ID_BACK) | (BIT_SET << BUTTON_ID_DOWN))
#endif

static ClickManager s_click_manager;
static uint8_t s_buttons_pressed = BIT_CLEAR;
static AppTimer *s_combo_back_hold_timer = NULL;
static uint8_t s_active_combo_buttons = BIT_CLEAR;

#ifdef CONFIG_TOUCH
//! Hold a finger on the watchface for this long to open the watchface selector.
#define TOUCH_HOLD_MS (2000)
//! Movement beyond this many pixels cancels the hold (it's a drag, not a press).
#define TOUCH_HOLD_SLOP_PX (20)

static AppTimer *s_touch_hold_timer = NULL;
static int16_t s_touch_hold_x;
static int16_t s_touch_hold_y;
#endif

static bool prv_should_ignore_button_click(void) {
  if (app_manager_get_task_context()->closing_state != ProcessRunState_Running) {
    // Ignore if the app is not running (such as if it is in the process of closing)
    return true;
  }
  if (low_power_is_active()) {
    // If we're in low power mode we dont allow any interaction
    return true;
  }
  return false;
}

static void prv_launch_app_via_button(AppLaunchEventConfig *config,
                                      ClickRecognizerRef recognizer) {
  config->common.button = click_recognizer_get_button_id(recognizer);
  app_manager_put_launch_app_event(config);
}

static bool prv_is_combo_pressed(uint8_t combo_buttons) {
  return (s_buttons_pressed & combo_buttons) == combo_buttons;
}

static bool prv_combo_is_enabled(uint8_t combo_buttons) {
#ifdef CONFIG_ORIENTATION_MANAGER
  const bool orientation_flipped = display_orientation_is_left();
  if (orientation_flipped && combo_buttons == COMBO_BACK_UP_FLIPPED_BUTTONS) {
    return quick_launch_combo_back_up_is_enabled();
  } else if (!orientation_flipped && combo_buttons == COMBO_BACK_UP_BUTTONS) {
    return quick_launch_combo_back_up_is_enabled();
  } else if (combo_buttons == COMBO_UP_DOWN_BUTTONS) {
    return quick_launch_combo_up_down_is_enabled();
  }
#else
  if (combo_buttons == COMBO_BACK_UP_BUTTONS) {
    return quick_launch_combo_back_up_is_enabled();
  } else if (combo_buttons == COMBO_UP_DOWN_BUTTONS) {
    return quick_launch_combo_up_down_is_enabled();
  }
#endif
  return false;
}

static AppInstallId prv_combo_get_app(uint8_t combo_buttons) {
#ifdef CONFIG_ORIENTATION_MANAGER
  if (combo_buttons == COMBO_BACK_UP_FLIPPED_BUTTONS) {
    return quick_launch_combo_back_up_get_app();
  } else if (combo_buttons == COMBO_BACK_UP_BUTTONS) {
    return quick_launch_combo_back_up_get_app();
  } else if (combo_buttons == COMBO_UP_DOWN_BUTTONS) {
    return quick_launch_combo_up_down_get_app();
  }
#else
  if (combo_buttons == COMBO_BACK_UP_BUTTONS) {
    return quick_launch_combo_back_up_get_app();
  } else if (combo_buttons == COMBO_UP_DOWN_BUTTONS) {
    return quick_launch_combo_up_down_get_app();
  }
#endif
  return INSTALL_ID_INVALID;
}

static bool prv_is_any_combo_active(void) {
  return (s_combo_back_hold_timer != NULL) ||
         prv_is_combo_pressed(COMBO_BACK_UP_BUTTONS) ||
         prv_is_combo_pressed(COMBO_UP_DOWN_BUTTONS);
}

static void prv_combo_back_timer_callback(void *data) {
  s_combo_back_hold_timer = NULL;
  if (!prv_is_combo_pressed(s_active_combo_buttons)) {
    s_active_combo_buttons = BIT_CLEAR;
    return;
  }

  if (!prv_combo_is_enabled(s_active_combo_buttons)) {
    s_active_combo_buttons = BIT_CLEAR;
    return;
  }

  AppInstallId app_id = prv_combo_get_app(s_active_combo_buttons);
  const ButtonId source_button =
      (s_active_combo_buttons == COMBO_BACK_UP_BUTTONS) ? BUTTON_ID_BACK : BUTTON_ID_UP;
  s_active_combo_buttons = BIT_CLEAR;
  if (app_id != INSTALL_ID_INVALID) {
    // Reset all button states before launching app to prevent state corruption.
    s_buttons_pressed = BIT_CLEAR;
    app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
      .id = app_id,
      .common.reason = APP_LAUNCH_QUICK_LAUNCH,
      .common.button = source_button,
      .common.args = (void*)APP_QUICK_LAUNCH_ACTION_COMBO,
    });
  }
}

static void prv_check_combo_back_hold(void) {
  uint8_t combo_buttons = BIT_CLEAR;

#ifdef CONFIG_ORIENTATION_MANAGER
  const bool orientation_flipped = display_orientation_is_left();
  if (orientation_flipped && prv_is_combo_pressed(COMBO_BACK_UP_FLIPPED_BUTTONS)) {
    combo_buttons = COMBO_BACK_UP_FLIPPED_BUTTONS;
  } else if (!orientation_flipped && prv_is_combo_pressed(COMBO_BACK_UP_BUTTONS)) {
    combo_buttons = COMBO_BACK_UP_BUTTONS;
  } else if (prv_is_combo_pressed(COMBO_UP_DOWN_BUTTONS)) {
    combo_buttons = COMBO_UP_DOWN_BUTTONS;
  }
#else
  if (prv_is_combo_pressed(COMBO_BACK_UP_BUTTONS)) {
    combo_buttons = COMBO_BACK_UP_BUTTONS;
  } else if (prv_is_combo_pressed(COMBO_UP_DOWN_BUTTONS)) {
    combo_buttons = COMBO_UP_DOWN_BUTTONS;
  }
#endif

  if (combo_buttons != BIT_CLEAR) {
    if (s_combo_back_hold_timer == NULL) {
      s_active_combo_buttons = combo_buttons;
      // Cancel individual button timers to prevent them from firing.
      // This ensures only the combo executes, not individual hold handlers.
#ifdef CONFIG_ORIENTATION_MANAGER
      if (combo_buttons == COMBO_BACK_UP_FLIPPED_BUTTONS) {
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_BACK]);
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_DOWN]);
      } else if (combo_buttons == COMBO_BACK_UP_BUTTONS) {
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_BACK]);
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_UP]);
      } else {
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_UP]);
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_DOWN]);
      }
#else
      if (combo_buttons == COMBO_BACK_UP_BUTTONS) {
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_BACK]);
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_UP]);
      } else {
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_UP]);
        click_recognizer_reset(&s_click_manager.recognizers[BUTTON_ID_DOWN]);
      }
#endif
      s_combo_back_hold_timer =
          app_timer_register(QUICK_LAUNCH_HOLD_MS, prv_combo_back_timer_callback, NULL);
    }
  } else {
    if (s_combo_back_hold_timer != NULL) {
      app_timer_cancel(s_combo_back_hold_timer);
      s_combo_back_hold_timer = NULL;
      s_active_combo_buttons = BIT_CLEAR;
    }
  }
}

static void prv_launch_timeline_app(AppInstallId app_id, ClickRecognizerRef recognizer,
                                    AppLaunchReason reason) {
  static TimelineArgs s_timeline_args;
  s_timeline_args.launch_into_pin = true;
  s_timeline_args.stay_in_list_view = true;
  timeline_peek_get_item_id(&s_timeline_args.pin_id);

  const ButtonId button = click_recognizer_get_button_id(recognizer);
  const CompositorTransition *animation = NULL;
  const bool is_up = (button == BUTTON_ID_UP);
  const bool is_future = (app_id == APP_ID_TIMELINE) || (app_id == APP_ID_TIMELINE_FULL && !is_up);

  if (app_id == APP_ID_TIMELINE) {
    s_timeline_args.direction = TimelineIterDirectionFuture;
  } else if (app_id == APP_ID_TIMELINE_PAST) {
    s_timeline_args.direction = TimelineIterDirectionPast;
  } else {
    s_timeline_args.direction = is_future ? TimelineIterDirectionFuture : TimelineIterDirectionPast;
  }

  const bool timeline_is_destination = true;
#if PBL_ROUND
  animation = compositor_dot_transition_timeline_get(is_future, timeline_is_destination);
#else
  const bool jump = (!uuid_is_invalid(&s_timeline_args.pin_id) && !timeline_peek_is_first_event());
  animation = jump ? compositor_peek_transition_timeline_get() :
                     compositor_slide_transition_timeline_get(is_future, timeline_is_destination,
                                                              timeline_peek_is_future_empty());
#endif
  prv_launch_app_via_button(&(AppLaunchEventConfig) {
    .id = app_id,
    .common.reason = reason,
    .common.args = &s_timeline_args,
    .common.transition = animation,
  }, recognizer);
}

static void prv_launch_quick_launch_app(AppInstallId app_id, ClickRecognizerRef recognizer,
                                        AppLaunchReason timeline_reason, void *non_timeline_args) {
  const bool is_timeline = (app_id == APP_ID_TIMELINE) ||
                           (app_id == APP_ID_TIMELINE_PAST) ||
                           (app_id == APP_ID_TIMELINE_FULL);
  if (is_timeline) {
    prv_launch_timeline_app(app_id, recognizer, timeline_reason);
  } else {
    prv_launch_app_via_button(&(AppLaunchEventConfig) {
      .id = app_id,
      .common.reason = APP_LAUNCH_QUICK_LAUNCH,
      .common.args = non_timeline_args,
    }, recognizer);
  }
}

static void prv_quick_launch_handler(ClickRecognizerRef recognizer, void *data) {
  ButtonId button = click_recognizer_get_button_id(recognizer);

  if (prv_is_any_combo_active()) {
    return;
  }

  AppInstallId app_id = quick_launch_is_enabled(button) ? quick_launch_get_app(button)
                                                        : INSTALL_ID_INVALID;
  if (app_id == INSTALL_ID_INVALID) {
    app_id = app_install_get_id_for_uuid(&quick_launch_setup_get_app_info()->uuid);
  }
  s_buttons_pressed = BIT_CLEAR;  // Reset our own tracking

  prv_launch_quick_launch_app(app_id, recognizer, APP_LAUNCH_QUICK_LAUNCH,
                              (void*)APP_QUICK_LAUNCH_ACTION_HOLD);
}

static void prv_launch_up_down(ClickRecognizerRef recognizer, void *data) {
  ButtonId button = click_recognizer_get_button_id(recognizer);
  
  if (prv_is_any_combo_active()) {
    return;
  }
  
  if (!quick_launch_single_click_is_enabled(button)) return;
  const AppInstallId app_id = quick_launch_single_click_get_app(button);

  prv_launch_quick_launch_app(app_id, recognizer, APP_LAUNCH_SYSTEM,
                              (void*)APP_QUICK_LAUNCH_ACTION_TAP);
}

static void prv_configure_click_handler(ButtonId button_id, ClickHandler single_click_handler) {
  ClickConfig *cfg = &s_click_manager.recognizers[button_id].config;
  cfg->long_click.delay_ms = QUICK_LAUNCH_HOLD_MS;
  cfg->long_click.handler = prv_quick_launch_handler;
  cfg->click.handler = single_click_handler;
}

static void prv_launch_launcher_app(ClickRecognizerRef recognizer, void *data) {
  static const LauncherMenuArgs s_launcher_args = { .reset_scroll = true };
  prv_launch_app_via_button(&(AppLaunchEventConfig) {
    .id = APP_ID_LAUNCHER_MENU,
    .common.args = &s_launcher_args,
  }, recognizer);
}

static void prv_dismiss_timeline_peek(ClickRecognizerRef recognizer, void *data) {
  if (prv_is_any_combo_active()) {
    return;
  }
  timeline_peek_dismiss();
}

#ifdef CONFIG_TOUCH
static bool prv_touch_ignore(void) {
  const bool modal_focused = modal_manager_get_enabled() &&
      !(modal_manager_get_properties() & ModalProperty_Unfocused);
  return prv_should_ignore_button_click() || modal_focused ||
         !app_manager_is_watchface_running();
}

static void prv_touch_hold_cancel(void) {
  if (s_touch_hold_timer != NULL) {
    app_timer_cancel(s_touch_hold_timer);
    s_touch_hold_timer = NULL;
  }
}

static void prv_touch_hold_timer_callback(void *data) {
  s_touch_hold_timer = NULL;
  if (prv_touch_ignore()) {
    return;
  }
  static const WatchfacesLaunchArgs s_selector_args = { .selector_mode = true };
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = APP_ID_WATCHFACES,
    .common.reason = APP_LAUNCH_SYSTEM,
    .common.args = &s_selector_args,
  });
}

static void prv_watchface_touch_handler(const TouchEvent *event, void *context) {
  if (prv_touch_ignore()) {
    prv_touch_hold_cancel();
    return;
  }
  switch (event->type) {
    case TouchEvent_Touchdown:
      prv_touch_hold_cancel();
      s_touch_hold_x = event->x;
      s_touch_hold_y = event->y;
      s_touch_hold_timer =
          app_timer_register(TOUCH_HOLD_MS, prv_touch_hold_timer_callback, NULL);
      break;
    case TouchEvent_PositionUpdate:
      if (s_touch_hold_timer != NULL &&
          (ABS(event->x - s_touch_hold_x) > TOUCH_HOLD_SLOP_PX ||
           ABS(event->y - s_touch_hold_y) > TOUCH_HOLD_SLOP_PX)) {
        prv_touch_hold_cancel();
      }
      break;
    case TouchEvent_Liftoff:
      prv_touch_hold_cancel();
      break;
  }
}
#endif // CONFIG_TOUCH

static void prv_watchface_configure_click_handlers(void) {
  prv_configure_click_handler(BUTTON_ID_UP, prv_launch_up_down);
  prv_configure_click_handler(BUTTON_ID_DOWN, prv_launch_up_down);
  prv_configure_click_handler(BUTTON_ID_SELECT, prv_launch_launcher_app);
  prv_configure_click_handler(BUTTON_ID_BACK, prv_dismiss_timeline_peek);
}

void watchface_init(void) {
  click_manager_init(&s_click_manager);
  prv_watchface_configure_click_handlers();
#ifdef CONFIG_TOUCH
  // Kernel-side subscription: keeps the touch sensor powered so a long press
  // on the watchface can open the selector. Foreground gating happens in the
  // handler itself. Marked as a system subscription so it doesn't trigger the
  // touch-backlight override meant for third-party touch apps.
  touch_service_subscribe(prv_watchface_touch_handler, NULL);
  touch_set_system_subscribed(true);
#endif
}

void watchface_handle_button_event(PebbleEvent *e) {
  if (prv_should_ignore_button_click()) {
    return;
  }
  switch (e->type) {
    case PEBBLE_BUTTON_DOWN_EVENT:
      s_buttons_pressed |= (BIT_SET << e->button.button_id);
      click_recognizer_handle_button_down(&s_click_manager.recognizers[e->button.button_id]);
      prv_check_combo_back_hold();
      break;
    case PEBBLE_BUTTON_UP_EVENT:
      s_buttons_pressed &= ~(BIT_SET << e->button.button_id);
      prv_check_combo_back_hold();
      click_recognizer_handle_button_up(&s_click_manager.recognizers[e->button.button_id]);
      break;
    default:
      PBL_CROAK("Invalid event type: %u", e->type);
      break;
  }
}

static void prv_watchface_launch_low_power(void) {
  PBL_LOG_DBG("Switching default watchface to low_power_mode watchface");
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = APP_ID_LOW_POWER_FACE,
  });
}

void watchface_launch_default(const CompositorTransition *animation) {
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = watchface_get_default_install_id(),
    .common.transition = animation,
  });
}

static void kernel_callback_watchface_launch(void* data) {
  watchface_launch_default(NULL);
}

void command_watch(void) {
  launcher_task_add_callback(kernel_callback_watchface_launch, NULL);
}

void watchface_start_low_power(void) {
  app_manager_set_minimum_run_level(ProcessAppRunLevelNormal);
  prv_watchface_launch_low_power();
}

void watchface_reset_click_manager(void) {
  click_manager_reset(&s_click_manager);
}
