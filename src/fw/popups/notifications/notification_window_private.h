/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/action_menu_window.h"
#include "applib/ui/status_bar_layer.h"
#include "apps/system/timeline/peek_layer.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/timeline/swap_layer.h"

typedef struct NotificationWindowData {
  Window window;

  RegularTimerInfo reminder_watchdog_timer_id; // Clear stale reminders once a minute

  EventedTimerID pop_timer_id; //!< Timer that automatically pops us in case of inactivity.
  bool pop_timer_is_final; // true, if pop_timer_id cannot be rescheduled anymore

  bool is_modal;
  bool window_frozen; // Don't pop when performing an action via a hotkey until the action completes
  bool first_notif_loaded;

  // Used to keep track of when a notification is modified from a different (event)
  // task, so the reload only occurs in the correct task when something changes
  bool notifications_modified;

  // nothing but rendering the action button
  Layer action_button_layer;

  Uuid notification_app_id; //!< app id for loading custom notification icons

  PeekLayer *peek_layer;
  TimelineResourceInfo peek_icon_info;
  EventedTimerID peek_layer_timer;
  Animation *peek_animation;

  // Handles the multiple layers
  SwapLayer swap_layer;
  StatusBarLayer status_layer;
  ActionMenu *action_menu;

  // Icon in status bar if in DND.
  // This should really be part of the status bar but support hasn't been
  // implemented yet. This also won't work well with round displays.
  // Remove this once the status bar layer supports icons
  // PBL-22859
  Layer dnd_icon_layer;
  GBitmap dnd_icon;
  bool dnd_icon_visible;

  // Delayed vibration support - vibe at end of animation instead of immediately
  bool pending_vibe;
  Uuid pending_vibe_id;
  bool pending_backlight;
  bool pending_sound;

  // Set once we've pushed a modal color preempt for this notification's
  // backlight pulse, so window_unload knows to balance it with a pop.
  bool color_preempted;
} NotificationWindowData;
