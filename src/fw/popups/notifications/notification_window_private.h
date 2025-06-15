/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "applib/ui/action_menu_window.h"
#include "applib/ui/status_bar_layer.h"
#include "apps/system_apps/timeline/peek_layer.h"
#include "services/common/evented_timer.h"
#include "services/normal/timeline/swap_layer.h"

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
  
  EventedTimerID vibration_timer_id; //!< Timer for delayed vibration on notification
} NotificationWindowData;
