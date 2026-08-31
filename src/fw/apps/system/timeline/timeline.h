/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "peek_layer.h"
#include "pin_window.h"
#include "model.h"
#include "layer.h"

#include "applib/ui/action_menu_layer.h"
#include "applib/ui/ui.h"
#include "popups/timeline/timeline_item_layer.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/evented_timer.h"

typedef enum {
  TimelineAppStateNone = 0,
  TimelineAppStatePeek,
  TimelineAppStateHidePeek,
  TimelineAppStateStationary,
  TimelineAppStateUpDown,
  TimelineAppStateFarDayHidePeek,
  TimelineAppStateShowDaySeparator,
  TimelineAppStateDaySeparator,
  TimelineAppStateHideDaySeparator,
  TimelineAppStatePushCard,
  TimelineAppStateCard,
  TimelineAppStatePopCard,
  TimelineAppStateNoEvents,
  TimelineAppStateInactive,
  TimelineAppStateExit,
} TimelineAppState;

typedef struct {
  TimelineDirection direction;
  bool launch_into_pin; //!< Launch to a pin specified by `pin_id`.
  bool stay_in_list_view; //!< Whether to stay in list view or launch into the detail view.
  Uuid pin_id;
  bool force_full;
  bool force_display_day_sep_on_start;
} TimelineArgs;

typedef struct {
  // Windows
  Window timeline_window;
  TimelinePinWindow pin_window;

  // Layers
  TimelineLayer timeline_layer;
  PeekLayer peek_layer;

  EventServiceInfo blobdb_event_info;
  EventServiceInfo focus_event_info;

  EventedTimerID inactive_timer_id; //!< To go back to watchface after inactivity
  EventedTimerID intro_timer_id; //!< To perform the intro animation after a peek
  EventedTimerID day_separator_timer_id; //!< To hide the day separator after a moment

  TimelineModel timeline_model;

  Animation *current_animation;

  TimelineAppState state;

  bool launch_into_deep_pin; //!< Whether we launched directly into a pin that isn't the first
  bool in_pin_view; //!< Whether we're in pin view
  bool full_app_view; //!< Whether we launched as full app mode
  bool day_sep_displayed_on_start;
  bool force_display_day_sep;
} TimelineAppData;

Animation *timeline_animate_back_from_card(void);

// uuid: 79C76B48-6111-4E80-8DEB-3119EEBEF33E
#define TIMELINE_UUID_INIT {0x79, 0xC7, 0x6B, 0x48, 0x61, 0x11, 0x4E, 0x80, \
                            0x8D, 0xEB, 0x31, 0x19, 0xEE, 0xBE, 0xF3, 0x3E}

// uuid: DAAE3686-BFF6-4BA5-921B-262F847BB6E8
#define TIMELINE_PAST_UUID_INIT {0xDA, 0xAE, 0x36, 0x86, 0xBF, 0xF6, 0x4B, 0xA5, \
                                 0x92, 0x1B, 0x26, 0x2F, 0x84, 0x7B, 0xB6, 0xE8}

// uuid: 426ccd53-b380-4d83-8d06-9893de3477ce
#define TIMELINE_FULL_UUID_INIT {0x42, 0x6c, 0xcd, 0x53, 0xb3, 0x80, 0x4d, 0x83, \
                                 0x8d, 0x06, 0x98, 0x93, 0xde, 0x34, 0x77, 0xce}

const PebbleProcessMd *timeline_get_app_info(void);
const PebbleProcessMd *timeline_past_get_app_info(void);
const PebbleProcessMd *timeline_full_get_app_info(void);