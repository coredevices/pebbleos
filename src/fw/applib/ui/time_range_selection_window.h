/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/date_time_selection_window_private.h"
#include "applib/ui/selection_layer.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"

struct TimeRangeSelectionWindowData;

typedef void (*TimeRangeSelectionCompleteCallback)(struct TimeRangeSelectionWindowData *window,
                                                   void *context);

typedef struct TimeRangeSelectionWindowData {
  Window window;
  SelectionLayer from_selection_layer;
  SelectionLayer to_selection_layer;
  TextLayer from_text_layer;
  TextLayer to_text_layer;

  TimeRangeSelectionCompleteCallback complete_callback;
  void *callback_context;

  TimeData from;
  TimeData to;
  char buf[3];
} TimeRangeSelectionWindowData;


void time_range_selection_window_init(TimeRangeSelectionWindowData *time_range_selection_window,
    GColor color, TimeRangeSelectionCompleteCallback complete_callback, void *callback_context);

void time_range_selection_window_deinit(TimeRangeSelectionWindowData *time_range_selection_window);
