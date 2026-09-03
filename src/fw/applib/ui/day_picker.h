/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/ui.h"
#include "util/time/time.h"
#include <stdbool.h>

typedef enum {
  DayPickerKindEveryday = 0,
  DayPickerKindWeekdays,
  DayPickerKindWeekends,
  DayPickerKindCustom,
  DayPickerKindJustOnce,
  DayPickerKindJustOnceThenDelete,
  DayPickerKindNumItems,
} DayPickerKind;

typedef struct {
  DayPickerKind kind;
  bool custom_days[DAYS_PER_WEEK];
} DayPickerResult;

typedef struct {
  DayPickerResult initial;
  GColor highlight_color;
  bool allow_once;
} DayPickerConfig;

typedef void (*DayPickerCallback)(DayPickerResult result, void *context);

void day_picker_push(DayPickerConfig config, DayPickerCallback callback,
                     void *context);

void custom_day_picker_push(bool initial_days[DAYS_PER_WEEK],
                            DayPickerCallback callback, void *context,
                            GColor highlight_color);

const char *day_picker_kind_get_string(DayPickerKind kind);