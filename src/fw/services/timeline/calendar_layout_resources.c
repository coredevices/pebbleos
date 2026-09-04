/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/calendar_layout_resources.h"

#include "pbl/util/size.h"

// FIXME: PBL-28898 GPath algorithm requires strange coordinates for pixel perfection
// The paths here result in pixel perfect icons with the current gpath filled algorithm,
// but when gpath filled is fixed to correctly match its coordinates, these MUST be updated.

CalendarStartIcon g_calendar_start_icon = {
  .image = {
    .version = 1,
    .size = { 9, 9 },
    .command_list = {
      .num_commands = 1,
    },
  },
  .command = {
    .type = GDrawCommandTypePath,
    .fill_color = { .argb = GColorBlackARGB8 },
    .num_points = STATIC_ARRAY_LENGTH(GPoint, START_ICON_POINTS),
  },
  .points = START_ICON_POINTS,
};

CalendarEndIcon g_calendar_end_icon = {
  .image = {
    .version = 1,
    .size = { 9, 9 },
    .command_list = {
      .num_commands = 1,
    },
  },
  .command = {
    .type = GDrawCommandTypePath,
    .fill_color = { .argb = GColorBlackARGB8 },
    .num_points = STATIC_ARRAY_LENGTH(GPoint, END_ICON_POINTS),
  },
  .points = END_ICON_POINTS,
};
