/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/date_time_selection_window_private.h"
#include "applib/ui/selection_layer.h"
#include "applib/ui/status_bar_layer.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"

#include <stdint.h>
#include <time.h>

struct DateSelectionWindowData;

typedef void (*DateSelectionCompleteCallback)(struct DateSelectionWindowData *window, void *ctx);

//! Date data stored as fields mirroring struct tm
typedef struct {
  int16_t year;  //!< years since 1900 (tm_year)
  int8_t month;  //!< 0-11 (tm_mon)
  int8_t day;    //!< 1-31 (tm_mday)
} DateData;

typedef struct DateSelectionWindowData {
  Window window;
  SelectionLayer selection_layer;
  TextLayer label_text_layer;
  StatusBarLayer status_layer;

  DateData date;

  DateSelectionCompleteCallback complete_callback;
  void *callback_context;

  //! Scratch buffer for cell text rendering (max 6 chars + NUL, e.g. "-30868")
  char cell_buf[7];
} DateSelectionWindowData;

//! Populate the date fields with the current local date.
void date_selection_window_set_to_current_date(DateSelectionWindowData *window);

//! Initialise the date selection window.
//! @param window    The window data to initialise.
//! @param label     Optional title string (may be NULL).
//! @param color     Highlight colour for the active selection cell.
//! @param complete  Callback invoked when the user confirms their selection.
//! @param context   Caller-provided pointer passed to the callback.
void date_selection_window_init(DateSelectionWindowData *window, const char *label,
                                GColor color, DateSelectionCompleteCallback complete,
                                void *context);

//! Deinitialize the date selection window.
void date_selection_window_deinit(DateSelectionWindowData *window);
