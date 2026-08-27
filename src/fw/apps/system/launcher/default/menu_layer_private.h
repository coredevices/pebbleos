/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/math.h"
#include "board/display.h"

// Smallest tier's row height, used only to size the glance cache (app_glance_service.c) for the
// worst case. Not the actual rendered height - keep in sync with s_launcher_cell_dimensions.
#define LAUNCHER_MENU_LAYER_CELL_RECT_CELL_HEIGHT_MIN (42)

#if PBL_ROUND && PBL_DISPLAY_HEIGHT >= 200
//! Two "unfocused" cells above and below one centered "focused" cell (5 total for larger display)
#define LAUNCHER_MENU_LAYER_NUM_UNFOCUSED_ROWS_PER_SIDE (2)
#define LAUNCHER_MENU_LAYER_NUM_VISIBLE_ROWS (5)
#elif PBL_ROUND
//! One "unfocused" cell above and below one centered "focused" cell
#define LAUNCHER_MENU_LAYER_NUM_UNFOCUSED_ROWS_PER_SIDE (1)
#define LAUNCHER_MENU_LAYER_NUM_VISIBLE_ROWS (3)
#else
#define LAUNCHER_MENU_LAYER_NUM_VISIBLE_ROWS \
    (DIVIDE_CEIL(DISP_ROWS, LAUNCHER_MENU_LAYER_CELL_RECT_CELL_HEIGHT_MIN))
#endif
