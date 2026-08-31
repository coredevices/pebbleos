/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "layer.h"

#include "applib/fonts/fonts.h"
#include "applib/graphics/text.h"

#include <stdint.h>

//! @file menu_layer.h
//! @addtogroup UI
//! @{
//!   @addtogroup Layer Layers
//!   @{
//!     @addtogroup MenuLayer
//!   @{

//! @internal
//! TODO: PBL-21467 Implement MenuCellLayer
//! MenuCellLayer is a virtual layer until it is actually implemented

typedef enum MenuCellLayerIconAlign {
  MenuCellLayerIconAlign_Left = GAlignLeft,
  MenuCellLayerIconAlign_Right = GAlignRight,
  MenuCellLayerIconAlign_TopLeft = GAlignTopLeft,
#if PBL_ROUND
  MenuCellLayerIconAlign_Top = GAlignTop,
#endif
} MenuCellLayerIconAlign;

typedef struct MenuCellLayerConfig {
  const char *title;
  const char *subtitle;
  const char *value;
  GFont title_font;
  GFont value_font;
  GFont subtitle_font;
  GTextOverflowMode overflow_mode;

  GBitmap *icon;
  MenuCellLayerIconAlign icon_align;
  const GBoxModel *icon_box_model;
  bool icon_form_fit;

  int horizontal_inset;
} MenuCellLayerConfig;

/////////////////////////////
// Cell Drawing functions

void menu_cell_layer_draw(GContext *ctx, const Layer *cell_layer,
                          const MenuCellLayerConfig *config);

//! Section drawing function to draw a basic section cell with the title, subtitle, and icon of the
//! section. Call this function inside the `.draw_row` callback implementation, see \ref
//! MenuLayerCallbacks. Note that if the size of `cell_layer` is too small to fit all of the cell
//! items specified, not all of them may be drawn.
//! @param ctx The destination graphics context
//! @param cell_layer The layer of the cell to draw
//! @param title If non-null, draws a title in larger text (24 points, bold
//! Raster Gothic system font).
//! @param subtitle If non-null, draws a subtitle in smaller text (18 points,
//! Raster Gothic system font). If `NULL`, the title will be centered vertically
//! inside the menu cell.
//! @param icon If non-null, draws an icon to the left of the text. If `NULL`,
//! the icon will be omitted and the leftover space is used for the title and
//! subtitle.
void menu_cell_basic_draw(GContext* ctx, const Layer *cell_layer, const char *title,
                          const char *subtitle, GBitmap *icon);

//! Cell drawing function similar to \ref menu_cell_basic_draw with the icon drawn on the right
//! Section drawing function to draw a basic section cell with the title, subtitle, and icon of
//! the section.
//! Call this function inside the `.draw_row` callback implementation, see \ref MenuLayerCallbacks
//! @param ctx The destination graphics context
//! @param cell_layer The layer of the cell to draw
//! @param title If non-null, draws a title in larger text (24 points, bold
//! Raster Gothic system font).
//! @param subtitle If non-null, draws a subtitle in smaller text (18 points,
//! Raster Gothic system font). If `NULL`, the title will be centered vertically
//! inside the menu cell.
//! @param icon If non-null, draws an icon to the right of the text. If `NULL`,
//! the icon will be omitted and the leftover space is used for the title and
//! subtitle.
void menu_cell_basic_draw_icon_right(GContext* ctx, const Layer *cell_layer, const char *title,
                                     const char *subtitle, GBitmap *icon);

//! Cell drawing function to draw a basic menu cell layout with title, subtitle
//! Cell drawing function to draw a menu cell layout with only one big title.
//! Call this function inside the `.draw_row` callback implementation, see
//! \ref MenuLayerCallbacks.
//! @param ctx The destination graphics context
//! @param cell_layer The layer of the cell to draw
//! @param title If non-null, draws a title in larger text (28 points, bold
//! Raster Gothic system font).
void menu_cell_title_draw(GContext* ctx, const Layer *cell_layer, const char *title);

//! @internal
//! Cell drawing function similar to \ref menu_cell_basic_draw_with_value and
//! \ref menu_cell_basic_draw_icon_right, except with specifiable fonts.
void menu_cell_basic_draw_custom(GContext* ctx, const Layer *cell_layer, GFont const title_font,
                                 const char *title, GFont const value_font, const char *value,
                                 GFont const subtitle_font, const char *subtitle, GBitmap *icon,
                                 bool icon_on_right, GTextOverflowMode overflow_mode);

//! Section header drawing function to draw a basic section header cell layout
//! with the title of the section.
//! Call this function inside the `.draw_header` callback implementation, see
//! \ref MenuLayerCallbacks.
//! @param ctx The destination graphics context
//! @param cell_layer The layer of the cell to draw
//! @param title If non-null, draws the title in small text (14 points, bold
//! Raster Gothic system font).
void menu_cell_basic_header_draw(GContext* ctx, const Layer *cell_layer, const char *title);

//! Returns whether or not the given cell layer is highlighted.
//! Using this for determining highlight behaviour is preferable to using
//! \ref menu_layer_get_selected_index. Row drawing callbacks may be invoked multiple
//! times with a different highlight status on the same cell in order to handle partially
//! highlighted cells during animation.
//! @param cell_layer The \ref Layer for the cell to check highlight status.
//! @return true if the given cell layer is highlighted in the menu.
bool menu_cell_layer_is_highlighted(const Layer *cell_layer);

//! Default cell height in pixels.
int16_t menu_cell_basic_cell_height(void);

//! Constant value representing \ref MenuLayer short cell height when this item is
//! the selected item on a round display.
#define MENU_CELL_ROUND_FOCUSED_SHORT_CELL_HEIGHT ((const int16_t) 68)

//! Constant value representing \ref MenuLayer short cell height when this item is
//! not the selected item on a round display.
#define MENU_CELL_ROUND_UNFOCUSED_SHORT_CELL_HEIGHT ((const int16_t) 24)

//! Constant value representing \ref MenuLayer tall cell height when this item is
//! the selected item on a round display.
#define MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT ((const int16_t) 84)

//! Constant value representing \ref MenuLayer tall cell height when this item is
//! not the selected item on a round display.
#define MENU_CELL_ROUND_UNFOCUSED_TALL_CELL_HEIGHT ((const int16_t) 32)

//! "Small" cell height in pixels.
int16_t menu_cell_small_cell_height(void);

//! Default section header height in pixels
#define MENU_CELL_BASIC_HEADER_HEIGHT ((const int16_t) 16)

//! Default menu separator height in pixels
#define MENU_CELL_BASIC_SEPARATOR_HEIGHT ((const int16_t) 0)

//! Default cell horizontal inset in pixels.
int16_t menu_cell_basic_horizontal_inset(void);
#define MENU_CELL_ROUND_FOCUSED_HORIZONTAL_INSET ((const int16_t) 16)
#define MENU_CELL_ROUND_UNFOCUSED_HORIZONTAL_INSET ((const int16_t) 34)

//!     @} // end addtogroup MenuLayer
//!   @} // end addtogroup Layer
//! @} // end addtogroup UI
