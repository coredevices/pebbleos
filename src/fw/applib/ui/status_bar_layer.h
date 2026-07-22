/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "applib/app_timer.h"
#include "applib/ui/animation.h"
#include "applib/graphics/gtypes.h"
#include "layer.h"
#include "applib/event_service_client.h"

//! @file status_bar_layer.h
//! @addtogroup UI
//! @{
//!   @addtogroup Layer Layers
//!   @{
//!     @addtogroup StatusBarLayer
//! \brief Layer that serves as a configurable status bar.
//!     @{

//! The fixed height of the status bar, including separator height, for all platforms.
#define _STATUS_BAR_LAYER_HEIGHT(plat) PBL_PLATFORM_SWITCH(plat, \
  /*aplite*/ 16, \
  /*basalt*/ 16, \
  /*chalk*/ 24, \
  /*diorite*/ 16, \
  /*emery*/ 20, \
  /*flint*/ 16, \
  /*gabbro*/ 20)

//! The fixed height of the status bar, including separator height
#define STATUS_BAR_LAYER_HEIGHT _STATUS_BAR_LAYER_HEIGHT(PBL_PLATFORM_TYPE_CURRENT)

//! The height of the status bar when rendering the "Big & Bold" clock, including
//! separator height, for all platforms.
#define _STATUS_BAR_LAYER_LARGE_BOLD_HEIGHT(plat) PBL_PLATFORM_SWITCH(plat, \
  /*aplite*/ 20, \
  /*basalt*/ 20, \
  /*chalk*/ 24, /* already tall enough; only the font grows */ \
  /*diorite*/ 20, \
  /*emery*/ 26, \
  /*flint*/ 20, \
  /*gabbro*/ 26)

//! The "Big & Bold" status bar height for the current platform.
#define STATUS_BAR_LAYER_LARGE_BOLD_HEIGHT \
  _STATUS_BAR_LAYER_LARGE_BOLD_HEIGHT(PBL_PLATFORM_TYPE_CURRENT)

//! The min width of the status bar
#define STATUS_BAR_LAYER_MIN_WIDTH 35
//! The distance from info_text to right edge of the status bar
#define STATUS_BAR_LAYER_INFO_PADDING 7
//! The vertical offset for the status bar layer
#define STATUS_BAR_LAYER_SEPARATOR_Y_OFFSET 2

//! System wide timeout when reverting back to clock mode from custom text
#define STATUS_BAR_LAYER_TITLE_TIMEOUT 5000

//! The size of the title buffer
#define TITLE_TEXT_BUFFER_SIZE 20
//! The size of the info buffer
#define INFO_TEXT_BUFFER_SIZE 8
//! The max size of the total value of set_info_progress before progress is displayed as percentage
#define MAX_INFO_TOTAL 99


//! Values that are used to indicate the different status bar modes
typedef enum {
  //! Default mode. Time display takes priority
  StatusBarLayerModeClock = 0,
  //! Indicates to the user that something is loading. May or may not manually revert when complete.
  StatusBarLayerModeLoading = 1,
  //! Custom text with an optional auto-revert to the default mode.
  StatusBarLayerModeCustomText = 2,
  //! Same as StatusBarLayerModeClock but renders the time using a bold font.
  StatusBarLayerModeClockBold = 3,
  //! Same as StatusBarLayerModeClock but renders the time using a larger bold
  //! font in a taller bar (the notification "Big & Bold" style).
  StatusBarLayerModeClockLargeBold = 4,
  //! Same as StatusBarLayerModeClockLargeBold but draws a 1px black outline around the glyphs, for
  //! legibility over busy backgrounds such as album art.
  StatusBarLayerModeClockLargeBoldOutlined = 5,
} StatusBarLayerMode;

//! Values that are used to indicate the different status bar separator modes.
typedef enum {
  //! The default mode. No separator will be shown.
  StatusBarLayerSeparatorModeNone = 0,
  //! A dotted separator at the bottom of the status bar.
  StatusBarLayerSeparatorModeDotted = 1,
} StatusBarLayerSeparatorMode;

//! The data structure of the StatusBarLayerSeparator
typedef struct StatusBarLayerSeparator {
  StatusBarLayerSeparatorMode mode;
  //! Separator Animation specific variables will eventually be placed here
} StatusBarLayerSeparator;

//! Configuration of a StatusBarLayer independently from Layer and timer code
typedef struct StatusBarLayerConfig {
  char title_text_buffer[TITLE_TEXT_BUFFER_SIZE];   // center title text buffer
  char info_text_buffer[INFO_TEXT_BUFFER_SIZE];     // right info text buffer
  GColor foreground_color;                          // default:GColorWhite
  GColor background_color;                          // default:GColorBlack
  StatusBarLayerSeparator separator;                // default:StatusBarLayerSeparatorModeDotted
  StatusBarLayerMode mode;                          // default:StatusBarLayerModeClock
} StatusBarLayerConfig;

//! renders a status bar as described into a given rectangle
void status_bar_layer_render(GContext *ctx, const GRect *bounds, StatusBarLayerConfig *config);

//! The data structure of a StatusBarLayer.
//! @note a `StatusBarLayer *` can safely be cast to a `Layer *` and can thus be
//! used with all other functions that take a `Layer *` as an argument.
//! <br/>For example, the following is legal:
//! \code{.c}
//! StatusBarLayer status_bar_layer;
//! ...
//! layer_set_hidden((Layer *)&status_bar_layer, true);
//! \endcode
typedef struct StatusBarLayer {
  Layer layer;
  StatusBarLayerConfig config;
  AppTimer *title_timer_id;                         // timer id for title revert
  EventServiceInfo tick_event;                      // Event service to update the time
  int previous_min_of_day;
} StatusBarLayer;

//! Creates a new StatusBarLayer on the heap and initializes it with the default values.
//!
//! * Text color: \ref GColorBlack
//! * Background color: \ref GColorWhite
//! * Frame: `GRect(0, 0, screen_width, STATUS_BAR_LAYER_HEIGHT)`
//! The status bar is automatically marked dirty after this operation.
//! You can call \ref layer_set_frame() to create a StatusBarLayer of a different width.
void status_bar_layer_init(StatusBarLayer *status_bar_layer);

//! Creates a new StatusBarLayer on the heap and initializes it with the default values.
//!
//! * Text color: \ref GColorBlack
//! * Background color: \ref GColorWhite
//! * Frame: `GRect(0, 0, screen_width, STATUS_BAR_LAYER_HEIGHT)`
//! The status bar is automatically marked dirty after this operation.
//! You can call \ref layer_set_frame() to create a StatusBarLayer of a different width.
//!
//! \code{.c}
//! // Change the status bar width to make space for the action bar
//! int16_t width = layer_get_bounds(root_layer).size.w - ACTION_BAR_WIDTH;
//! GRect frame = GRect(0, 0, width, STATUS_BAR_LAYER_HEIGHT);
//! layer_set_frame(status_bar_layer_get_layer(status_bar), frame);
//! layer_add_child(root_layer, status_bar_layer_get_layer(status_bar));
//! \endcode
//! @return A pointer to the StatusBarLayer, which will be allocated to the heap,
//! `NULL` if the StatusBarLayer could not be created
StatusBarLayer *status_bar_layer_create(void);

//! Destroys a StatusBarLayer previously created by status_bar_layer_create.
//! @param status_bar_layer The StatusBarLayer to destroy
void status_bar_layer_destroy(StatusBarLayer *status_bar_layer);

//! Deinitializes the StatusBarLayer and frees any caches.
//! @param status_bar_layer The StatusBarLayer to deinitialize
//! @internal
void status_bar_layer_deinit(StatusBarLayer *status_bar_layer);

//! Gets the "root" Layer of the status bar, which is the parent for the sub-
//! layers used for its implementation.
//! @param status_bar_layer Pointer to the StatusBarLayer for which to get the "root" Layer
//! @return The "root" Layer of the status bar.
//! @note The result is always equal to `(Layer *) status_bar_layer`.
Layer *status_bar_layer_get_layer(StatusBarLayer *status_bar_layer);

//! Gets background color of StatusBarLayer
//! @param status_bar_layer The StatusBarLayer of which to get the color
//! @return GColor of background color property
GColor status_bar_layer_get_background_color(const StatusBarLayer *status_bar_layer);

//! Sets the background and foreground colors of StatusBarLayer
//! @param status_bar_layer The StatusBarLayer of which to set the colors
//! @param background The new \ref GColor to set for background
//! @param foreground The new \ref GColor to set for text and other foreground elements
void status_bar_layer_set_colors(StatusBarLayer *status_bar_layer, GColor background,
                                 GColor foreground);

//! Gets foreground color of StatusBarLayer
//! @param status_bar_layer The StatusBarLayer of which to get the color
//! @return GColor of foreground color property
GColor status_bar_layer_get_foreground_color(const StatusBarLayer *status_bar_layer);

//! Sets text for title.
//! @param status_bar_layer - StatusBarLayer to set the title text
//! @param text - text to set as title. This will be copied to an internal buffer.
//! @param revert - go back to clock after STATUS_BAR_LAYER_TITLE_TIMEOUT milliseconds
//! @param animate - animate status bar to bounce in
void status_bar_layer_set_title(StatusBarLayer *status_bar_layer, const char *text, bool revert,
                                bool animate);

//! Gets title text of StatusBarLayer
//! @param status_bar_layer The StatusBarLayer of which to get the text
//! @return The char pointer to current title text
const char *status_bar_layer_get_title(const StatusBarLayer *status_bar_layer);

//! Resets title text to clock text
//! @param cb_data The StatusBarLayer of which to reset the title to clock
void status_bar_layer_reset_title(void *cb_data);

//! Sets the info section to display arbitrary text
//! @param status_bar_layer The StatusBarLayer of which to set the info text
//! @param text The text to set in the information section, limited to 8 characters
void status_bar_layer_set_info_text(StatusBarLayer *status_bar_layer, const char *text);

//! Sets values for the info section to intelligently represent progress, which
//! can be used for pagination. For a total below 99, a count will be displayed,
//! such as 2/8. For a total greater than 99, a percentage will be displayed.
//! @param status_bar_layer The StatusBarLayer of which to set the info text
//! @param current The current value, such as current page number
//! @param total The total value. A total over MAX_INFO_TOTAL will display as a percentage.
void status_bar_layer_set_info_progress(StatusBarLayer *status_bar_layer, uint16_t current,
                                        uint16_t total);

//! Gets info text of StatusBarLayer
//! @param status_bar_layer The StatusBarLayer of which to get the info text
//! @return The char pointer to current text of info text
const char *status_bar_layer_get_info_text(const StatusBarLayer *status_bar_layer);

//! Resets the text in the info section, clearing the value.
//! @param status_bar_layer The StatusBarLayer of which to clear the info text
void status_bar_layer_reset_info(StatusBarLayer *status_bar_layer);

//! Sets the mode of the StatusBarLayer separator, to help divide it from content
//! @param status_bar_layer The StatusBarLayer of which to set the separator mode
//! @param mode Determines the separator mode
void status_bar_layer_set_separator_mode(StatusBarLayer *status_bar_layer,
                                         StatusBarLayerSeparatorMode mode);

//! Sets the mode, resizing the layer frame/bounds when the new mode has a different bar height.
//! @param status_bar_layer The StatusBarLayer to configure
//! @param mode The StatusBarLayerMode to set
void status_bar_layer_set_mode(StatusBarLayer *status_bar_layer, StatusBarLayerMode mode);

//! Gets the mode of the StatusBarLayer separator
//! @param status_bar_layer The StatusBarLayer of which to get the separator mode
//! @return The current mode of the separator
StatusBarLayerSeparatorMode status_bar_layer_get_separator_mode(
    const StatusBarLayer *status_bar_layer);

//! @internal
bool layer_is_status_bar_layer(Layer *layer);
int16_t status_layer_get_title_text_width(StatusBarLayer *status_bar_layer);

//!     @} // end addtogroup StatusBarLayer
//!   @} // end addtogroup Layer
//! @} // end addtogroup UI
