/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/preferred_content_size.h"
#include "applib/ui/animation.h"
#include "applib/ui/window.h"
#include "kernel/events.h"

#define TIMELINE_PEEK_HEIGHT \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ PBL_IF_RECT_ELSE(51, 45),                        \
      /* medium */ PBL_IF_RECT_ELSE(51, 45),                       \
      /* large */ 59,                                              \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* x-large */ 59                                             \
    )

#define TIMELINE_PEEK_ICON_BOX_WIDTH \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ PBL_IF_RECT_ELSE(30, 51),                        \
      /* medium */ PBL_IF_RECT_ELSE(30, 51),                       \
      /* large */ PBL_IF_RECT_ELSE(34, 51),                        \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* x-large */ PBL_IF_RECT_ELSE(34, 51)                       \
    )

#define TIMELINE_PEEK_MARGIN (5)

// On round displays the peek is drawn at full width and the round display
// mask clips the corners that fall outside the circle. Spalding's original
// origin of 112 was chosen for an 180-tall display; scale it to the current
// display height so taller round panels (e.g. 260-tall getafix) sit in the
// equivalent lower portion of the circle rather than bottom-flush, which
// would put the peek deep in the narrow bottom curve of the circle.
#if PBL_ROUND
#define TIMELINE_PEEK_ORIGIN_Y_VISIBLE ((DISP_ROWS * 112) / 180)
#else
#define TIMELINE_PEEK_ORIGIN_Y_VISIBLE (DISP_ROWS - TIMELINE_PEEK_HEIGHT)
#endif

#define TIMELINE_PEEK_FRAME_VISIBLE GRect(0, TIMELINE_PEEK_ORIGIN_Y_VISIBLE, DISP_COLS, \
                                          TIMELINE_PEEK_HEIGHT)

//! Gets the concurrent height needed to render for the number of concurrent events.
//! @return The concurrent height
unsigned int timeline_peek_get_concurrent_height(unsigned int num_concurrent);

//! Draws the timeline peek background.
//! @param ctx Graphics context to draw with.
//! @param frame The rectangle of the peek to draw.
//! @param num_concurrent The number of events to indicate.
void timeline_peek_draw_background(GContext *ctx, const GRect *frame,
                                   unsigned int num_concurrent);

//! Initializes a TimelinePeek overlay (transparent, unfocusable modal window)
void timeline_peek_init(void);

//! Sets whether the peek is visible. The peek will animate in or out depending if it was
//! previously visible or not.
//! @param visible Whether to show the peek
//! @param animated Whether the peek animates into its new visibility state
void timeline_peek_set_visible(bool visible, bool animated);

//! Sets the pin information to display as well as the number of concurrent events
//! @param item TimelineItem reference which is stored and expected to exist until replaced. If
//! NULL, the peek will be emptied and no event information is displayed.
//! @param started Whether the item has started or not.
//! @param num_concurrent The number of concurrent events to indicate
//! @param first Whether the item is the first event in Timeline.
//! @param animated Whether the peek animates into its new visibility state
void timeline_peek_set_item(TimelineItem *item, bool started, unsigned int num_concurrent,
                            bool first, bool animated);

//! Returns whether the item in the peek is the first event in Timeline.
//! @return true if the peek is showing the first time, false otherwise.
bool timeline_peek_is_first_event(void);

//! Returns whether Timeline future is empty upon entering it.
//! @return true if Timeline future is empty, false otherwise.
bool timeline_peek_is_future_empty(void);

//! Dismisses the current TimelinePeek Timeline item.
void timeline_peek_dismiss(void);

//! Gets the current y of the peek
int16_t timeline_peek_get_origin_y(void);

//! Gets the current obstruction y from which the unobstructed area can be derived from
int16_t timeline_peek_get_obstruction_origin_y(void);

//! Gets the current timeline item id. If there is no item, UUID_INVALID is given instead.
//! @param item_id_out Pointer to the item id buffer to write to.
void timeline_peek_get_item_id(TimelineItemId *item_id_out);

//! Pushes the TimelinePeek window
void timeline_peek_push(void);

//! Pops the TimelinePeek window
void timeline_peek_pop(void);

//! Toggles whether TimelinePeek is enabled. Used by the qemu serial protocol for the SDK.
void timeline_peek_set_enabled(bool enabled);

//! Handles timeline peek events
void timeline_peek_handle_peek_event(PebbleTimelinePeekEvent *event);

//! Handles process start synchronously. This is synchronous because the app manager needs to know
//! the new unobstructed area that would result from process start in order to prepare the app
//! state initialization parameters with the new obstruction position.
void timeline_peek_handle_process_start(void);

//! Handles process kill synchronously. This is synchronous because process start is handled
//! synchronously -- a processing being killed and another process starting happen in sequence.
void timeline_peek_handle_process_kill(void);
