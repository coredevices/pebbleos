/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "applib/ui/layer.h"
#include "applib/ui/click.h"
#include "applib/ui/property_animation.h"
#include "pbl/services/timeline/layout_layer.h"

struct Window;
struct SwapLayer;

//! Function signature for the `.get_layout_handler` callback.
typedef LayoutLayer* (*SwapLayerGetLayoutHandler)(struct SwapLayer *swap_layer, int8_t rel_position,
    void *context);

//! Function signature for the `.layout_removed_handler` callback.
typedef void (*SwapLayerLayoutRemovedHandler)(struct SwapLayer *swap_layer, LayoutLayer *layer,
    void *context);

//! Function signature for the `.layout_did_appear_handler` callback.
typedef void (*SwapLayerLayoutDidAppearHandler)(struct SwapLayer *swap_layer, LayoutLayer *layer,
    int8_t rel_change, void *context);

//! Function signature for the `.layout_will_appear_handler` callback.
typedef void (*SwapLayerLayoutWillAppearHandler)(struct SwapLayer *swap_layer, LayoutLayer *layer,
    void *context);

//! Function signature for the `.update_colors_handler` callback.
typedef void (*SwapLayerUpdateColorsHandler)(struct SwapLayer *swap_layer, GColor bg_color,
    bool status_bar_filled, void *context);

//! Function signature for the `.interaction_handler` callback.
typedef void (*SwapLayerInteractionHandler)(struct SwapLayer *swap_layer, void *context);

//! All the callbacks that the SwapLayer exposes for use by applications.
//! @note The context parameter can be set using swap_layer_set_context() and
//! gets passed in as context with all of these callbacks.
typedef struct {
  SwapLayerGetLayoutHandler get_layout_handler;
  SwapLayerLayoutRemovedHandler layout_removed_handler;
  SwapLayerLayoutDidAppearHandler layout_did_appear_handler;
  SwapLayerLayoutWillAppearHandler layout_will_appear_handler;
  SwapLayerUpdateColorsHandler update_colors_handler;
  SwapLayerInteractionHandler interaction_handler;
  ClickConfigProvider click_config_provider;
} SwapLayerCallbacks;

typedef struct {
  Layer layer;
  GBitmap arrow_bitmap;
} ArrowLayer;

//! Data structure of a SwapLayer
//! @note a `SwapLayer *` can safely be casted to a `Layer *` and can thus be
//! used with all other functions that take a `Layer *` as an argument.
//! <br/>For example, the following is legal:
//! \code{.c}
//! SwapLayer swap_layer;
//! ...
//! layer_set_hidden((Layer *)&swap_layer, true);
//! \endcode
//! @note However, there are a few caveats:
//! * To add content layers, you must use \c swap_layer_add_child().
//! * To change the frame of a scroll layer, use \c swap_layer_set_frame().
typedef struct SwapLayer {
  Layer layer;
  ArrowLayer arrow_layer;
  Animation *animation;
  LayoutLayer *previous; //!< Previous LayoutLayer in the list.
  LayoutLayer *current; //!< Current LayoutLayer in the list.
  LayoutLayer *next; //!< Next LayoutLayer in the list.
  SwapLayerCallbacks callbacks;
  uint16_t swap_delay_remaining;
  bool swap_in_progress;
  bool is_deiniting;
  void *context;
#ifdef CONFIG_TOUCH
  //! @internal
  //! Intrusive Tier-1 touch-navigation registry node. Layout-compatible with \c TouchNavWidgetNode
  //! (four pointers — \c next, \c layer, \c ops, \c widget); a build-time assert in swap_layer.c
  //! keeps it in sync. Declared as opaque pointers so this timeline header does not pull in the
  //! recognizer stack.
  struct {
    void *next;
    void *layer;
    void *ops;
    void *widget;
  } touch_nav_node;
  //! True while this SwapLayer is threaded onto the Tier-1 registry (guards double add/remove and
  //! makes the init/focus re-registration idempotent).
  bool touch_registered;
#endif
} SwapLayer;

//! Init. Contains no layouts at this point.
void swap_layer_init(SwapLayer *swap_layer, const GRect *frame);

//! Deinits a SwapLayer and will call the .layout_removed_handler for all layers currently being
//! tracked by the SwapLayer
void swap_layer_deinit(SwapLayer *swap_layer);

//! Calls the .layout_removed_handler for each layout currently known by the SwapLayer,
//! then fetches the "current" and "next" layouts.
//! The callbacks "layout_will_appear" and "layout_did_appear" will both be called.
void swap_layer_reload_data(SwapLayer *swap_layer);

//! Returns the currently focused LayoutLayer of the SwapLayer.
LayoutLayer *swap_layer_get_current_layout(const SwapLayer *swap_layer);

Layer *swap_layer_get_layer(const SwapLayer *swap_layer);

void swap_layer_set_callbacks(SwapLayer *swap_layer, void *callback_context,
                              SwapLayerCallbacks callbacks);

void swap_layer_set_click_config_onto_window(SwapLayer *swap_layer, struct Window *window);

//! Will attempt to swap layers in the given "direction".
//! This will fail if there are no layouts to swap to. The client will know if it succeeded by
//! whether it got a "layout_will_appear" and "layout_did_appear" event.
//! Returns whether the swap attempt was successful.
bool swap_layer_attempt_layer_swap(SwapLayer *swap_layer, ScrollDirection direction);

#ifdef CONFIG_TOUCH
//! Directly offset the current notification frame by \a dy pixels of \c origin.y WITHOUT an
//! animation (unlike \c prv_scroll, which always animates), clamped so the resulting scroll offset
//! stays within [0, max_scroll]. A positive \a dy scrolls the content towards the top of the
//! notification (offset decreases); a negative \a dy scrolls further into the content (offset
//! increases). The \c next layout is pulled right under the current one so its peek tracks the
//! finger. Also refreshes the auto-close interaction timer so a long notification cannot close under
//! the finger mid-read. Used by the Tier-1 touch pan for live 1:1 scrolling.
void swap_layer_touch_scroll_by(SwapLayer *swap_layer, int16_t dy);

//! Release this SwapLayer's Tier-1 touch participation while it is covered by a higher modal: remove
//! it from the touch-navigation registry (which detaches the shared recognizer set and clears the
//! gesture target) so touch cannot leak into the now-hidden notification body. In our system-slot
//! bridge architecture registry membership is what routes touch, so deregistering is the release.
//! Idempotent and safe when touch is disabled. Re-registered by the click-config-provider on re-show.
void swap_layer_touch_release(SwapLayer *swap_layer);

//! @internal Test seam: zero the per-task Tier-1 gesture singletons for cross-test isolation.
void swap_layer_touch_nav_reset_all(void);

//! @internal Test seam: whether \a swap_layer is the current per-task gesture target.
bool swap_layer_touch_is_gesture_target(const SwapLayer *swap_layer);

//! @internal Test seam: register/deregister this SwapLayer as a Tier-1 touch widget. Normally driven
//! by swap_layer_init / the click-config-provider / swap_layer_deinit; exposed for unit tests.
void swap_layer_touch_register(SwapLayer *swap_layer);
void swap_layer_touch_deregister(SwapLayer *swap_layer);

//! @internal Test seam: the liftoff decision for a pan that started at \a base_offset and moved
//! \a delta_y (finger travel; negative is up). Encodes the two-threshold logic (DRAG_THRESHOLD_PX
//! and SWAP_OVERPULL_PX) independently of the animation/swap machinery.
typedef enum SwapTouchLiftoffAction {
  SwapTouchLiftoff_None,      //!< Sub-threshold drag: do nothing.
  SwapTouchLiftoff_SwapPrev,  //!< Over-pull past the top: swap to the previous notification.
  SwapTouchLiftoff_SwapNext,  //!< Over-pull past the bottom: swap to the next notification.
  SwapTouchLiftoff_Settle,    //!< A normal scroll: settle to the clamped offset.
} SwapTouchLiftoffAction;

SwapTouchLiftoffAction swap_layer_touch_liftoff_action(int16_t base_offset, int16_t delta_y,
                                                       int16_t max_dy);

//! @internal Test seam: the offset a Settle liftoff (or a cancelled pan) animates to. The raw
//! target is clamp(base_offset - delta_y, 0, max_dy); with \a page_h > 0 it is then quantized to
//! the nearest page boundary, never past the last whole page below \a max_dy (which may include
//! the next-notification peek). Round displays pass LAYOUT_HEIGHT — the circular text flow is
//! computed for page-aligned rest positions — while rect passes 0 and settles freely.
int16_t swap_layer_touch_settle_offset(int16_t base_offset, int16_t delta_y, int16_t max_dy,
                                       int16_t page_h);
#endif  // CONFIG_TOUCH
