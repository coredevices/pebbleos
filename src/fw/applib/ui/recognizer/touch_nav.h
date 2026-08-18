/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pan.h"
#include "recognizer.h"
#include "recognizer_manager.h"
#include "swipe.h"
#include "tap.h"

#include "applib/graphics/gtypes.h"
#include "applib/ui/layer.h"
#include "pbl/drivers/button_id.h"
#include "pbl/drivers/rtc.h"
#include "pbl/services/touch/touch_event.h"

#include <stdbool.h>
#include <stdint.h>

//! Touch-nav routing bridge (per-task).
//!
//! A single (tap, pan, swipe) system recognizer set lives by value here and is registered on the
//! task's global recognizer list. The event callback of the touch service (`touch_nav_dispatch`)
//! resolves a route once per Touchdown, latches it, and excludes tiers via `recognizer_set_failed`
//! at routing time. On completion the bridge emulates a button press. The concrete side effects
//! (window-stack animation query, back-pop, click synthesis, idle refresh) are provided by the
//! owning task through a \ref TouchNavOps vtable so the core is task-agnostic and unit-testable.

//! Height, in pixels, of the top status-bar dead zone. A Touchdown here does not drive the bridge;
//! it is routed to the window's sole Tier-1 widget, or dropped.
#define TOUCH_NAV_STATUS_BAR_DEAD_ZONE_PX (16)

//! Number of entries in the per-task observability ring buffer.
#define TOUCH_NAV_LOG_ENTRIES (16)

//! Route resolved once on Touchdown and latched for the whole gesture.
typedef enum TouchNavRoute {
  //! Nobody: all three system recognizers are failed (bridge disabled, or dead-zone drop).
  TouchNavRoute_None,
  //! A Tier-1 widget owns the gesture; the system set is failed so it does not also emulate.
  TouchNavRoute_Tier1,
  //! Tier-2 bridge: tap/swipe completions emulate buttons; pan is failed.
  TouchNavRoute_Tier2,
  //! The gesture is dropped (status-bar dead zone with no sole widget, or a gated Touchdown).
  TouchNavRoute_Dropped,
} TouchNavRoute;

//! Which Tier-1 registry a widget node belongs to; the registry head implies the type.
typedef enum TouchNavWidgetType {
  TouchNavWidgetType_Menu,
  TouchNavWidgetType_Swap,
  TouchNavWidgetType_Scroll,
} TouchNavWidgetType;

//! Polymorphic apply-logic for a migrated Tier-1 widget. The unified widget recognizer set (see
//! \ref TouchNavState) drives whichever registered widget the finger lands on entirely through this
//! vtable, so the touch-nav core (in applib/ui/recognizer/) never casts to a concrete widget type
//! and never depends on services/. `w` is the opaque widget pointer stored in the node.
typedef struct TouchNavWidgetOps {
  //! Optional readiness gate consulted before a pan Starts. NULL means always ready. When it returns
  //! false the gesture is declined for its whole lifetime (no pan_started/get_base_offset is called).
  bool (*can_start)(void *w);
  //! Pan Started: cancel any of the widget's own running animation so the finger takes over.
  void (*pan_started)(void *w);
  //! @return the widget's content offset at pan Start, latched as the base for the whole gesture.
  //! Uses \ref GPointReturn (a plain alias for GPoint) so the `GPoint (*` here does not collide with
  //! the function-like \c GPoint(x, y) constructor macro.
  GPointReturn (*get_base_offset)(void *w);
  //! Live pan: apply `base + delta` to the widget's content offset (throttled by the core).
  void (*pan_update)(void *w, GPoint base, GPoint delta);
  //! Liftoff after a pan: settle the final (unthrottled) offset. `velocity` is the liftoff
  //! velocity in px/s (see \ref pan_recognizer_get_velocity), for widgets that fling.
  void (*pan_snap)(void *w, GPoint base, GPoint final_delta, GPoint velocity);
  //! A pan cancelled mid-gesture (e.g. the widget is torn down): the core clears its latch first.
  void (*pan_cancel)(void *w);
  //! A tap completed on the widget. NULL means the widget has no tap action and the tap is dropped.
  void (*tap)(void *w, GPoint point_on_screen);
  //! A horizontal swipe completed. Called only for SwipeDirection_Left / SwipeDirection_Right.
  void (*swipe)(void *w, SwipeDirection dir);
} TouchNavWidgetOps;

//! Intrusive registry node embedded by value in a Tier-1 widget. `layer` identifies the widget for
//! the parent-walk match; `next` links the registry list. A migrated widget also supplies `ops` (its
//! apply-vtable) and `widget` (its opaque self pointer) so the unified widget set can drive it; an
//! un-migrated widget leaves `ops`/`widget` NULL and is driven by its own recognizer set instead.
typedef struct TouchNavWidgetNode {
  struct TouchNavWidgetNode *next;
  struct Layer *layer;
  const TouchNavWidgetOps *ops;
  void *widget;
} TouchNavWidgetNode;

//! Snapshot of the foreground ActionBarLayer, published from applib via a syscall on
//! add_to_window / set_icon(_animated) / remove_from_window. Read when synthesising a tap so the
//! tap is routed into the bar's UP / SELECT / DOWN zone instead of a plain SELECT.
typedef struct TouchNavActionBar {
  //! Bar geometry in GLOBAL (screen) coordinates.
  GRect frame;
  //! Bit i is set when button (i + 1) carries an icon: bit0 = UP, bit1 = SELECT, bit2 = DOWN.
  //! A tap in a zone whose icon bit is clear falls back to SELECT.
  uint8_t icon_mask;
  //! True while a bar is on the foreground window; false clears the snapshot so no stale bar routes
  //! taps after the owning window/app goes away.
  bool present;
} TouchNavActionBar;

//! Task-specific side effects for the bridge. All pointers are consulted through `ctx`. Any may be
//! NULL, in which case the effect is skipped (e.g. the app task has no idle-timeout refresh).
typedef struct TouchNavOps {
  //! @return true if the task's window stack is mid-transition.
  bool (*is_animating)(void *ctx);
  //! @return true if the top window overrides the back button.
  bool (*top_overrides_back)(void *ctx);
  //! @return true if the top window only accepts taps on action-bar icon zones.
  bool (*top_tap_requires_action_bar)(void *ctx);
  //! @return true if the top window has the touch bridge disabled.
  bool (*top_bridge_disabled)(void *ctx);
  //! Pop the top window (BACK on a window without a back handler).
  void (*pop_top)(void *ctx);
  //! Synthesize a button down+up pair for \a button on the task's click manager.
  void (*emit_button)(void *ctx, ButtonId button);
  //! Refresh the idle timeout. NULL on tasks without one.
  void (*idle_refresh)(void *ctx);
  void *ctx;
} TouchNavOps;

//! Kinds of ring-buffer log entries.
typedef enum TouchNavLogKind {
  TouchNavLog_Route,      //!< A route was latched (detail = TouchNavRoute).
  TouchNavLog_Emit,       //!< A button was emulated (detail = ButtonId).
  TouchNavLog_Dropped,    //!< A completion was dropped mid-animation (detail = ButtonId).
  TouchNavLog_Gated,      //!< A gated Touchdown (detail: 0 = dropped, 1 = wake, pans allowed).
} TouchNavLogKind;

typedef struct TouchNavLogEntry {
  uint8_t kind;
  uint8_t detail;
} TouchNavLogEntry;

//! Per-task touch-nav state. Held by value in the task's process state.
typedef struct TouchNavState {
  RecognizerManager *manager;
  const TouchNavOps *ops;

  //! The (tap, pan, swipe) system recognizer set, by value.
  _Alignas(void *) uint8_t tap_storage[TAP_RECOGNIZER_STATIC_SIZE];
  _Alignas(void *) uint8_t pan_storage[PAN_RECOGNIZER_STATIC_SIZE];
  _Alignas(void *) uint8_t swipe_storage[SWIPE_RECOGNIZER_STATIC_SIZE];
  Recognizer *tap;
  Recognizer *pan;
  Recognizer *swipe;

  //! The unified widget (tap, pan, swipe) recognizer set, by value. One set per twin drives every
  //! migrated Tier-1 widget through its \ref TouchNavWidgetOps vtable; the set is scoped by a filter
  //! that only matches a Touchdown resolving to a migrated (ops-bearing) widget.
  _Alignas(void *) uint8_t widget_tap_storage[TAP_RECOGNIZER_STATIC_SIZE];
  _Alignas(void *) uint8_t widget_pan_storage[PAN_RECOGNIZER_STATIC_SIZE];
  _Alignas(void *) uint8_t widget_swipe_storage[SWIPE_RECOGNIZER_STATIC_SIZE];
  Recognizer *widget_tap;
  Recognizer *widget_pan;
  Recognizer *widget_swipe;

  //! Tier-1 widget registry heads; the type is implied by which head a node hangs off.
  TouchNavWidgetNode *menu_head;
  TouchNavWidgetNode *swap_head;
  TouchNavWidgetNode *scroll_head;

  //! Per-gesture state for the unified widget set. `latched_target` is the migrated widget node the
  //! gesture is bound to (resolved on Touchdown, re-validated as a weak ref on every later event);
  //! `gesture_base` is the widget content offset latched on pan Start; `last_update_ticks` throttles
  //! live pan updates; `declined` is set when `can_start` refused the gesture, gating every later
  //! vtable call until the gesture ends.
  TouchNavWidgetNode *latched_target;
  GPoint gesture_base;
  RtcTicks last_update_ticks;
  bool declined;

  //! Route latched on the most recent Touchdown.
  TouchNavRoute route;

  //! Latest foreground ActionBarLayer snapshot; consulted when synthesising a tap.
  TouchNavActionBar action_bar;

  //! Observability counters.
  struct {
    uint16_t started;
    uint16_t completed;
    uint16_t failed;
    uint16_t cancelled;
    uint16_t dropped;
    uint16_t gated;
  } counters;

  //! Observability ring buffer.
  TouchNavLogEntry log[TOUCH_NAV_LOG_ENTRIES];
  uint8_t log_head;   //!< Index of the next slot to write.
  uint8_t log_count;  //!< Number of valid entries (saturates at TOUCH_NAV_LOG_ENTRIES).
} TouchNavState;

//! Initialize the per-task touch-nav state: build the system recognizer set into embedded storage,
//! register it on \a manager's global list, and zero the registry/counters. \a ops must outlive the
//! state.
void touch_nav_state_init(TouchNavState *state, RecognizerManager *manager, const TouchNavOps *ops);

//! Tear the state down: cancel any in-flight gesture and deregister the system recognizer set from
//! the manager's global list. No client callbacks are invoked for a widget-only rebuild.
void touch_nav_state_deinit(TouchNavState *state);

//! Touch-service system-slot handler. Conforms to the touch service handler prototype
//! (\ref TouchServiceHandler). \a context is the \ref TouchNavState.
void touch_nav_dispatch(const TouchEvent *touch_event, void *context);

//! Store the foreground ActionBarLayer snapshot into \a state. A NULL \a frame clears the snapshot
//! (bar removed / no window); otherwise \a frame is the bar's global-coordinate rectangle and
//! \a icon_mask its per-zone icon presence bits. Called from the \ref sys_touch_set_action_bar
//! syscall handler on the task that owns the bar.
void touch_nav_set_action_bar(TouchNavState *state, const GRect *frame, uint8_t icon_mask);

//! Resolve a tap at \a point against the action-bar snapshot \a bar. Returns the zoned button when
//! the bar is present and the point is inside its frame: the frame is split vertically into three
//! equal zones (top = UP, middle = SELECT, bottom = DOWN). A zone whose icon bit is clear, a point
//! outside the frame, or an absent snapshot all fall back to \ref BUTTON_ID_SELECT -- unless
//! \a require_icon_zone is set (the top window's touch_tap_requires_action_bar exception), in which
//! case every fallback returns NUM_BUTTONS so no button is synthesized. Swipes are not zoned; only
//! taps consult this.
ButtonId touch_nav_action_bar_zone_button(const TouchNavActionBar *bar, GPoint point,
                                          bool require_icon_zone);

//! Register a Tier-1 widget node under the given registry. Dedup-by-address (a re-add WARNs and is
//! a no-op). Robust to a node zeroed by the widget's *_init. \a ops and \a widget are the migrated
//! widget's apply-vtable and opaque self pointer (both NULL for an un-migrated widget still driven
//! by its own recognizer set); they are re-applied on a re-add so an init-without-deinit keeps
//! routing to the widget.
void touch_nav_registry_add(TouchNavState *state, TouchNavWidgetType type, TouchNavWidgetNode *node,
                            struct Layer *layer, const TouchNavWidgetOps *ops, void *widget);

//! Remove a Tier-1 widget node from its registry by predecessor traversal. A node that is not
//! present (e.g. zeroed by *_init and never added) is a safe no-op.
void touch_nav_registry_remove(TouchNavState *state, TouchNavWidgetType type,
                               TouchNavWidgetNode *node);

//! Effects of the master-pref enable/disable transaction. The ordering is mandatory and lives in
//! \ref touch_nav_transaction_apply; the owning shell provides the concrete effects (persist,
//! kernel/app subscription juggling, permanent sensor hold). Split out so the ordering is unit
//! testable independently of the kernel.
typedef struct TouchNavTxnOps {
  //! Persist the new pref value (1) and flip the master nav gate (touch_set_nav_enabled).
  void (*persist)(void *ctx, bool enable);
  //! Enable (2): on KernelMain, subscribe the kernel touch slot.
  void (*kernel_subscribe)(void *ctx);
  //! Enable (2): take the permanent sensor hold (touch_set_system_hold(true)).
  void (*take_system_hold)(void *ctx);
  //! Disable (2): synthesize a Liftoff if a finger is down (backlight+driver only).
  void (*synthesize_liftoff)(void *ctx);
  //! Disable (3): cancel_and_reset the kernel manager and unsubscribe the kernel slot.
  void (*kernel_cancel_reset_unsub)(void *ctx);
  //! Disable (4): send the app-task callback that cancel_and_resets the app manager and drops its
  //! subscription.
  void (*app_unsubscribe)(void *ctx);
  //! Disable (5): release the permanent sensor hold (touch_set_system_hold(false)).
  void (*release_system_hold)(void *ctx);
  void *ctx;
} TouchNavTxnOps;

//! Apply the master-pref transaction in the mandated order. On enable: persist+gate, then (on
//! KernelMain) subscribe the kernel slot and take the sensor hold. On disable: persist+gate, then
//! synthesize a Liftoff, cancel_and_reset+unsubscribe the kernel manager, send the app-unsubscribe
//! callback, and finally release the sensor hold.
void touch_nav_transaction_apply(const TouchNavTxnOps *ops, bool enable);

//! Gate for the app-task touch-nav twin. A participating app (system apps participate by default)
//! rides the SYSTEM nav state (\a system_nav_enabled: master pref AND the Touch Navigation
//! sub-pref), while an app that explicitly opted in via \ref app_touch_navigation_enable
//! (\a opted_in) follows the master pref alone (\a master_nav_enabled) -- turning the system
//! navigation off does not turn off an app's own gestures. A third-party app that never opted in
//! stays inert under every pref combination. Factored here (out of app_state.c) so the gate is
//! unit-testable without the kernel app-state singleton.
bool touch_nav_app_twin_active(bool system_nav_enabled, bool master_nav_enabled,
                               bool participating, bool opted_in);

//! Gate for the app-task Tier-2 bridge (feeds the \ref TouchNavOps top_bridge_disabled op). Reports
//! the bridge as disabled -- routing the gesture to \ref TouchNavRoute_None so NO button is
//! synthesized -- when the top window opted out (\a window_opt_out) OR the app installed its own
//! raw touch subscriber (\a app_has_raw_subscriber, via touch_service_subscribe). An app that
//! handles raw touch itself must never also receive synthesized button clicks: that would change
//! existing runtime behavior for a third-party app. Factored here (out of app_state.c) so it is
//! unit-testable without the kernel app-state singleton.
bool touch_nav_app_bridge_disabled(bool window_opt_out, bool app_has_raw_subscriber);

//! Concrete effects of the app-task touch-nav twin subscription. The owning task provides these so
//! the subscribe/reconcile state machine is unit-testable independently of the kernel app-state
//! singleton (mirrors \ref TouchNavTxnOps). Any op is consulted through \a ctx.
typedef struct TouchNavTwinOps {
  //! @return the effective SYSTEM nav state (touch_nav_enabled()).
  bool (*pref_enabled)(void *ctx);
  //! @return the master "Touch" switch alone (touch_service_is_globally_enabled(): the global
  //! touch kill). NULL reads as off, so an opted-in app cannot activate without the op wired.
  bool (*master_enabled)(void *ctx);
  //! Install the system touch handler for this task's nav twin (subscribe).
  void (*install_handler)(void *ctx);
  //! Cancel any in-flight gesture and clear the system touch handler (unsubscribe). Safe to call
  //! even when nothing is installed.
  void (*remove_handler)(void *ctx);
  void *ctx;
} TouchNavTwinOps;

//! Install the app twin's touch handler iff \ref touch_nav_app_twin_active says so. A no-op
//! otherwise. Safe to call repeatedly.
void touch_nav_app_twin_subscribe(const TouchNavTwinOps *ops, bool participating, bool opted_in);

//! Reconcile the twin with a new opt-in value (the app_touch_navigation_enable path). The flags
//! point at the caller's stored state; both are set to \a enable. Idempotent: when nothing changes
//! this is a no-op (no double-subscribe, no spurious unsubscribe). On enabling it subscribes
//! (installing only when the gate passes); on disabling it removes the handler.
void touch_nav_app_twin_reconcile(const TouchNavTwinOps *ops, bool *participating, bool *opted_in,
                                  bool enable);

//! Re-evaluate the gate for the RUNNING app after a pref flip and install or remove the handler
//! accordingly. Unlike the disable transaction's old unconditional unsubscribe, this keeps an
//! opted-in app subscribed when only the Touch Navigation sub-pref turned off.
void touch_nav_app_twin_resync(const TouchNavTwinOps *ops, bool participating, bool opted_in);
