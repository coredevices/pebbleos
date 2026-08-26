/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "weather_report.h"
#include "weather_math.h"
#include "forecast_list.h"   // arm_hslide_in — the report-BACK glide pair
#include "applib/app_timer.h"
#include "applib/graphics/gdraw_command_transforms.h"
#include "applib/ui/animation.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/ui/app_window_stack.h"
#include "pebble_compat.h"

// "The Weather Report" — the SELECT screen.
//
// RECT (emery/obelix): the classic condensed day view, revived — the original
// weather app's layout engine hosts the page (day label, hero temp, hi/lo,
// condition phrase, rain/wind, right-railed icon discs, dotted rule, next-day
// block, chevron, arc day-flip, fin). Same host shape as the round arm.
//
// ROUND (gabbro/getafix): the classic condensed layout host — hold UP/DOWN
// scrolls the week through weather_app_layout.

// Hard-cut latch: the unfold transition scene already delivered the entrance, so the
// next push renders the full page on frame one (only the rect arm ever arms this).
static bool s_pending_static_in;

void weather_report_arm_static_in(void) {
  s_pending_static_in = true;
}

#if PBL_ROUND
// ============================================================================
// ROUND — classic condensed layout host (weather_app_layout), unchanged.
// ============================================================================

#include "weather_app_layout.h"

typedef struct {
  Window *window;
  WeatherAppLayout layout;
  const WeatherLocationForecast *days;   // borrowed; owned by weather.c (app-lifetime)
  size_t num_days;
  int    current_day_index;
#ifdef CONFIG_TOUCH
  int16_t touch_start_x, touch_start_y;  // Touchdown origin for swipe detection
  bool    touch_active;
#endif
} WeatherReportData;

static WeatherReportData *s_report;

// today = focused day's large icon; next = the day below (small icon). fin marker shows on the
// last day. Pointers are borrowed straight from days[] (the layout stores them without copying).
static void prv_seed_day(int i) {
  const WeatherLocationForecast *today = &s_report->days[i];
  const WeatherLocationForecast *next  =
      (i + 1 < (int)s_report->num_days) ? &s_report->days[i + 1] : NULL;
  weather_app_layout_set_fin_allowed(&s_report->layout,
                                     s_report->num_days > 1 && i + 1 >= (int)s_report->num_days);
  weather_app_layout_set_data(&s_report->layout, today, next);
  weather_app_layout_set_down_arrow_visible(&s_report->layout, i + 1 < (int)s_report->num_days);
  if (today->location_name) {
    weather_app_layout_set_location(&s_report->layout, today->location_name);
  }
}

// Step one day and play the layout's 220ms arc scroll. Bounded — no wrap. Rapid calls (from the
// hold-scroll timer) are fine: weather_app_layout_animate cancels the in-flight arc and restarts,
// so an accelerating hold fast-forwards through the days.
static void prv_navigate(bool is_down) {
  if (!s_report) return;
  int i = s_report->current_day_index;
  if (is_down) {
    if (i + 1 >= (int)s_report->num_days) return;   // already at the last day
    i++;
  } else {
    if (i <= 0) return;                              // already at today
    i--;
  }
  s_report->current_day_index = i;
  const WeatherLocationForecast *new_today = &s_report->days[i];
  const WeatherLocationForecast *new_next  =
      (i + 1 < (int)s_report->num_days) ? &s_report->days[i + 1] : NULL;
  weather_app_layout_set_fin_allowed(&s_report->layout,
                                     s_report->num_days > 1 && i + 1 >= (int)s_report->num_days);
  weather_app_layout_animate(&s_report->layout, new_today, new_next, is_down);
}

static void prv_click_up_down(ClickRecognizerRef r, void *ctx) {
  if (!s_report) return;
  prv_navigate(click_recognizer_get_button_id(r) == BUTTON_ID_DOWN);
}

static Animation *s_slide_in_anim;   // entrance slide (defined below); shared exclusivity
// BACK: the page glide pair (globe grammar): this page slides out RIGHT while the mainscreen
// slides in from the left (forecast_list_arm_hslide_in). Same WEATHER_HSLIDE_MS + moook_soft1.
static Animation *s_back_slide_anim;

// System-app statics persist across launches (the fw image is not reloaded), and a
// crashed run never reaches unload — clear them before the next run reads them.
void weather_report_reset(void) {
  s_report = NULL;
  s_slide_in_anim = NULL;
  s_back_slide_anim = NULL;
  s_pending_static_in = false;
}

static void prv_back_slide_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)context;
  s_back_slide_anim = NULL;   // property animation auto-destroys after a normal stop
  if (finished) {
    forecast_list_arm_hslide_in();
    window_stack_pop(false);   // un-animated: the glide already happened
  }
}
static void prv_start_back_slide(void) {
  if (!s_report || s_back_slide_anim || s_slide_in_anim) return;   // entrance/exit exclusive
  Layer *root = s_report->layout.root_layer;
  GRect from = layer_get_frame_by_value(root);
  GRect to = from;
  to.origin.x += from.size.w;                     // out to the RIGHT
  PropertyAnimation *pa = property_animation_create_layer_frame(root, &from, &to);
  if (!pa) { window_stack_pop(true); return; }
  Animation *a = (Animation *)pa;
  animation_set_duration(a, WEATHER_HSLIDE_MS);
  animation_set_custom_interpolation(a, weather_interpolate_moook_soft1);
  animation_set_handlers(a, (AnimationHandlers){ .stopped = prv_back_slide_stopped }, NULL);
  s_back_slide_anim = a;
  animation_schedule(a);
}
static void prv_click_back(ClickRecognizerRef r, void *ctx) {
  prv_start_back_slide();   // glide out right; pops in the .stopped
}

// ---- Touch input (touch colour platforms) ----
#ifdef CONFIG_TOUCH
#define SWIPE_THRESHOLD 20   // px; same tap/swipe split as the other screens

static void prv_touch_handler(const TouchEvent *event, void *context) {
  (void)context;
  if (!s_report) return;
  if (event->type == TouchEvent_Touchdown) {
    s_report->touch_start_x = event->x;
    s_report->touch_start_y = event->y;
    s_report->touch_active  = true;
  } else if (event->type == TouchEvent_Liftoff && s_report->touch_active) {
    s_report->touch_active = false;
    int16_t dx = event->x - s_report->touch_start_x;
    int16_t dy = event->y - s_report->touch_start_y;
    int16_t adx = dx < 0 ? -dx : dx;
    int16_t ady = dy < 0 ? -dy : dy;
    if (adx <= SWIPE_THRESHOLD && ady <= SWIPE_THRESHOLD) return;  // tap — no action
    if (ady >= adx) {
      prv_navigate(dy < 0);      // swipe up = next day (like DOWN), swipe down = previous
    } else if (dx > 0) {
      prv_start_back_slide();    // swipe right = BACK, same glide
    }
  }
}
#endif

// ---- Hold-to-scroll with acceleration (raw click subscriber + AppTimer) ----
// Ported verbatim from the original main screen (the original condensed build's weather.c): a single tap steps one
// day; holding UP/DOWN kicks off a timer that steps repeatedly, shrinking the interval by 50ms each
// step (from 300ms down to a 90ms floor) so the scroll speeds up the longer it is held.
#define HOLD_INITIAL_MS  300
#define HOLD_MIN_MS       90

static AppTimer *s_hold_timer   = NULL;
static bool      s_hold_is_down = false;
static int       s_hold_repeat  = 0;

static void prv_hold_timer_cb(void *ctx) {
  s_hold_timer = NULL;
  if (!s_report) return;
  prv_navigate(s_hold_is_down);
  s_hold_repeat++;
  int interval = HOLD_INITIAL_MS - s_hold_repeat * 50;
  if (interval < HOLD_MIN_MS) interval = HOLD_MIN_MS;
  s_hold_timer = app_timer_register(interval, prv_hold_timer_cb, NULL);
}

static void prv_raw_up_down(ButtonId btn, bool pressed) {
  if (!s_report) return;
  if (pressed) {
    s_hold_is_down = (btn == BUTTON_ID_DOWN);
    s_hold_repeat  = 0;
    if (s_hold_timer) { app_timer_cancel(s_hold_timer); s_hold_timer = NULL; }
    s_hold_timer = app_timer_register(HOLD_INITIAL_MS, prv_hold_timer_cb, NULL);
  } else {
    if (s_hold_timer) { app_timer_cancel(s_hold_timer); s_hold_timer = NULL; }
  }
}

static void prv_raw_up_pressed(ClickRecognizerRef r, void *ctx)   { prv_raw_up_down(BUTTON_ID_UP,   true);  }
static void prv_raw_up_released(ClickRecognizerRef r, void *ctx)  { prv_raw_up_down(BUTTON_ID_UP,   false); }
static void prv_raw_down_pressed(ClickRecognizerRef r, void *ctx) { prv_raw_up_down(BUTTON_ID_DOWN, true);  }
static void prv_raw_down_released(ClickRecognizerRef r, void *ctx){ prv_raw_up_down(BUTTON_ID_DOWN, false); }

static void prv_click_provider(void *ctx) {
  // Single tap = one day; hold UP/DOWN = accelerating scroll (raw handlers drive the hold timer).
  window_single_click_subscribe(BUTTON_ID_UP,   prv_click_up_down);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_click_up_down);
  window_raw_click_subscribe(BUTTON_ID_UP,   prv_raw_up_pressed,   prv_raw_up_released,   NULL);
  window_raw_click_subscribe(BUTTON_ID_DOWN, prv_raw_down_pressed, prv_raw_down_released, NULL);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_click_back);
}

// The location bar's HH:MM needs a once-a-minute tick. Never overlaps clock_face's
// subscription — nothing stacks on top of this window.
static void prv_minute_tick(struct tm *tick_time, TimeUnits units_changed) {
  (void)tick_time; (void)units_changed;
  if (s_report) layer_mark_dirty(s_report->layout.root_layer);
}

static void prv_window_load(Window *window) {
  if (!s_report) return;
  GRect bounds = layer_get_bounds(window_get_root_layer(window));
  weather_app_layout_init(&s_report->layout, &bounds);
  layer_add_child(window_get_root_layer(window), s_report->layout.root_layer);
  prv_seed_day(s_report->current_day_index);
  tick_timer_service_subscribe(MINUTE_UNIT, prv_minute_tick);
}

// ---- Entrance: the scene's finale. The whole page slides in from the RIGHT on the FULL
// Timeline moook — the exact grammar of Timeline's next-day text intro after the
// smiley/day-separator animation (anticipation, fast travel, overshoot past rest, settle).
// A rigid translate, deliberately NOT a jelly squash: Timeline's text intro doesn't deform,
// and the squash grammar has already had its say in the four stages that precede this
// (mainscreen squash-left -> ball -> newspaper unfold -> the paper's squash-left bow).
// Restored: this machinery was once deleted as "unreachable",
// which was true only while arm_static_in was rect-only. weather.c now arms it on BOTH
// shapes, so round was landing on a hard cut. Mirrors the rect arm line for line.

static void prv_slide_in_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_report) return;
  Layer *root = s_report->layout.root_layer;
  const int w = layer_get_bounds(root).size.w;
  GRect f = layer_get_frame_by_value(root);
  f.origin.x = (int)interpolate_moook(progress, w, 0);   // slide in from the right (+w -> 0)
  layer_set_frame(root, f);
}

static void prv_slide_in_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)finished; (void)context;
  s_slide_in_anim = NULL;
  if (s_report) {
    Layer *root = s_report->layout.root_layer;
    GRect f = layer_get_frame_by_value(root);
    f.origin.x = 0;
    layer_set_frame(root, f);
    layer_mark_dirty(root);
  }
  animation_destroy(anim);
}

static const AnimationImplementation s_slide_in_impl = { .update = prv_slide_in_update };

static void prv_start_slide_in(void) {
  if (!s_report || s_slide_in_anim) return;
  Layer *root = s_report->layout.root_layer;
  const int w = layer_get_bounds(root).size.w;
  GRect f = layer_get_frame_by_value(root);
  f.origin.x = w;                 // start the whole page off the right edge
  layer_set_frame(root, f);
  s_slide_in_anim = animation_create();
  animation_set_implementation(s_slide_in_anim, &s_slide_in_impl);
  animation_set_duration(s_slide_in_anim, interpolate_moook_duration());   // full moook —
                                        // Timeline's day-text intro length, bounce included
  animation_set_curve(s_slide_in_anim, AnimationCurveLinear);
  animation_set_handlers(s_slide_in_anim,
                         (AnimationHandlers){ .stopped = prv_slide_in_stopped }, NULL);
  animation_schedule(s_slide_in_anim);
}

static void prv_window_appear(Window *window) {
#ifdef CONFIG_TOUCH
  if (s_report) touch_service_subscribe(prv_touch_handler, s_report);
#endif
  if (s_report && s_pending_static_in) {
    s_pending_static_in = false;
    prv_start_slide_in();     // the scene's finale: Timeline moook slide from the right
  }
}

static void prv_window_unload(Window *window) {
#ifdef CONFIG_TOUCH
  touch_service_unsubscribe();
#endif
  tick_timer_service_unsubscribe();
  if (s_hold_timer) { app_timer_cancel(s_hold_timer); s_hold_timer = NULL; }
  // Module-level animations: null-first, then unschedule (the synchronous .stopped
  // handlers see nulled handles and only destroy) — so a re-push can't find stale
  // handles or animate the next instance's layers. Mirrors the rect arm.
  if (s_slide_in_anim) {
    Animation *a = s_slide_in_anim;
    s_slide_in_anim = NULL;
    animation_unschedule(a);
  }
  if (s_back_slide_anim) {
    Animation *a = s_back_slide_anim;
    s_back_slide_anim = NULL;
    animation_unschedule(a);
  }
  if (s_report) {
    weather_app_layout_deinit(&s_report->layout);   // cancels glow/icon/fin timers, frees layers+bitmaps
  }
  window_destroy(window);
  if (s_report) {
    free(s_report);
    s_report = NULL;
  }
}

void weather_report_push(const WeatherLocationForecast *days, size_t num_days, int start_day_index) {
  // Consume the entrance latch up front so NO early return (guard, alloc, window failure)
  // can leave it armed for an unrelated future push; re-armed below just before the push.
  // static_in also suppresses the compositor's own push animation, so the moook slide is
  // the ONLY motion (two competing slides read as a stutter).
  const bool static_in = s_pending_static_in;
  s_pending_static_in = false;
  if (s_report || !days || num_days == 0) return;
  s_report = calloc(1, sizeof(WeatherReportData));
  if (!s_report) return;
  s_report->days = days;
  s_report->num_days = num_days;
  s_report->current_day_index =
      (start_day_index >= 0 && start_day_index < (int)num_days) ? start_day_index : 0;
  s_report->window = window_create();
  if (!s_report->window) {
    free(s_report);
    s_report = NULL;
    return;
  }
  window_set_background_color(s_report->window, system_theme_get_bg_color());
  window_set_window_handlers(s_report->window, (WindowHandlers){
    .load   = prv_window_load,
    .appear = prv_window_appear,
    .unload = prv_window_unload,
  });
  window_set_click_config_provider(s_report->window, prv_click_provider);
  s_pending_static_in = static_in;   // re-arm for prv_window_appear (-> the moook slide-in)
  window_stack_push(s_report->window, !static_in);
}

bool weather_report_is_showing(void) {
  return s_report && s_report->window;
}

// Re-point the borrowed days array after a weather-event refresh (the array is rewritten in
// place and can SHRINK, e.g. a location change to a v3-only record) — without this the view
// keeps navigating over the previous city's stale day slots. No-op when not showing.
void weather_report_update_data(const WeatherLocationForecast *days, size_t num_days) {
  if (!s_report || !days || num_days == 0) return;
  s_report->days = days;
  s_report->num_days = num_days;
  if (s_report->current_day_index >= (int)num_days) {
    s_report->current_day_index = (int)num_days - 1;
  }
  if (s_report->layout.root_layer) {   // update_data-before-load guard
    prv_seed_day(s_report->current_day_index);   // re-seed: may be a different city now
  }
}

#else  // !PBL_ROUND
// ============================================================================
// RECT — "The Weather Report": the classic condensed day view, revived.
// The original weather app's layout engine
// (weather_app_layout — original condensed version, styled to the original Pebble
// weather app) draws the page: day label + hero temp + "hi° / lo°" + condition
// phrase + rain/wind row down the left rail, the day's icon on its colored
// disc to the right, fine dotted rule, next-day block below, bottom chevron,
// the arc day-flip, and the fin at the week's end. This host mirrors the
// round arm, plus the SELECT-scene squash-in entrance.
// ============================================================================

#include "weather_app_layout.h"

typedef struct {
  Window *window;
  WeatherAppLayout layout;
  const WeatherLocationForecast *days;   // borrowed; owned by weather.c (app-lifetime)
  size_t num_days;
  int    current_day_index;
#ifdef CONFIG_TOUCH
  int16_t touch_start_x, touch_start_y;  // Touchdown origin for swipe detection
  bool    touch_active;
#endif
} WeatherReportData;

static WeatherReportData *s_report;

static Animation *s_slide_in_anim;

// System-app statics persist across launches (the fw image is not reloaded), and a
// crashed run never reaches unload — clear them before the next run reads them.
void weather_report_reset(void) {
  s_report = NULL;
  s_slide_in_anim = NULL;
  s_pending_static_in = false;
}

// today = focused day's large icon; next = the day below (small icon). fin marker shows on the
// last day. Pointers are borrowed straight from days[] (the layout stores them without copying).
static void prv_seed_day(int i) {
  const WeatherLocationForecast *today = &s_report->days[i];
  const WeatherLocationForecast *next  =
      (i + 1 < (int)s_report->num_days) ? &s_report->days[i + 1] : NULL;
  weather_app_layout_set_fin_allowed(&s_report->layout,
                                     s_report->num_days > 1 && i + 1 >= (int)s_report->num_days);
  weather_app_layout_set_data(&s_report->layout, today, next);
  weather_app_layout_set_down_arrow_visible(&s_report->layout, i + 1 < (int)s_report->num_days);
  if (today->location_name) {
    weather_app_layout_set_location(&s_report->layout, today->location_name);
  }
}

// Step one day and play the layout's 220ms arc scroll. Bounded — no wrap. Rapid calls (from the
// hold-scroll timer) are fine: weather_app_layout_animate cancels the in-flight arc and restarts,
// so an accelerating hold fast-forwards through the days.
static void prv_navigate(bool is_down) {
  if (!s_report || s_slide_in_anim) return;   // entrance gate
  int i = s_report->current_day_index;
  if (is_down) {
    if (i + 1 >= (int)s_report->num_days) return;   // already at the last day
    i++;
  } else {
    if (i <= 0) return;                              // already at today
    i--;
  }
  s_report->current_day_index = i;
  const WeatherLocationForecast *new_today = &s_report->days[i];
  const WeatherLocationForecast *new_next  =
      (i + 1 < (int)s_report->num_days) ? &s_report->days[i + 1] : NULL;
  weather_app_layout_set_fin_allowed(&s_report->layout,
                                     s_report->num_days > 1 && i + 1 >= (int)s_report->num_days);
  weather_app_layout_animate(&s_report->layout, new_today, new_next, is_down);
}

static void prv_click_up_down(ClickRecognizerRef r, void *ctx) {
  if (!s_report) return;
  prv_navigate(click_recognizer_get_button_id(r) == BUTTON_ID_DOWN);
}

static void prv_click_back(ClickRecognizerRef r, void *ctx) {
  window_stack_pop(true);   // back to the forecast list
}

// ---- Touch input (touch colour platforms) ----
#ifdef CONFIG_TOUCH
#define SWIPE_THRESHOLD 20   // px; same tap/swipe split as the other screens

static void prv_touch_handler(const TouchEvent *event, void *context) {
  (void)context;
  if (!s_report) return;
  if (event->type == TouchEvent_Touchdown) {
    s_report->touch_start_x = event->x;
    s_report->touch_start_y = event->y;
    s_report->touch_active  = true;
  } else if (event->type == TouchEvent_Liftoff && s_report->touch_active) {
    s_report->touch_active = false;
    int16_t dx = event->x - s_report->touch_start_x;
    int16_t dy = event->y - s_report->touch_start_y;
    int16_t adx = dx < 0 ? -dx : dx;
    int16_t ady = dy < 0 ? -dy : dy;
    if (adx <= SWIPE_THRESHOLD && ady <= SWIPE_THRESHOLD) return;  // tap — no action
    if (ady >= adx) {
      prv_navigate(dy < 0);      // swipe up = next day (like DOWN), swipe down = previous
    } else if (dx > 0) {
      window_stack_pop(true);    // swipe right = BACK to the forecast list
    }
  }
}
#endif

// ---- Hold-to-scroll with acceleration (raw click subscriber + AppTimer) ----
// The original condensed build's contract, verbatim timing: a single tap steps one day; holding UP/DOWN kicks
// off a timer that steps repeatedly, shrinking the interval by 50ms each step (300ms -> 90ms).
#define HOLD_INITIAL_MS  300
#define HOLD_MIN_MS       90

static AppTimer *s_hold_timer   = NULL;
static bool      s_hold_is_down = false;
static int       s_hold_repeat  = 0;

static void prv_hold_timer_cb(void *ctx) {
  s_hold_timer = NULL;
  if (!s_report) return;
  prv_navigate(s_hold_is_down);
  s_hold_repeat++;
  int interval = HOLD_INITIAL_MS - s_hold_repeat * 50;
  if (interval < HOLD_MIN_MS) interval = HOLD_MIN_MS;
  s_hold_timer = app_timer_register(interval, prv_hold_timer_cb, NULL);
}

static void prv_raw_up_down(ButtonId btn, bool pressed) {
  if (!s_report) return;
  if (pressed) {
    s_hold_is_down = (btn == BUTTON_ID_DOWN);
    s_hold_repeat  = 0;
    if (s_hold_timer) { app_timer_cancel(s_hold_timer); s_hold_timer = NULL; }
    s_hold_timer = app_timer_register(HOLD_INITIAL_MS, prv_hold_timer_cb, NULL);
  } else {
    if (s_hold_timer) { app_timer_cancel(s_hold_timer); s_hold_timer = NULL; }
  }
}

static void prv_raw_up_pressed(ClickRecognizerRef r, void *ctx)   { prv_raw_up_down(BUTTON_ID_UP,   true);  }
static void prv_raw_up_released(ClickRecognizerRef r, void *ctx)  { prv_raw_up_down(BUTTON_ID_UP,   false); }
static void prv_raw_down_pressed(ClickRecognizerRef r, void *ctx) { prv_raw_up_down(BUTTON_ID_DOWN, true);  }
static void prv_raw_down_released(ClickRecognizerRef r, void *ctx){ prv_raw_up_down(BUTTON_ID_DOWN, false); }

static void prv_click_provider(void *ctx) {
  // Single tap = one day; hold UP/DOWN = accelerating scroll (raw handlers drive the hold timer).
  window_single_click_subscribe(BUTTON_ID_UP,   prv_click_up_down);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_click_up_down);
  window_raw_click_subscribe(BUTTON_ID_UP,   prv_raw_up_pressed,   prv_raw_up_released,   NULL);
  window_raw_click_subscribe(BUTTON_ID_DOWN, prv_raw_down_pressed, prv_raw_down_released, NULL);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_click_back);
}

// ---- Entrance: the whole page slides in from the RIGHT on the FULL Timeline moook —
// the exact grammar of Timeline's next-day text intro after the smiley/day-separator
// animation (anticipation, fast travel, overshoot past rest, settle), rotated from-right.
// A rigid translate, deliberately NOT a jelly squash: Timeline's text intro doesn't deform.

static void prv_slide_in_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_report) return;
  Layer *root = s_report->layout.root_layer;
  const int w = layer_get_bounds(root).size.w;
  GRect f = layer_get_frame_by_value(root);
  f.origin.x = (int)interpolate_moook(progress, w, 0);   // slide in from the right (+w -> 0)
  layer_set_frame(root, f);
}

static void prv_slide_in_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)finished; (void)context;
  s_slide_in_anim = NULL;
  if (s_report) {
    Layer *root = s_report->layout.root_layer;
    GRect f = layer_get_frame_by_value(root);
    f.origin.x = 0;
    layer_set_frame(root, f);
    layer_mark_dirty(root);
  }
  animation_destroy(anim);
}

static const AnimationImplementation s_slide_in_impl = { .update = prv_slide_in_update };

static void prv_start_slide_in(void) {
  if (!s_report || s_slide_in_anim) return;
  Layer *root = s_report->layout.root_layer;
  const int w = layer_get_bounds(root).size.w;
  GRect f = layer_get_frame_by_value(root);
  f.origin.x = w;                 // start the whole page off the right edge
  layer_set_frame(root, f);
  s_slide_in_anim = animation_create();
  animation_set_implementation(s_slide_in_anim, &s_slide_in_impl);
  animation_set_duration(s_slide_in_anim, interpolate_moook_duration());   // full moook —
                                        // Timeline's day-text intro length, bounce included
  animation_set_curve(s_slide_in_anim, AnimationCurveLinear);
  animation_set_handlers(s_slide_in_anim,
                         (AnimationHandlers){ .stopped = prv_slide_in_stopped }, NULL);
  animation_schedule(s_slide_in_anim);
}

// ---- Window lifecycle ----

static void prv_window_load(Window *window) {
  if (!s_report) return;
  GRect bounds = layer_get_bounds(window_get_root_layer(window));
  weather_app_layout_init(&s_report->layout, &bounds);
  layer_add_child(window_get_root_layer(window), s_report->layout.root_layer);
  prv_seed_day(s_report->current_day_index);
}

static void prv_window_appear(Window *window) {
#ifdef CONFIG_TOUCH
  if (s_report) touch_service_subscribe(prv_touch_handler, s_report);
#endif
  if (s_report && s_pending_static_in) {
    s_pending_static_in = false;
    prv_start_slide_in();     // the scene's finale: Timeline moook slide from the right
  }
}

static void prv_window_unload(Window *window) {
#ifdef CONFIG_TOUCH
  touch_service_unsubscribe();
#endif
  if (s_hold_timer) { app_timer_cancel(s_hold_timer); s_hold_timer = NULL; }
  // Module-level animations: null-first, then unschedule (the synchronous .stopped
  // handlers see nulled handles and only destroy) — so a re-push can't find stale
  // handles or animate the next instance's layers.
  if (s_slide_in_anim) {
    Animation *a = s_slide_in_anim;
    s_slide_in_anim = NULL;
    animation_unschedule(a);
  }
  if (s_report) {
    weather_app_layout_deinit(&s_report->layout);  // cancels icon/fin anims, frees layers+bitmaps
  }
  window_destroy(window);
  if (s_report) {
    free(s_report);
    s_report = NULL;
  }
}

void weather_report_push(const WeatherLocationForecast *days, size_t num_days, int start_day_index) {
  // Consume the entrance latch up front so NO early return (guard, alloc, window failure)
  // can leave it armed for an unrelated future push; re-armed below just before the push.
  const bool static_in = s_pending_static_in;
  s_pending_static_in = false;
  if (s_report || !days || num_days == 0) return;
  s_report = calloc(1, sizeof(WeatherReportData));
  if (!s_report) return;
  s_report->days = days;
  s_report->num_days = num_days;
  s_report->current_day_index =
      (start_day_index >= 0 && start_day_index < (int)num_days) ? start_day_index : 0;
  s_report->window = window_create();
  if (!s_report->window) {
    free(s_report);
    s_report = NULL;
    return;
  }
  window_set_background_color(s_report->window, system_theme_get_bg_color());
  window_set_window_handlers(s_report->window, (WindowHandlers){
    .load   = prv_window_load,
    .appear = prv_window_appear,
    .unload = prv_window_unload,
  });
  window_set_click_config_provider(s_report->window, prv_click_provider);
  s_pending_static_in = static_in;   // re-arm for prv_window_appear (-> the moook slide-in)
  window_stack_push(s_report->window, !static_in);
}

bool weather_report_is_showing(void) {
  return s_report && s_report->window;
}

// Re-point the borrowed days array after a weather-event refresh (the array is rewritten in
// place and can SHRINK, e.g. a location change to a v3-only record) — without this the view
// keeps navigating over the previous city's stale day slots. No-op when not showing.
void weather_report_update_data(const WeatherLocationForecast *days, size_t num_days) {
  if (!s_report || !days || num_days == 0) return;
  s_report->days = days;
  s_report->num_days = num_days;
  if (s_report->current_day_index >= (int)num_days) {
    s_report->current_day_index = (int)num_days - 1;
  }
  if (s_report->layout.root_layer) {   // update_data-before-load guard
    prv_seed_day(s_report->current_day_index);   // re-seed: may be a different city now
  }
}

#endif  // PBL_ROUND
