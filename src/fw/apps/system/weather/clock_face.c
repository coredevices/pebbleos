/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clock_face.h"
#include "weather.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/animation_interpolate.h"
#include "weather_math.h"
#include "weather_types.h"
#include "pebble_compat.h"
#include <time.h>

// ---- Layout ----
#define LOCATION_BAR_H   18
#define LOCATION_BAR_Y   PBL_IF_ROUND_ELSE(18, 0)
#define LOCATION_BAR_TEXT_INSET PBL_IF_ROUND_ELSE(36, 0)
#define ROUND_LOCATION_BAR_TEXT_INSET 50
#define ROUND_BAR_DEPTH 58
#define ROUND_BAR_RADIUS 207
#define ROUND_BAR_Y_ADJUST -13
#define ROUND_TIME_TEXT_Y 2
#define ROUND_LOCATION_TEXT_Y 21
#define MAX_DAYS          7
// One bitmap slot per drawable WeatherType enum value (0..8).
#define NUM_TYPE_SLOTS   9

// Geometry — tuned so 25×25 icons at the orbit don't clip the screen or the bar.
#if PBL_DISPLAY_HEIGHT >= 200
  // Emery: 200×228 rect — oval to fill the screen
  #define ICON_ORBIT_RX   PBL_IF_ROUND_ELSE(100, 87)
  #define ICON_ORBIT_RY   PBL_IF_ROUND_ELSE(100, 94)
#else
  // Chalk: 180×180 round
  #define ICON_ORBIT_RX   63
  #define ICON_ORBIT_RY   63
#endif

#define ICON_SIZE         25   // Emery/native tiny weather bitmap size
// Round clock icons draw 1:1 from the 25x25 TINY PNGs (CLOCK ids now map to TINY),
// so the slot must be 25 to avoid the previous 50->30 crop. Emery uses ICON_SIZE.
#define CLOCK_ICON_SIZE   PBL_IF_ROUND_ELSE(25, ICON_SIZE)
#define SWIPE_THRESHOLD   20
#define CLOCK_INTRO_DURATION_MS 520
// Rect: 50ms. ROUND (gabbro smoothness step 4): 66ms = exactly 2 animation-service ticks,
// so every glow step lands ON a display frame instead of beating against the 33ms interval.
// CLOCK_GLOW_IDLE_TICKS derives from this (round: 75 ticks ≈ 4.95s — the 5s intent holds);
// the active sweep's per-tick advance is scaled to keep degrees-per-ms (see prv_glow_step).
// The idle wrap-out flourish keeps its tick counts (runs ~30% longer on round — accepted;
// it is a once-after-idle flourish, not a paced animation).
#define CLOCK_GLOW_TIMER_MS PBL_IF_ROUND_ELSE(66, 50)
#define CLOCK_GLOW_OUTER_R PBL_IF_ROUND_ELSE(42, 32)
// Today's centre shows the temperature; its glow ring is a smidge smaller than
// the other days' (which keep the full CLOCK_GLOW_OUTER_R).
#define CLOCK_GLOW_OUTER_R_TODAY PBL_IF_ROUND_ELSE(38, 28)
#define CLOCK_GLOW_IDLE_TIMEOUT_MS 5000
#define CLOCK_GLOW_IDLE_TICKS (CLOCK_GLOW_IDLE_TIMEOUT_MS / CLOCK_GLOW_TIMER_MS)

// ---- State ----
typedef struct {
  Window  *window;
  Layer   *canvas;
  WeatherLocationForecast days[MAX_DAYS];
  size_t   num_days;
  int      day_index;        // which day this clock represents (0 = today)
  uint8_t  hourly_types[24]; // WeatherType per hour 0-23
  bool     hourly_valid;     // true once hourly data received
  GBitmap *type_bitmaps[NUM_TYPE_SLOTS]; // one loaded bitmap per unique WeatherType
  GFont      bar_font;
  GFont      temp_font;
  GBitmap   *temp_text_bmp;    // captured once at full size; NULL = not yet captured
  GSize      temp_text_full_sz; // pixel dimensions of the captured bitmap
  int16_t    temp_text_cx;      // x pixel of the text visual centre within the bitmap
  int16_t    temp_text_cy;      // y pixel of the text visual centre within the bitmap
  AnimationProgress anim_progress; // intro animation progress (0 → ANIMATION_NORMALIZED_MAX)
  Animation        *intro_anim;
  bool     skip_intro_animation;
  AppTimer *glow_timer;
#if PBL_ROUND
  Animation *glow_drv;          // frame-coalesced glow driver (see prv_glow_drv_*); the 50ms
  uint32_t   glow_drv_last_ms;  // cadence survives as quanta inside the service tick
  uint32_t   glow_drv_acc_ms;
#endif
  uint32_t glow_phase;
  uint16_t glow_idle_ticks;
  // ---- Temperature reveal (tap) ----
  int8_t   hourly_temps[24]; // temperature per hour 0-23
  bool     hourly_temps_valid;
  bool     temps_shown;      // stable state: true once temps are fully revealed
  bool     reveal_to;        // animation target state (true=reveal, false=hide)
  bool     reveal_active;    // true while reveal/hide animation is in progress
  AnimationProgress reveal_p;
  Animation        *reveal_anim;
#if WEATHER_PLATFORM_TOUCH_COLOR
  int16_t  touch_start_x;
  int16_t  touch_start_y;
  bool     touch_active;
  bool     touch_started_during_intro;
#endif
  // ---- UP → forecast: whole-screen Timeline squash-stretch exit ----
  // The clock is captured each frame and jelly-stretched down off the bottom (two moook edges:
  // the bottom leads, the top trails — the bitmap analogue of scale_segmented's per-point lag).
  // BOTH shapes: weather_render_squash is chord-clamped, so round drives the same mode.
  AnimationProgress fwd_exit_p;     // 0 → MAX
  Animation        *fwd_exit_anim;
  bool              fwd_exit_active;
  bool              fwd_captured;   // ROUND capture-once: snapshot filled — later frames skip
                                    // the clock render + fb copy and only resample (step 2)
  AppTimer         *fwd_push_timer; // 0ms timer → hand off to the forecast once fully off-top
  uint8_t          *fwd_scratch;    // full-screen snapshot we re-sample while overwriting the fb
} ClockFaceData;

static ClockFaceData *s_cf;
static ClockFaceWrapCallback s_wrap_callback;
static void *s_wrap_context;

// ---- Helpers ----

#define prv_bg_color_for_type weather_type_bg_color
#if defined(PBL_PLATFORM_GABBRO)
  #define prv_icon_res weather_type_icon_clock_resource
#else
  #define prv_icon_res weather_type_icon_tiny_resource
#endif


// Load the tiny bitmap set once; the clock reuses these during animations.
static void prv_load_bitmaps(void) {
  if (!s_cf) return;
  for (int i = 0; i < NUM_TYPE_SLOTS; i++) {
    if (s_cf->type_bitmaps[i]) {
      gbitmap_destroy(s_cf->type_bitmaps[i]);
      s_cf->type_bitmaps[i] = NULL;
    }
  }
  // Load only the types this push can DRAW (the day set + the hourly reveal
  // series), not all nine — the type set is fixed for the window's life (day
  // flips re-push). Every consumer clamps out-of-range bytes to Generic, so
  // the used-set applies the same clamp, and Generic is ALWAYS loaded (it is
  // also the NULL-slot fallback at draw time). A few KB of session heap back.
  bool used[NUM_TYPE_SLOTS] = { false };
  used[WeatherType_Generic] = true;
  for (size_t i = 0; i < s_cf->num_days; i++) {
    uint8_t wt = (uint8_t)s_cf->days[i].current_weather_type;
    used[wt <= WeatherType_RainAndSnow ? wt : (uint8_t)WeatherType_Generic] = true;
  }
  if (s_cf->hourly_valid) {  // draw only samples hourly when the series is complete
    for (int h = 0; h < 24; h++) {
      uint8_t wt = s_cf->hourly_types[h];
      used[wt <= WeatherType_RainAndSnow ? wt : (uint8_t)WeatherType_Generic] = true;
    }
  }
  for (int i = WeatherType_PartlyCloudy; i <= WeatherType_RainAndSnow; i++) {
    if (used[i]) {
      s_cf->type_bitmaps[i] =
          gbitmap_create_with_resource(prv_icon_res((WeatherType)i));
    }
  }
}

// One 50ms quantum of glow animation. Returns false when the driver should stop (covered,
// or idle wrap complete).
static bool prv_glow_step(void) {
  if (!s_cf || !s_cf->window || window_stack_get_top_window() != s_cf->window) return false;
  if (s_cf->glow_idle_ticks >= CLOCK_GLOW_IDLE_TICKS + WEATHER_GLOW_WRAP_TICKS) return false;

  bool transition_busy = s_cf->anim_progress < ANIMATION_NORMALIZED_MAX;
  if (!transition_busy) {
    if (s_cf->glow_idle_ticks < CLOCK_GLOW_IDLE_TICKS) {
      // Sweep rate is degrees-per-MS, not per-tick: rect /80 at 50ms ticks; round /61 at
      // 66ms ticks is the same angular speed within 1% (step 4's clean-rate change).
      s_cf->glow_phase =
          (s_cf->glow_phase + (uint32_t)(TRIG_MAX_ANGLE / PBL_IF_ROUND_ELSE(61, 80))) %
          (uint32_t)TRIG_MAX_ANGLE;
    } else {
      s_cf->glow_phase =
          (s_cf->glow_phase + (uint32_t)(TRIG_MAX_ANGLE / 6)) %
          (uint32_t)TRIG_MAX_ANGLE;
    }
    s_cf->glow_idle_ticks++;
  }
  if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
  return true;
}

#if !PBL_ROUND
static void prv_clock_glow_timer_callback(void *context) {
  (void)context;
  if (s_cf) s_cf->glow_timer = NULL;
  if (prv_glow_step()) {
    s_cf->glow_timer = app_timer_register(CLOCK_GLOW_TIMER_MS,
                                          prv_clock_glow_timer_callback, NULL);
  }
}

static void prv_start_clock_glow(void) {
  if (!s_cf || s_cf->glow_timer) return;
  s_cf->glow_timer = app_timer_register(CLOCK_GLOW_TIMER_MS,
                                        prv_clock_glow_timer_callback, NULL);
}
#else
// Frame-coalesced glow (gabbro): same INFINITE-animation pattern as the forecast icon driver —
// glow stepping rides the animation service's tick, so at most one render per interval even
// while the intro/reveal animations run. (Free-running 50ms beat against the 33ms service and
// the ~20-25ms display period read as judder; see the display research.)
static uint32_t prv_glow_now_ms(void) {
  time_t s = 0; uint16_t ms = 0;
  time_ms(&s, &ms);
  return (uint32_t)s * 1000u + ms;
}

static void prv_glow_drv_stop_cb(void *context) {   // 0ms hop: never unschedule inside .update
  (void)context;
  if (s_cf && s_cf->glow_drv) {
    Animation *a = s_cf->glow_drv;
    s_cf->glow_drv = NULL;
    animation_unschedule(a);
    animation_destroy(a);
  }
}

static void prv_glow_drv_update(Animation *anim, AnimationProgress p) {
  (void)anim; (void)p;
  if (!s_cf) return;
  const uint32_t now = prv_glow_now_ms();
  uint32_t dt = now - s_cf->glow_drv_last_ms;
  s_cf->glow_drv_last_ms = now;
  if (dt > 500) dt = 500;
  s_cf->glow_drv_acc_ms += dt;
  bool alive = true;
  while (alive && s_cf->glow_drv_acc_ms >= CLOCK_GLOW_TIMER_MS) {
    s_cf->glow_drv_acc_ms -= CLOCK_GLOW_TIMER_MS;
    alive = prv_glow_step();
  }
  if (!alive) app_timer_register(0, prv_glow_drv_stop_cb, NULL);
}
static const AnimationImplementation s_glow_drv_impl = { .update = prv_glow_drv_update };

static void prv_start_clock_glow(void) {
  if (!s_cf || s_cf->glow_drv) return;
  Animation *a = animation_create();
  if (!a) return;
  animation_set_duration(a, ANIMATION_DURATION_INFINITE);
  animation_set_implementation(a, &s_glow_drv_impl);
  s_cf->glow_drv_last_ms = prv_glow_now_ms();
  s_cf->glow_drv_acc_ms  = 0;
  s_cf->glow_drv = a;
  animation_schedule(a);
}
#endif

static void prv_note_clock_interaction(void) {
  if (!s_cf) return;
  s_cf->glow_idle_ticks = 0;
  if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
  prv_start_clock_glow();
}

// Return the absolute 24h hour for a clock-face position.
// Today (day 0): maps to the CURRENT hour + the next 11:
//   position P → the current-or-upcoming hour h where h%12 == P%12.
//   At 11:00: pos 11→11 (now), pos 12→12, pos 1→13, pos 2→14 ... pos 10→22.
// Other days: a fixed daytime window so the morning reads as AM:
//   pos 7→07, 8→08 ... 12→12, 1→13 ... 6→18 (07:00 through 18:00).
// Current hour, localtime()d at most once per SECOND — this helper runs per icon per frame
// (12-24x/frame at up to 20fps via prv_type_for_pos/prv_temp_for_pos), and newlib's
// localtime re-derives timezone/DST every call.
static int prv_cur_hour(void) {
  static time_t cached_now;
  static int cached_hour = -1;
  time_t now = time(NULL);   // rtc read — cheap, unlike localtime
  if (now != cached_now || cached_hour < 0) {
    cached_now = now;
    cached_hour = localtime(&now)->tm_hour;
  }
  return cached_hour;
}

static int prv_abs_hour_for_pos(int pos_1_to_12) {
  if (s_cf && s_cf->day_index != 0) {
    return (pos_1_to_12 >= 7) ? pos_1_to_12 : pos_1_to_12 + 12;
  }
  int cur_h = prv_cur_hour();
  int target_mod = pos_1_to_12 % 12;  // pos 12→0, pos 1→1, ..., pos 11→11
  // i starts at 0 so the CURRENT hour lands on its own position (and is the
  // one highlighted) — starting at 1 put the current hour off the dial and
  // highlighted the NEXT hour, reading an hour ahead.
  for (int i = 0; i <= 11; i++) {
    int h = (cur_h + i) % 24;
    if (h % 12 == target_mod) return h;
  }
  return pos_1_to_12 % 12; // unreachable
}

// Map a clock-face hour position (1-12) to the WeatherType for that hour.
// Uses real hourly data if available, otherwise falls back to daily forecast.
static WeatherType prv_type_for_pos(int pos_1_to_12) {
  if (!s_cf) return WeatherType_Unknown;
  int abs_hour = prv_abs_hour_for_pos(pos_1_to_12);
  if (s_cf->hourly_valid && abs_hour >= 0 && abs_hour < 24) {
    return (WeatherType)s_cf->hourly_types[abs_hour];
  }
  // Fallback: daily
  if (s_cf->num_days == 0) return WeatherType_Unknown;
  return s_cf->days[0].current_weather_type;
}

// Map a clock-face hour position (1-12) to the temperature for that hour.
// Returns true and fills *out_temp if hourly temperature data is available.
static bool prv_temp_for_pos(int pos_1_to_12, int *out_temp) {
  if (!s_cf || !s_cf->hourly_temps_valid) return false;
  int abs_hour = prv_abs_hour_for_pos(pos_1_to_12);
  if (abs_hour < 0 || abs_hour >= 24) return false;
  *out_temp = (int)s_cf->hourly_temps[abs_hour];
  return true;
}

// Per-position reveal progress (0 → ANIMATION_NORMALIZED_MAX) for the tap
// temperature reveal. Icons swap clockwise starting from the 7 o'clock position.
static int prv_reveal_rp(AnimationProgress reveal_t, int pos) {
  int rank = (pos - 7 + 12) % 12;  // clockwise rank: 7→0, 8→1, ... 6→11
  int32_t item_dur = ANIMATION_NORMALIZED_MAX * 45 / 100;
  int32_t stagger = (ANIMATION_NORMALIZED_MAX - item_dur) / 11;
  int32_t local = (int32_t)reveal_t - rank * stagger;
  if (local <= 0) return 0;
  int32_t rp = weather_scale_i32(local, ANIMATION_NORMALIZED_MAX, item_dur);
  if (rp > ANIMATION_NORMALIZED_MAX) rp = ANIMATION_NORMALIZED_MAX;
  return (int)rp;
}

// ---- Intro animation (expand from dot + clockwise spin) ----

static void prv_intro_update(Animation *anim, AnimationProgress progress) {
  if (s_cf) {
    if (progress > ANIMATION_NORMALIZED_MAX * 99 / 100) {
      progress = ANIMATION_NORMALIZED_MAX;
    }
    s_cf->anim_progress = progress;
    if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
  }
}
static const AnimationImplementation s_intro_impl = { .update = prv_intro_update };

// ---- (Removed) clock -> detail_face exit/return animation ----
// The full-screen weather detail view (temperature + UV/rain + location + last-updated) was made
// redundant by the expanded weather card, so the clock's drill-in was removed. The exit_/return_
// struct fields remain (never set now) so the draw proc's dead branches keep compiling harmlessly.

// ---- Temperature reveal animation (tap) ----
static void prv_reveal_update(Animation *anim, AnimationProgress progress) {
  if (!s_cf) return;
  s_cf->reveal_p = progress;
  if (progress >= ANIMATION_NORMALIZED_MAX) {
    s_cf->reveal_active = false;
    s_cf->temps_shown = s_cf->reveal_to;
  }
  if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
}
static const AnimationImplementation s_reveal_impl = { .update = prv_reveal_update };

// Toggle the per-hour temperature reveal. Each tap flips between showing the
// weather bitmaps and showing the temperature bubbles.
static void prv_toggle_temp_reveal(void) {
  if (!s_cf || !s_cf->hourly_temps_valid) return;
  if (s_cf->reveal_active) return; // ignore taps until the current reveal/hide cycle finishes
  if (s_cf->anim_progress < ANIMATION_NORMALIZED_MAX) return; // wait for intro to finish
  s_cf->reveal_to = !s_cf->temps_shown;
  s_cf->reveal_active = true;
  s_cf->reveal_p = 0;
  if (s_cf->reveal_anim) {
    animation_unschedule(s_cf->reveal_anim);
    animation_destroy(s_cf->reveal_anim);
  }
  s_cf->reveal_anim = animation_create();
  animation_set_implementation(s_cf->reveal_anim, &s_reveal_impl);
  animation_set_duration(s_cf->reveal_anim, 700);
  animation_set_curve(s_cf->reveal_anim, AnimationCurveLinear);
  animation_schedule(s_cf->reveal_anim);
}

static void prv_play_animation(void) {
  if (!s_cf) return;
  if (s_cf->intro_anim) {
    animation_unschedule(s_cf->intro_anim);
    animation_destroy(s_cf->intro_anim);
    s_cf->intro_anim = NULL;
  }
  s_cf->anim_progress = 0;
  // Invalidate the cached text bitmap so it is re-captured on the first draw.
  if (s_cf->temp_text_bmp) {
    gbitmap_destroy(s_cf->temp_text_bmp);
    s_cf->temp_text_bmp = NULL;
  }
  s_cf->intro_anim = animation_create();
  animation_set_implementation(s_cf->intro_anim, &s_intro_impl);
  animation_set_duration(s_cf->intro_anim, CLOCK_INTRO_DURATION_MS);
  animation_set_curve(s_cf->intro_anim, AnimationCurveLinear);
  animation_schedule(s_cf->intro_anim);
}

// ---- Draw proc ----
static int32_t prv_clock_intro_motion_progress(AnimationProgress progress) {
  const int32_t max = ANIMATION_NORMALIZED_MAX;
  if ((int32_t)progress >= max) return max;

  const int32_t overshoot = max / 34;
  const int32_t rebound = max / 135;
  const int32_t hit = max * 78 / 100;
  const int32_t settle = max * 93 / 100;

  if ((int32_t)progress < hit) {
    return weather_scale_i32(progress, max + overshoot, hit);
  }

  if ((int32_t)progress < settle) {
    return max + overshoot -
        weather_scale_i32((int32_t)progress - hit, overshoot + rebound,
                          settle - hit);
  }

  return max - rebound +
      weather_scale_i32((int32_t)progress - settle, rebound, max - settle);
}

static int32_t prv_clock_center_scale_progress(AnimationProgress progress) {
  const int32_t max = ANIMATION_NORMALIZED_MAX;
  if ((int32_t)progress >= max) return max;
  const int32_t min_scale = max / 40;   // start from a point — blooms straight out of the dot
  // Clean ease-out from the point to full size. No overshoot/settle phase (that tail wobble was
  // dropping the centre temperature for a frame).
  const int32_t q = max - (int32_t)progress;
  const int32_t ease_out = max - (int32_t)((int64_t)q * q / max);
  return min_scale + weather_scale_i32(ease_out, max - min_scale, max);
}

// ---- UP → forecast: whole-screen Timeline squash-stretch ---------------------------------
// The whole-screen jelly squash-stretch lives in weather_math.c (weather_render_squash),
// shared with the forecast's squash modes. CLOCK_EXIT is the unhasted down-exit: unlike the
// forecast's DOWN_EXIT it runs the full timeline (no me=m+m/3 compression) — hand-tuned so
// the sunset-card staging reads right; do not switch modes.
// BOTH shapes: the renderer clamps every write to the row's chord, so round squashes inside
// the glass. The scratch is W*H (round 260x260 = 67,600B vs rect 200x228 = 45,600B); the
// malloc_try fallback below degrades to an instant return if the app heap can't take it.
static void prv_render_fwd_squash(GContext *ctx) {
  if (!s_cf || !s_cf->fwd_scratch) return;
  weather_render_squash(ctx, s_cf->fwd_scratch, s_cf->fwd_exit_p,
                        WEATHER_SQUASH_CLOCK_EXIT);
#if PBL_ROUND
  // Capture-once (gabbro smoothness step 2): the snapshot now holds the fully-drawn
  // clock — from the next frame prv_canvas_draw short-circuits to resample-only.
  s_cf->fwd_captured = true;
#endif
}

static void prv_fwd_exit_push_callback(void *ctx);
static void prv_fwd_exit_update(Animation *anim, AnimationProgress progress) {
  if (!s_cf) return;
  s_cf->fwd_exit_p = progress;
  if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
  if (progress >= ANIMATION_NORMALIZED_MAX && !s_cf->fwd_push_timer) {
    s_cf->fwd_push_timer = app_timer_register(0, prv_fwd_exit_push_callback, NULL);
  }
}
static const AnimationImplementation s_fwd_exit_impl = { .update = prv_fwd_exit_update };

static void prv_fwd_exit_push_callback(void *ctx) {
  if (!s_cf) return;
  s_cf->fwd_push_timer = NULL;
  if (s_cf->fwd_scratch) { free(s_cf->fwd_scratch); s_cf->fwd_scratch = NULL; }
  // Hand off to weather.c, which dismisses this clock and jelly-rises the forecast back in.
  if (s_wrap_callback) s_wrap_callback(s_wrap_context);
}

static void prv_start_fwd_exit_animation(void) {
  if (!s_cf || s_cf->fwd_exit_active || !s_cf->canvas) return;
  GRect bounds = layer_get_bounds(s_cf->canvas);
  s_cf->fwd_scratch = malloc_try((size_t)bounds.size.w * (size_t)bounds.size.h);
  if (!s_cf->fwd_scratch) {                       // out of heap: fall back to an instant return
    if (s_wrap_callback) s_wrap_callback(s_wrap_context);
    return;
  }
  s_cf->fwd_exit_active = true;
  s_cf->fwd_captured = false;   // round capture-once: fresh exit, fresh snapshot
  s_cf->fwd_exit_p = 0;
  if (s_cf->fwd_exit_anim) {
    animation_unschedule(s_cf->fwd_exit_anim);
    animation_destroy(s_cf->fwd_exit_anim);
  }
  s_cf->fwd_exit_anim = animation_create();
  animation_set_implementation(s_cf->fwd_exit_anim, &s_fwd_exit_impl);
  animation_set_duration(s_cf->fwd_exit_anim, interpolate_moook_soft_duration(3));
  animation_set_curve(s_cf->fwd_exit_anim, AnimationCurveLinear);
  animation_schedule(s_cf->fwd_exit_anim);
}

static void prv_canvas_draw(Layer *layer, GContext *ctx) {
#if PBL_ROUND
  // Transparent window bg on round (step 3) — the degenerate no-state frame must paint
  // the white itself, since this proc is what guarantees full coverage.
  if (!s_cf) {
    graphics_context_set_fill_color(ctx, GColorWhite);
    graphics_fill_rect(ctx, layer_get_bounds(layer), 0, GCornerNone);
    return;
  }
#else
  if (!s_cf) return;
#endif
#if PBL_ROUND
  // Capture-once (gabbro smoothness step 2): during the UP-to-forecast squash the clock
  // scene is frozen in fwd_scratch after the first frame — skip the whole clock render
  // and only resample. Nothing draws after the squash in this proc (it is the last call),
  // so a full short-circuit is safe; the resample overwrites every pixel.
  if (s_cf->fwd_exit_active && s_cf->fwd_captured && s_cf->fwd_scratch) {
    weather_render_squash_cached(ctx, s_cf->fwd_scratch, s_cf->fwd_exit_p,
                                 WEATHER_SQUASH_CLOCK_EXIT);
    return;
  }
#endif
  GRect bounds = layer_get_bounds(layer);
  int W = bounds.size.w;
  int H = bounds.size.h;

  // White background — matches other screens
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);

  // Clock face centre — full screen, no bar offset
  int cx = W / 2;
  int cy = H / 2;

  AnimationProgress p = s_cf->anim_progress;
  int32_t intro_p = prv_clock_intro_motion_progress(p);
  int32_t centre_scale_p = prv_clock_center_scale_progress(p);
  bool anim_done = (p >= ANIMATION_NORMALIZED_MAX);
  // Temperature reveal (tap) state — engaged once the intro has finished and the
  // temperatures are showing.
  bool reveal_engaged = s_cf->hourly_temps_valid && anim_done &&
      (s_cf->temps_shown || s_cf->reveal_active);
  AnimationProgress reveal_t = 0;
  if (reveal_engaged) {
    reveal_t = s_cf->reveal_active
        ? (s_cf->reveal_to ? s_cf->reveal_p
                           : (ANIMATION_NORMALIZED_MAX - s_cf->reveal_p))
        : ANIMATION_NORMALIZED_MAX;
  }
  // Clockwise spin: icons start 90° counter-clockwise from target, sweep clockwise to land
  int32_t angle_offset = -(int32_t)(TRIG_MAX_ANGLE / 4)
      * (int32_t)(ANIMATION_NORMALIZED_MAX - intro_p) / (int32_t)ANIMATION_NORMALIZED_MAX;

  // Current hour — the dial highlights the CURRENT hour's position ("now").
  int cur_h24 = prv_cur_hour();

  // ---- 12 weather icons at hour positions (oval orbit on Emery) ----
  // During exit animation the icons fall downward and shrink away.
  graphics_context_set_compositing_mode(ctx, GCompOpSet);
  int icon_xs[12];
  int icon_ys[12];
  GBitmap *icon_bmps[12];

  for (int pos = 1; pos <= 12; pos++) {
    int32_t angle = (int32_t)TRIG_MAX_ANGLE * pos / 12 + angle_offset;
    int nudge = PBL_IF_ROUND_ELSE(0, (pos == 3 ? 7 : pos == 9 ? 7 : 0));
    int orbit_rx = ((int)(ICON_ORBIT_RX - nudge) * (int)intro_p / (int)ANIMATION_NORMALIZED_MAX);
    int orbit_ry = ((int)ICON_ORBIT_RY * (int)intro_p / (int)ANIMATION_NORMALIZED_MAX);
    int ix = cx + (int)((int32_t)sin_lookup(angle) * orbit_rx / TRIG_MAX_RATIO);
    int iy = cy - (int)((int32_t)cos_lookup(angle) * orbit_ry / TRIG_MAX_RATIO);

    WeatherType wt = prv_type_for_pos(pos);
    int idx = (wt <= WeatherType_RainAndSnow) ? (int)wt : (int)WeatherType_Generic;

    int full_r = CLOCK_ICON_SIZE * 7 / 10;
    int circle_r = anim_done ? full_r
        : 2 + (int)((full_r - 2) * (int)intro_p / (int)ANIMATION_NORMALIZED_MAX);

    icon_xs[pos-1] = ix;
    icon_ys[pos-1] = iy;
    icon_bmps[pos-1] = s_cf->type_bitmaps[idx]
                   ? s_cf->type_bitmaps[idx]
                   : s_cf->type_bitmaps[WeatherType_Generic];

    if (reveal_engaged) {
      // Per-hour temperature reveal: each circle flips clockwise from a coloured
      // weather bubble into a white circle with a dithered grey outline + the temp.
      int rp = prv_reveal_rp(reveal_t, pos);
      // Pinch the circle as it flips: full size at the ends, half size at the
      // midpoint where the content swaps.
      int32_t mag = (rp <= ANIMATION_NORMALIZED_MAX / 2)
          ? (ANIMATION_NORMALIZED_MAX - 2 * rp)   // 1 → 0
          : (2 * rp - ANIMATION_NORMALIZED_MAX);  // 0 → 1
      int rr = full_r - weather_scale_i32(ANIMATION_NORMALIZED_MAX - mag,
                                          full_r / 2,
                                          ANIMATION_NORMALIZED_MAX);
      bool show_temp_side = (rp >= ANIMATION_NORMALIZED_MAX / 2);
      bool show_temp_text = (rp >= ANIMATION_NORMALIZED_MAX * 76 / 100);
      bool show_icon      = (rp <= ANIMATION_NORMALIZED_MAX * 24 / 100);

      if (rr < 1) continue; // fully shrunk — nothing to draw

      graphics_context_set_compositing_mode(ctx, GCompOpAssign);
      if (show_temp_side) {
        graphics_context_set_fill_color(ctx, GColorWhite);
        graphics_fill_circle(ctx, GPoint(ix, iy), rr);
        // Soft dithered grey outline (single pixel) instead of a hard black ring.
        graphics_context_set_stroke_color(ctx, GColorLightGray);
        graphics_context_set_stroke_width(ctx, 1);
        graphics_draw_circle(ctx, GPoint(ix, iy), rr);
      } else {
        graphics_context_set_fill_color(ctx, prv_bg_color_for_type(wt));
        graphics_fill_circle(ctx, GPoint(ix, iy), rr);
      }

      // Content: weather bitmap before the flip, temperature after — hidden during
      // the pinch so it never pokes outside the shrinking circle.
      if (show_icon) {
        GBitmap *bmp = icon_bmps[pos-1];
        if (bmp) {
          graphics_context_set_compositing_mode(ctx, GCompOpSet);
          graphics_draw_bitmap_in_rect(ctx, bmp,
              GRect(ix - CLOCK_ICON_SIZE / 2, iy - CLOCK_ICON_SIZE / 2,
                    CLOCK_ICON_SIZE, CLOCK_ICON_SIZE));
        }
      } else if (show_temp_text) {
        int temp;
        if (prv_temp_for_pos(pos, &temp)) {
          // Centre the number in the circle; the degree sign hangs off to the
          // right without shifting the number's centring.
          char nbuf[6];
          snprintf(nbuf, sizeof(nbuf), "%d", temp);
          static GFont tfont;   // stable system handle — don't re-look-up per icon per frame
          if (!tfont) tfont = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
          GSize nsz = graphics_text_layout_get_content_size(
              nbuf, tfont, GRect(0, 0, 40, 18),
              GTextOverflowModeFill, GTextAlignmentCenter);
          graphics_context_set_text_color(ctx, GColorBlack);
          graphics_draw_text(ctx, nbuf, tfont,
              GRect(ix - nsz.w / 2, iy - 11, nsz.w, 18),
              GTextOverflowModeFill, GTextAlignmentCenter, NULL);
          graphics_draw_text(ctx, "\xC2\xB0", tfont,
              GRect(ix + nsz.w / 2, iy - 13, 10, 18),
              GTextOverflowModeFill, GTextAlignmentLeft, NULL);
        }
      }
      continue; // reveal path draws both circle and content; skip the bitmap pass
    }

    if (circle_r < 1) continue; // fully shrunk — skip background circle

    graphics_context_set_compositing_mode(ctx, GCompOpAssign);
    graphics_context_set_fill_color(ctx, prv_bg_color_for_type(wt));
    graphics_fill_circle(ctx, GPoint(ix, iy), circle_r);
  }

  // Draw icon bitmaps — doing this in a separate pass minimizes framebuffer captures
  if (!reveal_engaged && (anim_done || p > ANIMATION_NORMALIZED_MAX / 2)) {
    graphics_context_set_compositing_mode(ctx, GCompOpSet);
    for (int i = 0; i < 12; i++) {
      if (!icon_bmps[i]) continue;
      int bx = icon_xs[i] - CLOCK_ICON_SIZE / 2;
      int by = icon_ys[i] - CLOCK_ICON_SIZE / 2;
      graphics_draw_bitmap_in_rect(ctx, icon_bmps[i],
          GRect(bx, by, CLOCK_ICON_SIZE, CLOCK_ICON_SIZE));
    }
  }

  // ---- Centre: temperature (today) or weekday label (other days) ----
  // For today (day 0) the captured bitmap is the current temperature + degree.
  // For any other day it is the uppercase weekday abbreviation ("MON", "TUE"...).
  bool centre_is_temp = (s_cf->day_index == 0);
  bool centre_valid = centre_is_temp
      ? (s_cf->num_days > 0 &&
         PBL_IF_ROUND_ELSE(s_cf->days[0].current_temp_now, s_cf->days[0].current_temp)
             != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP)
      : true;
  if (centre_valid) {
    // ---- Lazy-capture the centre text into a bitmap once, then scale it each frame. ----
    // On the very first draw call after a new animation starts, render the text at full size
    // into an off-screen corner of the framebuffer, copy those pixels into a GBitmap, and
    // erase the staging area.  Every subsequent frame just scales that bitmap.
    if (!s_cf->temp_text_bmp) {
      // Render the centre text at full size into a staging rect.
      // The text is centered at (stg_cx, stg_cy) within the staging area so we
      // know exactly where the visual centre is and can use it as the scale pivot.
      char stg_buf[8];
      GFont centre_font;
      if (centre_is_temp) {
        // ROUND: the CURRENT-HOUR temp — the same source as the mainscreen header, and the
        // same number the reveal shows at the highlighted hour. The synced current_temp said
        // "23" here while the header (hourly-now) said "18" — two screens, two "currents".
        snprintf(stg_buf, sizeof(stg_buf), "%d",
                 PBL_IF_ROUND_ELSE(s_cf->days[0].current_temp_now,
                                   s_cf->days[0].current_temp));
        centre_font = s_cf->temp_font;
      } else {
        weather_fill_weekday_abbrev(s_cf->day_index, NULL, stg_buf, sizeof(stg_buf));
        centre_font = fonts_get_system_font(FONT_KEY_BITHAM_30_BLACK);
      }
      GSize nsz = graphics_text_layout_get_content_size(
          stg_buf, centre_font, GRect(0, 0, 100, 50),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft);
      const int left_pad = 4;
      const int right_pad = 4;
      const int stg_w  = left_pad + nsz.w + right_pad;
      const int stg_h  = nsz.h + 8;   // vertical padding
      const int stg_x  = PBL_IF_ROUND_ELSE((W - stg_w) / 2, 0);
      const int stg_y  = PBL_IF_ROUND_ELSE((H - stg_h) / 2, 0);
      const int stg_cx = left_pad + nsz.w / 2;
      const int stg_cy = stg_h / 2;
      const int font_top_pad = -4;
      int ty_s = stg_cy - nsz.h / 2 + font_top_pad;
      // Clear staging area so no icon pixels bleed into the capture.
      graphics_context_set_fill_color(ctx, GColorWhite);
      graphics_fill_rect(ctx, GRect(stg_x, stg_y, stg_w, stg_h), 0, GCornerNone);
      graphics_context_set_text_color(ctx, GColorBlack);
      // Number: centred horizontally at stg_cx
      graphics_draw_text(ctx, stg_buf, centre_font,
          GRect(stg_x + left_pad, stg_y + ty_s, nsz.w + 2, nsz.h),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
      // Capture those pixels into a GBitmap
      GBitmap *fb = graphics_capture_frame_buffer(ctx);
      if (fb) {
        GBitmap *bmp = gbitmap_create_blank(GSize(stg_w, stg_h), GBitmapFormat8Bit);
        if (bmp) {
          weather_capture_framebuffer_rect(fb, bmp, GRect(stg_x, stg_y, stg_w, stg_h), 0);
          s_cf->temp_text_bmp     = bmp;
          s_cf->temp_text_full_sz = GSize(stg_w, stg_h);
          s_cf->temp_text_cx      = (int16_t)stg_cx;
          s_cf->temp_text_cy      = (int16_t)(stg_cy - font_top_pad / 2);
        }
        graphics_release_frame_buffer(ctx, fb);
      }
    }

    if (s_cf->temp_text_bmp) {
      int src_w = s_cf->temp_text_full_sz.w;
      int src_h = s_cf->temp_text_full_sz.h;
      int dst_w, dst_h;
      if (anim_done) {
        dst_w = src_w; dst_h = src_h;
      } else {
        dst_w = src_w * centre_scale_p / ANIMATION_NORMALIZED_MAX;
        dst_h = src_h * centre_scale_p / ANIMATION_NORMALIZED_MAX;
      }
      if (dst_w < 1) dst_w = 1;
      if (dst_h < 1) dst_h = 1;
      if (centre_is_temp) {
        dst_w = dst_w * 11 / 10;
        dst_h = dst_h * 11 / 10;
        if (dst_w < 1) dst_w = 1;
        if (dst_h < 1) dst_h = 1;
      }
      // Weekday label (non-today) renders a touch smaller than today's temperature.
      // Today's centre is left exactly as-is.
      if (!centre_is_temp) {
        dst_w = dst_w * 8 / 10;
        dst_h = dst_h * 8 / 10;
        if (dst_w < 1) dst_w = 1;
        if (dst_h < 1) dst_h = 1;
      }

      // Use the recorded visual centre of the text as the scale pivot,
      // so the text remains centred on (cx, cy) at all sizes.
      int cx_scaled = s_cf->temp_text_cx * dst_w / src_w;
      int cy_scaled = s_cf->temp_text_cy * dst_h / src_h;
      WeatherType centre_weather_type = WeatherType_Unknown;
      if (s_cf->num_days > 0) {
        int di = s_cf->day_index >= 0 && (size_t)s_cf->day_index < s_cf->num_days
                     ? s_cf->day_index
                     : 0;
        centre_weather_type = s_cf->days[di].current_weather_type;
      }
      if (anim_done) {
        GColor glow_color = prv_bg_color_for_type(centre_weather_type);
        uint8_t idle_progress = s_cf->glow_idle_ticks >= CLOCK_GLOW_IDLE_TICKS
            ? (uint8_t)(s_cf->glow_idle_ticks - CLOCK_GLOW_IDLE_TICKS) : 0;
        int glow_outer_r = centre_is_temp ? CLOCK_GLOW_OUTER_R_TODAY
                                          : CLOCK_GLOW_OUTER_R;
        weather_draw_lava_ring(ctx, GPoint(cx, cy), glow_outer_r,
                               glow_color, s_cf->glow_phase, idle_progress);
        prv_start_clock_glow();
      }

      int abs_x = cx - cx_scaled;
      int abs_y = cy - cy_scaled;

      GBitmap *fb = graphics_capture_frame_buffer(ctx);
      if (fb) {
        GRect fbb = gbitmap_get_bounds(fb);
        int fb_h = fbb.size.h;
        uint8_t *sdata = gbitmap_get_data(s_cf->temp_text_bmp);
        uint16_t sbpr  = gbitmap_get_bytes_per_row(s_cf->temp_text_bmp);
        int32_t sx_step = ((int32_t)src_w << 16) / dst_w;
        int32_t sy_step = ((int32_t)src_h << 16) / dst_h;
        int32_t sy_fp = sy_step >> 1;
        for (int dy = 0; dy < dst_h; dy++, sy_fp += sy_step) {
          int sy = sy_fp >> 16;
          if (sy >= src_h) sy = src_h - 1;
          int ay = abs_y + dy;
          if (ay < 0 || ay >= fb_h) continue;
          GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)ay);
          uint8_t *srow = sdata + (uint32_t)sy * sbpr;
          int32_t sx_fp2 = sx_step >> 1;
          for (int dx = 0; dx < dst_w; dx++, sx_fp2 += sx_step) {
            int sx = sx_fp2 >> 16;
            if (sx >= src_w) sx = src_w - 1;
            int ax = abs_x + dx;
            if (ax < (int)ri.min_x || ax > (int)ri.max_x) continue;
            uint8_t pixel = srow[sx];
            // Skip white pixels (background) — GColor8 white = 0xFF
            if (pixel == 0xFF) continue;
            ri.data[ax] = pixel;
          }
        }
        graphics_release_frame_buffer(ctx, fb);
      }
    }
  }

  // ---- Hour number labels — fade in during final third of animation ----
  // Pebble has no alpha; simulate fade: invisible (white) → light gray → dark gray
  // Start appearing at 60% progress, reach full color at 100%.
  AnimationProgress fade_start = (AnimationProgress)(ANIMATION_NORMALIZED_MAX * 6 / 10);
  if (p < fade_start) return;
  GColor label_color;
  if (anim_done || p > (AnimationProgress)(ANIMATION_NORMALIZED_MAX * 85 / 100)) {
    label_color = GColorDarkGray;
  } else if (p > (AnimationProgress)(ANIMATION_NORMALIZED_MAX * 72 / 100)) {
    label_color = GColorDarkGray;   // second step — already dark
  } else {
    label_color = GColorLightGray;  // first step — light gray on white
  }
  GFont label_font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
#if PBL_DISPLAY_HEIGHT >= 200
  #define LABEL_ORBIT_RX (ICON_ORBIT_RX - 42)
  #define LABEL_ORBIT_RY (ICON_ORBIT_RY - 42)
  #define DOT_ORBIT_RX  (ICON_ORBIT_RX - 27)
  #define DOT_ORBIT_RY  (ICON_ORBIT_RY - 27)
#else
  #define LABEL_ORBIT_RX (ICON_ORBIT_RX - 30)
  #define LABEL_ORBIT_RY (ICON_ORBIT_RY - 30)
  #define DOT_ORBIT_RX  (ICON_ORBIT_RX - 18)
  #define DOT_ORBIT_RY  (ICON_ORBIT_RY - 18)
#endif
  for (int pos = 1; pos <= 12; pos++) {
    int32_t angle = (int32_t)TRIG_MAX_ANGLE * pos / 12;
    // Label nudge: 3 and 9 o'clock pulled 5px inward (their text sits widest horizontally)
    int label_nudge = PBL_IF_ROUND_ELSE(0, (pos == 3 || pos == 9) ? 5 : 0);
    int lx = cx + (int)((int32_t)sin_lookup(angle) * (LABEL_ORBIT_RX - label_nudge) / TRIG_MAX_RATIO);
    int ly = cy - (int)((int32_t)cos_lookup(angle) * LABEL_ORBIT_RY / TRIG_MAX_RATIO);
    // Dot between number and icon — 3 and 9 o'clock nudged 6px inward
    int dot_nudge = PBL_IF_ROUND_ELSE(0, (pos == 3 || pos == 9) ? 6 : 0);
    int ddx = cx + (int)((int32_t)sin_lookup(angle) * (DOT_ORBIT_RX - dot_nudge) / TRIG_MAX_RATIO);
    int ddy = cy - (int)((int32_t)cos_lookup(angle) * (DOT_ORBIT_RY - dot_nudge) / TRIG_MAX_RATIO);
    // 24h hour number — the CURRENT hour gets a blue bubble (and no dot).
    // Only today's clock highlights "now" — for other days "now" isn't on that
    // day, so no hour is highlighted.
    int abs_h = prv_abs_hour_for_pos(pos);
    bool is_now = (s_cf->day_index == 0) && (abs_h == cur_h24);
    if (!is_now) {
      graphics_context_set_fill_color(ctx, label_color);
      graphics_fill_circle(ctx, GPoint(ddx, ddy), 2);
    }
    char lbuf[4];
    snprintf(lbuf, sizeof(lbuf), "%d", abs_h);
    int lw   = (abs_h >= 10) ? 28 : 20;
    int loff = (abs_h >= 10) ? -14 : -10;
    GRect label_rect = GRect(lx + loff, ly - 12, lw, 18);
    GTextAlignment label_align = GTextAlignmentCenter;
    int pill_len = lw + 12;
    int pill_center_x = lx;
    int pill_center_y = ly + 1;

#if defined(PBL_PLATFORM_GABBRO)
    if (pos == 3 || pos == 9) {
      const int side_slot_w = 30;
      const int side_slot_inner_half = 14;
      if (pos == 3) {
        label_rect = GRect(lx - side_slot_inner_half, ly - 12,
                           side_slot_w, 18);
        label_align = GTextAlignmentLeft;
      } else {
        label_rect = GRect(lx + side_slot_inner_half - side_slot_w, ly - 12,
                           side_slot_w, 18);
        label_align = GTextAlignmentRight;
      }
      pill_len = side_slot_w + 12;
      pill_center_x = label_rect.origin.x + label_rect.size.w / 2;
    }
#endif

    // The blue indicator fades in (scales up from a small dot) a few frames before
    // it settles, then stretches into the oval once the bitmaps have landed.
    int32_t pill_appear  = (ANIMATION_NORMALIZED_MAX * 70) / 100; // dot starts growing
    int32_t pill_settled = (ANIMATION_NORMALIZED_MAX * 78) / 100; // full circle, begins stretch
    bool show_pill = is_now && (anim_done || p > pill_appear);

    if (show_pill) {
      int r = 9;   // final radius => diameter 19
      int D = (pill_len - (r * 2)) / 2;

      int D_out = D - 5; // Pull the pill inwards 5px from the outside (the number-facing end)
      if (D_out < 0) D_out = 0;
      int D_in = D - 8; // Pull the pill inwards from the center of the watch face (less padding on the inside)
      if (D_in < 0) D_in = 0;

      if (!anim_done) {
        if (p < pill_settled) {
          // Grow the dot from a small radius up to full size while it fades in.
          int32_t grow_p = (p - pill_appear) * ANIMATION_NORMALIZED_MAX / (pill_settled - pill_appear);
          if (grow_p < 0) grow_p = 0;
          r = 2 + (int)((int32_t)(r - 2) * grow_p / ANIMATION_NORMALIZED_MAX);
          D_out = 0;
          D_in = 0;
        } else {
          // Stretch the full circle out into the oval.
          int32_t stretched_p = (p - pill_settled) * ANIMATION_NORMALIZED_MAX / (ANIMATION_NORMALIZED_MAX - pill_settled);
          D_out = (int)((int32_t)D_out * stretched_p / ANIMATION_NORMALIZED_MAX);
          D_in = (int)((int32_t)D_in * stretched_p / ANIMATION_NORMALIZED_MAX);
        }
      }

      int bcx = pill_center_x;
      int bcy = pill_center_y;

      int c1x = bcx + (int)((int32_t)sin_lookup(angle) * D_out / TRIG_MAX_RATIO);
      int c1y = bcy - (int)((int32_t)cos_lookup(angle) * D_out / TRIG_MAX_RATIO);
      int c2x = bcx - (int)((int32_t)sin_lookup(angle) * D_in / TRIG_MAX_RATIO);
      int c2y = bcy + (int)((int32_t)cos_lookup(angle) * D_in / TRIG_MAX_RATIO);

      graphics_context_set_fill_color(ctx, GColorVividCerulean);
      graphics_context_set_stroke_color(ctx, GColorVividCerulean);
      graphics_context_set_stroke_width(ctx, r * 2 + 1);

      graphics_fill_circle(ctx, GPoint(c1x, c1y), r);
      graphics_fill_circle(ctx, GPoint(c2x, c2y), r);
      graphics_draw_line(ctx, GPoint(c1x, c1y), GPoint(c2x, c2y));

      graphics_context_set_stroke_width(ctx, 1);
      graphics_context_set_text_color(ctx, GColorWhite);
    } else {
      graphics_context_set_text_color(ctx, label_color);
    }
    graphics_draw_text(ctx, lbuf, label_font, label_rect,
        GTextOverflowModeTrailingEllipsis, label_align, NULL);
  }

  // UP-to-forecast: wrap the fully-drawn clock in the whole-screen squash-stretch.
  if (s_cf->fwd_exit_active) prv_render_fwd_squash(ctx);
}

// ---- Tick handler — redraws every minute to keep hands + time current ----
static void prv_tick_handler(struct tm *tick_time, TimeUnits units_changed) {
  if (s_cf && s_cf->canvas) layer_mark_dirty(s_cf->canvas);
}

// ---- Touch input (touch colour platforms) ----
#if WEATHER_PLATFORM_TOUCH_COLOR
static void prv_touch_handler(const TouchEvent *event, void *context) {
  (void)context;
  if (!s_cf) return;

  if (event->type == TouchEvent_Touchdown) {
    s_cf->touch_start_x = event->x;
    s_cf->touch_start_y = event->y;
    s_cf->touch_active  = true;
    s_cf->touch_started_during_intro =
        s_cf->anim_progress < ANIMATION_NORMALIZED_MAX;
  } else if (event->type == TouchEvent_Liftoff && s_cf->touch_active) {
    s_cf->touch_active = false;
    if (s_cf->touch_started_during_intro) {
      s_cf->touch_started_during_intro = false;
      prv_note_clock_interaction();
      return;
    }

    int16_t dx = event->x - s_cf->touch_start_x;
    int16_t dy = event->y - s_cf->touch_start_y;
    int16_t adx = dx < 0 ? -dx : dx;
    int16_t ady = dy < 0 ? -dy : dy;
    if (adx <= SWIPE_THRESHOLD && ady <= SWIPE_THRESHOLD) {
      // Tap — reveal the per-hour temperatures (mirrors SELECT).
      prv_toggle_temp_reveal();
    } else if (ady >= adx && dy > 0) {
      // Swipe down — return to the forecast main screen (mirrors the UP button).
      if (!s_cf->fwd_exit_active && !s_cf->reveal_active &&
          s_cf->anim_progress >= ANIMATION_NORMALIZED_MAX) {
        prv_start_fwd_exit_animation();
      }
    }
    prv_note_clock_interaction();
  }
}
#endif

// ---- Button input ----
static void prv_click_back(ClickRecognizerRef r, void *ctx) {
  prv_note_clock_interaction();
  app_window_stack_pop_all(true);  // BACK exits the app from any ring page
}
static void prv_click_select(ClickRecognizerRef r, void *ctx) {
  // Single click — reveal the per-hour temperatures (mirrors the touch tap).
  prv_note_clock_interaction();
  prv_toggle_temp_reveal();
}
static void prv_click_down(ClickRecognizerRef r, void *ctx) {
  // The clock is the bottom of the vertical stack now that the detail drill-in is gone, so DOWN
  // has nothing below it — just keep the clock awake.
  prv_note_clock_interaction();
}
static void prv_click_up(ClickRecognizerRef r, void *ctx) {
  prv_note_clock_interaction();
  // UP returns to the forecast main screen: the clock jelly-stretches down off the bottom, then
  // the forecast jelly-drops in from above (handoff fires from prv_fwd_exit_push_callback). Ignore
  // while the intro or any other transition is still in flight.
  if (s_cf && !s_cf->fwd_exit_active &&
      !s_cf->reveal_active && s_cf->anim_progress >= ANIMATION_NORMALIZED_MAX) {
    prv_start_fwd_exit_animation();
  }
}
static void prv_click_provider(void *ctx) {
  window_single_click_subscribe(BUTTON_ID_BACK,   prv_click_back);
  window_single_click_subscribe(BUTTON_ID_UP,     prv_click_up);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_click_select);
  window_single_click_subscribe(BUTTON_ID_DOWN,   prv_click_down);
}

// ---- Window lifecycle ----
static void prv_window_load(Window *window) {
  // ROUND (gabbro smoothness step 3): transparent — the root-layer proc fills the whole
  // window with the bg colour every frame unless transparent, and prv_canvas_draw always
  // paints every pixel (its own white fill / the squash resample). See forecast_list's
  // matching change. The !s_cf early-return keeps a white guard.
  window_set_background_color(window, PBL_IF_ROUND_ELSE(GColorClear, GColorWhite));
  GRect bounds = layer_get_bounds(window_get_root_layer(window));
  s_cf->canvas = layer_create(bounds);
  layer_set_update_proc(s_cf->canvas, prv_canvas_draw);
  layer_add_child(window_get_root_layer(window), s_cf->canvas);

  s_cf->bar_font = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
#if PBL_DISPLAY_HEIGHT >= 200
  s_cf->temp_font = fonts_get_system_font(FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM);
#else
  s_cf->temp_font = fonts_get_system_font(FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM);
#endif

  prv_load_bitmaps();
  tick_timer_service_subscribe(MINUTE_UNIT, prv_tick_handler);

  if (s_cf->skip_intro_animation) {
    s_cf->anim_progress = ANIMATION_NORMALIZED_MAX;
    if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
  } else {
    // Schedule intro animation: bloom + clockwise spin into position (no HUD bar)
    prv_play_animation();
  }
}

static void prv_window_appear(Window *window) {
  if (!s_cf) return;
  // Reset the temperature reveal so the clock shows weather icons on return.
  s_cf->reveal_active = false;
  s_cf->temps_shown = false;
  s_cf->reveal_p = 0;
  if (s_cf->reveal_anim) {
    animation_unschedule(s_cf->reveal_anim);
    animation_destroy(s_cf->reveal_anim);
    s_cf->reveal_anim = NULL;
  }
  if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
#if WEATHER_PLATFORM_TOUCH_COLOR
  touch_service_subscribe(prv_touch_handler, s_cf);
#endif
  prv_note_clock_interaction();
}

static void prv_window_unload(Window *window) {
  if (s_cf->glow_timer) {
    app_timer_cancel(s_cf->glow_timer);
    s_cf->glow_timer = NULL;
  }
#if PBL_ROUND
  if (s_cf->glow_drv) {
    Animation *a = s_cf->glow_drv;
    s_cf->glow_drv = NULL;
    animation_unschedule(a);
    animation_destroy(a);
  }
#endif
  if (s_cf->intro_anim) {
    animation_unschedule(s_cf->intro_anim);
    animation_destroy(s_cf->intro_anim);
    s_cf->intro_anim = NULL;
  }
  if (s_cf->reveal_anim) {
    animation_unschedule(s_cf->reveal_anim);
    animation_destroy(s_cf->reveal_anim);
    s_cf->reveal_anim = NULL;
  }
  if (s_cf->fwd_exit_anim) {
    animation_unschedule(s_cf->fwd_exit_anim);
    animation_destroy(s_cf->fwd_exit_anim);
    s_cf->fwd_exit_anim = NULL;
  }
  if (s_cf->fwd_push_timer) {
    app_timer_cancel(s_cf->fwd_push_timer);
    s_cf->fwd_push_timer = NULL;
  }
  if (s_cf->fwd_scratch) {
    free(s_cf->fwd_scratch);
    s_cf->fwd_scratch = NULL;
  }
  if (s_cf->temp_text_bmp) {
    gbitmap_destroy(s_cf->temp_text_bmp);
    s_cf->temp_text_bmp = NULL;
  }
  tick_timer_service_unsubscribe();

#if WEATHER_PLATFORM_TOUCH_COLOR
  touch_service_unsubscribe();
#endif

  for (int i = 0; i < NUM_TYPE_SLOTS; i++) {
    if (s_cf->type_bitmaps[i]) {
      gbitmap_destroy(s_cf->type_bitmaps[i]);
      s_cf->type_bitmaps[i] = NULL;
    }
  }
  layer_destroy(s_cf->canvas);
  s_cf->canvas = NULL;
  window_destroy(window);
  free(s_cf);
  s_cf = NULL;
}

// ---- Public API ----

void clock_face_set_wrap_callback(ClockFaceWrapCallback callback, void *context) {
  s_wrap_callback = callback;
  s_wrap_context = context;
}

bool clock_face_is_showing(void) {
  return s_cf != NULL;
}

static void prv_clock_face_push(const WeatherLocationForecast *days, size_t num_days,
                                int day_index, const uint8_t *hourly_types,
                                size_t hourly_count, bool animated, bool play_intro) {
  if (s_cf) return;

  s_cf = calloc(1, sizeof(ClockFaceData));
  if (!s_cf) return;

  size_t n = num_days < MAX_DAYS ? num_days : MAX_DAYS;
  s_cf->num_days = n;
  for (size_t i = 0; i < n; i++) s_cf->days[i] = days[i];
  s_cf->day_index = (day_index >= 0 && (size_t)day_index < n) ? day_index : 0;
  s_cf->skip_intro_animation = !play_intro;

  // Seed hourly data so it's available immediately on first draw.
  if (hourly_count > 0) {
    size_t hn = hourly_count < 24 ? hourly_count : 24;
    for (size_t i = 0; i < hn; i++) s_cf->hourly_types[i] = hourly_types[i];
    s_cf->hourly_valid = (hn >= 24);
  }

  s_cf->window = window_create();
  window_set_background_color(s_cf->window, GColorBlack);
  window_set_window_handlers(s_cf->window, (WindowHandlers){
    .load   = prv_window_load,
    .unload = prv_window_unload,
    .appear = prv_window_appear,
  });
  window_set_click_config_provider(s_cf->window, prv_click_provider);
  window_stack_push(s_cf->window, animated);
}

void clock_face_push(const WeatherLocationForecast *days, size_t num_days,
                     int day_index, const uint8_t *hourly_types, size_t hourly_count,
                     bool animated) {
  prv_clock_face_push(days, num_days, day_index, hourly_types, hourly_count,
                      animated, true);
}

void clock_face_push_static(const WeatherLocationForecast *days, size_t num_days,
                            int day_index, const uint8_t *hourly_types,
                            size_t hourly_count, bool animated) {
  prv_clock_face_push(days, num_days, day_index, hourly_types, hourly_count,
                      animated, false);
}

void clock_face_dismiss(bool animated) {
  if (!s_cf || !s_cf->window) return;
  window_stack_remove(s_cf->window, animated);
}

void clock_face_update_hourly_temps_for_day(int day_index, const int8_t *temps, size_t count) {
  if (!s_cf || day_index != s_cf->day_index) return;
  size_t n = count < 24 ? count : 24;
  for (size_t i = 0; i < n; i++) s_cf->hourly_temps[i] = temps[i];
  s_cf->hourly_temps_valid = (n >= 24);
  if (s_cf->canvas) layer_mark_dirty(s_cf->canvas);
}
