/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "forecast_list.h"
#include "weather.h"
#include "expanded_view.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/status_bar_layer.h"   // status-bar height/offset constants (top time)
#include "weather_math.h"
#include "resource_ids.pin.h"
#include "pbl/services/clock.h"            // clock_copy_time_string — notification-style top time
#include "applib/graphics/gdraw_command_transforms.h"
#include "applib/graphics/gdraw_command.h"
#include "applib/graphics/gdraw_command_list.h"
#include "applib/graphics/gdraw_command_image.h"
#include "applib/graphics/gpath.h"          // octagonal animated-sun body (matches static PDC)
#include "applib/applib_malloc.auto.h"
#include "applib/ui/animation.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/ui/animation_timing.h"
#include "pbl/util/math_fixed.h"
#include "pbl/util/trig.h"

#include <time.h>

// The redesigned animated 5-day forecast — today header (large animated condition icon +
// current temp / date / phrase), the day fan, and the hi/lo dot graph — renders on the
// round watches (gabbro/getafix, PBL_ROUND) AND on emery/obelix (rectangular, CONFIG_TOUCH).
// Other rect platforms keep the condensed scrolling list. The geometry inside the shared
// draw branches on PBL_ROUND for the round-circle vs the emery-rectangle layout.
#define WEATHER_ANIM_5DAY WEATHER_PLATFORM_TOUCH_COLOR

#define MAX_ROWS         7
#define ROWS_VISIBLE     4
#define ICON_SIZE        25
#define ICON_X           8
#define DAY_LABEL_X      50
#define DAY_LABEL_W      PBL_IF_ROUND_ELSE(78, 34)
#define DAY_CONDITION_DIVIDER_GAP 4
#define CONDITION_TEXT_LEFT_GAP 11
#define CONDITION_RIGHT_MARGIN 4
#define ANIM_MS          200
#define LOCATION_BAR_H   18
#define LOCATION_BAR_Y   PBL_IF_ROUND_ELSE(20, 0)
#define LIST_TOP_Y       (LOCATION_BAR_Y + LOCATION_BAR_H)
#define ROUND_SIDE_INSET PBL_IF_ROUND_ELSE(28, 0)
#define ROUND_BAR_INSET  PBL_IF_ROUND_ELSE(44, 0)

#if defined(PBL_PLATFORM_GABBRO)
#define GABBRO_LIST_ROW_PITCH      45
#define GABBRO_LIST_FOCUSED_HEIGHT 55
#define GABBRO_LIST_ICON_SIZE      30
#define GABBRO_LIST_ICON_RADIUS    17
#define GABBRO_LIST_FOCUS_RADIUS   19
#define GABBRO_LIST_CENTER_Y_SHIFT 0
#define GABBRO_LIST_ICON_BASE_X    10
#define GABBRO_LIST_TEXT_GAP       11
#define GABBRO_LIST_EDGE_PAD       8
#define GABBRO_LIST_TOP_FOCUS_GAP  21
#define GABBRO_LIST_BOTTOM_FOCUS_MARGIN 48
#define GABBRO_LIST_CURVE_BOOST_DIVISOR 800
#define GABBRO_LIST_TOP_REST_VISIBLE_ROWS 4
#define GABBRO_CAP_DEPTH           58
#define GABBRO_CAP_RADIUS          207
#define GABBRO_CAP_Y_ADJUST        -13
#define GABBRO_CAP_TEXT_INSET      50
#define GABBRO_CAP_TIME_Y          2
#define GABBRO_CAP_LOCATION_Y      21
#endif

#define prv_weather_bg_color weather_type_bg_color
#if defined(PBL_PLATFORM_GABBRO)
#define prv_icon_res weather_type_icon_clock_resource
#else
#define prv_icon_res weather_type_icon_tiny_resource
#endif

// ---- State ----

typedef struct {
  Window  *window;
  Layer   *canvas;
  WeatherLocationForecast days[MAX_ROWS];
  size_t   num_days;
  int      scroll_offset_px;
  int      scroll_from;
  int      scroll_to;
  Animation *anim;
  GBitmap *icons[MAX_ROWS];
  GFont    day_font;
  GFont    condition_font;
  GFont    loc_font;
  char     condition_str[MAX_ROWS][24];  // pre-formatted condition strings
  int      row_height;  // computed at load: (screen_h - LOCATION_BAR_H) / ROWS_VISIBLE
#if defined(PBL_PLATFORM_GABBRO)
  GDrawCommandSequence *icon_sequence;
#endif
#if WEATHER_ANIM_5DAY
  GDrawCommandImage *pdc_icons[MAX_ROWS];  // official PebbleOS weather PDC icons (round 5-day)
  GDrawCommandImage *today_pdc;            // larger today icon for the summary header
  AppTimer *sun_timer;                     // drives the today sun-ray swirl (only when Sun)
#if PBL_ROUND
  // Frame-coalesced icon driver (gabbro): an INFINITE animation on the animation service
  // replaces the free-running 80ms app_timer, so icon stepping shares the SAME service tick
  // as every transition — at most one render per interval. The 80ms visual cadence is kept
  // as quanta inside the update (display research mixed timer cadences beat
  // against the ~20-25ms display period and read as judder).
  Animation *icon_drv;
  uint32_t   icon_drv_last_ms;
  uint32_t   icon_drv_acc_ms;
#endif
  int32_t   sun_phase;                     // ray rotation angle (0..TRIG_MAX_ANGLE)
  int       idle_ticks;                    // animation ticks since the last interaction
  int       icon_fade;                     // R5_FADE_MAX = animating … 0 = frozen static icon
  // Cached so the 40ms animation redraw never re-runs expensive text layout (which was
  // starving the system task that services touch). Recomputed only on load / data update.
  int16_t   today_icon_x;                  // today icon left-x (centred over the date string)
  void     *today_cond_font;               // GFont for today's condition phrase
  bool      today_header_dirty;            // recompute today_icon_x/font (set on data change)
  char      daydate_cache[24];             // "Sat, Jul 04" — refreshed at local midnight
  char      weekday_cache[PBL_IF_ROUND_ELSE(6, 5)][8];   // fan weekday labels ([R5_MAX_COLS])
  time_t    date_cache_expiry;             // next local midnight (0 = caches not built yet)
  // SELECT expand: the today icon squash-stretches off-left before the condensed view is pushed.
  bool      select_exiting;
  int32_t   select_exit_p;                 // 0..ANIMATION_NORMALIZED_MAX
  // UP-to-card "hero" transition: today's icon is lifted out of the squash-down and instead
  // scale-segment zooms from its header rect to the expanded card's 80x80 icon rect (the exact
  // Timeline pin->card icon zoom), synced to the squash animation's progress.
  bool               flying_icon;
  GRect              fly_src;               // today icon rect on the mainscreen (screen coords)
  GRect              fly_dest;              // card icon rect (screen coords; matches expanded_view)
  GDrawCommandImage *fly_pdc;              // LARGE weather PDC (writable clone), owned
  GPointIndexLookup *fly_lookup;           // delay-by-distance lookup — deterministic for the
                                           // whole fly, built once on its first frame, owned
  char               fly_sunset[20];        // card glance data, slid in from the left during the fly
  char               fly_temp[16];          // (synced to the icon landing) so the card can then
  int                fly_uv;                // appear static — see forecast_list_set_glance
  int                fly_precip;
  int                fly_wind;
  // DOWN/UP "header scroll" transition (emery / !PBL_ROUND only): today header slides UP off
  // the top while the 5-day section rises to take over the top, driven by the SAME 330ms
  // interpolate_moook_soft curve Pebble Timeline uses for its card slide (identical bounce).
  bool      header_shown;                  // true = State A (header visible); false = State B
  int       header_scroll;                 // px the header is currently translated UP (0..84)
  int       header_from;                   // moook animation start value for header_scroll
  int       header_to;                     // moook animation end value for header_scroll
  bool      header_hasted;                 // transition started mid-flight (Timeline "hasted")
  // Timeline per-icon transition effects (squash/stretch + zoom lines). Captured straight from
  // the running Animation each frame so they share the EXACT moook normalized that drives the
  // slide — the same source Timeline feeds gdraw_command_*_scale_segmented_to + its speed lines.
  bool      fx_active;                     // a header transition is currently animating
  int32_t   fx_progress;                   // raw AnimationProgress 0..ANIMATION_NORMALIZED_MAX
  GPointIndexLookup *jelly_lookup[6];      // per-icon delay-by-distance lookups ([0]=today,
                                           // [1+i]=fan icon i) — deterministic per transition,
                                           // built once on its first frame, freed on start/stop
  // Clock-burst transition (DOWN again while already scrolled). Stage 1 = whole section exits up
  // off the top while the bottom nav dot decouples and flies to screen centre; Stage 2 = the
  // centred dot orbital-shakes; then it hands off to the clock face.
  int       clock_fx;                      // 0 none, 1 stage-1 (exit + fly), 2 stage-2 (shake)
  int       fx_dot_y;                      // burst dot centre y (flies R5_DOT_REST_Y -> centre)
  int       fx_shake_dx, fx_shake_dy;      // stage-2 orbital shake offset from centre
  Animation *clock_anim;                   // separate from `anim` so the two never collide
  // SELECT -> weather report transition: squash-left, ball roll-in, paper unfold.
  int       report_fx;                     // 0 none, 1 squash-left, 2 ball roll, 3 unfold+hold
  int       fx_ball_x;                     // ball centre x (rolls W+14 -> W/2, y = R5_SCREEN_CY)
  AnimationProgress report_p;              // raw stage-3 progress (unfold_p = clip(p*6/5))
  int32_t   report_angle;                  // frozen unfold fan start-ray (picked at stage-3 arm)
  // ---- Whole-screen jelly squash-stretch (clock exit/return, card, SELECT exit). Shared:
  // weather_render_squash addresses rows absolutely and clamps to each row's chord, so it
  // is correct on the round framebuffer too. ----
  AnimationProgress squash_in_p;     // 0 → MAX
  Animation        *squash_in_anim;
  int               squash_mode;     // 0 = inactive; else SQUASH_DROP_IN / _DOWN_EXIT / _RISE_IN
  uint8_t          *squash_scratch;  // full-screen snapshot, re-sampled while overwriting the fb
  bool              squash_captured; // ROUND capture-once: snapshot filled — later frames skip
                                     // the scene render + fb copy and only resample (step 2)
  bool              refresh_deferred; // BOTH shapes: a phone sync landed mid-report-scene;
                                      // the heavy visual reload waits for the scene end (see
                                      // forecast_list_update_data / prv_apply_deferred_refresh)
#endif
#if WEATHER_PLATFORM_TOUCH_COLOR
  int16_t  touch_start_x, touch_start_y;   // Touchdown origin for swipe detection
  bool     touch_active;
#endif
  void   (*on_pop_cb)(void *ctx);
  void    *on_pop_ctx;
  void   (*on_city_select_cb)(void *ctx);
  void    *on_city_select_ctx;
  void   (*on_clock_request_cb)(void *ctx);   // burst transition -> push the clock face
  void    *on_clock_request_ctx;
  void   (*on_up_request_cb)(void *ctx);   // UP-at-top squash-out done -> push the expanded card
  void    *on_up_request_ctx;
  void   (*on_select_request_cb)(void *ctx);  // SELECT on the resting main view -> open the condensed view
  void    *on_select_request_ctx;
  int      start_day_index;  // focused day from main view; list opens scrolled here
  // First-entry clock-slot intro: the ACTIVE LOCATION holds the top clock slot for
  // 2s, then swaps out with the sunset card's own status-bar swap (both slide right,
  // the time trailing in from the left). Latched once per app run.
  bool     clock_loc_show;
  bool     clock_loc_done;
  bool     clock_swap_active;
  AnimationProgress clock_swap_p;
  Animation *clock_swap_anim;
  AppTimer  *clock_loc_timer;
} ForecastListData;

static ForecastListData *s_list;

// One 4x4 ordered-dither (bayer) table for every stipple on this screen
// (icon crossfade).
static const uint8_t s_bayer4[4][4] = {
  {  0,  8,  2, 10 }, { 12,  4, 14,  6 }, {  3, 11,  1,  9 }, { 15,  7, 13,  5 },
};

#if !PBL_ROUND
static void prv_clock_loc_city(char *dst, size_t dst_size, const char *src);  // defined below
#endif

// ---- Helpers ----

static int prv_max_scroll(void) {
#if WEATHER_ANIM_5DAY
  return 0;  // round 5-day view is a single static screen — no scrolling
#elif defined(PBL_PLATFORM_GABBRO)
  int extra = (int)s_list->num_days - 1;
  return extra > 0 ? extra * s_list->row_height : 0;
#else
  int extra = (int)s_list->num_days - ROWS_VISIBLE;
  return extra > 0 ? extra * s_list->row_height : 0;
#endif
}

static void prv_fill_weekday_label(int day_index, const char *fallback,
                                   char *buffer, size_t buffer_size) {
  if (!buffer || buffer_size == 0) return;

#if defined(PBL_PLATFORM_GABBRO)
  if (fallback && fallback[0]) {
    snprintf(buffer, buffer_size, "%s", fallback);
    for (char *c = buffer; *c; c++) {
      if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
    }
  } else {
    snprintf(buffer, buffer_size, "---");
  }
  return;
#else
  weather_fill_weekday_abbrev(day_index, fallback, buffer, buffer_size);
#endif
}

#if WEATHER_ANIM_5DAY
#define R5_TODAY_ICON 40   // scaled size of the today-summary header icon

// Official PebbleOS weather PDC icon (Pebble_25x25_*) for each app WeatherType.
static uint32_t prv_official_weather_pdc_res(WeatherType t) {
  switch (t) {
    case WeatherType_Sun:          return RESOURCE_ID_SUNNY_DAY_TINY;
    case WeatherType_PartlyCloudy: return RESOURCE_ID_PARTLY_CLOUDY_TINY;
    case WeatherType_CloudyDay:    return RESOURCE_ID_CLOUDY_DAY_TINY;
    case WeatherType_LightRain:    return RESOURCE_ID_LIGHT_RAIN_TINY;
    case WeatherType_HeavyRain:    return RESOURCE_ID_HEAVY_RAIN_TINY;
    case WeatherType_LightSnow:    return RESOURCE_ID_LIGHT_SNOW_TINY;
    case WeatherType_HeavySnow:    return RESOURCE_ID_HEAVY_SNOW_TINY;
    case WeatherType_RainAndSnow:  return RESOURCE_ID_RAINING_AND_SNOWING_TINY;
    default:                       return RESOURCE_ID_GENERIC_WEATHER_TINY;
  }
}

// Larger (Pebble_50x50_*) official PDC, scaled down for the today-summary header.
static uint32_t prv_official_weather_pdc_res_small(WeatherType t) {
  switch (t) {
    case WeatherType_Sun:          return RESOURCE_ID_SUNNY_DAY_SMALL;
    case WeatherType_PartlyCloudy: return RESOURCE_ID_PARTLY_CLOUDY_SMALL;
    case WeatherType_CloudyDay:    return RESOURCE_ID_CLOUDY_DAY_SMALL;
    case WeatherType_LightRain:    return RESOURCE_ID_LIGHT_RAIN_SMALL;
    case WeatherType_HeavyRain:    return RESOURCE_ID_HEAVY_RAIN_SMALL;
    case WeatherType_LightSnow:    return RESOURCE_ID_LIGHT_SNOW_SMALL;
    case WeatherType_HeavySnow:    return RESOURCE_ID_HEAVY_SNOW_SMALL;
    case WeatherType_RainAndSnow:  return RESOURCE_ID_RAINING_AND_SNOWING_SMALL;
    default:                       return RESOURCE_ID_GENERIC_WEATHER_SMALL;
  }
}
#endif


static WeatherType prv_header_type(void);
static void prv_load_icons(void) {
#if WEATHER_ANIM_5DAY
  s_list->today_header_dirty = true;   // today's date/condition changed → recompute layout
#endif
#if defined(PBL_PLATFORM_GABBRO)
  return;
#elif WEATHER_ANIM_5DAY
  // Round 5-day uses the official PebbleOS weather PDC icons, cloned into RAM
  // (a system-app resource is mmap'd READ-ONLY, so draw of the raw would fault).
  for (int i = 0; i < (int)s_list->num_days; i++) {
    if (s_list->pdc_icons[i]) {
      gdraw_command_image_destroy(s_list->pdc_icons[i]);
      s_list->pdc_icons[i] = NULL;
    }
    GDrawCommandImage *raw = gdraw_command_image_create_with_resource(
        prv_official_weather_pdc_res(s_list->days[i].current_weather_type));
    s_list->pdc_icons[i] = raw ? gdraw_command_image_clone(raw) : NULL;
    if (raw) gdraw_command_image_destroy(raw);
  }
  // Larger today icon for the summary header: official SMALL (50px) PDC scaled down.
  if (s_list->today_pdc) {
    gdraw_command_image_destroy(s_list->today_pdc);
    s_list->today_pdc = NULL;
  }
  if (s_list->num_days > 0) {
    GDrawCommandImage *traw = gdraw_command_image_create_with_resource(
        prv_official_weather_pdc_res_small(prv_header_type()));
    if (traw) {
      s_list->today_pdc = gdraw_command_image_clone(traw);
      gdraw_command_image_destroy(traw);
      if (s_list->today_pdc) {
        gdraw_command_image_scale(s_list->today_pdc, GSize(R5_TODAY_ICON, R5_TODAY_ICON));
      }
    }
  }
#else
  for (int i = 0; i < (int)s_list->num_days; i++) {
    if (s_list->icons[i]) {
      gbitmap_destroy(s_list->icons[i]);
      s_list->icons[i] = NULL;
    }
    s_list->icons[i] = gbitmap_create_with_resource(
        prv_icon_res(s_list->days[i].current_weather_type));
  }
#endif
}

#if defined(PBL_PLATFORM_GABBRO)
static void prv_draw_gabbro_pdc_icon(GContext *ctx, WeatherType weather_type,
                                     GPoint origin) {
  if (!s_list) return;
  if (!s_list->icon_sequence) {
    s_list->icon_sequence =
        gdraw_command_sequence_create_with_resource(RESOURCE_ID_WEATHER_CLOCK_ICONS_PDC);
    if (!s_list->icon_sequence) return;
  }

  int frame_index = (int)weather_type;
  if (frame_index > (int)WeatherType_RainAndSnow) {
    frame_index = WeatherType_Generic;
  }

  GDrawCommandFrame *frame =
      gdraw_command_sequence_get_frame_by_index(s_list->icon_sequence, frame_index);
  if (frame) {
    gdraw_command_frame_draw(ctx, s_list->icon_sequence, frame, origin);
  }
}
#endif

// BOTH SHAPES : the mainscreen header
// shows the CURRENT-HOUR conditions — type/temp/phrase derived from the hourly arrays at
// the location's local hour, falling back to the synced current when the record has no
// hourly block. The clock centre reads the same field so the two can never disagree.
static WeatherType prv_header_type(void) {
  return s_list->days[0].current_type_now;
}

// Pre-format condition strings so the draw proc does zero formatting work.
static void prv_format_conditions(void) {
  for (int i = 0; i < (int)s_list->num_days; i++) {
    const WeatherLocationForecast *f = &s_list->days[i];
    const char *phrase = (i == 0)
        ? f->current_phrase_now   // current-hour phrase, both shapes (falls back to synced)
        : f->current_weather_phrase;
    const char *condition = (phrase && phrase[0]) ? phrase : "--";
    snprintf(s_list->condition_str[i], sizeof(s_list->condition_str[i]), "%s", condition);
  }
}

// ---- Scroll animation ----

static void prv_anim_update(Animation *anim, AnimationProgress progress) {
  if (!s_list) return;
  int delta = s_list->scroll_to - s_list->scroll_from;
  int offset = s_list->scroll_from +
      weather_scale_i32(delta, progress, ANIMATION_NORMALIZED_MAX);

  s_list->scroll_offset_px = offset;
  layer_mark_dirty(s_list->canvas);
}

static const AnimationImplementation s_anim_impl = { .update = prv_anim_update };

static void prv_scroll_to_ms(int target, uint32_t duration_ms, bool button_scroll) {
  if (!s_list) return;
  if (target < 0) target = 0;
  if (target > prv_max_scroll()) target = prv_max_scroll();
  if (s_list->anim) {
    animation_unschedule(s_list->anim);
    animation_destroy(s_list->anim);
    s_list->anim = NULL;
  }
  s_list->scroll_from = s_list->scroll_offset_px;
  s_list->scroll_to   = target;
  if (s_list->scroll_from == s_list->scroll_to) return;
  if (duration_ms == 0) {
    // Instant: jump directly with no animation
    s_list->scroll_offset_px = target;
    layer_mark_dirty(s_list->canvas);
    return;
  }
  Animation *a = animation_create();
  animation_set_duration(a, duration_ms);
  animation_set_curve(a, AnimationCurveEaseOut);  // decelerate for fling feel
  animation_set_implementation(a, &s_anim_impl);
  animation_schedule(a);
  s_list->anim = a;
}

static void prv_scroll_to(int target) {
  prv_scroll_to_ms(target, ANIM_MS, false);
}

#if defined(PBL_PLATFORM_GABBRO)
static void prv_scroll_to_menu_repeat(int target) {
  if (!s_list) return;
  if (target < 0) target = 0;
  if (target > prv_max_scroll()) target = prv_max_scroll();

  if (s_list->anim) {
    animation_unschedule(s_list->anim);
    animation_destroy(s_list->anim);
    s_list->anim = NULL;
    s_list->scroll_offset_px = s_list->scroll_to;
  }

  prv_scroll_to_ms(target, 96, false);
}
#endif

// ---- Drawing ----

#if defined(PBL_PLATFORM_GABBRO)
static int prv_round_content_inset(int screen_y, int screen_h) {
  int radius = screen_h / 2;
  int y_offset = screen_y - radius;
  int32_t radius_sq = (int32_t)radius * radius;
  int32_t y_sq = (int32_t)y_offset * y_offset;
  int32_t sqrt_arg = radius_sq - y_sq;
  if (sqrt_arg < 0) sqrt_arg = 0;
  return GABBRO_LIST_ICON_BASE_X + radius - (int)weather_isqrt(sqrt_arg) +
      (int)(y_sq / GABBRO_LIST_CURVE_BOOST_DIVISOR);
}

static int prv_gabbro_focus_y_for_scroll(int scroll_offset, int screen_h) {
  int max_scroll = prv_max_scroll();
  int top_y = GABBRO_CAP_DEPTH + GABBRO_LIST_TOP_FOCUS_GAP;
  int bottom_y = screen_h - GABBRO_LIST_BOTTOM_FOCUS_MARGIN;
  if (bottom_y < top_y) bottom_y = top_y;
  if (max_scroll <= 0) return top_y;
  if (scroll_offset < 0) scroll_offset = 0;
  if (scroll_offset > max_scroll) scroll_offset = max_scroll;
  return top_y + weather_scale_i32(bottom_y - top_y, scroll_offset, max_scroll);
}

static void prv_draw_gabbro_cap(GContext *ctx, int w) {
  int hide_px = 0;
  int hide_distance = s_list->row_height * 2;
  if (hide_distance > 0) {
    hide_px = (int)((int32_t)s_list->scroll_offset_px * GABBRO_CAP_DEPTH /
                    hide_distance);
    if (hide_px > GABBRO_CAP_DEPTH) hide_px = GABBRO_CAP_DEPTH;
  }

  int center_y = GABBRO_CAP_DEPTH - GABBRO_CAP_RADIUS +
                 GABBRO_CAP_Y_ADJUST - hide_px;
  if (center_y + GABBRO_CAP_RADIUS <= 0) return;

  graphics_context_set_fill_color(ctx, GColorVividCerulean);
  graphics_fill_circle(ctx, GPoint(w / 2, center_y), GABBRO_CAP_RADIUS);

  graphics_context_set_text_color(ctx, GColorWhite);
  GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
  int safe = GABBRO_CAP_TEXT_INSET;

  char time_str[8];
  time_t now = time(NULL);
  struct tm *lt = localtime(&now);
  if (clock_is_24h_style()) {
    strftime(time_str, sizeof(time_str), "%H:%M", lt);
  } else {
    strftime(time_str, sizeof(time_str), "%I:%M", lt);
    if (time_str[0] == '0') memmove(time_str, time_str + 1, sizeof(time_str) - 1);
  }

  graphics_draw_text(ctx, time_str, font,
      GRect(safe, GABBRO_CAP_TIME_Y - hide_px, w - safe * 2, 18),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);

  const char *loc = (s_list->num_days > 0 && s_list->days[0].location_name[0])
                      ? s_list->days[0].location_name : "";
  char city_name[48];
  size_t city_len = 0;
  while (loc[city_len] && loc[city_len] != ',' &&
         city_len < sizeof(city_name) - 1) {
    city_name[city_len] = loc[city_len];
    city_len++;
  }
  while (city_len > 0 && city_name[city_len - 1] == ' ') city_len--;
  city_name[city_len] = '\0';
  graphics_draw_text(ctx, city_name, font,
      GRect(safe, GABBRO_CAP_LOCATION_Y - hide_px, w - safe * 2, 19),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
}

static void prv_canvas_draw_gabbro(Layer *layer, GContext *ctx) {
  GRect bounds = layer_get_bounds(layer);
  int W = bounds.size.w;
  int H = bounds.size.h;
  int off = s_list->scroll_offset_px;
  int focus_y = prv_gabbro_focus_y_for_scroll(off, H) + GABBRO_LIST_CENTER_Y_SHIFT;
  int rh = s_list->row_height;

  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);
  graphics_context_set_compositing_mode(ctx, GCompOpSet);

  for (int i = 0; i < (int)s_list->num_days; i++) {
    if (off <= 0 && i >= GABBRO_LIST_TOP_REST_VISIBLE_ROWS) {
      continue;
    }

    int row_center_y = focus_y + i * rh - off;
    if (row_center_y < -GABBRO_LIST_FOCUSED_HEIGHT ||
        row_center_y > H + GABBRO_LIST_FOCUSED_HEIGHT) {
      continue;
    }

    int delta = row_center_y - focus_y;
    int abs_delta = delta < 0 ? -delta : delta;
    bool focused = abs_delta <= rh / 2;
    const WeatherLocationForecast *f = &s_list->days[i];

    int inset = prv_round_content_inset(row_center_y, H);
    int icon_x = inset;
    int draw_icon_size = GABBRO_LIST_ICON_SIZE;
    int icon_y = row_center_y - draw_icon_size / 2;
    int icon_cx = icon_x + draw_icon_size / 2;
    int icon_r = focused ? GABBRO_LIST_FOCUS_RADIUS : GABBRO_LIST_ICON_RADIUS;

    GColor bg = prv_weather_bg_color(f->current_weather_type);
    if (!gcolor_equal(bg, GColorClear)) {
      graphics_context_set_fill_color(ctx, bg);
      graphics_fill_circle(ctx, GPoint(icon_cx, row_center_y), (uint16_t)icon_r);
    }

    prv_draw_gabbro_pdc_icon(ctx, f->current_weather_type, GPoint(icon_x, icon_y));

    char weekday_label[16];
    prv_fill_weekday_label(i, f->label, weekday_label, sizeof(weekday_label));

    int text_x = icon_x + draw_icon_size + GABBRO_LIST_TEXT_GAP;
    int text_w = W - text_x - inset - GABBRO_LIST_EDGE_PAD;
    if (text_w < 28) text_w = 28;
    graphics_context_set_text_color(ctx, focused ? GColorBlack : GColorDarkGray);

    if (focused) {
      graphics_draw_text(ctx, weekday_label, s_list->day_font,
          GRect(text_x, row_center_y - 29, text_w, 28),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
      graphics_draw_text(ctx, s_list->condition_str[i], s_list->condition_font,
          GRect(text_x, row_center_y - 2, text_w, 22),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
    } else {
      graphics_draw_text(ctx, weekday_label, s_list->loc_font,
          GRect(text_x, row_center_y - 11, text_w, 22),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
    }
  }

  prv_draw_gabbro_cap(ctx, W);
}
#endif

#if WEATHER_ANIM_5DAY
// ---- Single-view animated 5-day forecast --------------------------------------
// All five days fit on one screen (no scrolling): a today header (large animated
// condition icon + current temp / date / phrase), a centred row of weekday + icon
// discs, and a two-line hi/lo dot graph beneath (high temp above each warm dot, low
// below each cool dot). Renders on round (gabbro/getafix, 260x260) AND emery/obelix
// (200x228); the geometry below branches round-circle vs emery-rectangle.
// TODAY-IN-FAN (boss request, 2nd application — the first was demoed and reverted;
// re-applied onto the scroll-interpolated spread): today leads the fan as a 6th, LEFTMOST
// column so the current day anchors the week. All steps are the 5-col steps x4/5, holding
// both the rest span AND the scrolled (precipitation view) span exactly. REVERT: flip the
// round arm to 0 — every original value lives in an #else.
#define R5_TODAY_IN_FAN PBL_IF_ROUND_ELSE(0, 0)   // OFF — tried and reverted
#if R5_TODAY_IN_FAN
#define R5_MAX_COLS     6
#else
#define R5_MAX_COLS     5
#endif
// Column tiers. Round fans them inward as they descend to inscribe the circle; emery
// is a rectangle, so its three tiers share one uniform step (dots sit straight under
// the icons) and pack tighter to fit the 200px width.
// Round step is 42, not 50: scrolled, the row rises to where the circle narrows, and at
// 50 the outer day-name ink (x=30) falls outside the glass (which starts at x=36 that far
// up). 42 pulls the outer columns to x=46 — the widest 5-column spacing that reaches the
// top intact. Emery's rectangle has no such limit, so it keeps 38.
// Round steps re-spread each tier opens to its OWN chord's comfort, not a uniform
// pitch — icons/day-names and highs sit near the equator where the glass is wide (26px of
// slack was going unused), the lows stay at 42 because their row's chord is only 7px clear
// already. The widen amplifies the existing fan-inward-as-it-descends grammar.
// The wide values apply ONLY at the resting mainscreen; the *_SCROLLED values are the
// originals, and the column build interpolates between them on the section scroll so the
// precipitation view's icon row lands PIXEL-IDENTICAL to its pre-spread layout (the spread applies to the
// mainscreen only). On rect both pairs are equal -> the lerp is a no-op.
#if R5_TODAY_IN_FAN
#define R5_COL_STEP_ICON 38                         // 48*4/5 — 6 cols, rest span held
#define R5_COL_STEP_HIGH 40                         // 50*4/5
#define R5_COL_STEP_ICON_SCROLLED 34                // 42*4/5 — scrolled span held too
#define R5_COL_STEP_HIGH_SCROLLED 36                // 45*4/5
#define R5_COL_STEP_LOW  34                         // 42*4/5
#else
#define R5_COL_STEP_ICON PBL_IF_ROUND_ELSE(48, 38)  // icon / day-name row at REST (was 42)
#define R5_COL_STEP_HIGH PBL_IF_ROUND_ELSE(50, 38)  // high temps at REST (was 45)
#define R5_COL_STEP_ICON_SCROLLED PBL_IF_ROUND_ELSE(42, 38)  // precip view: original pitch
#define R5_COL_STEP_HIGH_SCROLLED PBL_IF_ROUND_ELSE(45, 38)
#define R5_COL_STEP_LOW  PBL_IF_ROUND_ELSE(42, 38)  // low temps — chord-bound, unchanged
#endif
#define R5_HEADER_Y     14    // header pinned at top; everything below is centred
#define R5_DAYNAME_Y    PBL_IF_ROUND_ELSE(94, 84)
#define R5_ICON_CY      PBL_IF_ROUND_ELSE(130, 120) // icon row centre-y
#define R5_ICON_SIZE    25
#if R5_TODAY_IN_FAN
#define R5_DISC_R       14   // r17 discs would touch at the 6-col pitch; r14 keeps clear gaps
#else
#define R5_DISC_R       (R5_ICON_SIZE * 7 / 10)  // 17 — same disc as old list + mainscreen
#endif
#define R5_PRECIP_Y     150
#define R5_GRAPH_TOP    PBL_IF_ROUND_ELSE(172, 166) // y of the warmest dot
#define R5_GRAPH_BOT    PBL_IF_ROUND_ELSE(202, 202) // y of the coolest dot
#define R5_DOT_R        5     // outer coloured dot radius
#define R5_DOT_INNER    2     // inner white radius (hollow dot)
#define R5_HAIRLINE_Y   PBL_IF_ROUND_ELSE(88, 80)   // hairline rule under today header
// The emery "today" banner blue. Used by BOTH the banner fill AND the icon crossfade's
// erase-to-background — they MUST stay the same colour or the animated↔static icon fade
// shows a coloured box on the banner. Change only here.
#define R5_TODAY_X      PBL_IF_ROUND_ELSE(68, 18)   // today icon draw offset (emery left margin)
// Emery today icon/temp top pushed down to y=20 — the exact slot a notification puts its top
// icon (STATUS_BAR_LAYER_HEIGHT 20 + CARD_ICON_UPPER_PADDING 0 on emery) — so the top clock has
// the same clearance above the icon that a Gmail-style notification gives. Date row follows so
// the header block stays clear of the moved icon and the hairline below it.
#define R5_TODAY_Y      PBL_IF_ROUND_ELSE(26, 20)
#define R5_DAYDATE_Y    PBL_IF_ROUND_ELSE(68, 62)   // day + date, below the today icon

// DOWN/UP header transition travel (emery / !PBL_ROUND). HEADER_TRAVEL slides the today
// header (icon/temp/date/condition/hairline) fully off the top; SECTION_TRAVEL lifts the
// 5-day fan+graph so its old top (day-names at R5_DAYNAME_Y) lands in the header's vacated
// top slot (R5_TODAY_Y). One scheduled moook animation drives header_scroll across
// [0..HEADER_TRAVEL]; the section offset is a synced scaled copy (see the draw proc), so the
// two groups bounce in perfect lockstep — exactly Timeline's "everything in one spawn" rule.
#define R5_HEADER_TRAVEL  R5_DAYNAME_Y                 // 84: header fully off the top edge
// Extra lift past today's old slot, applied uniformly to the whole scrolled 5-day section
// (day names, discs, graph, icon rest + its stretch travel all key off SECTION_TRAVEL, so they
// stay in lockstep) to free more room at the bottom of the scrolled screen for the new stats.
// Emery lifts PAST today's old slot (+19) so the day names top out at y6. Round cannot:
// at y6 the circle is only ~100px wide. Round lifts 14px SHORT instead, landing the day
// names at y40 and the discs at cy76 — the highest the 4-column row fits uncut.
// Round's scrolled column is SOLVED, not nudged: every element is placed against the
// circle's actual chord at its own top AND bottom edge. Lift 11px short of today's slot
// puts day names at y37 (outer, bowed: 47) and discs at cy73 (outer 83) — the outer ink
// clears the glass by 2px at its tightest point.
#define R5_SCROLL_EXTRA_LIFT PBL_IF_ROUND_ELSE(-8, 19)
#define R5_SECTION_TRAVEL ((R5_DAYNAME_Y - R5_TODAY_Y) + R5_SCROLL_EXTRA_LIFT)  // 82

// Round stops its section lift 14px short (above), so the graph lands lower than on
// emery and the stats band must follow it down or the two collide. Constant, not
// scaled: the band only exists on the scrolled screen (at rest it is off the bottom).
#define R5_BAND_DY PBL_IF_ROUND_ELSE(26, 0)
// Emery packs the values 1px under the pill because it has no spare height. Round does —
// open it to 8px so the percentages read as their own row, not a caption on the pill.
#define R5_BAND_ROW_GAP PBL_IF_ROUND_ELSE(4, 0)
// Scrolled, round's graph must sit LOWER (the bowed outer discs drop into where the high
// labels were — this is what made the temps overlap the icons) and SHORTER (the circle
// closes in below). Both scale with the lift, so the mainscreen graph is untouched.
#define R5_GRAPH_SCROLL_TOP_DY PBL_IF_ROUND_ELSE(-7, 0)
#define R5_GRAPH_SCROLL_BOT_DY PBL_IF_ROUND_ELSE(1, 0)
// How the graph's height splits: each series gets 1/DIV, the air between them gets the
// rest. Emery's /3 leaves only a third between the lines; on round's shorter scrolled
// band that read as one bunched blob, so round gives each series a QUARTER and the gap
// a HALF — the two rows separate without needing height the circle doesn't have.
#define R5_GRAPH_BAND_DIV PBL_IF_ROUND_ELSE(4, 3)
// Round drops the MON/TUE/... labels as the page scrolls: the mainscreen has just named
// the columns, and on the scrolled screen that 16px band is the most expensive real
// estate there is. Stop drawing them once the lift is underway — the rising discs are
// arriving in that band anyway, so the labels are gone before they could be orphaned.
// (Explicit, not relying on the discs to paint over them: a weather type whose disc is
// GColorClear draws no fill, and the labels would show through.)
#define R5_DAYNAME_SCROLL_HIDE PBL_IF_ROUND_ELSE(1, 0)
// The room the vanished labels free, handed to the discs: they rise this much FURTHER
// than the section as it scrolls (0 at rest, so the mainscreen is untouched).
#define R5_DISC_SCROLL_UP PBL_IF_ROUND_ELSE(18, 0)

#if PBL_ROUND
// A SLIGHT bow so the row echoes the glass above it. The circle's true sagitta at the
// 42px column step is 0 / 7 / 31px — using it whole fans the row out hard, so it is
// damped to ~45%: a curve you feel rather than read. Precomputed (no sqrt per frame);
// index is the column's distance from centre, so n<=R5_MAX_COLS keeps it in range.
static const uint8_t s_round_col_bow[3] = { 0, 2, 10 };
// Scaled by the section lift: flat at rest (the row sits at the circle's widest), fully
// bowed once scrolled to the top where the glass actually curves away.
#define R5_BOW_DY(i, n, ss) \
  ((int)s_round_col_bow[(i) < (n) / 2 ? (n) / 2 - (i) : (i) - (n) / 2] \
   * (ss) / R5_SECTION_TRAVEL)
#else
// Rect is a rectangle — no bow. (Round's draw path still compiles here, so this must exist.)
#define R5_BOW_DY(i, n, ss) 0
#endif

// ---- DOWN/UP header transition: the SAME 330ms moook-soft curve Pebble Timeline uses for
// its card slide, applied to one integer "header scroll offset" that the draw proc subtracts
// from both element groups (header up + 5-day up-to-top, in perfect lockstep). ----

// Standard transition: Timeline's interpolate_moook_soft(.,.,.,3) — 3 anticipation frames,
// 3 linear mid frames, then the 4px-overshoot OUT settle. The shared 330ms duration
// (interpolate_moook_soft_duration(3)) keeps the 10 frame slices aligned with this curve.
static int64_t prv_moook_soft3(int32_t n, int64_t from, int64_t to) {
  return interpolate_moook_soft(n, from, to, 3);
}

// Full Timeline moook for the DOWN/UP header transition: the classic
// anticipation wind-back + fast travel + overshoot-and-settle bounce
// (Timeline's pin-list scroll curve). The softer soft(3) stays in use for the
// other transitions (squash screens, hero fly, clock exit).
static int64_t prv_moook_full(int32_t n, int64_t from, int64_t to) {
  return interpolate_moook(n, from, to);
}

// Interrupted ("hasted") transition: when a press lands mid-flight, remap progress into the
// second half of the curve so the re-press resolves fast — exactly Timeline's
// was_stationary==false path (cut = (normalized + MAX) / 2, then the same moook_soft).
static int64_t prv_moook_hasted(int32_t n, int64_t from, int64_t to) {
  int32_t cut = (n + ANIMATION_NORMALIZED_MAX) / 2;
  return interpolate_moook_soft(cut, from, to, 3);
}

// Shared animation boilerplate: create + duration + curve + impl + optional stopped
// handler + schedule. Every moook here runs a manual curve in its update proc, so
// the wrapper just parameterizes the pieces that differ. (Shared region: used by
// both the rect-only squash block below and the round+rect transitions further down.)
static Animation *prv_start_anim(uint32_t dur_ms, AnimationCurve curve,
                                 const AnimationImplementation *impl,
                                 AnimationStoppedHandler stopped) {
  Animation *a = animation_create();
  animation_set_duration(a, dur_ms);
  animation_set_curve(a, curve);
  animation_set_implementation(a, impl);
  if (stopped) {
    animation_set_handlers(a, (AnimationHandlers){ .stopped = stopped }, NULL);
  }
  animation_schedule(a);
  return a;
}

// Shared: round's report stage 3 reuses the same fly slot for the paper clone.
static void prv_clear_fly(void) {
  if (!s_list) return;
  s_list->flying_icon = false;
  if (s_list->fly_pdc) {
    gdraw_command_image_destroy(s_list->fly_pdc);
    s_list->fly_pdc = NULL;
  }
  if (s_list->fly_lookup) {
    applib_free(s_list->fly_lookup);
    s_list->fly_lookup = NULL;
  }
}

// Squash directions (s_list->squash_mode; 0 = inactive). Shared: the renderer is
// chord-clamped, so round drives the same modes.
#define SQUASH_DROP_IN   1   // fall in from above   (returning from the clock)
#define SQUASH_DOWN_EXIT 2   // swoosh down off-bottom (heading up to the expanded card)
#define SQUASH_RISE_IN   3   // rise up from below   (returning from above)
#define SQUASH_UP_EXIT   5   // clock-burst stage-1: jelly-stretch up off the top
#define SQUASH_LEFT_EXIT 6   // select-exit stage-1: jelly-stretch off the LEFT

static void prv_render_squash_in(GContext *ctx) {
  if (!s_list || !s_list->squash_scratch) return;
#if PBL_ROUND
  // Capture-once (gabbro smoothness step 2): the source scene is static during every
  // squash (nub/arrows/dot all exclude themselves or draw after), so the first frame
  // fills the snapshot and every later frame only resamples it — dropping both the
  // full scene render and the 67.6KB framebuffer copy from the per-frame cost.
  if (s_list->squash_captured) {
    weather_render_squash_cached(ctx, s_list->squash_scratch, s_list->squash_in_p,
                                 s_list->squash_mode);
    return;
  }
  s_list->squash_captured = true;
#endif
  weather_render_squash(ctx, s_list->squash_scratch, s_list->squash_in_p,
                        s_list->squash_mode);
}

#if WEATHER_ANIM_5DAY
// ---- Return-from-clock: whole-screen Timeline squash-stretch IN -------------------------
// Squash directions (s_list->squash_mode; 0 = inactive).

// Entry squash armed by weather.c (0 = none) just before the covering window dismisses; consumed
// by prv_window_appear so the very first revealed frame is already off-screen (no rest flash).
static int s_pending_squash_mode;
// Report-BACK entrance (round): the mainscreen slides in from the LEFT while the report page
// slides out to the right — the same WEATHER_HSLIDE_MS / moook_soft1 pair the card<->globe
// glide uses. Armed by weather_report.c just before it pops.
static bool s_pending_hslide_in;
static Animation *s_hslide_anim;
// Reverse hero: armed with the RISE_IN when the expanded card dismisses back to the forecast —
// the card's big icon squash-stretch zooms back down into the today-header spot (the exact
// forward fly played backwards), landing as the rising screen settles.
static bool s_pending_return_fly;

// The whole-screen jelly squash-stretch lives in weather_math.c (weather_render_squash),
// shared with clock_face's forward exit. Modes: SQUASH_DROP_IN falls down from above into
// rest (returning from the clock); SQUASH_DOWN_EXIT swooshes down off the bottom, hasted
// into its first ~75% for the sunset-card text staging; SQUASH_RISE_IN is its reverse.

// Tear down the hero icon-fly state (idempotent).

static void prv_squash_in_update(Animation *anim, AnimationProgress progress) {
  if (!s_list) return;
  s_list->squash_in_p = progress;
  if (progress >= ANIMATION_NORMALIZED_MAX) {
    int done_mode = s_list->squash_mode;
    s_list->squash_mode = 0;
    if (s_list->squash_scratch) { free(s_list->squash_scratch); s_list->squash_scratch = NULL; }
    // The hero icon has landed on the card's icon spot — drop it, then reveal the card (pushed
    // un-animated so its static icon takes over exactly where the flown icon settled).
    prv_clear_fly();
    // Down-exit finished: the forecast has cleared off the bottom — hand off to the expanded card.
    if (done_mode == SQUASH_DOWN_EXIT && s_list->on_up_request_cb) {
      s_list->on_up_request_cb(s_list->on_up_request_ctx);
    }
  }
  if (s_list->canvas) layer_mark_dirty(s_list->canvas);
}
static const AnimationImplementation s_squash_in_impl = { .update = prv_squash_in_update };


static void prv_start_squash(int mode) {
  if (!s_list || !s_list->canvas || s_list->squash_mode) return;
  GRect bounds = layer_get_bounds(s_list->canvas);
  s_list->squash_scratch = malloc_try((size_t)bounds.size.w * (size_t)bounds.size.h);
  if (!s_list->squash_scratch) {                  // out of heap: skip the effect
    if (mode == SQUASH_DOWN_EXIT && s_list->on_up_request_cb) {
      s_list->on_up_request_cb(s_list->on_up_request_ctx);
    }
    return;
  }
  s_list->squash_mode = mode;
  s_list->squash_in_p = 0;
  s_list->squash_captured = false;   // round capture-once: fresh squash, fresh snapshot
  if (s_list->squash_in_anim) {
    animation_unschedule(s_list->squash_in_anim);
    animation_destroy(s_list->squash_in_anim);
  }
  // Slow ONLY the mainscreen->sunset-card exit (SQUASH_DOWN_EXIT) to ~495ms (1.5x) for a more
  // deliberate open; the return squashes (DROP_IN from the clock, RISE_IN from above) keep their
  // original 330ms. More mid frames (8 vs 3) keeps the moook curve one-keyframe-per-render-tick.
  // Every synced part (background jelly, hero icon-fly, glance-text slide) reads squash_in_p, so
  // they all stretch by the same factor and the icon + text still co-land on the final frame.
  s_list->squash_in_anim = prv_start_anim((mode == SQUASH_DOWN_EXIT)
      ? interpolate_moook_soft_duration(8)    // ~495ms
      : interpolate_moook_soft_duration(3),   // 330ms (unchanged)
      AnimationCurveLinear, &s_squash_in_impl, NULL);
  layer_mark_dirty(s_list->canvas);
}

// UP at the top of the resting forecast -> open the expanded card. The whole screen squashes down
// (SQUASH_DOWN_EXIT) as usual, but today's weather icon is lifted OUT of that squash and instead
// scale-segment "zooms" (Timeline pin->card icon animation) from its header rect to the card's
// 80x80 icon rect. On completion the card is revealed with its static icon exactly where the flown
// icon landed. Falls back to a plain squash if the hero icon can't be set up.
#endif  // WEATHER_ANIM_5DAY && !PBL_ROUND

// Shared: the hero icon-fly + DOWN_EXIT squash now work on round too.
static void prv_start_up_to_card(void) {
  if (!s_list || !s_list->canvas || s_list->flying_icon || s_list->squash_mode) return;
  if (!s_list->header_shown || s_list->num_days == 0) { prv_start_squash(SQUASH_DOWN_EXIT); return; }
  // LARGE (80x80) weather PDC — the SAME resource the card draws, so the hand-off is seamless.
  GDrawCommandImage *raw = gdraw_command_image_create_with_resource(
      weather_type_icon_large_resource(prv_header_type()));
  GDrawCommandImage *pdc = raw ? gdraw_command_image_clone(raw) : NULL;
  if (raw) gdraw_command_image_destroy(raw);
  if (!pdc) { prv_start_squash(SQUASH_DOWN_EXIT); return; }
  const int W = layer_get_bounds(s_list->canvas).size.w;
  s_list->fly_pdc   = pdc;
  s_list->fly_src   = GRect(s_list->today_icon_x, R5_TODAY_Y, R5_TODAY_ICON, R5_TODAY_ICON);
  // Must match expanded_view.c's static card icon — shared constants keep the fly landing pixel-exact.
  s_list->fly_dest  = GRect((W - EV_ICON_SIZE) / 2, EV_ICON_Y, EV_ICON_SIZE, EV_ICON_SIZE);
  s_list->flying_icon = true;
  prv_start_squash(SQUASH_DOWN_EXIT);
  if (!s_list->squash_mode) prv_clear_fly();   // squash couldn't start (OOM); cb already fired
}

// Free the cached jelly lookups (on transition start — the travel direction can flip — on
// transition stop, and at unload). The rect-only jelly draw is the only writer; on round these
// stay NULL and this is a no-op.
static void prv_free_jelly_lookups(void) {
  if (!s_list) return;
  for (size_t i = 0; i < sizeof(s_list->jelly_lookup) / sizeof(s_list->jelly_lookup[0]); i++) {
    if (s_list->jelly_lookup[i]) {
      applib_free(s_list->jelly_lookup[i]);
      s_list->jelly_lookup[i] = NULL;
    }
  }
}

static void prv_header_anim_update(Animation *anim, AnimationProgress progress) {
  if (!s_list) return;
  InterpolateInt64Function fn = s_list->header_hasted ? prv_moook_hasted : prv_moook_full;
  s_list->header_scroll = (int)fn(progress, s_list->header_from, s_list->header_to);
  // Stash the RAW normalized so the per-icon squash + the zoom lines run off the exact same
  // moook progress that positions the slide (this is the `normalized` Timeline hands its
  // scale_segmented transforms + speed-line interpolation).
  s_list->fx_progress = progress;
  s_list->fx_active   = true;
  layer_mark_dirty(s_list->canvas);
}

static void prv_header_anim_stopped(Animation *anim, bool finished, void *context) {
  if (!s_list) return;
  // Snap exactly onto the endpoint (mirror Timeline's cut-to-end) and latch the rest state.
  s_list->header_scroll = s_list->header_to;
  s_list->header_shown  = (s_list->header_to == 0);
  s_list->header_hasted = false;
  // At rest there must be NO residual deformation and NO zoom lines.
  s_list->fx_active = false;
  s_list->anim = NULL;
  prv_free_jelly_lookups();   // don't hold the 6 lookups while at rest
  layer_mark_dirty(s_list->canvas);
}

static const AnimationImplementation s_header_anim_impl = { .update = prv_header_anim_update };

static void prv_start_header_transition(bool to_scrolled) {
  if (!s_list) return;
  const int from = s_list->header_scroll;             // current px (supports interruption)
  const int to   = to_scrolled ? R5_HEADER_TRAVEL : 0;
  if (from == to) return;
  // A transition already running => this is an interrupting press (Timeline "hasted").
  s_list->header_hasted = (s_list->anim != NULL);
  if (s_list->anim) {
    // Capture first: unschedule fires .stopped, which NULLs s_list->anim — destroying via the
    // field afterwards destroyed NULL and leaked the unscheduled animation on every haste.
    Animation *old = s_list->anim;
    s_list->anim = NULL;
    animation_unschedule(old);
    animation_destroy(old);
  }
  s_list->header_from = from;
  s_list->header_to   = to;
  prv_free_jelly_lookups();   // travel direction flipped/changed — stale lookups must rebuild
  // Full-moook duration for the standard slide (keeps one curve keyframe per
  // render tick); the hasted remap still rides the soft(3) timing. Linear curve:
  // the manual moook in prv_header_anim_update IS the curve.
  s_list->anim = prv_start_anim(s_list->header_hasted
                                    ? interpolate_moook_soft_duration(3)
                                    : interpolate_moook_duration(),
                                AnimationCurveLinear, &s_header_anim_impl,
                                prv_header_anim_stopped);
  s_list->fx_active   = true;
  s_list->fx_progress = 0;  // start at the from-frame. Without this, the PREVIOUS transition left
                            // fx_progress at MAX, so the first frame (before the anim's first
                            // update) renders the bitmap at its destination/final frame — a flash.
}

// ---- Clock-burst transition: DOWN again while already scrolled. STAGE 1 (Timeline dot-to-centre):
// the scrolled section rides header_scroll PAST its rest (84) so header/fan/graph/stats all exit up
// in lockstep, while the bottom dot decouples and flies to screen centre. STAGE 2 (ported from the
// original burst): the centred dot does the orbital shake, then hands off to the clock. ----
// header_scroll value that clears ALL content off the top. The lift must be shape-specific:
// round's scrolled content sits ~35px lower than emery's (dot rest 233 vs 198), so emery's
// fixed 200 left the low-temp graph line, its labels and the precip %-row stranded at rows
// 3..37 (measured) for the whole of stage 2 — the "graph pops in at the top" bug. It was
// invisible until the stranded-squash_mode fix, because a permanently-squashed frame had been
// masking it. Round uses a FULL SCREEN HEIGHT of lift, which is the robust statement of intent:
// anything drawn inside the visible frame at scrolled rest is guaranteed off-top, whatever the
// temperature spread does to the graph's height. Emery's 200 is untouched.
#define R5_CLOCK_EXIT  (R5_HEADER_TRAVEL + PBL_IF_ROUND_ELSE(PBL_DISPLAY_HEIGHT, 200))
// Screen centre-y — the destination for the burst dot AND the whole report entry scene
// (ball + unfolding paper). Was hardcoded to rect's 228/2, which put round's animation
// 16px high and off the SELECT nub it is supposed to grow out of.
#define R5_SCREEN_CY   PBL_IF_ROUND_ELSE(130, 114)
// Dot's resting y on the scrolled screen. Round: the precip values end at y216 and the
// glass ends at 260, so 238 centres it in that gap (the circle is still 145px wide there).
#define R5_DOT_REST_Y  PBL_IF_ROUND_ELSE(233, 198)

static void prv_start_clock_stage2(void);

static void prv_clock_stage1_update(Animation *anim, AnimationProgress progress) {
  if (!s_list) return;
  // The PAGE exits at DOUBLE TIME (fully off by the 50% mark) while the dot keeps the
  // full-length ease below — the content clears the dot's climb path early and the dot
  // rises the whole second half alone: the focal point the transition is about.
  AnimationProgress pm = (progress > ANIMATION_NORMALIZED_MAX / 2)
                             ? ANIMATION_NORMALIZED_MAX : progress * 2;
  if (s_list->squash_mode == SQUASH_UP_EXIT) {
    // Squash path (BOTH shapes): the scene keeps drawing at the scrolled rest
    // (header_scroll pinned at R5_HEADER_TRAVEL by prv_start_clock_stage1) and the whole
    // frame — precip pill included — jelly-stretches up off the top in
    // prv_render_squash_in. Round used to be forced down the fallback below, which
    // TRANSLATES the content instead of stretching it, so nothing ever squashed.
    s_list->squash_in_p = pm;
  } else {
    // Fallback (round / squash scratch OOM): plain translated exit. CLAMPED at rest:
    // moook_soft3's anticipation frames dip header_scroll below R5_HEADER_TRAVEL,
    // which shoved the pill/%/graph DOWN into the burst dot for a beat — the content
    // must stay clear above the dot throughout. The squash path never dips (its jelly
    // edges are clamped the same way).
    int hsv = (int)prv_moook_soft3(pm, R5_HEADER_TRAVEL, R5_CLOCK_EXIT);
    if (hsv < R5_HEADER_TRAVEL) hsv = R5_HEADER_TRAVEL;
    s_list->header_scroll = hsv;
  }
  // The dot eases cleanly into the centre (ease-out quad — NO moook anticipation dip or overshoot
  // wobble) so it lands exactly at the animation's end and the shake fires the instant it lands.
  const int32_t q = ANIMATION_NORMALIZED_MAX - (int32_t)progress;
  const int32_t eased = ANIMATION_NORMALIZED_MAX - (int32_t)((int64_t)q * q / ANIMATION_NORMALIZED_MAX);
  s_list->fx_dot_y = R5_DOT_REST_Y +
      (R5_SCREEN_CY - R5_DOT_REST_Y) * eased / ANIMATION_NORMALIZED_MAX;
  layer_mark_dirty(s_list->canvas);
}

// BOTH shapes — must match the SQUASH_UP_EXIT arm in prv_clock_stage1_update above. This was
// rect-only while round still took the translate fallback; leaving it gated once round started
// driving UP_EXIT stranded squash_mode at 5 forever, which (a) leaked the full-screen scratch on
// every clock visit and (b) made prv_start_squash refuse the return DROP_IN, so the forecast came
// back permanently squashed off the top — a blank white screen.
static void prv_clock_stage1_end_squash(void) {
  if (s_list->squash_mode != SQUASH_UP_EXIT) return;
  s_list->squash_mode = 0;
  if (s_list->squash_scratch) { free(s_list->squash_scratch); s_list->squash_scratch = NULL; }
}

static void prv_clock_stage1_stopped(Animation *anim, bool finished, void *context) {
  if (!s_list) return;
  s_list->clock_anim = NULL;
  prv_clock_stage1_end_squash();
  if (!finished) {                                 // BACK cancelled — snap back to the scrolled screen
    s_list->clock_fx = 0;
    s_list->header_scroll = R5_HEADER_TRAVEL;
    layer_mark_dirty(s_list->canvas);
    return;
  }
  s_list->header_scroll = R5_CLOCK_EXIT;            // content fully off-top
  s_list->fx_dot_y      = R5_SCREEN_CY;             // dot parked at centre
  prv_start_clock_stage2();
}

static void prv_clock_stage2_update(Animation *anim, AnimationProgress progress) {
  if (!s_list) return;
  // Verbatim port of the original burst's orbital shake (weather_app_layout.c): 4 rotations,
  // bell-curve amplitude 3 -> 7 -> 0 px. Standalone here, so shake_t == progress (no CS rescale).
  int32_t angle = (int32_t)TRIG_MAX_ANGLE * 4 * progress / ANIMATION_NORMALIZED_MAX;
  int32_t amp_n = weather_norm_bell(progress);
  int amp = 3 + (int)(amp_n * 4 / ANIMATION_NORMALIZED_MAX);
  s_list->fx_shake_dx =  (int)((int32_t)sin_lookup(angle) * amp / TRIG_MAX_RATIO);
  s_list->fx_shake_dy = -(int)((int32_t)cos_lookup(angle) * amp / TRIG_MAX_RATIO);
  layer_mark_dirty(s_list->canvas);
}

static void prv_clock_stage2_stopped(Animation *anim, bool finished, void *context) {
  if (!s_list) return;
  s_list->clock_anim  = NULL;
  s_list->fx_shake_dx = s_list->fx_shake_dy = 0;
  if (finished && s_list->on_clock_request_cb) {
    s_list->on_clock_request_cb(s_list->on_clock_request_ctx);  // push the clock (spiral) on top
  }
  // Reset to the resting forecast (State A) BENEATH the now-on-top clock, so backing out of the
  // clock reveals a clean forecast rather than the post-burst frame.
  s_list->clock_fx      = 0;
  s_list->header_scroll = 0;
  s_list->header_shown  = true;
  s_list->fx_active     = false;
  layer_mark_dirty(s_list->canvas);
}

static const AnimationImplementation s_clock_stage1_impl = { .update = prv_clock_stage1_update };
static const AnimationImplementation s_clock_stage2_impl = { .update = prv_clock_stage2_update };

static void prv_start_clock_stage2(void) {
  if (!s_list) return;
  s_list->clock_fx    = 2;
  s_list->fx_shake_dx = s_list->fx_shake_dy = 0;
  // 220ms: legible (the original's shake tail was ~53ms); linear — amplitude self-modulates.
  s_list->clock_anim = prv_start_anim(220, AnimationCurveLinear, &s_clock_stage2_impl,
                                      prv_clock_stage2_stopped);
}

static void prv_start_clock_stage1(void) {
  if (!s_list || s_list->clock_fx) return;   // a burst is already running/parked — ignore re-press
  s_list->clock_fx    = 1;
  s_list->fx_dot_y    = R5_DOT_REST_Y;
  s_list->fx_shake_dx = s_list->fx_shake_dy = 0;
  s_list->fx_active   = false;                     // uniform exit — no per-icon jelly
  if (s_list->anim) {   // capture-first: .stopped NULLs the field during unschedule
    Animation *old = s_list->anim;
    s_list->anim = NULL;
    animation_unschedule(old);
    animation_destroy(old);
  }
  // Whole-frame squash-stretch exit (Timeline jelly): the scene stays drawn at the scrolled
  // rest and SQUASH_UP_EXIT sweeps it — precip pill and all — up off the top. On scratch
  // OOM the update falls back to the plain translated exit. Both shapes.
  if (!s_list->squash_mode && s_list->canvas) {
    GRect bounds = layer_get_bounds(s_list->canvas);
    s_list->squash_scratch = malloc_try((size_t)bounds.size.w * (size_t)bounds.size.h);
    if (s_list->squash_scratch) {
      s_list->squash_mode = SQUASH_UP_EXIT;
      s_list->squash_in_p = 0;
      s_list->squash_captured = false;   // round capture-once
    }
  }
  // 330ms — Timeline feel. LINEAR like every other squash driver: the jelly edges (and the
  // fallback's moook) carry ALL the shaping, so the curve must not pre-ease the progress.
  // The DOUBLE-TIME page exit (pm = 2*progress — everything off by the 50% mark) spends its
  // whole visible run in the first ~165ms of a 330ms stage 1. At the framework's 33ms target
  // that is only ~6 frames, and round has to cover 260px of travel in them (emery 228), so the
  // steps land coarser: measured pm went 25% -> 53% in ONE frame and the graph read as blinking
  // out rather than flying. Round is NOT slow — it renders ~27.5ms/frame, inside the target;
  // the budget is simply too few frames for the taller screen. Lengthening stage 1 on round
  // (330 -> 561ms) buys the visible half ~11 frames, measured. The STAGING RATIO is untouched
  // on both shapes — content is still clear of the dot's climb path by the halfway mark.
  s_list->clock_anim = prv_start_anim(
      interpolate_moook_soft_duration(PBL_IF_ROUND_ELSE(10, 3)),
      AnimationCurveLinear, &s_clock_stage1_impl, prv_clock_stage1_stopped);
}

// ===================================================================================
// SELECT -> weather report: squash-LEFT -> ball roll-in -> Timeline unfold -> hard cut.
// Stage 1 jelly-squashes the whole frame off the left (the clock burst's grammar rotated
// 90°); at 55% the SELECT nub's ball rolls in from the right edge to screen centre
// (stage 2); the ball then becomes the newspaper: the WEATHER_REPORT_PAPER PDC plays the
// Timeline day-separator unfold (stage 3) with "WEATHER REPORT" rising beneath, and the
// report window hard-cuts in on the final frame.
// ===================================================================================

// SELECT-exit contract state — shared with the round/OOM fallback slide below, so these
// three live OUTSIDE the rect gate.
static Animation *s_select_exit_anim;
static void (*s_select_exit_done)(void *ctx);
static void *s_select_exit_ctx;

#if WEATHER_ANIM_5DAY
static Animation *s_report_anim;               // stages 2-4 (stage 2 overlaps stage 1's tail)
#define R5_REPORT_BALL_R      13                                  // = the SELECT nub's radius
// The SELECT nub is fill_oval(GRect(W-5, (H-26)/2, 26, 26)) — a r13 circle centred at
// x = W + 8. The ball starts EXACTLY there, so the indicator visibly becomes the ball.
#define R5_REPORT_BALL_X0(W)  ((W) + 8)
#define R5_REPORT_BALL_START  (ANIMATION_NORMALIZED_MAX * 40 / 100)
#define R5_REPORT_BALL_MS     200
// Stage 3: 1000ms total, matching Timeline's day separator exactly — DAY_SEP_TIMEOUT_MS
// (timeline.c:570) starts when the separator APPEARS, so the 500ms unfold plays inside
// the second and the settled paper holds the remainder.
#define R5_REPORT_SHOW_MS     1000

static void prv_start_report_stage2(void);
static void prv_start_report_stage3(void);
static void prv_start_report_stage4(void);
static void prv_report_stage4_stopped(Animation *anim, bool finished, void *context);

#if WEATHER_ANIM_5DAY
// BOTH SHAPES : apply a weather refresh that
// arrived while the report scene was playing (deferred by forecast_list_update_data so its
// flash PDC re-reads can't starve the 240ms stage-4 bow of its few frames — the framework
// jumps missed animations straight to their final frame, which erased the bow). Called
// from every scene exit — finish and cancel alike.
static void prv_apply_deferred_refresh(void) {
  if (!s_list || !s_list->refresh_deferred) return;
  s_list->refresh_deferred = false;
  if (!s_list->canvas) return;
  prv_load_icons();
  prv_free_jelly_lookups();
  prv_format_conditions();
  layer_mark_dirty(s_list->canvas);
}
#else
#define prv_apply_deferred_refresh()
#endif

// Stage 1: drives the squash-left + spawns the overlapping ball once the frame has
// mostly cleared.
static void prv_report_stage1_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_list) return;
  s_list->squash_in_p = progress;
  if (s_list->report_fx == 1 && progress >= R5_REPORT_BALL_START) {
    prv_start_report_stage2();   // overlap: the ball rolls in while the squash tail clears
  }
  layer_mark_dirty(s_list->canvas);
}

static void prv_report_stage1_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)context;
  s_select_exit_anim = NULL;
  if (!s_list) return;
  if (s_list->squash_mode == SQUASH_LEFT_EXIT) {   // free scratch on finish AND cancel
    s_list->squash_mode = 0;
    if (s_list->squash_scratch) { free(s_list->squash_scratch); s_list->squash_scratch = NULL; }
  }
  if (!finished) {   // only unload cancels (inputs are guarded)
    s_list->report_fx = 0;
    prv_apply_deferred_refresh();
    layer_mark_dirty(s_list->canvas);
    return;
  }
  if (s_list->report_fx == 1) prv_start_report_stage2();   // threshold missed (paranoia)
}
static const AnimationImplementation s_report_stage1_impl = { .update = prv_report_stage1_update };

// Stage 2: the ball. Burst-dot grammar — clean ease-out-quad, no bounce (the unfold's
// deflate overshoot supplies the landing energy; a ball bounce would double-bounce).
static void prv_report_stage2_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_list) return;
  const int W = layer_get_bounds(s_list->canvas).size.w;
  const int x0 = R5_REPORT_BALL_X0(W), x1 = W / 2;
  const int32_t q = ANIMATION_NORMALIZED_MAX - (int32_t)progress;
  const int32_t eased =
      ANIMATION_NORMALIZED_MAX - (int32_t)((int64_t)q * q / ANIMATION_NORMALIZED_MAX);
  s_list->fx_ball_x = x0 + (x1 - x0) * (int)eased / ANIMATION_NORMALIZED_MAX;
  layer_mark_dirty(s_list->canvas);
}

static void prv_report_stage2_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)context;
  s_report_anim = NULL;
  if (!s_list) return;
  if (!finished) {
    s_list->report_fx = 0;
    prv_apply_deferred_refresh();
    layer_mark_dirty(s_list->canvas);
    return;
  }
  s_list->fx_ball_x = layer_get_bounds(s_list->canvas).size.w / 2;   // parked at centre
  prv_start_report_stage3();
}
static const AnimationImplementation s_report_stage2_impl = { .update = prv_report_stage2_update };

static void prv_start_report_stage2(void) {
  if (!s_list || s_report_anim) return;
  s_list->report_fx = 2;
  s_list->fx_ball_x = R5_REPORT_BALL_X0(layer_get_bounds(s_list->canvas).size.w);
  s_report_anim = prv_start_anim(R5_REPORT_BALL_MS, AnimationCurveLinear,
                                 &s_report_stage2_impl, prv_report_stage2_stopped);
}

// Stage 3: the unfold + the day-separator-length hold.
static void prv_report_stage3_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_list) return;
  s_list->report_p = progress;
  layer_mark_dirty(s_list->canvas);
}

static void prv_report_stage3_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)context;
  s_report_anim = NULL;
  if (!s_list) return;
  if (!finished) {   // only unload cancels; it clears the done cb + fly itself
    s_list->report_fx = 0;
    prv_clear_fly();
    prv_apply_deferred_refresh();
    layer_mark_dirty(s_list->canvas);
    return;
  }
  prv_start_report_stage4();   // the paper takes its bow: squash-stretch off the left
}
static const AnimationImplementation s_report_stage3_impl = { .update = prv_report_stage3_update };

static void prv_start_report_stage3(void) {
  s_list->report_fx = 3;
  s_list->report_p  = 0;
  prv_clear_fly();   // the fly slot carries the paper clone + fan lookup (idempotent)
  GDrawCommandImage *raw =
      gdraw_command_image_create_with_resource(RESOURCE_ID_WEATHER_REPORT_PAPER);
  s_list->fly_pdc = raw ? gdraw_command_image_clone(raw) : NULL;   // flash PDC is read-only
  if (raw) gdraw_command_image_destroy(raw);
  // Frozen fan start-ray (Timeline picks a random ray per unfold; RTC seconds vary plenty).
  s_list->report_angle = (int32_t)(rtc_get_time() % TRIG_MAX_ANGLE);
  if (!s_list->fly_pdc) {   // resource missing: skip straight to the handoff
    prv_report_stage4_stopped(NULL, true, NULL);
    return;
  }
  s_report_anim = prv_start_anim(R5_REPORT_SHOW_MS, AnimationCurveLinear,
                                 &s_report_stage3_impl, prv_report_stage3_stopped);
}

// Stage 4: the paper + caption squash-stretch off the LEFT (the paper deforms ITSELF —
// Timeline's smiley exit, not a framebuffer warp), then the done cb fires in the same
// call stack: weather.c arms the report's squash-in-from-the-right.
static void prv_report_stage4_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_list) return;
  s_list->report_p = progress;   // stage 4 owns report_p: the exit flight's raw progress
  layer_mark_dirty(s_list->canvas);
}

static void prv_report_stage4_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)context;
  s_report_anim = NULL;
  if (!s_list) return;
  // BOTH shapes: round drives the same squash now, so it must also be torn down here —
  // leaving squash_mode set makes the entry guard reject every later SELECT (and leaks
  // the full-screen scratch, 67.6 KB on round, every time).
  if (s_list->squash_mode == SQUASH_LEFT_EXIT) {
    s_list->squash_mode = 0;
    if (s_list->squash_scratch) { free(s_list->squash_scratch); s_list->squash_scratch = NULL; }
  }
  prv_clear_fly();
  s_list->report_fx = 0;
  // Deferred sync refresh lands HERE — after the bow, before the report push: the flash
  // re-reads stall in the gap between animations where nothing is moving.
  prv_apply_deferred_refresh();
  void (*done)(void *) = s_select_exit_done;
  void *dctx = s_select_exit_ctx;
  s_select_exit_done = NULL;
  s_select_exit_ctx  = NULL;
  if (finished && done) done(dctx);   // -> weather.c: arm the report's squash-in + push
  layer_mark_dirty(s_list->canvas);
}
static const AnimationImplementation s_report_stage4_impl = { .update = prv_report_stage4_update };

static void prv_start_report_stage4(void) {
  s_list->report_fx = 4;
  s_list->report_p  = 0;   // flight start: moook(0) = rest — no first-frame jump
  // Stage 3's angle-fan lookup indexes the same pristine PDC but the exit needs a
  // DISTANCE lookup — free it so prv_draw_exiting_paper builds its own in the slot.
  if (s_list->fly_lookup) {
    applib_free(s_list->fly_lookup);
    s_list->fly_lookup = NULL;
  }
  s_report_anim = prv_start_anim(240, AnimationCurveLinear,   // the moook table IS the easing
                                 &s_report_stage4_impl, prv_report_stage4_stopped);
  if (!s_report_anim) {   // OOM: skip the flourish, hand off directly
    prv_report_stage4_stopped(NULL, true, NULL);
  }
}

// The rolling ball: a black disc with a white notch orbiting at rolling-without-slipping
// speed (leftward travel = counter-clockwise spin; circumference 2*pi*13 ~= 82px).
static void prv_draw_report_ball(GContext *ctx) {
  const GRect mb = layer_get_bounds(s_list->canvas);
  const int cx = s_list->fx_ball_x, cy = R5_SCREEN_CY;
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_circle(ctx, GPoint(cx, cy), R5_REPORT_BALL_R);
  const int32_t a = -(int32_t)(mb.size.w + R5_REPORT_BALL_R + 1 - cx) * (TRIG_MAX_ANGLE / 82);
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_circle(ctx, GPoint(cx + (int)((int32_t)sin_lookup(a) * 7 / TRIG_MAX_RATIO),
                                   cy - (int)((int32_t)cos_lookup(a) * 7 / TRIG_MAX_RATIO)), 3);
}

// Timeline day-separator unfold, hand-rolled in the prv_draw_flying_icon idiom (clone per
// frame from the pristine fly_pdc; angle-fan lookup built once): collapse-at-centre ->
// deflate-8 intermediate -> rest, 3 clockwise delay groups, fat stroke decaying from the
// ball's diameter over the first quarter so the dot IS the paper's first frame.
static void prv_draw_unfolding_paper(GContext *ctx) {
  if (!s_list->fly_pdc) return;
  GDrawCommandImage *clone = gdraw_command_image_clone(s_list->fly_pdc);
  if (!clone) return;
  GDrawCommandList *list = gdraw_command_image_get_command_list(clone);
  if (!list) { gdraw_command_image_destroy(clone); return; }
  const GSize nsz = gdraw_command_image_get_bounds_size(clone);       // 80x80
  if (!s_list->fly_lookup) {                                          // pristine geometry: cache
    GPointIndexLookup *lu = gdraw_command_list_create_index_lookup_by_angle(
        list, GPoint(nsz.w / 2, nsz.h / 2), s_list->report_angle);
    if (!lu) { gdraw_command_image_destroy(clone); return; }
    gpoint_index_lookup_set_groups(lu, 3, Fixed_S32_16(3 * FIXED_S32_16_ONE.raw_value / 2));
    s_list->fly_lookup = lu;
  }
  AnimationProgress up = (AnimationProgress)((int64_t)s_list->report_p * 2);   // unfold done
  if (up > ANIMATION_NORMALIZED_MAX) up = ANIMATION_NORMALIZED_MAX;   // at 500/1000ms, then hold
  const GRect mb = layer_get_bounds(s_list->canvas);
  const GPoint origin = GPoint((mb.size.w - nsz.w) / 2, R5_SCREEN_CY - nsz.h / 2);
  if (4 * up < ANIMATION_NORMALIZED_MAX) {   // dot phase: keep the ball UNDER the fat stroke
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_circle(ctx, GPoint(mb.size.w / 2, R5_SCREEN_CY), R5_REPORT_BALL_R);
  }
  const GRect to   = GRect(0, 0, nsz.w, nsz.h);
  const GRect from = GRect(nsz.w / 2, nsz.h / 2, 0, 0);   // collapsed dot at centre = the ball
  const Fixed_S32_16 point_dur  = Fixed_S32_16(FIXED_S32_16_ONE.raw_value / 6);
  const Fixed_S32_16 effect_dur = Fixed_S32_16(3 * FIXED_S32_16_ONE.raw_value / 4);
  GRect intermediate = grect_scalar_expand(to, 8);        // deflate 8, no bounce (day-sep exact)
  GSize size = nsz;
  AnimationProgress seg = animation_timing_segmented(up, 0, 2, effect_dur);
  gdraw_command_list_scale_segmented_to(list, size, from, intermediate, seg,
                                        NULL, s_list->fly_lookup, point_dur, false);
  size = intermediate.size;
  seg = animation_timing_segmented(up, 1, 2, effect_dur);
  gdraw_command_list_scale_segmented_to(list, size, intermediate, to, seg,
                                        NULL, s_list->fly_lookup, point_dur, true);
  const AnimationProgress sw = animation_timing_curve(
      animation_timing_clip(4 * up), AnimationCurveEaseInOut);   // fat stroke decays, first 25%
  gdraw_command_list_scale_stroke_width(list,
      Fixed_S16_3((2 * R5_REPORT_BALL_R) << FIXED_S16_3_PRECISION), FIXED_S16_3_ONE,
      GStrokeWidthOpSet, GStrokeWidthOpMultiply, sw);
  gdraw_command_image_draw(ctx, clone, origin);
  gdraw_command_image_destroy(clone);
}

// "WEATHER REPORT" rises in under the paper, landing with the fan's last delay group.
// dx slides the settled caption during the stage-4 exit; settled skips the rise
// (stage 4 reuses report_p as its OWN progress — the rise must not replay).
static void prv_draw_report_caption(GContext *ctx, int dx, bool settled) {
  AnimationProgress up = settled ? ANIMATION_NORMALIZED_MAX
                                 : (AnimationProgress)((int64_t)s_list->report_p * 2);
  if (up > ANIMATION_NORMALIZED_MAX) up = ANIMATION_NORMALIZED_MAX;
  const AnimationProgress start = ANIMATION_NORMALIZED_MAX * 3 / 5;
  if (up < start) return;
  int dy = 0;   // 6px ease-out-quad rise over up in [0.6, 0.96]
  if (up < ANIMATION_NORMALIZED_MAX * 24 / 25) {
    int32_t t = (int32_t)((int64_t)(up - start) * ANIMATION_NORMALIZED_MAX
                          / (ANIMATION_NORMALIZED_MAX * 9 / 25));
    if (t > ANIMATION_NORMALIZED_MAX) t = ANIMATION_NORMALIZED_MAX;
    const int32_t q = ANIMATION_NORMALIZED_MAX - t;
    dy = (int)(6 * ((int64_t)q * q / ANIMATION_NORMALIZED_MAX) / ANIMATION_NORMALIZED_MAX);
  }
  const GRect mb = layer_get_bounds(s_list->canvas);
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx, "WEATHER REPORT", fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                     GRect(dx, R5_SCREEN_CY + 46 + dy, mb.size.w, 24),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
}

// Stage 4 — the paper takes its bow: the smiley's Timeline exit, rotated LEFT. One real
// TRAVEL scale-segment (rest -> off the left edge) exactly like the up/down icon move
// below: the moook's anticipation supplies the wind-up squash, the per-point
// delay-by-distance lag the stretch — trailing (right) edge pulls last, same target
// convention as the header jelly ({w/2, h} for upward travel), rotated: {w, h/2}.
static void prv_draw_exiting_paper(GContext *ctx) {
  if (!s_list->fly_pdc) return;
  GDrawCommandImage *clone = gdraw_command_image_clone(s_list->fly_pdc);
  if (!clone) return;
  GDrawCommandList *list = gdraw_command_image_get_command_list(clone);
  if (!list) { gdraw_command_image_destroy(clone); return; }
  const GSize nsz = gdraw_command_image_get_bounds_size(clone);
  const GRect mb = layer_get_bounds(s_list->canvas);
  const GPoint origin = GPoint((mb.size.w - nsz.w) / 2, R5_SCREEN_CY - nsz.h / 2);
  if (!s_list->fly_lookup) {   // stage-3's angle fan was freed at arm; build the exit's
    GPointIndexLookup *lu = gdraw_command_list_create_index_lookup_by_distance(
        list, GPoint(nsz.w, nsz.h / 2));
    if (!lu) { gdraw_command_image_destroy(clone); return; }
    s_list->fly_lookup = lu;
  }
  const GRect from = GRect(0, 0, nsz.w, nsz.h);
  // END fully past the left edge with stretch headroom (the moook overshoots beyond it).
  const GRect to = GRect(-(origin.x + nsz.w + 40), 0, nsz.w, nsz.h);
  gdraw_command_image_scale_segmented_to(clone, from, to, s_list->report_p,
                                         prv_moook_full, s_list->fly_lookup,
                                         Fixed_S32_16(5 * FIXED_S32_16_ONE.raw_value / 6),
                                         false);   // = R5_FX_POINT_DURATION (defined below)
  gdraw_command_image_draw(ctx, clone, origin);
  gdraw_command_image_destroy(clone);
}
#endif  // WEATHER_ANIM_5DAY

// ===================================================================================
// Timeline up/down icon transition: a real TRAVEL scale-segment (NOT an in-place squash).
// This is the EXACT Timeline behaviour from services/timeline/timeline_layout_animations.c
// (timeline_layout_create_up_down_animation): a SINGLE kino_reel_scale_segmented from the icon's
// START frame to its END frame — an actual move — with delay_by_distance(target) and
// point_duration 5/6. The move itself + the per-point distance delay produce the across-the-travel
// stretch: the trailing edge lags so the bitmap STRETCHES nearly the whole travel between the two
// points at the midframe, then snaps to the destination. No manual intermediate, no deflate/bounce
// (Timeline calls neither, so its prv_apply_transform takes the single-stage path).
// ===================================================================================
// Shared: round plays the same card transition (hero icon-fly + jelly squash).

// Timeline's POINT_DURATION = 5/6 (timeline_layout_create_up_down_animation). EFFECT_DURATION (2/3)
// is only needed for the two-edge disc capsule envelope below (the icon move is single-stage).
#define R5_FX_POINT_DURATION   Fixed_S32_16(5 * FIXED_S32_16_ONE.raw_value / 6)
#define R5_FX_EFFECT_DURATION  Fixed_S32_16(2 * FIXED_S32_16_ONE.raw_value / 3)

// Clone `src`, scale-segment it FROM its start frame TO its end frame across `travel` px (an actual
// move, exactly like Timeline), then draw the clone at its END position `end_origin`. The persistent
// s_list icons are never touched. `travel` = how far up the icon has come (R5_SECTION_TRAVEL for the
// 5-day discs, R5_HEADER_TRAVEL for the today bitmap). Returns true if it drew the deformed clone.
//
// from/to are in the icon's LOCAL (0-based) coords — gdraw_command_image_draw applies the draw
// offset separately — so the travel is encoded as a y-DIFFERENCE: from sits +travel below the end,
// to sits at the end. We then draw at `end_origin` (the icon's fully-scrolled rest spot). The
// delay_by_distance lookup pulls from middle-x + the leading edge (top when going up-screen, bottom
// when going down), so as fx_progress advances the leading points reach `to` first while the
// trailing points still lag near `from` — that gap IS the across-the-travel stretch.
__attribute__((unused)) static bool prv_draw_jelly_icon(GContext *ctx, GDrawCommandImage *src, GPoint end_origin,
                                GSize isz, int travel, GPointIndexLookup **lookup_cache) {
  if (!src) return false;
  GDrawCommandImage *clone = gdraw_command_image_clone(src);
  if (!clone) return false;
  GDrawCommandList *list = gdraw_command_image_get_command_list(clone);
  if (!list) {
    gdraw_command_image_destroy(clone);
    return false;
  }
  // Timeline pulls from the middle-x and the leading edge: top (.y = h) when the icon heads
  // up-screen (DOWN press), bottom (.y = 0) when it heads down (UP press) — same as Timeline's
  // target = {w/2, going_up ? h : 0}. delay_by_distance => distance lookup from that target.
  // The lookup is deterministic for the whole transition (pristine geometry + fixed target), so
  // it is built on the transition's first frame and cached — every fresh clone shares the same
  // point indices.
  const bool going_up_screen = (s_list->header_to == R5_HEADER_TRAVEL);
  GPointIndexLookup *lookup = *lookup_cache;
  if (!lookup) {
    const GPoint target = GPoint(isz.w / 2, going_up_screen ? isz.h : 0);
    lookup = gdraw_command_list_create_index_lookup_by_distance(list, target);
    if (!lookup) {
      gdraw_command_image_destroy(clone);
      return false;
    }
    *lookup_cache = lookup;
  }
  InterpolateInt64Function interp = s_list->header_hasted ? prv_moook_hasted : prv_moook_full;
  // START frame is `travel` px away from the END along the direction of travel: BELOW (+travel)
  // on the DOWN press (the icon rises to its scrolled rest), ABOVE (-travel) on the UP press (it
  // drops back to the header-shown rest). A single scale_segment moves START -> END; the per-point
  // delay does the stretch.
  const GRect from = GRect(0, going_up_screen ? travel : -travel, isz.w, isz.h);
  const GRect to   = GRect(0, 0, isz.w, isz.h);
  gdraw_command_image_scale_segmented_to(clone, from, to, s_list->fx_progress, interp, lookup,
                                         R5_FX_POINT_DURATION, false);
  gdraw_command_image_draw(ctx, clone, end_origin);  // draw at the END (fully-scrolled) position
  gdraw_command_image_destroy(clone);
  return true;
}

// Timeline pin->card deflate/bounce: 20px overshoot along the travel direction.
static GPoint prv_fly_bounce_offset(GRect from, GRect to, int16_t bounce) {
  const int dx = to.origin.x - from.origin.x;
  const int dy = to.origin.y - from.origin.y;
  if (!dx && !dy) return GPointZero;
  const int mag = weather_isqrt(dx * dx + dy * dy);
  if (!mag) return GPointZero;
  return GPoint(bounce * dx / mag, bounce * dy / mag);
}

// Hero icon-fly: scale-segment "zoom" the LARGE today PDC from its header rect (fly_src) to the
// card's icon rect (fly_dest), driven by the squash animation's progress. This is the EXACT
// Timeline pin->card icon open (peek_layer.c: scale_segmented, deflate 10 + bounce 20, delay-by-
// distance, moook_soft) reproduced by hand via gdraw_command_list_scale_segmented_to. The two-stage
// deflate/bounce gives the jelly: the moook anticipation on stage 0 makes it wind back (bounce
// left) at liftoff, and stage 1 settles from the over-grown/overshot intermediate back into the
// destination (the overshoot bounce on landing). Drawn on top of the squashing screen.
static void prv_draw_flying_icon(GContext *ctx) {
  if (!s_list->fly_pdc) return;
  GDrawCommandImage *clone = gdraw_command_image_clone(s_list->fly_pdc);
  if (!clone) return;
  GDrawCommandList *list = gdraw_command_image_get_command_list(clone);
  if (!list) { gdraw_command_image_destroy(clone); return; }
  // Local coords: draw at fly_dest.origin, so `to` is the dest box at 0,0 and `from` carries the
  // source's size + its offset relative to the dest.
  const GRect to   = GRect(0, 0, s_list->fly_dest.size.w, s_list->fly_dest.size.h);
  const GRect from = GRect(s_list->fly_src.origin.x - s_list->fly_dest.origin.x,
                           s_list->fly_src.origin.y - s_list->fly_dest.origin.y,
                           s_list->fly_src.size.w, s_list->fly_src.size.h);
  // delay-by-distance target, matching peek_layer: (dest_centre - src_centre) + native_size/2.
  const GSize nsz = gdraw_command_image_get_bounds_size(clone);
  const GPoint c_from = grect_center_point(&s_list->fly_src);
  const GPoint c_to   = grect_center_point(&s_list->fly_dest);
  const GPoint target = gpoint_add(gpoint_sub(c_to, c_from), GPoint(nsz.w / 2, nsz.h / 2));
  // Deterministic for the whole fly (fly_src/fly_dest fixed at arm time): build once, cache.
  GPointIndexLookup *lookup = s_list->fly_lookup;
  if (!lookup) {
    lookup = gdraw_command_list_create_index_lookup_by_distance(list, target);
    if (!lookup) { gdraw_command_image_destroy(clone); return; }
    s_list->fly_lookup = lookup;
  }

  const AnimationProgress p = s_list->squash_in_p;
  // Two-stage (deflate 10, bounce 20) — the intermediate over-grows past `to` and overshoots along
  // the travel; stage 0 scales from -> intermediate, stage 1 settles intermediate -> to.
  const int16_t EV_DEFLATE = 10, EV_BOUNCE = 20;
  GRect intermediate = grect_scalar_expand(to, EV_DEFLATE);
  gpoint_add_eq(&intermediate.origin, prv_fly_bounce_offset(from, to, EV_BOUNCE));

  GSize size = nsz;
  const AnimationProgress seg0 = animation_timing_segmented(p, 0, 2, R5_FX_EFFECT_DURATION);
  gdraw_command_list_scale_segmented_to(list, size, from, intermediate, seg0, prv_moook_soft3,
                                        lookup, R5_FX_POINT_DURATION, false);
  size = intermediate.size;
  const AnimationProgress seg1 = animation_timing_segmented(p, 1, 2, R5_FX_EFFECT_DURATION);
  gdraw_command_list_scale_segmented_to(list, size, intermediate, to, seg1, prv_moook_soft3,
                                        lookup, R5_FX_POINT_DURATION, true);

  gdraw_command_image_draw(ctx, clone, s_list->fly_dest.origin);
  gdraw_command_image_destroy(clone);
}

// The whole card body (status time + sunset title + high/low° + UV/RAIN meters) slid in FROM THE
// LEFT + bounced, synced to the icon-fly's progress so it lands the moment the icon does. Drawn by
// the card's OWN shared routine so the sliding content and the static card are pixel-identical.
static void prv_draw_flying_content(GContext *ctx) {
  const int W = layer_get_bounds(s_list->canvas).size.w;
  // Staged entrance: hold the text off-screen for the first 40% of the
  // transition (while the forecast is still clearing — its exit is
  // compressed into the first ~75%), then slide it in on its own moook so
  // it still co-lands with the hero icon on the final frame.
  const int32_t MAXN = ANIMATION_NORMALIZED_MAX;
  int32_t local = (int32_t)s_list->squash_in_p - (MAXN * 2) / 5;
  if (local < 0) local = 0;
  local = (int32_t)((int64_t)local * 5 / 3);
  if (local > MAXN) local = MAXN;
  const int tdx = (int)prv_moook_soft3(local, -W, 0);
  // This preview must match what the card will REST on, or the status line visibly changes
  // in the first frames after the hand-off.
  char updated[24] = "";
#if PBL_ROUND
  clock_copy_time_string(updated, sizeof(updated));   // round's card shows the time only
#else
  if (s_list->num_days > 0) {   // rect starts on "Last updated ..." for its first 2s
    expanded_view_format_updated(&s_list->days[0], updated, sizeof(updated));
  }
#endif
  expanded_view_draw_glance_content(ctx, W, tdx, updated, s_list->fly_sunset, s_list->fly_temp,
                                    s_list->fly_uv, s_list->fly_precip, s_list->fly_wind);
}

// The disc behind each 5-day icon must stretch across the SAME travel as its icon: a capsule whose
// LEADING edge races to the destination while the TRAILING edge lags, so mid-transition it spans
// ~the whole travel + the disc, then closes to a circle at the destination. Both edges come from
// the SAME two-segment timing the icon move rides on — leading = segment 0, trailing = segment 1 —
// each eased by the moook, interpolating the disc CENTRE between its start (R5_ICON_CY) and end
// (R5_ICON_CY - R5_SECTION_TRAVEL). At rest / at the destination the two edges coincide => circle.
//
// Returns the capsule rect for column centre-x `cx`; *is_capsule_out is true while stretched.
__attribute__((unused)) static GRect prv_fx_disc_capsule(int cx, bool *is_capsule_out) {
  if (is_capsule_out) *is_capsule_out = false;
  const bool going_up = (s_list->header_to == R5_HEADER_TRAVEL);  // DOWN press -> section travels up
  const int cy_a = R5_ICON_CY;                      // header-shown row centre
  const int cy_b = R5_ICON_CY - R5_SECTION_TRAVEL;  // scrolled row centre
  const int cy_start = going_up ? cy_a : cy_b;      // this transition's start centre (A->B / B->A)
  const int cy_end   = going_up ? cy_b : cy_a;      // ... and end centre
  if (!s_list->fx_active) {
    // Rest: a plain circle at the icon row's current (post-slide) centre.
    const int dcy = R5_ICON_CY - (s_list->header_scroll * R5_SECTION_TRAVEL / R5_HEADER_TRAVEL);
    return GRect(cx - R5_DISC_R, dcy - R5_DISC_R, 2 * R5_DISC_R, 2 * R5_DISC_R);
  }
  const int32_t n = s_list->fx_progress;
  InterpolateInt64Function interp = s_list->header_hasted ? prv_moook_hasted : prv_moook_full;
  // Two segments of the same 2/3-effect timing the icon scale uses. Segment 0 leads (reaches the
  // destination first), segment 1 trails. Ease each by the moook, then map to a disc-centre y.
  const AnimationProgress lead_n  = animation_timing_segmented(n, 0, 2, R5_FX_EFFECT_DURATION);
  const AnimationProgress trail_n = animation_timing_segmented(n, 1, 2, R5_FX_EFFECT_DURATION);
  const int lead_cy  = (int)interp(lead_n,  cy_start, cy_end);
  const int trail_cy = (int)interp(trail_n, cy_start, cy_end);
  // Going up-screen (DOWN press): the TOP is the leading edge (smaller y) and the BOTTOM trails.
  // Going down (UP press): flip — the BOTTOM leads, the TOP trails.
  const int top_cy    = going_up ? lead_cy  : trail_cy;
  const int bottom_cy = going_up ? trail_cy : lead_cy;
  const int top    = top_cy    - R5_DISC_R;
  const int bottom = bottom_cy + R5_DISC_R;
  if (is_capsule_out) *is_capsule_out = (bottom - top > 2 * R5_DISC_R);
  return GRect(cx - R5_DISC_R, top, 2 * R5_DISC_R, bottom - top);
}

// (the bottom-gap stats below are shared too)

// ---- Bottom-gap stats (revealed by scrolling DOWN): a Pebble-Health-style "PRECIPITATION"
// banner pill (the Health "TYPICAL" pill — rounded-rect, bold centred label — made thinner),
// with the five per-day precip values aligned under their day columns beneath. The block
// slides up from below into the gap on the same moook bounce.
#define R5_STAT_PILL_H 18   // thinner than Health's ~35px pill
static void prv_draw_stat_pill(GContext *ctx, const char *label, int y, int W, GColor fill) {
  // Pebble Health's pill geometry EXACTLY: a full-width rect inset on each side by
  // HEALTH_INSET = 18 + HEALTH_X_OFFSET/5, where HEALTH_X_OFFSET = (DISP_COLS - 144)/2 — i.e. 23px
  // on emery's 200px width (health/ui.c). Corner radius 3, GCornersAll. `fill` is per-metric
  // (grey for wind, light blue for precip); black bold label.
  const int inset = 18 + (W - 144) / 2 / 5;
  const GRect pill = GRect(inset, y, W - 2 * inset, R5_STAT_PILL_H);
  graphics_context_set_fill_color(ctx, fill);
  graphics_fill_round_rect(ctx, &pill, 3, GCornersAll);
  graphics_context_set_text_color(ctx, GColorBlack);
  // Vertically centre the label. graphics_draw_text top-aligns, and GOTHIC_14_BOLD carries ~7px of
  // top padding inside its ascent (measured: in an 18px pill the caps land 7px down / 2px up), so
  // centring the line box reads low. Lift it by that asymmetry so the caps sit on the pill's centre.
  GFont lf = s_list->day_font;   // GOTHIC_14_BOLD, cached at load — this draws per scroll frame
  const int th = fonts_get_font_height(lf);
  graphics_draw_text(ctx, label, lf,
      GRect(inset, y + (R5_STAT_PILL_H - th) / 2 - 3, W - 2 * inset, th),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
}

// One row of five per-day precip-% values, each centred under its day column.
static void prv_draw_stat_row(GContext *ctx, const WeatherLocationForecast *fan, int n,
                              const int *col_x, int y) {
  GFont vfont = s_list->day_font;   // GOTHIC_14_BOLD, cached at load
  const int cw = R5_COL_STEP_ICON;
  graphics_context_set_text_color(ctx, GColorBlack);
  for (int i = 0; i < n; i++) {
    const int v = fan[i].today_precip_mm;
    char buf[16];
    if (v < 0) snprintf(buf, sizeof(buf), "--");
    else       snprintf(buf, sizeof(buf), "%d%%", v);
    graphics_draw_text(ctx, buf, vfont, GRect(col_x[i] - cw / 2, y, cw, 16),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  }
}

static void prv_draw_bottom_stats(GContext *ctx, const WeatherLocationForecast *fan, int n,
                                  const int *col_x, int W) {
  if (s_list->header_scroll <= 0) return;   // hidden until the gap starts opening
  const int slide = R5_HEADER_TRAVEL - s_list->header_scroll;  // off the bottom at A, in-gap at B
  // The precip banner rides `slide` like everything else, INCLUDING during the clock burst:
  // stage 1 squashes the whole drawn frame up off the top (SQUASH_UP_EXIT), so the pill must
  // stay in the frame to exit with it; in stage 2 (header_scroll = R5_CLOCK_EXIT) slide puts
  // it far off-screen. The burst dot is decoupled — drawn AFTER the squash in prv_canvas_draw.
  // Pill at 145 / row at 164: with the deeper section lift (EXTRA_LIFT 19) the
  // scrolled column's air splits evenly — solved for disc->graph = graph->pill =
  // %-row->dot (~10-11px each; the graph auto-centres in its band, so the split
  // holds for any temp spread).
  prv_draw_stat_pill(ctx, "PRECIPITATION", 145 + slide + R5_BAND_DY, W, GColorPictonBlue);
  prv_draw_stat_row (ctx, fan, n, col_x,   164 + slide + R5_BAND_DY + R5_BAND_ROW_GAP);
  if (!s_list->clock_fx) {
    // Timeline's day-separator peek dot (the one that unwinds into "Tomorrow"/"Tuesday"): a 12px
    // black circle, horizontally centred, the same 30px above the bottom edge as Timeline. On emery
    // (>=200px height -> s_style_large) its centre lands at y = future_top_margin(7) + fat_pin(131) +
    // future_dot_offset(16) + thin_pin(88)/2 = 198 on the 228px screen.
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_circle(ctx, GPoint(W / 2, R5_DOT_REST_Y + slide), 6);
  }
}

// ---- Animated sun: 10 rays orbit the sun clockwise, each ray's length recomputed
// from its CURRENT angle to the sun's natural profile (short at top, long at bottom)
// so the rays grow as they descend + shrink as they rise — a living glow. ----
#define R5_SUN_RAYS       10
#define R5_SUN_SCALE      100  // body + ray scale, percent of the 40px reference
// Partly cloudy reuses the same sun, smaller, poking out behind the cloud.
#define R5_PC_SUN_SCALE   60

// Animation cadence (sun + rain share one timer). Rect: 80 ms tick = 12.5 fps; prv_sun_tick
// advances 2 phase steps per tick so the VISUAL speed matches the old 40ms/25fps cadence
// while the redraw rate is halved — the full-screen redraw was over the per-frame budget on
// real hardware and backed up the task that services touch. The rain/snow derive a tick
// count from the accumulated angle so their fall speed follows automatically.
// ROUND (gabbro smoothness step 4): 66 ms = exactly 2 ticks of the animation service's 33ms
// frame interval, so every icon step lands ON a display frame. 80ms landed on alternating
// 66/99ms boundaries — a visible advance/skip/advance micro-judder. The per-tick phase
// advance is scaled (see prv_sun_step) so the visual speed is unchanged.
#define R5_SUN_PERIOD_MS   PBL_IF_ROUND_ELSE(66, 80)
#define R5_SUN_ANGLE_STEP  (TRIG_MAX_ANGLE / 168)

// Power saving: after R5_IDLE_TICKS of no interaction (10 s at each shape's cadence: rect
// 125 × 80 ms, round 152 × 66 ms) the animated colour today-icon fades over R5_FADE_MAX
// ticks into Pebble's own static weather artwork, then the timer stops so the screen draws
// no more frames. A touch wakes it again. The animated overlays (flakes / rays / drops)
// scale by icon_fade/R5_FADE_MAX during the fade.
#define R5_IDLE_TICKS      PBL_IF_ROUND_ELSE(152, 125)
// Fade duration in ticks — round carries 14 so the fade-to-static lasts ~the same wall time
// at the 66ms cadence (14x66 = 924ms vs rect 12x80 = 960ms); kept EVEN because the dither
// cross-fade splits it into two symmetric R5_FADE_MAX/2 halves.
#define R5_FADE_MAX        PBL_IF_ROUND_ELSE(14, 12)
#define R5_FADE(x)         ((x) * s_list->icon_fade / R5_FADE_MAX)
// Transition look (mirrors the clock screen's weather-bitmap drop/rise): the colour anim
// drops down + dither-fades out while the static icon rises up + dither-fades in.
#define R5_ICON_SLIDE      14    // px the icon travels during the fade transition
#define R5_FADE_LEVELS     4     // dither steps for the cross-fade

// Set to 1 to render the animated weather bitmaps (rain streaks, snow flakes, sun body +
// rays) in monochrome black instead of their per-condition colours. Flip to 0 to restore the
// original palette — the source colours live untouched in the PrecipStyle structs + the #else
// branches here, so this one toggle is the whole revert. (Clouds are the official PDC art and
// are unaffected either way.)
#define WEATHER_ANIM_MONO_BLACK 1

// The sun's body, drawn the way the PDC draws it: the EXACT octagon polygon from
// Pebble_50x50_Sunny_day.svg (points (+-4.5,+-11.5)/(+-11.5,+-4.5) about the body centre,
// x0.8 for the 40px icon -> (+-4,+-9)/(+-9,+-4)), rendered as WHITE FILL + a stroke-3 BLACK
// OUTLINE with round joins — NOT two nested filled octagons: an antialiased small filled
// path smears its corners and reads as an oval; the stroked path keeps the flats crisp,
// exactly like the static bitmap.
static void prv_draw_sun_body(GContext *ctx, GPoint c, int scale_pct, GColor outline_col) {
  const int a = (92 * scale_pct + 500) / 1000;   // apothem, from the art's 11.5 x 0.8
  const int d = (36 * scale_pct + 500) / 1000;   // corner half-flat, from 4.5 x 0.8
  GPoint pts[8] = {
    {(int16_t)-d, (int16_t)-a}, {(int16_t)-a, (int16_t)-d},
    {(int16_t)-a, (int16_t)d},  {(int16_t)-d, (int16_t)a},
    {(int16_t)d,  (int16_t)a},  {(int16_t)a,  (int16_t)d},
    {(int16_t)a,  (int16_t)-d}, {(int16_t)d,  (int16_t)-a},
  };
  GPath path = { .num_points = 8, .points = pts, .offset = c };
  graphics_context_set_fill_color(ctx, GColorWhite);
  gpath_draw_filled(ctx, &path);
  graphics_context_set_stroke_color(ctx, outline_col);
  graphics_context_set_stroke_width(ctx, 3);     // the PDC body's own stroke width
  gpath_draw_outline(ctx, &path);
}

// Ray geometry lifted straight off Pebble_50x50_Sunny_day.svg (x0.8 for the 40px icon).
// TWO things make the PDC's sun read right, and both are kept exactly:
// 1) The rays are NOT uniformly spaced — they cluster (43/30/31/33/43-degree gaps,
//    mirrored). The constellation below carries the PDC's own angular offsets and
//    rotates as a rigid wheel.
// 2) The lengths + inner radii are irregular: stubs up top, medium horizontals, the
//    LONGEST rays on the lower diagonals, and per-ray inner radii that breathe.
//    Each ray re-samples the profile at its CURRENT angle as it orbits, so the
//    silhouette stays the PDC's composition while the glow lives.
// Values in tenths of a pixel; inner radii carry +1px over the raw art so the stroke-2
// round caps land where the PDC's ink starts — the gap around the body survives.
static const uint16_t kSunRayAngles[10] = {   // the PDC's own ray positions (from top, cw)
  0,
  (uint16_t)(TRIG_MAX_ANGLE * 43 / 360),
  (uint16_t)(TRIG_MAX_ANGLE * 73 / 360),
  (uint16_t)(TRIG_MAX_ANGLE * 104 / 360),
  (uint16_t)(TRIG_MAX_ANGLE * 137 / 360),
  (uint16_t)(TRIG_MAX_ANGLE / 2),
  (uint16_t)(TRIG_MAX_ANGLE * 223 / 360),
  (uint16_t)(TRIG_MAX_ANGLE * 256 / 360),
  (uint16_t)(TRIG_MAX_ANGLE * 287 / 360),
  (uint16_t)(TRIG_MAX_ANGLE * 317 / 360),
};
static const struct { uint16_t ang; uint8_t len10; uint8_t inner10; } kSunRayProfile[] = {
  { 0,                            24, 138 },   // top stub
  { TRIG_MAX_ANGLE * 43 / 360,    34, 163 },   // upper diagonal stub (pushed out)
  { TRIG_MAX_ANGLE * 73 / 360,    66, 144 },   // upper horizontal
  { TRIG_MAX_ANGLE * 104 / 360,   66, 142 },   // lower horizontal
  { TRIG_MAX_ANGLE * 137 / 360,  113, 152 },   // the long SE/SW diagonal
  { TRIG_MAX_ANGLE / 2,          104, 138 },   // long bottom ray
};

// Piecewise-linear sample of the profile at `ang` (any angle; folded to 0..180deg).
static void prv_sun_ray_at(int32_t ang, int *len10, int *inner10) {
  int32_t f = ang % TRIG_MAX_ANGLE;
  if (f < 0) f += TRIG_MAX_ANGLE;
  if (f > TRIG_MAX_ANGLE / 2) f = TRIG_MAX_ANGLE - f;
  const int n = (int)(sizeof(kSunRayProfile) / sizeof(kSunRayProfile[0]));
  for (int i = 1; i < n; i++) {
    if (f <= kSunRayProfile[i].ang) {
      const int32_t a0 = kSunRayProfile[i - 1].ang, a1 = kSunRayProfile[i].ang;
      const int32_t t = (a1 > a0) ? ((f - a0) * 256 / (a1 - a0)) : 0;
      *len10 = kSunRayProfile[i - 1].len10 +
               (int)((kSunRayProfile[i].len10 - kSunRayProfile[i - 1].len10) * t / 256);
      *inner10 = kSunRayProfile[i - 1].inner10 +
                 (int)((kSunRayProfile[i].inner10 - kSunRayProfile[i - 1].inner10) * t / 256);
      return;
    }
  }
  *len10 = kSunRayProfile[n - 1].len10;
  *inner10 = kSunRayProfile[n - 1].inner10;
}

// Rounded projection of a tenths-of-a-px radius along (sin, -cos) — truncation was
// eating a full pixel of the body gap on the axis-aligned rays.
static int prv_sun_proj(int32_t r10, int32_t trig, int c) {
  int32_t v = r10 * trig / TRIG_MAX_RATIO;   // tenths of a px, signed
  v = (v >= 0) ? (v + 5) / 10 : (v - 5) / 10;
  return c + (int)v;
}

static void prv_draw_animated_sun(GContext *ctx, GPoint c, int32_t phase, int scale_pct) {
#if WEATHER_ANIM_MONO_BLACK
  const GColor sun_color = GColorBlack;
#else
  // Warm yellow-orange = the partly-cloudy disc colour on this screen (not pure red
  // orange). Pulled from the same source so it matches exactly. This icon only.
  const GColor sun_color = prv_weather_bg_color(WeatherType_PartlyCloudy);
#endif
  graphics_context_set_antialiased(ctx, true);
  // The PDC's own octagon, fill + stroked outline (see prv_draw_sun_body).
  prv_draw_sun_body(ctx, c, scale_pct, sun_color);
  // Rays: the PDC's clustered constellation rotates as one wheel; each ray samples the
  // length/inner profile at its current angle.
  graphics_context_set_stroke_color(ctx, sun_color);
  graphics_context_set_stroke_width(ctx, 2);   // the PDC rays' own stroke width
  for (int i = 0; i < R5_SUN_RAYS; i++) {
    const int32_t ang = phase + (int32_t)kSunRayAngles[i];
    int len10, inner10;
    prv_sun_ray_at(ang, &len10, &inner10);
    len10 = R5_FADE(len10 * scale_pct / 100);    // rays retract as the icon fades to static
    inner10 = inner10 * scale_pct / 100;
    if (len10 < 8) continue;                     // sub-pixel nub: not worth a cap dot
    const int32_t cos_a = cos_lookup(ang);
    const int32_t sin_a = sin_lookup(ang);
    // Radial direction from the sun centre, clockwise from top = (sin, -cos).
    const GPoint p0 = GPoint(prv_sun_proj(inner10, sin_a, c.x),
                             prv_sun_proj(inner10, -cos_a, c.y));
    const GPoint p1 = GPoint(prv_sun_proj(inner10 + len10, sin_a, c.x),
                             prv_sun_proj(inner10 + len10, -cos_a, c.y));
    graphics_draw_line(ctx, p0, p1);
  }
}

// ---- Animated precipitation (light rain + storm): the official cloud stays (black,
// static) but gently floats, while blue drops fall beneath it — a clear gap below the
// cloud, each drop fading in (grow), falling, fading out (shrink + pale) above the
// date. Drops have independent x, start offset + speed so they scatter, not sweep.
// Storm = darker blue, diagonal (wind-swept, down-left) drops + a more violent 2-axis
// cloud float. The PDC's own static rain lines are pushed off-screen by the processor.
// Particle band, measured DOWN from the today-icon origin (pdc_offset.y) so the fall
// follows the cloud on any platform. Round icon sits at y=26, so 28/42/25 here reproduce
// the original absolute 54/68/51; emery's icon sits higher (y=10) so the band rides up
// with it, staying just below the cloud and clear of the date row.
#define R5_RAIN_TOP    28     // drops emerge here — a clear gap below the cloud bottom
#define R5_RAIN_BOT    42     // drops are destroyed here — just above the date
#define R5_RAIN_LEN    5      // full streak length
#define R5_RAIN_FADE   4      // px over which a drop grows in / shrinks out
#define R5_SNOW_TOP    25     // snow starts a touch higher than rain → more room to grow
#define R5_SNOW_MAX    5      // snowflake half-size (px) at its peak, just before it vanishes

typedef struct { int8_t dx; uint8_t spd; uint8_t off; } RainDrop;
typedef struct {
  GColor  main_col;          // streak colour
  GColor  pale_col;          // colour at the fade-in/out ends (toward the background)
  int8_t  slant;             // horizontal drift per px of fall, in 1/16 px (0 = vertical)
  uint8_t bob_y;             // gentle vertical cloud bob amplitude (px)
  uint8_t bob_ticks;         // cloud bob period (ticks)
  const RainDrop *drops;
  uint8_t n_drops;
  uint8_t sway;              // snow side-to-side amplitude (px); 0 = falls dead straight
  uint8_t sway_div;          // snow sway period divisor (bigger = slower, gentler drift)
  bool    snow;              // true = flakes (sway/grow), false = streaks (slant/fade)
} PrecipStyle;

// Light rain: 4 vertical drops on the columns the PDC implies, gentle vertical float.
static const RainDrop kRainDrops[4] = {
  { -11, 14,  0 }, {  -4, 18,  7 }, {   4, 12,  3 }, {  11, 16, 10 },
};
static const PrecipStyle kLightRainStyle = {
  GColorVividCerulean, GColorCeleste, 0, 2, 88, kRainDrops, 4, 0, 45,  // rain: sway unused
};
// Storm (heavy rain): 4 diagonal (wind-swept, down-left) drops in darker blue, with the
// same gentle vertical bob as the rain cloud.
static const RainDrop kStormDrops[4] = {
  { -10, 15,  0 }, {  -4, 19,  8 }, {  2, 13,  4 }, {  8, 17, 11 },  // centred under the cloud
};
static const PrecipStyle kStormStyle = {
  GColorDukeBlue, GColorVividCerulean, 4, 2, 88, kStormDrops, 4, 0, 45,  // rain: sway unused
};
// Light snow: mirrors Pebble's snow bitmap — a cloud above 4 evenly-spaced snowflake
// "lanes". Each flake falls dead straight in its own lane and drifts side-to-side only a
// hair (sway=2) and slowly (sway_div=72), so the lanes never cross. `dx` = lane centre,
// `spd` = fall speed, `off` = phase (staggered so they don't fall in lockstep).
static const RainDrop kSnowFlakes[4] = {
  { -12, 4, 0 }, { -4, 4, 9 }, { 4, 4, 4 }, { 12, 4, 13 },
};
static const PrecipStyle kLightSnowStyle = {
  GColorPictonBlue, GColorPictonBlue, 0, 2, 88, kSnowFlakes, 4, 2, 72, true,
};
// Heavy snow: same flakes, denser (6) and a touch faster — a thicker fall.
static const RainDrop kHeavySnowFlakes[6] = {
  { -13, 5, 0 }, { -8, 6, 9 }, { -2, 5, 4 }, { 4, 6, 13 }, { 9, 5, 7 }, { 13, 6, 2 },
};
static const PrecipStyle kHeavySnowStyle = {
  GColorPictonBlue, GColorPictonBlue, 0, 2, 88, kHeavySnowFlakes, 6, 3, 45, true,  // busier drift
};
// Rain & snow (sleet): a moderate flake count falling a little quicker than plain snow.
static const RainDrop kRainSnowFlakes[4] = {
  { -11, 6, 0 }, { -3, 7, 9 }, { 5, 6, 4 }, { 12, 7, 12 },
};
static const PrecipStyle kRainSnowStyle = {
  GColorPictonBlue, GColorPictonBlue, 0, 2, 88, kRainSnowFlakes, 4, 2, 55, true,
};

static void prv_hide_rain_proc(GDrawCommandProcessor *processor, GDrawCommand *pc,
                               size_t max_size, const GDrawCommandList *list,
                               const GDrawCommand *command) {
  if (gdraw_command_get_num_points(pc) == 2) {  // 2-pt commands = the PDC rain/snow lines
    gdraw_command_set_hidden(pc, true);          // hide cleanly — the processor runs on a
  }                                              // per-frame copy, so no off-screen stray dot
}

// Hide the PartlyCloudy PDC's sun (8-pt body octagon + 2-pt rays), leaving just the
// 13-pt cloud + its 3-pt highlight — so we can draw our own animated sun behind it.
static void prv_hide_sun_proc(GDrawCommandProcessor *processor, GDrawCommand *pc,
                              size_t max_size, const GDrawCommandList *list,
                              const GDrawCommand *command) {
  // Keep only the cloud (13/14-pt polygon) + its highlight (3-pt polyline); hide the
  // sun body (8 or 9 pts, depending on the explicit polygon close) + its rays (2 pts).
  const uint16_t n = gdraw_command_get_num_points(pc);
  if (n != 3 && n < 13) {
    gdraw_command_set_hidden(pc, true);
  }
}

// Partly cloudy: a small sun with orbiting rays pokes out (top-right, behind the cloud),
// and the official cloud floats on top with the same gentle bob as the rain cloud.
static void prv_draw_animated_partly_cloudy(GContext *ctx, GPoint pdc_offset, int32_t phase) {
  const int tick = phase / R5_SUN_ANGLE_STEP;
  const int32_t ay = (int32_t)(tick % 88) * (TRIG_MAX_ANGLE / 88);
  const int bob_y = 2 * sin_lookup(ay) / TRIG_MAX_RATIO;
  // Sun first (behind) at the PDC sun centre (33,16) scaled into the today box.
  prv_draw_animated_sun(ctx,
      GPoint(pdc_offset.x + 33 * R5_TODAY_ICON / 50, pdc_offset.y + 16 * R5_TODAY_ICON / 50),
      phase, R5_PC_SUN_SCALE);
  // Floating cloud on top, its sun hidden (we drew our own animated one).
  if (s_list->today_pdc) {
    GDrawCommandProcessor proc = { .command = prv_hide_sun_proc };
    gdraw_command_image_draw_processed(ctx, s_list->today_pdc,
        GPoint(pdc_offset.x, pdc_offset.y + bob_y), &proc);
  }
}

// Cloudy: float the two clouds the same clean way we float the rain / partly-cloudy cloud —
// by translating the whole PDC's draw offset, not by nudging individual points. To move the
// clouds independently we draw the PDC twice; each pass hides the OTHER cloud off-screen so
// the kept cloud renders crisply at its own offset. CloudyDay PDC = lower cloud (cmds 0+1)
// then upper cloud (cmds 2+3).
typedef struct {
  GDrawCommandProcessor base;   // must be first — callback casts back to this
  int counter;
  int keep;                     // 0 = render lower cloud only, 1 = render upper cloud only
} CloudHideProc;

static void prv_cloud_hide_proc(GDrawCommandProcessor *processor, GDrawCommand *pc,
                                size_t max_size, const GDrawCommandList *list,
                                const GDrawCommand *command) {
  CloudHideProc *p = (CloudHideProc *)processor;
  const int which = (p->counter < 2) ? 0 : 1;  // cmds 0,1 = lower cloud; 2,3 = upper cloud
  p->counter++;
  if (which == p->keep) {
    return;  // keep this cloud exactly as authored
  }
  gdraw_command_set_hidden(pc, true);  // hide the other cloud cleanly (per-frame copy)
}

static void prv_draw_animated_cloudy(GContext *ctx, GPoint pdc_offset, int32_t phase) {
  if (!s_list->today_pdc) return;
  const int tick = phase / R5_SUN_ANGLE_STEP;
  // Each cloud floats a gentle ellipse — wider than tall, like a breeze — at different
  // rates + phases so they drift around and slide relative to one another.
  const int32_t aL = (int32_t)tick * (TRIG_MAX_ANGLE / 140);          // lower cloud, slower
  const int32_t aU = (int32_t)tick * (TRIG_MAX_ANGLE / 100) + 0x5000; // upper cloud, faster + offset
  const int lx = 5 * cos_lookup(aL) / TRIG_MAX_RATIO, ly = 2 * sin_lookup(aL) / TRIG_MAX_RATIO;
  const int ux = 5 * cos_lookup(aU) / TRIG_MAX_RATIO, uy = 2 * sin_lookup(aU) / TRIG_MAX_RATIO;

  CloudHideProc keep_lower = { .base = { .command = prv_cloud_hide_proc }, .counter = 0, .keep = 0 };
  gdraw_command_image_draw_processed(ctx, s_list->today_pdc,
      GPoint(pdc_offset.x + lx, pdc_offset.y + ly), &keep_lower.base);
  CloudHideProc keep_upper = { .base = { .command = prv_cloud_hide_proc }, .counter = 0, .keep = 1 };
  gdraw_command_image_draw_processed(ctx, s_list->today_pdc,
      GPoint(pdc_offset.x + ux, pdc_offset.y + uy), &keep_upper.base);
}

// A small snowflake: a "+" with shorter diagonals (8-pointed asterisk) at half-size r.
static void prv_draw_flake(GContext *ctx, GPoint c, int r) {
  if (r < 1) return;
  graphics_draw_line(ctx, GPoint(c.x, c.y - r), GPoint(c.x, c.y + r));
  graphics_draw_line(ctx, GPoint(c.x - r, c.y), GPoint(c.x + r, c.y));
  const int d = (r * 3) / 4;  // diagonals a touch shorter so the star stays round
  graphics_draw_line(ctx, GPoint(c.x - d, c.y - d), GPoint(c.x + d, c.y + d));
  graphics_draw_line(ctx, GPoint(c.x - d, c.y + d), GPoint(c.x + d, c.y - d));
}

static void prv_draw_animated_precip(GContext *ctx, GPoint pdc_offset, int32_t phase,
                                     const PrecipStyle *st) {
#if WEATHER_ANIM_MONO_BLACK
  const GColor main_col = GColorBlack;   // one ink for all precip — flakes included
  const GColor pale_col = main_col;      // (white flakes vanished on the white page)
#else
  const GColor main_col = st->main_col, pale_col = st->pale_col;
#endif
  // phase is the sun-angle accumulator -> back to a tick count (sun-speed independent).
  const int tick = phase / R5_SUN_ANGLE_STEP;
  const int32_t ay = (int32_t)(tick % st->bob_ticks) * (TRIG_MAX_ANGLE / st->bob_ticks);
  const int bob_y = st->bob_y * sin_lookup(ay) / TRIG_MAX_RATIO;
  if (s_list->today_pdc) {
    GDrawCommandProcessor proc = { .command = prv_hide_rain_proc };
    gdraw_command_image_draw_processed(ctx, s_list->today_pdc,
        GPoint(pdc_offset.x, pdc_offset.y + bob_y), &proc);
  }
  const int top   = st->snow ? R5_SNOW_TOP : R5_RAIN_TOP;
  const int range = R5_RAIN_BOT - top;
  const int peak  = (range * 3) / 5;
  const int cx    = pdc_offset.x + R5_TODAY_ICON / 2;
  graphics_context_set_antialiased(ctx, true);
  graphics_context_set_stroke_width(ctx, st->snow ? 1 : 2);
  if (st->snow) graphics_context_set_stroke_color(ctx, main_col);
  for (int i = 0; i < st->n_drops; i++) {
    const RainDrop *d = &st->drops[i];
    const int prog = ((tick * d->spd) / 10 + d->off) % range;
    if (st->snow) {
      const int y = pdc_offset.y + top + prog;
      const int r = (prog <= peak)
          ? 1 + (R5_SNOW_MAX - 1) * prog / peak
          : R5_SNOW_MAX * (range - 1 - prog) / (range - 1 - peak);
      const int32_t sa = (int32_t)tick * (TRIG_MAX_ANGLE / st->sway_div) + (int32_t)i * 0x2800;
      const int sway = st->sway * sin_lookup(sa) / TRIG_MAX_RATIO;
      prv_draw_flake(ctx, GPoint(cx + d->dx + sway, y), R5_FADE(r));
    } else {
      const int y_bot = pdc_offset.y + top + prog;
      int len;
      if (prog < R5_RAIN_FADE) {
        len = 1 + (R5_RAIN_LEN - 1) * prog / R5_RAIN_FADE;
      } else if (prog >= range - R5_RAIN_FADE) {
        len = 1 + (R5_RAIN_LEN - 1) * (range - 1 - prog) / R5_RAIN_FADE;
      } else {
        len = R5_RAIN_LEN;
      }
      len = R5_FADE(len);
      const GColor col = (prog < 2 || prog >= range - 2) ? pale_col : main_col;
      graphics_context_set_stroke_color(ctx, col);
      const int head_x = cx + d->dx - prog * st->slant / 16;
      const int tail_x = head_x + len * st->slant / 16;
      graphics_draw_line(ctx, GPoint(head_x, y_bot), GPoint(tail_x, y_bot - len));
    }
  }
}


// Light snow: same feel as the rain — flakes fall beneath a gently bobbing cloud, fade in
// under it and shrink away just above the date — but drawn as light-blue flakes that fall
// slower and sway side to side. Reuses prv_hide_rain_proc (the PDC's static flakes are 2-pt
// lines, same as rain lines) so only the cloud is kept.
// Draw today's animated condition icon with its top-left at (x, y). Factored so the
// power-save transition can render it at a dropped position.
static const PrecipStyle *const kFallStyles[] = {
  [WeatherType_LightSnow]   = &kLightSnowStyle,
  [WeatherType_LightRain]   = &kLightRainStyle,
  [WeatherType_HeavyRain]   = &kStormStyle,
  [WeatherType_HeavySnow]   = &kHeavySnowStyle,
  [WeatherType_RainAndSnow] = &kRainSnowStyle,
};

static void prv_draw_today_anim(GContext *ctx, const WeatherLocationForecast *today,
                                int x, int y) {
  (void)today;
  // Dispatch on prv_header_type() — THE SAME FIELD today_pdc was loaded from — never on
  // today->current_weather_type. The animated families all composite today_pdc into the
  // frame, so dispatching on the phone's synced 'current' while the artwork came from the
  // hourly 'now' painted TWO weather types stacked whenever the fields disagreed (seen
  // on-watch during an overcast->rain change: overcast clouds over rain streaks). On rect
  // prv_header_type() == current_weather_type, so this is a no-op there.
  const unsigned t = (unsigned)prv_header_type();
  const PrecipStyle *st = (t < (sizeof(kFallStyles)/sizeof(kFallStyles[0]))) ? kFallStyles[t] : NULL;
  if (st) {
    prv_draw_animated_precip(ctx, GPoint(x, y), s_list->sun_phase, st);
    return;
  }
  switch (prv_header_type()) {
    case WeatherType_Sun:
      prv_draw_animated_sun(ctx, GPoint(x + R5_TODAY_ICON / 2, y + R5_TODAY_ICON * 2 / 5),
          s_list->sun_phase, R5_SUN_SCALE);
      break;
    case WeatherType_PartlyCloudy:
      prv_draw_animated_partly_cloudy(ctx, GPoint(x, y), s_list->sun_phase); break;
    case WeatherType_CloudyDay:
      prv_draw_animated_cloudy(ctx, GPoint(x, y), s_list->sun_phase); break;
    default:
      if (s_list->today_pdc) gdraw_command_image_draw(ctx, s_list->today_pdc, GPoint(x, y));
      break;
  }
}

// Dither-erase part of a rectangle to `bg`, simulating a stepped fade — the same
// no-alpha technique the clock screen uses for its weather bitmaps. keep: 0 (gone) ..
// R5_FADE_LEVELS (fully opaque). `bg` is the surface the icon fades into: the blue banner
// on emery, white on round — erasing to the wrong colour paints a visible box. Operates
// directly on the framebuffer.
static void prv_fade_erase(GContext *ctx, GRect r, int keep, GColor bg) {
  if (keep >= R5_FADE_LEVELS) return;   // fully opaque — nothing to erase
  const int thr = keep * 4;             // erase any pixel whose dither value >= thr
  GBitmap *fb = graphics_capture_frame_buffer(ctx);
  if (!fb) return;
  const GRect fbb = gbitmap_get_bounds(fb);
  for (int y = r.origin.y; y < r.origin.y + r.size.h; y++) {
    if (y < fbb.origin.y || y >= fbb.origin.y + fbb.size.h) continue;
    GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)y);
    for (int x = r.origin.x; x < r.origin.x + r.size.w; x++) {
      if (x < (int)ri.min_x || x > (int)ri.max_x) continue;
      if (s_bayer4[y & 3][x & 3] >= thr) {
        ri.data[x] = bg.argb;
      }
    }
  }
  graphics_release_frame_buffer(ctx, fb);
}

// Connector polyline for one masked series (skip segments touching an unknown value).
static void prv_draw_series_links(GContext *ctx, const int *colx, const int *y,
                                  const bool *ok, int n, GColor c) {
  graphics_context_set_stroke_color(ctx, c);
  for (int i = 0; i < n - 1; i++) {
    if (ok[i] && ok[i + 1]) {
      graphics_draw_line(ctx, GPoint(colx[i], y[i]), GPoint(colx[i + 1], y[i + 1]));
    }
  }
}

#if !PBL_ROUND
// Hollow ring dot: coloured ring, white centre (emery graph).
static void prv_draw_ring_dot(GContext *ctx, GPoint p, GColor ring) {
  graphics_context_set_fill_color(ctx, ring);
  graphics_fill_circle(ctx, p, 4);
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_circle(ctx, p, 2);
}

// Centred bold temp numeral ("%d\xC2\xB0") in a 52px box.
static void prv_draw_temp_centered(GContext *ctx, int temp, GFont font, int cx, int y) {
  char b[8];
  snprintf(b, sizeof(b), "%d\xC2\xB0", temp);
  graphics_draw_text(ctx, b, font, GRect(cx - 26, y, 52, 16),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
}

// Fold min/max over the masked entries of v[0..n); *have tracks whether any seen yet.
static void prv_fold_minmax(const int *v, const bool *ok, int n, bool *have, int *mn, int *mx) {
  for (int i = 0; i < n; i++) {
    if (!ok[i]) continue;
    if (!*have) { *mn = *mx = v[i]; *have = true; }
    if (v[i] > *mx) *mx = v[i];
    if (v[i] < *mn) *mn = v[i];
  }
}
#endif

static void prv_canvas_draw_round_5day(Layer *layer, GContext *ctx) {
  GRect bounds = layer_get_bounds(layer);
  const int W = bounds.size.w;

  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);

  // DOWN/UP header transition offsets (BOTH boards). hs = px the today header is translated
  // UP this frame; ss = the synced 5-day-section lift (a scaled copy of the same moook curve,
  // so header and section bounce frame-for-frame in lockstep).
  const int hs = s_list->header_scroll;
  const int ss = hs * R5_SECTION_TRAVEL / R5_HEADER_TRAVEL;

  int total = (int)s_list->num_days;
  if (total <= 0) return;
#if !PBL_ROUND
  // Current time at the very top, drawn exactly like the notification status-bar clock
  // (StatusBarLayerModeClock, status_bar_layer.c): white, centred, FONT_KEY_GOTHIC_18, and
  // bottom-aligned within the STATUS_BAR_LAYER_HEIGHT (20px on emery) band minus its 2px
  // separator padding. clock_copy_time_string is the same helper notifications use (same
  // 12/24h format). It rides the header scroll (- hs) so it collapses up with the header.
  // Cached per wall-clock minute: a per-frame localtime+strftime would starve the touch task
  // (see the day/date cache below), so refresh only when the minute changes.
  {
    static char s_time_str[16];
    static time_t s_time_min = -1;
    const time_t cur_min = time(NULL) / 60;
    if (cur_min != s_time_min) {
      clock_copy_time_string(s_time_str, sizeof(s_time_str));
      s_time_min = cur_min;
    }
    GFont sb_font = fonts_get_system_font(FONT_KEY_GOTHIC_18);
    const int sb_fh = fonts_get_font_height(sb_font);
    const int sb_ty = (STATUS_BAR_LAYER_HEIGHT - 2 * STATUS_BAR_LAYER_SEPARATOR_Y_OFFSET - sb_fh)
                      - hs;
    graphics_context_set_text_color(ctx, GColorBlack);
    if (s_list->clock_loc_show || s_list->clock_swap_active) {
      // First entry: the active location holds the slot, then swaps out exactly like
      // the sunset card's status bar — location exits right, time trails from the left.
      char loc[24] = "";
      if (s_list->num_days > 0) {
        prv_clock_loc_city(loc, sizeof(loc), s_list->days[0].location_name);
      }
      const int sx = s_list->clock_swap_active
          ? (int)interpolate_moook_soft(s_list->clock_swap_p, 0, W, 3) : 0;
      graphics_draw_text(ctx, loc, sb_font, GRect(sx, sb_ty, W, sb_fh),
                         GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
      if (s_list->clock_swap_active) {
        graphics_draw_text(ctx, s_time_str, sb_font, GRect(sx - W, sb_ty, W, sb_fh),
                           GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
      }
    } else {
      graphics_draw_text(ctx, s_time_str, sb_font, GRect(0, sb_ty, W, sb_fh),
                         GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
    }
  }
#endif
#if R5_TODAY_IN_FAN
  int n = total;                      // today LEADS the fan (6 columns)
  if (n > R5_MAX_COLS) n = R5_MAX_COLS;
  const WeatherLocationForecast *fan = &s_list->days[0];  // fan = today onward
#else
  // the strip STARTS AT TODAY — the header above shows conditions RIGHT NOW
  // (current-hour), and the strip's first column is today's daily forecast, then the next
  // four days Same five
  // columns, same geometry — only the day window shifts.
  const int fan_start = 0;
  int n = total - fan_start;
  if (n > R5_MAX_COLS) n = R5_MAX_COLS;
  if (n < 0) n = 0;
  const WeatherLocationForecast *fan = &s_list->days[fan_start];
#endif

  // Three column tiers, each horizontally centred, fanning inward as they descend.
  int col_icon[R5_MAX_COLS], col_high[R5_MAX_COLS], col_low[R5_MAX_COLS];
  // Effective pitch eases from the wide rest values to the originals as the section scrolls
  // up, so the scrolled (precipitation) view is untouched by the mainscreen spread. ss is
  // CLAMPED: the clock exit drives header_scroll far past its rest range (R5_CLOCK_EXIT),
  // and an unclamped lerp would extrapolate the pitch negative mid-transition.
  {
    int ssc = ss;
    if (ssc < 0) ssc = 0;
    if (ssc > R5_SECTION_TRAVEL) ssc = R5_SECTION_TRAVEL;
    const int step_icon = R5_COL_STEP_ICON -
        (R5_COL_STEP_ICON - R5_COL_STEP_ICON_SCROLLED) * ssc / R5_SECTION_TRAVEL;
    const int step_high = R5_COL_STEP_HIGH -
        (R5_COL_STEP_HIGH - R5_COL_STEP_HIGH_SCROLLED) * ssc / R5_SECTION_TRAVEL;
    for (int i = 0; i < n; i++) {
      col_icon[i] = W / 2 - (n - 1) * step_icon / 2 + i * step_icon;
      col_high[i] = W / 2 - (n - 1) * step_high / 2 + i * step_high;
      col_low[i]  = W / 2 - (n - 1) * R5_COL_STEP_LOW  / 2 + i * R5_COL_STEP_LOW;
    }
  }
  const int box_w = 52;  // text box; centred short text never overlaps neighbours
  // Fan weekday row demoted to match the today header weight (so TODAY outranks the fan).
  GFont day_font   = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
  GFont small_font = fonts_get_system_font(FONT_KEY_GOTHIC_14);

  // Today's date caption + fan weekday labels change only at local midnight (or on a data
  // refresh) — the 6 localtime+strftime pairs this draw used to run per frame were real work
  // on a 12.5-30fps path, so both live in caches keyed on the next-midnight timestamp.
  {
    time_t now = time(NULL);
    if (now >= s_list->date_cache_expiry || s_list->today_header_dirty) {
      struct tm *lt = localtime(&now);
      if (lt) {
        strftime(s_list->daydate_cache, sizeof(s_list->daydate_cache), "%a, %b %d",
                 lt);  // title-case "Thu, Jun 25"
        // Cap the expiry at an hour: DST-transition days aren't 86400s long and backward
        // clock/timezone changes would otherwise leave stale strings up for hours. One extra
        // strftime per hour is noise; the midnight rollover still lands within the hour.
        time_t to_midnight =
            (time_t)(86400 - (lt->tm_hour * 3600 + lt->tm_min * 60 + lt->tm_sec));
        s_list->date_cache_expiry = now + (to_midnight < 3600 ? to_midnight : 3600);
      } else {
        s_list->daydate_cache[0] = '\0';
        s_list->date_cache_expiry = now + 60;   // retry shortly; localtime failed
      }
      for (int i = 0; i < n && i < R5_MAX_COLS; i++) {
        // Day offset == column index: the fan starts at TODAY on both shapes .
        prv_fill_weekday_label(i, fan[i].label, s_list->weekday_cache[i],
                               sizeof(s_list->weekday_cache[i]));
      }
      s_list->today_header_dirty = true;   // icon centring depends on the date width
    }
  }
  const char *daydate = s_list->daydate_cache;
  // Recompute the expensive text-layout-dependent header geometry ONLY when the data
  // changed — never every animation frame (that re-measuring was starving the system task
  // that services touch). today_icon_x centres the icon over the date; today_cond_font
  // steps the condition phrase down a size if it won't fit its band.
  if (s_list->today_header_dirty) {
    s_list->today_icon_x = R5_TODAY_X;
#if !PBL_ROUND
    const GSize dsz = graphics_text_layout_get_content_size(
        daydate, day_font, GRect(0, 0, W, 20),
        GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft);
    s_list->today_icon_x = R5_TODAY_X + (dsz.w - R5_TODAY_ICON) / 2;
    if (s_list->today_icon_x < R5_TODAY_X) s_list->today_icon_x = R5_TODAY_X;
#endif
    const char *cph = (s_list->days[0].current_weather_phrase &&
                       s_list->days[0].current_weather_phrase[0])
                          ? s_list->days[0].current_weather_phrase : "--";
    const int cl = PBL_IF_ROUND_ELSE(110, W / 2), cr = PBL_IF_ROUND_ELSE(230, W - R5_TODAY_X);
    GFont cf = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
    const GSize cw0 = graphics_text_layout_get_content_size(
        cph, cf, GRect(0, 0, 300, 20), GTextOverflowModeWordWrap, GTextAlignmentLeft);
    if (cw0.w > cr - cl) cf = fonts_get_system_font(FONT_KEY_GOTHIC_14);
    s_list->today_cond_font = cf;
    s_list->today_header_dirty = false;
  }
  const int today_x = s_list->today_icon_x;

  // ---- Today summary header: large icon top-left, current temp top-right,
  //      condition below the temp, day + date below the icon ----
  const WeatherLocationForecast *today = &s_list->days[0];
  const int fade = s_list->icon_fade;
#if !PBL_ROUND
  bool today_jellied = false;
  if (s_list->fx_active && fade <= 0 && s_list->today_pdc && !s_list->flying_icon) {
    // Mid-transition with the icon already power-saved to its STATIC PDC:
    // scale-segment it across the full travel (Timeline up/down move). When
    // the icon is still ANIMATING we deliberately skip this — the live
    // animation keeps playing and rides the moook slide un-deformed (see the
    // fade >= R5_FADE_MAX branch below), instead of visibly swapping to the
    // static artwork for the duration of the transition.
    const GSize tsz = gdraw_command_image_get_bounds_size(s_list->today_pdc);
    // END position: off the top on the DOWN press, the normal header row on the UP press; the
    // scale-segment supplies the travel + stretch from there.
    const GPoint tend = GPoint(today_x, (s_list->header_to == R5_HEADER_TRAVEL)
                                            ? (R5_TODAY_Y - R5_HEADER_TRAVEL) : R5_TODAY_Y);
    today_jellied = prv_draw_jelly_icon(ctx, s_list->today_pdc, tend, tsz, R5_HEADER_TRAVEL,
                                        &s_list->jelly_lookup[0]);
  }
  if (today_jellied) {
    // already drawn deformed above
  } else
#endif
  if (s_list->flying_icon) {
    // Hero icon-fly (UP -> card): today's icon is lifted out of the squash and drawn separately,
    // zooming to the card's icon spot (see prv_draw_flying_icon). Draw nothing for it here.
  } else if (fade >= R5_FADE_MAX) {
    // Fully animating.
    prv_draw_today_anim(ctx, today, today_x, R5_TODAY_Y - hs);
  } else if (fade <= 0) {
    // Fully static / power-saving (timer stopped → no more frames).
    if (s_list->today_pdc) {
      gdraw_command_image_draw(ctx, s_list->today_pdc, GPoint(today_x, R5_TODAY_Y - hs));
    }
  } else {
    // Two-phase, NON-overlapping transition (mirrors the clock screen). First half: the
    // colour animation drops down + dither-fades out into the page. Second half: from that
    // blank frame, the static icon rises up into position + fades in. The dither erases to
    // the page white so only the bitmap fades — no box.
    const GColor icon_bg = GColorWhite;
    const int half = R5_FADE_MAX / 2;
    if (fade > half) {
      // Outgoing colour animation: drop down + fade out into the banner.
      const int prog = R5_FADE_MAX - fade;                          // 1 → half
      const int drop = R5_ICON_SLIDE * prog / half;                 // 0 → full drop
      const int keep_out = (fade - half) * R5_FADE_LEVELS / half;   // opaque → gone
      prv_draw_today_anim(ctx, today, today_x, R5_TODAY_Y - hs + drop);
      // Erase wider than the 40px box: the cloud art (and the two-cloud cloudy float) spills
      // past the icon edges, so a tight rect would leave its far sides un-faded.
      prv_fade_erase(ctx, GRect(today_x - 10, R5_TODAY_Y - hs + drop, R5_TODAY_ICON + 20,
                                R5_TODAY_ICON + 8), keep_out, icon_bg);
    } else if (s_list->today_pdc) {
      // Incoming static icon: rise up from below + fade in (blank at the midpoint).
      const int prog = half - fade;                                 // 0 → half
      const int rise = R5_ICON_SLIDE * fade / half;                 // full below → 0
      const int keep_in = prog * R5_FADE_LEVELS / half;             // gone → opaque
      gdraw_command_image_draw(ctx, s_list->today_pdc, GPoint(today_x, R5_TODAY_Y - hs + rise));
      prv_fade_erase(ctx, GRect(today_x - 10, R5_TODAY_Y - hs + rise, R5_TODAY_ICON + 20,
                                R5_TODAY_ICON), keep_in, icon_bg);
    }
  }
  char tnow[16];
  const int hdr_temp = today->current_temp_now;   // current-hour temp, both shapes
  if (hdr_temp != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
    snprintf(tnow, sizeof(tnow), "%d\xC2\xB0", hdr_temp);
  } else {
    snprintf(tnow, sizeof(tnow), "--\xC2\xB0");
  }
  // Large temp, sized + vertically aligned to balance the 40px today icon.
#if PBL_ROUND
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx, tnow, fonts_get_system_font(FONT_KEY_LECO_36_BOLD_NUMBERS),
      GRect(112, 22 - hs, 120, 44),   // -hs: ride the header scroll off the top
      GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
#else
  // Emery: FLAT BLACK LECO on the white page — the Timeline sloth-screen grammar: black
  // type directly on the field, white reserved for fills inside the black-inked artwork.
  // Right margin equals the icon's left margin (R5_TODAY_X).
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx, tnow, fonts_get_system_font(FONT_KEY_LECO_36_BOLD_NUMBERS),
      GRect(W - R5_TODAY_X - 120, R5_TODAY_Y - 2 - hs, 120, 44),   // -2: LECO parks low
      GTextOverflowModeTrailingEllipsis, GTextAlignmentRight, NULL);
#endif
  char cond[24];
  snprintf(cond, sizeof(cond), "%s",
           (today->current_weather_phrase && today->current_weather_phrase[0])
               ? today->current_weather_phrase : "--");
  // Title-case (e.g. "Partly Cloudy") — Pebble reserves ALL CAPS for terse labels/units.
  // Condition matches the date: same font/size/weight, same baseline (date is on
  // the left under the icon, condition on the right under the temp).
  graphics_context_set_text_color(ctx, GColorBlack);   // black on the page
  // Condition: right-anchored at x=230 (mirrors the left date, aligned to the fan's
  // 30/230 columns), growing LEFT as it lengthens down to x=110 (just clear of the
  // date). Length-dependent: measure the phrase + step the font down one size if even
  // that band can't hold it, so long phrases like "PARTLY CLOUDY" are never clipped.
  const int cond_l = PBL_IF_ROUND_ELSE(110, W / 2), cond_r = PBL_IF_ROUND_ELSE(230, W - R5_TODAY_X);
  GFont cond_font = (GFont)s_list->today_cond_font;  // measured once on data change, not per frame
  graphics_draw_text(ctx, cond, cond_font,
      GRect(cond_l, R5_DAYDATE_Y - hs, cond_r - cond_l, 16),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentRight, NULL);
  graphics_context_set_text_color(ctx, GColorBlack);   // black on the page
  // Date: left box, left-aligned (the today icon above is centred over this string).
  graphics_draw_text(ctx, daydate, day_font,   // day_font IS GOTHIC_14_BOLD — no re-lookup
      GRect(PBL_IF_ROUND_ELSE(30, R5_TODAY_X), R5_DAYDATE_Y - hs,
            PBL_IF_ROUND_ELSE(95, W / 2 - R5_TODAY_X), 16),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);

  // Hairline rule separating the today header from the 5-day fan (timeline-card idiom).
  graphics_context_set_stroke_color(ctx, GColorLightGray);
  graphics_context_set_stroke_width(ctx, 1);
  graphics_draw_line(ctx, GPoint(PBL_IF_ROUND_ELSE(40, 8), R5_HAIRLINE_Y - hs),
                          GPoint(W - PBL_IF_ROUND_ELSE(40, 8), R5_HAIRLINE_Y - hs));

  // ---- Per-day fan column: weekday, disc + icon, precip % ----
  for (int i = 0; i < n; i++) {
    const WeatherLocationForecast *f = &fan[i];
    const int cx = col_icon[i];

    const char *weekday = s_list->weekday_cache[i];   // built in the date-cache block above
    graphics_context_set_text_color(ctx, GColorBlack);
    const int bow = R5_BOW_DY(i, n, ss);   // slight follow-the-glass curve (round only)
    if (!R5_DAYNAME_SCROLL_HIDE || ss * 4 < R5_SECTION_TRAVEL) {
      graphics_draw_text(ctx, weekday, day_font,
          GRect(cx - box_w / 2, R5_DAYNAME_Y - ss + bow, box_w, 22),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
    }

    GColor bg = prv_weather_bg_color(f->current_weather_type);
    // Discs claim the vanished label band on top of the section lift.
    const int dsc = R5_DISC_SCROLL_UP * ss / R5_SECTION_TRAVEL;
    const int dcy = R5_ICON_CY - ss + bow - dsc;  // disc centre-y after the slide
#if !PBL_ROUND
    // Disc capsule: leading edge races to the destination, trailing edge lags — both edges from
    // the SAME two-segment timing the icon move uses, so the disc stretches across the whole
    // travel mid-flight then closes to a circle at the destination. At rest it IS a circle.
    bool disc_capsule = false;
    const GRect disc_rect = prv_fx_disc_capsule(cx, &disc_capsule);
    if (!gcolor_equal(bg, GColorClear)) {
      graphics_context_set_fill_color(ctx, bg);
      if (disc_capsule) {
        graphics_fill_round_rect_by_value(ctx, disc_rect, R5_DISC_R, GCornersAll);
      } else {
        graphics_fill_circle(ctx, GPoint(cx, dcy), R5_DISC_R);
      }
    }
    // Emery: a 1px outline so every disc has a defined edge on the white field (the pale
    // condition colours otherwise dissolve into the background). Capsule outline mid-transition.
    graphics_context_set_stroke_color(ctx, GColorLightGray);
    graphics_context_set_stroke_width(ctx, 1);
    if (disc_capsule) {
      graphics_draw_round_rect_by_value(ctx, disc_rect, R5_DISC_R);
    } else {
      graphics_draw_circle(ctx, GPoint(cx, dcy), R5_DISC_R);
    }
#else
    if (!gcolor_equal(bg, GColorClear)) {
      graphics_context_set_fill_color(ctx, bg);
      graphics_fill_circle(ctx, GPoint(cx, dcy), R5_DISC_R);
    }
#endif
    // pdc_icons[] is indexed by DAY (loaded from days[k]); the fan window starts at day 0
    // (today-first, both shapes since ), so the icon for column i is
    // pdc_icons[i]. The old hard-coded +1 predates the today-first window and put
    // TOMORROW'S icon on TODAY'S disc (the icons visibly mismatched their columns).
    const int icon_day = i;
    if (s_list->pdc_icons[icon_day]) {
      GSize isz = gdraw_command_image_get_bounds_size(s_list->pdc_icons[icon_day]);
      const GRect ibox = { GPoint(cx - isz.w / 2, R5_ICON_CY - ss + bow - dsc - isz.h / 2), isz };
#if !PBL_ROUND
      // The icon's END position: scrolled row on the DOWN press, header-shown row on the UP press.
      const int iend_cy = (s_list->header_to == R5_HEADER_TRAVEL) ? (R5_ICON_CY - R5_SECTION_TRAVEL)
                                                                  : R5_ICON_CY;
      const GPoint iend = GPoint(cx - isz.w / 2, iend_cy - isz.h / 2);
      // Mid-transition: scale-segment the disc icon across its FULL travel (Timeline up/down move),
      // drawing at the END position — the scale-segment provides the travel + the stretch. At rest
      // (fx_active false) draw it plain at its slid position — no residual deformation.
      if (s_list->fx_active &&
          prv_draw_jelly_icon(ctx, s_list->pdc_icons[icon_day], iend, isz, R5_SECTION_TRAVEL,
                              &s_list->jelly_lookup[1 + i])) {
        // drawn deformed across the travel
      } else
#endif
      {
        gdraw_command_image_draw(ctx, s_list->pdc_icons[icon_day], ibox.origin);
      }
    }
  }

  // ---- Two-line hi/lo dot graph ----
  if (n < 1) return;  // no future days → just the today header
#if !PBL_ROUND
  // Emery: ONE shared temperature scale so highs sit above lows with the true gap shown
  // (the white space between the lines IS the daily range). Highs are the HERO series —
  // hollow orange-ring dots on a muted 2px Windsor-Tan link; lows recede — small solid
  // Cadet-Blue dots on a thin 1px link. No axes, no area fill.
  {
    // Unknown-temp sentinels (32767) are real on sparse v3/v4 records: skip them when
    // scaling and draw no dot/link/label for the missing values (okh/okl gates below).
    bool okh[R5_MAX_COLS], okl[R5_MAX_COLS];
    int th[R5_MAX_COLS], tl[R5_MAX_COLS];
    int vmin = 0, vmax = 0;
    bool have = false;
    for (int i = 0; i < n; i++) {
      th[i] = fan[i].today_high;
      tl[i] = fan[i].today_low;
      okh[i] = th[i] != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
      okl[i] = tl[i] != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
    }
    prv_fold_minmax(th, okh, n, &have, &vmin, &vmax);
    prv_fold_minmax(tl, okl, n, &have, &vmin, &vmax);
    if (!have) {  // every hi/lo unknown: skip the graph, keep the stats banner
      prv_draw_bottom_stats(ctx, fan, n, col_icon, W);
      return;
    }
    // No headroom padding: let the highs and lows fill the full plot height so the two
    // series sit further apart and have room to breathe.
    int vr = vmax - vmin; if (vr < 4) vr = 4;   // floor avoids over-scaling a flat week
    const int ph = R5_GRAPH_BOT - R5_GRAPH_TOP; // plot travel
    int yh[R5_MAX_COLS], yl[R5_MAX_COLS];
    for (int i = 0; i < n; i++) {
      // The high (hero) series rides 3px above the shared scale for a little extra air
      // between the warm and cool lines; the lows stay anchored to the scale. Subtract the
      // section lift (ss) here so every downstream line/dot/label inherits the slide.
      yh[i] = okh[i] ? R5_GRAPH_BOT - (fan[i].today_high - vmin) * ph / vr - 3 - ss : 0;
      yl[i] = okl[i] ? R5_GRAPH_BOT - (fan[i].today_low  - vmin) * ph / vr - ss : 0;
    }
    // Centre the graph's visible span between the weather discs above and the PRECIPITATION banner
    // below, keyed off the ACTUAL peaks: yh_min = highest high dot, yl_max = lowest low dot; their
    // midpoint is the graph centre. Worked in section coords (+ss) so gshift is a constant the whole
    // fan still scrolls by. -1 corrects for the temp labels reaching ~2px further below the low dot
    // than above the high dot.
    int yh_min = 0, yl_max = 0;
    bool have_y = false;  // fold only known dots (highs sit above lows, so this matches
    prv_fold_minmax(yh, okh, n, &have_y, &yh_min, &yl_max);
    prv_fold_minmax(yl, okl, n, &have_y, &yh_min, &yl_max);
    const int dot_mid   = (yh_min + yl_max) / 2 + ss;                      // peak midpoint (section)
    // Band bounds in SCREEN coords, morphing with the section slide: at REST the peaks
    // centre between the disc bottoms and the SCREEN BOTTOM (hand-tuned); fully scrolled
    // they centre between the lifted discs and the stats pill top (145,
    // prv_draw_bottom_stats). avail_mid converts back to section coords (+ss) so gshift
    // stays a per-frame constant the whole fan scrolls by.
    const int H = layer_get_bounds(s_list->canvas).size.h;
    const int band_top_scr = R5_ICON_CY + R5_DISC_R - ss;
    const int band_bot_scr = H - (H - 145) * ss / R5_SECTION_TRAVEL;
    const int avail_mid = (band_top_scr + band_bot_scr) / 2 + ss;
    const int gshift = avail_mid - dot_mid - 1;
    for (int i = 0; i < n; i++) { yh[i] += gshift; yl[i] += gshift; }
    graphics_context_set_antialiased(ctx, true);
    // High (warm) and low (cold) links match in weight (2px) — the low one was previously a
    // thin recessive line that vanished on the real display. High link = muted Windsor Tan;
    // low link = blue.
    graphics_context_set_stroke_width(ctx, 2);
    prv_draw_series_links(ctx, col_low, yl, okl, n, GColorBlue);
    prv_draw_series_links(ctx, col_high, yh, okh, n, GColorWindsorTan);
    // Low dots match the high dots: hollow ring with a white centre — blue ring for the lows.
    for (int i = 0; i < n; i++) {
      if (okl[i]) prv_draw_ring_dot(ctx, GPoint(col_low[i], yl[i]), GColorBlue);
    }
    for (int i = 0; i < n; i++) {
      if (okh[i]) prv_draw_ring_dot(ctx, GPoint(col_high[i], yh[i]), GColorOrange);
    }
    // Numerals: both bold black — high ABOVE its dot, low BELOW its dot.
    graphics_context_set_text_color(ctx, GColorBlack);
    for (int i = 0; i < n; i++) {
      if (okh[i]) prv_draw_temp_centered(ctx, th[i], day_font, col_high[i], yh[i] - 4 - 16);
      if (okl[i]) prv_draw_temp_centered(ctx, tl[i], day_font, col_low[i], yl[i] + 4);
    }
  }
  // Bottom-gap banner stats (the PRECIPITATION pill with per-day values), slid up
  // into the gap as the section lifts.
  prv_draw_bottom_stats(ctx, fan, n, col_icon, W);
  return;
#endif
  // Scale each line to its OWN day-to-day range, in its own band (high band on top, low
  // band below), so the trend stays visible even when temps cluster — never a flat line.
  // A 4° floor stops a tiny spread from over-dramatising into a full-band swing.
  bool okh[R5_MAX_COLS], okl[R5_MAX_COLS];
  int hmin = 0, hmax = 0, lmin = 0, lmax = 0;
  bool haveh = false, havel = false;
  for (int i = 0; i < n; i++) {
    okh[i] = fan[i].today_high != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
    okl[i] = fan[i].today_low  != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
    if (okh[i]) {
      if (!haveh) { hmin = hmax = fan[i].today_high; haveh = true; }
      if (fan[i].today_high > hmax) hmax = fan[i].today_high;
      if (fan[i].today_high < hmin) hmin = fan[i].today_high;
    }
    if (okl[i]) {
      if (!havel) { lmin = lmax = fan[i].today_low; havel = true; }
      if (fan[i].today_low  > lmax) lmax = fan[i].today_low;
      if (fan[i].today_low  < lmin) lmin = fan[i].today_low;
    }
  }
  if (!haveh && !havel) return;  // every hi/lo unknown: nothing to plot
  int hrange = hmax - hmin; if (hrange < 2) hrange = 2;
  int lrange = lmax - lmin; if (lrange < 2) lrange = 2;
  // Scrolled-only shift/squeeze (see R5_GRAPH_SCROLL_*_DY): 0 at rest, full when scrolled.
  const int gtop = R5_GRAPH_TOP + R5_GRAPH_SCROLL_TOP_DY * ss / R5_SECTION_TRAVEL;
  const int gbot = R5_GRAPH_BOT + R5_GRAPH_SCROLL_BOT_DY * ss / R5_SECTION_TRAVEL;
  const int band   = (gbot - gtop) / R5_GRAPH_BAND_DIV;  // each line's vertical band
  const int hi_bot = gtop + band;                        // high band = [TOP, hi_bot]

  int y_hi[R5_MAX_COLS], y_lo[R5_MAX_COLS];
  for (int i = 0; i < n; i++) {
    // -ss: ride the section lift so the graph rises in lockstep with the discs above.
    y_hi[i] = okh[i] ? hi_bot - (fan[i].today_high - hmin) * band / hrange - ss : 0;
    y_lo[i] = okl[i] ? gbot - (fan[i].today_low - lmin) * band / lrange - ss : 0;
  }

  graphics_context_set_antialiased(ctx, true);
  graphics_context_set_stroke_width(ctx, 2);
  // Muted connector lines — quiet links so the saturated dots + labels read as the data.
  prv_draw_series_links(ctx, col_high, y_hi, okh, n, GColorWindsorTan);
  prv_draw_series_links(ctx, col_low, y_lo, okl, n, GColorCadetBlue);
  // Hollow dots: a coloured ring with a white centre (warm high, cool low).
  for (int i = 0; i < n; i++) {
    if (okh[i]) {
      graphics_context_set_fill_color(ctx, GColorOrange);
      graphics_fill_circle(ctx, GPoint(col_high[i], y_hi[i]), R5_DOT_R);
      graphics_context_set_fill_color(ctx, GColorWhite);
      graphics_fill_circle(ctx, GPoint(col_high[i], y_hi[i]), R5_DOT_INNER);
    }
    if (okl[i]) {
      graphics_context_set_fill_color(ctx, GColorVividCerulean);
      graphics_fill_circle(ctx, GPoint(col_low[i], y_lo[i]), R5_DOT_R);
      graphics_context_set_fill_color(ctx, GColorWhite);
      graphics_fill_circle(ctx, GPoint(col_low[i], y_lo[i]), R5_DOT_INNER);
    }
  }
  // Temperature labels: high ABOVE its warm dot, low BELOW its cool dot.
  for (int i = 0; i < n; i++) {
    char hs[16], ls[16];
    graphics_context_set_text_color(ctx, GColorBlack);  // black numerals; dots/lines stay coloured
    if (okh[i]) {
      snprintf(hs, sizeof(hs), "%d\xC2\xB0", fan[i].today_high);
      graphics_draw_text(ctx, hs, small_font,
          GRect(col_high[i] - box_w / 2, y_hi[i] - R5_DOT_R - 17, box_w, 16),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
    }
    if (okl[i]) {
      snprintf(ls, sizeof(ls), "%d\xC2\xB0", fan[i].today_low);
      graphics_draw_text(ctx, ls, small_font,
          GRect(col_low[i] - box_w / 2, y_lo[i] + R5_DOT_R + 1, box_w, 16),
          GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
    }
  }
  // Bottom-gap banner stats, slid up into the gap the lifting section opens.
  prv_draw_bottom_stats(ctx, fan, n, col_icon, W);
}
#endif

static void prv_canvas_draw(Layer *layer, GContext *ctx) {
#if PBL_ROUND
  // The window bg is transparent on round (step 3) — this proc is what guarantees full
  // coverage, so the degenerate no-state frame must paint the white itself.
  if (!s_list) {
    graphics_context_set_fill_color(ctx, GColorWhite);
    graphics_fill_rect(ctx, layer_get_bounds(layer), 0, GCornerNone);
    return;
  }
#else
  if (!s_list) return;
#endif
#if defined(PBL_PLATFORM_GABBRO)
  prv_canvas_draw_gabbro(layer, ctx);
  return;
#endif
#if WEATHER_ANIM_5DAY
  if (s_list->report_fx == 4) {
    // Stage 4 — the paper's bow: white card, the paper squash-stretching off the left
    // (its own points deform — Timeline smiley exit), the caption sliding out with it.
    graphics_context_set_fill_color(ctx, GColorWhite);
    graphics_fill_rect(ctx, layer_get_bounds(layer), 0, GCornerNone);
    prv_draw_exiting_paper(ctx);
    const int dx = (int)prv_moook_full(s_list->report_p, 0,
                                       -layer_get_bounds(layer).size.w);
    prv_draw_report_caption(ctx, dx, true);
    return;
  }
  if (PBL_IF_ROUND_ELSE(s_list->squash_mode && s_list->squash_captured &&
                            s_list->squash_scratch != NULL,
                        false)) {
    // Capture-once (gabbro smoothness step 2): the squash snapshot is already filled, and
    // prv_render_squash_in below overwrites every pixel from it — re-rendering the scene
    // here would be pure per-frame waste. Skip it.
  } else if (s_list->report_fx >= 2 && !PBL_IF_ROUND_ELSE(0, s_list->squash_mode)) {
    // Report scene, content cleared off-left: a pure-white title-card backdrop (matches the
    // squash memset and the report window bg — seamless at both cuts).
    graphics_context_set_fill_color(ctx, GColorWhite);
    graphics_fill_rect(ctx, layer_get_bounds(layer), 0, GCornerNone);
  } else
  prv_canvas_draw_round_5day(layer, ctx);
  // Pebble-Health "select" marker: a half-circle nub on the centre-right edge identifying the
  // SELECT button. Same radius-13 oval as applib action_button_draw, but pushed further off-screen
  // so it only protrudes ~5px (origin.x = W - protrusion) — shallow enough to clear the right-hand
  // weather disc. Only on the resting main view — hidden during any transition so it never gets
  // captured into the jelly/squash.
  if (s_list->header_shown && !s_list->fx_active && !s_list->clock_fx && !s_list->squash_mode &&
      !s_list->report_fx) {
    const int protrusion = 5;
    GRect mb = layer_get_bounds(layer);
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_oval(ctx, GRect(mb.size.w - protrusion, (mb.size.h - 26) / 2, 26, 26),
                       GOvalScaleModeFitCircle);
  }

#if PBL_ROUND
  // Up/down nav arrows, taken from the system Health app's card view rather than redrawn:
  // the same applib component (content_indicator_draw_arrow), the same full-width 18px bands
  // top and bottom (health/card_view.c: PBL_IF_ROUND_ELSE(18, 11)), and the same GAlignCenter —
  // Health leaves ContentIndicatorConfig.alignment unset and GAlignCenter is 0. Identical inputs
  // to the identical function, so these rasterise exactly as Health's do.
  //
  // Gated LOOSER than the SELECT nub above, and the two arrows differ:
  //   UP   — drawn on the resting main view AND on the scrolled precipitation view.
  //   DOWN — resting main view only (header_shown), like the nub.
  // Both are excluded during every transition, or they would be captured into the jelly/squash.
  if (!s_list->fx_active && !s_list->clock_fx && !s_list->squash_mode && !s_list->report_fx) {
    const GRect ab = layer_get_bounds(layer);
    const int arrow_band = 18;
    const GRect up_frame = GRect(0, 0, ab.size.w, arrow_band);
    content_indicator_draw_arrow(ctx, &up_frame, ContentIndicatorDirectionUp,
                                 GColorBlack, GColorWhite, GAlignCenter);
    if (s_list->header_shown) {
      const GRect down_frame = GRect(0, ab.size.h - arrow_band, ab.size.w, arrow_band);
      content_indicator_draw_arrow(ctx, &down_frame, ContentIndicatorDirectionDown,
                                   GColorBlack, GColorWhite, GAlignCenter);
    }
  }
#endif

  if (s_list->squash_mode) prv_render_squash_in(ctx);
  if (s_list->clock_fx) {
    // Burst dot: decoupled from the content — drawn AFTER the squash so it flies to centre
    // (stage 1) / orbital-shakes (stage 2) crisply while the frame jelly-stretches off the top.
    // Shared: it is the subject of the transition on both shapes.
    GRect mb = layer_get_bounds(layer);
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_circle(ctx, GPoint(mb.size.w / 2 + s_list->fx_shake_dx,
                                     s_list->fx_dot_y + s_list->fx_shake_dy), 6);
  }
#if !PBL_ROUND
  // Report scene: the ball (parked on the nub's spot during stage 1, rolling in stage 2)
  // and the unfolding paper + caption (stage 3), drawn after the squash like the burst
  // dot so they ride crisply over the clearing frame.
  if (s_list->report_fx == 1 || s_list->report_fx == 2) {
    prv_draw_report_ball(ctx);
  } else if (s_list->report_fx == 3) {
    prv_draw_unfolding_paper(ctx);
    prv_draw_report_caption(ctx, 0, false);
  }
#endif
  // Hero icon-fly + the card's glance text ride ON TOP of the squashed screen (both excluded
  // from the squash), landing together as the mainscreen clears. Shared: round plays the same
  // card transition. On the reverse fly (card -> forecast, riding the RISE_IN) only the icon
  // flies — there is no incoming card text.
  if (s_list->flying_icon) {
    prv_draw_flying_icon(ctx);
    if (s_list->squash_mode != SQUASH_RISE_IN) {
      prv_draw_flying_content(ctx);
    }
  }
#if PBL_ROUND
  // Round's report scene. (Rect draws it inside the block above, paired with the burst
  // dot and hero icon-fly — neither of which exists on round.)
  if (s_list->report_fx == 1 || s_list->report_fx == 2) {
    prv_draw_report_ball(ctx);
  } else if (s_list->report_fx == 3) {
    prv_draw_unfolding_paper(ctx);
    prv_draw_report_caption(ctx, 0, false);
  }
#endif
  return;
#endif
  GRect bounds = layer_get_bounds(layer);
  int W   = bounds.size.w;
  int H   = bounds.size.h;
  int off = s_list->scroll_offset_px;

  // Background
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);

  // Hoist context state changes outside the row loop.
  graphics_context_set_stroke_color(ctx, PBL_IF_COLOR_ELSE(GColorLightGray, GColorBlack));
  graphics_context_set_compositing_mode(ctx, GCompOpSet);

  int rh = s_list->row_height;
  const int side_inset = ROUND_SIDE_INSET;
  const int icon_x = ICON_X + side_inset;
  const int day_label_x = DAY_LABEL_X + side_inset;
  const int right_margin = CONDITION_RIGHT_MARGIN + side_inset;
  for (int i = 0; i < (int)s_list->num_days; i++) {
    int ry = LIST_TOP_Y + i * rh - off;
    if (ry + rh <= LIST_TOP_Y || ry >= H) continue;

    const WeatherLocationForecast *f = &s_list->days[i];

    // Row separator (skip very first row)
    if (ry > LIST_TOP_Y) {
      graphics_draw_line(ctx, GPoint(day_label_x, ry),
                         GPoint(W - right_margin, ry));
    }

    // Coloured circle — use fill_circle (much faster than fill_radial for full circles)
    int ic_cx = icon_x + ICON_SIZE / 2;
    int ic_cy = ry + (rh - ICON_SIZE) / 2 + ICON_SIZE / 2;
    int radius = ICON_SIZE * 7 / 10;  // same as diam/2 = ICON_SIZE*14/10/2
    GColor bg = prv_weather_bg_color(f->current_weather_type);
    if (!gcolor_equal(bg, GColorClear)) {
      graphics_context_set_fill_color(ctx, bg);
      graphics_fill_circle(ctx, GPoint(ic_cx, ic_cy), (uint16_t)radius);
    }

    // Icon
    if (s_list->icons[i]) {
      int iy = ry + (rh - ICON_SIZE) / 2;
      graphics_draw_bitmap_in_rect(ctx, s_list->icons[i],
          GRect(icon_x, iy, ICON_SIZE, ICON_SIZE));
    }

    // Vertical text baseline — subtract 3px to correct for GOTHIC_18 internal top padding
    // so glyphs are visually centred in the row rather than sitting low.
    int ty = ry + (rh - 18) / 2 - 3;

    int divider_x = day_label_x + DAY_LABEL_W + DAY_CONDITION_DIVIDER_GAP;
    int condition_x = divider_x + CONDITION_TEXT_LEFT_GAP;
    int condition_w = W - right_margin - condition_x;
    if (condition_w < 20) condition_w = 20;

    // Day label
    char weekday_label[16];
    prv_fill_weekday_label(i, f->label, weekday_label, sizeof(weekday_label));
    graphics_context_set_text_color(ctx, GColorBlack);
    graphics_draw_text(ctx, weekday_label, s_list->day_font,
        GRect(day_label_x, ty, DAY_LABEL_W, 20),
        GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);

    graphics_context_set_stroke_color(ctx, PBL_IF_COLOR_ELSE(GColorLightGray, GColorBlack));
    graphics_draw_line(ctx, GPoint(divider_x, ry + 5), GPoint(divider_x, ry + rh - 5));

    // Hi / Lo temperature — use pre-formatted string, zero allocation per frame
    graphics_draw_text(ctx, s_list->condition_str[i], s_list->condition_font,
        GRect(condition_x, ty, condition_w, 20),
        GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
  }

  // Scroll indicator
  int max_s = prv_max_scroll();
  if (max_s > 0) {
    int scroll_h = H - LIST_TOP_Y;
    int total_h  = (int)s_list->num_days * rh;
    int bar_h    = scroll_h * scroll_h / total_h;
    int bar_y    = LIST_TOP_Y + off * scroll_h / total_h;
    graphics_context_set_fill_color(ctx, GColorLightGray);
    graphics_fill_rect(ctx, GRect(W - side_inset - 3, bar_y, 3, bar_h),
                       0, GCornerNone);
  }

  // Blue location bar drawn last (covers any row that scrolled under it)
  graphics_context_set_fill_color(ctx, GColorVividCerulean);
  graphics_fill_rect(ctx, GRect(0, LOCATION_BAR_Y, W, LOCATION_BAR_H),
                     0, GCornerNone);
  graphics_context_set_text_color(ctx, GColorWhite);
  GFont bar_font = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
  const int bar_inset = ROUND_BAR_INSET;
  // Location name: left-aligned
  const char *loc = (s_list->num_days > 0 && s_list->days[0].location_name[0])
                      ? s_list->days[0].location_name : "";
  graphics_draw_text(ctx, loc, bar_font,
      GRect(bar_inset + 4, LOCATION_BAR_Y + 1,
            W - (bar_inset * 2) - 52, LOCATION_BAR_H),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
  // Digital time: right-aligned (matches main screen)
  char time_str[8];
  time_t now = time(NULL);
  struct tm *lt = localtime(&now);
  if (clock_is_24h_style()) {
    strftime(time_str, sizeof(time_str), "%H:%M", lt);
  } else {
    strftime(time_str, sizeof(time_str), "%I:%M", lt);
    if (time_str[0] == '0') memmove(time_str, time_str + 1, sizeof(time_str) - 1);
  }
  graphics_draw_text(ctx, time_str, bar_font,
      GRect(W - bar_inset - 50, LOCATION_BAR_Y + 1, 48, LOCATION_BAR_H),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentRight, NULL);
}

// ---- Touch input (Emery only) ----

#if WEATHER_PLATFORM_TOUCH_COLOR
// Touch scroll/fling + swipe-navigation removed (the 5-day mainscreen no longer takes touch nav;
// gestures will be re-added against the current layout later). The icon-interaction helpers below
// stay — they are also driven by the button click handler.

#if !PBL_ROUND
static void prv_sun_tick(void *ctx);  // fwd decl: the rect timer path re-registers it
#endif
static void prv_icon_driver_start(void);

// True if today's condition has an animated icon (so there is a fade/timer to wake).
static bool prv_today_animated(void) {
  if (!s_list || s_list->num_days == 0) return false;
  switch (prv_header_type()) {
    case WeatherType_Sun: case WeatherType_PartlyCloudy: case WeatherType_CloudyDay:
    case WeatherType_LightRain: case WeatherType_HeavyRain:
    case WeatherType_LightSnow: case WeatherType_HeavySnow: case WeatherType_RainAndSnow:
      return true;
    default: return false;
  }
}

// Any interaction resets the 10s power-save countdown and, if the icon had already frozen
// to its static state, wakes the colour animation back up.
static void prv_note_icon_interaction(void) {
  if (!prv_today_animated()) return;
  s_list->idle_ticks = 0;
  if (s_list->icon_fade < R5_FADE_MAX) {
    s_list->icon_fade = R5_FADE_MAX;
    prv_icon_driver_start();
  }
}

#endif

// ---- Button input ----
// (The select-exit statics now live with the report-scene block above.)

#if !PBL_ROUND
// ---- First-entry clock-slot intro: location -> time (the sunset card's swap) ----
// City-only copy (split at the comma, like the round location bar).
static void prv_clock_loc_city(char *dst, size_t dst_size, const char *src) {
  size_t i = 0;
  for (; src && src[i] && src[i] != ',' && i < dst_size - 1; i++) dst[i] = src[i];
  while (i > 0 && dst[i - 1] == ' ') i--;
  dst[i] = '\0';
}

static void prv_clock_swap_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_list) return;
  s_list->clock_swap_p = progress;
  if (s_list->canvas) layer_mark_dirty(s_list->canvas);
}

static void prv_clock_swap_stopped(Animation *anim, bool finished, void *context) {
  (void)finished; (void)context;
  if (s_list) {
    if (s_list->clock_swap_anim == anim) s_list->clock_swap_anim = NULL;
    s_list->clock_swap_active = false;
    s_list->clock_loc_show = false;   // the slot now rests on the time
    if (s_list->canvas) layer_mark_dirty(s_list->canvas);
  }
  animation_destroy(anim);
}

static const AnimationImplementation s_clock_swap_impl = { .update = prv_clock_swap_update };

static void prv_clock_loc_timer_cb(void *ctx) {
  (void)ctx;
  if (!s_list) return;
  s_list->clock_loc_timer = NULL;
  if (!s_list->clock_loc_show || s_list->clock_swap_anim) return;
  s_list->clock_swap_active = true;
  s_list->clock_swap_p = 0;
  s_list->clock_swap_anim = animation_create();
  if (!s_list->clock_swap_anim) {   // OOM: hard cut to the time
    s_list->clock_swap_active = false;
    s_list->clock_loc_show = false;
    if (s_list->canvas) layer_mark_dirty(s_list->canvas);
    return;
  }
  animation_set_implementation(s_list->clock_swap_anim, &s_clock_swap_impl);
  animation_set_duration(s_list->clock_swap_anim, interpolate_moook_soft_duration(3));
  animation_set_curve(s_list->clock_swap_anim, AnimationCurveLinear);  // the moook shapes it
  animation_set_handlers(s_list->clock_swap_anim,
                         (AnimationHandlers){ .stopped = prv_clock_swap_stopped }, NULL);
  animation_schedule(s_list->clock_swap_anim);
}

// Re-arm the intro (globe city commit): the next appear — the globe's self-pop lands
// straight on this window — replays the location hold + swap with the NEW city.
#endif  // !PBL_ROUND (clock-slot intro draws only on the rect mainscreen)

void forecast_list_replay_location_intro(void) {
#if PBL_ROUND
  // Round's mainscreen has no clock-slot intro — the API stays for weather.c.
#else
  if (!s_list) return;
  if (s_list->clock_loc_timer) {
    app_timer_cancel(s_list->clock_loc_timer);
    s_list->clock_loc_timer = NULL;
  }
  if (s_list->clock_swap_anim) {   // null-first: .stopped only destroys
    Animation *a = s_list->clock_swap_anim;
    s_list->clock_swap_anim = NULL;
    animation_unschedule(a);
  }
  s_list->clock_swap_active = false;
  s_list->clock_loc_show = false;
  s_list->clock_loc_done = false;   // the appear-arming runs again
#endif
}

// Start the 2s hold — called when the screen first appears.
#if !PBL_ROUND
static void prv_clock_loc_hold(void) {
  if (!s_list || !s_list->clock_loc_show || s_list->clock_loc_timer ||
      s_list->clock_swap_active) {
    return;
  }
  s_list->clock_loc_timer = app_timer_register(2000, prv_clock_loc_timer_cb, NULL);
}
#endif

static void prv_click_up_down(ClickRecognizerRef recognizer, void *context) {
  if (!s_list) return;
  bool down = click_recognizer_get_button_id(recognizer) == BUTTON_ID_DOWN;
#if WEATHER_ANIM_5DAY
  // Mirror prv_click_select: while an exclusive transition owns the screen (clock burst,
  // squash to/from the card, hero icon-fly, select-exit slide) UP/DOWN presses are dropped —
  // starting a header transition mid-flight would fight it for header_scroll and desync the
  // resting state. Header transitions themselves stay interruptible (hasted re-press).
  if (s_list->clock_fx || s_list->flying_icon || s_select_exit_anim || s_list->report_fx
#if !PBL_ROUND
      || s_list->squash_mode
#endif
  ) {
    return;
  }
  // DOWN slides the today header off the top (State A->B); UP brings it back (B->A). Both use
  // the shared moook transition and never fall through to vertical scroll / city-select (the
  // 5-day view is a single screen). Vertical swipes (prv_touch_handler) drive these same
  // transitions.
  if (down) {
    if (s_list->header_shown) prv_start_header_transition(true);
    else                      prv_start_clock_stage1();   // DOWN again when scrolled -> clock burst
  } else {
    if (!s_list->header_shown) prv_start_header_transition(false);
    else                       prv_start_up_to_card();              // UP at top -> squash + hero icon-fly to the card
  }
  prv_note_icon_interaction();  // wake a faded today icon so it animates during the move
  return;
#endif
  if (down && s_list->scroll_to >= prv_max_scroll()) {
#if defined(PBL_PLATFORM_GABBRO)
    return;
#else
    if (s_list->on_city_select_cb) {
      s_list->on_city_select_cb(s_list->on_city_select_ctx);
    }
    return;
#endif
  }
#if defined(PBL_PLATFORM_GABBRO)
  if (click_recognizer_is_repeating(recognizer)) {
    prv_scroll_to_menu_repeat(s_list->scroll_to +
                              (down ? s_list->row_height : -s_list->row_height));
    return;
  }
#endif
  prv_scroll_to(s_list->scroll_to + (down ? s_list->row_height : -s_list->row_height));
}

static void prv_click_back(ClickRecognizerRef recognizer, void *context) {
  app_window_stack_pop_all(true);  // BACK exits the app from any ring page
}

// ---- SELECT expand, fallback slide (round / squash-scratch OOM): the main forecast slides
// off to the left, then hard-cuts — the analog of Timeline's prv_animate_to_pin_window
// (timeline.c:398). On rect the full squash->ball->unfold scene above replaces this.

static void prv_select_exit_update(Animation *anim, AnimationProgress progress) {
  (void)anim;
  if (!s_list || !s_list->canvas) return;
  const int w = layer_get_bounds(s_list->canvas).size.w;
  GRect f = layer_get_frame_by_value(s_list->canvas);
  f.origin.x = (int)interpolate_moook(progress, 0, -w);
  layer_set_frame(s_list->canvas, f);
}

static void prv_select_exit_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)finished; (void)context;
  s_select_exit_anim = NULL;
  if (!s_list) return;
#if PBL_ROUND
  // Fallback slide over — back to the transparent bg (step 3); the canvas frame is reset
  // to full coverage below, so the invariant holds again.
  if (s_list->window) window_set_background_color(s_list->window, GColorClear);
#endif
  // Reset the (base) main window to rest so a later BACK returns cleanly, then fire the hard cut.
  if (s_list->canvas) {
    GRect f = layer_get_frame_by_value(s_list->canvas);
    f.origin.x = 0;
    layer_set_frame(s_list->canvas, f);
  }
  void (*done)(void *) = s_select_exit_done;
  void *ctx = s_select_exit_ctx;
  s_select_exit_done = NULL;
  s_select_exit_ctx = NULL;
  if (done) done(ctx);
}

static const AnimationImplementation s_select_exit_impl = { .update = prv_select_exit_update };

bool forecast_list_get_today_icon_rect(GRect *out) {
  if (!s_list || !out) return false;
#if WEATHER_ANIM_5DAY && !PBL_ROUND
  if (!s_list->header_shown) return false;
  *out = GRect(s_list->today_icon_x, R5_TODAY_Y, R5_TODAY_ICON, R5_TODAY_ICON);
  return true;
#else
  return false;
#endif
}

void forecast_list_start_select_exit(void (*done_cb)(void *ctx), void *ctx) {
  if (!s_list || !s_list->canvas) { if (done_cb) done_cb(ctx); return; }
  if (s_select_exit_anim) return;
#if WEATHER_ANIM_5DAY
  if (s_report_anim || s_list->report_fx) return;
  // Mid-transition: REFUSE (the press is ignored) rather than fall back to the legacy
  // slide — the fallback exists for genuine scratch-OOM at rest, not as a degraded scene
  // for badly-timed presses. Belt to prv_click_select's braces; both shapes.
  if (s_list->squash_mode || s_list->clock_fx || s_list->flying_icon) return;
#endif
  s_select_exit_done = done_cb;
  s_select_exit_ctx  = ctx;
#if WEATHER_ANIM_5DAY
  // The full scene: whole-frame squash-left (stage 1), ball roll-in (2), unfold (3).
  if (!s_list->squash_mode) {
    if (s_list->anim) {   // capture-first, same defensive kill as prv_start_clock_stage1
      Animation *old = s_list->anim;
      s_list->anim = NULL;
      animation_unschedule(old);
      animation_destroy(old);
    }
    GRect b = layer_get_bounds(s_list->canvas);
    s_list->squash_scratch = malloc_try((size_t)b.size.w * (size_t)b.size.h);
    if (s_list->squash_scratch) {
      s_list->squash_mode = SQUASH_LEFT_EXIT;
      s_list->squash_in_p = 0;
      s_list->squash_captured = false;   // round capture-once
      s_list->report_fx   = 1;
      // The ball is born on the SELECT nub's own spot (decoupled from the squash, like
      // the burst dot) — the indicator visibly becomes the ball.
      s_list->fx_ball_x   = R5_REPORT_BALL_X0(b.size.w);
      s_select_exit_anim = prv_start_anim(interpolate_moook_soft_duration(3),  // 330ms
                                          AnimationCurveLinear,   // jelly does the shaping
                                          &s_report_stage1_impl, prv_report_stage1_stopped);
      return;
    }
  }
#endif
  // Fallback (scratch OOM on either shape): the legacy 115ms moook slide; done cb fires in its .stopped.
#if PBL_ROUND
  // The slide moves the CANVAS FRAME, exposing window pixels nobody paints now that the
  // window bg is transparent (step 3) — the vacated strip would smear stale frames. Restore
  // the white bg for just this rare 115ms fallback; prv_select_exit_stopped re-clears it.
  window_set_background_color(s_list->window, GColorWhite);
#endif
  s_select_exit_anim = prv_start_anim(interpolate_moook_duration() / 2,  // 115ms (Timeline)
                                      AnimationCurveLinear,               // moook is the shaping
                                      &s_select_exit_impl, prv_select_exit_stopped);
}

static void prv_click_select(ClickRecognizerRef recognizer, void *context) {
  (void)recognizer;
  (void)context;
  if (!s_list || !s_list->on_select_request_cb) return;
#if WEATHER_ANIM_5DAY
  // Only from the resting main view (State A, no transition in flight) — same condition as the
  // "select" marker that advertises this button. BOTH shapes: the !PBL_ROUND on this gate
  // dated from before round drove the squashes — without it, SELECT pressed during a
  // DROP_IN/RISE_IN/DOWN_EXIT window skipped the whole newspaper scene via the legacy
  // fallback slide (and during the clock burst it raced the clock push).
  if (!s_list->header_shown || s_list->fx_active || s_list->clock_fx || s_list->squash_mode ||
      s_select_exit_anim || s_list->report_fx) return;
#endif
  s_list->on_select_request_cb(s_list->on_select_request_ctx);  // -> weather.c pushes the report
}

// ---- Touch input (touch colour platforms) ----
#if WEATHER_PLATFORM_TOUCH_COLOR
#define SWIPE_THRESHOLD 20   // px; same tap/swipe split as clock_face + the original app

static void prv_touch_handler(const TouchEvent *event, void *context) {
  (void)context;
  if (!s_list) return;
  if (event->type == TouchEvent_Touchdown) {
    s_list->touch_start_x = event->x;
    s_list->touch_start_y = event->y;
    s_list->touch_active  = true;
  } else if (event->type == TouchEvent_Liftoff && s_list->touch_active) {
    s_list->touch_active = false;
    // Same exclusive-transition guard as the buttons — see prv_click_up_down.
    // BOTH shapes, including squash_mode: the rect-only gate dated from when round never
    // drove a squash — round runs UP_EXIT/DROP_IN/LEFT_EXIT now, and an unguarded swipe
    // mid-squash could double-start a transition (same class of bug as the stranded
    // stage-1 teardown).
    if (s_list->clock_fx || s_list->flying_icon || s_select_exit_anim || s_list->report_fx ||
        s_list->squash_mode) {
      return;
    }
    int16_t dx = event->x - s_list->touch_start_x;
    int16_t dy = event->y - s_list->touch_start_y;
    int16_t adx = dx < 0 ? -dx : dx;
    int16_t ady = dy < 0 ? -dy : dy;
    if (adx <= SWIPE_THRESHOLD && ady <= SWIPE_THRESHOLD) return;   // tap — no action here
    if (ady >= adx) {
      // Vertical swipe follows the finger: UP pulls the deeper screens up (scrolled stats,
      // then the clock burst — the DOWN button); DOWN backs out (rest, then the card — UP).
      if (dy < 0) {
        if (s_list->header_shown) prv_start_header_transition(true);
        else                      prv_start_clock_stage1();
      } else {
        if (!s_list->header_shown) prv_start_header_transition(false);
        else                       prv_start_up_to_card();   // BOTH shapes: the hero fly +
                                                             // card open work on round
      }
      prv_note_icon_interaction();
    } else if (dx < 0) {
      // Swipe LEFT from the resting main view opens the condensed view (the SELECT slide).
      if (s_list->header_shown && !s_list->fx_active && s_list->on_select_request_cb) {
        s_list->on_select_request_cb(s_list->on_select_request_ctx);
      }
    }
  }
}
#endif

// ---- Hold-to-scroll ----
#if !defined(PBL_PLATFORM_GABBRO) && !WEATHER_ANIM_5DAY
// Rectangular builds keep the previous physics-style continuous scrolling.
// A 16ms ticker nudges scroll_offset_px by a velocity that ramps up each tick.
// Velocity stored in 1/16 px units to allow smooth sub-pixel accumulation.
#define SCROLL_TICK_MS    16   // ~60fps
#define SCROLL_VEL_START  16   // initial:  1 px/frame  (units = 1/16 px)
#define SCROLL_VEL_MAX    96   // maximum:  6 px/frame
#define SCROLL_VEL_ACCEL   2   // added to velocity each tick

static AppTimer *s_list_hold_timer = NULL;
static bool      s_list_hold_down  = false;
static int       s_list_velocity   = 0;
static int       s_list_subpx      = 0;

static void prv_list_tick_cb(void *ctx) {
  s_list_hold_timer = NULL;
  if (!s_list) return;

  s_list_velocity += SCROLL_VEL_ACCEL;
  if (s_list_velocity > SCROLL_VEL_MAX) s_list_velocity = SCROLL_VEL_MAX;

  s_list_subpx += s_list_velocity;
  int delta = s_list_subpx / 16;
  s_list_subpx %= 16;

  if (delta > 0) {
    int next = s_list->scroll_offset_px + (s_list_hold_down ? delta : -delta);
    int max  = prv_max_scroll();
    if (next < 0) next = 0;
    if (next > max) next = max;

    // Cancel any in-flight eased animation and take direct control
    if (s_list->anim) {
      animation_unschedule(s_list->anim);
      animation_destroy(s_list->anim);
      s_list->anim = NULL;
    }
    s_list->scroll_offset_px = next;
    s_list->scroll_to        = next;
    s_list->scroll_from      = next;
    layer_mark_dirty(s_list->canvas);
  }

  s_list_hold_timer = app_timer_register(SCROLL_TICK_MS, prv_list_tick_cb, NULL);
}

static void prv_list_raw(ButtonId btn, bool pressed) {
  if (pressed) {
    s_list_hold_down = (btn == BUTTON_ID_DOWN);
    s_list_velocity  = SCROLL_VEL_START;
    s_list_subpx     = 0;
    if (s_list_hold_timer) { app_timer_cancel(s_list_hold_timer); s_list_hold_timer = NULL; }
    // 300ms hold threshold before continuous scroll begins
    s_list_hold_timer = app_timer_register(300, prv_list_tick_cb, NULL);
  } else {
    if (s_list_hold_timer) { app_timer_cancel(s_list_hold_timer); s_list_hold_timer = NULL; }
  }
}

static void prv_list_raw_up_press(ClickRecognizerRef r, void *ctx)   { prv_list_raw(BUTTON_ID_UP,   true);  }
static void prv_list_raw_up_rel(ClickRecognizerRef r, void *ctx)     { prv_list_raw(BUTTON_ID_UP,   false); }
static void prv_list_raw_down_press(ClickRecognizerRef r, void *ctx) { prv_list_raw(BUTTON_ID_DOWN, true);  }
static void prv_list_raw_down_rel(ClickRecognizerRef r, void *ctx)   { prv_list_raw(BUTTON_ID_DOWN, false); }
#endif

static void prv_click_provider(void *context) {
#if defined(PBL_PLATFORM_GABBRO)
  window_single_repeating_click_subscribe(BUTTON_ID_UP, 100, prv_click_up_down);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, 100, prv_click_up_down);
#else
  window_single_click_subscribe(BUTTON_ID_UP,     prv_click_up_down);
  window_single_click_subscribe(BUTTON_ID_DOWN,   prv_click_up_down);
#endif
  window_single_click_subscribe(BUTTON_ID_BACK,   prv_click_back);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_click_select);
#if !defined(PBL_PLATFORM_GABBRO) && !WEATHER_ANIM_5DAY
  // The legacy continuous hold-to-scroll spins prv_list_tick_cb against scroll_offset_px,
  // which is meaningless for the single-screen 5-day view (max scroll 0) and would consume
  // a held DOWN press alongside the header transition. Disabled for WEATHER_ANIM_5DAY.
  window_raw_click_subscribe(BUTTON_ID_UP,   prv_list_raw_up_press,   prv_list_raw_up_rel,   NULL);
  window_raw_click_subscribe(BUTTON_ID_DOWN, prv_list_raw_down_press, prv_list_raw_down_rel, NULL);
#endif
}

// ---- Window lifecycle ----

#if WEATHER_ANIM_5DAY
// One 80ms quantum of icon animation. Returns false when the driver should stop entirely
// (covered by another window, or faded fully static); prv_window_appear rearms on return.
static bool prv_sun_step(void) {
  if (!s_list) return false;
  if (!s_list->window || window_stack_get_top_window() != s_list->window) return false;
  s_list->idle_ticks++;
  // Power saving: once idle for 10s, ramp icon_fade down to 0 (fading the colour overlay
  // into the static icon). Only advance the animation phase while there is still motion.
  if (s_list->icon_fade == R5_FADE_MAX && s_list->idle_ticks >= R5_IDLE_TICKS) {
    s_list->icon_fade = R5_FADE_MAX - 1;            // begin the fade-to-static
  } else if (s_list->icon_fade > 0 && s_list->icon_fade < R5_FADE_MAX) {
    s_list->icon_fade--;                            // continue fading
  }
  if (s_list->icon_fade > 0) {
    // Rect: 2 phase steps per 80ms tick preserves the visual speed of the old 40ms cadence
    // while halving the redraw rate (the full-screen redraw was over the per-frame budget on
    // real hardware, backing up the task that services touch).
    // Round: the 66ms clean-rate tick (step 4) carries 66/80 of that advance — 33*STEP/20 —
    // so degrees-per-ms is unchanged (0.1% under, invisible on a cyclic rotation).
    s_list->sun_phase += PBL_IF_ROUND_ELSE((33 * R5_SUN_ANGLE_STEP) / 20,
                                           2 * R5_SUN_ANGLE_STEP);
  }
  if (s_list->canvas) {
    layer_mark_dirty(s_list->canvas);
  }
  // Stop ticking once fully static — the screen is now frozen and draws no more frames.
  return s_list->icon_fade > 0;
}

#if !PBL_ROUND
static void prv_sun_tick(void *ctx) {
  (void)ctx;
  if (!s_list) return;
  s_list->sun_timer = NULL;
  if (prv_sun_step()) {
    s_list->sun_timer = app_timer_register(R5_SUN_PERIOD_MS, prv_sun_tick, NULL);
  }
}
#else
static uint32_t prv_now_ms32(void) {
  time_t s = 0; uint16_t ms = 0;
  time_ms(&s, &ms);
  return (uint32_t)s * 1000u + ms;
}

static void prv_icon_drv_stop_cb(void *ctx) {   // 0ms hop: never unschedule from inside .update
  (void)ctx;
  if (s_list && s_list->icon_drv) {
    Animation *a = s_list->icon_drv;
    s_list->icon_drv = NULL;
    animation_unschedule(a);
    animation_destroy(a);
  }
}

static void prv_icon_drv_update(Animation *anim, AnimationProgress p) {
  (void)anim; (void)p;   // INFINITE animations tick with progress 0 — time is tracked here
  if (!s_list) return;
  const uint32_t now = prv_now_ms32();
  uint32_t dt = now - s_list->icon_drv_last_ms;
  s_list->icon_drv_last_ms = now;
  if (dt > 500) dt = 500;                       // clock hiccup / long-suspend guard
#if WEATHER_ANIM_5DAY
  // Capture-once companion: while a squash freezes the scene at its snapshot, PAUSE the
  // icon's clock instead of letting the phase advance invisibly — otherwise the rays/drops
  // jump ~4 ticks the instant the live scene resumes. Swallowing dt here means the icon
  // resumes exactly at the pose the snapshot froze it in.
  if (s_list->squash_mode) return;
#endif
  s_list->icon_drv_acc_ms += dt;
  bool alive = true;
  while (alive && s_list->icon_drv_acc_ms >= R5_SUN_PERIOD_MS) {
    s_list->icon_drv_acc_ms -= R5_SUN_PERIOD_MS;
    alive = prv_sun_step();
  }
  if (!alive) app_timer_register(0, prv_icon_drv_stop_cb, NULL);
}
static const AnimationImplementation s_icon_drv_impl = { .update = prv_icon_drv_update };
#endif

// Start the icon animation driver, whichever kind this shape uses. Self-guarding.
static void prv_icon_driver_start(void) {
  if (!s_list) return;
#if PBL_ROUND
  if (s_list->icon_drv) return;
  Animation *a = animation_create();
  if (!a) return;
  animation_set_duration(a, ANIMATION_DURATION_INFINITE);
  animation_set_implementation(a, &s_icon_drv_impl);
  s_list->icon_drv_last_ms = prv_now_ms32();
  s_list->icon_drv_acc_ms  = 0;
  s_list->icon_drv = a;
  animation_schedule(a);
#else
  if (!s_list->sun_timer) {
    s_list->sun_timer = app_timer_register(R5_SUN_PERIOD_MS, prv_sun_tick, NULL);
  }
#endif
}
#endif

static void prv_window_load(Window *window) {
  GRect bounds = layer_get_bounds(window_get_root_layer(window));
#if defined(PBL_PLATFORM_GABBRO)
  s_list->row_height = GABBRO_LIST_ROW_PITCH;
#else
  // Divide available height evenly so exactly ROWS_VISIBLE rows fill the screen.
  s_list->row_height = (bounds.size.h - LIST_TOP_Y) / ROWS_VISIBLE;
#endif
  // Apply initial scroll so the focused day is at the top.
  {
    int initial = s_list->start_day_index * s_list->row_height;
    int max_s   = prv_max_scroll();
    if (initial > max_s) initial = max_s;
    if (initial < 0) initial = 0;  // 0 rows: max_s goes negative — never rest above the top
    s_list->scroll_offset_px = initial;
    s_list->scroll_to        = initial;
    s_list->scroll_from      = initial;
  }
  s_list->canvas = layer_create(bounds);
  layer_set_update_proc(s_list->canvas, prv_canvas_draw);
  layer_add_child(window_get_root_layer(window), s_list->canvas);

#if defined(PBL_PLATFORM_GABBRO)
  s_list->day_font       = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  s_list->condition_font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
  s_list->loc_font       = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
#else
  s_list->day_font       = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
  s_list->condition_font = s_list->day_font;
  s_list->loc_font       = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
#endif

  prv_load_icons();
  prv_format_conditions();

#if WEATHER_ANIM_5DAY
  s_list->header_shown  = true;   // State A: today header visible (calloc zeroed header_scroll)
  s_list->header_scroll = 0;
  if (s_list->num_days > 0 &&
      (prv_header_type() == WeatherType_Sun ||
       prv_header_type() == WeatherType_PartlyCloudy ||
       prv_header_type() == WeatherType_CloudyDay ||
       prv_header_type() == WeatherType_LightRain ||
       prv_header_type() == WeatherType_HeavyRain ||
       prv_header_type() == WeatherType_LightSnow ||
       prv_header_type() == WeatherType_HeavySnow ||
       prv_header_type() == WeatherType_RainAndSnow)) {
    s_list->sun_phase = 0;
    s_list->idle_ticks = 0;
    s_list->icon_fade = R5_FADE_MAX;   // start animating; fades to static after 10s idle
    prv_icon_driver_start();
  }
#endif
}

static void prv_hslide_in_stopped(Animation *anim, bool finished, void *context) {
  (void)anim; (void)finished; (void)context;
  s_hslide_anim = NULL;   // property animation auto-destroys after a normal stop
  if (s_list && s_list->canvas) {
    GRect home = layer_get_frame(s_list->canvas);
    home.origin.x = 0;
    layer_set_frame(s_list->canvas, home);
  }
}

void forecast_list_arm_hslide_in(void) {
  s_pending_hslide_in = true;
}

static void prv_window_appear(Window *window) {
  (void)window;
  // First-entry clock-slot intro: show the active location for 2s, then swap to the
  // time (sunset-card swap). Latched once per app run.
#if !PBL_ROUND
  if (s_list && !s_list->clock_loc_done && s_list->num_days > 0 &&
      s_list->days[0].location_name && s_list->days[0].location_name[0]) {
    s_list->clock_loc_done = true;
    s_list->clock_loc_show = true;
    prv_clock_loc_hold();
  }
#endif
#if WEATHER_PLATFORM_TOUCH_COLOR
  // (Re)claim the touch slot — a popping child (condensed/card/clock) unsubscribed in its
  // unload, which runs BEFORE this appear (window_transition_context_appearance_call_all).
  if (s_list) touch_service_subscribe(prv_touch_handler, s_list);
#endif
#if WEATHER_ANIM_5DAY
  // Rearm the icon driver if it parked itself while this window was covered.
  if (s_list && s_list->icon_fade > 0) {
    prv_icon_driver_start();
  }
#endif
#if WEATHER_ANIM_5DAY
  // Returning from the clock via UP: jelly-rise the forecast back in. Armed before the clock
  // dismissed, so this fires on the first revealed frame — no flash of the rested screen.
  if (s_list && s_pending_hslide_in && s_list->canvas) {
    s_pending_hslide_in = false;
    GRect to = layer_get_frame(s_list->canvas);
    GRect from = to;
    from.origin.x -= to.size.w;                    // start one screen-width off-left
    layer_set_frame(s_list->canvas, from);
    PropertyAnimation *pa = property_animation_create_layer_frame(s_list->canvas, &from, &to);
    if (pa) {
      Animation *anim = (Animation *)pa;
      animation_set_duration(anim, WEATHER_HSLIDE_MS);
      animation_set_custom_interpolation(anim, weather_interpolate_moook_soft1);
      animation_set_handlers(anim,
          (AnimationHandlers){ .stopped = prv_hslide_in_stopped }, NULL);
      s_hslide_anim = anim;
      animation_schedule(anim);
    } else {
      layer_set_frame(s_list->canvas, to);         // OOM: appear in place
    }
  }
  if (s_list && s_pending_squash_mode) {
    int m = s_pending_squash_mode;
    s_pending_squash_mode = 0;
    // Card -> forecast return: fly the card's icon back down into the header
    // (the forward hero in reverse), synced to the rise-in it rides on.
    if (s_pending_return_fly && m == SQUASH_RISE_IN && !s_list->flying_icon &&
        s_list->num_days > 0) {
      // prv_header_type(), not current_weather_type: the return fly lands on the header,
      // which is typed by the hourly 'now' on round — a synced-vs-hourly disagreement
      // otherwise repaints the icon's type at the landing instant. (The card it leaves is
      // now typed the same way — see expanded_view's matching change.) Rect: identical.
      GDrawCommandImage *raw = gdraw_command_image_create_with_resource(
          weather_type_icon_large_resource(prv_header_type()));
      GDrawCommandImage *pdc = raw ? gdraw_command_image_clone(raw) : NULL;
      if (raw) gdraw_command_image_destroy(raw);
      if (pdc) {
        const int W = layer_get_bounds(s_list->canvas).size.w;
        s_list->fly_pdc  = pdc;
        s_list->fly_src  = GRect((W - EV_ICON_SIZE) / 2, EV_ICON_Y,
                                 EV_ICON_SIZE, EV_ICON_SIZE);
        s_list->fly_dest = GRect(s_list->today_icon_x, R5_TODAY_Y,
                                 R5_TODAY_ICON, R5_TODAY_ICON);
        s_list->flying_icon = true;
      }
    }
    s_pending_return_fly = false;
    prv_start_squash(m);   // DROP_IN (from clock) or RISE_IN (from globe/card)
    if (!s_list->squash_mode) prv_clear_fly();   // squash couldn't start (OOM)
  }
#endif
}

static void prv_window_unload(Window *window) {
#if WEATHER_PLATFORM_TOUCH_COLOR
  touch_service_unsubscribe();
#endif
  if (s_list && s_list->clock_loc_timer) {
    app_timer_cancel(s_list->clock_loc_timer);
    s_list->clock_loc_timer = NULL;
  }
  if (s_list && s_list->clock_swap_anim) {   // null-first: .stopped only destroys
    Animation *a = s_list->clock_swap_anim;
    s_list->clock_swap_anim = NULL;
    animation_unschedule(a);
  }
  s_list->clock_loc_show = false;
  // Fire pop callback before tearing down so the main screen can start its return animation.
  if (s_list && s_list->on_pop_cb) {
    void (*cb)(void *) = s_list->on_pop_cb;
    void *ctx = s_list->on_pop_ctx;
    s_list->on_pop_cb  = NULL;
    s_list->on_pop_ctx = NULL;
    cb(ctx);
  }

  if (s_list->anim) {   // capture-first: .stopped NULLs the field during unschedule
    Animation *old = s_list->anim;
    s_list->anim = NULL;
    animation_unschedule(old);
    animation_destroy(old);
  }
  if (s_list->clock_anim) {
    Animation *old = s_list->clock_anim;
    s_list->clock_anim = NULL;
    animation_unschedule(old);
    animation_destroy(old);
  }
  // The select-exit slide is module-level, not per-instance: clear the done callback FIRST
  // (unschedule fires .stopped, which would otherwise invoke it mid-teardown), then cancel.
  s_select_exit_done = NULL;
  s_select_exit_ctx  = NULL;
  if (s_select_exit_anim) {
    Animation *a = s_select_exit_anim;
    s_select_exit_anim = NULL;
    animation_unschedule(a);
    animation_destroy(a);
  }
#if WEATHER_ANIM_5DAY
  // BOTH shapes (the !PBL_ROUND exclusions dated from before round drove the report scene
  // and the squashes): cancel the scene animation and free the squash machinery at unload,
  // or the scratch (67.6KB on round) leaks and a scheduled animation outlives its window.
  if (s_report_anim) {   // capture-first: the synchronous .stopped sees the NULLed handle
    Animation *a = s_report_anim;
    s_report_anim = NULL;
    animation_unschedule(a);
    animation_destroy(a);
  }
  s_list->report_fx = 0;
  if (s_list->squash_in_anim) {
    animation_unschedule(s_list->squash_in_anim);
    animation_destroy(s_list->squash_in_anim);
    s_list->squash_in_anim = NULL;
  }
  if (s_list->squash_scratch) {
    free(s_list->squash_scratch);
    s_list->squash_scratch = NULL;
  }
#endif

#if defined(PBL_PLATFORM_GABBRO)
  if (s_list->icon_sequence) {
    gdraw_command_sequence_destroy(s_list->icon_sequence);
    s_list->icon_sequence = NULL;
  }
#endif
#if WEATHER_ANIM_5DAY
  if (s_list->sun_timer) {
    app_timer_cancel(s_list->sun_timer);
    s_list->sun_timer = NULL;
  }
#if PBL_ROUND
  if (s_list->icon_drv) {
    Animation *a = s_list->icon_drv;
    s_list->icon_drv = NULL;
    animation_unschedule(a);
    animation_destroy(a);
  }
#endif
  for (int i = 0; i < MAX_ROWS; i++) {
    if (s_list->pdc_icons[i]) {
      gdraw_command_image_destroy(s_list->pdc_icons[i]);
      s_list->pdc_icons[i] = NULL;
    }
  }
  if (s_list->today_pdc) {
    gdraw_command_image_destroy(s_list->today_pdc);
    s_list->today_pdc = NULL;
  }
  if (s_list->fly_pdc) {
    gdraw_command_image_destroy(s_list->fly_pdc);
    s_list->fly_pdc = NULL;
  }
  if (s_list->fly_lookup) {
    applib_free(s_list->fly_lookup);
    s_list->fly_lookup = NULL;
  }
  prv_free_jelly_lookups();
#endif

  for (int i = 0; i < MAX_ROWS; i++) {
    if (s_list->icons[i]) {
      gbitmap_destroy(s_list->icons[i]);
      s_list->icons[i] = NULL;
    }
  }
  layer_destroy(s_list->canvas);
  s_list->canvas = NULL;

  window_destroy(window);
  free(s_list);
  s_list = NULL;
}

// ---- Public API ----

static void prv_forecast_list_push(const WeatherLocationForecast *days, size_t num_days,
                                   int start_day_index, bool animated,
                                   void (*on_pop)(void *ctx), void *on_pop_ctx,
                                   void (*on_city_select)(void *ctx),
                                   void *on_city_select_ctx) {
  if (s_list) return;  // already showing

  s_list = calloc(1, sizeof(ForecastListData));
  if (!s_list) return;

  size_t n = num_days < MAX_ROWS ? num_days : MAX_ROWS;
  s_list->num_days = n;
  for (size_t i = 0; i < n; i++) {
    s_list->days[i] = days[i];
  }
  s_list->on_pop_cb          = on_pop;
  s_list->on_pop_ctx         = on_pop_ctx;
  s_list->on_city_select_cb  = on_city_select;
  s_list->on_city_select_ctx = on_city_select_ctx;
  {
    int sdi = start_day_index < (int)n ? start_day_index : (int)n - 1;
    s_list->start_day_index = sdi < 0 ? 0 : sdi;  // n == 0 (or a negative caller index)
  }

  s_list->window = window_create();
  // ROUND (gabbro smoothness step 3): transparent — window.c's root-layer proc fills the
  // whole window with the bg colour EVERY frame unless it is transparent (~53KB of writes
  // on the 260px glass), and every branch of prv_canvas_draw already paints every pixel
  // (scene white-fill first / report white card / squash resample full-coverage), so the
  // window fill was pure duplicate work. The !s_list early-return keeps a white guard.
  window_set_background_color(s_list->window, PBL_IF_ROUND_ELSE(GColorClear, GColorWhite));
  window_set_window_handlers(s_list->window, (WindowHandlers){
    .load   = prv_window_load,
    .appear = prv_window_appear,
    .unload = prv_window_unload,
  });
  window_set_click_config_provider(s_list->window, prv_click_provider);
  window_stack_push(s_list->window, animated);
}

void forecast_list_push(const WeatherLocationForecast *days, size_t num_days,
                        int start_day_index, bool animated,
                        void (*on_pop)(void *ctx), void *on_pop_ctx,
                        void (*on_city_select)(void *ctx), void *on_city_select_ctx) {
  prv_forecast_list_push(days, num_days, start_day_index, animated,
                         on_pop, on_pop_ctx, on_city_select, on_city_select_ctx);
}

// The DOWN-again clock burst calls this on completion to push the clock face. Set by the host so
// the list (which has no hourly data) can delegate the actual clock push back to weather.c.
void forecast_list_set_on_clock_request(void (*cb)(void *ctx), void *ctx) {
  if (!s_list) return;
  s_list->on_clock_request_cb  = cb;
  s_list->on_clock_request_ctx = ctx;
}

void forecast_list_set_on_up_request(void (*cb)(void *ctx), void *ctx) {
  if (!s_list) return;
  s_list->on_up_request_cb  = cb;
  s_list->on_up_request_ctx = ctx;
}

void forecast_list_set_on_select_request(void (*cb)(void *ctx), void *ctx) {
  if (!s_list) return;
  s_list->on_select_request_cb  = cb;
  s_list->on_select_request_ctx = ctx;
}

void forecast_list_arm_squash_in(void) {
  // Latched here (before the clock dismisses) and consumed by the forecast's appear handler,
  // so the jelly drop-in plays on the very first revealed frame. Rect-only; no-op elsewhere.
#if WEATHER_ANIM_5DAY
  s_pending_squash_mode = SQUASH_DROP_IN;
#endif
}

void forecast_list_arm_return_fly(void) {
  // Card -> forecast: rise-in PLUS the card icon flying back down into the header.
#if WEATHER_ANIM_5DAY
  s_pending_squash_mode = SQUASH_RISE_IN;
  s_pending_return_fly = true;
#endif
}

void forecast_list_update_data(const WeatherLocationForecast *days, size_t num_days) {
  if (!s_list) return;

  size_t n = num_days < MAX_ROWS ? num_days : MAX_ROWS;
  s_list->num_days = n;
  for (size_t i = 0; i < n; i++) {
    s_list->days[i] = days[i];
  }

  if (s_list->canvas) {
#if WEATHER_ANIM_5DAY
    // BOTH SHAPES: a phone sync landing mid-report-scene starves the animation service —
    // prv_load_icons re-reads every PDC from flash on the app task, and the 240ms stage-4
    // bow only gets a handful of frames to begin with; one such stall collapses it to its
    // final (blank) frame (the stage-4 bow was intermittently invisible on round hardware —
    // the same scene and the same window exist on rect). The day data is copied above
    // (cheap); the heavy visual refresh waits for the scene end
    // (prv_apply_deferred_refresh runs from every stage-4/cancel exit).
    if (s_list->report_fx != 0 || s_report_anim) {
      s_list->refresh_deferred = true;
      return;
    }
#endif
    prv_load_icons();
    // The icons were just destroyed + re-created (possibly with different point counts): any
    // jelly lookups cached mid-transition were built from the OLD geometry and would be
    // indexed out of bounds by the next scale_segmented frame — drop them so they rebuild.
    prv_free_jelly_lookups();
    prv_format_conditions();
    layer_mark_dirty(s_list->canvas);
  }
}

void forecast_list_set_glance(const char *sunset, const char *temp, int uv, int precip,
                              int wind) {
  if (!s_list) return;
  snprintf(s_list->fly_sunset, sizeof(s_list->fly_sunset), "%s", sunset ? sunset : "");
  snprintf(s_list->fly_temp,   sizeof(s_list->fly_temp),   "%s", temp   ? temp   : "");
  s_list->fly_uv     = uv;
  s_list->fly_precip = precip;
  s_list->fly_wind   = wind;
}
