/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "expanded_view.h"
#include "weather_types.h"
#include "weather_math.h"
#include "weather_app_layout.h"   // the shared UV bar (round)
#include "applib/ui/content_indicator.h"   // Health's nav arrow
#include "pebble_compat.h"
#include "applib/app_timer.h"
#include "applib/graphics/gdraw_command_image.h"
#include "applib/graphics/gdraw_command_transforms.h"
#include "applib/ui/animation.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/property_animation.h"
#include "util/time/time.h"
#include <stdint.h>
#include <string.h>

// Standalone recreation of the Timeline weather pin card (services/timeline/
// weather_layout.c) as it renders on emery (PreferredContentSizeLarge):
//   status time | 80x80 icon | "Sunset H:MM" (GOTHIC_24_BOLD title) |
//   "high/low°" (LECO_36 subtitle) | location (GOTHIC_24_BOLD) | (page-break)
// The page-break/scroll-down detail page carries the UV + precipitation gauges.
//
// Buttons: on the glance, DOWN scrolls to the detail page, UP -> globe. On the
// detail page, UP scrolls back to the glance, DOWN -> forecast. BACK -> forecast.
// (This mirrors the Timeline card's own button-scroll: scroll within, navigate at
// the ends.) Touch swipe scrolls the two pages.
typedef struct {
  Window            *window;
  Layer             *canvas;
  GDrawCommandImage *icon;         // 80x80 PDC weather icon (owned)
  int      high;
  int      low;
  int      uv;                     // 0-11, -1 unknown
  int      precip;                 // rain chance %, -1 unknown
  int      wind;                   // mph, -1 unknown
  int16_t  lat_e2;                 // degrees*100, INT16_MIN unknown (for sunset)
  int16_t  lon_e2;
  int16_t  utc_off_min;            // location tz, minutes east of UTC; INT16_MIN = watch tz
  char     time_str[10];           // status bar time
  char     sunset_str[20];         // "Sunset 9:30 PM"
  char     temp_str[16];           // "23/14°"
  // Content entrance: the card content slides in from the left + bounces (Timeline card entrance).
  AnimationProgress text_p;
  Animation        *text_anim;
  bool              text_active;
  ExpandedViewEntrance entrance;            // how the card animates in on first appear (latched off after)
  // Status bar: show "Last updated ..." for 2s, then swap it out to the right while the time swoops
  // in from the left.
  char              updated_str[24];        // "Last updated 8:23 PM"
  bool              show_updated;           // true until the swap completes
  bool              swap_active;            // the last-updated -> time slide is playing
  AnimationProgress swap_p;
  Animation        *swap_anim;
  AppTimer         *updated_timer;          // fires 3s after appear to start the swap
  // Whole-card horizontal slide (moook bounce), shared by the SELECT slide-out-left (then on_select
  // fires -> globe slides in from right) and the globe-BACK slide-in-from-left entrance.
  Animation        *slide_anim;
#ifdef CONFIG_TOUCH
  int16_t  touch_start_x, touch_start_y;   // Touchdown origin for swipe detection
  bool     touch_active;
#endif
  void   (*on_down)(void *ctx);   void *on_down_ctx;   // DOWN/BACK -> forecast
  void   (*on_select)(void *ctx); void *on_select_ctx; // SELECT -> globe (slide-in from right)
} ExpandedViewData;

static ExpandedViewData *s_ev;

// System-app statics persist across launches (the fw image is not reloaded), and a
// crashed run never reaches unload — clear them before the next run reads them.
void expanded_view_reset(void) {
  s_ev = NULL;
}

// ---- Sunset computation (fixed-point) -------------------------------------
// There is no sunset field in the synced weather data and no firmware sunset API,
// so compute it from lat/lon + date with the standard sunrise/sunset algorithm,
// using Pebble's integer trig (sin_lookup/cos_lookup; acos via binary search).
// Returns local minutes-from-midnight [0,1440), or -1 if unknown/degenerate.

static int32_t prv_norm_angle(int32_t a) {
  a %= TRIG_MAX_ANGLE;
  if (a < 0) a += TRIG_MAX_ANGLE;
  return a;
}
static int32_t prv_angle_from_cdeg(int32_t cdeg) {   // degrees*100 -> trig angle
  return prv_norm_angle((int32_t)(((int64_t)cdeg * TRIG_MAX_ANGLE) / 36000));
}

static int prv_sunset_minutes(int16_t lat_e2, int16_t lon_e2, int16_t utc_off_min) {
  if (lat_e2 == INT16_MIN || lon_e2 == INT16_MIN) {
    return -1;
  }
  time_t now = rtc_get_time();
  struct tm lt = *localtime(&now);
  const int N = lt.tm_yday + 1;                    // day of year 1..366
  // The LOCATION's UTC offset when the phone synced one (v4.1) — so a saved city's
  // sunset reads in that city's local clock; otherwise the watch's own offset.
  const int tz_min = (utc_off_min != INT16_MIN) ? utc_off_min
                                                : (time_get_gmtoffset() / 60);

  const int b_cdeg = (36000 * (N - 81)) / 365;     // B = (360/365)*(N-81), deg*100
  const int32_t sinB  = sin_lookup(prv_angle_from_cdeg(b_cdeg));
  const int32_t cosB  = cos_lookup(prv_angle_from_cdeg(b_cdeg));
  const int32_t sin2B = sin_lookup(prv_angle_from_cdeg(2 * b_cdeg));

  const int decl_cdeg = (int)(((int64_t)2345 * sinB) / TRIG_MAX_RATIO);  // 23.45*sin(B)
  const int32_t sinDec = sin_lookup(prv_angle_from_cdeg(decl_cdeg));
  const int32_t cosDec = cos_lookup(prv_angle_from_cdeg(decl_cdeg));
  const int32_t sinLat = sin_lookup(prv_angle_from_cdeg(lat_e2));
  const int32_t cosLat = cos_lookup(prv_angle_from_cdeg(lat_e2));

  const int64_t den = (int64_t)cosLat * cosDec;
  if (den == 0) return -1;
  int64_t cosH = -((int64_t)sinLat * sinDec) * 1000000 / den;   // ratio *1e6
  if (cosH >  1000000) cosH =  1000000;
  if (cosH < -1000000) cosH = -1000000;

  int32_t alo = 0, ahi = TRIG_MAX_ANGLE / 2;       // omega = acos(cosH), binary search
  for (int i = 0; i < 22; i++) {
    int32_t amid = (alo + ahi) / 2;
    int64_t c = (int64_t)cos_lookup(amid) * 1000000 / TRIG_MAX_RATIO;
    if (c > cosH) alo = amid; else ahi = amid;
  }
  const int omega_cdeg = (int)(((int64_t)((alo + ahi) / 2) * 36000) / TRIG_MAX_ANGLE);

  const int eot_m1000 = (int)(((int64_t)9870 * sin2B - (int64_t)7530 * cosB
                               - (int64_t)1500 * sinB) / TRIG_MAX_RATIO);
  const int tc_m1000 = 40 * (int)lon_e2 - tz_min * 1000 + eot_m1000;
  int sunset_min = (720000 + 40 * omega_cdeg - tc_m1000) / 1000;
  sunset_min %= 1440;
  if (sunset_min < 0) sunset_min += 1440;
  return sunset_min;
}

static void prv_fmt_hhmm(char *out, size_t n, int h24, int mm, const char *prefix) {
  if (clock_is_24h_style()) {
    snprintf(out, n, "%s%d:%02d", prefix, h24, mm);
  } else {
    int h12 = h24 % 12; if (h12 == 0) h12 = 12;
    snprintf(out, n, "%s%d:%02d %s", prefix, h12, mm, h24 < 12 ? "AM" : "PM");
  }
}

static void prv_build_sunset(char *out, size_t n, int16_t lat_e2, int16_t lon_e2,
                             int16_t utc_off_min) {
  int m = prv_sunset_minutes(lat_e2, lon_e2, utc_off_min);
  if (m < 0) { snprintf(out, n, "Sunset --:--"); return; }
  prv_fmt_hhmm(out, n, (m / 60) % 24, m % 60, "Sunset ");
}

static void prv_build_time(char *out, size_t n) {
  time_t now = rtc_get_time();
  struct tm *lt = localtime(&now);
  if (!lt) { snprintf(out, n, "--:--"); return; }
  int h = lt->tm_hour, mm = lt->tm_min;
  prv_fmt_hhmm(out, n, h, mm, "");
}

// Round's status bar is a chord, not a full-width strip, so the rect wording overruns it
// and clips. "Updated, 8:23 PM" says the same thing inside the space there actually is.
#define EV_STATUS_Y    0
#define EV_STATUS_FONT FONT_KEY_GOTHIC_18_BOLD

#define EV_UPDATED_PREFIX  "Last updated "
#define EV_UPDATED_UNKNOWN "Last updated --:--"

void expanded_view_format_updated(const WeatherLocationForecast *f, char *out, size_t n) {
  if (!f || f->time_updated_utc <= 0) { snprintf(out, n, EV_UPDATED_UNKNOWN); return; }
  time_t t = f->time_updated_utc;
  struct tm *lt = localtime(&t);
  if (!lt) { snprintf(out, n, EV_UPDATED_UNKNOWN); return; }
  prv_fmt_hhmm(out, n, lt->tm_hour, lt->tm_min, EV_UPDATED_PREFIX);
}

void expanded_view_format_glance(const WeatherLocationForecast *f, int16_t lat_e2, int16_t lon_e2,
                                 int16_t utc_off_min,
                                 char *sunset, size_t sunset_sz, char *temp, size_t temp_sz,
                                 char *loc, size_t loc_sz) {
  prv_build_sunset(sunset, sunset_sz, lat_e2, lon_e2, utc_off_min);
  if (!f) {
    snprintf(temp, temp_sz, "--");
    snprintf(loc, loc_sz, "--");
    return;
  }
  snprintf(loc, loc_sz, "%s", f->location_name ? f->location_name : "--");
  const bool hi = f->today_high != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  const bool lo = f->today_low  != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  if (hi && lo) snprintf(temp, temp_sz, "%d/%d\xC2\xB0", f->today_high, f->today_low);
  else if (hi)  snprintf(temp, temp_sz, "%d\xC2\xB0", f->today_high);
  else          snprintf(temp, temp_sz, "--");
}

static void prv_set_from_forecast(const WeatherLocationForecast *f,
                                  int16_t lat_e2, int16_t lon_e2, int16_t utc_off_min) {
  if (!s_ev) return;
  s_ev->lat_e2 = lat_e2;
  s_ev->lon_e2 = lon_e2;
  s_ev->utc_off_min = utc_off_min;
  prv_build_time(s_ev->time_str, sizeof(s_ev->time_str));
  expanded_view_format_updated(f, s_ev->updated_str, sizeof(s_ev->updated_str));
  char loc_scratch[64];   // the card doesn't display it; the shared formatter needs a sink
  expanded_view_format_glance(f, lat_e2, lon_e2, utc_off_min,
                              s_ev->sunset_str, sizeof(s_ev->sunset_str),
                              s_ev->temp_str, sizeof(s_ev->temp_str),
                              loc_scratch, sizeof(loc_scratch));

  if (s_ev->icon) { gdraw_command_image_destroy(s_ev->icon); s_ev->icon = NULL; }
  if (!f) {
    s_ev->high = s_ev->low = WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
    s_ev->uv = s_ev->precip = s_ev->wind = -1;
    return;
  }
  s_ev->high   = f->today_high;
  s_ev->low    = f->today_low;
  // BOTH SHAPES: the sunset card's
  // UV readout shows the CURRENT hour's UV (today_uv_now) — the live reading once the phone
  // sends the minor-4 hourly block, falling back to the day's figure until then.
  s_ev->uv     = f->today_uv_now;
  s_ev->precip = f->today_precip_mm;
  s_ev->wind   = f->today_wind_mph;
  // BOTH SHAPES: the card's icon is typed by the CURRENT HOUR
  // (current_type_now) — the same field the mainscreen header and the hero fly use — so the
  // flown icon and the card icon can never disagree when the phone's synced 'current' and
  // its hourly slot diverge (seen on the round watch as a rain->overcast repaint at the
  // landing instant; rect's header reads the same field now, so it needs the same seam fix).
  GDrawCommandImage *raw = gdraw_command_image_create_with_resource(
      weather_type_icon_large_resource(f->current_type_now));
  if (raw) {
    s_ev->icon = gdraw_command_image_clone(raw);   // writable copy so we can scale it
    gdraw_command_image_destroy(raw);
    if (s_ev->icon) gdraw_command_image_scale(s_ev->icon, GSize(EV_ICON_SIZE, EV_ICON_SIZE));
  }
}

// ---- UV / precipitation gauge (ported from detail_face.c) ------------------
// ---- ROUND (gabbro) sunset-card grid ------------------------------------------------------
// The weather icon + "Sunset H:MM" + "high/low°" are ONE GROUP, centred on the screen's
// vertical midline. Measured ink after the shift: icon 73..125, sunset 140..153, temp
// 163..187 -> group 73..187, centre exactly 130.
//
// That needed 38px of downward room, which only exists once the dials shrink AND pull
// inward+down. At the old r=25 the arithmetic is impossible: a dial's footprint is 57px
// (16px label + ring, whose lowest ink is cy+16 — the arc is open at the bottom), so
// clearing a group that ends at 187 forces cy>=233, and at that depth the glass only admits
// dial centres within ±36px of the centre column, where the two 50px rings overlap.
// At r=20 the footprint drops to ~50px and cy=228 with centres at x=87/173 fits with a 4px
// margin to the glass and a clear gap between the two labels.
// Everything here is round-only — emery's grid is frozen.
// the dials were replaced by the report's UV bar. The group KEEPS the exact
// centring it had with the dials (ink 73..187, centre 130) — the bar is made COMPACT to fit
// beneath it rather than the group moving to make room.
#define EV_ROUND_GROUP_DY 11                          // sunset+temp ink 113..160: centred in the
                                                      // icon-bottom(103) .. pill-top(170) band,
                                                      // 9 rows clear each side (measured)                          // icon/sunset/temp shift (EV_ICON_Y carries
                                                      // the icon's share, see expanded_view.h)
#define EV_UV_BAR_Y       194                         // bar top row, just under the group
#define EV_SUNSET_Y    PBL_IF_ROUND_ELSE(92 + EV_ROUND_GROUP_DY, (EV_SMALL_RECT ? 82 : 92))
#define EV_TEMP_Y      PBL_IF_ROUND_ELSE(114 + EV_ROUND_GROUP_DY, (EV_SMALL_RECT ? 102 : 114))
// Small rect: the UV/RAIN meters as one centred text row (no room for dials).
#define EV_METERS_Y    140
#define EV_GAUGE_R     PBL_IF_ROUND_ELSE(20, 25)      // ring radius
#define EV_GAUGE_CY    PBL_IF_ROUND_ELSE(228, 200)    // dial centre row
#define EV_GAUGE_INSET PBL_IF_ROUND_ELSE(22, 0)       // pull both dials toward the centre column
// Round's flanking dials. cy 174 puts the gauge's visual block (label top cy-36 .. ring's lowest
// ink ~cy+13) centred on the sunset/hi-lo text block (ink 140..187, centre 163.5) and leaves the
// ring clear of the UV bar at 194. The x values keep each 72px-wide label BOX overlapping the
// text only where the label's centred ink does not reach.
// The flanking ARCH dials : an open arch, no bottom. The outer leg IS the
// glass-concentric arc (RIM_R about the screen centre — same radius band as the compact UV
// bar's frame ends, so bar end and dial leg read as one continuous curve up the side of the
// glass). The inner leg is that arc REFLECTED about the dial's axis x=CX_L, which also gives
// both feet their natural inward hook (the reflection of the rim curving in). A semicircular
// cap centred on the axis at CAP_Y joins the legs over the top; its radius is derived
// (CX_L - outer_leg_x(CAP_Y)), so cap and legs always meet where they stand.
// Stat discs: the mainscreen 5-day rows' disc-and-icon at their own scale (their discs are
// ~34px with the TINY icon set; ours are 36px with the same 1x icons). Disc row 152 centres
// the disc on the sunset+temp block; the value ink lands ~176..186, clear of the bar at 194.
#define EV_PRECIP_PILL_Y  170  // pill rows 170..187; 5px below the temp ink, 6 above the bar
#define EV_PRECIP_PILL_H   18  // same as the mainscreen pill (R5_STAT_PILL_H)
#define EV_ROUND_RAIN_CX    44
#define EV_ROUND_WIND_CX   218

#define EV_GAUGE_DEG_TO_ANGLE(d) ((int32_t)(TRIG_MAX_ANGLE * (d) / 360))
#define EV_GAUGE_ARC_START  EV_GAUGE_DEG_TO_ANGLE(225)
#define EV_GAUGE_ARC_SPAN   EV_GAUGE_DEG_TO_ANGLE(270)
#define EV_GAUGE_RING_W     7

// WHO UV-index severity color — the SAME ramp as the condensed screen's UV
// tally squares (weather_app_layout.c), so the dial and the squares always
// tell the same story: 0-2 green, 3-5 yellow, 6-7 orange, 8+ red (incl. off-scale).
#define prv_uv_severity_color weather_uv_severity_color

#if !PBL_ROUND
// Rect only: round replaced its dials with the rain/wind glyphs below (and the shared UV bar).
static void prv_draw_gauge(GContext *ctx, int cx, int cy, int r, const char *label,
                           int value, int max_val, const char *unit, bool unknown,
                           GColor fill) {
  GRect ring = GRect(cx - r, cy - r, 2 * r, 2 * r);
  graphics_context_set_fill_color(ctx, GColorLightGray);
  graphics_fill_radial(ctx, ring, GOvalScaleModeFitCircle, EV_GAUGE_RING_W,
                       EV_GAUGE_ARC_START, EV_GAUGE_ARC_START + EV_GAUGE_ARC_SPAN);
  if (!unknown && max_val > 0) {
    int v = value < 0 ? 0 : (value > max_val ? max_val : value);
    int32_t filled = weather_scale_i32(EV_GAUGE_ARC_SPAN, v, max_val);
    if (filled > 0) {
      graphics_context_set_fill_color(ctx, PBL_IF_COLOR_ELSE(fill, GColorBlack));
      graphics_fill_radial(ctx, ring, GOvalScaleModeFitCircle, EV_GAUGE_RING_W,
                           EV_GAUGE_ARC_START, EV_GAUGE_ARC_START + filled);
    }
  }
  char vbuf[12];
  if (unknown) snprintf(vbuf, sizeof(vbuf), "--");
  else         snprintf(vbuf, sizeof(vbuf), "%d%s", value, unit);
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx, vbuf, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                     GRect(cx - r + 2, cy - 12, 2 * r - 4, 24),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  graphics_draw_text(ctx, label, fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD),
                     GRect(cx - 36, cy - r - 16, 72, 16),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
}
#endif  // !PBL_ROUND

// ---- Draw -----------------------------------------------------------------
// Single card: time (status bar) | 74x74 icon | "Sunset H:MM" | high/low° | UV + RAIN meters.

// Everything except the icon, each element shifted right by `tdx` (the slide-in offset). Shared by
// the card (static, or its own slide) and the forecast's hero icon-fly (synced to the icon), so the
// two are pixel-identical and time/sunset/temp/meters all animate in together.

// (Round's rain + wind icons and their prv_draw_metric helper were removed for now.
// The originals are in commit 4db271f3 (weather_app_layout.c prv_draw_raindrop_icon /
// prv_draw_wind_icon); the 2x solid-black redraw is in this file's history. `wind` is still
// plumbed all the way through to here and to the hero-fly's copy, so bringing them back is just
// the two draw calls plus the icon helpers.)

#if PBL_ROUND
// ---- Precipitation pill ----------------------------------------------------------------
// The scrolled mainscreen's own PRECIPITATION banner (forecast_list.c prv_draw_stat_pill),
// replicated to the pixel: Pebble Health's pill geometry (inset 18 + (W-144)/2/5, height 18,
// corner radius 3), GColorPictonBlue, black GOTHIC_18_BOLD label centred with the same -3px
// optical lift (measured from the live screen — the pill comment says 14_BOLD but round's
// day_font resolves to 18_BOLD; the label caps measure 11px). One difference: this one
// carries the value inline — "PRECIPITATION: 20%".
static void prv_draw_precip_pill(GContext *ctx, int W, int tdx, int precip) {
  char label[28];
  if (precip < 0) snprintf(label, sizeof(label), "PRECIPITATION: --");
  else            snprintf(label, sizeof(label), "PRECIPITATION: %d%%", precip);
  const int inset = 18 + (W - 144) / 2 / 5;
  const GRect pill = GRect(inset + tdx, EV_PRECIP_PILL_Y, W - 2 * inset, EV_PRECIP_PILL_H);
  graphics_context_set_fill_color(ctx, GColorPictonBlue);
  graphics_fill_round_rect(ctx, &pill, 3, GCornersAll);
  graphics_context_set_text_color(ctx, GColorBlack);
  GFont lf = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
  const int th = fonts_get_font_height(lf);
  graphics_draw_text(ctx, label, lf,
      GRect(inset + tdx, EV_PRECIP_PILL_Y + (EV_PRECIP_PILL_H - th) / 2 - 3,
            W - 2 * inset, th),
      GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
}

#endif  // PBL_ROUND

void expanded_view_draw_glance_content(GContext *ctx, int W, int tdx, const char *status,
                                       const char *sunset, const char *temp, int uv, int precip,
                                       int wind) {
  graphics_context_set_text_color(ctx, GColorBlack);
  // Status bar (top) — the time or "Last updated ..."; NULL while the card draws the swap itself.
  if (status) {
    graphics_draw_text(ctx, status, fonts_get_system_font(EV_STATUS_FONT),
                       GRect(tdx, EV_STATUS_Y, W, 20),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  }
  // Title = "Sunset H:MM" (Header font on emery = GOTHIC_24_BOLD).
  graphics_draw_text(ctx, sunset, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD),
                     GRect(4 + tdx, EV_SUNSET_Y, W - 8, 30),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  // Subtitle = "high/low°" (LECO_36, big) — OPTICALLY centred: the trailing degree sign is
  // excluded from the width measure (it reads as an appendage, so true centring of the full
  // string looks left-shifted), then the full string draws left-aligned from that origin with
  // the ° hanging off the right. Same trick as the "feels" cell and the clock centre capture.
  {
    // 144px wide: LECO_36 leaves "24/14\xc2\xb0" ~5px of margin — it can't read
    // as centred. One step down keeps the hierarchy and restores the margins.
    GFont tf = fonts_get_system_font(
        EV_SMALL_RECT ? FONT_KEY_LECO_32_BOLD_NUMBERS : FONT_KEY_LECO_36_BOLD_NUMBERS);
    char body[16];
    strncpy(body, temp, sizeof(body) - 1);
    body[sizeof(body) - 1] = '\0';
    size_t blen = strlen(body);
    if (blen >= 2 && (uint8_t)body[blen - 2] == 0xC2 && (uint8_t)body[blen - 1] == 0xB0) {
      body[blen - 2] = '\0';   // strip the 2-byte UTF-8 degree sign
    }
    // Small rect: LECO's degree glyph is a full-size ring, not a superscript —
    // excluding it from the measure visibly right-shifts the row, so the whole
    // string centres there.
    GSize bsz = graphics_text_layout_get_content_size(
        EV_SMALL_RECT ? temp : body, tf,
        GRect(0, 0, W, 46), GTextOverflowModeFill, GTextAlignmentLeft);
    // BOTH shapes: optical centring, degree sign excluded from the measure (it reads as an
    // appendage; true centring of the full string looks left-shifted). Round briefly used
    // true centring while dials flanked this row — the dials are gone, so the override is too.
    graphics_draw_text(ctx, temp, tf,
                       GRect(tdx + (W - bsz.w) / 2, EV_TEMP_Y, W, 46),
                       GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  }
  // UV + precipitation meters at the bottom. The UV dial takes the WHO
  // severity color for its value; rain stays water-blue.
#if PBL_ROUND
  // ROUND: the two dials are replaced by the weather report's UV bar — the SAME component,
  // drawn by weather_app_layout_draw_uv_bar (see that header). The card's canvas is
  // full-screen, so the glass centre is simply the screen centre; `tdx` carries the
  // glance-text slide so the bar rides in with the rest of the body.
  // NOTE: this drops precipitation from the card entirely. The report page keeps it in its
  // prose sentence ("... Precipitation 20%."), but this screen has no such line.
  // Flanking dials: RAIN to the LEFT of the sunset/hi-lo text, WIND to the RIGHT. They ride
  // `tdx` with the rest of the body so they slide in with it. Sized and placed so the LABEL ink
  // clears the text block horizontally (the labels share rows with the "Sunset H:MM" line) and
  // the ring's lowest ink stays clear of the UV bar below.
  // Wind on the LEFT, rain on the RIGHT (matching the design). Fill level = how windy / how
  // rainy; the icon rides on top so it stays legible over the fill.
  {
    (void)wind;   // wind was dropped from this screen by design;
                  // the plumbing stays for whatever wants it next.
    prv_draw_precip_pill(ctx, W, tdx, precip);
    // tall from origin.y+1, so -12 left it measurably 1px low (audit finding).

  }
  if (uv >= 0) {
    char uvbuf[12];
    snprintf(uvbuf, sizeof(uvbuf), "%d", uv);
    weather_app_layout_draw_uv_bar(ctx, GPoint(W / 2 + tdx, PBL_DISPLAY_HEIGHT / 2),
                                   EV_UV_BAR_Y, uv, uvbuf, WeatherUvBarCompact,
                                   "CURRENT UV");
  }
#else
  if (EV_SMALL_RECT) {
    // 144x168: the r=25 dials cannot fit under the temp — one text row
    // carries both readouts instead.
    char meters[40];
    char uv_part[16], rain_part[20];
    if (uv >= 0) snprintf(uv_part, sizeof(uv_part), "UV %d", uv);
    else         snprintf(uv_part, sizeof(uv_part), "UV --");
    if (precip >= 0) snprintf(rain_part, sizeof(rain_part), "RAIN %d%%", precip);
    else             snprintf(rain_part, sizeof(rain_part), "RAIN --");
    snprintf(meters, sizeof(meters), "%s   %s", uv_part, rain_part);
    graphics_context_set_text_color(ctx, GColorBlack);
    graphics_draw_text(ctx, meters, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                       GRect(tdx, EV_METERS_Y, W, 22),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  } else {
    prv_draw_gauge(ctx, W / 4 + EV_GAUGE_INSET + tdx,     EV_GAUGE_CY, EV_GAUGE_R,
                   "UV",   uv,     11,  "",  uv < 0, prv_uv_severity_color(uv));
    prv_draw_gauge(ctx, 3 * W / 4 - EV_GAUGE_INSET + tdx, EV_GAUGE_CY, EV_GAUGE_R,
                   "RAIN", precip, 100, "%", precip < 0, GColorVividCerulean);
  }
#endif
}

static void prv_canvas_draw(Layer *layer, GContext *ctx) {
  if (!s_ev) return;
  GRect b = layer_get_bounds(layer);
  const int W = b.size.w;

  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, b, 0, GCornerNone);

  // Icon stays fixed (the hero fly placed it); everything else slides in from the left + bounces.
  // Position (EV_ICON_Y) + size (EV_ICON_SIZE) match forecast_list.c's fly_dest for a seamless hand-off.
  if (s_ev->icon) {
    GSize sz = gdraw_command_image_get_bounds_size(s_ev->icon);
    gdraw_command_image_draw(ctx, s_ev->icon, GPoint((W - sz.w) / 2, EV_ICON_Y));
  }
  const int tdx = s_ev->text_active
      ? (int)interpolate_moook_soft(s_ev->text_p, -W, 0, 3) : 0;

  if (s_ev->swap_active) {
    // Body without the status bar; the last-updated -> time swap is drawn on top: both slide RIGHT,
    // "Last updated" exiting off the right while the time swoops in from the left, trailing it.
    expanded_view_draw_glance_content(ctx, W, tdx, NULL, s_ev->sunset_str, s_ev->temp_str,
                                      s_ev->uv, s_ev->precip, s_ev->wind);
    char time_str[10];
    prv_build_time(time_str, sizeof(time_str));
    const int sx = (int)interpolate_moook_soft(s_ev->swap_p, 0, W, 3);
    GFont f = fonts_get_system_font(EV_STATUS_FONT);
    graphics_context_set_text_color(ctx, GColorBlack);
    graphics_draw_text(ctx, s_ev->updated_str, f, GRect(sx, EV_STATUS_Y, W, 20),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
    graphics_draw_text(ctx, time_str, f, GRect(sx - W, EV_STATUS_Y, W, 20),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  } else {
    char time_str[10];
    const char *status = s_ev->updated_str;
    if (!s_ev->show_updated) { prv_build_time(time_str, sizeof(time_str)); status = time_str; }
    expanded_view_draw_glance_content(ctx, W, tdx, status, s_ev->sunset_str, s_ev->temp_str,
                                      s_ev->uv, s_ev->precip, s_ev->wind);
  }

  // SELECT marker: black half-circle nub on the centre-right edge — the same radius-13 oval the
  // mainscreen draws (forecast_list.c), pushed off-screen so only ~5px protrudes. BOTH shapes:
  // SELECT opens the globe from this card on round too, so the affordance belongs there as well.
  // The nub sits on the glass's widest rows (117..142, where the chord still reaches x259), so
  // the whole protruding sliver stays on screen — nothing to re-inset for round.
  // Small rect: skipped — beside the full-height content column the clipped
  // half-circle reads as a rendering glitch, not an affordance.
  if (!EV_SMALL_RECT) {
    const int select_protrusion = 5;
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_oval(ctx, GRect(W - select_protrusion, (b.size.h - 26) / 2, 26, 26),
                       GOvalScaleModeFitCircle);
  }

#if PBL_ROUND
  // DOWN nav arrow — the system Health app's card-view indicator, same applib component and
  // geometry as the forecast mainscreen's (health/card_view.c: 18px full-width band on round,
  // GAlignCenter). DOWN on this card returns to the forecast. Deliberately NOT offset by tdx:
  // the arrow is chrome, so it holds still while the glance text slides in behind it. No UP
  // arrow here — not asked for.
  {
    const int arrow_band = 18;
    const GRect down_frame = GRect(0, b.size.h - arrow_band, W, arrow_band);
    content_indicator_draw_arrow(ctx, &down_frame, ContentIndicatorDirectionDown,
                                 GColorBlack, GColorWhite, GAlignCenter);
  }
#endif
}

// ---- SELECT -> globe: slide the whole card out to the left ----------------
// The card's canvas layer slides one screen-width left with the same moook bounce the globe uses
// for its entrance; the vacated area is the window's white background (no base flash). On completion
// on_select fires and weather.c pushes the globe sliding in from the right.

// Slide duration + interpolation are shared with globe_view via weather_math.h
// (WEATHER_HSLIDE_MS / weather_interpolate_moook_soft1) so the pair can't drift.

// BOTH shapes.
static void prv_slide_out_stopped(Animation *anim, bool finished, void *context) {
  if (!s_ev) return;
  s_ev->slide_anim = NULL;   // property animation auto-destroys after a normal stop
  if (finished && s_ev->on_select) {
    s_ev->on_select(s_ev->on_select_ctx);   // push globe (slide-in-right) + dismiss this card
  }
}

static void prv_start_slide_out_left(void) {
  if (!s_ev || !s_ev->canvas || s_ev->slide_anim) return;
  GRect from = layer_get_frame(s_ev->canvas);
  GRect to = from;
  to.origin.x -= from.size.w;                     // one screen-width to the left, off screen
  PropertyAnimation *pa = property_animation_create_layer_frame(s_ev->canvas, &from, &to);
  if (!pa) { if (s_ev->on_select) s_ev->on_select(s_ev->on_select_ctx); return; }
  Animation *a = (Animation *)pa;
  animation_set_duration(a, WEATHER_HSLIDE_MS);
  animation_set_custom_interpolation(a, weather_interpolate_moook_soft1);
  animation_set_handlers(a, (AnimationHandlers){ .stopped = prv_slide_out_stopped }, NULL);
  s_ev->slide_anim = a;
  animation_schedule(a);
}

// Globe-BACK entrance: the whole card starts one screen-width to the left (set in prv_window_load)
// and slides home with the same moook bounce — the mirror of the SELECT slide-out-left.
static void prv_slide_in_stopped(Animation *anim, bool finished, void *context) {
  if (s_ev) s_ev->slide_anim = NULL;   // property animation auto-destroys after a normal stop
}

static void prv_start_slide_in_left(void) {
  if (!s_ev || !s_ev->canvas || s_ev->slide_anim) return;
  GRect from = layer_get_frame(s_ev->canvas);     // off-left (-W), set in prv_window_load
  GRect to = from;
  to.origin.x += from.size.w;                     // bring it home to x = 0
  PropertyAnimation *pa = property_animation_create_layer_frame(s_ev->canvas, &from, &to);
  if (!pa) { layer_set_frame(s_ev->canvas, to); return; }
  Animation *a = (Animation *)pa;
  animation_set_duration(a, WEATHER_HSLIDE_MS);
  animation_set_custom_interpolation(a, weather_interpolate_moook_soft1);
  animation_set_handlers(a, (AnimationHandlers){ .stopped = prv_slide_in_stopped }, NULL);
  s_ev->slide_anim = a;
  animation_schedule(a);
}


// ---- Navigation -----------------------------------------------------------

static void prv_click_down(ClickRecognizerRef r, void *ctx) {
  if (s_ev && s_ev->on_down) s_ev->on_down(s_ev->on_down_ctx); // -> forecast
}
static void prv_click_select(ClickRecognizerRef r, void *ctx) {
  if (s_ev && s_ev->on_select && !s_ev->slide_anim) prv_start_slide_out_left();  // -> globe
}

// ---- Touch input (touch colour platforms) ----
#ifdef CONFIG_TOUCH
#define SWIPE_THRESHOLD 20   // px; same tap/swipe split as the other screens

static void prv_touch_handler(const TouchEvent *event, void *context) {
  (void)context;
  if (!s_ev) return;
  if (event->type == TouchEvent_Touchdown) {
    s_ev->touch_start_x = event->x;
    s_ev->touch_start_y = event->y;
    s_ev->touch_active  = true;
  } else if (event->type == TouchEvent_Liftoff && s_ev->touch_active) {
    s_ev->touch_active = false;
    if (s_ev->slide_anim) return;    // card is mid-slide — same guard as SELECT
    int16_t dx = event->x - s_ev->touch_start_x;
    int16_t dy = event->y - s_ev->touch_start_y;
    int16_t adx = dx < 0 ? -dx : dx;
    int16_t ady = dy < 0 ? -dy : dy;
    if (adx <= SWIPE_THRESHOLD && ady <= SWIPE_THRESHOLD) return;  // tap — no action
    if (adx > ady) {
      if (dx < 0 && s_ev->on_select) {
        // Swipe left -> globe (the card slides out left, mirroring SELECT).
        prv_start_slide_out_left();
      }
    } else if (dy < 0 && s_ev->on_down) {
      s_ev->on_down(s_ev->on_down_ctx);     // swipe up -> back DOWN to the forecast (mirrors DOWN)
    }
  }
}
#endif

static void prv_click_provider(void *ctx) {
  window_single_click_subscribe(BUTTON_ID_DOWN,   prv_click_down);    // -> forecast
  window_single_click_subscribe(BUTTON_ID_BACK,   prv_click_down);    // -> forecast
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_click_select);  // -> globe (slide-in-right)
}

// ---- Content entrance: slide in from the left + bounce --------------------

static void prv_text_in_update(Animation *anim, AnimationProgress progress) {
  if (!s_ev) return;
  s_ev->text_p = progress;
  if (s_ev->canvas) layer_mark_dirty(s_ev->canvas);
}
static void prv_text_in_stopped(Animation *anim, bool finished, void *context) {
  if (s_ev) {
    s_ev->text_active = false;
    s_ev->text_anim = NULL;
    if (s_ev->canvas) layer_mark_dirty(s_ev->canvas);
  }
  animation_destroy(anim);
}
static const AnimationImplementation s_text_in_impl = { .update = prv_text_in_update };

static void prv_start_text_in(void) {
  if (!s_ev || s_ev->text_anim) return;
  s_ev->text_active = true;
  s_ev->text_p = 0;
  s_ev->text_anim = animation_create();
  animation_set_implementation(s_ev->text_anim, &s_text_in_impl);
  animation_set_duration(s_ev->text_anim, interpolate_moook_soft_duration(3));
  animation_set_curve(s_ev->text_anim, AnimationCurveLinear);   // interpolate_moook_soft shapes it
  animation_set_handlers(s_ev->text_anim,
                         (AnimationHandlers){ .stopped = prv_text_in_stopped }, NULL);
  animation_schedule(s_ev->text_anim);
}

// ---- Status bar: "Last updated ..." -> time swap (2s after appear) --------

static void prv_swap_update(Animation *anim, AnimationProgress progress) {
  if (!s_ev) return;
  s_ev->swap_p = progress;
  if (s_ev->canvas) layer_mark_dirty(s_ev->canvas);
}
static void prv_swap_stopped(Animation *anim, bool finished, void *context) {
  if (s_ev) {
    s_ev->swap_active = false;
    s_ev->show_updated = false;    // the status bar now rests on the time
    s_ev->swap_anim = NULL;
    if (s_ev->canvas) layer_mark_dirty(s_ev->canvas);
  }
  animation_destroy(anim);
}
static const AnimationImplementation s_swap_impl = { .update = prv_swap_update };

static void prv_start_swap(void) {
  if (!s_ev || s_ev->swap_anim || !s_ev->show_updated) return;
  s_ev->swap_active = true;
  s_ev->swap_p = 0;
  s_ev->swap_anim = animation_create();
  animation_set_implementation(s_ev->swap_anim, &s_swap_impl);
  animation_set_duration(s_ev->swap_anim, interpolate_moook_soft_duration(3));
  animation_set_curve(s_ev->swap_anim, AnimationCurveLinear);   // interpolate_moook_soft shapes it
  animation_set_handlers(s_ev->swap_anim,
                         (AnimationHandlers){ .stopped = prv_swap_stopped }, NULL);
  animation_schedule(s_ev->swap_anim);
}

static void prv_updated_timer_cb(void *ctx) {
  if (!s_ev) return;
  s_ev->updated_timer = NULL;
  prv_start_swap();
}

// ---- Window lifecycle -----------------------------------------------------

static void prv_window_appear(Window *window) {
  if (!s_ev) return;
#ifdef CONFIG_TOUCH
  touch_service_subscribe(prv_touch_handler, s_ev);
#endif
  // Hold "Last updated ..." for 2s, then swap it out for the time (once).
  if (!s_ev->updated_timer && s_ev->show_updated && !s_ev->swap_active) {
    s_ev->updated_timer = app_timer_register(2000, prv_updated_timer_cb, NULL);
  }
  // Play the entrance animation (latched off after the first appearance so button-nav re-appears
  // don't replay it). Static = no entrance (the forecast's hero icon-fly already animated it in).
  switch (s_ev->entrance) {
    case ExpandedViewEntranceTextFromLeft: prv_start_text_in();      break;
    case ExpandedViewEntranceCardFromLeft: prv_start_slide_in_left(); break;
    case ExpandedViewEntranceStatic:       default:                   break;
  }
  s_ev->entrance = ExpandedViewEntranceStatic;
}

static void prv_window_load(Window *window) {
  if (!s_ev) return;
  window_set_background_color(window, GColorWhite);
  GRect bounds = layer_get_bounds(window_get_root_layer(window));
  GRect frame = bounds;
  // Globe-BACK entrance (BOTH shapes): start the whole card off-screen to the left so its first
  // painted frame is already off-left; prv_window_appear then slides it home. (layer_create
  // zeroes the bounds origin, so the draw proc is unaffected.)
  if (s_ev->entrance == ExpandedViewEntranceCardFromLeft) {
    frame.origin.x -= bounds.size.w;
  }
  s_ev->canvas = layer_create(frame);
  layer_set_update_proc(s_ev->canvas, prv_canvas_draw);
  layer_add_child(window_get_root_layer(window), s_ev->canvas);
}

#ifdef CONFIG_TOUCH
// Unsubscribe on DISAPPEAR, not unload: when SELECT/swipe-left pushes the globe, weather.c
// then REMOVES this card from under it — the card's late unload would clobber the touch
// subscription the globe just made in its own appear (touch_service is a single slot).
static void prv_window_disappear(Window *window) {
  (void)window;
  touch_service_unsubscribe();
}
#endif

static void prv_window_unload(Window *window) {
  if (s_ev && s_ev->updated_timer) {
    app_timer_cancel(s_ev->updated_timer);
    s_ev->updated_timer = NULL;
  }
  if (s_ev && s_ev->swap_anim) {
    animation_unschedule(s_ev->swap_anim);    // fires prv_swap_stopped -> destroys + NULLs it
  }
  if (s_ev && s_ev->text_anim) {
    animation_unschedule(s_ev->text_anim);    // fires prv_text_in_stopped -> destroys + NULLs it
  }
  if (s_ev && s_ev->slide_anim) {
    animation_unschedule(s_ev->slide_anim);  // fires prv_slide_*_stopped -> NULLs it
  }
  if (s_ev) {
    if (s_ev->icon)   { gdraw_command_image_destroy(s_ev->icon); s_ev->icon = NULL; }
    if (s_ev->canvas) { layer_destroy(s_ev->canvas); s_ev->canvas = NULL; }
  }
  window_destroy(window);
  if (s_ev) { free(s_ev); s_ev = NULL; }
}

// ---- Public API -----------------------------------------------------------

void expanded_view_push(const WeatherLocationForecast *today,
                        int16_t lat_e2, int16_t lon_e2, int16_t utc_off_min,
                        ExpandedViewEntrance entrance,
                        void (*on_down)(void *ctx), void *on_down_ctx,
                        void (*on_select)(void *ctx), void *on_select_ctx) {
  if (s_ev) return;
  s_ev = calloc(1, sizeof(ExpandedViewData));
  if (!s_ev) return;
  s_ev->on_down = on_down;     s_ev->on_down_ctx = on_down_ctx;
  s_ev->on_select = on_select; s_ev->on_select_ctx = on_select_ctx;
  s_ev->entrance = entrance;
  // Round's status band is a narrow chord — no room for "Last updated H:MM" at this size,
  // and shrinking or moving it was rejected. It rests on the TIME from the outset, so the
  // 2s hold + swap never arm (they would only slide the time out and back in).
  s_ev->show_updated = PBL_IF_ROUND_ELSE(false, true);
  prv_set_from_forecast(today, lat_e2, lon_e2, utc_off_min);

  s_ev->window = window_create();
  if (!s_ev->window) { free(s_ev); s_ev = NULL; return; }
  window_set_window_handlers(s_ev->window, (WindowHandlers){
    .load      = prv_window_load,
    .appear    = prv_window_appear,
#ifdef CONFIG_TOUCH
    .disappear = prv_window_disappear,
#endif
    .unload    = prv_window_unload,
  });
  window_set_click_config_provider(s_ev->window, prv_click_provider);
  // Un-animated: the forecast's hero icon-fly is the entrance; the card's static icon takes over
  // exactly where the flown icon landed (a system slide would fight that hand-off).
  window_stack_push(s_ev->window, false);
}

void expanded_view_dismiss(bool animated) {
  if (s_ev && s_ev->window) {
    window_stack_remove(s_ev->window, animated);
  }
}

bool expanded_view_is_showing(void) {
  return s_ev && s_ev->window;
}

void expanded_view_update_data(const WeatherLocationForecast *today,
                               int16_t lat_e2, int16_t lon_e2, int16_t utc_off_min) {
  if (!s_ev) return;
  prv_set_from_forecast(today, lat_e2, lon_e2, utc_off_min);
  if (s_ev->canvas) layer_mark_dirty(s_ev->canvas);
}
