/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "weather_app_layout.h"
#include "weather_math.h"
#include "applib/graphics/gdraw_command_transforms.h"
#include "shell/prefs.h"
#include "resource_ids.pin.h"

#include <string.h>
#include <time.h>

#define WEATHER_APP_LAYOUT_TOP_PADDING PBL_IF_RECT_ELSE(4, 0)
// Small rect (flint/asterix, 144x168): the emery 200x228 grid does not fit.
// The page keeps its masthead + hero temp but drops the condition-phrase row
// and the UV box (the description's alert line still carries UV), and the
// today:tomorrow split moves up so the footer keeps its full row.
#define WEATHER_APP_LAYOUT_RECT_SMALL (!PBL_ROUND && PBL_DISPLAY_HEIGHT < 200)
#define WEATHER_APP_LAYOUT_CONTENT_LAYER_HORIZONTAL_INSET PBL_IF_RECT_ELSE(3, 2)
#define WEATHER_APP_LAYOUT_LOCATION_BAR_Y PBL_IF_ROUND_ELSE(18, 0)
#define WEATHER_APP_LAYOUT_ROUND_BAR_DEPTH 58
#define WEATHER_APP_LAYOUT_ROUND_BAR_RADIUS 207
#define WEATHER_APP_LAYOUT_ROUND_BAR_Y_ADJUST -13
#define WEATHER_APP_LAYOUT_BAR_LAYER_Y PBL_IF_ROUND_ELSE(0, WEATHER_APP_LAYOUT_LOCATION_BAR_Y)
#define WEATHER_APP_LAYOUT_BAR_LAYER_HEIGHT \
  PBL_IF_ROUND_ELSE(WEATHER_APP_LAYOUT_ROUND_BAR_DEPTH, WEATHER_APP_LAYOUT_MAIN_BAR_HEIGHT)
// Rect: THE ORIGINAL CONDENSED PAGE, revived (from the original
// condensed layout / a14c2675, styled to the original Pebble weather app): no bar at all —
// white page to the top edge, the flow text stack left, discs right, dotted
// rule, next-day block, bottom-centre chevron. Content owns the whole screen.
// ROUND lost its location bar too -- the report page is now a white
// field to the bezel on BOTH shapes, exactly like emery. That reclaims the 53px
// top inset, which is precisely the deficit emery's content set ran at on a
// circle. (Both arms are 0 now; rect's arm was already 0.)
#define WEATHER_APP_LAYOUT_MAIN_CONTENT_TOP 0
#define WEATHER_APP_LAYOUT_CONTENT_Y_SHIFT  0
#define WEATHER_APP_LAYOUT_ACTIVE_BAR_DEPTH \
  PBL_IF_ROUND_ELSE(WEATHER_APP_LAYOUT_ROUND_BAR_DEPTH, WEATHER_APP_LAYOUT_MAIN_BAR_HEIGHT)
#define WEATHER_APP_LAYOUT_LOCATION_BAR_TEXT_INSET PBL_IF_ROUND_ELSE(50, 0)
#define WEATHER_APP_LAYOUT_LOCATION_BAR_TEXT_Y PBL_IF_ROUND_ELSE(22, 1)
#define WEATHER_APP_LAYOUT_ROUND_TIME_TEXT_Y 2
#define WEATHER_APP_LAYOUT_ROUND_LOCATION_TEXT_Y 21
#define WEATHER_APP_LAYOUT_DAY_X_SHIFT PBL_IF_ROUND_ELSE(12, 0)
#define WEATHER_APP_LAYOUT_TEMP_X_SHIFT PBL_IF_ROUND_ELSE(-4, 0)
#define WEATHER_APP_LAYOUT_HIGHLOW_X_SHIFT PBL_IF_ROUND_ELSE(-2, 0)
#define WEATHER_APP_LAYOUT_METRICS_X_SHIFT PBL_IF_ROUND_ELSE(5, 0)
#define WEATHER_APP_LAYOUT_DAY_Y_SHIFT PBL_IF_ROUND_ELSE(-4, 0)
#define WEATHER_APP_LAYOUT_TEXT_STACK_STEP PBL_IF_ROUND_ELSE(34, 0)
#define WEATHER_APP_LAYOUT_TEXT_STACK_GAP PBL_IF_ROUND_ELSE(4, 0)
#define WEATHER_APP_LAYOUT_METRIC_TEXT_Y_INSET PBL_IF_RECT_ELSE(17, 18)
#define WEATHER_APP_LAYOUT_TEMP_Y_SHIFT \
  PBL_IF_ROUND_ELSE(WEATHER_APP_LAYOUT_TEXT_STACK_GAP, 0)
// Rect: +3 drops the high/low line so the LECO temp's glyphs sit visually
// CENTERED between the day label above and the high/low below (LECO carries
// ~6px of top bearing, so equal box gaps read as lopsided without this).
#define WEATHER_APP_LAYOUT_HIGHLOW_Y_SHIFT \
  PBL_IF_ROUND_ELSE(WEATHER_APP_LAYOUT_TEXT_STACK_GAP * 2, 5)
#define WEATHER_APP_LAYOUT_METRICS_Y_SHIFT \
  PBL_IF_ROUND_ELSE(WEATHER_APP_LAYOUT_TEXT_STACK_GAP * 3 - \
                    WEATHER_APP_LAYOUT_METRIC_TEXT_Y_INSET, 2)
#define WEATHER_APP_LAYOUT_TOMORROW_ICON_X_ADJUST PBL_IF_ROUND_ELSE(23, 0)
// (TODAY_ICON_X/Y_ADJUST deleted dead defines, referenced nowhere — the round
// hero disc is positioned by WEATHER_APP_LAYOUT_ROUND_TODAY_ICON_X_INSET / _Y below.)
#define WEATHER_APP_LAYOUT_ICON_Y_ADJUST PBL_IF_ROUND_ELSE(-15, 0)
#define WEATHER_APP_LAYOUT_TOMORROW_ICON_Y_ADJUST PBL_IF_ROUND_ELSE(15, 0)
// Divider lands at screen y194 on round (separator_y 201, less the round-only 3px lift at the
// draw site), y192 on emery.
#define WEATHER_APP_LAYOUT_SEPARATOR_Y_ADJUST \
  PBL_IF_ROUND_ELSE(28, (WEATHER_APP_LAYOUT_RECT_SMALL ? 30 : 44))
// Footer rail: content x2 + 28 -> screen 42, clear of min_x 37 at the
// label's bottom ink row (221).
#define WEATHER_APP_LAYOUT_BOTTOM_TEXT_X_SHIFT PBL_IF_ROUND_ELSE(39, 0)
#define WEATHER_APP_LAYOUT_BOTTOM_LABEL_X_ADJUST PBL_IF_ROUND_ELSE(-18, 0)
#define WEATHER_APP_LAYOUT_BOTTOM_TEMP_X_ADJUST PBL_IF_ROUND_ELSE(4, 0)
// Arc animation: circle is off-screen to the RIGHT.
// REST_ANGLE = 3*TRIG_MAX_ANGLE/4 (9 o'clock from centre) puts the icon
// on-screen to the LEFT of the centre, which is right of screen centre.
// Both today and tomorrow icons ride the same circle with a fixed angular gap.
#define ICON_ARC_RADIUS       PBL_IF_RECT_ELSE(220, 190)
// 50°: at radius 220 the outgoing icon travels ~169px vertically + ~79px
// horizontally — enough to carry the 50px day icon fully past the screen
// corner. The old 40° sweep stopped with an ~18px sliver still on-screen at
// the bottom-right, which then popped away when the animation ended.
#define ICON_ARC_SWEEP_ANGLE  (TRIG_MAX_ANGLE * 50 / 360)
// 9 o'clock: icon is to the left of the circle centre (which is off-screen right)
#define ICON_ARC_TODAY_REST   (TRIG_MAX_ANGLE * 3 / 4)
// Timeline-style landing bounce for day text: anticipation, fast travel,
// then a tiny overshoot back into place.
#define TEXT_MOOOK_MID_FRAMES 3
#define TEXT_MOOOK_BOUNCE_BACK 4
#define ICON_ANIM_DURATION_MS 220

// ---- ROUND report page: a FIXED row grid (screen y == content y; the content
// layer's origin is (12,0)). Emery flows each row from its measured height,
// which cannot be chord-checked at compile time; on a circle every ink band has
// to be proven against BOTH edges of its own row, so round pins them instead.
// Every band below was checked against display_getafix.c's real 260-row mask.
#define WEATHER_APP_LAYOUT_ROUND_TEXT_RAIL   16   // + off->x (2) -> content 18 -> screen 30
// 28_BOLD ink 46..63. This is as high as the day name can go: 'WEDNESDAY' is
// 113px, and between the glass on its left and the today disc's white halo on
// its right (the halo is drawn AFTER the text stack and erases it) the row only
// has ~114px of clear width. Emery's 14px label->temp air is NOT reachable at
// 28_BOLD -- it needs either a smaller day font or a smaller today disc.
// 28_BOLD ink 42..59, CENTRED in the air between the city and the temp: 10px
// above (city ink ends 31) and 10px below (temp ink starts 70). Emery's own
// number is 14px, but emery has nothing above its day name -- matching 14 here
// pushed the day up into the city and read as top-heavy.
#define WEATHER_APP_LAYOUT_ROUND_LABEL_Y      8   // G28_BOLD caps ink ~18..35, centred at
                                                  // the top arc (the round reference's city
                                                  // slot). Measured: widest day name is
                                                  // 112px; chord at ink-top 18 is 138 — fits
                                                  // with 26px total margin.
// reference stack (the original round app's arrangement): the temp,
// hi/lo and condition left the rail and sit on a SHARED AXIS LEFT OF CENTRE
// (STACK_AXIS_DX below) directly under the day name, with the FULL-SIZE 83px
// disc to their right, its top ~level with the temp's cap line — exactly as
// pictured. Verified halo bounds live at DISC_RATIO_NUM.
// Column verticals (design review): temp lowered 4 from its first slot; the
// CONDITION'S BOTTOM ink row stays exactly on the disc fill's bottom row (123);
// the hi/lo splits the remaining air as evenly as whole pixels allow (10 above,
// 9 below).
#define WEATHER_APP_LAYOUT_ROUND_TEMP_Y      42   // LECO_36   ink 53..77
#define WEATHER_APP_LAYOUT_ROUND_HIGHLOW_Y   82   // G18_BOLD  ink 88..100 (10px under temp)
#define WEATHER_APP_LAYOUT_ROUND_PHRASE_Y  103    // G18       ink 110..123 (9px under hi/lo;
                                                  // descender bottom == disc bottom 123)
// The temp / hi-lo / condition column (final): each line's left
// edge FOLLOWS THE CURVE OF THE SCREEN via prv_round_rail — the same parabola
// the description rides — so the column leans with the bezel instead of standing
// on a straight rail. (A straight shared-left-edge column at screen 39 was the
// previous iteration; the curve won design review.) Rail is solved from each
// row's INK TOP; the BEAR trims cancel each font's left side-bearing so the INK,
// not the box, sits on the rail (measured: LECO indents +2, G18/G18_BOLD +1).
#define WEATHER_APP_LAYOUT_ROUND_TEMP_BEAR  (-2)
#define WEATHER_APP_LAYOUT_ROUND_HILO_BEAR  (-1)
#define WEATHER_APP_LAYOUT_ROUND_PHRASE_BEAR (-1)
#define WEATHER_APP_LAYOUT_ROUND_DESC_Y     125   // G14_BOLD  ink 130..154 (2 lines)
#define WEATHER_APP_LAYOUT_ROUND_DESC_H      32   // TWO G14_BOLD lines -- emery's full
                                                  // sentence wraps, and the space the
                                                  // glyph row used to hold is now its.
#define WEATHER_APP_LAYOUT_ROUND_LABEL_W    114   // measure/draw box width for temp + hi/lo
#define WEATHER_APP_LAYOUT_ROUND_PHRASE_W   104   // left-aligned box 38..142 keeps
                                                  // every phrase clear of the disc halo
#define WEATHER_APP_LAYOUT_ROUND_DESC_W     190   // emery's full-measure description rail

// ---- The LEFT RAIL CURVES with the glass, the way official round Pebble apps
// hug the bezel: every row starts further left as it nears the display's widest
// point. Modelled as a parabola matched to the circle (x ~ dy^2/2R for small dy)
// rather than a true chord -- a true chord sags ~10px between the hi/lo and
// phrase rows and reads as a kink, where this stays continuous down the stack.
// 
#define WEATHER_APP_LAYOUT_ROUND_RAIL_MIN     6   // content x -> screen 18
#define WEATHER_APP_LAYOUT_ROUND_RAIL_PIVOT 118   // ink y where the rail bottoms out
#define WEATHER_APP_LAYOUT_ROUND_RAIL_K     320   // 2 * effective radius

// ---- ROUND UV bar: emery's grid, but its LEFT AND RIGHT EDGES ARE ARCS
// CONCENTRIC WITH THE GLASS, so the bar leans with the bezel and keeps a
// near-constant 15-17px margin down its whole height. A straight full-measure
// box would clear at its top row and clip at its bottom -- the circle narrows
// downward, which is exactly the trap that bit the UV box on the 5-day screen.
#define WEATHER_APP_LAYOUT_ROUND_UV_BOX_Y   157   // nudged up 3 with the rule (was 160)
#define WEATHER_APP_LAYOUT_ROUND_UV_BOX_H    34   // emery is 40
#define WEATHER_APP_LAYOUT_ROUND_UV_ARC_R_OUT 116
#define WEATHER_APP_LAYOUT_ROUND_UV_ARC_R_IN  114
#define WEATHER_APP_LAYOUT_ROUND_UV_SQ_DX    (-90) // squares start, RELATIVE TO THE GLASS
                                                   // CENTRE (was content x 28; the report's
                                                   // content layer is inset 12, so its glass
                                                   // centre sits at content x 118 -> 28-118)
#define WEATHER_APP_LAYOUT_ROUND_UV_SQ_DY     21  // BY+21 (emery BY+25)
#define WEATHER_APP_LAYOUT_ROUND_UV_SUN_DX   (-86) // sun-glyph centre (was content x 32)
// Sun + 'UV INDEX' are centred in the band between the widget's top edge (BY)
// and the top of the tally squares (BY+SQ_DY) -- rows 157..177, centre 167.
// Sun glyph is centre-anchored (core r3 + rays to r7, so bbox 160..174); the
// header is box-anchored, and G18 puts its cap ink at box_y+7 .. +17, so
// box BY-2 lands ink 162..172. Both centre on 167. (All BY-relative: the whole
// widget moved up 3 with the rule, so only these absolute rows changed.)
#define WEATHER_APP_LAYOUT_ROUND_UV_SUN_DY    10  // BY+10 -> centre row 167
#define WEATHER_APP_LAYOUT_ROUND_UV_HDR_DY    -2  // BY-2  -> ink 162..172
#define WEATHER_APP_LAYOUT_ROUND_UV_HDR_DX   (-74) // 'UV INDEX' box (was content x 44)
#define WEATHER_APP_LAYOUT_ROUND_UV_NUM_DX    (43) // LECO value box (was content x 161)
#define WEATHER_APP_LAYOUT_ROUND_UV_NUM_DY    -6  // BY-6 (emery BY-5)

// ---- ROUND icon rails (the 75px PDC today icon + the bare 25px footer icon)
// The coloured disc behind the today icon, in HUNDREDTHS of the icon layer's
// width. Rect keeps 1.40x (50 -> 70, unchanged). Round: 1.11x -> 75*111/100 = 83 —
// the intended hero size for this page; a smaller "reference-proportioned" accent
// disc was tried and rejected in design review. The text column instead sits LEFT-ALIGNED
// at STACK_LEFT (ink from screen 38) so the full-size disc owns the right.
// Verified bounds (disc centre (196,83), fill r41.5, halo r45.5; column ink
// starts at screen 38):
//   - glass:  dist((196,83),(130,130)) = 81.0, + halo 45.5 = 126.5 <= 130
//   - label:  halo top edge at the label's last column (185) is row ~39,
//             ~4px under the label's ink bottom (35)
//   - temp:   a "101°" from x38 ends by ~119; halo left across the °'s
//             rows (44..54) is >= 160
//   - hi/lo:  winter worst "-28° / -18°" from x38 ends by ~122; halo left
//             across rows 77..89 is >= 150
//   - phrase: left-aligned W=104 box ends at 142; halo left at its cap row (98)
//             is >= 153
//   - desc:   halo bottom ~128.5 stays above the description's first ink row (130)
#define WEATHER_APP_LAYOUT_DISC_RATIO_NUM PBL_IF_ROUND_ELSE(111, 140)   // 75*111/100 = 83
// Where the outgoing disc has fully collapsed during the UP day-flip, as a raw
// icon width. Rect 32 of 50 (64%); round 48 of 75 (the same 64%).
#define WEATHER_APP_LAYOUT_DISC_COLLAPSE_W0 PBL_IF_ROUND_ELSE(48, 32)
// The full-size disc, back at its user-approved morning position: fill rows
// 41..124 cols 154..236, centre (196,83). Its top (41) sits ~level with the
// temp's cap ink row (44) — the pictured top-alignment — and its bottom clears
// the description's first ink row (130) with the halo at ~128.5.
#define WEATHER_APP_LAYOUT_ROUND_TODAY_ICON_X_INSET    15  // 236-75-15 = 146 -> screen 158
#define WEATHER_APP_LAYOUT_ROUND_TODAY_ICON_Y          45  // layer top; disc top = 41
// The PDC's native viewbox is 75x75 and gdraw_command_frame_draw ignores
// sequence->size, so the ONLY way to shrink the art is to scale its command
// lists in place -- done once at init against the RAM clone. The icon LAYER
// stays 75px, which is what the disc diameter is derived from, so the circle is
// unchanged and only the artwork inside it gets smaller.
#define WEATHER_APP_LAYOUT_ROUND_PDC_NATIVE 75
#define WEATHER_APP_LAYOUT_ROUND_PDC_ART    65   // the user's art size — restored with the
                                                 // 83px disc (65/83 = 0.78)
#define WEATHER_APP_LAYOUT_ROUND_PDC_INSET \
  ((WEATHER_APP_LAYOUT_ROUND_PDC_NATIVE - WEATHER_APP_LAYOUT_ROUND_PDC_ART) / 2)

// ---- ROUND chin: the classic bottom affordance -- a grey segment across the
// chin with a black chevron, shown while there IS another day below. The fin
// owns the footer on the last day, so the two can never coexist.
// Proportions read off the reference: the band is a shallow strip (~7.7% of the
// 260px diameter, not the 10% first tried) and the chevron is nearly as wide as
// the band is tall, where the first pass had it at half that.
// The CITY at the crown, as in the reference. Sits above the today disc's halo
// (whose top row is 32) -- the halo is drawn after the label and would erase it.
// Footer icon disc (round only): a plain 35px filled circle, no white halo.
// The disc, the day label and the hi/lo are ONE GROUP, vertically centred in the
// band between the dotted rule and the chin: rule ends 195, chin starts 240, so
// the free band is rows 196..239 (44) and the group's ink extent is the disc's
// 35 rows -- the label (ink 203..213) and temps (ink 220..230) sit inside it.
// 44-35 leaves 9px of odd slack, so exact centring is impossible: 4 above / 5
// below puts the group centre on 217 against the band's 217.5, and the spare
// pixel goes to the heavy chin side so the group doesn't read as crowded by it.
// Disc centre (193,217) -> rows 200..234, still exactly 5 clear of the rule
// (both moved up 3 together). Moving the group UP is safe against the round
// mask -- higher rows have a WIDER chord -- but if it ever moves DOWN, check the
// bitmap's bottom-right corner first: it is the tightest margin on the page.
#define WEATHER_APP_LAYOUT_ROUND_FOOT_DISC 35
#define WEATHER_APP_LAYOUT_ROUND_FOOT_LABEL_DY  -5
#define WEATHER_APP_LAYOUT_ROUND_FOOT_TEMP_DY   12
#define WEATHER_APP_LAYOUT_ROUND_CITY_X   57   // (city removed; kept for reference)
#define WEATHER_APP_LAYOUT_ROUND_CITY_Y    8   // G24_BOLD ink 18..31; halo top row is 32
#define WEATHER_APP_LAYOUT_ROUND_CITY_W  122   // chord at row 18 is x 64..196; 'LOS ANGELES'
                                               // measures ~119px in 24_BOLD, so it just fits

#define WEATHER_APP_LAYOUT_ROUND_CHIN_Y  240   // band rows 240..259 = 20px
#define WEATHER_APP_LAYOUT_ROUND_CHEV_W   19   // odd width -> 19,17,...,1 scanlines
#define WEATHER_APP_LAYOUT_ROUND_CHEV_Y  249   // rows 249..258, sitting on the band's
                                               // lower edge as in the reference
#define WEATHER_APP_LAYOUT_ROUND_CHEV_H   10
#define WEATHER_APP_LAYOUT_ROUND_TOMORROW_ICON_X_INSET 42  // 236-25-42 = 169 -> screen 181

// Round bar layer height keeps the (removed) pull-to-refresh drawer's sizing so the
// round location bar renders unchanged: ROUND_BAR_DEPTH + 40 (old PULL_TRIGGER_PX) + 6.
#define WEATHER_APP_LAYOUT_ROUND_PULL_LAYER_HEIGHT \
  (WEATHER_APP_LAYOUT_ROUND_BAR_DEPTH + 40 + 6)

#define weather_type_get_bg_color weather_type_bg_color
#define weather_type_get_icon_res_tiny weather_type_icon_tiny_resource
#define weather_type_get_icon_res_today weather_type_icon_small_resource

#if PBL_DISPLAY_HEIGHT >= 200
static const GSize s_today_icon_size = {
  PBL_IF_ROUND_ELSE(75, 50),
  PBL_IF_ROUND_ELSE(75, 50)
};
#else
static const GSize s_today_icon_size = {25, 25};
#endif
static const GSize s_tomorrow_icon_size = {25, 25};

static bool prv_prepare_day_transition(WeatherAppLayout *layout,
                                       const WeatherLocationForecast *new_today,
                                       const WeatherLocationForecast *new_next,
                                       bool animate_down);

__attribute__((unused)) static int prv_draw_text(GPoint offset, int max_width, GContext *context,
                         const char *text, const GFont font,
                         GColor font_color, GTextAlignment alignment) {
  GSize size = graphics_text_layout_get_content_size(
      text, font, GRect(0, 0, max_width, 1000), GTextOverflowModeFill, alignment);
  const int height = size.h;
  const GRect box = (GRect){offset, GSize(max_width, height + 2)};
  graphics_context_set_text_color(context, font_color);
  graphics_draw_text(context, text, font, box, GTextOverflowModeFill, alignment, NULL);
  return height;
}

static void prv_draw_weather_background(const GRect *circle_bounding_box, GContext *context,
                                        GColor background_color) {
  GRect box = *circle_bounding_box;
  GPoint center = grect_center_point(&box);
  // White halo, 4px past the disc: invisible on the white field, but it
  // erases background chrome (the section rule) behind the icon art's
  // transparent edges while an icon crosses it — icons always read as in
  // front of the rule.
  graphics_context_set_fill_color(context, GColorWhite);
  graphics_fill_circle(context, center, (uint16_t)(box.size.w / 2 + 4));
  if (!gcolor_equal(background_color, GColorClear)) {
    graphics_context_set_fill_color(context, background_color);
    graphics_fill_circle(context, center, (uint16_t)(box.size.w / 2));
  }
}

static void prv_set_outgoing_weather_icon_frame(WeatherAppLayout *layout, GRect frame) {
  layer_set_frame(layout->outgoing_weather_icon_layer, frame);
}

#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
static void prv_move_day_icons_to_root(WeatherAppLayout *layout) {
  (void)layout;
}

static void prv_move_day_icons_to_content(WeatherAppLayout *layout) {
  (void)layout;
}
#endif

// One hi/lo formatter for both styles — spaced " / " (classic) and tight "/"
// (newspaper footer). Per-side scratch keeps output byte-identical across all
// four known/unknown combinations (char[12] holds "-32768°" with room).
// snprintf truncation IS the intended bound here (exactly the old per-branch
// behavior); GCC's format-truncation heuristic can't see the value ranges.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
static void prv_fill_high_low_buffer(int high, int low, bool tight,
                                     char *buffer, size_t buffer_size) {
  char hs[12], ls[12];
  if (high == WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
    snprintf(hs, sizeof(hs), "--\xC2\xB0");
  } else {
    snprintf(hs, sizeof(hs), "%d\xC2\xB0", high);
  }
  if (low == WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
    snprintf(ls, sizeof(ls), "--\xC2\xB0");
  } else {
    snprintf(ls, sizeof(ls), "%d\xC2\xB0", low);
  }
  snprintf(buffer, buffer_size, tight ? "%s/%s" : "%s / %s", hs, ls);
}
#pragma GCC diagnostic pop
#define prv_fill_high_low_temp_buffer(h, l, b, n) \
  prv_fill_high_low_buffer((h), (l), false, (b), (n))

static void prv_fill_uv_value_buffer(const WeatherLocationForecast *forecast,
                                     char *buffer, const size_t buffer_size) {
  if (!buffer || buffer_size == 0) return;
  if (forecast && forecast->today_uv >= 0) {
    snprintf(buffer, buffer_size, "%d", forecast->today_uv);
  } else {
    snprintf(buffer, buffer_size, "--");
  }
}

// (Feels-like removed by design: the wind-derived "feels 21°" whisper
// cell and the round "Feels like" metric row are gone. today_feels_like_temp stays
// in the v4 schema should a real phone-supplied feels-like ever be displayed.)


#if PBL_ROUND
// Left rail x (CONTENT coords) for a row whose ink starts at ink_top_y.
static int prv_round_rail(int ink_top_y) {
  const int dy = ink_top_y - WEATHER_APP_LAYOUT_ROUND_RAIL_PIVOT;
  return WEATHER_APP_LAYOUT_ROUND_RAIL_MIN + (dy * dy) / WEATHER_APP_LAYOUT_ROUND_RAIL_K;
}

// The UV bar's frame. Emery draws two concentric rectangles; round draws two
// concentric ARCS OF THE DISPLAY ITSELF -- top and bottom rails stay straight,
// but the left and right edges follow the bezel (r=116 outer, r=114 inner about
// the glass centre). The bar leans with the glass and keeps a near-constant
// 15-17px margin down its whole 34px height, where a straight full-measure box
// would clear at its top row and clip at its bottom (the circle narrows down).
static void prv_draw_uv_bar_frame(GContext *ctx, GPoint gc, int by, int h,
                                  int r_out, int r_in) {
  graphics_context_set_fill_color(ctx, GColorBlack);
  for (int y = by; y < by + h; y++) {
    const int dy = y - gc.y;
    const int ho = (int)weather_isqrt(
        r_out * r_out - dy * dy);
    const int hi = (int)weather_isqrt(
        r_in * r_in - dy * dy);
    if (y < by + 2 || y >= by + h - 2) {
      graphics_fill_rect(ctx, GRect(gc.x - ho, y, 2 * ho + 1, 1), 0, GCornerNone);
    } else {
      graphics_fill_rect(ctx, GRect(gc.x - ho, y, ho - hi, 1), 0, GCornerNone);
      // +1 so the right band lands FLUSH with the rails above/below it; without
      // it the rails overhang the arc by a pixel at every intermediate row.
      graphics_fill_rect(ctx, GRect(gc.x + hi + 1, y, ho - hi, 1), 0, GCornerNone);
    }
  }
}
#endif

// (WHO UV ramp now shared: weather_uv_severity_color in weather_math.c)
#define prv_uv_severity_color weather_uv_severity_color

// What falls from the sky when it falls — for the "Chance of ..." sentence.
static const char *prv_precip_noun(WeatherType t) {
  switch (t) {
    case WeatherType_LightSnow:
    case WeatherType_HeavySnow:   return "snow";
    case WeatherType_RainAndSnow: return "sleet";
    default:                      return "rain";
  }
}

// "Chance of X"/"X likely" are only news when X is NOT already the headline — a
// light-rain day announcing "Chance of rain" reads like the paper didn't look outside.
static bool prv_precip_is_the_headline(WeatherType t) {
  return (t == WeatherType_LightRain) || (t == WeatherType_HeavyRain) ||
         (t == WeatherType_LightSnow) || (t == WeatherType_HeavySnow) ||
         (t == WeatherType_RainAndSnow);
}

// The phone sends temperatures already converted to the user's units, so every absolute
// threshold below has to be stated in those same units.
static bool prv_is_imperial(void) {
  return shell_prefs_get_units_distance() == UnitsDistance_Miles;
}

static int prv_temp(int celsius) {
  return prv_is_imperial() ? (celsius * 9 / 5) + 32 : celsius;
}

// The day's weather WARNING, one short call, picked by severity — dangerous
// conditions first, then exposure, then the rain question, then a calm sign-off
// (the newspaper version's ladder, verbatim). v4.2 raw readings are -1 on older
// records and those rungs simply never fire.
static void prv_build_alert(const WeatherLocationForecast *f, char *buf, size_t buf_size) {
  const WeatherType t = f->current_weather_type;
  const int wmo = f->today_wmo;   // WMO 4677 code, -1 unknown
  const bool feels_known = f->today_feels != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  const bool temp_known  = f->current_temp != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  const bool high_known  = f->today_high != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  const bool low_known   = f->today_low != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  const char *alert;
  char likely[20];
  if (wmo == 96 || wmo == 99) {                        // thunderstorm with hail
    alert = "Hail";
  } else if (wmo >= 95 && wmo <= 99) {                 // thunderstorm codes
    alert = "Chance of storms";
  } else if (t == WeatherType_HeavySnow || wmo == 75 || wmo == 86) {
    alert = "Heavy snow";
  } else if (t == WeatherType_HeavyRain) {
    alert = "Heavy rain";
  } else if (t == WeatherType_RainAndSnow) {
    alert = "Wintry mix";
  } else if (f->today_precip_sum_mm >= 30) {           // a real soaking on the day
    alert = "Flood risk";
  } else if (f->today_wind_mph >= 25) {
    alert = "Strong winds";
  } else if (high_known && f->today_high <= prv_temp(0)) {   // an ice day
    alert = "Below freezing";
  } else if (low_known && f->today_low <= prv_temp(-4)) {
    alert = "Hard frost";
  } else if (feels_known && f->today_feels <= prv_temp(0) &&
             temp_known && f->current_temp > prv_temp(0)) {   // wind chill crosses zero
    alert = "Feels below freezing";
  } else if (high_known && f->today_high >= prv_temp(30)) {
    alert = "Heatwave";
  } else if (f->today_uv >= 8) {
    alert = "Very high UV";
  } else if (f->today_uv >= 6) {
    alert = "High UV";
  } else if (wmo == 45 || wmo == 48 ||                 // fog / depositing rime fog
             (f->today_visibility_m >= 0 && f->today_visibility_m <= 1000)) {
    alert = "Poor visibility";
  } else if (f->today_humidity >= 85 && high_known && f->today_high >= prv_temp(20)) {
    alert = "High humidity";
  } else if (!prv_precip_is_the_headline(t) && f->today_precip_mm >= 60) {
    snprintf(likely, sizeof(likely), "%s likely", prv_precip_noun(t));
    likely[0] = (char)(likely[0] - 'a' + 'A');
    alert = likely;
  } else if (!prv_precip_is_the_headline(t) && f->today_precip_mm > 0) {
    snprintf(likely, sizeof(likely), "Chance of %s", prv_precip_noun(t));
    alert = likely;
  } else if (f->today_wind_mph >= 20) {
    alert = "Windy";
  } else {
    // No warning on file — a calm sign-off, deterministic per day's conditions.
    if (t == WeatherType_Sun) {
      alert = "Clear skies";
    } else if (f->today_wind_mph >= 10) {
      alert = "Light winds";
    } else if (f->today_wind_mph >= 0 && f->today_wind_mph <= 4) {
      alert = "Calm breeze";
    } else if (t == WeatherType_PartlyCloudy) {
      alert = "Fair conditions";
    } else if (temp_known && f->current_temp >= prv_temp(12) &&
               f->current_temp <= prv_temp(24)) {
      alert = "Mild conditions";
    } else {
      alert = "All clear";
    }
  }
  snprintf(buf, buf_size, "%s", alert);
}

// 8-point compass name from degrees (0 = N, clockwise).
static const char *prv_compass8(int deg) {
  static const char kDirs[8][3] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  deg %= 360;
  if (deg < 0) deg += 360;
  return kDirs[((deg + 22) / 45) & 7];
}

// The forecast description: "High UV. Winds SW at 12mph. Precipitation 20%." —
// warning first, then wind (direction when the record carries it), then precip.
// Sentences drop out cleanly when their reading is off the wire.
static void prv_build_forecast_desc(const WeatherLocationForecast *f,
                                    char *buf, size_t buf_size) {
  char alert[24];
  prv_build_alert(f, alert, sizeof(alert));
  int n = snprintf(buf, buf_size, "%s.", alert);
  if (n < 0) return;
  // A wind-flavored warning already said "winds" -- the wind sentence then
  // drops its prefix ("Light winds. SW at 12mph.", not "...Winds SW at...").
  const bool wind_alert = (strstr(alert, "wind") != NULL) ||
                          (strstr(alert, "Wind") != NULL) ||
                          (strstr(alert, "breeze") != NULL);
  if (f->today_wind_mph >= 0 && (size_t)n < buf_size) {
    const bool mph = shell_prefs_get_units_wind() == UnitsWind_Mph;
    const int speed = mph ? f->today_wind_mph
                          : ((f->today_wind_mph * 1609) + 500) / 1000;
    const char *unit = mph ? "mph" : "km/h";
    if (f->today_wind_dir_deg >= 0) {
      n += snprintf(buf + n, buf_size - n, wind_alert ? " %s at %d%s." : " Winds %s at %d%s.",
                    prv_compass8(f->today_wind_dir_deg), speed, unit);
    } else {
      n += snprintf(buf + n, buf_size - n, wind_alert ? " At %d%s." : " Winds at %d%s.",
                    speed, unit);
    }
    if (n < 0) return;
  }
  if (f->today_precip_mm >= 0 && (size_t)n < buf_size) {
    snprintf(buf + n, buf_size - n, " Precipitation %d%%.", f->today_precip_mm);
  }
}

// Little radiant sun for the UV bar: core disc + 8 rays.
static void prv_draw_sun_glyph(GContext *ctx, GPoint c) {
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_context_set_stroke_width(ctx, 1);
  graphics_fill_circle(ctx, c, 3);
  for (int i = 0; i < 8; i++) {
    int32_t a = TRIG_MAX_ANGLE * i / 8;
    int x0 = c.x + (int)(sin_lookup(a) * 5 / TRIG_MAX_RATIO);
    int y0 = c.y - (int)(cos_lookup(a) * 5 / TRIG_MAX_RATIO);
    int x1 = c.x + (int)(sin_lookup(a) * 7 / TRIG_MAX_RATIO);
    int y1 = c.y - (int)(cos_lookup(a) * 7 / TRIG_MAX_RATIO);
    graphics_draw_line(ctx, GPoint(x0, y0), GPoint(x1, y1));
  }
}

#if PBL_ROUND
// These MUST track the two fonts weather_app_layout_init resolves for this board, and its gate
// is `PBL_DISPLAY_HEIGHT >= 200` — NOT shape. Gabbro is 260 tall so it takes the tall branch
// (metrics_value_font = GOTHIC_18, temperature_font = LECO_36). Assuming the short branch here
// silently shifted the report's UV value up ~13px and out to x217; caught by a row-signature
// diff of the report page against its pre-refactor capture.
// Compact preset — the sunset card's bar (rows 189..220), which hangs BELOW that card's centred
// group where the glass chord is much shorter than at the report's rows 157..190.
//
// SAME STACK ORDER AS THE REPORT: sun + "UV INDEX" on top, the ten squares beneath, the LECO
// value up the right. The squares stay FULL SIZE (10px); only the pitch tightens 13 -> 12, the
// same pitch the report itself uses for uv>10.
//
// The one real difference is HORIZONTAL: every element is pushed toward the centre column,
// because down here the chord clips the corners. The squares' bottom row (219) has a half-chord
// of only 74px, so a left-railed row at x42 (what the report uses) would hang outside the arc —
// at SQ_DX -72 they start at x58. Nothing needed to be reordered; the earlier squares-on-top
// version was solving the wrong problem.
//
// DEPTH: h=36 (vs the report's 34). The frame paints 2px rails top AND bottom, so at h=32 the
// squares' last row landed INSIDE the bottom rail and read as touching it; h=36 leaves 3 clear
// rows beneath them. The bar also sits 5px lower now (EV_UV_BAR_Y 194), which costs chord width
// — hence the compact-only arcs at r=124/122 instead of 116/114. That is still well inside the
// glass (7-10px) and still narrower than the report's bar, which is 225px at its top row.
#define WX_UVC_H         36
#define WX_UVC_R_OUT    124
#define WX_UVC_R_IN     122
#define WX_UVC_SQ_SIZE   10
#define WX_UVC_SQ_PITCH  12
#define WX_UVC_SQ_DX   (-72)
#define WX_UVC_SQ_DY     21
#define WX_UVC_NUM_DX    32
#define WX_UVC_NUM_W     46
#define WX_UVC_NUM_DY   (-6)
#define WX_UVC_SUN_DX  (-78)
#define WX_UVC_SUN_DY    10
#define WX_UVC_HDR_DX  (-68)
#define WX_UVC_HDR_DY   (-2)

#if PBL_DISPLAY_HEIGHT >= 200
#define WX_UV_HDR_FONT FONT_KEY_GOTHIC_18
#define WX_UV_NUM_FONT FONT_KEY_LECO_36_BOLD_NUMBERS
#else
#define WX_UV_HDR_FONT FONT_KEY_GOTHIC_14_BOLD
#define WX_UV_NUM_FONT FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM
#endif

// The UV bar as a shared component (declared in weather_app_layout.h). The weather report and
// the sunset card both call this, so there is ONE implementation.
//
// Every element hangs off `gc` — the glass centre in the CALLER'S coordinate space — so the bar
// renders identically whether it is drawn into the report's content layer (inset 12px, glass
// centre at content x 118) or the card's full-screen canvas (glass centre 130). The fonts are
// looked up here rather than taken from a layout so a caller without one can use it; they are
// the same two the report's layout resolves to on round.
int weather_app_layout_uv_bar_height(WeatherUvBarSize size) {
  return (size == WeatherUvBarCompact) ? WX_UVC_H : WEATHER_APP_LAYOUT_ROUND_UV_BOX_H;
}

void weather_app_layout_draw_uv_bar(GContext *ctx, GPoint gc, int by,
                                    int uv_value, const char *uv_text, WeatherUvBarSize size,
                                    const char *label) {
  const bool c = (size == WeatherUvBarCompact);
  const int h       = c ? WX_UVC_H       : WEATHER_APP_LAYOUT_ROUND_UV_BOX_H;
  const int r_out   = c ? WX_UVC_R_OUT   : WEATHER_APP_LAYOUT_ROUND_UV_ARC_R_OUT;
  const int r_in    = c ? WX_UVC_R_IN    : WEATHER_APP_LAYOUT_ROUND_UV_ARC_R_IN;
  const int sun_dx  = c ? WX_UVC_SUN_DX  : WEATHER_APP_LAYOUT_ROUND_UV_SUN_DX;
  const int hdr_dx  = c ? WX_UVC_HDR_DX  : WEATHER_APP_LAYOUT_ROUND_UV_HDR_DX;
  const int hdr_dy  = c ? WX_UVC_HDR_DY  : WEATHER_APP_LAYOUT_ROUND_UV_HDR_DY;
  const int sq_dx   = c ? WX_UVC_SQ_DX   : WEATHER_APP_LAYOUT_ROUND_UV_SQ_DX;
  const int sq_dy   = c ? WX_UVC_SQ_DY   : WEATHER_APP_LAYOUT_ROUND_UV_SQ_DY;
  const int sq_sz   = c ? WX_UVC_SQ_SIZE : 10;
  const int num_dx  = c ? WX_UVC_NUM_DX  : WEATHER_APP_LAYOUT_ROUND_UV_NUM_DX;
  const int num_dy  = c ? WX_UVC_NUM_DY  : WEATHER_APP_LAYOUT_ROUND_UV_NUM_DY;
  const int num_w   = c ? WX_UVC_NUM_W   : 46;
  const char *hdr_font = WX_UV_HDR_FONT;   // both sizes use the report's label font

  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_context_set_stroke_width(ctx, 1);
  prv_draw_uv_bar_frame(ctx, gc, by, h, r_out, r_in);
  prv_draw_sun_glyph(ctx, GPoint(gc.x + sun_dx,
                                 by + (c ? WX_UVC_SUN_DY : WEATHER_APP_LAYOUT_ROUND_UV_SUN_DY)));
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx, label, fonts_get_system_font(hdr_font),
                     GRect(gc.x + hdr_dx, by + hdr_dy, c ? 96 : 70, 20),
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  const int filled = uv_value < 10 ? uv_value : 10;
  const int pitch = c ? WX_UVC_SQ_PITCH : ((uv_value > 10) ? 12 : 13);
  for (int i = 0; i < 10; i++) {
    const GRect sq = GRect(gc.x + sq_dx + i * pitch, by + sq_dy, sq_sz, sq_sz);
    if (i < filled) {
      graphics_context_set_fill_color(ctx, prv_uv_severity_color(uv_value));
      graphics_fill_rect(ctx, sq, 0, GCornerNone);
    }
    graphics_draw_rect(ctx, sq);
  }
  if (uv_value > 10 && filled >= 10) {   // "10+" tick past the last square
    const int px = gc.x + sq_dx + 9 * pitch + sq_sz + 3;
    const int py = by + sq_dy + sq_sz / 2;
    graphics_context_set_stroke_width(ctx, 2);
    graphics_draw_line(ctx, GPoint(px, py), GPoint(px + 6, py));
    graphics_draw_line(ctx, GPoint(px + 3, py - 3), GPoint(px + 3, py + 3));
    graphics_context_set_stroke_width(ctx, 1);
  }
  graphics_context_set_fill_color(ctx, GColorBlack);
  // The VALUE stays LECO_36 in both sizes — the compact bar shrinks its squares and pulls the
  // value box in, never the digit itself.
  graphics_draw_text(ctx, uv_text, fonts_get_system_font(WX_UV_NUM_FONT),
                     GRect(gc.x + num_dx, by + num_dy, num_w, 44),
                     GTextOverflowModeFill, GTextAlignmentRight, NULL);
}
#endif  // PBL_ROUND

static int prv_interpolate_text_moook(AnimationProgress progress, int from, int to) {
  static const int8_t frames_in[] = {0, 1, 20};
  static const int8_t frames_out[] = {TEXT_MOOOK_BOUNCE_BACK, 2, 1, 0};
  const int32_t num_in = sizeof(frames_in) / sizeof(frames_in[0]);
  const int32_t num_mid = TEXT_MOOOK_MID_FRAMES;
  const int32_t num_out = sizeof(frames_out) / sizeof(frames_out[0]);
  const int32_t num_total = num_in + num_mid + num_out;
  const int32_t dir = (from == to) ? 0 : ((from < to) ? 1 : -1);
  if (dir == 0 || progress >= ANIMATION_NORMALIZED_MAX) return to;

  int32_t frame_idx = (progress * num_total +
                       (ANIMATION_NORMALIZED_MAX / (2 * num_total))) /
                      ANIMATION_NORMALIZED_MAX;
  if (frame_idx < 0) frame_idx = 0;
  if (frame_idx >= num_total) frame_idx = num_total - 1;

  if (frame_idx < num_in) {
    return from + (int)(dir * frames_in[frame_idx]);
  }

  if (frame_idx < num_in + num_mid) {
    int32_t shifted = progress -
        (num_in * ANIMATION_NORMALIZED_MAX / num_total);
    int32_t mid_normalized = num_total * shifted / num_mid;
    if (mid_normalized < 0) mid_normalized = 0;
    if (mid_normalized > ANIMATION_NORMALIZED_MAX) {
      mid_normalized = ANIMATION_NORMALIZED_MAX;
    }

    const int32_t start = from + dir * frames_in[num_in - 1];
    const int32_t end = to + dir * frames_out[0];
    return (int)(start + ((end - start) * mid_normalized / ANIMATION_NORMALIZED_MAX));
  }

  return to + (int)(dir * frames_out[frame_idx - (num_in + num_mid)]);
}

static int32_t prv_interpolate_icon_landing_progress(AnimationProgress progress) {
  const int32_t max = ANIMATION_NORMALIZED_MAX;
  if (progress >= max) return max;

  const int32_t overshoot = max / 36;
  const int32_t rebound = max / 120;
  const int32_t hit = max * 86 / 100;
  const int32_t settle = max * 95 / 100;

  if ((int32_t)progress < hit) {
    return weather_scale_i32(progress, max + overshoot, hit);
  }

  if ((int32_t)progress < settle) {
    return max + overshoot -
        weather_scale_i32(progress - hit, overshoot + rebound, settle - hit);
  }

  return max - rebound +
      weather_scale_i32(progress - settle, rebound, max - settle);
}

static int32_t prv_interpolate_icon_landing_progress_down(AnimationProgress progress) {
  const int32_t max = ANIMATION_NORMALIZED_MAX;
  if (progress >= max) return max;

  const int32_t rebound = max / 140;
  const int32_t hit = max * 88 / 100;
  const int32_t settle = max * 95 / 100;

  if ((int32_t)progress < hit) {
    return weather_scale_i32(progress, max, hit);
  }

  if ((int32_t)progress < settle) {
    return max - weather_scale_i32(progress - hit, rebound, settle - hit);
  }

  return max - rebound +
      weather_scale_i32(progress - settle, rebound, max - settle);
}

static void prv_draw_fin_layer(Layer *layer, GContext *context) {
  WeatherAppLayout *layout = *(WeatherAppLayout **)layer_get_data(layer);
  if (!layout || !layout->fin_pdc) return;

  graphics_context_set_compositing_mode(context, GCompOpSet);
  gdraw_command_image_draw(context, layout->fin_pdc, GPointZero);
}

static void prv_fin_anim_update(Animation *anim, AnimationProgress progress) {
  WeatherAppLayout *layout = (WeatherAppLayout *)animation_get_context(anim);
  GRect frame = layout->fin_anim.to;
  frame.origin.x = prv_interpolate_text_moook(
      progress, layout->fin_anim.from.origin.x, layout->fin_anim.to.origin.x);
  frame.origin.y = prv_interpolate_text_moook(
      progress, layout->fin_anim.from.origin.y, layout->fin_anim.to.origin.y);
  layer_set_frame(layout->fin_layer, frame);
}

static void prv_fin_anim_stopped(Animation *anim, bool finished, void *context) {
  WeatherAppLayout *layout = (WeatherAppLayout *)context;
  if (layout) {
    layout->fin_animation = NULL;
    if (finished && layout->fin_layer) {
      layer_set_frame(layout->fin_layer, layout->fin_anim.to);
    }
  }
  animation_destroy(anim);
}

static const AnimationImplementation s_fin_anim_impl = {
  .update = prv_fin_anim_update,
};

// ---- Text animation snapshot helpers ----
// Featured temperature: current temp, or the daily high on future days.
static void prv_fill_featured_temp_buffer(const WeatherLocationForecast *f,
                                          char *buffer, size_t buffer_size) {
  if (f->current_temp == WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
    if (f->today_high != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP)
      snprintf(buffer, buffer_size, "%d\xC2\xB0", f->today_high);
    else
      snprintf(buffer, buffer_size, "--\xC2\xB0");
  } else {
    snprintf(buffer, buffer_size, "%d\xC2\xB0", f->current_temp);
  }
}

// Uppercase ASCII copy — the sketch spec sets headers/condition in caps.
__attribute__((unused)) static void prv_upcase_into(char *dst, size_t dst_size, const char *src) {
  size_t i = 0;
  for (; src && src[i] && i < dst_size - 1; i++) {
    char c = src[i];
    dst[i] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
  }
  dst[i] = '\0';
}

// Capture current forecast strings into text_anim before the forecast pointer is updated.
static void prv_snapshot_text(WeatherAppLayout *layout) {
  const WeatherLocationForecast *f = layout->forecast;
  if (f) {
    const char *lbl = (f->label && f->label[0]) ? f->label : "TODAY";
    strncpy(layout->text_anim.top_label, lbl, sizeof(layout->text_anim.top_label) - 1);
    layout->text_anim.top_label[sizeof(layout->text_anim.top_label) - 1] = '\0';

    prv_fill_featured_temp_buffer(f, layout->text_anim.top_temp,
                                  sizeof(layout->text_anim.top_temp));
#if PBL_ROUND
    prv_fill_high_low_temp_buffer(f->today_high, f->today_low,
                                  layout->text_anim.top_highlow,
                                  sizeof(layout->text_anim.top_highlow));
#else
    prv_fill_high_low_temp_buffer(f->today_high, f->today_low,
                                  layout->text_anim.top_highlow,
                                  sizeof(layout->text_anim.top_highlow));
#endif
    strncpy(layout->text_anim.top_phrase,
            f->current_weather_phrase ? f->current_weather_phrase : "",
            sizeof(layout->text_anim.top_phrase) - 1);
    layout->text_anim.top_phrase[sizeof(layout->text_anim.top_phrase) - 1] = '\0';
    prv_build_forecast_desc(f, layout->text_anim.top_desc,
                            sizeof(layout->text_anim.top_desc));
    prv_fill_uv_value_buffer(f,
                             layout->text_anim.top_uv,
                             sizeof(layout->text_anim.top_uv));
  } else {
    layout->text_anim.top_label[0] = '\0';
    layout->text_anim.top_temp[0]  = '\0';
    layout->text_anim.top_phrase[0] = '\0';
    layout->text_anim.top_desc[0] = '\0';
    layout->text_anim.top_highlow[0] = '\0';
    layout->text_anim.top_uv[0] = '\0';
  }

  const WeatherLocationForecast *n = layout->next_forecast;
  if (n) {
    const char *lbl = (n->label && n->label[0]) ? n->label : "TOMORROW";
    strncpy(layout->text_anim.bot_label, lbl, sizeof(layout->text_anim.bot_label) - 1);
    layout->text_anim.bot_label[sizeof(layout->text_anim.bot_label) - 1] = '\0';
    prv_fill_high_low_buffer(n->today_high, n->today_low,
                             true /* tight on both shapes now */,
                             layout->text_anim.bot_highlow,
                             sizeof(layout->text_anim.bot_highlow));
    layout->text_anim.bot_valid = true;
  } else {
    layout->text_anim.bot_valid = false;
  }
}

// One string-driven renderer for the top-half rows, shared by the outgoing
// snapshot pass and the live pass — the row geometry lives here exactly once.
typedef struct {
  const char *label, *temp, *highlow;
  const char *uv, *phrase, *desc;   // UV bar, condition line, forecast description
} TopText;

static void prv_draw_top_rows(const WeatherAppLayout *layout, GPoint *off, int cw,
                              GContext *ctx, const TopText *t, int label_temp_gap,
                              bool draw_uv_box) {
#if PBL_ROUND
  (void)label_temp_gap;
  // ROUND : emery's page, on a FIXED grid. Same rows, same
  // fonts, same order -- day label, hero temp, hi/lo, condition phrase, the
  // forecast sentence, the rain/wind glyphs, then the UV bar. Every box is an
  // explicit graphics_draw_text with TrailingEllipsis rather than prv_draw_text,
  // because prv_draw_text measures into a 1000px-tall box and WRAPS: a long
  // phone-supplied phrase would spill a second line straight through the
  // description below it, which a fixed grid cannot absorb the way a flow can.
  // Ink-top offsets per font (box y -> first ink row): G24_BOLD/G28_BOLD +10,
  // LECO_36 +11, G18_BOLD/G18 +7, G14_BOLD +5. Label ink 42..55 then leaves a
  // 14px gap to the temp's ink at 70 -- emery's exact label->temp air. The rail is solved from the ink, not the
  // box, so the curve follows what the eye actually sees.
  graphics_context_set_text_color(ctx, GColorBlack);

  char caps[24];
  prv_upcase_into(caps, sizeof(caps), t->label);
  // Glass centre in CONTENT coords — derived, never hard-coded (the content layer's
  // inset is asymmetric-safe only via the derived centre, same derivation as the UV
  // bar's gc). Every centred row below hangs off this. off->x (a constant 2 from the
  // round HORIZONTAL_INSET, in both the static and anim passes) is subtracted here
  // because every box below adds it back — without this the whole centred stack sat
  // 2px right of the glass axis (verified by pixel measurement ).
  const int gcx = PBL_DISPLAY_WIDTH / 2 - layout->content_layer_origin.x - off->x;
  // Day name centred at the crown. Rides off->x/off->y so the day-flip slide
  // carries it like every other row. Font/size unchanged (G28_BOLD caps).
  graphics_draw_text(ctx, caps, layout->location_font,
                     GRect(off->x + gcx - 100,
                           off->y + WEATHER_APP_LAYOUT_ROUND_LABEL_Y, 200, 30),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  // Temp / hi-lo / condition: a left column whose edge FOLLOWS THE CURVE OF THE
  // SCREEN (design review) — each line starts from prv_round_rail solved at
  // its own ink-top row, the same parabola the description rides, so the stack
  // leans with the bezel. BEAR trims put the INK (not the box) on the rail.
  graphics_draw_text(ctx, t->temp, layout->temperature_font,
                     GRect(off->x + prv_round_rail(WEATHER_APP_LAYOUT_ROUND_TEMP_Y + 11) +
                               WEATHER_APP_LAYOUT_ROUND_TEMP_BEAR,
                           off->y + WEATHER_APP_LAYOUT_ROUND_TEMP_Y,
                           WEATHER_APP_LAYOUT_ROUND_LABEL_W, 38),
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  graphics_draw_text(ctx, t->highlow, layout->high_low_phrase_font,
                     GRect(off->x + prv_round_rail(WEATHER_APP_LAYOUT_ROUND_HIGHLOW_Y + 6) +
                               WEATHER_APP_LAYOUT_ROUND_HILO_BEAR,
                           off->y + WEATHER_APP_LAYOUT_ROUND_HIGHLOW_Y,
                           WEATHER_APP_LAYOUT_ROUND_LABEL_W, 20),
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  if (t->phrase && t->phrase[0]) {
    graphics_draw_text(ctx, t->phrase, layout->metrics_value_font,
                       GRect(off->x + prv_round_rail(WEATHER_APP_LAYOUT_ROUND_PHRASE_Y + 7) +
                                 WEATHER_APP_LAYOUT_ROUND_PHRASE_BEAR,
                             off->y + WEATHER_APP_LAYOUT_ROUND_PHRASE_Y,
                             WEATHER_APP_LAYOUT_ROUND_PHRASE_W, 20),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
  }
  // Emery's sentence, verbatim -- "Light winds. SW at 12mph. Precipitation 20%."
  // It wraps to two lines at this measure, which is why it gets DESC_H and the
  // row the rain/wind glyphs used to occupy.
  if (t->desc && t->desc[0]) {
    graphics_draw_text(ctx, t->desc, layout->metrics_font,
                       GRect(off->x + prv_round_rail(WEATHER_APP_LAYOUT_ROUND_DESC_Y + 5),
                             off->y + WEATHER_APP_LAYOUT_ROUND_DESC_Y,
                             WEATHER_APP_LAYOUT_ROUND_DESC_W,
                             WEATHER_APP_LAYOUT_ROUND_DESC_H),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
  }

  // The UV bar. Same gate as emery: it draws only on the STATIC render, so it
  // appears in place once the day-flip settles instead of sliding with the text.
  if (draw_uv_box && t->uv && t->uv[0] && t->uv[0] != '-') {
    int uvv = 0;
    for (const char *ch = t->uv; *ch >= '0' && *ch <= '9'; ch++) {
      uvv = uvv * 10 + (*ch - '0');
    }
    const int by = off->y + WEATHER_APP_LAYOUT_ROUND_UV_BOX_Y;
    // Glass centre in CONTENT coords -- derived, never hard-coded, so the bar
    // stays concentric if the content layer ever moves.
    const GPoint gc = GPoint(PBL_DISPLAY_WIDTH / 2 - layout->content_layer_origin.x,
                             PBL_DISPLAY_HEIGHT / 2 - layout->content_layer_origin.y);
    weather_app_layout_draw_uv_bar(ctx, gc, by, uvv, t->uv, WeatherUvBarFull, "UV INDEX");
  }
#else
  // Rect: the ORIGINAL flow layout (the original condensed layout / a14c2675, styled to the
  // original Pebble weather app): each line advances by its measured height
  // from the content rail; off->y carries the day-scroll slide (0 at rest).
  // Stack: day label, hero temp (LECO, inline °), "hi° / lo°", the condition
  // phrase (restored per the original design), then the rain/wind row.
  GPoint line = GPoint(off->x, off->y + 1);
  char caps[24];
  prv_upcase_into(caps, sizeof(caps), t->label);
  // Day name in GOTHIC_28_BOLD (heavier/thicker caps to match the reference's
  // chunky "PALO ALTO"; the original app's 18_BOLD reads chunky at 144px, and
  // 24_BOLD's 2px stems looked thin scaled up). ADVANCE is pinned to the
  // 24_BOLD height, NOT the 28_BOLD one, so the temp/hilo/phrase stack below
  // keeps its calibrated positions — the bigger caps grow down into the
  // existing 14px label→temp whitespace, they don't push the temp down.
  // Label→temp air: 17px ink gap minus the user's −3 trim = 14 (gap param 3);
  // the rows below stay reference-tight (9/4) and ride with the temp.
  prv_draw_text(line, cw, ctx, caps, layout->location_font,
                GColorBlack, GTextAlignmentLeft);
  line.y += fonts_get_font_height(fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  line.y += label_temp_gap;
  // Even ink gaps down the temp / hi-lo / condition stack (matching
  // gabbro's equal spacing), with the CONDITION anchored back at its original row so it
  // sits just above the forecast sentence (10px, as before). Measured: temp ink ends 63,
  // condition ink 93..106, sentence at 117. The -1 after the temp centres the hi/lo in
  // the 29px between (air 7 above / 6 below its mass); the -1 after the hi/lo keeps the
  // condition riding at 93. (The temp trim moves everything below it; only the hi/lo trim
  // moves the condition relative to the hi/lo.)
  line.y += prv_draw_text(line, cw, ctx, t->temp, layout->temperature_font,
                          GColorBlack, GTextAlignmentLeft) - 1;
  line.y += prv_draw_text(line, cw, ctx, t->highlow, layout->high_low_phrase_font,
                          GColorBlack, GTextAlignmentLeft) - 1;
  if (!WEATHER_APP_LAYOUT_RECT_SMALL && t->phrase && t->phrase[0]) {
    // Width-limited so a long phrase ellipsizes before the disc (disc's left
    // edge sits ~x116 in content coords). The small rects skip the phrase row
    // (the header icon carries the condition) to keep the description on-page.
    line.y += prv_draw_text(line, 112, ctx, t->phrase, layout->metrics_value_font,
                            GColorBlack, GTextAlignmentLeft) - 2;
  }
  // Forecast description ("High UV. Winds SW at 12mph. Precipitation 20%."):
  // warning, wind, precip. Measured and CENTERED in the band between the
  // condition text's bottom (the flow's current line) and the UV box top —
  // one- and two-line descriptions both float mid-band.
  const int band_bot_anchor = WEATHER_APP_LAYOUT_RECT_SMALL ? 122 : 150;
  int box_top = off->y + band_bot_anchor;   // no-description fallback: the classic anchor
  if (t->desc && t->desc[0]) {
    const GSize dsz = graphics_text_layout_get_content_size(
        t->desc, layout->metrics_font, GRect(0, 0, cw, 40),
        GTextOverflowModeWordWrap, GTextAlignmentLeft);
    const int band_top = line.y;
    const int band_bot = off->y + band_bot_anchor;  // the box's nominal top (keeps desc placement)
    int desc_y = band_top + (band_bot - band_top - dsz.h) / 2;
    if (desc_y < band_top) desc_y = band_top;
    graphics_context_set_text_color(ctx, GColorBlack);
    graphics_draw_text(ctx, t->desc, layout->metrics_font,
                       GRect(off->x, desc_y, cw, dsz.h + 2),
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
    // Center the UV box between the description's ink bottom and the y195
    // divider — per page, since one- and two-line descriptions end at
    // different depths (same optical rule as the fin below the divider).
    const int desc_bot = desc_y + dsz.h + 2;   // +2: G14's descent runs past the metric
    const int divider_y = off->y + 195;
    box_top = desc_bot + (divider_y - desc_bot - 40) / 2;
    if (box_top < desc_bot) box_top = desc_bot;
  }

  // The BIG UV BAR (brought back from the newspaper version, verbatim grid):
  // double-bordered full-measure box, sun + "UV INDEX" header top-left, ten
  // 10px tally squares (13px pitch) filled in the WHO severity color, and a
  // near-box-height LECO numeral on the right. The 40px box floats centred
  // between the description above and the y195 divider (box_top, computed
  // per page). Hidden when UV is off the wire.
  //
  // Decoupled from the day-scroll slide: draw_uv_box is FALSE for every frame
  // of the transition (outgoing snapshot + incoming text both pass false), so
  // the box is absent while the page slides and the weather bitmaps arc into
  // place, then simply APPEARS — fully drawn, no animation — on the first
  // static render once the animation settles (text_anim.active == false).
  if (!WEATHER_APP_LAYOUT_RECT_SMALL && draw_uv_box && t->uv && t->uv[0] && t->uv[0] != '-') {
    int uvv = 0;
    for (const char *c = t->uv; *c >= '0' && *c <= '9'; c++) {
      uvv = uvv * 10 + (*c - '0');
    }
    const int by = box_top;
    graphics_context_set_stroke_color(ctx, GColorBlack);
    graphics_context_set_stroke_width(ctx, 1);
    graphics_draw_rect(ctx, GRect(off->x, by, cw, 40));
    graphics_draw_rect(ctx, GRect(off->x + 1, by + 1, cw - 2, 38));
    prv_draw_sun_glyph(ctx, GPoint(off->x + 12, by + 14));
    // Gothic-14-Bold's V glyph fuses into a solid stem (reads as "UY") —
    // Gothic 18's V tapers properly (metrics_value_font is G18 on rect).
    graphics_draw_text(ctx, "UV INDEX", layout->metrics_value_font,
                       GRect(off->x + 24, by + 2, 70, 20),
                       GTextOverflowModeFill, GTextAlignmentLeft, NULL);
    // Fully filled the instant the box appears — no tick-in (the box no longer
    // rides any animation; it just renders in once the transition settles).
    const int filled = uvv < 10 ? uvv : 10;
    // Off the WHO scale (11+) the pitch tightens 13->12: the meter maxes out
    // and the freed lane takes a '+' after the last square (instead of an
    // eleventh box) with clean air on both sides — a 2-digit LECO numeral's
    // foot serif reaches in to ~x141 content, so the 13-pitch lane is too slim.
    const int pitch = (uvv > 10) ? 12 : 13;
    for (int i = 0; i < 10; i++) {
      const GRect sq = GRect(off->x + 5 + i * pitch, by + 25, 10, 10);
      if (i < filled) {
        graphics_context_set_fill_color(ctx, prv_uv_severity_color(uvv));
        graphics_fill_rect(ctx, sq, 0, GCornerNone);
      }
      graphics_draw_rect(ctx, sq);
    }
    if (uvv > 10 && filled >= 10) {              // appears with the final tick
      const int px = off->x + 5 + 9 * pitch + 10 + 3;
      const int py = by + 30;                    // the squares' vertical center
      graphics_context_set_stroke_width(ctx, 2);
      graphics_draw_line(ctx, GPoint(px, py), GPoint(px + 6, py));
      graphics_draw_line(ctx, GPoint(px + 3, py - 3), GPoint(px + 3, py + 3));
      graphics_context_set_stroke_width(ctx, 1);
    }
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_draw_text(ctx, t->uv, layout->temperature_font,
                       GRect(off->x + cw - 52, by - 5, 46, 44),
                       GTextOverflowModeFill, GTextAlignmentRight, NULL);
  }
#endif
}

// Draw top-half text from the snapshot (outgoing frame).
static void prv_draw_snapshot_top(const WeatherAppLayout *layout, GPoint *off,
                                   int cw, GContext *ctx) {
  const TopText t = {
    .label = layout->text_anim.top_label,
    .temp = layout->text_anim.top_temp,
    .highlow = layout->text_anim.top_highlow,
    .uv = layout->text_anim.top_uv,
    .phrase = layout->text_anim.top_phrase,
    .desc = layout->text_anim.top_desc,
  };
  // Outgoing (snapshot) frame is only ever drawn mid-transition -> never the box.
  prv_draw_top_rows(layout, off, cw, ctx, &t, PBL_IF_RECT_ELSE(3, 0), false);
}

static void prv_draw_top_half_text(const WeatherAppLayout *layout, GPoint *current_offset,
                                   int content_width, GContext *context) {
  const WeatherLocationForecast *forecast = layout->forecast;

  char temp_buffer[15] = {0};
  char highlow_buffer[15] = {0};
  char desc_buffer[72] = {0};
  char uv_buffer[12] = {0};
  prv_build_forecast_desc(forecast, desc_buffer, sizeof(desc_buffer));
  prv_fill_uv_value_buffer(forecast, uv_buffer, sizeof(uv_buffer));
  prv_fill_featured_temp_buffer(forecast, temp_buffer, sizeof(temp_buffer));
  prv_fill_high_low_temp_buffer(forecast->today_high, forecast->today_low,
                                highlow_buffer, sizeof(highlow_buffer));
  const TopText t = {
    .label = (forecast->label && forecast->label[0]) ? forecast->label : "TODAY",
    .temp = temp_buffer,
    .highlow = highlow_buffer,
    .uv = uv_buffer,
    .phrase = forecast->current_weather_phrase,
    .desc = desc_buffer,
  };
  // The box draws only on the STATIC render (transition settled) — never while
  // the page/text is sliding, so it appears in place rather than sliding with it.
  prv_draw_top_rows(layout, current_offset, content_width, context, &t,
                    PBL_IF_RECT_ELSE(3, 0), !layout->text_anim.active);
}

// Shared bottom-half rows (label + high/low), string-driven.
static void prv_draw_bottom_rows(const WeatherAppLayout *layout, GPoint *off, int cw,
                                 GContext *ctx, const char *label, const char *highlow,
                                 const char *phrase, int gap) {
#if PBL_ROUND
  // ROUND: the reference's STACKED block -- the day name bold on its own line
  // with the high/low beneath it, bare icon railed right. The rail is pushed in
  // to content 41 (screen 53) because the lower line's ink reaches row 233,
  // where the glass only starts at x50.7 -- the single-row footer could sit at
  // screen 42 precisely because it never went that deep.
  (void)gap; (void)phrase;
  off->x += WEATHER_APP_LAYOUT_BOTTOM_TEXT_X_SHIFT;
  char rcaps[24];
  prv_upcase_into(rcaps, sizeof(rcaps), label);
  graphics_context_set_text_color(ctx, GColorBlack);
  const int r_icon_left = layout->tomorrow_icon_rest_frame.origin.x;
  const int r_w = r_icon_left - 4 - off->x;
  graphics_draw_text(ctx, rcaps, layout->tomorrow_font,
                     GRect(off->x, off->y + WEATHER_APP_LAYOUT_ROUND_FOOT_LABEL_DY, r_w, 20),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
  graphics_draw_text(ctx, highlow, layout->tomorrow_font,
                     GRect(off->x, off->y + WEATHER_APP_LAYOUT_ROUND_FOOT_TEMP_DY, r_w, 20),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
#else
  // The NEWSPAPER FOOTER SLOT (rect)  // Rect: the NEWSPAPER FOOTER SLOT (brought back from the front-page-spread
  // version): one compact row under the rule — next-day label + tight temps on
  // the left, the bare 25px bitmap right-railed by its rest frame. Fixed
  // anchors so nothing jumps between pages; temps ellipsize before they can
  // reach the icon (double-negative winters).
  (void)gap; (void)phrase;
  off->x += WEATHER_APP_LAYOUT_BOTTOM_TEXT_X_SHIFT;   // 0 on rect
  const int row_y = off->y + 3;
  // Small rect: no room for the 25px icon under the rule, so the footer is
  // text-only in G14_BOLD and the label+temps take the full width.
  GFont foot_font = WEATHER_APP_LAYOUT_RECT_SMALL ? layout->metrics_font
                                                  : layout->tomorrow_font;
  char caps[24];
  prv_upcase_into(caps, sizeof(caps), label);
  graphics_context_set_text_color(ctx, GColorBlack);
  const GSize lsz = graphics_text_layout_get_content_size(
      caps, foot_font, GRect(0, 0, 200, 20), GTextOverflowModeFill,
      GTextAlignmentLeft);
  graphics_draw_text(ctx, caps, foot_font,
                     GRect(off->x, row_y, lsz.w + 2, 20),
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  // Derived from the frame the bitmap actually rests in, so the ellipsis point
  // can never drift away from the icon. On rect this is 190-25-3 = 162, exactly
  // what the old cw-based formula produced; on round it self-syncs to 177.
  const int icon_left = WEATHER_APP_LAYOUT_RECT_SMALL
      ? (off->x + cw + 4)
      : layout->tomorrow_icon_rest_frame.origin.x;
  const int tx = off->x + lsz.w + PBL_IF_RECT_ELSE(10, 5);
  graphics_draw_text(ctx, highlow, foot_font,
                     GRect(tx, row_y, icon_left - 4 - tx, 20),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, NULL);
#endif
}

// Draw bottom-half text from the snapshot (outgoing frame).
static void prv_draw_snapshot_bot(const WeatherAppLayout *layout, GPoint *off,
                                   int cw, GContext *ctx) {
  if (!layout->text_anim.bot_valid) return;
  prv_draw_bottom_rows(layout, off, cw, ctx, layout->text_anim.bot_label,
                       layout->text_anim.bot_highlow, NULL, 6);
}

static void prv_draw_bottom_half_text(const WeatherAppLayout *layout, GPoint *current_offset,
                                      int content_width, GContext *context) {
  const WeatherLocationForecast *next = layout->next_forecast;
  if (!next) return;
  char text_buffer[15] = {0};
  prv_fill_high_low_buffer(next->today_high, next->today_low,
                           true /* tight on both shapes now */,
                           text_buffer, sizeof(text_buffer));
  prv_draw_bottom_rows(layout, current_offset, content_width, context,
                       (next->label && next->label[0]) ? next->label : "TOMORROW",
                       text_buffer, next->current_weather_phrase,
                       PBL_IF_RECT_ELSE(2, 10));
}

#if PBL_ROUND
// Round's day-flip disc size for the SCALER layer. Gabbro now rests a 35px
// circle in the footer slot, so the travelling disc must land on it: it lerps
// between the footer disc and the today disc across the icon's own width range.
// (Emery keeps collapsing to zero -- its footer bitmap is bare, so there is
// nothing there for the disc to become.)
static int prv_round_scaler_disc(int w) {
  const int lo = WEATHER_APP_LAYOUT_ROUND_FOOT_DISC;
  const int hi = s_today_icon_size.w * WEATHER_APP_LAYOUT_DISC_RATIO_NUM / 100;
  const int w0 = s_tomorrow_icon_size.w;
  const int w1 = s_today_icon_size.w;
  if (w <= w0 || w1 <= w0) return lo;
  if (w >= w1) return hi;
  return lo + (hi - lo) * (w - w0) / (w1 - w0);
}
#endif

static void prv_draw_circle_at_layer(Layer *icon_layer, GContext *context,
                                     WeatherType weather_type) {
  GRect frame = layer_get_frame(icon_layer);
  const GSize sz = frame.size;
  const int diam = (int)(sz.w * WEATHER_APP_LAYOUT_DISC_RATIO_NUM / 100);
  GRect bg_circle = (GRect){
    .origin = GPoint(frame.origin.x + sz.w / 2 - diam / 2,
                     frame.origin.y + sz.h / 2 - diam / 2),
    .size = GSize(diam, diam),
  };
  prv_draw_weather_background(&bg_circle, context, weather_type_disc_color(weather_type));
}

#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
static void prv_draw_weather_pdc_frame(const WeatherAppLayout *layout,
                                       GContext *ctx,
                                       WeatherType weather_type,
                                       GRect frame_rect) {
  int frame_index = ((int)weather_type <= WeatherType_RainAndSnow)
      ? (int)weather_type : WeatherType_Generic;
  // No set_bounds_size: the PDC's native viewbox is already the displayed size
  // (75x75) and gdraw_command_frame_draw never reads sequence->size — it draws at
  // the native coords offset by frame_rect.origin. (The sequence is also a RAM
  // clone now, so a write would be legal, but it's simply unnecessary here.)
  GDrawCommandFrame *frame =
      gdraw_command_sequence_get_frame_by_index(layout->weather_icon_pdc_sequence,
                                                frame_index);
  gdraw_command_frame_draw(ctx, layout->weather_icon_pdc_sequence, frame,
                           GPoint(frame_rect.origin.x + WEATHER_APP_LAYOUT_ROUND_PDC_INSET,
                                  frame_rect.origin.y + WEATHER_APP_LAYOUT_ROUND_PDC_INSET));
}

static GRect prv_icon_background_frame_for_icon(GRect icon_frame) {
  const GSize sz = icon_frame.size;
  const int diam = (int)(sz.w * WEATHER_APP_LAYOUT_DISC_RATIO_NUM / 100);
  return (GRect){
    .origin = GPoint(icon_frame.origin.x + sz.w / 2 - diam / 2,
                     icon_frame.origin.y + sz.h / 2 - diam / 2),
    .size = GSize(diam, diam),
  };
}

static void prv_set_current_escape_icon_frame(WeatherAppLayout *layout,
                                              GRect content_icon_frame) {
  if (!layout->current_weather_escape_layer) return;
  GRect root_icon = content_icon_frame;
  root_icon.origin.x += layout->content_layer_origin.x;
  root_icon.origin.y += layout->content_layer_origin.y;
  layer_set_frame(layout->current_weather_escape_layer,
                  prv_icon_background_frame_for_icon(root_icon));
}

static void prv_hide_current_escape_icon(WeatherAppLayout *layout) {
  layout->anim_params.current_root_overlay = false;
  if (layout->current_weather_escape_layer) {
    layer_set_hidden(layout->current_weather_escape_layer, true);
  }
}

static void prv_draw_current_escape_icon(Layer *layer, GContext *ctx) {
  WeatherAppLayout *layout = *(WeatherAppLayout **)layer_get_data(layer);
  if (!layout || !layout->anim_params.current_root_overlay
      || !layout->weather_icon_pdc_sequence) {
    return;
  }
  GRect bounds = layer_get_bounds(layer);
  prv_draw_weather_background(&bounds, ctx,
      weather_type_disc_color(layout->anim_params.incoming_weather_type));

  GSize icon_size = layout->today_icon_rest_frame.size;
  GRect icon_rect = {
    GPoint((bounds.size.w - icon_size.w) / 2,
           (bounds.size.h - icon_size.h) / 2),
    icon_size
  };
  prv_draw_weather_pdc_frame(layout, ctx,
                             layout->anim_params.incoming_weather_type,
                             icon_rect);
}

static bool prv_weather_pdc_paused(const WeatherAppLayout *layout) {
  return layout->text_anim.active;
}

static void prv_draw_current_weather_pdc(const WeatherAppLayout *layout, GContext *ctx) {
  if (prv_weather_pdc_paused(layout)) return;
  prv_draw_weather_pdc_frame(layout, ctx, layout->forecast->current_weather_type,
                             layout->today_icon_rest_frame);
}

static void prv_set_current_weather_pdc(WeatherAppLayout *layout,
                                        const WeatherLocationForecast *today) {
  layer_set_hidden(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                   today && layout->weather_icon_pdc_sequence);
}
#endif

static void prv_draw_weather_icon_backgrounds(const WeatherAppLayout *layout,
                                              GContext *context) {
  bool animating = !layer_get_hidden(layout->outgoing_weather_icon_layer);

  if (animating) {
    if (!layout->anim_params.animate_down) {
      // UP only: the outgoing today icon lands in the BARE newspaper footer
      // slot, so the colored disc collapses into the art mid-flight (gone by
      // ~72% of the 50->25 shrink — the end-of-anim hide becomes visually a
      // no-op instead of a pop). NO white halo here: the descent crosses the
      // UV bar / description / footer, and an eraser disc bites white holes
      // out of them (the halo's rule-erasing job mattered on the EMPTY
      // classic page; the grey dots through the art's gaps for the last few
      // frames are the lesser evil on the furnished one).
      const GRect f = layer_get_frame(layout->outgoing_weather_icon_layer);
      const int w = f.size.w;                       // 50 -> 25 (moook rebound: 26)
      const GPoint c = GPoint(f.origin.x + w / 2, f.origin.y + f.size.h / 2);
      // Disc/icon ratio falls linearly RATIO -> 0 over w = <rest size> -> W0,
      // hard 0 below (zero-at-W0 also clamps the moook rebound's undershoot to
      // nothing). Derived from s_today_icon_size rather than a baked-in 50:
      //   diam(w) = RATIO*w*(w - W0) / (100 * (rest - W0))
      // On rect that is still exactly 7*w*(w-32)/90. It MUST be derived: with
      // the 50 hard-coded, round's 75px icon produced a 250px disc on the first
      // frame -- a full-screen colour flash before it collapsed.
#if PBL_ROUND
      const int diam = prv_round_scaler_disc(w);
#else
      const int w0 = WEATHER_APP_LAYOUT_DISC_COLLAPSE_W0;
      const int wrest = s_today_icon_size.w;
      const int diam = (w > w0 && wrest > w0)
          ? (WEATHER_APP_LAYOUT_DISC_RATIO_NUM * w * (w - w0) / (100 * (wrest - w0)))
          : 0;
#endif
      const GColor bg =
          weather_type_disc_color(layout->anim_params.outgoing_weather_type);
      if (diam >= 4 && !gcolor_equal(bg, GColorClear)) {   // skip sub-dot residue
        graphics_context_set_fill_color(context, bg);
        graphics_fill_circle(context, c, (uint16_t)(diam / 2));
      }
    } else {
#if PBL_ROUND
      // Mirror of the UP case: the incoming disc grows OUT of the footer circle.
      // prv_draw_circle_at_layer would size it from the frame at 1.11x, which
      // starts at 27px and visibly pops against the 35px circle it leaves.
      {
        const GRect gf = layer_get_frame(layout->outgoing_weather_icon_layer);
        const int gd = prv_round_scaler_disc(gf.size.w);
        graphics_context_set_fill_color(context,
            weather_type_get_bg_color(layout->anim_params.outgoing_weather_type));
        graphics_fill_circle(context,
                             GPoint(gf.origin.x + gf.size.w / 2, gf.origin.y + gf.size.h / 2),
                             (uint16_t)(gd / 2));
      }
#else
      prv_draw_circle_at_layer(layout->outgoing_weather_icon_layer,
                               context, layout->anim_params.outgoing_weather_type);
#endif
    }
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    if (layout->text_anim.active && !layout->outgoing_weather_icon) {
      prv_draw_weather_pdc_frame(layout, context,
                                 layout->anim_params.outgoing_weather_type,
                                 layer_get_frame(layout->outgoing_weather_icon_layer));
    }
#endif
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    if (!layout->anim_params.current_root_overlay) {
#else
    {
#endif
      prv_draw_circle_at_layer(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                               context, layout->anim_params.incoming_weather_type);
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
      if (layout->text_anim.active && layout->text_anim.progress > 0) {
        prv_draw_weather_pdc_frame(layout, context,
                                   layout->anim_params.incoming_weather_type,
                                   layer_get_frame(bitmap_layer_get_layer(
                                       layout->current_weather_icon_layer)));
      }
#endif
    }
    // Draw circle for the reparented/incoming tomorrow icon during UP/DOWN day animation.
    // The layer lives on root_layer so its frame is in root coordinates; convert to
    // content-layer coordinates so the circle is drawn correctly in this context.
    // This must be done here (after white text-animation fills) so it is not erased.
#if PBL_ROUND
    // ROUND: the footer icon's disc TRAVELS with it. The layer is reparented to
    // root_layer for the flight, so its frame is in ROOT coords -- convert back
    // to content coords for this context. Drawing the disc only in the static
    // branch below is what made it pop in after the icon had already settled.
    // Plain filled circle, no halo: the arc passes the dotted rule and a halo
    // would bite a white gap out of it (the same reason the resting disc is bare).
    if (layout->anim_params.tomorrow_reparented || layout->anim_params.tomorrow_incoming) {
      const GRect fr = layer_get_frame(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer));
      graphics_context_set_fill_color(context,
          weather_type_get_bg_color(layout->anim_params.tomorrow_exit_weather_type));
      graphics_fill_circle(context,
          GPoint(fr.origin.x - layout->content_layer_origin.x + fr.size.w / 2,
                 fr.origin.y - layout->content_layer_origin.y + fr.size.h / 2),
          WEATHER_APP_LAYOUT_ROUND_FOOT_DISC / 2);
    }
#endif
    // Rect: the 25px footer bitmap flies BARE — no disc AND no halo (the old
    // halo-only draw bit white holes out of the UV bar and text as it passed).

  } else {
    if (layout->forecast) {
      prv_draw_circle_at_layer(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                               context, layout->forecast->current_weather_type);
      // Glow ring removed on this screen — just the plain background disc behind the icon.
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
      prv_draw_current_weather_pdc(layout, context);
#endif
    }
    if (layout->next_forecast) {
#if PBL_ROUND
      // ROUND: the footer bitmap gets its own coloured disc. It canNOT use
      // DISC_RATIO_NUM -- that 1.11x is calibrated for the today icon, whose
      // 65px art sits in a 75px layer; the footer art fills its 25px box, so
      // 1.11x would leave a 27px disc barely wider than the icon. Fixed 35px.
      // NOT prv_draw_weather_background: that paints a WHITE halo 4px proud of
      // the disc before filling it, and up here the halo reached back over the
      // dotted rule and bit a white gap out of it. The footer disc is a plain
      // filled circle -- nothing above it needs erasing.
      GRect fi = layer_get_frame(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer));
      graphics_context_set_fill_color(context,
          weather_type_get_bg_color(layout->next_forecast->current_weather_type));
      graphics_fill_circle(context,
                           GPoint(fi.origin.x + fi.size.w / 2, fi.origin.y + fi.size.h / 2),
                           WEATHER_APP_LAYOUT_ROUND_FOOT_DISC / 2);
#endif
    }
  }
}

static void prv_render_layout(Layer *layer, GContext *context) {
  const GRect bounds = layer_get_bounds(layer);
  const GRect content_bounds =
      grect_inset(bounds, GEdgeInsets(0, WEATHER_APP_LAYOUT_CONTENT_LAYER_HORIZONTAL_INSET, 0));
  const int content_x_offset = content_bounds.origin.x;
  const int content_width = content_bounds.size.w;

  const WeatherAppLayout *layout = *(WeatherAppLayout **)layer_get_data(layer);
  const WeatherLocationForecast *forecast = layout->forecast;

  if (!forecast) {
    return;
  }

  // Separator fixed at 2/3 of the content layer height for a consistent today:tomorrow split
  const int separator_y = bounds.size.h * 2 / 3 + WEATHER_APP_LAYOUT_SEPARATOR_Y_ADJUST;
  const int full_h = bounds.size.h;

  if (layout->text_anim.active) {
    AnimationProgress p = layout->text_anim.progress;
    bool dir_down = layout->text_anim.dir_down;

    int out_dy = prv_interpolate_text_moook(p, 0, dir_down ? -full_h : full_h);
    int in_dy  = prv_interpolate_text_moook(p, dir_down ? full_h : -full_h, 0);

    // ---- TOP HALF (region 0 → separator_y) ----
    // Fill white first so it acts as a clip: any text drawn outside is invisible.
    graphics_context_set_fill_color(context, GColorWhite);
    graphics_fill_rect(context, GRect(0, 0, bounds.size.w, separator_y), 0, GCornerNone);
    GPoint top_out = GPoint(content_x_offset, 0 + out_dy);
    prv_draw_snapshot_top(layout, &top_out, content_width, context);
    GPoint top_in = GPoint(content_x_offset, 0 + in_dy);
    prv_draw_top_half_text(layout, &top_in, content_width, context);

    // ---- BOTTOM HALF (region separator_y → full_h) ----
    graphics_context_set_fill_color(context, GColorWhite);
    graphics_fill_rect(context, GRect(0, separator_y, bounds.size.w, full_h - separator_y),
                       0, GCornerNone);
    {
      // Clip the bottom-half passes to below the separator: during the moook bounce the
      // incoming/outgoing text overshoots above it, and the un-clipped bleed used to be
      // erased by re-drawing the entire top half a second time — twice the text-layout
      // work per frame on the hold-scroll hot path.
      GDrawState saved = context->draw_state;
      const int16_t sep_abs_y =
          (int16_t)(context->draw_state.drawing_box.origin.y + separator_y);
      GRect below = context->draw_state.clip_box;
      if (below.origin.y < sep_abs_y) {
        below.size.h = (int16_t)(below.size.h - (sep_abs_y - below.origin.y));
        if (below.size.h < 0) below.size.h = 0;
        below.origin.y = sep_abs_y;
      }
      context->draw_state.clip_box = below;
      if (layout->text_anim.bot_valid) {
        GPoint bot_out = GPoint(content_x_offset, separator_y + out_dy);
        prv_draw_snapshot_bot(layout, &bot_out, content_width, context);
      }
      if (layout->next_forecast) {
        GPoint bot_in = GPoint(content_x_offset, separator_y + in_dy);
        prv_draw_bottom_half_text(layout, &bot_in, content_width, context);
      }
      context->draw_state = saved;
    }

  } else {
    // Static (no text animation active)
    GPoint current_offset = GPoint(content_x_offset, 0);
    prv_draw_top_half_text(layout, &current_offset, content_width, context);

    if (layout->next_forecast) {
      current_offset = GPoint(content_x_offset, separator_y);
      prv_draw_bottom_half_text(layout, &current_offset, content_width, context);
    } else if (layout->forecast && layout->fin_pdc && !layout->icon_animation) {
      // Handled by the Timeline Fin bitmap layer.
    }
  }

  // Section rule after the sliding text (whose erase pass would wipe an early
  // draw) but BEFORE the discs — the white halo under each disc
  // (prv_draw_weather_background) then erases it locally, so crossing icons
  // always read as IN FRONT of the rule.
  // Divider (reference photo): dark gray, bigger dots — EXACTLY 45 of them
  // (counted from the reference by hand), EVERY gap identical (fixed integer
  // pitch, whole group centered — 200px isn't divisible by 45 so a true
  // edge-to-edge span would wobble; margins beat uneven internal spacing).
  // Darker to match the reference: DarkGray (#555, the only gray darker than
  // #AAA the hardware offers). 2x2 dots (the darker tone alone carries the
  // "bigger/heavier" read — the taller 3px version was reverted),
  // pitch 4 keeps the reference's ~1:1 dot:gap rhythm.
  graphics_context_set_fill_color(context, PBL_IF_COLOR_ELSE(GColorDarkGray, GColorBlack));
  // Round sits the rule 3px higher than the region split (and the UV widget moved with it).
  // The split itself is deliberately NOT moved: separator_y also anchors the bottom-half fill
  // and the footer's content offset, so shifting it would carry the TOMORROW block up too.
  const int divider_dot_y = separator_y - 1 - 3 - PBL_IF_ROUND_ELSE(3, 0);
  const int dot_w = 2;
  const int dot_h = 2;
  const int pitch = 4;   // dot + gap, IDENTICAL for every one of the 45 dots
  const int total_span = 44 * pitch + dot_w;
  const int start_x = (bounds.size.w - total_span) / 2;
  for (int i = 0; i < 45; i++) {
    graphics_fill_rect(context, GRect(start_x + i * pitch, divider_dot_y, dot_w, dot_h),
                       0, GCornerNone);
  }

  prv_draw_weather_icon_backgrounds(layout, context);

#if PBL_ROUND
  // Drawn LAST so it sits over the footer row and the icon rail. The condition
  // mirrors the fin's exactly (prv_fin_* draws only when !next_forecast), so
  // stepping onto the final day swaps the chevron out for the fin.
  if (layout->next_forecast) {
    const int chin_x = -layout->content_layer_origin.x;   // content coords -> screen 0
    graphics_context_set_fill_color(context, GColorLightGray);
    graphics_fill_rect(context,
                       GRect(chin_x, WEATHER_APP_LAYOUT_ROUND_CHIN_Y, PBL_DISPLAY_WIDTH,
                             PBL_DISPLAY_HEIGHT - WEATHER_APP_LAYOUT_ROUND_CHIN_Y),
                       0, GCornerNone);
    // Solid chevron, built a scanline at a time so it stays a crisp odd-width
    // triangle at any size (13,11,9,...,1) rather than an antialiased path.
    graphics_context_set_fill_color(context, GColorBlack);
    const int cx = PBL_DISPLAY_WIDTH / 2 + chin_x;
    for (int i = 0; i < WEATHER_APP_LAYOUT_ROUND_CHEV_H; i++) {
      const int half = WEATHER_APP_LAYOUT_ROUND_CHEV_W / 2 - i;
      if (half < 0) break;
      graphics_fill_rect(context,
                         GRect(cx - half, WEATHER_APP_LAYOUT_ROUND_CHEV_Y + i,
                               2 * half + 1, 1),
                         0, GCornerNone);
    }
  }
#endif
}

// ---- Arc animation ----

static GRect prv_icon_frame_at_angle(GPoint circle_center, int32_t radius,
                                      int32_t angle, GSize icon_size) {
  // Pebble trig: 0=up (12 o'clock), increases clockwise.
  // x = cx + sin(angle)*r,  y = cy - cos(angle)*r
  int16_t cx = circle_center.x +
      (int16_t)((int32_t)sin_lookup(angle) * radius / TRIG_MAX_RATIO);
  int16_t cy = circle_center.y -
      (int16_t)((int32_t)cos_lookup(angle) * radius / TRIG_MAX_RATIO);
  return (GRect){
    .origin = GPoint(cx - icon_size.w / 2, cy - icon_size.h / 2),
    .size = icon_size,
  };
}


// Lerp between two rects: centres and sizes interpolate together (p in 0..MAX).
static GRect prv_lerp_rect_center(GRect a, GRect b, int32_t p) {
  const int16_t a_cx = (int16_t)(a.origin.x + a.size.w / 2);
  const int16_t a_cy = (int16_t)(a.origin.y + a.size.h / 2);
  const int16_t b_cx = (int16_t)(b.origin.x + b.size.w / 2);
  const int16_t b_cy = (int16_t)(b.origin.y + b.size.h / 2);
  const int16_t cx = a_cx + (int16_t)((int32_t)(b_cx - a_cx) * p / ANIMATION_NORMALIZED_MAX);
  const int16_t cy = a_cy + (int16_t)((int32_t)(b_cy - a_cy) * p / ANIMATION_NORMALIZED_MAX);
  const int16_t sw = (int16_t)((int32_t)a.size.w +
      (int32_t)(b.size.w - a.size.w) * p / ANIMATION_NORMALIZED_MAX);
  const int16_t sh = (int16_t)((int32_t)a.size.h +
      (int32_t)(b.size.h - a.size.h) * p / ANIMATION_NORMALIZED_MAX);
  return (GRect){ .origin = GPoint(cx - sw / 2, cy - sh / 2), .size = GSize(sw, sh) };
}

// Lerp only the origin between two equal-size rects.
static GRect prv_lerp_rect_origin(GRect from, GRect to, int32_t p, GSize sz) {
  int16_t x = from.origin.x +
      (int16_t)((int32_t)(to.origin.x - from.origin.x) * p / ANIMATION_NORMALIZED_MAX);
  int16_t y = from.origin.y +
      (int16_t)((int32_t)(to.origin.y - from.origin.y) * p / ANIMATION_NORMALIZED_MAX);
  return (GRect){ GPoint(x, y), sz };
}

// The tomorrow rest frame translated into root-layer coordinates.
static GRect prv_tomorrow_rest_root(const WeatherAppLayout *layout) {
  return (GRect){
    GPoint(layout->content_layer_origin.x + layout->tomorrow_icon_rest_frame.origin.x,
           layout->content_layer_origin.y + layout->tomorrow_icon_rest_frame.origin.y),
    layout->tomorrow_icon_rest_frame.size
  };
}

static void prv_apply_day_transition_progress(WeatherAppLayout *layout,
                                              AnimationProgress progress) {
  layout->text_anim.progress = progress;
  const int32_t motion_p = layout->anim_params.animate_down
      ? prv_interpolate_icon_landing_progress_down(progress)
      : prv_interpolate_icon_landing_progress(progress);

  GPoint cc      = layout->anim_params.circle_center;
  int32_t r      = layout->anim_params.radius;
  GSize today_sz = layout->today_icon_rest_frame.size;

  GRect in_frame;
  GRect out_frame;

  if (layout->anim_params.animate_down) {
    // DOWN — roles swapped vs UP:
    //   outgoing_weather_icon_layer (scaler)      = INCOMING: scales up from tomorrow_rest
    //   current_weather_icon_layer  (BitmapLayer) = OUTGOING: sweeps along arc and exits

    // Incoming (scaler): straight-line from tomorrow_rest → today_rest while scaling up.
    // This is the exact reverse of UP’s outgoing shrink path — same motion, opposite direction.
    in_frame = prv_lerp_rect_center(layout->tomorrow_icon_rest_frame,
                                    layout->today_icon_rest_frame, motion_p);

    // Outgoing (BitmapLayer, old today): sweeps CW along arc at full size and exits.
    int32_t out_angle = layout->anim_params.outgoing_start_angle +
        weather_scale_i32(layout->anim_params.outgoing_end_angle -
                              layout->anim_params.outgoing_start_angle,
                          motion_p, ANIMATION_NORMALIZED_MAX);
    out_frame = prv_icon_frame_at_angle(cc, r, out_angle, today_sz);

    // Roles swapped: scaler gets in_frame, BitmapLayer gets out_frame.
    prv_set_outgoing_weather_icon_frame(layout, in_frame);
    layer_set_frame(bitmap_layer_get_layer(layout->current_weather_icon_layer), out_frame);
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    if (layout->anim_params.current_root_overlay) {
      prv_set_current_escape_icon_frame(layout, out_frame);
    }
#endif

    // Animate new tomorrow icon in from off-screen arc position → tomorrow rest slot.
    if (layout->anim_params.tomorrow_incoming) {
      GSize tmr_sz2 = layout->tomorrow_icon_rest_frame.size;
      GPoint cc_root = GPoint(
          layout->anim_params.circle_center.x + layout->content_layer_origin.x,
          layout->anim_params.circle_center.y + layout->content_layer_origin.y);
      GRect tmr_arc_start = prv_icon_frame_at_angle(cc_root, layout->anim_params.radius,
                                                     layout->anim_params.incoming_start_angle,
                                                     tmr_sz2);
      layer_set_frame(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer),
                      prv_lerp_rect_origin(tmr_arc_start, prv_tomorrow_rest_root(layout),
                                           motion_p, tmr_sz2));
    }
  } else {
    // UP: incoming (previous day) arrives full-size along arc, no scaling
    int32_t in_angle = layout->anim_params.incoming_start_angle +
        weather_scale_i32(ICON_ARC_TODAY_REST -
                              layout->anim_params.incoming_start_angle,
                          motion_p, ANIMATION_NORMALIZED_MAX);
    in_frame = prv_icon_frame_at_angle(cc, r, in_angle, today_sz);

    // Outgoing (old today) shrinks and slides straight to tomorrow slot
    out_frame = prv_lerp_rect_center(layout->today_icon_rest_frame,
                                     layout->tomorrow_icon_rest_frame, motion_p);

    prv_set_outgoing_weather_icon_frame(layout, out_frame);
    layer_set_frame(bitmap_layer_get_layer(layout->current_weather_icon_layer), in_frame);
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    if (layout->anim_params.current_root_overlay) {
      prv_set_current_escape_icon_frame(layout, in_frame);
    }
#endif

    // Also sweep the reparented tomorrow icon off-screen along the same arc as the
    // outgoing today icon — faster exit so it clears the screen early.
    if (layout->anim_params.tomorrow_reparented) {
      GSize tmr_sz2 = layout->tomorrow_icon_rest_frame.size;
      GPoint cc_root = GPoint(
          layout->anim_params.circle_center.x + layout->content_layer_origin.x,
          layout->anim_params.circle_center.y + layout->content_layer_origin.y);
      GRect tmr_end = prv_icon_frame_at_angle(cc_root, layout->anim_params.radius,
                                               layout->anim_params.outgoing_end_angle, tmr_sz2);

      // Faster exit: accelerate so it's off-screen well before animation ends.
      int32_t fast_p = motion_p * 5 / 3;
      if (fast_p > ANIMATION_NORMALIZED_MAX) fast_p = ANIMATION_NORMALIZED_MAX;

      layer_set_frame(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer),
                      prv_lerp_rect_origin(prv_tomorrow_rest_root(layout), tmr_end,
                                           fast_p, tmr_sz2));
    }
  }

  layer_mark_dirty(layout->root_layer);
}


// Destroy *slot, load the day/tiny icon for f (NULL f -> no bitmap), point bl at it.
// Unconditional: on round the CURRENT icon goes through the PDC path, but the
// tomorrow icon is still a bitmap and reloads through here.
static void prv_reload_icon(GBitmap **slot, BitmapLayer *bl,
                            const WeatherLocationForecast *f, bool tiny) {
  if (*slot) {
    gbitmap_destroy(*slot);
    *slot = NULL;
  }
  if (f) {
    *slot = gbitmap_create_with_resource(
        tiny ? weather_type_get_icon_res_tiny(f->current_weather_type)
             : weather_type_get_icon_res_today(f->current_weather_type));
  }
  bitmap_layer_set_bitmap(bl, *slot);
}

static void prv_icon_anim_update(Animation *anim, AnimationProgress progress) {
  WeatherAppLayout *layout = (WeatherAppLayout *)animation_get_context(anim);
  prv_apply_day_transition_progress(layout, progress);
}

static void prv_icon_anim_stopped(Animation *anim, bool finished, void *context) {
  WeatherAppLayout *layout = (WeatherAppLayout *)context;
  if (layout && layout->icon_animation) {
    layout->icon_animation = NULL;
    layout->text_anim.active = false;
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    prv_move_day_icons_to_content(layout);
    prv_hide_current_escape_icon(layout);
#endif

    if (layout->anim_params.animate_down) {
      // Layer roles were swapped during DOWN animation:
      //   outgoing_weather_icon_layer held the incoming (tiny scaler) bitmap.
      //   current_weather_icon_layer  held the outgoing (old today) bitmap.
      // Now reload current_weather_icon_layer with the new today icon so the
      // resting state is correct once we snap the frame.
#if !WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
      prv_reload_icon(&layout->current_weather_icon, layout->current_weather_icon_layer,
                      layout->forecast, false);
#endif
      // The tiny scaler bitmap is no longer needed.
      if (layout->outgoing_weather_icon) {
        gbitmap_destroy(layout->outgoing_weather_icon);
        layout->outgoing_weather_icon = NULL;
      }
    }
    // Snap incoming icon to exact today rest position
    layer_set_frame(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                    layout->today_icon_rest_frame);
    // Hide the outgoing (old today)
    layer_set_hidden(layout->outgoing_weather_icon_layer, true);
    // Reparent tomorrow icon back to content_layer if it was animated
    if (layout->anim_params.tomorrow_reparented || layout->anim_params.tomorrow_incoming) {
      bool was_reparented = layout->anim_params.tomorrow_reparented;
      Layer *tmr = bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer);
      layer_remove_from_parent(tmr);
      layer_add_child(layout->content_layer, tmr);
      layer_set_frame(tmr, layout->tomorrow_icon_rest_frame);
      layout->anim_params.tomorrow_reparented = false;
      layout->anim_params.tomorrow_incoming = false;
      // For exit (UP navigation), the old bitmap was kept alive during animation.
      // Now load the correct new tomorrow bitmap.
      if (was_reparented) {
        prv_reload_icon(&layout->tomorrow_weather_icon, layout->tomorrow_weather_icon_layer,
                        layout->next_forecast, true);
      }
    }
    // Restore the static tomorrow icon
    if (!WEATHER_APP_LAYOUT_RECT_SMALL) {   // small rect: footer is text-only
      layer_set_hidden(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer), false);
    }
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    prv_set_current_weather_pdc(layout, layout->forecast);
#endif
    layer_mark_dirty(layout->root_layer);

    // If there is no next forecast, the FIN was already started in
    // weather_app_layout_animate — nothing more to do here.
  }
  animation_destroy(anim);
}

static const AnimationImplementation s_icon_anim_impl = {
  .update = prv_icon_anim_update,
};

// ---- Right-swipe list transition animation ----
// Icons fly diagonally to their list-screen positions with squash-stretch.
// All text is hidden (transitioning_to_list flag gates prv_render_layout).
// Rows 0 and 1 positions match forecast_list.c's draw formula exactly so
// the instant window-push is invisible.

#define LIST_ROWS_VISIBLE     4
#define LIST_ICON_X           8



// Shared boilerplate: create + duration + linear curve + impl + stopped(ctx) + schedule.
static Animation *prv_start_anim(uint32_t dur_ms, const AnimationImplementation *impl,
                                 AnimationStoppedHandler stopped, void *ctx) {
  Animation *a = animation_create();
  animation_set_duration(a, dur_ms);
  animation_set_curve(a, AnimationCurveLinear);
  animation_set_implementation(a, impl);
  animation_set_handlers(a, (AnimationHandlers){ .stopped = stopped }, ctx);
  animation_schedule(a);
  return a;
}

// Move the Timeline Fin marker from the next scroll slot into its resting spot,
// matching Timeline's single frame animation rather than a delayed two-phase slide.
static void prv_animate_fin_in(WeatherAppLayout *layout, uint32_t total_ms) {
  if (!layout->fin_layer || !layout->fin_pdc) return;
  if (layout->fin_animation) {
    animation_unschedule(layout->fin_animation);
    layout->fin_animation = NULL;
  }

  Layer *fin = layout->fin_layer;
  GRect cl = layer_get_frame(layout->content_layer);
  GSize fin_size = gdraw_command_image_get_bounds_size(layout->fin_pdc);
  GRect to = (GRect){
    GPoint(cl.origin.x + (cl.size.w - fin_size.w) / 2,
           // Rect: rest in the footer zone — the PDC's ink (rows 12..32 of its
           // 50px box) vertically centred between the y195 divider and the
           // screen bottom; round keeps the classic centred rest.
           cl.origin.y + cl.size.h - fin_size.h + PBL_IF_RECT_ELSE(11, -16)),
    fin_size
  };
  GRect from = to;
  from.origin.y += cl.size.h / 3;

  layout->fin_anim.from = from;
  layout->fin_anim.to = to;
  layer_set_frame(fin, from);
  layer_set_hidden(fin, false);

  layout->fin_animation = prv_start_anim(total_ms, &s_fin_anim_impl,
                                         prv_fin_anim_stopped, layout);
}

// Per-pixel scaler for the outgoing icon, identical technique to the map app's zoom path.
// Walks the parent-layer chain to find screen-absolute coords, then captures the framebuffer
// and writes scaled pixels directly — the only way to truly scale a bitmap on Pebble.
static void prv_draw_bitmap_scaled_to_root(GContext *ctx, GBitmap *src, GRect root_frame) {
  if (!src) return;

  // Use layer_get_frame for BOTH position and animated size.
  // layer_get_bounds does not auto-update when layer_set_frame shrinks the layer,
  // so layer_get_frame is the only reliable source of the current animated size.
  int dst_w = root_frame.size.w;
  int dst_h = root_frame.size.h;
  if (dst_w <= 0 || dst_h <= 0) return;

  GRect src_bounds = gbitmap_get_bounds(src);
  int src_w = src_bounds.size.w;
  int src_h = src_bounds.size.h;
  if (src_w <= 0 || src_h <= 0) return;

  uint8_t *sdata = gbitmap_get_data(src);
  uint16_t sbpr  = gbitmap_get_bytes_per_row(src);

  // Pebble tools compile small-colour PNGs as 4-bit or 2-bit palette bitmaps,
  // not GBitmapFormat8Bit. We must look up each pixel's palette entry to get
  // the actual GColor8 ARGB byte — otherwise we read raw palette indices and
  // get garbage colours (including the grey silhouette this bug caused).
  GBitmapFormat fmt = gbitmap_get_format(src);
  GColor *palette   = gbitmap_get_palette(src);  // NULL for non-palette formats

  GBitmap *fb = graphics_capture_frame_buffer(ctx);
  if (!fb) return;
  GRect fbb = gbitmap_get_bounds(fb);
  int fb_w = fbb.size.w;
  int fb_h = fbb.size.h;

  int32_t sx_step = ((int32_t)src_w << 16) / dst_w;
  int32_t sy_step = ((int32_t)src_h << 16) / dst_h;

  int32_t sy_fp = sy_step >> 1;
  for (int dy = 0; dy < dst_h; dy++, sy_fp += sy_step) {
    int sy = sy_fp >> 16;
    if (sy >= src_h) sy = src_h - 1;
    int ay = root_frame.origin.y + dy;
    if (ay < 0 || ay >= fb_h) continue;

    uint8_t *srow = sdata + (uint32_t)sy * sbpr;
    GBitmapDataRowInfo row = gbitmap_get_data_row_info(fb, ay);

    int32_t sx_fp2 = sx_step >> 1;
    for (int dx = 0; dx < dst_w; dx++, sx_fp2 += sx_step) {
      int sx = sx_fp2 >> 16;
      if (sx >= src_w) sx = src_w - 1;
      int ax = root_frame.origin.x + dx;
      if (ax < 0 || ax >= fb_w) continue;
      if (ax < row.min_x || ax > row.max_x) continue;

      uint8_t pixel;
      if (fmt == GBitmapFormat8Bit || fmt == GBitmapFormat8BitCircular) {
        // Direct 8-bit ARGB — one byte per pixel
        pixel = srow[sx];
      } else if (fmt == GBitmapFormat4BitPalette) {
        // Two pixels packed per byte; high nibble = left (even) pixel
        uint8_t raw = srow[sx >> 1];
        uint8_t idx = (sx & 1) ? (raw & 0xF) : (raw >> 4);
        pixel = palette ? palette[idx].argb : 0;
      } else if (fmt == GBitmapFormat2BitPalette) {
        // Four pixels packed per byte; bits 7:6 = leftmost pixel
        uint8_t raw = srow[sx >> 2];
        uint8_t idx = (raw >> (6 - ((sx & 3) << 1))) & 0x3;
        pixel = palette ? palette[idx].argb : 0;
      } else if (fmt == GBitmapFormat1BitPalette) {
        // Eight pixels per byte; bit 7 = leftmost pixel
        uint8_t raw = srow[sx >> 3];
        uint8_t idx = (raw >> (7 - (sx & 7))) & 0x1;
        pixel = palette ? palette[idx].argb : 0;
      } else {
        // GBitmapFormat1Bit: set bit = opaque white
        uint8_t raw = srow[sx >> 3];
        pixel = ((raw >> (7 - (sx & 7))) & 0x1) ? 0xFF : 0x00;
      }

      // Top 2 bits are the alpha channel; 0b00 = transparent, skip it
      if (pixel >> 6) weather_fb_row_set(row.data, ax, pixel);
    }
  }

  graphics_release_frame_buffer(ctx, fb);
}

static void prv_draw_outgoing_icon_scaled(Layer *layer, GContext *ctx) {
  WeatherAppLayout *layout = *(WeatherAppLayout **)layer_get_data(layer);
  GRect root_frame = layer_get_frame(layer);
  root_frame.origin.x += layout->content_layer_origin.x;
  root_frame.origin.y += layout->content_layer_origin.y;
#if PBL_ROUND
  // The RESTING today icon is the PDC drawn at ROUND_PDC_ART inside a 75px layer,
  // but this scaler fills the layer frame -- so on the DOWN entrance the bitmap
  // grew all the way to 75 and then visibly snapped back to the art size the
  // instant the static render took over. Inset the ART so it lands exactly where
  // the PDC will draw it.
  //
  // The art size LERPS WITH THE FRAME WIDTH (mirror of prv_round_scaler_disc, and
  // for the same reason): at the today end (w=75) it is ROUND_PDC_ART, matching
  // the resting PDC; at the footer end (w=25) it is the FULL frame, matching the
  // bare 25px footer bitmap that takes over there. A fixed ART/NATIVE fraction
  // landed 25*ART/75 art on the footer (21px at ART=65), which then popped to
  // 25 at the handoff instant.
  //
  // The FRAME is deliberately left alone: the coloured disc is derived from it
  // (prv_draw_circle_at_layer), so shrinking the frame would shrink the disc too
  // and just move the snap from the icon onto the circle.
  const int w0 = s_tomorrow_icon_size.w;                // 25: art fills the frame
  const int w1 = s_today_icon_size.w;                   // 75: art is ROUND_PDC_ART
  int art_w;
  if (root_frame.size.w <= w0 || w1 <= w0) {
    art_w = root_frame.size.w;
  } else if (root_frame.size.w >= w1) {
    art_w = WEATHER_APP_LAYOUT_ROUND_PDC_ART;
  } else {
    art_w = w0 + (WEATHER_APP_LAYOUT_ROUND_PDC_ART - w0) *
                     (root_frame.size.w - w0) / (w1 - w0);
  }
  const int art_h = art_w;   // the flip's frames are square (75x75 <-> 25x25)
  root_frame.origin.x += (root_frame.size.w - art_w) / 2;
  root_frame.origin.y += (root_frame.size.h - art_h) / 2;
  root_frame.size = GSize(art_w, art_h);
#endif
  prv_draw_bitmap_scaled_to_root(ctx, layout->outgoing_weather_icon, root_frame);
}

static GRect prv_fin_rest_frame(WeatherAppLayout *layout) {
  GRect cl = layer_get_frame(layout->content_layer);
  GSize fin_size = layout->fin_pdc ? gdraw_command_image_get_bounds_size(layout->fin_pdc)
                                      : GSize(0, 0);
  return (GRect){
    GPoint(cl.origin.x + (cl.size.w - fin_size.w) / 2,
           // Rect: rest in the footer zone — the PDC's ink (rows 12..32 of its
           // 50px box) vertically centred between the y195 divider and the
           // screen bottom; round keeps the classic centred rest.
           cl.origin.y + cl.size.h - fin_size.h + PBL_IF_RECT_ELSE(11, -16)),
    fin_size
  };
}

static void prv_restore_fin_rest(WeatherAppLayout *layout) {
  if (!layout->fin_layer || !layout->fin_pdc) return;
  if (layout->fin_animation) {
    animation_unschedule(layout->fin_animation);
    layout->fin_animation = NULL;
  }
  if (layout->next_forecast || !layout->fin_allowed) {
    layer_set_hidden(layout->fin_layer, true);
    return;
  }
  layer_set_frame(layout->fin_layer, prv_fin_rest_frame(layout));
  layer_set_hidden(layout->fin_layer, false);
}

void weather_app_layout_init(WeatherAppLayout *layout, const GRect *frame) {
#if PBL_DISPLAY_HEIGHT >= 200
  // Rect = the ideal/a14 hierarchy (the original app's, scaled for emery):
  // G24B day label, LECO_36 hero, G24B hi/lo, G18 phrase (metrics_value_font
  // doubles as the phrase face on rect), G14B metrics, G18B next-day label.
  // Day name: heavier on rect (28_BOLD) to match the reference's chunky caps;
  // the flow advance is pinned to 24_BOLD in prv_draw_top_rows so the stack
  // below is undisturbed. Round unchanged (28_BOLD).
  layout->location_font        = fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD);
  layout->temperature_font     = fonts_get_system_font(FONT_KEY_LECO_36_BOLD_NUMBERS);
  layout->high_low_phrase_font = fonts_get_system_font(
      PBL_IF_RECT_ELSE(WEATHER_APP_LAYOUT_RECT_SMALL ? FONT_KEY_GOTHIC_18_BOLD
                                                     : FONT_KEY_GOTHIC_24_BOLD,
                       FONT_KEY_GOTHIC_18_BOLD));
  layout->metrics_font         = fonts_get_system_font(
      FONT_KEY_GOTHIC_14_BOLD);
  layout->metrics_value_font   = fonts_get_system_font(
      FONT_KEY_GOTHIC_18);
  layout->tomorrow_font        = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
#else
  layout->location_font        = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
  layout->metrics_value_font   = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
  layout->temperature_font     = fonts_get_system_font(FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM);
  layout->high_low_phrase_font = fonts_get_system_font(FONT_KEY_GOTHIC_18);
  layout->metrics_font         = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
  layout->tomorrow_font        = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
#endif

  layout->fin_pdc = NULL;
  layout->fin_layer = NULL;
  layout->fin_animation = NULL;
  // Real timeline 'fin' flag (END_OF_TIMELINE PDC, from Pebble_50x50_Fin.svg).
  // A system-app PDC is mmap'd READ-ONLY in flash, so clone it into a writable
  // heap copy before any draw (same trap as the weather-icons sequence).
  GDrawCommandImage *fin_raw =
      gdraw_command_image_create_with_resource(RESOURCE_ID_END_OF_TIMELINE);
  layout->fin_pdc = fin_raw ? gdraw_command_image_clone(fin_raw) : NULL;
  if (fin_raw) {
    gdraw_command_image_destroy(fin_raw);  // munmaps the read-only flash mapping
  }
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  layout->weather_icon_pdc_sequence = NULL;
  layout->current_weather_escape_layer = NULL;
  layout->anim_params.current_root_overlay = false;
#endif

  layout->root_layer = layer_create_with_data(*frame, sizeof(WeatherAppLayout *));
  *(WeatherAppLayout **)layer_get_data(layout->root_layer) = layout;

  // Location bar: full-width strip at the very top of the screen.
  // Taller than the forecast-list bar (MAIN_BAR_HEIGHT vs LOCATION_BAR_HEIGHT)
  // for better readability on the main screen.
  layout->city_layer = NULL;
  layout->location_bar_layer = NULL;   // no bar on either shape -- content owns
                                       // the whole screen (round joined rect
                                       // in the report-page port)
  // Added to root_layer last (after fin_layer) so it renders on top of everything.

  // Down-arrow removed — location bar provides sufficient bottom UI.
  layout->down_arrow_layer = NULL;

  // Content layer fills the root below the location bar.
  const int content_layer_side_padding = PBL_IF_RECT_ELSE(5, 12);
  GRect content_layer_frame = grect_inset(
      *frame, GEdgeInsets(WEATHER_APP_LAYOUT_MAIN_CONTENT_TOP,
                          content_layer_side_padding,
                          -WEATHER_APP_LAYOUT_CONTENT_Y_SHIFT));
  content_layer_frame.origin.y += WEATHER_APP_LAYOUT_CONTENT_Y_SHIFT;

  layout->content_layer = layer_create_with_data(content_layer_frame, sizeof(WeatherAppLayout *));
  WeatherAppLayout **stored = layer_get_data(layout->content_layer);
  *stored = layout;
  layer_set_update_proc(layout->content_layer, prv_render_layout);
  // Don't clip children/drawing to the content frame: the day-scroll arc and
  // the vertical text slide must run to the REAL screen edges. Everything
  // that overshoots the top passes UNDER the opaque location bar (added
  // after us, so it draws on top); the framebuffer bounds clip the rest.
  // With clipping on, the icons visibly vanished at the content boundary
  // 30px below the top edge instead of sliding off-screen.
  layer_set_clips(layout->content_layer, false);
  layer_add_child(layout->root_layer, layout->content_layer);

  // Store the screen-absolute origin of the content layer for the per-pixel scaler.
  // root_layer is at *frame.origin (0,0 for fullscreen), content_layer is inset from that.
  layout->content_layer_origin = GPoint(
      frame->origin.x + content_layer_frame.origin.x,
      frame->origin.y + content_layer_frame.origin.y);

  const int content_separator_y = content_layer_frame.size.h * 2 / 3 +
                                  WEATHER_APP_LAYOUT_SEPARATOR_Y_ADJUST;

  GRect today_icon_frame = (GRect){
    // Rect: right-railed like the original app — the disc + icon own the right
    // side while the text stack flows down the left rail.
    .origin = PBL_IF_RECT_ELSE(
        // Disc top rides 5px ABOVE the hero temp's cap line (temp ink top 39
        // after the −3 trim, disc top 34 -> icon y 44), right margin 9px —
        // calibrated relationship, kept through moves;
        // later raised 8px more -> y 36.
        GPoint(content_layer_frame.size.w - s_today_icon_size.w -
               (WEATHER_APP_LAYOUT_CONTENT_LAYER_HORIZONTAL_INSET + 11), 36),
        // Round: 75px PDC layer, right-railed. The 52px accent disc centres on
        // this layer -> screen (197.5,69.5); its r=30 halo's furthest point is
        // 120.3px from the glass centre, contained with ~10px to spare.
        GPoint(content_layer_frame.size.w - s_today_icon_size.w -
                   WEATHER_APP_LAYOUT_ROUND_TODAY_ICON_X_INSET,
               WEATHER_APP_LAYOUT_ROUND_TODAY_ICON_Y)),
    .size = s_today_icon_size,
  };
  layout->today_icon_rest_frame = today_icon_frame;
  layout->current_weather_icon_layer = bitmap_layer_create(today_icon_frame);
  bitmap_layer_set_compositing_mode(layout->current_weather_icon_layer, GCompOpSet);
  // current_weather_icon_layer carries the incoming (new today) — centred so
  // it looks like a grow/zoom as the frame expands during the animation
  bitmap_layer_set_alignment(layout->current_weather_icon_layer, GAlignCenter);
  layer_add_child(layout->content_layer,
                  bitmap_layer_get_layer(layout->current_weather_icon_layer));

  // Outgoing icon layer: plain Layer with per-pixel framebuffer scaler.
  // This is the same technique the map app uses for zoom — it's the only way
  // to truly scale (not crop) a bitmap on Pebble hardware.
  layout->outgoing_weather_icon_layer =
      layer_create_with_data(today_icon_frame, sizeof(WeatherAppLayout *));
  *(WeatherAppLayout **)layer_get_data(layout->outgoing_weather_icon_layer) = layout;
  layer_set_update_proc(layout->outgoing_weather_icon_layer, prv_draw_outgoing_icon_scaled);
  layer_set_hidden(layout->outgoing_weather_icon_layer, true);
  layer_add_child(layout->content_layer, layout->outgoing_weather_icon_layer);

  GRect tomorrow_icon_frame = (GRect){
    // Rect: right-railed under the today disc, vertically on the next-day block.
    .origin = PBL_IF_RECT_ELSE(
        GPoint(content_layer_frame.size.w - s_tomorrow_icon_size.w -
               WEATHER_APP_LAYOUT_CONTENT_LAYER_HORIZONTAL_INSET,
               content_separator_y + 3),
        // Round: the bare 25px footer bitmap. Its BOTTOM-right corner is the
        // tightest margin on the page (x213 vs mask max_x 215 at row 228) --
        // check that corner first on any screendump.
        GPoint(content_layer_frame.size.w - s_tomorrow_icon_size.w -
                   WEATHER_APP_LAYOUT_ROUND_TOMORROW_ICON_X_INSET,
               content_separator_y + 4)),   // disc centre (193,217), r17 -> rows 200..234
    .size = s_tomorrow_icon_size,
  };
  layout->tomorrow_icon_rest_frame = tomorrow_icon_frame;
  layout->tomorrow_weather_icon_layer = bitmap_layer_create(tomorrow_icon_frame);
  bitmap_layer_set_compositing_mode(layout->tomorrow_weather_icon_layer, GCompOpSet);
  layer_add_child(layout->content_layer,
                  bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer));
#if WEATHER_APP_LAYOUT_RECT_SMALL
  // The footer is text-only on the small rects (the icon cannot fit under the
  // rule); the layer stays for the shared animation plumbing, just hidden.
  layer_set_hidden(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer), true);
#endif

#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  // A system app's PDC resource is mmap'd directly into READ-ONLY flash (no RAM
  // copy), so clone it into a writable heap copy — otherwise any write to the
  // sequence (e.g. gdraw_command_sequence_set_bounds_size -> sequence->size = ...)
  // stores into read-only flash and faults the app on launch. (The globe uses the
  // same clone-into-RAM pattern for its writable sequence.)
  GDrawCommandSequence *pdc_raw =
      gdraw_command_sequence_create_with_resource(RESOURCE_ID_WEATHER_ICONS_PDC);
  layout->weather_icon_pdc_sequence =
      pdc_raw ? gdraw_command_sequence_clone(pdc_raw) : NULL;
  if (pdc_raw) {
    gdraw_command_sequence_destroy(pdc_raw);  // munmaps the read-only flash mapping
  }
  // Shrink the art inside its unchanged 75px layer. gdraw_command_list_scale
  // scales toward the list's origin, so the draw origin is nudged by
  // ROUND_PDC_INSET to keep it centred (see prv_draw_weather_pdc_frame).
  if (layout->weather_icon_pdc_sequence &&
      WEATHER_APP_LAYOUT_ROUND_PDC_ART != WEATHER_APP_LAYOUT_ROUND_PDC_NATIVE) {
    const int nframes =
        gdraw_command_sequence_get_num_frames(layout->weather_icon_pdc_sequence);
    for (int i = 0; i < nframes; i++) {
      GDrawCommandFrame *f =
          gdraw_command_sequence_get_frame_by_index(layout->weather_icon_pdc_sequence, i);
      if (!f) continue;
      gdraw_command_list_scale(gdraw_command_frame_get_command_list(f),
                               GSize(WEATHER_APP_LAYOUT_ROUND_PDC_NATIVE,
                                     WEATHER_APP_LAYOUT_ROUND_PDC_NATIVE),
                               GSize(WEATHER_APP_LAYOUT_ROUND_PDC_ART,
                                     WEATHER_APP_LAYOUT_ROUND_PDC_ART));
    }
  }
  layout->current_weather_escape_layer =
      layer_create_with_data(GRect(0, 0, 1, 1), sizeof(WeatherAppLayout *));
  if (layout->current_weather_escape_layer) {
    *(WeatherAppLayout **)layer_get_data(layout->current_weather_escape_layer) = layout;
    layer_set_update_proc(layout->current_weather_escape_layer,
                          prv_draw_current_escape_icon);
    layer_set_hidden(layout->current_weather_escape_layer, true);
    layer_add_child(layout->root_layer, layout->current_weather_escape_layer);
  }
#endif

  // Timeline Fin marker: child of root_layer so it can animate like the system timeline.
  if (layout->fin_pdc) {
    GRect fin_offscreen = (GRect){GPoint(0, frame->size.h),
                                  gdraw_command_image_get_bounds_size(layout->fin_pdc)};
    layout->fin_layer = layer_create_with_data(fin_offscreen, sizeof(WeatherAppLayout *));
    *(WeatherAppLayout **)layer_get_data(layout->fin_layer) = layout;
    layer_set_update_proc(layout->fin_layer, prv_draw_fin_layer);
    layer_set_hidden(layout->fin_layer, true);
    layer_add_child(layout->root_layer, layout->fin_layer);
  }
}

void weather_app_layout_set_data(WeatherAppLayout *layout,
                                 const WeatherLocationForecast *today_forecast,
                                 const WeatherLocationForecast *next_forecast) {
  layout->forecast = today_forecast;
  layout->next_forecast = next_forecast;

  if (layout->current_weather_icon) {
    gbitmap_destroy(layout->current_weather_icon);
    layout->current_weather_icon = NULL;
  }
  if (layout->tomorrow_weather_icon) {
    gbitmap_destroy(layout->tomorrow_weather_icon);
    layout->tomorrow_weather_icon = NULL;
  }

#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  bitmap_layer_set_bitmap(layout->current_weather_icon_layer, NULL);
  layer_set_frame(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                  layout->today_icon_rest_frame);
#else
  if (today_forecast) {
    layout->current_weather_icon = gbitmap_create_with_resource(
        weather_type_get_icon_res_today(today_forecast->current_weather_type));
    bitmap_layer_set_bitmap(layout->current_weather_icon_layer, layout->current_weather_icon);
    layer_set_frame(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                    layout->today_icon_rest_frame);
  } else {
    bitmap_layer_set_bitmap(layout->current_weather_icon_layer, NULL);
  }
#endif

  if (next_forecast) {
    layout->tomorrow_weather_icon = gbitmap_create_with_resource(
        weather_type_get_icon_res_tiny(next_forecast->current_weather_type));
    bitmap_layer_set_bitmap(layout->tomorrow_weather_icon_layer, layout->tomorrow_weather_icon);
  } else {
    bitmap_layer_set_bitmap(layout->tomorrow_weather_icon_layer, NULL);
  }

#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  prv_set_current_weather_pdc(layout, today_forecast);
#endif

  prv_restore_fin_rest(layout);
  layer_mark_dirty(layout->root_layer);
}

void weather_app_layout_set_fin_allowed(WeatherAppLayout *layout,
                                        bool fin_allowed) {
  if (!layout) return;
  layout->fin_allowed = fin_allowed;
  prv_restore_fin_rest(layout);
}

void weather_app_layout_deinit(WeatherAppLayout *layout) {
  if (layout->icon_animation) {
    Animation *anim = layout->icon_animation;
    layout->icon_animation = NULL;
    animation_unschedule(anim);
  }
  if (layout->fin_animation) {
    animation_unschedule(layout->fin_animation);
    layout->fin_animation = NULL;
  }
  if (layout->current_weather_icon) {
    gbitmap_destroy(layout->current_weather_icon);
  }
  if (layout->outgoing_weather_icon) {
    gbitmap_destroy(layout->outgoing_weather_icon);
  }
  if (layout->tomorrow_weather_icon) {
    gbitmap_destroy(layout->tomorrow_weather_icon);
  }
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  if (layout->weather_icon_pdc_sequence) {
    gdraw_command_sequence_destroy(layout->weather_icon_pdc_sequence);
  }
  if (layout->current_weather_escape_layer) {
    layer_destroy(layout->current_weather_escape_layer);
  }
#endif
  if (layout->fin_pdc) gdraw_command_image_destroy(layout->fin_pdc);
  if (layout->fin_layer) layer_destroy(layout->fin_layer);
  if (layout->city_layer) layer_destroy(layout->city_layer);
  bitmap_layer_destroy(layout->current_weather_icon_layer);
  layer_destroy(layout->outgoing_weather_icon_layer);
  bitmap_layer_destroy(layout->tomorrow_weather_icon_layer);
  if (layout->location_bar_layer) layer_destroy(layout->location_bar_layer);
  layer_destroy(layout->content_layer);
  layer_destroy(layout->root_layer);
}

void weather_app_layout_set_location(WeatherAppLayout *layout, const char *name) {
  if (!name) return;
  strncpy(layout->location_name, name, sizeof(layout->location_name) - 1);
  layout->location_name[sizeof(layout->location_name) - 1] = '\0';
#if PBL_ROUND
  if (layout->city_layer) layer_mark_dirty(layout->city_layer);
#endif
}

static bool prv_prepare_day_transition(WeatherAppLayout *layout,
                                       const WeatherLocationForecast *new_today,
                                       const WeatherLocationForecast *new_next,
                                       bool animate_down) {
  // Cancel any running animation
  if (layout->icon_animation) {
    Animation *old = layout->icon_animation;
    layout->icon_animation = NULL;
    bool was_down = layout->anim_params.animate_down;
    if (layout->anim_params.tomorrow_reparented || layout->anim_params.tomorrow_incoming) {
      Layer *tmr = bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer);
      layer_remove_from_parent(tmr);
      layer_add_child(layout->content_layer, tmr);
      layer_set_frame(tmr, layout->tomorrow_icon_rest_frame);
      layout->anim_params.tomorrow_reparented = false;
      layout->anim_params.tomorrow_incoming = false;
    }
    animation_unschedule(old);
    layer_set_hidden(layout->outgoing_weather_icon_layer, true);
    if (!WEATHER_APP_LAYOUT_RECT_SMALL) {   // small rect: footer is text-only
      layer_set_hidden(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer), false);
    }
    layer_set_frame(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                    layout->today_icon_rest_frame);
    // For a cancelled DOWN animation the layer roles were swapped: current_weather_icon_layer
    // still holds the OLD today bitmap while layout->forecast was already advanced.
    // Reload it now so the next animation's outgoing icon shows the correct (current) day.
    if (was_down) {
#if !WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
      prv_reload_icon(&layout->current_weather_icon, layout->current_weather_icon_layer,
                      layout->forecast, false);
#endif
    }
  }
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  prv_hide_current_escape_icon(layout);
#endif

  // Save weather types for background circle drawing during animation.
  // For animate_down the layer roles are swapped (scaler = incoming, BitmapLayer = outgoing),
  // so swap the type assignments too so the circles get the right colour.
  if (animate_down) {
    layout->anim_params.outgoing_weather_type =
        new_today ? new_today->current_weather_type : WeatherType_Unknown;
    layout->anim_params.incoming_weather_type =
        layout->forecast ? layout->forecast->current_weather_type : WeatherType_Unknown;
  } else {
    layout->anim_params.outgoing_weather_type =
        layout->forecast ? layout->forecast->current_weather_type : WeatherType_Unknown;
    layout->anim_params.incoming_weather_type =
        new_today ? new_today->current_weather_type : WeatherType_Unknown;
  }
  // Capture tomorrow weather type before layout->next_forecast pointer is updated.
  // For DOWN (entering next day), use new_next (the day that will become tomorrow).
  // For UP (returning), use the current tomorrow that's about to exit.
  layout->anim_params.tomorrow_exit_weather_type =
      (animate_down && new_next) ? new_next->current_weather_type :
      (layout->next_forecast ? layout->next_forecast->current_weather_type : WeatherType_Unknown);

  // ---- Layer bitmap setup ----
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  layer_set_hidden(bitmap_layer_get_layer(layout->current_weather_icon_layer), true);
#endif
  if (animate_down) {
    // DOWN: scaler layer (outgoing_weather_icon_layer) = INCOMING tiny icon that scales up.
    //       BitmapLayer (current_weather_icon_layer)   = OUTGOING old today at full size.
    // Load the tiny resource of new_today into the scaler; start it at the tomorrow slot.
    if (layout->outgoing_weather_icon) gbitmap_destroy(layout->outgoing_weather_icon);
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    bitmap_layer_set_bitmap(layout->current_weather_icon_layer, NULL);
#endif
    layout->outgoing_weather_icon = new_today ?
        gbitmap_create_with_resource(
            weather_type_get_icon_res_today(new_today->current_weather_type)) : NULL;
    // Frame set to arc start position below, after circle_center is computed.
    layer_set_hidden(layout->outgoing_weather_icon_layer, false);
    // current_weather_icon_layer already holds the old today bitmap — keep it as-is.
  } else {
    // UP: scaler layer = OUTGOING old today (shrinks + slides to tomorrow slot).
    //     BitmapLayer  = INCOMING new today (previous day, full size, sweeps along arc).
    if (layout->outgoing_weather_icon) gbitmap_destroy(layout->outgoing_weather_icon);
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    layout->outgoing_weather_icon = layout->forecast ?
        gbitmap_create_with_resource(
            weather_type_get_icon_res_today(layout->forecast->current_weather_type)) : NULL;
#else
    layout->outgoing_weather_icon = layout->current_weather_icon;
    layout->current_weather_icon = NULL;
#endif
    layer_set_frame(layout->outgoing_weather_icon_layer, layout->today_icon_rest_frame);
    layer_set_hidden(layout->outgoing_weather_icon_layer, false);

    if (layout->current_weather_icon) {
      gbitmap_destroy(layout->current_weather_icon);
      layout->current_weather_icon = NULL;
    }
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    bitmap_layer_set_bitmap(layout->current_weather_icon_layer, NULL);
#else
    prv_reload_icon(&layout->current_weather_icon, layout->current_weather_icon_layer,
                    new_today, false);
#endif
  }

  // Reparent tomorrow icon to root_layer for any arc animation (exit or entrance).
  // UP (going back):   reparent and animate it OUT along the arc.
  // DOWN (next day):   reparent and animate it IN from the arc if new_next exists.
  // Otherwise:         hide it immediately.
  // Small rect: the footer is text-only — the icon never joins the arc.
  if (!WEATHER_APP_LAYOUT_RECT_SMALL && !animate_down && layout->next_forecast) {
    Layer *tmr = bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer);
    GRect tmr_root = (GRect){
      GPoint(layout->content_layer_origin.x + layout->tomorrow_icon_rest_frame.origin.x,
             layout->content_layer_origin.y + layout->tomorrow_icon_rest_frame.origin.y),
      layout->tomorrow_icon_rest_frame.size
    };
    layer_set_frame(tmr, tmr_root);
    layer_remove_from_parent(tmr);
    layer_add_child(layout->root_layer, tmr);
    layer_set_hidden(tmr, false);
    layout->anim_params.tomorrow_reparented = true;
    layout->anim_params.tomorrow_incoming = false;
  } else if (!WEATHER_APP_LAYOUT_RECT_SMALL && animate_down && new_next) {
    // Entrance: reparent but start frame will be set after circle_center is computed below.
    Layer *tmr = bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer);
    layer_remove_from_parent(tmr);
    layer_add_child(layout->root_layer, tmr);
    layer_set_hidden(tmr, false);
    layout->anim_params.tomorrow_reparented = false;
    layout->anim_params.tomorrow_incoming = true;
  } else {
    layer_set_hidden(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer), true);
    layout->anim_params.tomorrow_reparented = false;
    layout->anim_params.tomorrow_incoming = false;
  }
  // Hide and reset the FIN layer whenever a new animation starts.
  if (layout->fin_layer) {
    layer_set_hidden(layout->fin_layer, true);
  }

  // Pre-load the new tomorrow bitmap for after the animation.
  // Exception: when the tomorrow layer is animated OUT (reparented for UP navigation),
  // keep the existing bitmap alive so the exit animation shows the correct icon.
  // The bitmap is reloaded in prv_icon_anim_stopped after the animation ends.
  if (!layout->anim_params.tomorrow_reparented) {
#if !WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    prv_reload_icon(&layout->tomorrow_weather_icon, layout->tomorrow_weather_icon_layer,
                    new_next, true);
#else
    if (layout->tomorrow_weather_icon) {
      gbitmap_destroy(layout->tomorrow_weather_icon);
      layout->tomorrow_weather_icon = NULL;
    }
    if (new_next) {
      layout->tomorrow_weather_icon = gbitmap_create_with_resource(
          weather_type_get_icon_res_tiny(new_next->current_weather_type));
      bitmap_layer_set_bitmap(layout->tomorrow_weather_icon_layer, layout->tomorrow_weather_icon);
    } else {
      bitmap_layer_set_bitmap(layout->tomorrow_weather_icon_layer, NULL);
    }
#endif
  }

  // Snapshot outgoing text strings before forecast pointer is updated.
  prv_snapshot_text(layout);
  layout->text_anim.dir_down = animate_down;
  layout->text_anim.progress = 0;
  layout->text_anim.active   = true;
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  prv_move_day_icons_to_root(layout);
#endif

  // Update forecast pointers — text redraws immediately with new data
  layout->forecast = new_today;
  layout->next_forecast = new_next;

  // ---- Arc geometry ----
  // Circle centre is off-screen to the RIGHT.
  // At 9 o'clock (TODAY_REST = 3*MAX/4) icon sits left of centre = on-screen.
  // DOWN scroll: both icons sweep CW (increasing angle).
  //   Outgoing:  TODAY_REST  →  TODAY_REST + SWEEP  (exits upward-right)
  //   Incoming:  TODAY_REST - SWEEP  →  TODAY_REST  (arrives from below-right)
  // UP scroll: both sweep CCW (decreasing angle).
  //   Outgoing:  TODAY_REST  →  TODAY_REST - SWEEP  (exits downward-right)
  //   Incoming:  TODAY_REST + SWEEP  →  TODAY_REST  (arrives from above-right)
  GRect today_rest = layout->today_icon_rest_frame;
  GPoint today_center = GPoint(today_rest.origin.x + today_rest.size.w / 2,
                                today_rest.origin.y + today_rest.size.h / 2);
  int32_t radius = ICON_ARC_RADIUS;
  layout->anim_params.circle_center = GPoint(today_center.x + radius, today_center.y);
  layout->anim_params.radius = radius;
  layout->anim_params.animate_down = animate_down;
  layout->anim_params.outgoing_start_angle = ICON_ARC_TODAY_REST;
  if (animate_down) {
    layout->anim_params.outgoing_end_angle   = ICON_ARC_TODAY_REST + ICON_ARC_SWEEP_ANGLE;
    layout->anim_params.incoming_start_angle = ICON_ARC_TODAY_REST - ICON_ARC_SWEEP_ANGLE;
  } else {
    layout->anim_params.outgoing_end_angle   = ICON_ARC_TODAY_REST - ICON_ARC_SWEEP_ANGLE;
    layout->anim_params.incoming_start_angle = ICON_ARC_TODAY_REST + ICON_ARC_SWEEP_ANGLE;
  }
  // Place each icon at its starting position.
  if (animate_down) {
    // Scaler (incoming): starts at the actual on-screen tomorrow_rest position.
    prv_set_outgoing_weather_icon_frame(layout, layout->tomorrow_icon_rest_frame);
    // BitmapLayer (outgoing) is already at today_icon_rest_frame — no change needed.
    // New tomorrow icon entrance: start it at the arc incoming position in root coords.
    if (layout->anim_params.tomorrow_incoming) {
      GPoint cc_root = GPoint(
          layout->content_layer_origin.x + layout->anim_params.circle_center.x,
          layout->content_layer_origin.y + layout->anim_params.circle_center.y);
      layer_set_frame(bitmap_layer_get_layer(layout->tomorrow_weather_icon_layer),
                      prv_icon_frame_at_angle(cc_root, radius,
                                              layout->anim_params.incoming_start_angle,
                                              layout->tomorrow_icon_rest_frame.size));
    }
  } else {
    // BitmapLayer (incoming): starts at arc entry point above-right (TODAY_REST + SWEEP).
    layer_set_frame(bitmap_layer_get_layer(layout->current_weather_icon_layer),
                    prv_icon_frame_at_angle(layout->anim_params.circle_center, radius,
                                            layout->anim_params.incoming_start_angle,
                                            layout->today_icon_rest_frame.size));
  }
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  if (layout->current_weather_escape_layer) {
    GRect start_frame = animate_down
        ? layout->today_icon_rest_frame
        : prv_icon_frame_at_angle(layout->anim_params.circle_center, radius,
                                  layout->anim_params.incoming_start_angle,
                                  layout->today_icon_rest_frame.size);
    layout->anim_params.current_root_overlay = true;
    prv_set_current_escape_icon_frame(layout, start_frame);
    layer_set_hidden(layout->current_weather_escape_layer, false);
  }
#endif

  return true;
}

void weather_app_layout_animate(WeatherAppLayout *layout,
                                const WeatherLocationForecast *new_today,
                                const WeatherLocationForecast *new_next,
                                bool animate_down) {
  if (!prv_prepare_day_transition(layout, new_today, new_next, animate_down)) return;

  // Linear time: the moook frame tables ARE the easing. Stacking EaseOut on
  // top crushed the 3-frame anticipation (~25ms, invisible) and smeared the
  // bounce-back across ~130ms of mush — every firmware moook runs linear.
  layout->icon_animation = prv_start_anim(ICON_ANIM_DURATION_MS, &s_icon_anim_impl,
                                          prv_icon_anim_stopped, layout);

  // If this scroll lands on the last day (no next forecast), kick off the FIN
  // ribbon slide at the same time so it arrives exactly as the icon settles.
  if (!new_next && layout->fin_allowed) {
    prv_animate_fin_in(layout, ICON_ANIM_DURATION_MS);
  }

  layer_mark_dirty(layout->root_layer);
}

// ---- Return transition: list screen → main screen ----
// Mirror of the forward transition: icons collapse to flat bars at their list
// positions, fly right back to their main-screen rest frames, then spring open.
