/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pebble_compat.h"
#include <stdint.h>
#include "weather_types.h"

#define WEATHER_APP_LAYOUT_LOCATION_BAR_HEIGHT (18)
// Main-screen bar is taller than the forecast-list bar for readability.
// The small rects (144x168) need every row: keep the forecast-list height there.
#if !PBL_ROUND && PBL_DISPLAY_HEIGHT < 200
#define WEATHER_APP_LAYOUT_MAIN_BAR_HEIGHT (20)
#else
#define WEATHER_APP_LAYOUT_MAIN_BAR_HEIGHT (26)
#endif
// Round (gabbro) draws the large 75px vector weather icon (WX_WEATHER_ICONS_PDC)
// + the day swoop. Emery (PBL_ROUND==0) keeps the raster BitmapLayer path.
// IMPORTANT: a system app's PDC resource is mmap'd READ-ONLY into flash, so the
// sequence must be CLONED into RAM before any write (gdraw_command_sequence_set_
// bounds_size writes sequence->size and faulted on launch). See the clone in
// weather_app_layout_init.
// Round uses the LARGE (80x80) PDC art so the icon fills its quadrant; rect
// keeps the 50px raster. The 14/10 disc ratio is too fat at that size, so round
// also takes a tighter ratio -- see WEATHER_APP_LAYOUT_DISC_RATIO_NUM.
#define WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS PBL_ROUND

typedef struct WeatherAppLayout {
  Layer *root_layer;
  Layer *content_layer;
  BitmapLayer *current_weather_icon_layer;
  Layer *outgoing_weather_icon_layer;
  BitmapLayer *tomorrow_weather_icon_layer;
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
  GDrawCommandSequence *weather_icon_pdc_sequence;
  Layer *current_weather_escape_layer;
#endif
  GBitmap *current_weather_icon;
  GBitmap *outgoing_weather_icon;
  GBitmap *tomorrow_weather_icon;
  GDrawCommandImage *fin_pdc;  // real timeline 'fin' flag (END_OF_TIMELINE PDC)
  Layer *fin_layer;
  Layer *city_layer;   //!< round: the crown city masthead (topmost root child)
  Animation *fin_animation;
  const WeatherLocationForecast *forecast;
  const WeatherLocationForecast *next_forecast;
  GFont location_font;
  GFont temperature_font;
  GFont high_low_phrase_font;
  GFont metrics_font;
  GFont metrics_value_font;   // bold metric VALUES (Timeline label/value texture)
  GFont tomorrow_font;
  Layer *down_arrow_layer;
  Layer *location_bar_layer;
  char location_name[32];
  bool fin_allowed;
  GPoint content_layer_origin;   // screen-absolute origin of content_layer
  GRect today_icon_rest_frame;
  GRect tomorrow_icon_rest_frame;
  Animation *icon_animation;
  struct {
    GPoint circle_center;
    int32_t radius;
    int32_t outgoing_start_angle;
    int32_t outgoing_end_angle;
    int32_t incoming_start_angle;
    // incoming always ends at ICON_ARC_TODAY_REST
    WeatherType outgoing_weather_type;
    WeatherType incoming_weather_type;
    WeatherType tomorrow_exit_weather_type; // weather type of tomorrow icon while it exits/enters
    bool animate_down;
    bool tomorrow_reparented;  // true when tomorrow_layer lives in root_layer during animation
    bool tomorrow_incoming;    // true when tomorrow_layer is animating IN during DOWN animation
#if WEATHER_APP_LAYOUT_USE_PDC_WEATHER_ICONS
    bool current_root_overlay; // Gabbro-only: current icon exits above content clipping
#endif
  } anim_params;
  struct {
    GRect from;
    GRect to;
  } fin_anim;
  struct {
    // Pre-formatted strings snapshot of the OUTGOING text (captured before forecast pointer updates)
    char top_label[16];
    char top_temp[15];
    char top_highlow[15];
    char top_uv[12];       // UV bar numeral
    char top_phrase[20];   // condition line
    char top_desc[72];     // forecast description (warning/wind/precip)
    char bot_label[16];
    char bot_highlow[15];
    bool bot_valid;
    AnimationProgress progress;
    bool dir_down;
    bool active;
  } text_anim;
} WeatherAppLayout;

void weather_app_layout_init(WeatherAppLayout *layout, const GRect *frame);

void weather_app_layout_set_data(WeatherAppLayout *layout,
                                 const WeatherLocationForecast *today_forecast,
                                 const WeatherLocationForecast *next_forecast);

#define weather_app_layout_set_down_arrow_visible(layout, is_down_visible) \
  do { (void)(layout); (void)(is_down_visible); } while (0)

void weather_app_layout_set_fin_allowed(WeatherAppLayout *layout,
                                        bool fin_allowed);

void weather_app_layout_set_location(WeatherAppLayout *layout, const char *name);

void weather_app_layout_deinit(WeatherAppLayout *layout);

void weather_app_layout_animate(WeatherAppLayout *layout,
                                const WeatherLocationForecast *new_today,
                                const WeatherLocationForecast *new_next,
                                bool animate_down);


#if PBL_ROUND
// The weather report's UV bar, as a reusable component: glass-concentric arc ends, sun glyph,
// "UV INDEX", ten tally squares and the big value. The sunset card (expanded_view.c) draws the
// SAME bar, so this is the single implementation — do not clone it.
//
// `gc` is the glass centre IN THE CALLER'S coordinate space and every element is positioned
// relative to it, so the bar lands identically whether the caller draws into the report's inset
// content layer or the card's full-screen canvas. `by` is the bar's top row in that same space.
// `uv_value` drives the fill/severity colour; `uv_text` is drawn verbatim as the big number
// (callers may pass a string the integer parse would not reproduce).
//
// The bar's WIDTH is the glass chord at its own rows, so it narrows as it moves down: keep the
// content rows above ~y200 or the fixed-width contents overrun the arc ends.
// Two sizes. Full = the weather report's bar. Compact = a narrower bar for the sunset card,
// which hangs BELOW the card's centred content group where the glass has far less width: same
// glass-concentric arc ends and the same large LECO value, but tighter tally squares and a
// pulled-in value box so all ten squares still fit.
typedef enum {
  WeatherUvBarFull = 0,
  WeatherUvBarCompact,
} WeatherUvBarSize;

// `label` is the caption drawn beside the sun glyph — the report says "UV INDEX", the
// sunset card says "CURRENT UV" (its value is the current hour's, not the day's).
void weather_app_layout_draw_uv_bar(GContext *ctx, GPoint gc, int by,
                                    int uv_value, const char *uv_text, WeatherUvBarSize size,
                                    const char *label);

// Height in rows of each bar size, so callers can budget vertical space without guessing.
int weather_app_layout_uv_bar_height(WeatherUvBarSize size);
#endif
