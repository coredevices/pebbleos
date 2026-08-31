/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

/**
 * globe_view.c
 * Globe animation view implementation
 */

#include "globe_view.h"
// GLOBE_* resource ids come from the real resource_ids.auto.h (via
// pebble_compat.h); the stored-app pinned header is not used in the system app.

#include "weather_data_source.h"   // WX_DS_UNKNOWN_TEMP
#include "weather_math.h"
#include "applib/graphics/gdraw_command_transforms.h"
#include "weather.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/vibes.h"
#include "applib/applib_malloc.auto.h"
#include "applib/vendor/tinflate/tinflate.h"

#include <string.h>

#define GLOBE_FRAME_INTERVAL_MS 35  // ~28.6 FPS, matching Pebble's shredder PDC cadence
#define GLOBE_IDLE_TIMEOUT_MS 5000
#define GLOBE_IDLE_SLOWDOWN_STEPS 8
#define GLOBE_IDLE_SLOWDOWN_STEP_MS 6
#define NUM_BW_FRAMES 60
#define NUM_COLOR_LON_FRAMES 24
#define GLOBE_CUBEMAP_FACE_COUNT 6
#define GLOBE_CUBEMAP_FACE_SIZE 64
#define GLOBE_CUBEMAP_DATA_SIZE \
    ((GLOBE_CUBEMAP_FACE_COUNT * GLOBE_CUBEMAP_FACE_SIZE * GLOBE_CUBEMAP_FACE_SIZE) / 2)
#define GLOBE_CUBEMAP_PALETTE_SIZE 12
#define GLOBE_ROT_SHIFT 10
#define GLOBE_ROT_SCALE (1 << GLOBE_ROT_SHIFT)
#define GLOBE_DRAG_TRIGANGLE_PER_PX (TRIG_MAX_ANGLE / 420)
#define GLOBE_RENDER_BASE_DIAMETER 110
#define GLOBE_CITY_ROTATION_DURATION_MS 260
#define GLOBE_CITY_BOUNCE_DURATION_MS 160
#define GLOBE_CITY_BOUNCE_PX 8
#define GLOBE_CITY_LABEL_HEIGHT 28
#define GLOBE_CITY_LABEL_ROUND_BOTTOM_INSET PBL_IF_ROUND_ELSE(9, 0)
#define GLOBE_CITY_LABEL_ROUND_SIDE_INSET PBL_IF_ROUND_ELSE(50, 0)
#define GLOBE_INTRO_TITLE_HEIGHT 34
#define GLOBE_SAVED_LABEL_HEIGHT 34
#define GLOBE_SAVED_LABEL_ROUND_BOTTOM_INSET PBL_IF_ROUND_ELSE(18, 0)
#define GLOBE_SAVED_COG_SIZE 18
#define GLOBE_SAVED_LABEL_GAP 6
#define GLOBE_INTRO_SELECTION_DURATION_MS 175
#define GLOBE_INTRO_SELECTION_OFFSET_PX 5
#define GLOBE_PLANET_CENTER_Y_OFFSET -11
#define GLOBE_COLOR_PLANET_CENTER_Y_OFFSET \
    PBL_IF_ROUND_ELSE(-GLOBE_CRADLE_CENTER_Y_OFFSET, GLOBE_PLANET_CENTER_Y_OFFSET)
#define GLOBE_REVEALED_CENTER_Y_OFFSET \
    PBL_IF_ROUND_ELSE(0, GLOBE_CRADLE_CENTER_Y_OFFSET + GLOBE_PLANET_CENTER_Y_OFFSET)
#define GLOBE_CURRENT_LOCATION_INDEX 0
#define GLOBE_FIRST_PRESET_INDEX 1
#define GLOBE_CUSTOM_LOCATION_INDEX (CITY_PRESET_COUNT + GLOBE_FIRST_PRESET_INDEX)
#define GLOBE_SELECTOR_COUNT (CITY_PRESET_COUNT + 2)
#define GLOBE_CRUMPLE_DELAY_SEGMENTS 8
#define GLOBE_CRUMPLE_POINT_DURATION_NUM 2
#define GLOBE_CRUMPLE_POINT_DURATION_DEN 3
#define GLOBE_CRUMPLE_SWEEP_ANGLE DEG_TO_TRIGANGLE(-45)
#define GLOBE_CRADLE_CENTER_Y_OFFSET 5
// Small rect (flint/asterix 144x168): the intro art was drawn for taller
// screens — the cradle sequence, stand and arm scale down to fit between a
// slimmer title band and a slimmer saved-locations bar.
#define GLOBE_SMALL_RECT (!PBL_ROUND && PBL_DISPLAY_HEIGHT < 200)
#define GLOBE_CRADLE_SCALE_PCT (GLOBE_SMALL_RECT ? 78 : 100)
#define GLOBE_REVEAL_DURATION_MS 900
#define GLOBE_REVEAL_REVERSE_DURATION_MS 420
#define GLOBE_REVEAL_SPIN_STEPS 4
#define GLOBE_REVEAL_COLOR_SPIN_DEGREES_E2 72000
#define GLOBE_REVEAL_BW_SPEED_NUM 4
#define GLOBE_REVEAL_BW_SPEED_DEN 1
#define GLOBE_CRADLE_DROP_PX 24
#define GLOBE_COLOR_FINAL_SCALE_PERCENT PBL_IF_ROUND_ELSE(164, 150)
#define GLOBE_COLOR_GROW_START_PERCENT 22
#define GLOBE_CRADLE_ARM_CENTER_X 62
#define GLOBE_CRADLE_ARM_CENTER_Y 60
#define GLOBE_CRADLE_ARM_OUTER_RADIUS 64
#define GLOBE_CRADLE_ARM_THICKNESS 6
#define GLOBE_CRADLE_ARM_START_ANGLE DEG_TO_TRIGANGLE(20)
#define GLOBE_CRADLE_ARM_END_ANGLE DEG_TO_TRIGANGLE(205)
#define GLOBE_CRADLE_WIPE_MARGIN 8
#define GLOBE_TAP_THRESHOLD 20
#define GLOBE_DRAG_AXIS_THRESHOLD 5
#define GLOBE_COAST_FRAME_MS 48
#define GLOBE_COAST_STEP_NUM 3
#define GLOBE_COAST_STEP_DEN 2
#define GLOBE_COAST_DECAY_NUM 86
#define GLOBE_COAST_DECAY_DEN 100
#define GLOBE_COAST_MIN_SPEED_Q8 90
#define GLOBE_COAST_SETTLE_SPEED_Q8 210
#define GLOBE_COAST_START_SPEED_Q8 170
#define GLOBE_COAST_MAX_SPEED_Q8 2300
#define GLOBE_LOCK_RADIUS_PX 50
#define GLOBE_LOCK_SWOOP_DURATION_MS 300
#define GLOBE_LOCK_SETTLE_STEPS 2
#define GLOBE_LOCK_SETTLE_DIVISOR 2
#define GLOBE_LOCK_SETTLE_DEADZONE_PX 2
#define GLOBE_COMMIT_LOCK_RADIUS_PX 40
#define GLOBE_Q8_ONE 256
#define GLOBE_STARFIELD_WIDTH 400
#define GLOBE_STARFIELD_HEIGHT 228
#define GLOBE_ATMOSPHERE_GLOW_PX 10
#define GLOBE_ATMOSPHERE_LAYER_MARGIN_PX 2
#define GLOBE_OUTLINE_PX 0
#define GLOBE_PIN_EDGE_FRONT_Q10 (GLOBE_ROT_SCALE / 5)
#define GLOBE_SPACE_FADE_DITHER_SIZE 4
#define GLOBE_SPACE_FADE_FULL_PERCENT 86
#define GLOBE_SPACE_STAR_REVEAL_START_PERCENT 72
#define GLOBE_CRUMPLE_MAX_POINTS 96
// Fixed view-space light for the revealed globe: upper-left, toward the
// viewer (Q10 unit vector). The screen-space normal (sx, sy, sz) is already
// computed per pixel, so the lambert term costs two multiplies per mirrored
// pixel pair.
#define GLOBE_LIGHT_X_Q10 (-358)
#define GLOBE_LIGHT_Y_Q10 563
#define GLOBE_LIGHT_Z_Q10 768
// Posterized shading bands (thresholds on the Q10 lambert dot product),
// softened by the 4x4 ordered dither so the terminator doesn't band harshly.
// Tuned bright: full daylight across roughly half the
// disc, shadow reserved for the far limb.
#define GLOBE_SHADE_LIT_MIN_Q10 320
#define GLOBE_SHADE_MID_MIN_Q10 (-64)
// Wii-Forecast-Channel style atmosphere: the limb itself glows. Pixels whose
// view depth (sz) falls below RIM render as solid Celeste — a luminous band
// straddling the planet edge that merges with the halo outside — and pixels
// below BLEND dither toward PictonBlue, the bluish haze of atmosphere washing
// over terrain near the horizon.
#define GLOBE_HAZE_RIM_SZ_Q10 230
#define GLOBE_HAZE_BLEND_SZ_Q10 450
// Fresnel limb glint: the solid Celeste rim whitens where the limb faces the
// light. The lambert term is view-space, so the glint is FIXED at the screen's
// upper-left (like the terminator) and terrain rotates beneath it. HOT = solid
// White arc (~33 deg half-angle at the limb), WARM = 50% Bayer-dithered skirt.
#define GLOBE_RIM_HOT_MIN_Q10 560
#define GLOBE_RIM_WARM_MIN_Q10 400
// Ocean sun-glint: daylight (band-0) pixels whose lambert exceeds GLINT_MIN
// promote to the specular palette row (water only — land stays matte), with
// an ordered-dither density ramp reaching full at GLINT_MIN + SPAN. Max
// lambert is |L| ~= 1017, so this is a ~15-20px pool with a wide soft fringe.
#define GLOBE_GLINT_MIN_Q10 820
#define GLOBE_GLINT_SPAN_Q10 192
// Magnetic lock swoop: duration is proportional to the capture distance
// (5px capture clamps up to the 140ms floor, 49px = 267ms) clamped to
// [GLOBE_LOCK_SWOOP_MIN_MS, GLOBE_LOCK_SWOOP_DURATION_MS].
#define GLOBE_LOCK_SWOOP_BASE_MS 120
#define GLOBE_LOCK_SWOOP_MS_PER_PX 3
#define GLOBE_LOCK_SWOOP_MIN_MS 140
// Lock pulse: a Celeste ring that expands from the locked pin's head and
// fades out — fired on magnetic capture and button-nav arrival.
#define GLOBE_LOCK_PULSE_DURATION_MS 160
#define GLOBE_LOCK_PULSE_R_MIN_PX 7
#define GLOBE_LOCK_PULSE_R_GROW_PX 9
#define GLOBE_LOCK_PULSE_THICKNESS_PX 2
// Pin depth classes (post-cull depth_q8 range is 51..255): pins shrink toward
// the limb so cities read as sitting ON a sphere. Class 2 (front) is
// bit-identical to the original single-size pin.
#define GLOBE_PIN_DEPTH_MID_Q8   112
#define GLOBE_PIN_DEPTH_FRONT_Q8 192

static const uint8_t GLOBE_SPACE_FADE_DITHER[GLOBE_SPACE_FADE_DITHER_SIZE]
                                           [GLOBE_SPACE_FADE_DITHER_SIZE] = {
    { 0,  8,  2, 10 },
    { 12, 4, 14,  6 },
    { 3, 11,  1,  9 },
    { 15, 7, 13,  5 },
};

// Cubemap palette by lighting band: [0] daylight (the original colors),
// [1] terminator, [2] night side. Column order matches the packed cubemap's
// palette indices: DukeBlue, BlueMoon, VividCerulean, PictonBlue, Celeste,
// White, MayGreen, DarkGreen, JaegerGreen, SpringBud, ScreaminGreen, Mint.
static const uint8_t GLOBE_SHADE_PALETTE[4][GLOBE_CUBEMAP_PALETTE_SIZE] = {
    { 0xC2, 0xC7, 0xCB, 0xDB, 0xEF, 0xFF, 0xD9, 0xC4, 0xC9, 0xEC, 0xDD, 0xEE },
    { 0xC2, 0xC2, 0xC7, 0xCB, 0xDB, 0xEF, 0xC8, 0xC4, 0xC8, 0xD8, 0xC9, 0xD9 },
    // Shadow ocean floors at DukeBlue so the dark limb stays visibly blue
    // against space; OxfordBlue appears only for the rare deep-trench texels.
    { 0xC1, 0xC2, 0xC6, 0xC7, 0xCB, 0xDB, 0xC4, 0xC4, 0xC4, 0xC8, 0xC8, 0xC8 },
    // [3] specular sun-glint: every OCEAN column promoted one step up the blue
    // ramp (sun-on-water sheen); LAND columns identical to daylight so the
    // glint self-limits to water. Selected only for band-0 pixels inside the
    // GLOBE_GLINT_MIN_Q10 cap.
    { 0xC7, 0xCB, 0xDB, 0xEF, 0xFF, 0xFF, 0xD9, 0xC4, 0xC9, 0xEC, 0xDD, 0xEE },
};

// Haze wash keyed by lighting band: the sunlit limb atmosphere catches the sun
// (Celeste bloom decaying inward — the Wii limb); terminator/night keep the
// original PictonBlue wash.
static const uint8_t GLOBE_HAZE_WASH_BY_BAND[3] = {
    GColorCelesteARGB8, GColorPictonBlueARGB8, GColorPictonBlueARGB8,
};
// Haze reach keyed by band: the brightening wash is withheld sooner on the
// night side, so the atmosphere reads thick on the sun side and thin in
// shadow — a pure differential-BRIGHTENING depth cue (nothing drops below its
// base palette value). 220 == the full BLEND-RIM span; set [2] back to 220 to
// restore a symmetric night limb with one byte.
static const uint8_t GLOBE_HAZE_SPAN_BY_BAND[3] = { 220, 220, 128 };

static void animation_timer_handler(void *context);
static void schedule_frame_timer(GlobeView *view);
static void start_globe_idle_timer(GlobeView *view);
static GRect clip_rect_to_bounds(GRect rect, GRect bounds);
static void update_city_label_layer(GlobeView *view);
#ifdef CONFIG_TOUCH
static void set_free_roam_enabled(GlobeView *view, bool enabled);
#endif
static void show_intro_canvas(GlobeView *view);
static void show_revealed_space_layers(GlobeView *view);
static void mark_dynamic_globe_dirty(GlobeView *view);
static int32_t ease_out_quad(AnimationProgress progress);
static void start_lock_pulse(GlobeView *view);
static void ensure_visual_resources(GlobeView *view);
static void release_visual_resources(GlobeView *view);
static int bounce_offset(GlobeView *view);
static void framebuffer_draw_starfield(GBitmap *fb, GlobeView *view,
                                       GRect bounds, GRect clip_rect);
#ifdef CONFIG_TOUCH
static void stop_globe_coast(GlobeView *view, bool mark_dirty);
static bool try_start_magnetic_city_lock(GlobeView *view, int radius_px);
static bool point_is_on_revealed_globe(GlobeView *view, int16_t x, int16_t y);
static void apply_globe_screen_delta_q8(GlobeView *view,
                                        int32_t dx_q8,
                                        int32_t dy_q8);
#endif

static uint32_t bw_frame_count_for_view(GlobeView *view) {
    return (view && view->bw_frame_count > 0) ? view->bw_frame_count : NUM_BW_FRAMES;
}

static int bw_frame_index_for_view(GlobeView *view, int frame) {
    uint32_t frame_count = bw_frame_count_for_view(view);
    if (frame_count == 0) return 0;

    int result = frame % (int)frame_count;
    return result < 0 ? result + (int)frame_count : result;
}

static uint32_t bw_sequence_duration_for_view(GlobeView *view) {
    if (view && view->bw_sequence_duration_ms > 0) {
        return view->bw_sequence_duration_ms;
    }
    return bw_frame_count_for_view(view) * GLOBE_FRAME_INTERVAL_MS;
}

static void sync_bw_elapsed_to_current_frame(GlobeView *view) {
    if (!view) return;

    uint32_t duration = bw_sequence_duration_for_view(view);
    if (duration == 0) {
        view->bw_elapsed_ms = 0;
        return;
    }

    uint32_t frame = (uint32_t)bw_frame_index_for_view(view, view->current_frame);
    view->bw_elapsed_ms = (frame * GLOBE_FRAME_INTERVAL_MS) % duration;
}

// positive_modulo is provided by util/math.h (same semantics) — using that.

#ifdef CONFIG_TOUCH
static int32_t abs_i32(int32_t value) {
    return value < 0 ? -value : value;
}
#endif

static int rounded_divide(int value, int divisor) {
    if (divisor == 0) return 0;
    return value >= 0 ? (value + (divisor / 2)) / divisor
                      : (value - (divisor / 2)) / divisor;
}

static int bw_frame_for_longitude_e2(int16_t longitude_e2) {
    return positive_modulo(rounded_divide(-longitude_e2, 600), NUM_BW_FRAMES);
}

__attribute__((noinline)) static int32_t longitude_e2_for_bw_frame(int bw_frame) {
    int frame = ((bw_frame * NUM_COLOR_LON_FRAMES) + (NUM_BW_FRAMES / 2)) /
        NUM_BW_FRAMES % NUM_COLOR_LON_FRAMES;
    return -frame * 1500;
}

static int32_t shortest_longitude_delta_e2(int32_t from_e2, int32_t to_e2);

static int32_t selected_city_latitude_e2(GlobeView *view);
static int32_t selected_city_longitude_e2(GlobeView *view);

static SavedLocationEntry *saved_entry_for_index(GlobeView *view, int index) {
    if (!view || index < 0 || index >= view->saved_entry_count) return NULL;
    return &view->saved_entries[index];
}

static SavedLocationEntry *selected_saved_entry(GlobeView *view) {
    if (!view || view->saved_entry_count <= 0) return NULL;
    if (view->selected_city_index < 0) {
        view->selected_city_index = 0;
    } else if (view->selected_city_index >= view->saved_entry_count) {
        view->selected_city_index = view->saved_entry_count - 1;
    }
    return saved_entry_for_index(view, view->selected_city_index);
}

static int find_current_saved_entry_index(GlobeView *view) {
    if (!view) return -1;
    for (int i = 0; i < view->saved_entry_count; i++) {
        if (view->saved_entries[i].is_current_location) {
            return i;
        }
    }
    return view->saved_entry_count > 0 ? 0 : -1;
}

static bool saved_entry_matches(const SavedLocationEntry *a,
                                const SavedLocationEntry *b) {
    if (!a || !b) return false;
    // A re-sync can renumber records, so fall back to the name when the index
    // no longer lines up.
    if (a->ds_index == b->ds_index) return true;
    return a->label[0] && b->label[0] && strcmp(a->label, b->label) == 0;
}

static int find_saved_entry_index(GlobeView *view,
                                  const SavedLocationEntry *needle) {
    if (!view || !needle) return -1;
    for (int i = 0; i < view->saved_entry_count; i++) {
        if (saved_entry_matches(&view->saved_entries[i], needle)) {
            return i;
        }
    }
    return -1;
}

static void update_selected_transition_target(GlobeView *view) {
    if (!view) return;
    view->transition_color_target_latitude_e2 = selected_city_latitude_e2(view);
    view->transition_color_target_longitude_e2 = selected_city_longitude_e2(view);
}

// In-file only (defined below with the other saved-locations plumbing).
static void globe_view_reload_saved_locations(GlobeView *view);

static bool selected_city_is_current_location(GlobeView *view) {
    SavedLocationEntry *entry = selected_saved_entry(view);
    return entry && entry->is_current_location;
}

static int globe_max_selector_index(GlobeView *view) {
    return (view && view->saved_entry_count > 0)
               ? view->saved_entry_count - 1
               : 0;
}

static int32_t selected_city_latitude_e2(GlobeView *view) {
    SavedLocationEntry *entry = selected_saved_entry(view);
    return entry ? entry->latitude_e2 : 0;
}

static int32_t selected_city_longitude_e2(GlobeView *view) {
    SavedLocationEntry *entry = selected_saved_entry(view);
    return entry ? entry->longitude_e2 : 0;
}

static bool selected_city_is_valid(GlobeView *view, int city_index) {
    SavedLocationEntry *e = saved_entry_for_index(view, city_index);
    if (!e) return false;
    // A location the phone sent WITHOUT coordinates cannot be placed on the
    // globe (the watch has no geocoder), so skip it while cycling rather than
    // rotating to (0,0). The caller already retries and falls back to a bounce.
    return e->has_coordinates ||
           (e->is_current_location && view && view->has_current_location);
}

static bool saved_entry_has_globe_coordinates(GlobeView *view,
                                              SavedLocationEntry *entry) {
    if (!entry) return false;
    if (entry->has_coordinates) return true;
    return entry->is_current_location &&
           view && view->has_current_location;
}

static int16_t saved_entry_latitude_e2(GlobeView *view,
                                       SavedLocationEntry *entry) {
    if (entry && entry->is_current_location &&
        view && view->has_current_location) {
        return (int16_t)view->current_location_latitude_e2;
    }
    return entry ? entry->latitude_e2 : 0;
}

static int16_t saved_entry_longitude_e2(GlobeView *view,
                                        SavedLocationEntry *entry) {
    if (entry && entry->is_current_location &&
        view && view->has_current_location) {
        return (int16_t)view->current_location_longitude_e2;
    }
    return entry ? entry->longitude_e2 : 0;
}

#ifdef CONFIG_TOUCH
// Merged has-coordinates + coordinate fetch: one call per entry instead of
// three, false when the entry has no usable globe position.
static bool saved_entry_globe_coords(GlobeView *view, SavedLocationEntry *entry,
                                     int32_t *latitude_e2_out,
                                     int32_t *longitude_e2_out) {
    if (!saved_entry_has_globe_coordinates(view, entry)) return false;
    if (entry->is_current_location &&
        view && view->has_current_location) {
        *latitude_e2_out = (int16_t)view->current_location_latitude_e2;
        *longitude_e2_out = (int16_t)view->current_location_longitude_e2;
    } else {
        *latitude_e2_out = entry->latitude_e2;
        *longitude_e2_out = entry->longitude_e2;
    }
    return true;
}

static int nearest_city_index_for_orientation(GlobeView *view) {
    if (!view) return 0;

    int best_index = 0;
    int32_t best_score = INT32_MAX;
    int max_index = globe_max_selector_index(view);
    for (int i = 0; i <= max_index; i++) {
        if (!selected_city_is_valid(view, i)) continue;
        SavedLocationEntry *entry = saved_entry_for_index(view, i);
        int32_t candidate_latitude_e2;
        int32_t candidate_longitude_e2;
        if (!saved_entry_globe_coords(view, entry, &candidate_latitude_e2,
                                      &candidate_longitude_e2)) {
            continue;
        }

        int32_t lat_delta = candidate_latitude_e2 - view->color_latitude_e2;
        int32_t lon_delta = shortest_longitude_delta_e2(
            view->color_longitude_e2,
            candidate_longitude_e2);
        int32_t score = (lat_delta * lat_delta) + (lon_delta * lon_delta);
        if (score < best_score) {
            best_score = score;
            best_index = i;
        }
    }

    return best_index;
}
#endif

static GSize get_vector_or_bitmap_size(GlobeView *view) {
    if (view->bw_sequence) {
        return gdraw_command_sequence_get_bounds_size(view->bw_sequence);
    }
    if (view->cradle_pdc) {
        return gdraw_command_image_get_bounds_size(view->cradle_pdc);
    }
    return GSize(124, 144);
}

static void draw_bw_frame(GContext *ctx, GlobeView *view, GPoint origin, GSize frame_size) {
    (void)frame_size;
    if (view->bw_sequence) {
        uint32_t duration = bw_sequence_duration_for_view(view);
        uint32_t elapsed = duration > 0 ? view->bw_elapsed_ms % duration : 0;
        GDrawCommandFrame *frame =
            gdraw_command_sequence_get_frame_by_elapsed(view->bw_sequence, elapsed);
        if (!frame) {
            frame = gdraw_command_sequence_get_frame_by_index(
                view->bw_sequence,
                bw_frame_index_for_view(view, view->current_frame));
        }
        if (frame) {
            gdraw_command_frame_draw(ctx, view->bw_sequence, frame, origin);
            return;
        }
    }
}

static AnimationProgress ease_in_out(AnimationProgress progress) {
    int32_t p = (int32_t)progress;
    int32_t square = weather_norm_square(p);
    int32_t shape = (3 * ANIMATION_NORMALIZED_MAX) - (2 * p);
    return (AnimationProgress)weather_scale_i32(shape, square,
                                                ANIMATION_NORMALIZED_MAX);
}

static int crumple_delay_index(GPoint point, GPoint center) {
    int angle = positive_modulo(
        atan2_lookup(point.y - center.y, point.x - center.x) + GLOBE_CRUMPLE_SWEEP_ANGLE,
        TRIG_MAX_ANGLE);
    int distance_from_sweep = abs(angle - (TRIG_MAX_ANGLE / 2));
    return (int)weather_scale_i32(distance_from_sweep,
                                  GLOBE_CRUMPLE_DELAY_SEGMENTS,
                                  TRIG_MAX_ANGLE / 2);
}

static AnimationProgress segmented_crumple_progress(AnimationProgress amount, int delay_index) {
    int duration = (ANIMATION_NORMALIZED_MAX * GLOBE_CRUMPLE_POINT_DURATION_NUM) /
                   GLOBE_CRUMPLE_POINT_DURATION_DEN;
    int delay_per_item = (ANIMATION_NORMALIZED_MAX - duration) /
                         GLOBE_CRUMPLE_DELAY_SEGMENTS;
    int offset = amount - (delay_index * delay_per_item);
    if (offset <= 0) return 0;

    int32_t relative = weather_scale_i32(offset, ANIMATION_NORMALIZED_MAX,
                                         duration);
    if (relative >= ANIMATION_NORMALIZED_MAX) return ANIMATION_NORMALIZED_MAX;
    return ease_in_out((AnimationProgress)relative);
}

static GPoint crumple_point(GPoint point, GPoint center, AnimationProgress amount) {
    AnimationProgress progress =
        segmented_crumple_progress(amount, crumple_delay_index(point, center));
    int scale = ANIMATION_NORMALIZED_MAX - progress;
    return GPoint(
        center.x + (int16_t)((int32_t)(point.x - center.x) * scale / ANIMATION_NORMALIZED_MAX),
        center.y + (int16_t)((int32_t)(point.y - center.y) * scale / ANIMATION_NORMALIZED_MAX)
    );
}

// (bw_crumple_sequence machinery deleted — the field was only ever NULL)

typedef struct {
    GContext *ctx;
    GPoint origin;
    GPoint center;
    AnimationProgress amount;
    bool drew_command;
} DirectCrumpleContext;

static bool draw_direct_crumple_command(GDrawCommand *command,
                                        uint32_t index,
                                        void *context) {
    (void)index;
    DirectCrumpleContext *crumple = context;
    if (!command || gdraw_command_get_hidden(command)) return true;

    GDrawCommandType type = gdraw_command_get_type(command);
    GColor fill_color = gdraw_command_get_fill_color(command);
    GColor stroke_color = gdraw_command_get_stroke_color(command);
    uint8_t stroke_width = gdraw_command_get_stroke_width(command);
    uint32_t stroke_scale = ANIMATION_NORMALIZED_MAX -
                            ((uint32_t)crumple->amount * 55 / 100);
    uint8_t scaled_stroke =
        (uint8_t)((uint32_t)stroke_width * stroke_scale / ANIMATION_NORMALIZED_MAX);
    if (scaled_stroke == 0) scaled_stroke = 1;

    if (type == GDrawCommandTypeCircle) {
        uint16_t radius = gdraw_command_get_radius(command);
        AnimationProgress progress = ease_in_out(crumple->amount);
        uint16_t scaled_radius =
            (uint16_t)((uint32_t)radius * (ANIMATION_NORMALIZED_MAX - progress) /
                       ANIMATION_NORMALIZED_MAX);
        if (scaled_radius < 2) {
            return true;
        }

        GPoint center = gdraw_command_get_point(command, 0);
        center = GPoint(center.x + crumple->origin.x,
                        center.y + crumple->origin.y);

        if (!gcolor_equal(fill_color, GColorClear)) {
            graphics_context_set_fill_color(crumple->ctx, fill_color);
            graphics_fill_circle(crumple->ctx, center, scaled_radius);
        }
        if (!gcolor_equal(stroke_color, GColorClear)) {
            graphics_context_set_stroke_color(crumple->ctx, stroke_color);
            graphics_context_set_stroke_width(crumple->ctx, scaled_stroke);
            graphics_draw_circle(crumple->ctx, center, scaled_radius);
        }
        crumple->drew_command = true;
        return true;
    }

    if (type != GDrawCommandTypePath && type != GDrawCommandTypePrecisePath) {
        return true;
    }

    uint16_t num_points = gdraw_command_get_num_points(command);
    if (num_points < 3) return true;
    if (num_points > GLOBE_CRUMPLE_MAX_POINTS) {
        num_points = GLOBE_CRUMPLE_MAX_POINTS;
    }

    GPoint points[GLOBE_CRUMPLE_MAX_POINTS];
    int16_t min_x = INT16_MAX;
    int16_t min_y = INT16_MAX;
    int16_t max_x = INT16_MIN;
    int16_t max_y = INT16_MIN;
    for (uint16_t i = 0; i < num_points; i++) {
        GPoint point = gdraw_command_get_point(command, i);
        GPoint crumpled = crumple_point(point, crumple->center, crumple->amount);
        crumpled = GPoint(crumpled.x + crumple->origin.x,
                          crumpled.y + crumple->origin.y);
        points[i] = crumpled;

        if (crumpled.x < min_x) min_x = crumpled.x;
        if (crumpled.y < min_y) min_y = crumpled.y;
        if (crumpled.x > max_x) max_x = crumpled.x;
        if (crumpled.y > max_y) max_y = crumpled.y;
    }

    if ((max_x - min_x) < 2 && (max_y - min_y) < 2) {
        return true;
    }

    GPath path = {
        .num_points = num_points,
        .points = points,
        .rotation = 0,
        .offset = GPoint(0, 0),
    };

    if (!gdraw_command_get_path_open(command) &&
        !gcolor_equal(fill_color, GColorClear)) {
        graphics_context_set_fill_color(crumple->ctx, fill_color);
        gpath_draw_filled(crumple->ctx, &path);
    }
    if (!gcolor_equal(stroke_color, GColorClear)) {
        graphics_context_set_stroke_color(crumple->ctx, stroke_color);
        graphics_context_set_stroke_width(crumple->ctx, scaled_stroke);
        if (gdraw_command_get_path_open(command)) {
            gpath_draw_outline_open(crumple->ctx, &path);
        } else {
            gpath_draw_outline(crumple->ctx, &path);
        }
    }
    crumple->drew_command = true;
    return true;
}

static bool draw_bw_crumple_frame(GContext *ctx, GlobeView *view, GPoint origin,
                                  GSize frame_size, int amount) {
    if (!view || !view->bw_sequence) return false;

    int bw_frame = bw_frame_index_for_view(view, view->transition_bw_frame);
    GDrawCommandFrame *frame =
        gdraw_command_sequence_get_frame_by_index(view->bw_sequence, bw_frame);
    if (!frame) return false;

    GDrawCommandList *list = gdraw_command_frame_get_command_list(frame);
    if (!list) return false;

    DirectCrumpleContext context = {
        .ctx = ctx,
        .origin = origin,
        .center = GPoint(frame_size.w / 2, (frame_size.h / 2) + GLOBE_PLANET_CENTER_Y_OFFSET),
        .amount = (AnimationProgress)amount,
        .drew_command = false,
    };
    gdraw_command_list_iterate(list, draw_direct_crumple_command, &context);
    return context.drew_command;
}

static bool color_highlight_bw_command(GDrawCommand *command,
                                       uint32_t index,
                                       void *context) {
    (void)context;

    // Command 0 of every frame is the (low-poly) ocean disc; the land
    // polygons follow it, so recolor by position rather than command type.
    gdraw_command_set_stroke_color(command, GColorBlack);
#if PBL_BW
    // The colour tints quantize to a solid black blob on 1-bit — shade just
    // the ocean disc 50% instead; land stays white line-art.
    gdraw_command_set_fill_color(command, index == 0 ? GColorLightGray : GColorWhite);
#else
    gdraw_command_set_fill_color(command, index == 0 ? GColorVividCerulean
                                                     : GColorScreaminGreen);
#endif
    return true;
}

// Restore pass for the in-place highlight below: the bw art is UNIFORMLY
// stroke-Black / fill-White (byte-verified across all 60 frames, widths vary
// but are never touched), so constant restore is exactly invertible.
static bool restore_bw_command(GDrawCommand *command, uint32_t index,
                               void *context) {
    (void)index; (void)context;
    gdraw_command_set_stroke_color(command, GColorBlack);
    gdraw_command_set_fill_color(command, GColorWhite);
    return true;
}

static void draw_intro_world_frame(GContext *ctx, GlobeView *view,
                                   GPoint origin, GSize frame_size) {
    // Highlight IN PLACE: recolor the single frame being drawn, draw, restore —
    // this replaced an 18.7 KB full-sequence clone (size pass). The
    // sequence lives on the heap (prv_load_inflated), never in mmap'd flash.
    if (view && view->intro_world_selected && view->bw_sequence) {
        uint32_t duration = bw_sequence_duration_for_view(view);
        uint32_t elapsed = duration > 0 ? view->bw_elapsed_ms % duration : 0;
        GDrawCommandFrame *frame =
            gdraw_command_sequence_get_frame_by_elapsed(view->bw_sequence, elapsed);
        if (!frame) {
            frame = gdraw_command_sequence_get_frame_by_index(
                view->bw_sequence,
                bw_frame_index_for_view(view, view->current_frame));
        }
        if (frame) {
            GDrawCommandList *list = gdraw_command_frame_get_command_list(frame);
            if (list) {
                gdraw_command_list_iterate(list, color_highlight_bw_command, NULL);
                gdraw_command_frame_draw(ctx, view->bw_sequence, frame, origin);
                gdraw_command_list_iterate(list, restore_bw_command, NULL);
                return;
            }
        }
    }
    draw_bw_frame(ctx, view, origin, frame_size);
}

static void draw_cradle(GContext *ctx, GlobeView *view, GPoint origin, GSize frame_size) {
    (void)frame_size;
    graphics_context_set_antialiased(ctx, false);
    graphics_context_set_fill_color(ctx, GColorBlack);
    const int acx = GLOBE_CRADLE_ARM_CENTER_X * GLOBE_CRADLE_SCALE_PCT / 100;
    const int acy = GLOBE_CRADLE_ARM_CENTER_Y * GLOBE_CRADLE_SCALE_PCT / 100;
    const int aor = GLOBE_CRADLE_ARM_OUTER_RADIUS * GLOBE_CRADLE_SCALE_PCT / 100;
    const int ath = GLOBE_CRADLE_ARM_THICKNESS * GLOBE_CRADLE_SCALE_PCT / 100;
    graphics_fill_radial(
        ctx,
        GRect(origin.x + acx - aor, origin.y + acy - aor, aor * 2, aor * 2),
        GOvalScaleModeFitCircle,
        ath,
        GLOBE_CRADLE_ARM_START_ANGLE,
        GLOBE_CRADLE_ARM_END_ANGLE
    );

    if (view->cradle_pdc) {
        gdraw_command_image_draw(ctx, view->cradle_pdc, origin);
    }
    graphics_context_set_antialiased(ctx, true);
}

static int32_t normalize_longitude_e2(int32_t longitude_e2) {
    while (longitude_e2 < -18000) longitude_e2 += 36000;
    while (longitude_e2 >= 18000) longitude_e2 -= 36000;
    return longitude_e2;
}

__attribute__((noinline)) static int32_t clamp_latitude_e2(int32_t latitude_e2) {
    if (latitude_e2 > 8900) return 8900;
    if (latitude_e2 < -8900) return -8900;
    return latitude_e2;
}

static int32_t shortest_longitude_delta_e2(int32_t from_e2, int32_t to_e2) {
    int32_t delta = normalize_longitude_e2(to_e2 - from_e2);
    if (delta >= 18000) delta -= 36000;
    if (delta < -18000) delta += 36000;
    return delta;
}

__attribute__((noinline)) static int trigangle_from_degrees_e2(int32_t degrees_e2) {
    return (int)weather_scale_i32(degrees_e2, TRIG_MAX_ANGLE, 36000);
}

static int32_t trig_sin_q10(int angle) {
    return (int32_t)(sin_lookup(positive_modulo(angle, TRIG_MAX_ANGLE)) *
                     GLOBE_ROT_SCALE / TRIG_MAX_RATIO);
}

static int32_t trig_cos_q10(int angle) {
    return (int32_t)(cos_lookup(positive_modulo(angle, TRIG_MAX_ANGLE)) *
                     GLOBE_ROT_SCALE / TRIG_MAX_RATIO);
}

// (globe_isqrt was token-identical to weather_isqrt — merged, size pass)

// floor(sqrt(value)) via integer Newton iteration from a seed known to be
// >= the true root (the previous pixel's depth in a row scan). From such a
// seed the iteration is strictly decreasing and lands exactly on the floor,
// so results match weather_isqrt() bit-for-bit at a fraction of the cost.
static inline int32_t floor_sqrt_seeded(int32_t value, int32_t seed) {
    if (value <= 0) return 0;
    if (seed <= 0) return weather_isqrt(value);
    int32_t s = seed;
    for (;;) {
        int32_t next = (s + (value / s)) >> 1;
        if (next >= s) break;
        s = next;
    }
    return s;
}

static void matrix_identity(int32_t matrix[9]) {
    matrix[0] = GLOBE_ROT_SCALE;
    matrix[1] = 0;
    matrix[2] = 0;
    matrix[3] = 0;
    matrix[4] = GLOBE_ROT_SCALE;
    matrix[5] = 0;
    matrix[6] = 0;
    matrix[7] = 0;
    matrix[8] = GLOBE_ROT_SCALE;
}

#ifdef CONFIG_TOUCH
static void matrix_copy(int32_t destination[9], const int32_t source[9]) {
    for (int i = 0; i < 9; i++) {
        destination[i] = source[i];
    }
}

static void matrix_multiply(int32_t destination[9],
                            const int32_t left[9],
                            const int32_t right[9]) {
    int32_t result[9];
    for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
            int64_t value = 0;
            for (int i = 0; i < 3; i++) {
                value += (int64_t)left[(row * 3) + i] * right[(i * 3) + column];
            }
            result[(row * 3) + column] = (int32_t)(value >> GLOBE_ROT_SHIFT);
        }
    }
    matrix_copy(destination, result);
}

static void matrix_screen_rotation(int32_t matrix[9], int yaw_angle, int pitch_angle) {
    int32_t yaw[9];
    int32_t pitch[9];
    int32_t cy = trig_cos_q10(yaw_angle);
    int32_t sy = trig_sin_q10(yaw_angle);
    int32_t cp = trig_cos_q10(pitch_angle);
    int32_t sp = trig_sin_q10(pitch_angle);

    yaw[0] = cy;
    yaw[1] = 0;
    yaw[2] = sy;
    yaw[3] = 0;
    yaw[4] = GLOBE_ROT_SCALE;
    yaw[5] = 0;
    yaw[6] = -sy;
    yaw[7] = 0;
    yaw[8] = cy;

    pitch[0] = GLOBE_ROT_SCALE;
    pitch[1] = 0;
    pitch[2] = 0;
    pitch[3] = 0;
    pitch[4] = cp;
    pitch[5] = -sp;
    pitch[6] = 0;
    pitch[7] = sp;
    pitch[8] = cp;

    matrix_multiply(matrix, yaw, pitch);
}

static void normalize_axis(int32_t *x, int32_t *y, int32_t *z) {
    int32_t length = weather_isqrt((*x * *x) + (*y * *y) + (*z * *z));
    if (length <= 0) return;

    *x = (*x * GLOBE_ROT_SCALE) / length;
    *y = (*y * GLOBE_ROT_SCALE) / length;
    *z = (*z * GLOBE_ROT_SCALE) / length;
}

static void normalize_globe_rotation(GlobeView *view) {
    if (!view) return;

    int32_t rx = view->globe_rotation[0];
    int32_t ry = view->globe_rotation[3];
    int32_t rz = view->globe_rotation[6];
    int32_t ux = view->globe_rotation[1];
    int32_t uy = view->globe_rotation[4];
    int32_t uz = view->globe_rotation[7];
    normalize_axis(&rx, &ry, &rz);
    normalize_axis(&ux, &uy, &uz);

    int32_t fx = ((ry * uz) - (rz * uy)) >> GLOBE_ROT_SHIFT;
    int32_t fy = ((rz * ux) - (rx * uz)) >> GLOBE_ROT_SHIFT;
    int32_t fz = ((rx * uy) - (ry * ux)) >> GLOBE_ROT_SHIFT;
    normalize_axis(&fx, &fy, &fz);

    ux = ((fy * rz) - (fz * ry)) >> GLOBE_ROT_SHIFT;
    uy = ((fz * rx) - (fx * rz)) >> GLOBE_ROT_SHIFT;
    uz = ((fx * ry) - (fy * rx)) >> GLOBE_ROT_SHIFT;
    normalize_axis(&ux, &uy, &uz);

    view->globe_rotation[0] = rx;
    view->globe_rotation[1] = ux;
    view->globe_rotation[2] = fx;
    view->globe_rotation[3] = ry;
    view->globe_rotation[4] = uy;
    view->globe_rotation[5] = fy;
    view->globe_rotation[6] = rz;
    view->globe_rotation[7] = uz;
    view->globe_rotation[8] = fz;
}
#endif

static void matrix_from_lat_lon(int32_t matrix[9],
                                int32_t latitude_e2,
                                int32_t longitude_e2) {
    latitude_e2 = clamp_latitude_e2(latitude_e2);
    longitude_e2 = normalize_longitude_e2(longitude_e2);

    int lat_angle = trigangle_from_degrees_e2(latitude_e2);
    int lon_angle = trigangle_from_degrees_e2(longitude_e2);
    int32_t sin_lat = trig_sin_q10(lat_angle);
    int32_t cos_lat = trig_cos_q10(lat_angle);
    int32_t sin_lon = trig_sin_q10(lon_angle);
    int32_t cos_lon = trig_cos_q10(lon_angle);

    int32_t right_x = cos_lon;
    int32_t right_y = 0;
    int32_t right_z = -sin_lon;
    int32_t forward_x = (int32_t)((int64_t)sin_lon * cos_lat >> GLOBE_ROT_SHIFT);
    int32_t forward_y = sin_lat;
    int32_t forward_z = (int32_t)((int64_t)cos_lon * cos_lat >> GLOBE_ROT_SHIFT);
    int32_t up_x = (int32_t)(-((int64_t)sin_lat * sin_lon >> GLOBE_ROT_SHIFT));
    int32_t up_y = cos_lat;
    int32_t up_z = (int32_t)(-((int64_t)sin_lat * cos_lon >> GLOBE_ROT_SHIFT));

    matrix[0] = right_x;
    matrix[1] = up_x;
    matrix[2] = forward_x;
    matrix[3] = right_y;
    matrix[4] = up_y;
    matrix[5] = forward_y;
    matrix[6] = right_z;
    matrix[7] = up_z;
    matrix[8] = forward_z;
}

static void set_color_orientation(GlobeView *view,
                                  int32_t latitude_e2,
                                  int32_t longitude_e2) {
    if (!view) return;

    latitude_e2 = clamp_latitude_e2(latitude_e2);
    longitude_e2 = normalize_longitude_e2(longitude_e2);
    view->color_latitude_e2 = latitude_e2;
    view->color_longitude_e2 = longitude_e2;
    view->current_frame = bw_frame_for_longitude_e2((int16_t)longitude_e2);
    sync_bw_elapsed_to_current_frame(view);
    matrix_from_lat_lon(view->globe_rotation, latitude_e2, longitude_e2);
#ifdef CONFIG_TOUCH
    // 64-bit product: pm(lon) * width * 256 overflows int32 for most
    // western longitudes (pm >= 20972), which used to scatter the starfield.
    view->starfield_offset_x_q8 =
        (int32_t)(((int64_t)positive_modulo(longitude_e2, 36000) *
                   GLOBE_STARFIELD_WIDTH * GLOBE_Q8_ONE) / 36000);
    view->starfield_offset_y_q8 =
        (latitude_e2 * GLOBE_STARFIELD_HEIGHT * GLOBE_Q8_ONE) / 36000;
#endif
}

// The globe blobs are stored raw-deflate compressed in the pack ([u32 LE
// inflated_size][raw DEFLATE stream], tools/deflate_resource.py) and inflated
// here via the firmware's tinflate (the same raw-deflate decoder uPNG uses).
// Buffers come from malloc_try — the app-task heap free() and the allocator
// gdraw_command_sequence_destroy's munmap-or-free path pair with, so the
// inflated BW sequence can be handed to the normal destroy path. The croaking
// allocators are off-limits here: the globe is an optional visual, so heap
// pressure must degrade to "no globe", never reboot the watch.
// Returns NULL on any failure; *out_size gets the inflated size.
static void *prv_load_inflated(uint32_t res_id, uint32_t *out_size) {
    ResHandle handle = resource_get_handle(res_id);
    size_t csize = resource_size(handle);
    if (csize <= sizeof(uint32_t)) return NULL;
    uint8_t *cbuf = malloc_try(csize);
    if (!cbuf) return NULL;
    void *out = NULL;
    uint32_t inflated_size = 0;
    if (resource_load(handle, cbuf, csize) == csize) {
        memcpy(&inflated_size, cbuf, sizeof(inflated_size));   // LE prefix
        out = malloc_try(inflated_size);
        if (out) {
            unsigned int dlen = inflated_size;
            if (tinflate_uncompress(out, &dlen, cbuf + sizeof(uint32_t),
                                    (unsigned int)(csize - sizeof(uint32_t))) != TINF_OK ||
                dlen != inflated_size) {
                free(out);
                out = NULL;
            }
        }
    }
    free(cbuf);
    if (out && out_size) *out_size = inflated_size;
    return out;
}

static bool load_cubemap_resource(GlobeView *view) {
    if (!view) return false;
    if (view->cubemap_data && view->cubemap_size >= GLOBE_CUBEMAP_DATA_SIZE) {
        return true;
    }

    uint32_t size = 0;
    uint8_t *data = prv_load_inflated(RESOURCE_ID_GLOBE_CUBEMAP, &size);
    if (!data) return false;
    if (size < GLOBE_CUBEMAP_DATA_SIZE) {
        applib_free(data);
        return false;
    }

    if (view->cubemap_data) {
        free(view->cubemap_data);
    }
    view->cubemap_data = data;
    view->cubemap_size = size;
    return true;
}

static void unload_cubemap_resource(GlobeView *view) {
    if (!view || !view->cubemap_data) return;

    free(view->cubemap_data);
    view->cubemap_data = NULL;
    view->cubemap_size = 0;
}

static bool load_starfield_resource(GlobeView *view) {
    if (!view) return false;
    if (view->starfield_data && view->starfield_size > 2) return true;

    uint32_t size = 0;
    uint8_t *data = prv_load_inflated(RESOURCE_ID_GLOBE_STARFIELD, &size);
    if (!data) return false;
    if (size <= 2) {
        applib_free(data);
        return false;
    }

    if (view->starfield_data) free(view->starfield_data);
    view->starfield_data = data;
    view->starfield_size = size;
    return true;
}

static void unload_starfield_resource(GlobeView *view) {
    if (!view || !view->starfield_data) return;

    free(view->starfield_data);
    view->starfield_data = NULL;
    view->starfield_size = 0;
}

static void ensure_visual_resources(GlobeView *view) {
    if (!view) return;

    if (!view->bw_sequence) {
        // Stored payload-deflated: inflate into an applib_malloc buffer and
        // validate with the exact gate create_with_resource uses. Ownership
        // passes to the sequence — gdraw_command_sequence_destroy applib_free()s
        // heap pointers (applib_resource_munmap_or_free address-range check).
        uint32_t seq_size = 0;
        GDrawCommandSequence *seq =
            prv_load_inflated(RESOURCE_ID_GLOBE_BW_SEQUENCE, &seq_size);
        if (seq && !gdraw_command_sequence_validate(seq, seq_size)) {
            applib_free(seq);
            seq = NULL;
        }
        view->bw_sequence = seq;
        if (view->bw_sequence) {
            view->bw_frame_count =
                gdraw_command_sequence_get_num_frames(view->bw_sequence);
            view->bw_sequence_duration_ms =
                gdraw_command_sequence_get_total_duration(view->bw_sequence);
            if (view->bw_frame_count == 0) {
                view->bw_frame_count = NUM_BW_FRAMES;
            }
            if (view->bw_sequence_duration_ms == 0) {
                view->bw_sequence_duration_ms =
                    view->bw_frame_count * GLOBE_FRAME_INTERVAL_MS;
            }
        }
    }

    if (!view->cradle_pdc) {
        view->cradle_pdc = gdraw_command_image_create_with_resource(
            RESOURCE_ID_GLOBE_CRADLE_PDC);
    }

#if GLOBE_SMALL_RECT
    // Scale the writable clones once, right after load. (Both live in RAM —
    // scaling an mmap'd resource would write read-only flash.)
    if (view->bw_sequence && !view->bw_sequence_scaled) {
        const GSize from = gdraw_command_sequence_get_bounds_size(view->bw_sequence);
        if (from.w > 0 && from.h > 0) {
            const GSize to = GSize((int16_t)(from.w * GLOBE_CRADLE_SCALE_PCT / 100),
                                   (int16_t)(from.h * GLOBE_CRADLE_SCALE_PCT / 100));
            const uint32_t frames = gdraw_command_sequence_get_num_frames(view->bw_sequence);
            for (uint32_t i = 0; i < frames; i++) {
                GDrawCommandFrame *frame =
                    gdraw_command_sequence_get_frame_by_index(view->bw_sequence, i);
                if (frame) {
                    gdraw_command_list_scale(gdraw_command_frame_get_command_list(frame),
                                             from, to);
                }
            }
            gdraw_command_sequence_set_bounds_size(view->bw_sequence, to);
        }
        view->bw_sequence_scaled = true;
    }
    if (view->cradle_pdc && !view->cradle_pdc_scaled) {
        const GSize from = gdraw_command_image_get_bounds_size(view->cradle_pdc);
        if (from.w > 0 && from.h > 0) {
            gdraw_command_image_scale(view->cradle_pdc,
                                      GSize((int16_t)(from.w * GLOBE_CRADLE_SCALE_PCT / 100),
                                            (int16_t)(from.h * GLOBE_CRADLE_SCALE_PCT / 100)));
        }
        view->cradle_pdc_scaled = true;
    }
#endif

    load_cubemap_resource(view);
    load_starfield_resource(view);
}

static void release_visual_resources(GlobeView *view) {
    if (!view) return;

    unload_cubemap_resource(view);
    unload_starfield_resource(view);
    if (view->bw_sequence) {
        gdraw_command_sequence_destroy(view->bw_sequence);
        view->bw_sequence = NULL;
    }
    if (view->cradle_pdc) {
        gdraw_command_image_destroy(view->cradle_pdc);
        view->cradle_pdc = NULL;
    }
}

// Returns the packed cubemap PALETTE INDEX (0..11); the caller picks the
// final color through GLOBE_SHADE_PALETTE using its lighting band.
static inline uint8_t cubemap_sample(const uint8_t *cubemap_data,
                                     int32_t x, int32_t y, int32_t z) {
    int32_t ax = x < 0 ? -x : x;
    int32_t ay = y < 0 ? -y : y;
    int32_t az = z < 0 ? -z : z;
    int face;
    int32_t major;
    int32_t uc;
    int32_t vc;

    // C division truncates toward zero, so folding the sign into the
    // numerator gives the same result as negating the quotient — one shared
    // division pair replaces the per-face copies.
    if (ax >= ay && ax >= az) {
        major = ax;
        face = x >= 0 ? 0 : 1;
        uc = x >= 0 ? -z : z;
        vc = -y;
    } else if (ay >= ax && ay >= az) {
        major = ay;
        face = y >= 0 ? 2 : 3;
        uc = x;
        vc = y >= 0 ? z : -z;
    } else {
        major = az;
        face = z >= 0 ? 4 : 5;
        uc = z >= 0 ? x : -x;
        vc = -y;
    }

    int32_t u = uc * GLOBE_ROT_SCALE / major;
    int32_t v = vc * GLOBE_ROT_SCALE / major;

    int sx = ((u + GLOBE_ROT_SCALE) * GLOBE_CUBEMAP_FACE_SIZE) /
             (GLOBE_ROT_SCALE * 2);
    int sy = ((v + GLOBE_ROT_SCALE) * GLOBE_CUBEMAP_FACE_SIZE) /
             (GLOBE_ROT_SCALE * 2);
    if (sx < 0) sx = 0;
    if (sy < 0) sy = 0;
    if (sx >= GLOBE_CUBEMAP_FACE_SIZE) sx = GLOBE_CUBEMAP_FACE_SIZE - 1;
    if (sy >= GLOBE_CUBEMAP_FACE_SIZE) sy = GLOBE_CUBEMAP_FACE_SIZE - 1;

    int face_area = GLOBE_CUBEMAP_FACE_SIZE * GLOBE_CUBEMAP_FACE_SIZE;
    int pixel_index = (face * face_area) + (sy * GLOBE_CUBEMAP_FACE_SIZE) + sx;
    uint8_t packed = cubemap_data[pixel_index >> 1];
    uint8_t palette_index = (pixel_index & 1) ? (packed & 0x0F) : (packed >> 4);
    if (palette_index >= GLOBE_CUBEMAP_PALETTE_SIZE) {
        return 1;  // BlueMoon slot — same fallback color as before
    }
    return palette_index;
}

static uint8_t starfield_color_from_flags(uint8_t flags) {
    switch (flags & 0x03) {
        case 3: return GColorWhiteARGB8;
        case 2: return GColorCelesteARGB8;
        default: return GColorLightGrayARGB8;
    }
}

static void draw_forward_space_fade(GContext *ctx, GRect bounds, int amount,
                                    GlobeView *view) {
    if (amount <= 0) return;

    const int full_at =
        (ANIMATION_NORMALIZED_MAX * GLOBE_SPACE_FADE_FULL_PERCENT) / 100;
    const int stars_at =
        (ANIMATION_NORMALIZED_MAX * GLOBE_SPACE_STAR_REVEAL_START_PERCENT) / 100;

    if (amount >= full_at) {
        graphics_context_set_fill_color(ctx, GColorBlack);
        graphics_fill_rect(ctx, bounds, 0, GCornerNone);
        if (amount >= stars_at) {
            GBitmap *fb = graphics_capture_frame_buffer(ctx);
            if (fb) {
                framebuffer_draw_starfield(fb, view, bounds,
                                           clip_rect_to_bounds(bounds, gbitmap_get_bounds(fb)));
                graphics_release_frame_buffer(ctx, fb);
            }
        }
        return;
    }

    int relative = weather_scale_i32(amount, ANIMATION_NORMALIZED_MAX, full_at);
    int inv = ANIMATION_NORMALIZED_MAX - relative;
    int eased = ANIMATION_NORMALIZED_MAX -
        weather_norm_square(inv);
    int coverage =
        (eased * 16 + (ANIMATION_NORMALIZED_MAX - 1)) /
        ANIMATION_NORMALIZED_MAX;
    if (coverage <= 0) return;
    if (coverage >= 16) {
        graphics_context_set_fill_color(ctx, GColorBlack);
        graphics_fill_rect(ctx, bounds, 0, GCornerNone);
        return;
    }

    GBitmap *fb = graphics_capture_frame_buffer(ctx);
    if (!fb) return;

    GRect fbb = gbitmap_get_bounds(fb);
    GRect clipped = clip_rect_to_bounds(bounds, fbb);
    int left = clipped.origin.x;
    int right = clipped.origin.x + clipped.size.w - 1;
    int bottom = clipped.origin.y + clipped.size.h;

    for (int ay = clipped.origin.y; ay < bottom; ay++) {
        GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)ay);
        int row_left = left < (int)ri.min_x ? (int)ri.min_x : left;
        int row_right = right > (int)ri.max_x ? (int)ri.max_x : right;
        for (int ax = row_left; ax <= row_right; ax++) {
            if (GLOBE_SPACE_FADE_DITHER[ay % GLOBE_SPACE_FADE_DITHER_SIZE]
                                      [ax % GLOBE_SPACE_FADE_DITHER_SIZE] <
                coverage) {
                weather_fb_row_set(ri.data, ax, GColorBlackARGB8);
            }
        }
    }

    if (amount >= stars_at) {
        framebuffer_draw_starfield(fb, view, bounds, clipped);
    }
    graphics_release_frame_buffer(ctx, fb);
}

static int revealed_globe_radius(void) {
    return ((GLOBE_RENDER_BASE_DIAMETER * GLOBE_COLOR_FINAL_SCALE_PERCENT) /
            100) / 2;
}

__attribute__((noinline)) static GPoint revealed_globe_center_for_bounds(GRect bounds, int offset) {
    return GPoint(bounds.origin.x + (bounds.size.w / 2),
                  bounds.origin.y + (bounds.size.h / 2) +
                      GLOBE_REVEALED_CENTER_Y_OFFSET + offset);
}

static GRect clip_rect_to_bounds(GRect rect, GRect bounds) {
    int left = rect.origin.x;
    int top = rect.origin.y;
    int right = rect.origin.x + rect.size.w;
    int bottom = rect.origin.y + rect.size.h;
    int bounds_right = bounds.origin.x + bounds.size.w;
    int bounds_bottom = bounds.origin.y + bounds.size.h;

    if (left < bounds.origin.x) left = bounds.origin.x;
    if (top < bounds.origin.y) top = bounds.origin.y;
    if (right > bounds_right) right = bounds_right;
    if (bottom > bounds_bottom) bottom = bounds_bottom;
    if (right <= left || bottom <= top) return GRectZero;

    return GRect(left, top, right - left, bottom - top);
}

static GRect revealed_globe_layer_frame(GRect bounds) {
    int extent = revealed_globe_radius() + GLOBE_ATMOSPHERE_GLOW_PX +
                 GLOBE_CITY_BOUNCE_PX + GLOBE_ATMOSPHERE_LAYER_MARGIN_PX;
    GPoint center = revealed_globe_center_for_bounds(bounds, 0);
    return clip_rect_to_bounds(
        GRect(center.x - extent, center.y - extent,
              (extent * 2) + 1, (extent * 2) + 1),
        bounds);
}

static void clear_framebuffer_rect(GBitmap *fb, GRect rect) {
    if (!fb || rect.size.w <= 0 || rect.size.h <= 0) return;

    GRect fbb = gbitmap_get_bounds(fb);
    GRect clipped = clip_rect_to_bounds(rect, fbb);
    int left = clipped.origin.x;
    int right = clipped.origin.x + clipped.size.w - 1;
    int bottom = clipped.origin.y + clipped.size.h;

    for (int ay = clipped.origin.y; ay < bottom; ay++) {
        GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)ay);
        int row_left = left < (int)ri.min_x ? (int)ri.min_x : left;
        int row_right = right > (int)ri.max_x ? (int)ri.max_x : right;
        if (row_right >= row_left) {
#if PBL_BW
            for (int ax = row_left; ax <= row_right; ax++) {
                weather_fb_row_set(ri.data, ax, GColorBlackARGB8);
            }
#else
            memset(&ri.data[row_left], GColorBlackARGB8,
                   (size_t)(row_right - row_left + 1));
#endif
        }
    }
}

static void framebuffer_draw_starfield(GBitmap *fb, GlobeView *view,
                                       GRect bounds, GRect clip_rect) {
    if (!fb || !view || !view->starfield_data || view->starfield_size <= 2 ||
        bounds.size.w <= 0 || bounds.size.h <= 0) {
        return;
    }

    uint16_t count = view->starfield_data[0] |
                     ((uint16_t)view->starfield_data[1] << 8);
    size_t available = (view->starfield_size - 2) / 3;
    if (count > available) count = (uint16_t)available;

#ifdef CONFIG_TOUCH
    int offset_x = positive_modulo(
        (int)(view->starfield_offset_x_q8 / GLOBE_Q8_ONE),
        GLOBE_STARFIELD_WIDTH);
    int offset_y = positive_modulo(
        (int)(view->starfield_offset_y_q8 / GLOBE_Q8_ONE),
        GLOBE_STARFIELD_HEIGHT);
#else
    // No touch pan/coast on this platform, so the starfield never scrolls.
    int offset_x = 0;
    int offset_y = 0;
#endif
    int clip_left = clip_rect.origin.x;
    int clip_top = clip_rect.origin.y;
    int clip_right = clip_rect.origin.x + clip_rect.size.w - 1;
    int clip_bottom = clip_rect.origin.y + clip_rect.size.h - 1;
    const uint8_t *records = view->starfield_data + 2;

    for (uint16_t i = 0; i < count; i++) {
        uint8_t flags = records[(i * 3) + 2];
        int source_x = records[i * 3] + ((flags & 0x80) ? 256 : 0);
        int source_y = records[(i * 3) + 1];
        // Parallax: bright (near) stars sweep at 2x, the rest at 1x. The
        // factor must be an INTEGER: the offset derives from a longitude
        // that wraps by exactly one field width per revolution, and only
        // whole multiples of the width vanish under the modulo — a
        // fractional factor would make stars jump when a spin crosses
        // longitude zero.
        int mult = ((flags & 0x03) == 3) ? 2 : 1;
        int first_x = positive_modulo(source_x - (offset_x * mult),
                                      GLOBE_STARFIELD_WIDTH);
        int first_y = positive_modulo(source_y - (offset_y * mult),
                                      GLOBE_STARFIELD_HEIGHT);
        for (int local_y = first_y; local_y < bounds.size.h;
             local_y += GLOBE_STARFIELD_HEIGHT) {
            for (int local_x = first_x; local_x < bounds.size.w;
                 local_x += GLOBE_STARFIELD_WIDTH) {
                int ax = bounds.origin.x + local_x;
                int ay = bounds.origin.y + local_y;
                if (ay < clip_top || ay > clip_bottom ||
                    ax < clip_left || ax > clip_right) {
                    continue;
                }

                GBitmapDataRowInfo ri =
                    gbitmap_get_data_row_info(fb, (uint16_t)ay);
                if (ax < (int)ri.min_x || ax > (int)ri.max_x) continue;
                weather_fb_row_set(ri.data, ax, starfield_color_from_flags(flags));
            }
        }
    }
}

static void draw_space_background(GContext *ctx, GRect bounds, GlobeView *view) {
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_rect(ctx, bounds, 0, GCornerNone);

    if (!view || !view->starfield_data) return;

    GBitmap *fb = graphics_capture_frame_buffer(ctx);
    if (!fb) return;
    framebuffer_draw_starfield(fb, view, bounds,
                               clip_rect_to_bounds(bounds, gbitmap_get_bounds(fb)));
    graphics_release_frame_buffer(ctx, fb);
}

// A filled circle IS a ring with inner radius 0 (d2 >= 0 is always true) —
// one rasterizer serves both (size pass). NOINLINE on the ring keeps
// the sharing real; inlined, it would clone into every caller.
static void framebuffer_draw_ring(GBitmap *fb, GPoint center, int outer_r,
                                  int inner_r, uint8_t color, GRect clip_rect);

static void framebuffer_fill_circle(GBitmap *fb, GPoint center, int radius,
                                    uint8_t color, GRect clip_rect) {
    framebuffer_draw_ring(fb, center, radius, 0, color, clip_rect);
}

// Ring (annulus) rasterizer for the lock pulse — framebuffer_fill_circle's
// bbox scan with an inner-radius reject.
static NOINLINE void framebuffer_draw_ring(GBitmap *fb, GPoint center, int outer_r,
                                           int inner_r, uint8_t color, GRect clip_rect) {
    if (!fb || outer_r <= 0) return;
    if (inner_r < 0) inner_r = 0;

    GRect fbb = gbitmap_get_bounds(fb);
    GRect clipped = clip_rect_to_bounds(clip_rect, fbb);
    int outer_sq = outer_r * outer_r;
    int inner_sq = inner_r * inner_r;
    int clip_left = clipped.origin.x;
    int clip_top = clipped.origin.y;
    int clip_right = clipped.origin.x + clipped.size.w - 1;
    int clip_bottom = clipped.origin.y + clipped.size.h - 1;

    for (int ay = center.y - outer_r; ay <= center.y + outer_r; ay++) {
        if (ay < clip_top || ay > clip_bottom) continue;
        int dy = ay - center.y;
        GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)ay);
        for (int ax = center.x - outer_r; ax <= center.x + outer_r; ax++) {
            if (ax < clip_left || ax > clip_right ||
                ax < (int)ri.min_x || ax > (int)ri.max_x) {
                continue;
            }
            int dx = ax - center.x;
            int d2 = (dx * dx) + (dy * dy);
            if (d2 <= outer_sq && d2 >= inner_sq) {
                weather_fb_row_set(ri.data, ax, color);
            }
        }
    }
}

// Depth-posterized pin styles (3 classes echoing the 3-band lit shading):
// pins shrink toward the limb and their raise flattens, so the cities read
// as floating on a sphere. Class 2 (front) is the original pin geometry.
typedef struct { uint8_t outline_r, head_r, base_r, raise; } GlobePinStyle;
static const GlobePinStyle GLOBE_PIN_STYLES[3] = {
    { 3, 2, 1, 2 },   // limb  (depth 51..111)
    { 4, 2, 2, 3 },   // mid   (112..191)
    { 5, 3, 2, 4 },   // front (192..255) — original geometry
};

static void framebuffer_draw_raised_pin(GBitmap *fb, GPoint globe_center,
                                        GPoint surface, int depth_q8,
                                        bool emphasized, GRect clip_rect) {
    int cls = depth_q8 >= GLOBE_PIN_DEPTH_FRONT_Q8
                  ? 2
                  : (depth_q8 >= GLOBE_PIN_DEPTH_MID_Q8 ? 1 : 0);
    const GlobePinStyle *st = &GLOBE_PIN_STYLES[cls];
    int g = emphasized ? 1 : 0;   // locked pin grows 1px (lands with the pulse)
    int raise = st->raise + g;
    int dx = surface.x - globe_center.x;
    int dy = surface.y - globe_center.y;
    int distance = weather_isqrt((dx * dx) + (dy * dy));
    GPoint head = surface;
    if (distance > 0) {
        head.x += (dx * raise) / distance;
        head.y += (dy * raise) / distance;
    } else {
        head.y -= raise;
    }

    framebuffer_fill_circle(fb, surface, st->base_r, GColorBlackARGB8, clip_rect);
    framebuffer_fill_circle(fb, head, st->outline_r + g, GColorBlackARGB8, clip_rect);
    framebuffer_fill_circle(fb, head, st->head_r + g, GColorWhiteARGB8, clip_rect);
}

static bool project_lat_lon_to_globe_point_with_depth(GlobeView *view,
                                                     int32_t latitude_e2,
                                                     int32_t longitude_e2,
                                                     GPoint center,
                                                     int radius,
                                                     int32_t min_front_scale,
                                                     GPoint *point_out,
                                                     int *depth_q8_out) {
    if (!view || !point_out || radius <= 0) return false;

    latitude_e2 = clamp_latitude_e2(latitude_e2);
    longitude_e2 = normalize_longitude_e2(longitude_e2);

    int lat_angle = trigangle_from_degrees_e2(latitude_e2);
    int lon_angle = trigangle_from_degrees_e2(longitude_e2);
    int32_t sin_lat = trig_sin_q10(lat_angle);
    int32_t cos_lat = trig_cos_q10(lat_angle);
    int32_t sin_lon = trig_sin_q10(lon_angle);
    int32_t cos_lon = trig_cos_q10(lon_angle);

    int32_t wx = (int32_t)((int64_t)sin_lon * cos_lat >> GLOBE_ROT_SHIFT);
    int32_t wy = sin_lat;
    int32_t wz = (int32_t)((int64_t)cos_lon * cos_lat >> GLOBE_ROT_SHIFT);

    int32_t sx =
        (int32_t)(((int64_t)view->globe_rotation[0] * wx +
                   (int64_t)view->globe_rotation[3] * wy +
                   (int64_t)view->globe_rotation[6] * wz) >>
                  GLOBE_ROT_SHIFT);
    int32_t sy =
        (int32_t)(((int64_t)view->globe_rotation[1] * wx +
                   (int64_t)view->globe_rotation[4] * wy +
                   (int64_t)view->globe_rotation[7] * wz) >>
                  GLOBE_ROT_SHIFT);
    int32_t sz =
        (int32_t)(((int64_t)view->globe_rotation[2] * wx +
                   (int64_t)view->globe_rotation[5] * wy +
                   (int64_t)view->globe_rotation[8] * wz) >>
                  GLOBE_ROT_SHIFT);

    if (sz <= min_front_scale) return false;

    point_out->x = center.x + (int)(sx * radius / GLOBE_ROT_SCALE);
    point_out->y = center.y - (int)(sy * radius / GLOBE_ROT_SCALE);
    if (depth_q8_out) {
        int depth_q8 = (sz * 256) / GLOBE_ROT_SCALE;
        if (depth_q8 < 0) depth_q8 = 0;
        else if (depth_q8 > 255) depth_q8 = 255;
        *depth_q8_out = depth_q8;
    }
    return true;
}

#ifdef CONFIG_TOUCH
__attribute__((noinline)) static bool project_lat_lon_to_globe_point(GlobeView *view,
                                           int32_t latitude_e2,
                                           int32_t longitude_e2,
                                           GPoint center,
                                           int radius,
                                           GPoint *point_out) {
    return project_lat_lon_to_globe_point_with_depth(view, latitude_e2, longitude_e2,
                                                    center, radius,
                                                    GLOBE_ROT_SCALE / 10,
                                                    point_out, NULL);
}

static int nearest_centered_city_index(GlobeView *view, int radius_px) {
    if (!view || !view->window || radius_px <= 0) return -1;

    Layer *root_layer = window_get_root_layer(view->window);
    if (!root_layer) return -1;

    GRect bounds = layer_get_bounds(root_layer);
    GPoint center = revealed_globe_center_for_bounds(bounds, bounce_offset(view));
    int globe_radius = revealed_globe_radius() - GLOBE_OUTLINE_PX;
    if (globe_radius <= 0) return -1;
    int best_index = -1;
    int best_distance_sq = (radius_px * radius_px) + 1;

    int max_index = globe_max_selector_index(view);
    for (int i = 0; i <= max_index; i++) {
        SavedLocationEntry *entry = saved_entry_for_index(view, i);
        int32_t entry_latitude_e2;
        int32_t entry_longitude_e2;
        if (!saved_entry_globe_coords(view, entry, &entry_latitude_e2,
                                      &entry_longitude_e2)) {
            continue;
        }

        GPoint point;
        if (!project_lat_lon_to_globe_point(view,
                                            entry_latitude_e2,
                                            entry_longitude_e2,
                                            center,
                                            globe_radius,
                                            &point)) {
            continue;
        }

        int dx = point.x - center.x;
        int dy = point.y - center.y;
        int distance_sq = (dx * dx) + (dy * dy);
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_index = i;
        }
    }

    return best_index;
}

static bool city_pin_offset_from_center(GlobeView *view,
                                        int city_index,
                                        int *dx_out,
                                        int *dy_out) {
    if (!view || !view->window || !dx_out || !dy_out) return false;

    SavedLocationEntry *entry = saved_entry_for_index(view, city_index);
    int32_t entry_latitude_e2;
    int32_t entry_longitude_e2;
    if (!saved_entry_globe_coords(view, entry, &entry_latitude_e2,
                                  &entry_longitude_e2)) {
        return false;
    }

    Layer *root_layer = window_get_root_layer(view->window);
    if (!root_layer) return false;

    GRect bounds = layer_get_bounds(root_layer);
    GPoint center = revealed_globe_center_for_bounds(bounds, bounce_offset(view));
    int globe_radius = revealed_globe_radius() - GLOBE_OUTLINE_PX;
    if (globe_radius <= 0) return false;
    GPoint point;
    if (!project_lat_lon_to_globe_point(view,
                                        entry_latitude_e2,
                                        entry_longitude_e2,
                                        center,
                                        globe_radius,
                                        &point)) {
        return false;
    }

    *dx_out = point.x - center.x;
    *dy_out = point.y - center.y;
    return true;
}

static void settle_city_pin_to_center(GlobeView *view, int city_index) {
    int deadzone_sq = GLOBE_LOCK_SETTLE_DEADZONE_PX *
                      GLOBE_LOCK_SETTLE_DEADZONE_PX;

    for (int i = 0; i < GLOBE_LOCK_SETTLE_STEPS; i++) {
        int dx;
        int dy;
        if (!city_pin_offset_from_center(view, city_index, &dx, &dy)) return;
        if ((dx * dx) + (dy * dy) <= deadzone_sq) {
            return;
        }

        int nudge_x = -dx / GLOBE_LOCK_SETTLE_DIVISOR;
        int nudge_y = -dy / GLOBE_LOCK_SETTLE_DIVISOR;
        if (nudge_x == 0 && dx != 0) nudge_x = dx < 0 ? 1 : -1;
        if (nudge_y == 0 && dy != 0) nudge_y = dy < 0 ? 1 : -1;

        apply_globe_screen_delta_q8(view,
                                    (int32_t)nudge_x * GLOBE_Q8_ONE,
                                    (int32_t)nudge_y * GLOBE_Q8_ONE);
    }
}
#endif

static void draw_saved_location_pins(GBitmap *fb, GlobeView *view,
                                     GPoint center, int radius,
                                     GRect clip_rect) {
    if (!fb || !view || !view->is_revealed || view->is_revealing) return;

    int pin_radius = radius - GLOBE_OUTLINE_PX;
    if (pin_radius <= 0) return;

    // Two passes: project + cull into a list, depth-sort (back pins paint
    // first so near cities overlap far ones — the 3D read), then draw. The
    // sort is stable and Current is appended last, so at equal depth the
    // current-location pin still paints on top (old semantics).
    typedef struct { GPoint pt; uint8_t depth; bool emphasized; } PinDrawEntry;
    PinDrawEntry pins[SAVED_LOCATIONS_MAX_ENTRIES + 1];
    int n = 0;

    int max_index = globe_max_selector_index(view);
    for (int i = 0; i <= max_index && n < (int)(sizeof(pins) / sizeof(pins[0])); i++) {
        SavedLocationEntry *entry = saved_entry_for_index(view, i);
        if (!saved_entry_has_globe_coordinates(view, entry)) continue;
        if (entry->is_current_location) continue;

        GPoint pin;
        int depth = 255;
        if (!project_lat_lon_to_globe_point_with_depth(view,
                                                       saved_entry_latitude_e2(view, entry),
                                                       saved_entry_longitude_e2(view, entry),
                                                       center, pin_radius,
                                                       GLOBE_PIN_EDGE_FRONT_Q10,
                                                       &pin, &depth)) {
            continue;
        }

        bool emphasized = (i == view->selected_city_index);
#ifdef CONFIG_TOUCH
        emphasized = emphasized && !view->is_free_roam;
        if (view->city_anim && view->hover_lock_active) {
            // Grow mid-swoop so the emphasis lands as the pin centres.
            emphasized = emphasized &&
                         view->city_anim_progress > ANIMATION_NORMALIZED_MAX / 2;
        }
#endif
        pins[n++] = (PinDrawEntry){ .pt = pin, .depth = (uint8_t)depth,
                                    .emphasized = emphasized };
    }

    if (view->has_current_location && n < (int)(sizeof(pins) / sizeof(pins[0]))) {
        GPoint pin;
        int depth = 255;
        if (project_lat_lon_to_globe_point_with_depth(view,
                                                       view->current_location_latitude_e2,
                                                       view->current_location_longitude_e2,
                                                       center, pin_radius,
                                                       GLOBE_PIN_EDGE_FRONT_Q10,
                                                       &pin, &depth)) {
            pins[n++] = (PinDrawEntry){ .pt = pin, .depth = (uint8_t)depth,
                                        .emphasized = false };
        }
    }

    for (int i = 1; i < n; i++) {   // stable insertion sort, ascending depth
        PinDrawEntry key = pins[i];
        int j = i;
        while (j > 0 && pins[j - 1].depth > key.depth) {
            pins[j] = pins[j - 1];
            j--;
        }
        pins[j] = key;
    }

    for (int k = 0; k < n; k++) {
        framebuffer_draw_raised_pin(fb, center, pins[k].pt, pins[k].depth,
                                    pins[k].emphasized, clip_rect);
    }
}

#if PBL_BW
// Ordered-dither quantizer for the globe raster: 2-bit-per-channel luminance
// (green-weighted) against a 4x4 Bayer threshold, so the BW globe keeps its
// shading bands — terrain vs sea vs glint vs atmosphere — instead of
// collapsing into a solid disc.
static inline bool prv_bw_lit(uint8_t argb8, int x, int y) {
    static const uint8_t kBayer[4][4] = {
        {  0,  8,  2, 10 }, { 12,  4, 14,  6 }, {  3, 11,  1,  9 }, { 15,  7, 13,  5 },
    };
    const int lum = ((argb8 >> 4) & 0x3) * 3 + ((argb8 >> 2) & 0x3) * 4 +
                    (argb8 & 0x3) * 2;   // 0..27
    return (lum * 16) > (kBayer[y & 3][x & 3] * 27 + 13);
}
#endif

// One shaded globe pixel: colour boards write the byte; BW boards write the
// Bayer-dithered bit. (weather_fb_row_set stays for pure black/white intents.)
static inline void prv_globe_px(uint8_t *row_data, int x, int y, uint8_t argb8) {
#if PBL_BW
    bitset8_update(row_data, (unsigned)x, prv_bw_lit(argb8, x, y));
#else
    row_data[x] = argb8;
#endif
}

static void fill_row_span(uint8_t *row_data, int y, int x0, int x1,
                          int lo, int hi, uint8_t color) {
    if (x0 < lo) x0 = lo;
    if (x1 > hi) x1 = hi;
    if (x1 < x0) return;
#if PBL_BW
    // Packed 1-bit rows: a byte memset would overrun the 20-byte row (it
    // stomped the render context on flint) — write dithered bits instead.
    for (int x = x0; x <= x1; x++) {
        prv_globe_px(row_data, x, y, color);
    }
#else
    (void)y;
    memset(&row_data[x0], color, (size_t)(x1 - x0 + 1));
#endif
}

// Draws the non-textured rings of one globe row (the 4-step atmosphere
// gradient + the black outline) as memset spans and returns the half-width
// of the textured segment (-1 when the row has none). radii_sq holds,
// outermost first, the squared radii of the ring boundaries; the innermost
// entry is offset by -1 because the textured test is a strict inequality
// (dist^2 < inner^2).
#define GLOBE_RING_COUNT 5
static int fill_globe_ring_row(uint8_t *row_data, int y, int cx, int dy_sq,
                               const int32_t *radii_sq, int lo, int hi) {
    // Atmosphere halo: a luminous Celeste band on the rim itself (no hard
    // outline — it merges with the on-planet rim glow), fading into space.
    static const uint8_t ring_colors[GLOBE_RING_COUNT] = {
        GColorDukeBlueARGB8,       // outermost wisp
        GColorBlueMoonARGB8,
        GColorVividCeruleanARGB8,
        GColorPictonBlueARGB8,
        GColorCelesteARGB8,        // brightest, straddling the limb
    };
    int spans[GLOBE_RING_COUNT + 1];
    for (int r = 0; r <= GLOBE_RING_COUNT; r++) {
        int32_t rem = radii_sq[r] - dy_sq;
        spans[r] = rem >= 0 ? (int)weather_isqrt(rem) : -1;
    }
    for (int r = 0; r < GLOBE_RING_COUNT; r++) {
        fill_row_span(row_data, y, cx - spans[r], cx - spans[r + 1] - 1,
                      lo, hi, ring_colors[r]);
        fill_row_span(row_data, y, cx + spans[r + 1] + 1, cx + spans[r],
                      lo, hi, ring_colors[r]);
    }
    return spans[GLOBE_RING_COUNT];
}

static void draw_cubemap_globe_at_center(GContext *ctx, GlobeView *view,
                                         GPoint center, int scale_percent,
                                         GRect clip_rect,
                                         bool clear_clip) {
    if (!view || !view->cubemap_data || scale_percent <= 0) return;

    int diameter = (GLOBE_RENDER_BASE_DIAMETER * scale_percent) / 100;
    if (diameter < 4) return;

    int radius = diameter / 2;
    int outline_inner_radius = radius - GLOBE_OUTLINE_PX;
    if (outline_inner_radius < 0) outline_inner_radius = 0;
    int outline_inner_radius_sq = outline_inner_radius * outline_inner_radius;
    int glow_outer_radius = radius + GLOBE_ATMOSPHERE_GLOW_PX;
    const int32_t ring_radii_sq[GLOBE_RING_COUNT + 1] = {
        glow_outer_radius * glow_outer_radius,     // deep-space wisp
        (radius + 8) * (radius + 8),
        (radius + 6) * (radius + 6),
        (radius + 4) * (radius + 4),
        (radius + 2) * (radius + 2),               // Celeste rim band
        outline_inner_radius_sq - 1,               // textured face (strict <)
    };

    GBitmap *fb = graphics_capture_frame_buffer(ctx);
    if (!fb) return;

    GRect fbb = gbitmap_get_bounds(fb);
    GRect clipped = clip_rect_to_bounds(clip_rect, fbb);
    if (clear_clip) {
        clear_framebuffer_rect(fb, clipped);
        framebuffer_draw_starfield(fb, view, fbb, clipped);
    }

    int clip_left = clipped.origin.x;
    int clip_top = clipped.origin.y;
    int clip_right = clipped.origin.x + clipped.size.w - 1;
    int clip_bottom = clipped.origin.y + clipped.size.h - 1;

    // Hoist the rotation matrix and texture pointer into locals: the
    // framebuffer byte writes below would otherwise force the compiler to
    // reload them from the view struct on every pixel.
    int32_t m[9];
    memcpy(m, view->globe_rotation, sizeof(m));
    const uint8_t *cubemap = view->cubemap_data;

    const int cx = center.x;
    for (int dy = -glow_outer_radius; dy <= glow_outer_radius; dy++) {
        int ay = center.y + dy;
        if (ay < clip_top || ay > clip_bottom) continue;

        GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)ay);

        int row_left = clip_left < (int)ri.min_x ? (int)ri.min_x : clip_left;
        int row_right = clip_right > (int)ri.max_x ? (int)ri.max_x : clip_right;
        if (row_left > row_right) continue;

        int t_span = fill_globe_ring_row(ri.data, ay, cx, dy * dy, ring_radii_sq,
                                         row_left, row_right);
        if (t_span < 0) continue;
        int lo = cx - t_span < row_left ? row_left : cx - t_span;
        int hi = cx + t_span > row_right ? row_right : cx + t_span;
        if (lo > hi) continue;

        int32_t sy = ((int32_t)-dy * GLOBE_ROT_SCALE) / radius;
        int32_t plane_sq = (GLOBE_ROT_SCALE * GLOBE_ROT_SCALE) - (sy * sy);
        int32_t row_x = m[1] * sy;
        int32_t row_y = m[4] * sy;
        int32_t row_z = m[7] * sy;
        int32_t light_row = GLOBE_LIGHT_Y_Q10 * sy;
        const uint8_t *dither_row =
            GLOBE_SPACE_FADE_DITHER[ay % GLOBE_SPACE_FADE_DITHER_SIZE];

        int d_max = cx - lo > hi - cx ? cx - lo : hi - cx;

        // The two halves of the row mirror in sx, so one depth (sz) and one
        // sz/sy matrix contribution serve both pixels. sz shrinks
        // monotonically as d grows, so the seeded Newton sqrt can reuse the
        // previous pixel's depth as its starting point; the row seed below
        // is the depth at d = 0, an upper bound for the whole row.
        int32_t sz = weather_isqrt(plane_sq);
        for (int d = 0; d <= d_max; d++) {
            int32_t sx = ((int32_t)d * GLOBE_ROT_SCALE) / radius;
            sz = floor_sqrt_seeded(plane_sq - (sx * sx), sz);

            int32_t base_x = row_x + (m[2] * sz);
            int32_t base_y = row_y + (m[5] * sz);
            int32_t base_z = row_z + (m[8] * sz);
            int32_t col_x = m[0] * sx;
            int32_t col_y = m[3] * sx;
            int32_t col_z = m[6] * sx;
            int32_t light_base = light_row + (GLOBE_LIGHT_Z_Q10 * sz);
            int32_t light_col = GLOBE_LIGHT_X_Q10 * sx;

            for (int side = 0; side < 2; side++) {
                if (side) {
                    if (d == 0) break;
                    col_x = -col_x;
                    col_y = -col_y;
                    col_z = -col_z;
                    light_col = -light_col;
                }
                int ax = side ? cx - d : cx + d;
                if (ax < lo || ax > hi) continue;
                if (sz < GLOBE_HAZE_RIM_SZ_Q10) {
                    // Luminous rim: the atmosphere outshines the terrain at
                    // the very edge of the disc (merges with the halo ring),
                    // whitened into a fresnel glint where the limb faces the
                    // light — solid White arc with a 50%-dithered skirt.
                    int32_t rim_light =
                        (light_base + light_col) >> GLOBE_ROT_SHIFT;
                    uint8_t rim_color = GColorCelesteARGB8;
                    if (rim_light >= GLOBE_RIM_HOT_MIN_Q10 ||
                        (rim_light >= GLOBE_RIM_WARM_MIN_Q10 &&
                         dither_row[ax % GLOBE_SPACE_FADE_DITHER_SIZE] < 8)) {
                        rim_color = GColorWhiteARGB8;
                    }
                    prv_globe_px(ri.data, ax, ay, rim_color);
                    continue;
                }
                uint8_t tex = cubemap_sample(
                    cubemap,
                    (base_x + col_x) >> GLOBE_ROT_SHIFT,
                    (base_y + col_y) >> GLOBE_ROT_SHIFT,
                    (base_z + col_z) >> GLOBE_ROT_SHIFT);
                // Posterized lambert shading, dithered at the band edges.
                int32_t dith =
                    (int32_t)dither_row[ax % GLOBE_SPACE_FADE_DITHER_SIZE];
                int32_t light_pre = (light_base + light_col) >> GLOBE_ROT_SHIFT;
                int32_t light = light_pre + ((dith - 8) << 4);
                int band = light >= GLOBE_SHADE_LIT_MIN_Q10
                               ? 0
                               : (light >= GLOBE_SHADE_MID_MIN_Q10 ? 1 : 2);
                uint8_t color = GLOBE_SHADE_PALETTE[band][tex];
                if (band == 0 && light_pre >= GLOBE_GLINT_MIN_Q10 &&
                    ((dith * GLOBE_GLINT_SPAN_Q10) >> 4) <=
                        (light_pre - GLOBE_GLINT_MIN_Q10)) {
                    // Specular sun pool (screen-fixed; terrain scrolls under
                    // it) — water brightens one ramp step, land stays matte.
                    color = GLOBE_SHADE_PALETTE[3][tex];
                }
                if (sz < GLOBE_HAZE_BLEND_SZ_Q10) {
                    // Atmospheric wash over terrain near the horizon, dithered
                    // denser toward the rim; color + reach keyed by band (the
                    // sunlit limb blooms Celeste and reaches deeper — haze
                    // wins over the glint, so the rim stays atmospheric).
                    int32_t haze_span = GLOBE_HAZE_SPAN_BY_BAND[band];
                    if (((dith * haze_span) >> 4) >=
                        (sz - GLOBE_HAZE_RIM_SZ_Q10)) {
                        color = GLOBE_HAZE_WASH_BY_BAND[band];
                    }
                }
                prv_globe_px(ri.data, ax, ay, color);
            }
        }
    }

    draw_saved_location_pins(fb, view, center, radius, clipped);

    if (view->lock_pulse_anim) {
        // Lock pulse: a Celeste ring expanding from the settled pin's head
        // (raised 4px above center) and easing out — "target acquired".
        int eased = ease_out_quad(view->lock_pulse_progress);
        int outer_r = GLOBE_LOCK_PULSE_R_MIN_PX +
                      (GLOBE_LOCK_PULSE_R_GROW_PX * eased) /
                          ANIMATION_NORMALIZED_MAX;
        framebuffer_draw_ring(fb, GPoint(center.x, center.y - 5), outer_r,
                              outer_r - GLOBE_LOCK_PULSE_THICKNESS_PX,
                              GColorCelesteARGB8, clipped);
    }

    graphics_release_frame_buffer(ctx, fb);
}

static void draw_cubemap_globe(GContext *ctx, GlobeView *view, GPoint origin,
                               GSize frame_size, int scale_percent,
                               GRect clip_rect, bool clear_clip) {
    if (!view || !view->cubemap_data || scale_percent <= 0) return;

    int diameter = (GLOBE_RENDER_BASE_DIAMETER * scale_percent) / 100;
    if (diameter < 4) return;

    GPoint center = GPoint(origin.x + (frame_size.w / 2),
                           origin.y + (frame_size.h / 2) +
                               GLOBE_COLOR_PLANET_CENTER_Y_OFFSET);

    draw_cubemap_globe_at_center(ctx, view, center, scale_percent,
                                 clip_rect, clear_clip);
}

static void reload_current_frame(GlobeView *view) {
    if (!view) return;
    if (view->is_revealed && !view->is_revealing) {
        update_city_label_layer(view);
    }
    mark_dynamic_globe_dirty(view);
}

// Identical to reload_current_frame; keep one body.
#define reload_color_frame reload_current_frame

static void mark_dynamic_globe_dirty(GlobeView *view) {
    if (!view) return;

    if (view->is_revealed && !view->is_revealing && view->globe_layer) {
        layer_mark_dirty(view->globe_layer);
    } else if (view->canvas_layer) {
        layer_mark_dirty(view->canvas_layer);
    }
}

static int32_t ease_out_quad(AnimationProgress progress) {
    int32_t inv = ANIMATION_NORMALIZED_MAX - (int32_t)progress;
    return ANIMATION_NORMALIZED_MAX - weather_norm_square(inv);
}

#ifdef CONFIG_TOUCH
// Integer ease-out-back (s = 7/4, ~10.5% peak overshoot at t~=0.58):
// f(t) = 1 - (s+1)*inv^3 + s*inv^2, inv = 1-t. Fed to the hover-lock SERVO,
// values past MAX drive the pin slightly PAST center and back — a real
// magnetic snap that self-scales with capture distance (50px capture
// overshoots ~5px, 5px capture ~0.5px). Endpoints exact; defensively clamped.
// MUST only be used where a closed-loop correction follows (the hover branch)
// — never for open-loop lat/lon interpolation (10% of a long arc is huge).
static int32_t ease_out_back(AnimationProgress progress) {
    int32_t inv = ANIMATION_NORMALIZED_MAX - (int32_t)progress;
    if (inv <= 0) return ANIMATION_NORMALIZED_MAX;
    int32_t inv2 = weather_norm_square(inv);                               // inv^2/N
    int32_t inv3 = weather_scale_i32(inv2, inv, ANIMATION_NORMALIZED_MAX); // inv^3/N^2
    int32_t eased = ANIMATION_NORMALIZED_MAX + (7 * inv2 - 11 * inv3) / 4;
    int32_t cap = ANIMATION_NORMALIZED_MAX + ANIMATION_NORMALIZED_MAX / 8;
    return eased > cap ? cap : eased;
}
#endif

static AnimationProgress color_transition_grow_progress(int amount) {
    const int grow_start =
        (ANIMATION_NORMALIZED_MAX * GLOBE_COLOR_GROW_START_PERCENT) / 100;
    if (amount <= grow_start) {
        return 0;
    }

    return (AnimationProgress)weather_scale_i32(amount - grow_start,
                                                ANIMATION_NORMALIZED_MAX,
                                                ANIMATION_NORMALIZED_MAX -
                                                    grow_start);
}

static int color_transition_scale_percent(int amount) {
    AnimationProgress relative = color_transition_grow_progress(amount);
    if (relative <= 0) {
        return 0;
    }

    int eased = ease_out_quad(relative);
    return weather_scale_i32(eased, GLOBE_COLOR_FINAL_SCALE_PERCENT,
                             ANIMATION_NORMALIZED_MAX);
}

static int transition_color_amount(GlobeView *view) {
    int progress = (int)view->reveal_progress;
    return view->reveal_direction > 0
        ? progress
        : (ANIMATION_NORMALIZED_MAX - progress);
}

static int transition_bw_amount(GlobeView *view) {
    int amount = transition_color_amount(view);
    if (view && view->reveal_direction > 0) {
        amount = (amount * GLOBE_REVEAL_BW_SPEED_NUM) /
                 GLOBE_REVEAL_BW_SPEED_DEN;
        if (amount > ANIMATION_NORMALIZED_MAX) {
            amount = ANIMATION_NORMALIZED_MAX;
        }
    }
    return amount;
}

static int intro_selection_offset(GlobeView *view) {
    if (!view || view->intro_selection_ms == 0) return 0;
    return (view->intro_selection_ms * GLOBE_INTRO_SELECTION_OFFSET_PX) /
           GLOBE_INTRO_SELECTION_DURATION_MS;
}

static int bounce_offset(GlobeView *view) {
    if (!view || !view->bounce_anim) return 0;

    int p = (int)view->bounce_progress;
    int peak = p <= ANIMATION_NORMALIZED_MAX / 2
        ? p
        : ANIMATION_NORMALIZED_MAX - p;
    return (view->bounce_direction * GLOBE_CITY_BOUNCE_PX * peak * 2) /
           ANIMATION_NORMALIZED_MAX;
}

// One canceller for all Animation-slot fields: clears the slot first (the
// stopped handler checks ownership), then unschedules + destroys.
static void cancel_animation_slot(Animation **slot) {
    Animation *anim = *slot;
    if (!anim) return;
    *slot = NULL;
    animation_unschedule(anim);
    animation_destroy(anim);
}

static void cancel_timer_slot(AppTimer **slot) {
    if (*slot) {
        app_timer_cancel(*slot);
        *slot = NULL;
    }
}

static void cancel_city_animation(GlobeView *view) {
    if (view) cancel_animation_slot(&view->city_anim);
}

static void cancel_bounce_animation(GlobeView *view) {
    if (view) cancel_animation_slot(&view->bounce_anim);
}

#ifdef CONFIG_TOUCH
static void stop_globe_coast(GlobeView *view, bool mark_dirty) {
    if (!view) return;

    cancel_timer_slot(&view->coast_timer);
    view->coast_active = false;
    view->coast_velocity_x_q8 = 0;
    view->coast_velocity_y_q8 = 0;

    if (mark_dirty) {
        mark_dynamic_globe_dirty(view);
    }
}
#endif

static void notify_city_selected(GlobeView *view, bool force) {
    if (!view) return;

    SavedLocationEntry *entry = selected_saved_entry(view);
    if (entry && view->location_select_callback) {
        view->location_select_callback(entry->ds_index, force,
                                       view->location_select_context);
    }
}

static bool commit_hovered_location(GlobeView *view) {
    if (!view || !view->is_revealed || view->is_revealing) return false;

    if (view->city_anim) {
        set_color_orientation(view,
                              view->city_anim_target_latitude_e2,
                              view->city_anim_target_longitude_e2);
        cancel_city_animation(view);
    }

#ifdef CONFIG_TOUCH
    stop_globe_coast(view, false);
    if (view->is_free_roam) {
        int city_index = nearest_centered_city_index(view,
                                                     GLOBE_COMMIT_LOCK_RADIUS_PX);
        if (city_index < 0) {
            mark_dynamic_globe_dirty(view);
            return false;
        }
        view->selected_city_index = city_index;
        view->hover_city_index = city_index;
        view->hover_lock_active = true;
        set_color_orientation(view,
                              selected_city_latitude_e2(view),
                              selected_city_longitude_e2(view));
        set_free_roam_enabled(view, false);
    }
#endif

    update_city_label_layer(view);
    notify_city_selected(view, true);
    return true;
}

static void format_selected_label(GlobeView *view, char *buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) return;

    SavedLocationEntry *entry = selected_saved_entry(view);
    const char *label = "Saved Location";
    if (entry && entry->label[0]) {
        label = entry->label;
    } else if (entry && entry->is_current_location) {
        label = "Current Location";
    }

#if PBL_ROUND
    const char *first_comma = strchr(label, ',');
    if (first_comma) {
        const char *last_comma = strrchr(label, ',');
        const char *country_start = last_comma ? last_comma + 1 : first_comma + 1;
        while (*country_start == ' ') country_start++;

        size_t city_len = (size_t)(first_comma - label);
        while (city_len > 0 && label[city_len - 1] == ' ') city_len--;

        size_t country_len = 0;
        while (country_start[country_len]) country_len++;
        while (country_len > 0 && country_start[country_len - 1] == ' ') country_len--;

        const char *abbr = NULL;
        if (country_len == 13 && strncmp(country_start, "United States", 13) == 0) abbr = "USA";
        else if (country_len == 14 && strncmp(country_start, "United Kingdom", 14) == 0) abbr = "UK";
        else if (country_len == 9 && strncmp(country_start, "Australia", 9) == 0) abbr = "AUS";
        else if (country_len == 20 && strncmp(country_start, "United Arab Emirates", 20) == 0) abbr = "UAE";
        else if (country_len == 12 && strncmp(country_start, "South Africa", 12) == 0) abbr = "SA";

        if (city_len > 0 && (abbr || country_len > 0)) {
            if (abbr) {
                snprintf(buffer, buffer_size, "%.*s, %s", (int)city_len, label, abbr);
            } else {
                snprintf(buffer, buffer_size, "%.*s, %.*s",
                         (int)city_len, label,
                         (int)country_len, country_start);
            }
            buffer[buffer_size - 1] = '\0';
            return;
        }
    }
#endif

    strncpy(buffer, label, buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
}

static void update_city_label_layer(GlobeView *view) {
    if (!view || !view->city_label_layer) return;

    format_selected_label(view, view->city_label_text,
                          sizeof(view->city_label_text));
    text_layer_set_text(view->city_label_layer, view->city_label_text);
}

static void set_text_layer_hidden(TextLayer *text_layer, bool hidden) {
    if (text_layer) {
        layer_set_hidden(text_layer_get_layer(text_layer), hidden);
    }
}

#ifdef CONFIG_TOUCH
static void set_free_roam_enabled(GlobeView *view, bool enabled) {
    if (!view) return;

    if (enabled) {
        cancel_city_animation(view);
        cancel_bounce_animation(view);
        view->hover_lock_active = false;
        view->hover_city_index = -1;
    }

    view->is_free_roam = enabled;

    if (view->is_revealed && !view->is_revealing) {
        if (!enabled) {
            update_city_label_layer(view);
        }
        set_text_layer_hidden(view->city_label_layer,
                              enabled && !view->hover_lock_active);
        mark_dynamic_globe_dirty(view);
    }
}
#endif

static void show_intro_canvas(GlobeView *view) {
    if (!view) return;

    if (view->window) {
        window_set_background_color(view->window, GColorWhite);
    }
    if (view->canvas_layer) layer_set_hidden(view->canvas_layer, false);
    if (view->space_layer) layer_set_hidden(view->space_layer, true);
    if (view->globe_layer) layer_set_hidden(view->globe_layer, true);
    set_text_layer_hidden(view->city_label_layer, true);

    if (view->canvas_layer) layer_mark_dirty(view->canvas_layer);
}

static void show_revealed_space_layers(GlobeView *view) {
    if (!view) return;

    if (view->window) {
        window_set_background_color(view->window, GColorBlack);
    }
    update_city_label_layer(view);

    if (view->canvas_layer) layer_set_hidden(view->canvas_layer, true);
    if (view->space_layer) {
        layer_set_hidden(view->space_layer, false);
        layer_mark_dirty(view->space_layer);
    }
    if (view->globe_layer) {
        Layer *root_layer = window_get_root_layer(view->window);
        layer_set_frame(view->globe_layer,
                        revealed_globe_layer_frame(layer_get_bounds(root_layer)));
        layer_set_hidden(view->globe_layer, false);
        layer_mark_dirty(view->globe_layer);
    }
#ifdef CONFIG_TOUCH
    set_text_layer_hidden(view->city_label_layer,
                          view->is_free_roam && !view->hover_lock_active);
#else
    set_text_layer_hidden(view->city_label_layer, view->is_free_roam);
#endif
}

static void city_anim_update(Animation *anim, AnimationProgress progress) {
    GlobeView *view = (GlobeView *)animation_get_context(anim);
    if (!view) return;

    view->city_anim_progress = progress;

#ifdef CONFIG_TOUCH
    if (view->hover_lock_active && view->hover_city_index >= 0) {
        // Ease-out-BACK in the servo: eased exceeds MAX near the end, so
        // `remaining` goes briefly negative and the pin swings a few px PAST
        // center and settles back — a magnetic snap that self-scales with the
        // capture distance. (Safe only here: the servo re-corrects per frame.)
        int eased = ease_out_back(progress);
        int remaining = ANIMATION_NORMALIZED_MAX - eased;
        int target_dx = weather_scale_i32(view->city_anim_longitude_delta_e2,
                                          remaining,
                                          ANIMATION_NORMALIZED_MAX);
        int target_dy = weather_scale_i32(view->city_anim_latitude_delta_e2,
                                          remaining,
                                          ANIMATION_NORMALIZED_MAX);
        int current_dx;
        int current_dy;
        if (city_pin_offset_from_center(view, view->hover_city_index,
                                        &current_dx, &current_dy)) {
            int correction_x = target_dx - current_dx;
            int correction_y = target_dy - current_dy;
            if (correction_x || correction_y) {
                apply_globe_screen_delta_q8(
                    view,
                    (int32_t)correction_x * GLOBE_Q8_ONE,
                    (int32_t)correction_y * GLOBE_Q8_ONE);
                return;
            }
        }
        mark_dynamic_globe_dirty(view);
        return;
    }
#endif

    int eased = ease_out_quad(progress);
    int32_t latitude_e2 = view->city_anim_start_latitude_e2 +
        weather_scale_i32(view->city_anim_latitude_delta_e2, eased,
                          ANIMATION_NORMALIZED_MAX);
    int32_t longitude_e2 = view->city_anim_start_longitude_e2 +
        weather_scale_i32(view->city_anim_longitude_delta_e2, eased,
                          ANIMATION_NORMALIZED_MAX);

    set_color_orientation(view, latitude_e2, longitude_e2);
    mark_dynamic_globe_dirty(view);
}

static void city_anim_stopped(Animation *anim, bool finished, void *context) {
    GlobeView *view = (GlobeView *)context;
    if (!view) return;

    bool owns_anim = view->city_anim == anim;
    if (owns_anim) {
        view->city_anim = NULL;
    }

#ifdef CONFIG_TOUCH
    if (view->hover_lock_active && view->hover_city_index >= 0) {
        if (owns_anim && finished) {
            settle_city_pin_to_center(view, view->hover_city_index);
            update_city_label_layer(view);
            start_lock_pulse(view);   // ring flash: target acquired
            mark_dynamic_globe_dirty(view);
        }
        if (owns_anim) {
            animation_destroy(anim);
        }
        return;
    }
#endif

    if (owns_anim && finished) {
        set_color_orientation(view,
                              view->city_anim_target_latitude_e2,
                              view->city_anim_target_longitude_e2);
        reload_color_frame(view);
        start_lock_pulse(view);   // button-nav arrival gets the same pulse
    }

    if (owns_anim) {
        animation_destroy(anim);
    }
}

static const AnimationImplementation s_city_anim_impl = {
    .update = city_anim_update
};

static void bounce_anim_update(Animation *anim, AnimationProgress progress) {
    GlobeView *view = (GlobeView *)animation_get_context(anim);
    if (!view) return;

    view->bounce_progress = progress;
    mark_dynamic_globe_dirty(view);
}

static void bounce_anim_stopped(Animation *anim, bool finished, void *context) {
    (void)finished;
    GlobeView *view = (GlobeView *)context;
    if (!view) return;

    bool owns_anim = view->bounce_anim == anim;
    if (owns_anim) {
        view->bounce_anim = NULL;
    }
    view->bounce_progress = 0;
    mark_dynamic_globe_dirty(view);

    if (owns_anim) {
        animation_destroy(anim);
    }
}

static const AnimationImplementation s_bounce_anim_impl = {
    .update = bounce_anim_update
};

// ---- Lock pulse: an expanding Celeste ring from the locked pin's head ----
static void lock_pulse_anim_update(Animation *anim, AnimationProgress progress) {
    GlobeView *view = (GlobeView *)animation_get_context(anim);
    if (!view) return;

    view->lock_pulse_progress = progress;
    mark_dynamic_globe_dirty(view);
}

static void lock_pulse_anim_stopped(Animation *anim, bool finished, void *context) {
    (void)finished;
    GlobeView *view = (GlobeView *)context;
    if (!view) return;

    bool owns_anim = view->lock_pulse_anim == anim;
    if (owns_anim) {
        view->lock_pulse_anim = NULL;
    }
    view->lock_pulse_progress = 0;
    mark_dynamic_globe_dirty(view);

    if (owns_anim) {
        animation_destroy(anim);
    }
}

static const AnimationImplementation s_lock_pulse_anim_impl = {
    .update = lock_pulse_anim_update
};

static void cancel_lock_pulse_animation(GlobeView *view) {
    if (view) cancel_animation_slot(&view->lock_pulse_anim);
}

static Animation *start_view_animation(GlobeView *view, uint32_t duration_ms,
                                       AnimationCurve curve,
                                       const AnimationImplementation *impl,
                                       AnimationStoppedHandler stopped);

static void start_lock_pulse(GlobeView *view) {
    if (!view) return;
    cancel_lock_pulse_animation(view);
    view->lock_pulse_progress = 0;
    view->lock_pulse_anim = start_view_animation(view,
                                                 GLOBE_LOCK_PULSE_DURATION_MS,
                                                 AnimationCurveLinear,
                                                 &s_lock_pulse_anim_impl,
                                                 lock_pulse_anim_stopped);
    // The haptic nudge, landing with the ring. Emery keeps the original whisper tick
    // (25ms @ 25%). Round runs FIRMER (45ms @ 70%): the softer pulse was imperceptible on
    // the round hardware — a pulse this short at low amplitude can sit under the vibe
    // motor's start-up threshold, so it must be long/strong enough to actually spin up.
    // Still a tick, nowhere near a notification buzz (those are 100s of ms at 100%).
    static const uint32_t nudge_ms[]  = { PBL_IF_ROUND_ELSE(45, 25) };
    static const uint32_t nudge_amp[] = { PBL_IF_ROUND_ELSE(70, 25) };
    vibes_enqueue_custom_pattern_with_amplitudes((VibePatternWithAmplitudes){
        .durations = nudge_ms,
        .amplitudes = nudge_amp,
        .num_segments = 1,
    });
}

// Shared boilerplate for the view's stopped-handler animations: create,
// configure, schedule. Returns the animation (NULL if creation failed) so
// callers can store it in their slot; behavior matches the previous inline
// sequences (applib animation_* functions tolerate NULL).
static Animation *start_view_animation(GlobeView *view, uint32_t duration_ms,
                                       AnimationCurve curve,
                                       const AnimationImplementation *impl,
                                       AnimationStoppedHandler stopped) {
    Animation *anim = animation_create();
    animation_set_duration(anim, duration_ms);
    animation_set_curve(anim, curve);
    animation_set_implementation(anim, impl);
    animation_set_handlers(anim, (AnimationHandlers){ .stopped = stopped },
                           view);
    animation_schedule(anim);
    return anim;
}

static void start_city_bounce(GlobeView *view, bool is_down) {
    if (!view) return;

    cancel_bounce_animation(view);
    view->bounce_direction = is_down ? -1 : 1;
    view->bounce_progress = 0;
    view->bounce_anim = start_view_animation(view,
                                             GLOBE_CITY_BOUNCE_DURATION_MS,
                                             AnimationCurveEaseOut,
                                             &s_bounce_anim_impl,
                                             bounce_anim_stopped);
}

static void start_city_rotation(GlobeView *view, int city_index) {
    if (!view || view->city_anim) return;

    if (!selected_city_is_valid(view, city_index)) return;

#ifdef CONFIG_TOUCH
    stop_globe_coast(view, false);
#endif
    cancel_bounce_animation(view);
    view->selected_city_index = city_index;
#ifdef CONFIG_TOUCH
    set_free_roam_enabled(view, false);
    view->hover_lock_active = false;
    view->hover_city_index = -1;
#endif
    view->city_anim_start_latitude_e2 = view->color_latitude_e2;
    view->city_anim_start_longitude_e2 = view->color_longitude_e2;
    view->city_anim_target_latitude_e2 = selected_city_latitude_e2(view);
    view->city_anim_target_longitude_e2 = selected_city_longitude_e2(view);
    view->city_anim_latitude_delta_e2 = view->city_anim_target_latitude_e2 -
                                        view->city_anim_start_latitude_e2;
    view->city_anim_longitude_delta_e2 = shortest_longitude_delta_e2(
        view->city_anim_start_longitude_e2,
        view->city_anim_target_longitude_e2);
    view->city_anim_progress = 0;

    if (view->city_anim_longitude_delta_e2 == 0 &&
        view->city_anim_latitude_delta_e2 == 0) {
        update_city_label_layer(view);
        mark_dynamic_globe_dirty(view);
        return;
    }

    // Linear: city_anim_update applies ease_out_quad itself — a curved
    // AnimationCurve here would double-ease into mush (brisk start, crisp
    // landing is the point; the arrival pulse supplies the punctuation).
    view->city_anim = start_view_animation(view,
                                           GLOBE_CITY_ROTATION_DURATION_MS,
                                           AnimationCurveLinear,
                                           &s_city_anim_impl,
                                           city_anim_stopped);
    update_city_label_layer(view);
    mark_dynamic_globe_dirty(view);
}

#ifdef CONFIG_TOUCH
// vx_q8/vy_q8: the coast velocity at the call (zeros when called at rest).
// A pin still moving AWAY from center is not captured — the coast tick
// retries at dead-stop, so a retrograde fling locks gently from standstill
// instead of being yanked mid-flight.
static bool try_start_magnetic_city_lock_v(GlobeView *view, int radius_px,
                                           int32_t vx_q8, int32_t vy_q8) {
    if (!view || view->city_anim) return false;

    int city_index = nearest_centered_city_index(view, radius_px);
    if (city_index < 0) return false;

    int start_dx = 0;
    int start_dy = 0;
    bool has_offset = city_pin_offset_from_center(view, city_index,
                                                 &start_dx, &start_dy);
    int deadzone_sq = GLOBE_LOCK_SETTLE_DEADZONE_PX *
                      GLOBE_LOCK_SETTLE_DEADZONE_PX;

    if ((vx_q8 | vy_q8) != 0 && has_offset) {
        // Directional gate: coast deltas move the pin WITH the velocity, so a
        // positive dot product = the pin is receding from center — defer.
        int32_t dot = vx_q8 * start_dx + vy_q8 * start_dy;
        if (dot > 0) return false;
    }

    stop_globe_coast(view, false);
    cancel_bounce_animation(view);
    view->selected_city_index = city_index;
    view->hover_city_index = city_index;
    view->hover_lock_active = true;
    set_free_roam_enabled(view, false);

    if (!has_offset || (start_dx * start_dx) + (start_dy * start_dy) <=
        deadzone_sq) {
        start_lock_pulse(view);   // instant lock (already centered)
        mark_dynamic_globe_dirty(view);
        return true;
    }

    view->city_anim_start_latitude_e2 = view->color_latitude_e2;
    view->city_anim_start_longitude_e2 = view->color_longitude_e2;
    view->city_anim_target_latitude_e2 = view->color_latitude_e2;
    view->city_anim_target_longitude_e2 = view->color_longitude_e2;
    view->city_anim_latitude_delta_e2 = start_dy;
    view->city_anim_longitude_delta_e2 = start_dx;
    view->city_anim_progress = 0;

    // Swoop duration proportional to the capture distance: a 5px capture no
    // longer crawls and a 49px capture no longer whips.
    int dist_px = weather_isqrt(start_dx * start_dx + start_dy * start_dy);
    uint32_t swoop_ms = GLOBE_LOCK_SWOOP_BASE_MS +
                        (uint32_t)dist_px * GLOBE_LOCK_SWOOP_MS_PER_PX;
    if (swoop_ms < GLOBE_LOCK_SWOOP_MIN_MS) swoop_ms = GLOBE_LOCK_SWOOP_MIN_MS;
    if (swoop_ms > GLOBE_LOCK_SWOOP_DURATION_MS) {
        swoop_ms = GLOBE_LOCK_SWOOP_DURATION_MS;
    }

    // Linear is MANDATORY: city_anim_update's servo applies ease_out_back
    // itself; a curved AnimationCurve would compose with it and distort.
    view->city_anim = start_view_animation(view,
                                           swoop_ms,
                                           AnimationCurveLinear,
                                           &s_city_anim_impl,
                                           city_anim_stopped);
    if (!view->city_anim) {
        settle_city_pin_to_center(view, city_index);
        mark_dynamic_globe_dirty(view);
    }
    return true;
}

static bool try_start_magnetic_city_lock(GlobeView *view, int radius_px) {
    return try_start_magnetic_city_lock_v(view, radius_px, 0, 0);
}
#endif

static void navigate_city(GlobeView *view, bool is_down) {
    if (!view || view->city_anim) return;

#ifdef CONFIG_TOUCH
    stop_globe_coast(view, false);
    if (view->is_free_roam) {
        int nearest_index = nearest_centered_city_index(view, revealed_globe_radius());
        view->selected_city_index = nearest_index >= 0
            ? nearest_index
            : nearest_city_index_for_orientation(view);
        set_free_roam_enabled(view, false);
    }
#endif

    int current_index = view->selected_city_index;
    int max_index = globe_max_selector_index(view);
    int next_index = current_index;
    int selector_count = view->saved_entry_count > 0 ? view->saved_entry_count : 1;
    for (int attempt = 0; attempt < selector_count; attempt++) {
        next_index += is_down ? 1 : -1;
        if (next_index > max_index) {
            next_index = 0;
        } else if (next_index < 0) {
            next_index = max_index;
        }

        if (selected_city_is_valid(view, next_index)) {
            start_city_rotation(view, next_index);
            return;
        }
    }

    start_city_bounce(view, is_down);
}

#ifdef CONFIG_TOUCH
__attribute__((noinline)) static int32_t clamp_globe_velocity_q8(int32_t value) {
    if (value > GLOBE_COAST_MAX_SPEED_Q8) return GLOBE_COAST_MAX_SPEED_Q8;
    if (value < -GLOBE_COAST_MAX_SPEED_Q8) return -GLOBE_COAST_MAX_SPEED_Q8;
    return value;
}

static void apply_globe_screen_delta_q8(GlobeView *view,
                                        int32_t dx_q8,
                                        int32_t dy_q8) {
    if (!view || !view->is_revealed || view->is_revealing) return;
    if (dx_q8 == 0 && dy_q8 == 0) {
        return;
    }

    int yaw_angle = -(dx_q8 * GLOBE_DRAG_TRIGANGLE_PER_PX) / GLOBE_Q8_ONE;
    int pitch_angle = -(dy_q8 * GLOBE_DRAG_TRIGANGLE_PER_PX) / GLOBE_Q8_ONE;
    if (yaw_angle == 0 && pitch_angle == 0) return;

    int32_t delta[9];
    int32_t next[9];
    matrix_screen_rotation(delta, yaw_angle, pitch_angle);
    matrix_multiply(next, view->globe_rotation, delta);

    matrix_copy(view->globe_rotation, next);
    normalize_globe_rotation(view);

    view->color_longitude_e2 = normalize_longitude_e2(
        view->color_longitude_e2 - ((dx_q8 * 86) / GLOBE_Q8_ONE));
    view->color_latitude_e2 = clamp_latitude_e2(
        view->color_latitude_e2 - ((dy_q8 * 86) / GLOBE_Q8_ONE));
    view->starfield_offset_x_q8 -= (dx_q8 * 34400) / 36000;
    view->starfield_offset_y_q8 -= dy_q8;
    view->current_frame = bw_frame_for_longitude_e2((int16_t)view->color_longitude_e2);
    sync_bw_elapsed_to_current_frame(view);
    mark_dynamic_globe_dirty(view);
}

static void begin_globe_drag_capture(GlobeView *view,
                                     int16_t x,
                                     int16_t y,
                                     bool whole_screen) {
    if (!view || !view->is_revealed || view->is_revealing) return;

    view->touch_down_on_globe = point_is_on_revealed_globe(view, x, y);
    view->touch_controls_globe = whole_screen || view->touch_down_on_globe;
    if (!view->touch_controls_globe) return;

    stop_globe_coast(view, false);
    cancel_city_animation(view);
    cancel_bounce_animation(view);
    set_free_roam_enabled(view, true);
    view->touch_start_x = x;
    view->touch_start_y = y;
    view->touch_last_x = x;
    view->touch_last_y = y;
    view->touch_velocity_x_q8 = 0;
    view->touch_velocity_y_q8 = 0;
    view->coast_velocity_x_q8 = 0;
    view->coast_velocity_y_q8 = 0;
    view->touch_drag_axis_set = false;
    view->touch_drag_rotated = false;
}

static void update_globe_drag(GlobeView *view, int16_t x, int16_t y) {
    if (!view || !view->touch_controls_globe ||
        !view->is_revealed || view->is_revealing) return;

    int16_t dx = x - view->touch_last_x;
    int16_t dy = y - view->touch_last_y;
    if (dx == 0 && dy == 0) return;

    apply_globe_screen_delta_q8(view,
                                (int32_t)dx * GLOBE_Q8_ONE,
                                (int32_t)dy * GLOBE_Q8_ONE);

    view->touch_last_x = x;
    view->touch_last_y = y;
    view->touch_velocity_x_q8 = clamp_globe_velocity_q8(
        ((view->touch_velocity_x_q8 * 3) + ((int32_t)dx * GLOBE_Q8_ONE)) / 4);
    view->touch_velocity_y_q8 = clamp_globe_velocity_q8(
        ((view->touch_velocity_y_q8 * 3) + ((int32_t)dy * GLOBE_Q8_ONE)) / 4);
    view->coast_velocity_x_q8 = view->touch_velocity_x_q8;
    view->coast_velocity_y_q8 = view->touch_velocity_y_q8;

    int16_t total_dx = x - view->touch_start_x;
    int16_t total_dy = y - view->touch_start_y;
    int16_t total_adx = total_dx < 0 ? -total_dx : total_dx;
    int16_t total_ady = total_dy < 0 ? -total_dy : total_dy;
    if (total_adx > GLOBE_DRAG_AXIS_THRESHOLD ||
        total_ady > GLOBE_DRAG_AXIS_THRESHOLD) {
        view->touch_drag_axis_set = true;
        view->touch_drag_rotated = true;
    }
}

static void globe_coast_timer_handler(void *context) {
    GlobeView *view = (GlobeView *)context;
    if (!view) return;

    view->coast_timer = NULL;
    if (!view->coast_active || view->touch_active ||
        !view->is_revealed || view->is_revealing) {
        view->coast_active = false;
        view->coast_velocity_x_q8 = 0;
        view->coast_velocity_y_q8 = 0;
        return;
    }

    apply_globe_screen_delta_q8(view,
                                (view->coast_velocity_x_q8 *
                                 GLOBE_COAST_STEP_NUM) /
                                    GLOBE_COAST_STEP_DEN,
                                (view->coast_velocity_y_q8 *
                                 GLOBE_COAST_STEP_NUM) /
                                    GLOBE_COAST_STEP_DEN);

    view->coast_velocity_x_q8 =
        (view->coast_velocity_x_q8 * GLOBE_COAST_DECAY_NUM) /
        GLOBE_COAST_DECAY_DEN;
    view->coast_velocity_y_q8 =
        (view->coast_velocity_y_q8 * GLOBE_COAST_DECAY_NUM) /
        GLOBE_COAST_DECAY_DEN;

    int32_t speed = abs_i32(view->coast_velocity_x_q8) +
                    abs_i32(view->coast_velocity_y_q8);
    if (speed <= GLOBE_COAST_SETTLE_SPEED_Q8 &&
        try_start_magnetic_city_lock_v(view, GLOBE_LOCK_RADIUS_PX,
                                       view->coast_velocity_x_q8,
                                       view->coast_velocity_y_q8)) {
        return;
    }

    if (speed <= GLOBE_COAST_MIN_SPEED_Q8) {
        view->coast_active = false;
        view->coast_velocity_x_q8 = 0;
        view->coast_velocity_y_q8 = 0;
        // Dead-stop retry: a lock deferred by the directional gate (pin was
        // still receding mid-flight) captures gently now that we're still.
        if (try_start_magnetic_city_lock(view, GLOBE_LOCK_RADIUS_PX)) {
            return;
        }
        mark_dynamic_globe_dirty(view);
        return;
    }

    view->coast_timer = app_timer_register(GLOBE_COAST_FRAME_MS,
                                           globe_coast_timer_handler,
                                           view);
}

static bool start_globe_coast(GlobeView *view) {
    if (!view || !view->is_revealed || view->is_revealing) return false;

    int32_t vx = clamp_globe_velocity_q8(view->touch_velocity_x_q8 * 5 / 4);
    int32_t vy = clamp_globe_velocity_q8(view->touch_velocity_y_q8 * 5 / 4);
    int32_t speed = abs_i32(vx) + abs_i32(vy);
    if (speed < GLOBE_COAST_START_SPEED_Q8) {
        view->coast_active = false;
        view->coast_velocity_x_q8 = 0;
        view->coast_velocity_y_q8 = 0;
        return false;
    }

    stop_globe_coast(view, false);
    view->coast_velocity_x_q8 = vx;
    view->coast_velocity_y_q8 = vy;
    view->coast_active = true;
    view->coast_timer = app_timer_register(GLOBE_COAST_FRAME_MS,
                                           globe_coast_timer_handler,
                                           view);
    return true;
}
#endif

static void reveal_anim_update(Animation *anim, AnimationProgress progress) {
    GlobeView *view = (GlobeView *)animation_get_context(anim);
    if (!view) return;

    view->reveal_progress = progress;
    int32_t next_latitude_e2;
    int32_t next_longitude_e2;
    if (view->reveal_direction > 0) {
        int eased = ease_out_quad(progress);
        int32_t lon_delta = shortest_longitude_delta_e2(
            view->transition_color_start_longitude_e2,
            view->transition_color_target_longitude_e2);
        int32_t lat_delta = view->transition_color_target_latitude_e2 -
                            view->transition_color_start_latitude_e2;
        next_latitude_e2 = view->transition_color_start_latitude_e2 +
            weather_scale_i32(lat_delta, eased, ANIMATION_NORMALIZED_MAX);
        int32_t base_longitude_e2 = view->transition_color_start_longitude_e2 +
            weather_scale_i32(lon_delta, eased, ANIMATION_NORMALIZED_MAX);
        int spin_eased = ease_out_quad(color_transition_grow_progress(progress));
        int32_t reveal_spin_e2 =
            weather_scale_i32(GLOBE_REVEAL_COLOR_SPIN_DEGREES_E2,
                              spin_eased, ANIMATION_NORMALIZED_MAX);
        next_longitude_e2 = normalize_longitude_e2(base_longitude_e2 - reveal_spin_e2);
    } else {
        next_latitude_e2 = 0;
        next_longitude_e2 = normalize_longitude_e2(
            view->transition_color_start_longitude_e2 -
            weather_scale_i32((int32_t)progress,
                              GLOBE_REVEAL_SPIN_STEPS * 1500,
                              ANIMATION_NORMALIZED_MAX));
    }

    set_color_orientation(view, next_latitude_e2, next_longitude_e2);
    if (view->reveal_direction < 0) {
        view->transition_bw_frame = view->current_frame;
    }
    layer_mark_dirty(view->canvas_layer);
}

static void reveal_anim_stopped(Animation *anim, bool finished, void *context) {
    GlobeView *view = (GlobeView *)context;
    if (!view) return;

    bool owns_anim = view->reveal_anim == anim;
    if (owns_anim) {
        view->reveal_anim = NULL;
    }
    view->is_revealing = false;

    if (owns_anim && finished) {
        view->is_revealed = view->reveal_direction > 0;
        view->reveal_progress = 0;
        if (view->is_revealed) {
            set_color_orientation(view,
                                  view->transition_color_target_latitude_e2,
                                  view->transition_color_target_longitude_e2);
            cancel_timer_slot(&view->animation_timer);
            show_revealed_space_layers(view);
        } else {
            view->is_free_roam = false;
            int landed_bw_frame = bw_frame_index_for_view(view, view->current_frame);
            view->transition_bw_frame = landed_bw_frame;
            set_color_orientation(view, 0,
                                  longitude_e2_for_bw_frame(landed_bw_frame));
            view->current_frame = landed_bw_frame;
            sync_bw_elapsed_to_current_frame(view);
            view->bw_idle = false;
            view->bw_idle_slowdown_step = 0;
            if (view->is_animating && !view->animation_timer) {
                schedule_frame_timer(view);
            }
            start_globe_idle_timer(view);
            show_intro_canvas(view);
        }
    }

    if (owns_anim) {
        animation_destroy(anim);
    }
}

static const AnimationImplementation s_reveal_impl = {
    .update = reveal_anim_update
};

static void cancel_reveal_animation(GlobeView *view) {
    if (view) cancel_animation_slot(&view->reveal_anim);
}

static void cancel_all_view_animations(GlobeView *view) {
    cancel_reveal_animation(view);
    cancel_city_animation(view);
    cancel_bounce_animation(view);
    cancel_lock_pulse_animation(view);
}

static void toggle_reveal(GlobeView *view) {
    if (!view) return;

    if (view->is_revealing) {
        return;
    }

#ifdef CONFIG_TOUCH
    stop_globe_coast(view, false);
    view->hover_lock_active = false;
#endif

    if (!view->is_revealed) {
        globe_view_reload_saved_locations(view);
    }

    view->reveal_direction = view->is_revealed ? -1 : 1;
    view->is_revealing = true;
    view->transition_bw_frame = view->current_frame;
    set_color_orientation(view, 0,
                          longitude_e2_for_bw_frame(view->current_frame));
    view->transition_color_start_latitude_e2 = view->color_latitude_e2;
    view->transition_color_start_longitude_e2 = view->color_longitude_e2;
    view->transition_color_target_latitude_e2 = selected_city_latitude_e2(view);
    view->transition_color_target_longitude_e2 = selected_city_longitude_e2(view);
    view->reveal_progress = 0;
    show_intro_canvas(view);

    cancel_all_view_animations(view);
    view->reveal_anim = start_view_animation(view,
                                             view->reveal_direction < 0
                                                 ? GLOBE_REVEAL_REVERSE_DURATION_MS
                                                 : GLOBE_REVEAL_DURATION_MS,
                                             AnimationCurveEaseOut,
                                             &s_reveal_impl,
                                             reveal_anim_stopped);
}

static void schedule_frame_timer(GlobeView *view) {
    view->animation_timer = app_timer_register(GLOBE_FRAME_INTERVAL_MS,
                                               animation_timer_handler,
                                               view);
}

static void note_globe_interaction(GlobeView *view) {
    if (!view) return;
    cancel_timer_slot(&view->idle_timer);
    view->bw_idle = false;
    view->bw_idle_slowdown_step = 0;
    if (view->is_animating && !view->is_revealed &&
        !view->animation_timer) {
        schedule_frame_timer(view);
    }
    start_globe_idle_timer(view);
}

static void globe_idle_timer_handler(void *context) {
    GlobeView *view = (GlobeView *)context;
    if (!view) return;
    view->idle_timer = NULL;
    if (!view->is_animating || view->is_revealed || view->is_revealing) {
        return;
    }
    view->bw_idle = true;
    view->bw_idle_slowdown_step = 0;
    if (!view->animation_timer) {
        schedule_frame_timer(view);
    }
}

static void start_globe_idle_timer(GlobeView *view) {
    if (!view || view->idle_timer) return;
    view->idle_timer = app_timer_register(GLOBE_IDLE_TIMEOUT_MS,
                                          globe_idle_timer_handler,
                                          view);
}

/**
 * Animation timer callback - advances to next frame
 */
static void animation_timer_handler(void *context) {
    GlobeView *view = (GlobeView *)context;

    if (!view || !view->is_animating || view->is_revealed) {
        if (view) view->animation_timer = NULL;
        return;
    }

    view->animation_timer = NULL;

    if (!view->is_revealing) {
        if (view->intro_selection_ms > GLOBE_FRAME_INTERVAL_MS) {
            view->intro_selection_ms -= GLOBE_FRAME_INTERVAL_MS;
        } else {
            view->intro_selection_ms = 0;
        }
        if (!view->bw_idle ||
            view->bw_idle_slowdown_step < GLOBE_IDLE_SLOWDOWN_STEPS) {
            uint32_t duration = bw_sequence_duration_for_view(view);
            view->bw_elapsed_ms = duration > 0
                ? (view->bw_elapsed_ms + GLOBE_FRAME_INTERVAL_MS) % duration
                : 0;
            view->current_frame = bw_frame_index_for_view(
                view,
                (int)(view->bw_elapsed_ms / GLOBE_FRAME_INTERVAL_MS));
            if (view->canvas_layer) {
                layer_mark_dirty(view->canvas_layer);
            }
        }
    }

    if (view->bw_idle && !view->is_revealing) {
        if (view->bw_idle_slowdown_step >= GLOBE_IDLE_SLOWDOWN_STEPS) {
            return;
        }
        view->bw_idle_slowdown_step++;
        uint32_t interval = GLOBE_FRAME_INTERVAL_MS +
            (uint32_t)view->bw_idle_slowdown_step *
            GLOBE_IDLE_SLOWDOWN_STEP_MS;
        view->animation_timer = app_timer_register(
            interval,
            animation_timer_handler,
            view);
    } else {
        schedule_frame_timer(view);
    }
}

static void draw_transition_state(GContext *ctx, GlobeView *view, GPoint origin,
                                  GSize frame_size, GRect bounds) {
    if (!view->bw_sequence) return;
    if (!view->cradle_pdc) return;

    const int color_amount = transition_color_amount(view);
    const int bw_amount = transition_bw_amount(view);
    const int eased = ease_out_quad((AnimationProgress)bw_amount);
    const int drop = (eased * GLOBE_CRADLE_DROP_PX) / ANIMATION_NORMALIZED_MAX;

    if (view->reveal_direction > 0) {
        draw_forward_space_fade(ctx, bounds, color_amount, view);
    }

    draw_cradle(ctx, view, GPoint(origin.x, origin.y + drop), frame_size);

    int wipe_h = (frame_size.h * bw_amount) / ANIMATION_NORMALIZED_MAX;
    if (wipe_h > 0) {
        GRect wipe_rect =
            GRect(origin.x - GLOBE_CRADLE_WIPE_MARGIN,
                  origin.y + drop - GLOBE_CRADLE_WIPE_MARGIN,
                  frame_size.w + (GLOBE_CRADLE_WIPE_MARGIN * 2),
                  wipe_h + (GLOBE_CRADLE_WIPE_MARGIN * 2));

        graphics_context_set_fill_color(ctx, GColorWhite);
        graphics_fill_rect(ctx, wipe_rect,
                           0, GCornerNone);
        if (view->reveal_direction > 0) {
            draw_forward_space_fade(ctx, wipe_rect, color_amount, view);
        }
    }

    draw_cubemap_globe(ctx, view, origin, frame_size,
                       color_transition_scale_percent(color_amount),
                       bounds, false);

    if (!draw_bw_crumple_frame(ctx, view, origin, frame_size, bw_amount) &&
        bw_amount <= 0) {
        draw_bw_frame(ctx, view, origin, frame_size);
    }
}

static GRect city_label_frame_for_bounds(GRect bounds) {
    int side = GLOBE_CITY_LABEL_ROUND_SIDE_INSET;
    return GRect(side,
                 bounds.size.h - GLOBE_CITY_LABEL_HEIGHT -
                     GLOBE_CITY_LABEL_ROUND_BOTTOM_INSET,
                 bounds.size.w - (side * 2),
                 GLOBE_CITY_LABEL_HEIGHT);
}

static void draw_city_label(GContext *ctx, GlobeView *view, GRect bounds) {
    char label[48];
    format_selected_label(view, label, sizeof(label));

    GRect label_rect = city_label_frame_for_bounds(bounds);
    graphics_context_set_text_color(ctx, GColorWhite);
    graphics_draw_text(ctx, label,
                       fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                       GRect(label_rect.origin.x + 4, label_rect.origin.y + 3,
                             label_rect.size.w - 8, label_rect.size.h - 3),
                       GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentCenter,
                       NULL);
}

static void draw_intro_title(GContext *ctx, GRect bounds, int globe_y,
                             GSize frame_size) {
#if !PBL_ROUND
    (void)globe_y;
    (void)frame_size;

    const int header_height = GLOBE_SMALL_RECT ? 24 : 38;
    graphics_context_set_text_color(ctx, GColorBlack);
    graphics_draw_text(ctx, "CITY SELECT",
                       fonts_get_system_font(GLOBE_SMALL_RECT ? FONT_KEY_GOTHIC_18_BOLD
                                                              : FONT_KEY_GOTHIC_28_BOLD),
                       GRect(0, -2, bounds.size.w, header_height + 2),
                       GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentCenter,
                       NULL);
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_rect(ctx,
                       GRect(0, header_height - 2, bounds.size.w, 2),
                       0, GCornerNone);
#else
    const char *title = "CITY SELECT";
    GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD);
    int planet_center_y = globe_y + frame_size.h / 2 +
                          GLOBE_PLANET_CENTER_Y_OFFSET;
    int planet_top = planet_center_y - GLOBE_RENDER_BASE_DIAMETER / 2;
    int title_y = (planet_top - GLOBE_INTRO_TITLE_HEIGHT) / 2;
    if (title_y < 0) title_y = 0;

    graphics_context_set_text_color(ctx, GColorBlack);
    graphics_draw_text(ctx, title,
                       font,
                       GRect(0, title_y - 4,
                             bounds.size.w,
                             GLOBE_INTRO_TITLE_HEIGHT + 8),
                       GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentCenter,
                       NULL);
    // Divider under the title (round port of the rect city-select redesign): the same
    // 2px black rule, FULL WIDTH — the round framebuffer clips each row to the glass,
    // so drawing edge to edge lands it bezel-to-bezel. Rides title_y like the title.
    {
      const int rule_y = title_y + GLOBE_INTRO_TITLE_HEIGHT - 2;
      graphics_context_set_fill_color(ctx, GColorBlack);
      graphics_fill_rect(ctx, GRect(0, rule_y, bounds.size.w, 2), 0, GCornerNone);
    }
#endif
}

// Map-pin artwork lifted from the world cup app's location_pin.pdc (18x22 viewbox,
// resources/svg/location/location_pin.svg): outer teardrop filled in `color`, inner
// octagonal window in `bg_color`. Embedded as GPaths so both label states (black-on-
// white / white-on-cerulean) recolor for free.
static void draw_saved_locations_pin(GContext *ctx, GPoint origin,
                                     GColor color, GColor bg_color) {
    static const GPoint kPinBody[] = {
        {8, 1}, {10, 1}, {11, 2}, {13, 3}, {15, 5}, {16, 7}, {16, 10},
        {15, 11}, {15, 13}, {14, 14}, {13, 16}, {10, 19}, {10, 20}, {9, 21},
        {8, 20}, {8, 19}, {5, 16}, {4, 14}, {3, 13}, {3, 11}, {2, 10},
        {2, 7}, {3, 5}, {5, 3}, {7, 2},
    };
    static const GPoint kPinWindow[] = {
        {10, 5}, {12, 7}, {12, 9}, {10, 11}, {8, 11}, {6, 9}, {6, 7}, {8, 5},
    };
    // The art is 22px tall in an 18px (GLOBE_SAVED_COG_SIZE) slot: lift 2px so it
    // optically centres in the 34px label row.
    const GPoint off = GPoint(origin.x, origin.y - 2);
    GPath body = {
        .num_points = sizeof(kPinBody) / sizeof(kPinBody[0]),
        .points = (GPoint *)kPinBody,
        .offset = off,
    };
    GPath window = {
        .num_points = sizeof(kPinWindow) / sizeof(kPinWindow[0]),
        .points = (GPoint *)kPinWindow,
        .offset = off,
    };

    graphics_context_set_antialiased(ctx, false);
    graphics_context_set_fill_color(ctx, color);
    gpath_draw_filled(ctx, &body);
    graphics_context_set_fill_color(ctx, bg_color);
    gpath_draw_filled(ctx, &window);
    graphics_context_set_antialiased(ctx, true);
}

static void draw_saved_locations_label(GContext *ctx, GlobeView *view,
                                       GRect bounds, bool selected) {
#if !PBL_ROUND
    const char *label = "SAVED LOCATIONS";
    const int side_inset = 10;
    const int bar_h = GLOBE_SMALL_RECT ? 24 : GLOBE_SAVED_LABEL_HEIGHT;
    int y = bounds.size.h - bar_h;
    if (selected) y += intro_selection_offset(view);

    // BW: the dithered blue read as noise under white text — the selected bar
    // goes solid black instead.
    const GColor sel_fill = PBL_IF_COLOR_ELSE(GColorVividCerulean, GColorBlack);
    graphics_context_set_fill_color(ctx, selected ? sel_fill : GColorWhite);
    graphics_fill_rect(ctx, GRect(0, y, bounds.size.w, bar_h), 0, GCornerNone);
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_rect(ctx, GRect(0, y, bounds.size.w, 2),
                       0, GCornerNone);

    GColor label_color = selected ? GColorWhite : GColorBlack;
    GColor bg_color = selected ? sel_fill : GColorWhite;
    draw_saved_locations_pin(ctx,
                             GPoint(side_inset,
                                    y + (bar_h - GLOBE_SAVED_COG_SIZE) / 2),
                             label_color, bg_color);

    graphics_context_set_text_color(ctx, label_color);
    // Small rect: centring the label under-laps the pin glyph — align it left
    // of the remaining width instead.
    const int text_x = GLOBE_SMALL_RECT ? (side_inset + GLOBE_SAVED_COG_SIZE + 6) : 0;
    graphics_draw_text(ctx, label,
                       fonts_get_system_font(GLOBE_SMALL_RECT ? FONT_KEY_GOTHIC_14_BOLD
                                                              : FONT_KEY_GOTHIC_18_BOLD),
                       GRect(text_x, y + (GLOBE_SMALL_RECT ? 2 : 3),
                             bounds.size.w - text_x,
                             bar_h - 3),
                       GTextOverflowModeTrailingEllipsis,
                       GLOBE_SMALL_RECT ? GTextAlignmentLeft : GTextAlignmentCenter,
                       NULL);
#else
    // Uppercase to match the rect city-select redesign's footer treatment.
    const char *label = "SAVED LOCATIONS";
    GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
    // Constant string + constant font: measure once, not per intro frame (28.6fps while the
    // cradle animates).
    static GSize text_size;
    if (text_size.w == 0) {
        text_size = graphics_text_layout_get_content_size(
            label,
            font,
            GRect(0, 0, 300, GLOBE_SAVED_LABEL_HEIGHT),   // unconstrained: natural label width
            GTextOverflowModeTrailingEllipsis,
            GTextAlignmentLeft
        );
    }
    // The TEXT centres on the screen; the pin hangs off its left (so the label reads
    // centred rather than the pin+text group being centred, which right-shifted the text).
    int text_x = (bounds.size.w - text_size.w) / 2;
    // +4: pulled right off the chin's curve so the pin reads fully inside the glass.
    int pin_x = text_x - GLOBE_SAVED_LABEL_GAP - GLOBE_SAVED_COG_SIZE + 4;
    if (pin_x < 2) pin_x = 2;
    int y = bounds.size.h - GLOBE_SAVED_LABEL_HEIGHT -
            GLOBE_SAVED_LABEL_ROUND_BOTTOM_INSET;
    if (selected) y += intro_selection_offset(view);
    if (selected) {
        int fill_h = PBL_IF_ROUND_ELSE(bounds.size.h - y,
                                       GLOBE_SAVED_LABEL_HEIGHT);
        graphics_context_set_fill_color(ctx, GColorVividCerulean);
        graphics_fill_rect(ctx, GRect(0, y, bounds.size.w, fill_h),
                           0, GCornerNone);
    }
    // Divider above the row, ALWAYS drawn (rect redesign parity): 2px black rule replacing
    // the old selection-only 1px grey line, FULL WIDTH (the glass mask clips it edge to edge).
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_rect(ctx, GRect(0, y, bounds.size.w, 2), 0, GCornerNone);

    GColor label_color = selected ? GColorWhite : GColorBlack;
    GColor bg_color = selected ? GColorVividCerulean : GColorWhite;
    draw_saved_locations_pin(ctx,
                             GPoint(pin_x, y + (GLOBE_SAVED_LABEL_HEIGHT -
                                                GLOBE_SAVED_COG_SIZE) / 2),
                             label_color, bg_color);

    graphics_context_set_text_color(ctx, label_color);
    graphics_draw_text(ctx, label,
                       font,
                       GRect(text_x, y + 3,
                             bounds.size.w - text_x,
                             GLOBE_SAVED_LABEL_HEIGHT - 3),
                       GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentLeft,
                       NULL);
#endif
}

/**
 * Canvas layer draw handler - renders current globe frame
 */
static void canvas_layer_draw(Layer *layer, GContext *ctx) {
    GlobeView *view = *(GlobeView **)layer_get_data(layer);

    if (!view || view->current_frame >= (int)bw_frame_count_for_view(view)) return;

    // Draw frame centered in canvas.
    GRect bounds = layer_get_bounds(layer);
    GSize frame_size = get_vector_or_bitmap_size(view);

    // Center the planet itself; the cradle extends below the globe bitmap.
    int x = (bounds.size.w - frame_size.w) / 2;
    int y = ((bounds.size.h - frame_size.h) / 2) + GLOBE_CRADLE_CENTER_Y_OFFSET;

    if (view->is_revealing) {
        draw_transition_state(ctx, view, GPoint(x, y), frame_size, bounds);
    } else if (view->is_revealed) {
        int offset = bounce_offset(view);
        draw_space_background(ctx, bounds, view);
        draw_cubemap_globe(ctx, view, GPoint(x, y + offset),
                           frame_size, GLOBE_COLOR_FINAL_SCALE_PERCENT,
                           bounds, false);
        draw_city_label(ctx, view, bounds);
    } else {
        draw_intro_title(ctx, bounds, y, frame_size);
        if (view->intro_world_selected) {
            y += intro_selection_offset(view);
        }
        draw_cradle(ctx, view, GPoint(x, y), frame_size);
        draw_intro_world_frame(ctx, view, GPoint(x, y), frame_size);
        draw_saved_locations_label(ctx, view, bounds,
                                   !view->intro_world_selected);
    }
}

static void space_layer_draw(Layer *layer, GContext *ctx) {
    GlobeView *view = *(GlobeView **)layer_get_data(layer);
    draw_space_background(ctx, layer_get_bounds(layer), view);
}

static void globe_layer_draw(Layer *layer, GContext *ctx) {
    GlobeView *view = *(GlobeView **)layer_get_data(layer);
    if (!view || !view->is_revealed || view->is_revealing) return;

    Layer *root_layer = window_get_root_layer(view->window);
    GRect root_bounds = layer_get_bounds(root_layer);
    GRect frame = layer_get_frame(layer);
    GPoint center = revealed_globe_center_for_bounds(root_bounds,
                                                     bounce_offset(view));
    draw_cubemap_globe_at_center(ctx, view, center,
                                 GLOBE_COLOR_FINAL_SCALE_PERCENT,
                                 frame, true);
}

static void set_intro_world_selected(GlobeView *view, bool selected) {
    if (!view || view->is_revealed || view->is_revealing) return;
    if (view->intro_world_selected == selected) return;
    view->intro_world_selected = selected;
    view->intro_selection_ms = GLOBE_INTRO_SELECTION_DURATION_MS;
    if (view->canvas_layer) {
        layer_mark_dirty(view->canvas_layer);
    }
}

static void activate_intro_selection(GlobeView *view) {
    if (!view || view->is_revealing) return;
    if (view->intro_world_selected) {
        toggle_reveal(view);
    } else if (view->saved_locations_callback) {
        view->saved_locations_callback(view->saved_locations_context);
    } else {
        globe_view_pop(view);
    }
}

/**
 * Create window click handler
 */
static void window_click_handler(ClickRecognizerRef recognizer, void *context) {
    GlobeView *view = (GlobeView *)context;
    note_globe_interaction(view);

    ButtonId button = click_recognizer_get_button_id(recognizer);
    if (button == BUTTON_ID_BACK) {
        if (view->is_revealed && !view->is_revealing) {
            toggle_reveal(view);  // colour earth -> animated un-reveal back to the B+W cradle
        } else if (!view->is_revealing) {
            if (view->back_callback) {
                globe_view_slide_out_right(view);  // cradle BACK -> slide out right, card slides back in
            } else {
                app_window_stack_pop_all(true);    // no card to return to -> exit the app (carousel)
            }
        }
    } else if (button == BUTTON_ID_SELECT) {
        if (!view->is_revealed) {
            activate_intro_selection(view);
        } else if (!view->is_revealing) {
            if (commit_hovered_location(view)) {
                globe_view_pop(view);
            }
        }
    } else if (button == BUTTON_ID_UP) {
        if (view->is_revealed && !view->is_revealing) {
            navigate_city(view, false);
        } else if (!view->is_revealing) {
            set_intro_world_selected(view, true);
        }
    } else if (button == BUTTON_ID_DOWN) {
        if (view->is_revealed && !view->is_revealing) {
            navigate_city(view, true);
        } else if (!view->is_revealing) {
            if (view->intro_world_selected) {
                set_intro_world_selected(view, false);   // world -> saved-locations cursor (in-screen)
            }
            // DOWN on the saved-locations cursor no longer exits to the sunset card (removed); the
            // way back to the card is the BACK button (globe_view_slide_out_right).
        }
    }
}

#ifdef CONFIG_TOUCH
static bool point_is_on_intro_globe(GlobeView *view, int16_t x, int16_t y) {
    if (!view || !view->window) return false;

    Layer *root_layer = window_get_root_layer(view->window);
    GRect bounds = layer_get_bounds(root_layer);
    GSize frame_size = get_vector_or_bitmap_size(view);
    int origin_x = (bounds.size.w - frame_size.w) / 2;
    int origin_y = ((bounds.size.h - frame_size.h) / 2) +
        GLOBE_CRADLE_CENTER_Y_OFFSET;
    int center_x = origin_x + frame_size.w / 2;
    int center_y = origin_y + frame_size.h / 2 + GLOBE_PLANET_CENTER_Y_OFFSET;
    int radius = GLOBE_RENDER_BASE_DIAMETER / 2;
    int dx = x - center_x;
    int dy = y - center_y;
    return (dx * dx) + (dy * dy) <= radius * radius;
}

static bool point_is_on_saved_locations_label(GlobeView *view, int16_t y) {
    if (!view || !view->window) return false;
    GRect bounds = layer_get_bounds(window_get_root_layer(view->window));
    // Top of the label band — same expression as draw_saved_locations_label; everything
    // from the rule line down (incl. the round bottom inset) counts as the touch target.
    return y >= (bounds.size.h - GLOBE_SAVED_LABEL_HEIGHT -
                 GLOBE_SAVED_LABEL_ROUND_BOTTOM_INSET);
}

static bool point_is_on_revealed_globe(GlobeView *view, int16_t x, int16_t y) {
    if (!view || !view->window) return false;

    Layer *root_layer = window_get_root_layer(view->window);
    GRect bounds = layer_get_bounds(root_layer);
    GPoint center = revealed_globe_center_for_bounds(bounds, bounce_offset(view));
    int radius = revealed_globe_radius() + GLOBE_ATMOSPHERE_GLOW_PX;
    int dx = x - center.x;
    int dy = y - center.y;
    return (dx * dx) + (dy * dy) <= radius * radius;
}

static void touch_handler(const TouchEvent *event, void *context) {
    GlobeView *view = (GlobeView *)context;
    if (!view) return;
    // Pop-transition gap: globe_view_pop stops the animation state BEFORE the
    // async window pop finishes, but touch stays subscribed until disappear.
    // Ignore events in that window — a drag could re-arm timers the disappear
    // handler already skipped, and a tap could double-pop the stack.
    if (!view->is_animating) return;

    if (event->type == TouchEvent_Touchdown) {
        note_globe_interaction(view);
        view->touch_start_x = event->x;
        view->touch_start_y = event->y;
        view->touch_last_x = event->x;
        view->touch_last_y = event->y;
        view->touch_velocity_x_q8 = 0;
        view->touch_velocity_y_q8 = 0;
        view->touch_active = true;
        view->touch_drag_axis_set = false;
        view->touch_drag_rotated = false;
        view->touch_down_on_globe = false;
        view->touch_controls_globe = false;

        if (view->is_revealed && !view->is_revealing) {
            bool whole_screen =
                view->is_free_roam || view->coast_active ||
                view->coast_timer || view->city_anim;
            begin_globe_drag_capture(view, event->x, event->y, whole_screen);
        }

    } else if (event->type == TouchEvent_PositionUpdate) {
        if (view->is_revealed && !view->is_revealing &&
            !view->touch_active &&
            (view->is_free_roam || view->coast_active ||
             view->coast_timer || view->city_anim)) {
            view->touch_active = true;
            begin_globe_drag_capture(view, event->x, event->y, true);
            return;
        }

        if (!view->touch_active) return;

        if (view->is_revealed && !view->is_revealing &&
            view->touch_controls_globe) {
            update_globe_drag(view, event->x, event->y);
        }

    } else if (event->type == TouchEvent_Liftoff && view->touch_active) {
        view->touch_active = false;
        int16_t dx = event->x - view->touch_start_x;
        int16_t dy = event->y - view->touch_start_y;
        int16_t adx = dx < 0 ? -dx : dx;
        int16_t ady = dy < 0 ? -dy : dy;

        if (view->is_revealed && !view->is_revealing &&
            view->touch_controls_globe) {
            bool was_dragged = view->touch_drag_rotated;
            bool was_globe_tap = view->touch_down_on_globe &&
                adx <= GLOBE_TAP_THRESHOLD && ady <= GLOBE_TAP_THRESHOLD;
            view->touch_controls_globe = false;
            view->touch_down_on_globe = false;

            if (was_dragged) {
                if (start_globe_coast(view)) {
                    mark_dynamic_globe_dirty(view);
                } else if (!try_start_magnetic_city_lock(view, GLOBE_LOCK_RADIUS_PX)) {
                    mark_dynamic_globe_dirty(view);
                }
            } else if (was_globe_tap) {
                if (commit_hovered_location(view)) {
                    globe_view_pop(view);
                }
            } else {
                mark_dynamic_globe_dirty(view);
            }
            return;
        }

        view->touch_controls_globe = false;
        view->touch_down_on_globe = false;

        if (adx <= GLOBE_TAP_THRESHOLD && ady <= GLOBE_TAP_THRESHOLD) {
            // B&W cradle taps route by target: tap the globe to reveal it, tap the
            // saved-locations bar to open the list. Taps elsewhere (title/dead space) no-op.
            if (!view->is_revealed) {
                if (point_is_on_intro_globe(view, event->x, event->y)) {
                    toggle_reveal(view);
                } else if (!view->is_revealing &&
                           point_is_on_saved_locations_label(view, event->y) &&
                           view->saved_locations_callback) {
                    view->saved_locations_callback(view->saved_locations_context);
                }
            }
        } else if (!view->is_revealed && !view->is_revealing &&
                   adx > ady && dx > 0) {
            // Cradle swipe RIGHT backs out to the sunset card (mirrors BACK). The only
            // cradle swipe — vertical swipes stay inert (taps handle globe/saved-list).
            if (view->back_callback) {
                globe_view_slide_out_right(view);
            } else {
                app_window_stack_pop_all(true);
            }
        } else if (view->is_revealed && !view->is_revealing) {
            if (view->touch_drag_axis_set) {
                mark_dynamic_globe_dirty(view);   // revealed-globe drag settle (in-screen)
            } else if (adx > ady && dx > 0) {
                toggle_reveal(view);   // off-globe swipe right = BACK (un-reveal to the cradle)
            }
        }
    }
}
#endif

/**
 * Register window click handlers
 */
static void window_click_provider(void *context) {
    window_single_click_subscribe(BUTTON_ID_BACK, window_click_handler);
    window_single_click_subscribe(BUTTON_ID_SELECT, window_click_handler);
    window_single_click_subscribe(BUTTON_ID_UP, window_click_handler);
    window_single_click_subscribe(BUTTON_ID_DOWN, window_click_handler);
}

static void window_appear_handler(Window *window) {
    GlobeView *view = (GlobeView *)window_get_user_data(window);
    if (!view || !view->is_animating) return;

    ensure_visual_resources(view);
    if (!view->is_revealed && !view->animation_timer) {
        schedule_frame_timer(view);
        start_globe_idle_timer(view);
    }
#ifdef CONFIG_TOUCH
    touch_service_subscribe(touch_handler, view);
#endif
    mark_dynamic_globe_dirty(view);
}

static void window_disappear_handler(Window *window) {
    GlobeView *view = (GlobeView *)window_get_user_data(window);
#ifdef CONFIG_TOUCH
    // Always release the touch slot here (not in stop_animation): the globe is often
    // DESTROYED after the next window's appear already re-subscribed (commit -> forecast,
    // BACK -> card), and a late unsubscribe would clobber that window's subscription.
    touch_service_unsubscribe();
#endif
    if (!view || !view->is_animating) return;

#ifdef CONFIG_TOUCH
    stop_globe_coast(view, false);
#endif
    cancel_timer_slot(&view->animation_timer);
    cancel_timer_slot(&view->idle_timer);
    view->bw_idle = false;
    view->bw_idle_slowdown_step = 0;
    release_visual_resources(view);
}

/**
 * Create a new globe view
 */
GlobeView *globe_view_create(void) {
    GlobeView *view = malloc(sizeof(GlobeView));
    if (!view) return NULL;
    memset(view, 0, sizeof(*view));

    // Create window
    view->window = window_create();
    if (!view->window) {
        free(view);
        return NULL;
    }

    // Configure window
    window_set_user_data(view->window, view);
    window_set_background_color(view->window, GColorWhite);
    window_set_click_config_provider_with_context(view->window, window_click_provider, view);
    window_set_window_handlers(view->window, (WindowHandlers) {
        .appear = window_appear_handler,
        .disappear = window_disappear_handler,
    });
    // Create canvas layer for animation
    Layer *root_layer = window_get_root_layer(view->window);
    GRect bounds = layer_get_bounds(root_layer);

    view->canvas_layer = layer_create_with_data(bounds, sizeof(GlobeView *));
    if (!view->canvas_layer) {
        window_destroy(view->window);
        free(view);
        return NULL;
    }
    *(GlobeView **)layer_get_data(view->canvas_layer) = view;
    layer_set_update_proc(view->canvas_layer, canvas_layer_draw);
    layer_add_child(root_layer, view->canvas_layer);

    view->space_layer = layer_create_with_data(bounds, sizeof(GlobeView *));
    if (!view->space_layer) {
        layer_destroy(view->canvas_layer);
        window_destroy(view->window);
        free(view);
        return NULL;
    }
    *(GlobeView **)layer_get_data(view->space_layer) = view;
    layer_set_update_proc(view->space_layer, space_layer_draw);
    layer_set_hidden(view->space_layer, true);
    layer_add_child(root_layer, view->space_layer);

    view->globe_layer = layer_create_with_data(
        revealed_globe_layer_frame(bounds), sizeof(GlobeView *));
    if (!view->globe_layer) {
        layer_destroy(view->space_layer);
        layer_destroy(view->canvas_layer);
        window_destroy(view->window);
        free(view);
        return NULL;
    }
    *(GlobeView **)layer_get_data(view->globe_layer) = view;
    layer_set_update_proc(view->globe_layer, globe_layer_draw);
    layer_set_hidden(view->globe_layer, true);
    layer_add_child(root_layer, view->globe_layer);

    view->city_label_layer = text_layer_create(
        city_label_frame_for_bounds(bounds));
    if (!view->city_label_layer) {
        layer_destroy(view->globe_layer);
        layer_destroy(view->space_layer);
        layer_destroy(view->canvas_layer);
        window_destroy(view->window);
        free(view);
        return NULL;
    }

    // BW: white-on-clear text sinks into the dithered globe — give the label a
    // solid black chip. Colour keeps the borderless look over the dark space bg.
    text_layer_set_background_color(view->city_label_layer,
                                    PBL_IF_COLOR_ELSE(GColorClear, GColorBlack));
    text_layer_set_text_color(view->city_label_layer, GColorWhite);
    text_layer_set_font(view->city_label_layer,
                        fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
    text_layer_set_text_alignment(view->city_label_layer,
                                  GTextAlignmentCenter);
    text_layer_set_overflow_mode(view->city_label_layer,
                                 GTextOverflowModeTrailingEllipsis);
    text_layer_set_text(view->city_label_layer, "");
    layer_set_hidden(text_layer_get_layer(view->city_label_layer), true);
    layer_add_child(root_layer, text_layer_get_layer(view->city_label_layer));

    // Initialize animation state (all-zero fields come from the memset above)
    matrix_identity(view->globe_rotation);
    view->bw_frame_count = NUM_BW_FRAMES;
    view->bw_sequence_duration_ms = NUM_BW_FRAMES * GLOBE_FRAME_INTERVAL_MS;
    view->reveal_direction = 1;
#ifdef CONFIG_TOUCH
    view->hover_city_index = -1;
#endif
    globe_view_reload_saved_locations(view);

    return view;
}

void globe_view_set_location_select_callback(GlobeView *view,
                                             GlobeLocationSelectCallback callback,
                                             void *context) {
    if (!view) return;
    view->location_select_callback = callback;
    view->location_select_context = context;
}

static void globe_view_reload_saved_locations(GlobeView *view) {
    if (!view) return;

    SavedLocationEntry previous_entry;
    bool had_previous_entry = false;
    SavedLocationEntry *current_entry = selected_saved_entry(view);
    if (current_entry) {
        previous_entry = *current_entry;
        had_previous_entry = true;
    }

    // The phone's synced records ARE the pin list.
    view->saved_entry_count = saved_locations_get_entries(
        view->saved_entries, SAVED_LOCATIONS_MAX_ENTRIES);

    if (view->saved_entry_count <= 0) {
        // Nothing synced yet: keep one entry so the globe still has a subject.
        view->saved_entry_count = 1;
        view->saved_entries[0] = (SavedLocationEntry) {
            .ds_index = -1,
            .is_current_location = true,
            .latitude_e2 = (int16_t)view->current_location_latitude_e2,
            .longitude_e2 = (int16_t)view->current_location_longitude_e2,
            .has_coordinates = view->has_current_location,
            .temp = (int16_t)WX_DS_UNKNOWN_TEMP,
        };
        snprintf(view->saved_entries[0].label,
                 sizeof(view->saved_entries[0].label), "%s",
                 view->current_location_label[0]
                     ? view->current_location_label
                     : "Current Location");
    }

    int next_index = had_previous_entry
        ? find_saved_entry_index(view, &previous_entry)
        : -1;
    if (next_index < 0) {
        next_index = find_current_saved_entry_index(view);
    }
    if (next_index < 0) next_index = 0;

    view->selected_city_index = next_index;
    update_selected_transition_target(view);

    if (view->is_revealed && !view->is_revealing && !view->is_free_roam) {
        set_color_orientation(view,
                              view->transition_color_target_latitude_e2,
                              view->transition_color_target_longitude_e2);
        reload_current_frame(view);
    } else {
        update_city_label_layer(view);
        mark_dynamic_globe_dirty(view);
    }
}

void globe_view_set_saved_locations_callback(GlobeView *view,
                                             GlobeSavedLocationsCallback callback,
                                             void *context) {
    if (!view) return;
    view->saved_locations_callback = callback;
    view->saved_locations_context = context;
}

// (main_callback chain deleted — registered but never invoked; size pass)

void globe_view_set_back_callback(GlobeView *view,
                                  GlobeMainCallback callback,
                                  void *context) {
    if (!view) return;
    view->back_callback = callback;
    view->back_context = context;
}

void globe_view_set_current_location(GlobeView *view,
                                     const char *label,
                                     int16_t latitude_e2,
                                     int16_t longitude_e2) {
    if (!view) return;

    bool should_select_current =
        !view->is_animating || selected_city_is_current_location(view);

    view->has_current_location = true;
    view->current_location_latitude_e2 = latitude_e2;
    view->current_location_longitude_e2 = longitude_e2;
    if (label && label[0]) {
        strncpy(view->current_location_label, label,
                sizeof(view->current_location_label) - 1);
        view->current_location_label[sizeof(view->current_location_label) - 1] = '\0';
    } else {
        strncpy(view->current_location_label, "Current Location",
                sizeof(view->current_location_label) - 1);
        view->current_location_label[sizeof(view->current_location_label) - 1] = '\0';
    }

    globe_view_reload_saved_locations(view);

    if (should_select_current) {
        int current_index = find_current_saved_entry_index(view);
        if (current_index >= 0) {
            view->selected_city_index = current_index;
        }
        update_selected_transition_target(view);
        if (view->is_revealed && !view->is_revealing &&
            !view->is_free_roam) {
            set_color_orientation(view,
                                  view->transition_color_target_latitude_e2,
                                  view->transition_color_target_longitude_e2);
            reload_current_frame(view);
        }
    }
}

/**
 * Destroy the globe view
 */
void globe_view_destroy(GlobeView *view) {
    if (!view) return;

    // Stop animation
    globe_view_stop_animation(view);

    cancel_reveal_animation(view);

    release_visual_resources(view);

    if (view->city_label_layer) {
        text_layer_destroy(view->city_label_layer);
        view->city_label_layer = NULL;
    }
    if (view->globe_layer) {
        layer_destroy(view->globe_layer);
        view->globe_layer = NULL;
    }
    if (view->space_layer) {
        layer_destroy(view->space_layer);
        view->space_layer = NULL;
    }

    // Destroy canvas layer
    if (view->canvas_layer) {
        layer_destroy(view->canvas_layer);
        view->canvas_layer = NULL;
    }

    // Destroy window
    if (view->window) {
        window_destroy(view->window);
        view->window = NULL;
    }

    free(view);
}

/**
 * Start the globe animation
 */
void globe_view_start_animation(GlobeView *view) {
    if (!view || view->is_animating) return;

    globe_view_reload_saved_locations(view);

    view->is_animating = true;
    view->current_frame = 0;
    view->bw_elapsed_ms = 0;
    set_color_orientation(view, 0, longitude_e2_for_bw_frame(view->current_frame));
    view->transition_color_start_latitude_e2 = 0;
    view->transition_color_start_longitude_e2 = view->color_longitude_e2;
    view->transition_color_target_latitude_e2 = selected_city_latitude_e2(view);
    view->transition_color_target_longitude_e2 = selected_city_longitude_e2(view);
    view->transition_bw_frame = 0;
    view->reveal_progress = 0;
    view->reveal_direction = 1;
    view->is_revealing = false;
    view->is_revealed = false;
    view->intro_world_selected = true;   // cursor starts on the globe (SELECT reveals it immediately)
    view->bw_idle = false;
    view->bw_idle_slowdown_step = 0;
    view->intro_selection_ms = 0;
#ifdef CONFIG_TOUCH
    stop_globe_coast(view, false);
    view->hover_lock_active = false;
#endif
    cancel_all_view_animations(view);
    ensure_visual_resources(view);

    show_intro_canvas(view);
    layer_mark_dirty(view->canvas_layer);
    schedule_frame_timer(view);
    start_globe_idle_timer(view);
#ifdef CONFIG_TOUCH
    touch_service_subscribe(touch_handler, view);
#endif
}

/**
 * Stop the globe animation
 */
void globe_view_stop_animation(GlobeView *view) {
    if (!view) return;
    // BOTH shapes: round drives the card<->globe hslide pair now, so a mid-slide dismissal
    // must cancel this before the layer is torn down, exactly like rect's drop-in.
    if (view->entry_drop_anim) {
        Animation *a = view->entry_drop_anim;
        view->entry_drop_anim = NULL;
        animation_unschedule(a);
        animation_destroy(a);
    }
    if (!view->is_animating) return;

    view->is_animating = false;

    // Cancel animation timer
    cancel_timer_slot(&view->animation_timer);
    cancel_timer_slot(&view->idle_timer);
    view->bw_idle = false;
    view->bw_idle_slowdown_step = 0;

    cancel_all_view_animations(view);
    view->is_revealing = false;
#ifdef CONFIG_TOUCH
    stop_globe_coast(view, false);
    // touch_service_unsubscribe lives in window_disappear_handler — stop_animation runs on
    // destroy AFTER the next window re-subscribed and must not clobber it.
    view->touch_active = false;
    view->touch_drag_axis_set = false;
    view->touch_drag_rotated = false;
    view->touch_down_on_globe = false;
    view->touch_controls_globe = false;
    view->coast_active = false;
    view->hover_lock_active = false;
#endif

    release_visual_resources(view);
}

// Slide duration + interpolation are shared with expanded_view via weather_math.h
// (WEATHER_HSLIDE_MS / weather_interpolate_moook_soft1) so the pair can't drift.
// BOTH shapes.

static void prv_drop_stopped(Animation *anim, bool finished, void *context) {
    (void)finished;
    GlobeView *view = (GlobeView *)context;
    if (view && view->entry_drop_anim == anim) {
        view->entry_drop_anim = NULL;   // auto-destroyed after a normal stop
    }
}

// Shared moook horizontal slide for the intro canvas: create + configure +
// schedule; returns false when the PropertyAnimation could not be created so
// callers can run their fallback.
static bool start_canvas_hslide(GlobeView *view, GRect *from, GRect *to,
                                AnimationStoppedHandler stopped) {
    PropertyAnimation *pa =
        property_animation_create_layer_frame(view->canvas_layer, from, to);
    if (!pa) return false;
    Animation *a = (Animation *)pa;
    animation_set_duration(a, WEATHER_HSLIDE_MS);
    animation_set_custom_interpolation(a, weather_interpolate_moook_soft1);
    animation_set_handlers(a, (AnimationHandlers){ .stopped = stopped }, view);
    view->entry_drop_anim = a;
    animation_schedule(a);
    return true;
}

// SELECT-from-card entrance: the intro canvas slides in from one screen-width to the RIGHT (with
// the same moook bounce as the drop-in), pairing with the card's slide-out to the left.
void globe_view_push_slide_in_right(GlobeView *view) {
    if (!view || !view->canvas_layer) { globe_view_push_animated(view, false); return; }
    GRect to = layer_get_frame(view->canvas_layer);
    GRect from = to;
    from.origin.x += to.size.w;                  // start one screen-width off the right
    layer_set_frame(view->canvas_layer, from);   // first painted frame is already off the right
    window_stack_push(view->window, false);
    globe_view_start_animation(view);
    if (!start_canvas_hslide(view, &from, &to, prv_drop_stopped)) {
        layer_set_frame(view->canvas_layer, to);
    }
}

// Cradle BACK exit: the intro canvas slides out to the right, then hands off via back_callback
// (weather.c dismisses the globe and slides the card back in from the left).
static void prv_slide_out_right_stopped(Animation *anim, bool finished, void *context) {
    GlobeView *view = (GlobeView *)context;
    if (!view) return;
    if (view->entry_drop_anim == anim) view->entry_drop_anim = NULL;
    if (finished && view->back_callback) {
        view->back_callback(view->back_context);   // dismiss globe + slide card in from the left
    }
    // Restore the canvas to its on-screen home so a later push isn't left off-screen.
    if (view->canvas_layer && view->window) {
        layer_set_frame(view->canvas_layer,
                        layer_get_bounds(window_get_root_layer(view->window)));
    }
}

void globe_view_slide_out_right(GlobeView *view) {
    if (!view || !view->canvas_layer) {
        if (view && view->back_callback) view->back_callback(view->back_context);
        return;
    }
    GRect from = layer_get_frame(view->canvas_layer);   // current, on-screen
    GRect to = from;
    to.origin.x += from.size.w;                          // right and off the edge
    if (!start_canvas_hslide(view, &from, &to, prv_slide_out_right_stopped)) {
        if (view->back_callback) view->back_callback(view->back_context);
    }
}


// Entry coords for the focus search — platform-neutral (the touch build's
// saved_entry_globe_coords twin lives inside CONFIG_TOUCH).
static bool prv_entry_coords_for_focus(GlobeView *view, SavedLocationEntry *e,
                                       int32_t *lat, int32_t *lon) {
  if (!e) return false;
  if (e->is_current_location) {
    if (!view->has_current_location) return false;
    *lat = view->current_location_latitude_e2;
    *lon = view->current_location_longitude_e2;
    return true;
  }
  if (!e->has_coordinates) return false;
  *lat = e->latitude_e2;
  *lon = e->longitude_e2;
  return true;
}

// Snap the globe's rest state onto the saved pin nearest (lat,lon) — the ACTIVE
// location — so a re-opened globe hovers the city the user last selected. Call
// BEFORE the push: the reveal intro targets selected_city_index, and the direct
// orientation snap covers re-pushes that skip the intro. No-op without coords.
void globe_view_focus_coords(GlobeView *view, int16_t lat_e2, int16_t lon_e2) {
  if (!view || view->saved_entry_count <= 0) return;
  if (lat_e2 == INT16_MIN || lon_e2 == INT16_MIN) return;
  int best = -1;
  int32_t best_d = 0;
  for (int i = 0; i < view->saved_entry_count; i++) {
    SavedLocationEntry *e = saved_entry_for_index(view, i);
    int32_t elat, elon;
    if (!prv_entry_coords_for_focus(view, e, &elat, &elon)) continue;
    const int32_t dlat = elat - lat_e2;
    const int32_t dlon = shortest_longitude_delta_e2(lon_e2, elon);
    const int32_t d = dlat * dlat + dlon * dlon;
    if (best < 0 || d < best_d) { best = i; best_d = d; }
  }
  if (best < 0) return;
  cancel_city_animation(view);
  cancel_bounce_animation(view);
  view->selected_city_index = best;
#ifdef CONFIG_TOUCH
  view->hover_city_index = best;
  view->hover_lock_active = true;
  set_free_roam_enabled(view, false);
#endif
  {
    SavedLocationEntry *e = saved_entry_for_index(view, best);
    int32_t elat, elon;
    if (prv_entry_coords_for_focus(view, e, &elat, &elon)) {
      view->color_latitude_e2 = elat;
      view->color_longitude_e2 = elon;
    }
  }
  update_city_label_layer(view);
  mark_dynamic_globe_dirty(view);
}

void globe_view_push_animated(GlobeView *view, bool animated) {
    if (!view) return;

    window_stack_push(view->window, animated);
    globe_view_start_animation(view);
}

/**
 * Pop the globe view from the window stack
 */
void globe_view_pop(GlobeView *view) {
    if (!view) return;

    globe_view_stop_animation(view);
    window_stack_pop(true);
}

void globe_view_dismiss(GlobeView *view, bool animated) {
    if (!view) return;

    globe_view_stop_animation(view);
    window_stack_remove(view->window, animated);
}
