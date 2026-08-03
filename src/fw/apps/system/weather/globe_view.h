/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

/**
 * globe_view.h
 * Globe animation view for Pebble Weather
 * Displays rotating globe with weather forecast overlay
 */

#pragma once

#include "pebble_compat.h"
#include "saved_locations.h"
#include "weather_platform.h"

//! `ds_index` identifies the phone's weather record directly — no matching.
typedef void (*GlobeLocationSelectCallback)(int ds_index, bool force,
                                            void *context);
typedef void (*GlobeSavedLocationsCallback)(void *context);
typedef void (*GlobeMainCallback)(void *context);

typedef struct {
    Window *window;
    Layer *canvas_layer;
    Layer *space_layer;
    Layer *globe_layer;
    TextLayer *city_label_layer;
    uint8_t *cubemap_data;
    size_t cubemap_size;
    uint8_t *starfield_data;
    size_t starfield_size;
    GDrawCommandSequence *bw_sequence;
    GDrawCommandImage *cradle_pdc;
    uint32_t bw_frame_count;
    uint32_t bw_sequence_duration_ms;
    uint32_t bw_elapsed_ms;
    int current_frame;
    int32_t color_latitude_e2;
    int32_t color_longitude_e2;
    int32_t transition_color_start_latitude_e2;
    int32_t transition_color_start_longitude_e2;
    int32_t transition_color_target_latitude_e2;
    int32_t transition_color_target_longitude_e2;
    int32_t city_anim_start_latitude_e2;
    int32_t city_anim_start_longitude_e2;
    int32_t city_anim_target_latitude_e2;
    int32_t city_anim_target_longitude_e2;
    int32_t city_anim_latitude_delta_e2;
    int32_t city_anim_longitude_delta_e2;
    int32_t globe_rotation[9];
    int transition_bw_frame;
    Animation *reveal_anim;
    Animation *city_anim;
    Animation *bounce_anim;
    Animation *lock_pulse_anim;    // expanding Celeste ring on lock/nav arrival
    AnimationProgress reveal_progress;
    AnimationProgress city_anim_progress;
    AnimationProgress bounce_progress;
    AnimationProgress lock_pulse_progress;
    int reveal_direction;
    int selected_city_index;
    SavedLocationEntry saved_entries[SAVED_LOCATIONS_MAX_ENTRIES];
    int saved_entry_count;
    int32_t current_location_latitude_e2;
    int32_t current_location_longitude_e2;
    int bounce_direction;
    char current_location_label[48];
    char city_label_text[48];
    GlobeLocationSelectCallback location_select_callback;
    void *location_select_context;
    GlobeSavedLocationsCallback saved_locations_callback;
    void *saved_locations_context;
    GlobeMainCallback back_callback;   // cradle BACK -> weather card (reverse of the SELECT entrance)
    void *back_context;
    AppTimer *animation_timer;
    AppTimer *idle_timer;
#if WEATHER_PLATFORM_TOUCH_COLOR
    AppTimer *coast_timer;
#endif
    bool is_animating;
    bool is_revealing;
    bool is_revealed;
    bool has_current_location;
    bool has_custom_location;
    bool is_free_roam;
    bool intro_world_selected;
    bool bw_idle;
    uint8_t bw_idle_slowdown_step;
    uint8_t intro_selection_ms;
#if WEATHER_PLATFORM_TOUCH_COLOR
    int16_t touch_start_x;
    int16_t touch_start_y;
    int16_t touch_last_x;
    int16_t touch_last_y;
    int32_t touch_velocity_x_q8;
    int32_t touch_velocity_y_q8;
    int32_t coast_velocity_x_q8;
    int32_t coast_velocity_y_q8;
    int32_t starfield_offset_x_q8;
    int32_t starfield_offset_y_q8;
    int hover_city_index;
    bool touch_active;
    bool touch_drag_axis_set;
    bool touch_drag_rotated;
    bool touch_down_on_globe;
    bool touch_controls_globe;
    bool coast_active;
    bool hover_lock_active;
#endif
    // UP-from-forecast entrance: Pebble-Health drop-from-above + bounce (rect only).
    Animation *entry_drop_anim;
} GlobeView;

/**
 * Create a new globe view
 * @return Pointer to initialized GlobeView, or NULL on error
 */
GlobeView *globe_view_create(void);

void globe_view_set_location_select_callback(GlobeView *view,
                                             GlobeLocationSelectCallback callback,
                                             void *context);

void globe_view_set_saved_locations_callback(GlobeView *view,
                                             GlobeSavedLocationsCallback callback,
                                             void *context);

// Set the callback fired when BACK is pressed on the intro cradle: the globe slides out to the
// RIGHT, then this fires so weather.c can bring the weather card back in from the LEFT. When unset,
// cradle BACK keeps its default (exit the app).
void globe_view_set_back_callback(GlobeView *view,
                                  GlobeMainCallback callback,
                                  void *context);

void globe_view_set_current_location(GlobeView *view,
                                     const char *label,
                                     int16_t latitude_e2,
                                     int16_t longitude_e2);

/**
 * Destroy the globe view and free all resources
 * @param view Pointer to GlobeView to destroy
 */
void globe_view_destroy(GlobeView *view);

/**
 * Start the globe animation
 * @param view Pointer to GlobeView
 */
void globe_view_start_animation(GlobeView *view);

/**
 * Stop the globe animation
 * @param view Pointer to GlobeView
 */
void globe_view_stop_animation(GlobeView *view);

void globe_view_push_animated(GlobeView *view, bool animated);

//! Snap the globe's hovered pin to the saved location nearest these coords (the
//! ACTIVE location) — call before a push so the globe opens on the city the user
//! last selected. INT16_MIN coords = no-op.
void globe_view_focus_coords(GlobeView *view, int16_t lat_e2, int16_t lon_e2);

// Push with the intro cradle sliding in from one screen-width to the RIGHT (same moook bounce;
// rect only, falls back to a plain push on round). Pairs with the card's slide-out-left on
// SELECT-from-expanded-card.
void globe_view_push_slide_in_right(GlobeView *view);

// Reverse of the slide-in-from-right: slide the intro cradle out to the RIGHT (same moook bounce),
// then fire back_callback so weather.c can bring the card back in from the LEFT. Cradle BACK.
void globe_view_slide_out_right(GlobeView *view);

/**
 * Pop the globe view from the window stack
 * @param view Pointer to GlobeView
 */
void globe_view_pop(GlobeView *view);

void globe_view_dismiss(GlobeView *view, bool animated);
