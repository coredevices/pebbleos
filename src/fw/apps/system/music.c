/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "music.h"

#include "applib/app.h"
#include "applib/event_service_client.h"
#include "applib/app_timer.h"
#include "applib/accel_service.h"
#include "applib/fonts/fonts.h"
#include "applib/preferred_content_size.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/system_icons.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/imaging.h"
#include "pbl/services/music.h"
#include "pbl/services/vibes/vibe_score.h"
#include "shell/prefs.h"
#include "shell/system_theme.h"
#include "process_management/app_manager.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"
#include "pbl/util/trig.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

// Album art needs a colour display with enough RAM for a full-screen 4-bpp cover; only emery and
// gabbro qualify. Flint and lower never request it, so their layout stays text-only.
#if defined(CONFIG_PLATFORM_EMERY) || defined(CONFIG_PLATFORM_GABBRO)
#define MUSIC_ALBUM_ART_SUPPORTED 1
#else
#define MUSIC_ALBUM_ART_SUPPORTED 0
#endif

// On round platforms with album art the media layout is used all the time (with the cover as the
// backdrop when art is available, the plain background otherwise); on rect it applies only while
// art is showing, with the stock layout otherwise.
#define MUSIC_ROUND_MEDIA_LAYOUT (PBL_ROUND && MUSIC_ALBUM_ART_SUPPORTED)

#if PBL_ROUND
// Unified round media layout: a full-screen backdrop, a progress arc along the bezel whose gap
// hugs the action bar, a centred artist / title / times stack across the widest band of the
// circle, and the state icon anchoring the bottom.
#define ART_ROUND_ARTIST_Y (DISP_ROWS / 2 - 70)
#define ART_ROUND_TITLE_Y (DISP_ROWS / 2 - 48)  // region symmetric about the display centre
#define ART_ROUND_TITLE_H 96  // up to three Gothic 28 Bold lines, ellipsised beyond
#define ART_ROUND_TIMES_Y (DISP_ROWS / 2 + 58)
// Symmetric side margin for the text stack: centred on the true display centre, and wide lines
// still clear the action bar's crescent on the right.
#define ART_ROUND_TEXT_MARGIN 48
#define ART_ROUND_ICON_CENTER_Y (DISP_ROWS - 26)
#define ART_ROUND_RING_THICKNESS 7
// Progress arc endpoints, degrees clockwise from 12 o'clock: it runs from the bottom edge of the
// action bar around the bottom, left and top of the circle to the action bar's top edge. The
// endpoints reach a few degrees INTO the action bar's crescent (which draws on top) so the tips
// tuck flush underneath it with no background sliver.
#define ART_ROUND_ARC_START_DEG 124
#define ART_ROUND_ARC_END_DEG (360 + 56)
#endif

// The art-mode title font; the round media layout sizes it up for the wide centre band.
#define TITLE_SCROLL_FONT_KEY \
    PBL_IF_RECT_ELSE(FONT_KEY_GOTHIC_24_BOLD, FONT_KEY_GOTHIC_28_BOLD)

enum ActionBarState {
  ActionBarStateSkip,
  ActionBarStateVolume,
  ActionBarStateLongPress,
};

typedef struct MusicAppSizeConfig {
  const char *music_time_font_key;
  const char *no_music_font_key;
  int16_t horizontal_margin;

  GRangeVertical artist_field;
  GRangeVertical title_field;
  GRangeVertical time_field;

  GRect cassette_rect;
  int16_t cassette_animation_x;
  int32_t cassette_animation_time;

  GRangeVertical track_field;
  int16_t track_corner_radius;

  GPoint no_music_img_pos;
  GRangeVertical no_music_text_field;
} MusicAppSizeConfig;

// The reference animations ran at 28fps.
#define ANIMATION_FRAME_MS (1000 / 28)

#define CONTENT_VERTICAL_OFFSET PBL_IF_RECT_ELSE(0, 5)
static const MusicAppSizeConfig s_music_size_config_medium = {
  .music_time_font_key = FONT_KEY_GOTHIC_14,
  .no_music_font_key = FONT_KEY_GOTHIC_18_BOLD,
  .horizontal_margin = PBL_IF_RECT_ELSE(12, 25),

  .artist_field = {
    .origin_y = 31 + CONTENT_VERTICAL_OFFSET,
    .size_h = 21,
  },
  .title_field = {
    .origin_y = 53 + CONTENT_VERTICAL_OFFSET,
    .size_h = 44,
  },
  .time_field = {
    .origin_y = 106 + CONTENT_VERTICAL_OFFSET,
    .size_h = 14,
  },

  .cassette_rect = {{0, -1 + CONTENT_VERTICAL_OFFSET}, {43, 28}},
  .cassette_animation_x = 60,
  .cassette_animation_time = 1 * ANIMATION_FRAME_MS,

  .track_field = {
    .origin_y = 120 + CONTENT_VERTICAL_OFFSET,
    .size_h = 4,
  },
  .track_corner_radius = 1,

  .no_music_img_pos = {PBL_IF_RECT_ELSE(29, 53), PBL_IF_RECT_ELSE(25, 26)},
  .no_music_text_field = {
    .origin_y = PBL_IF_RECT_ELSE(107, 104),
    .size_h = 58,
  },
};

static const MusicAppSizeConfig s_music_size_config_large = {
  .music_time_font_key = FONT_KEY_GOTHIC_18_BOLD,
  .no_music_font_key = FONT_KEY_GOTHIC_28,
  .horizontal_margin = PBL_IF_RECT_ELSE(10, 30),

  .artist_field = {
    .origin_y = PBL_IF_RECT_ELSE(30, 48),
    .size_h = 21,
  },
  .title_field = {
    .origin_y = PBL_IF_RECT_ELSE(60, 72),
    .size_h = 80,
  },
  .time_field = {
    .origin_y = PBL_IF_RECT_ELSE(146, 158),
    .size_h = 20,
  },

  .cassette_rect = {{0, -8}, {43, 28}},
  .cassette_animation_x = 140,
  .cassette_animation_time = 3 * ANIMATION_FRAME_MS,

  .track_field = {
    .origin_y = PBL_IF_RECT_ELSE(168, 182),
    .size_h = 10,
  },
  .track_corner_radius = 4,

  .no_music_img_pos = {PBL_IF_RECT_ELSE(57, 72), PBL_IF_RECT_ELSE(46, 58)},
  .no_music_text_field = {
    .origin_y = PBL_IF_RECT_ELSE(131, 143),
    .size_h = 58,
  },
};

static const MusicAppSizeConfig * const s_music_size_configs[NumPreferredContentSizes] = {
  [PreferredContentSizeSmall] = &s_music_size_config_medium,
  [PreferredContentSizeMedium] = &s_music_size_config_medium,
  [PreferredContentSizeLarge] = &s_music_size_config_large,
  [PreferredContentSizeExtraLarge] = &s_music_size_config_large,
};

static const MusicAppSizeConfig *prv_config(void) {
  return s_music_size_configs[PreferredContentSizeDefault];
}

static int prv_content_width(void) {
  return DISP_COLS - ACTION_BAR_WIDTH - (prv_config()->horizontal_margin * 2);
}

static int prv_text_layer_width(void) {
  return prv_content_width() + PBL_IF_RECT_ELSE(prv_config()->horizontal_margin / 2, 0);
}

static GRect prv_artist_rect(void) {
  const MusicAppSizeConfig *config = prv_config();
  return GRect(config->horizontal_margin, config->artist_field.origin_y,
               prv_text_layer_width(), config->artist_field.size_h);
}

static GRect prv_title_rect(void) {
  const MusicAppSizeConfig *config = prv_config();
  return GRect(config->horizontal_margin, config->title_field.origin_y,
               prv_text_layer_width(), config->title_field.size_h);
}

static GRect prv_time_rect(void) {
  const MusicAppSizeConfig *config = prv_config();
  return GRect(config->horizontal_margin, config->time_field.origin_y,
               prv_content_width(), config->time_field.size_h);
}

static GRect prv_cassette_rect(void) {
  const MusicAppSizeConfig *config = prv_config();
  const int16_t cassette_x = config->horizontal_margin +
      PBL_IF_RECT_ELSE(0, prv_content_width() - config->cassette_rect.size.w);
  return GRect(cassette_x, config->cassette_rect.origin.y,
               config->cassette_rect.size.w, config->cassette_rect.size.h);
}

static GRect prv_track_rect(void) {
  const MusicAppSizeConfig *config = prv_config();
  return GRect(config->horizontal_margin, config->track_field.origin_y,
               prv_content_width(), config->track_field.size_h);
}

static const ButtonId BUTTON_FORWARD = BUTTON_ID_DOWN;
static const ButtonId BUTTON_BACKWARD = BUTTON_ID_UP;

static const int16_t BOUNCEBACK_OFFSET = 6;
// These offsets get rid of the empty space above the first line of text,
// enabling neater animations by clipping immediately at the top of the text.
static const int16_t TITLE_BOUNDS_OFFSET = 5;
static const int16_t ARTIST_BOUNDS_OFFSET = 3;
static const int16_t TIME_BOUNDS_OFFSET = 2;

static const uint32_t VOLUME_REPEAT_INTERVAL_MS = 400;
static const uint32_t ACTION_BAR_TIMEOUT_MS = 2000;
static const uint32_t VOLUME_ICON_TIMEOUT_MS = 2000;


typedef struct {
  Window window;
  BitmapLayer bitmap_layer;
  GBitmap bitmap;
  TextLayer text_layer;
} MusicNoMusicWindow;

typedef struct {
  Window window;
  ActionBarLayer action_bar;

  TextLayer artist_text_layer;
  char artist_buffer[MUSIC_BUFFER_LENGTH];

  TextLayer title_text_layer;
  char title_buffer[MUSIC_BUFFER_LENGTH];

  StatusBarLayer status_layer;

  TextLayer position_text_layer;
  char position_buffer[9]; // 9 will fit "00:00:00"

  TextLayer length_text_layer;
  char length_buffer[9];

  Animation *transition;
  AppTimer *volume_icon_timer;

  MusicPlayState current_play_state;

  GBitmap icon_skip_forward;
  GBitmap icon_skip_backward;
  GBitmap icon_ellipsis;
  GBitmap icon_pause;
  GBitmap icon_play;
  GBitmap icon_play_pause;
  GBitmap icon_volume_up;
  GBitmap icon_volume_down;
  GBitmap image_cassette;
  GBitmap image_pause;
  GBitmap image_volume_up;
  GBitmap image_volume_down;

  Layer cassette_container;
  BitmapLayer cassette_layer;
  GBitmap *cassette_current_icon;

  EventServiceInfo event_info;

  // Album art fills the top (where the artist/title normally sit). When shown, the artist is hidden
  // and the track title moves down beside the tape. Art itself is owned by the music service; this
  // layer just blits it.
  Layer album_art_layer;
  bool has_album_art;
  // Last now-playing generation we requested art for, so a track change re-fetches even if the
  // title/artist strings happen to match.
  uint8_t last_art_generation;
  // One-way "digital sign" scroll for the title beside the tape (album-art mode). Runs for a couple
  // of cycles after a track change then rests at the start; only scrolls when the title overflows.
  AppTimer *title_marquee_timer;
  int16_t title_marquee_offset;  // px scrolled left (0..span)
  int16_t title_marquee_span;    // px of overflow to reveal; 0 => fits, no scroll
  uint32_t title_marquee_elapsed_ms;  // position within the scroll cycle
  bool title_marquee_on;         // art-mode title/artist layout applied (transition guard)
  // Stock TextLayer render, used for the artist/title layers off album art; over art we draw them
  // ourselves (outlined artist, scrolling title).
  LayerUpdateProc orig_text_update_proc;
  // Separate subscription for preference changes, so toggling "Show Album Art" from the phone while
  // the app is open flips the UI immediately instead of only on the next app open.
  EventServiceInfo pref_change_event_info;

  ProgressLayer track_pos_bar;
  uint32_t track_length;
  uint32_t track_pos;
  bool pause_track_pos_updates;

  enum ActionBarState action_bar_state;
  AppTimer *action_bar_revert_timer;
  AppTimer *volume_repeat_timer;
  bool volume_is_up;

  MusicNoMusicWindow *no_music_window;

  VibeScore *score;
  bool temporarily_show_progress;
  AppTimer *temporarily_show_progress_timer;
} MusicAppData;

//! True when the screen uses the media layout (backdrop + centred stack) rather than the stock
//! layout. Always on for the unified round layout; on rect only while art is showing.
static bool prv_use_media_layout(MusicAppData *data) {
#if MUSIC_ROUND_MEDIA_LAYOUT
  (void)data;
  return true;
#else
  return data->has_album_art;
#endif
}

static void prv_set_action_bar_state(MusicAppData *data, enum ActionBarState state);

static void prv_trigger_cassette_icon_switch(GBitmap *bitmap, bool animated);
static void prv_update_cassette_icon(MusicAppData *data, bool animated);


// Add these two:
static void prv_update_layout(MusicAppData *data);
static void prv_set_pos_update_timer(MusicAppData *data, MusicPlayState playstate);
static void prv_apply_art_appearance(MusicAppData *data);
static void prv_maybe_request_album_art(void);


static void prv_do_haptic_feedback_vibe(MusicAppData *data) {
  vibe_score_do_vibe(data->score);
}

static void prv_handle_volume_icon_timer(void *context) {
  MusicAppData *data = context;
  data->volume_icon_timer = NULL;
  prv_update_cassette_icon(data, true);
}

static void prv_show_volume_image(GBitmap *bitmap) {
  MusicAppData *data = app_state_get_user_data();
  if (data->volume_icon_timer) {
    app_timer_reschedule(data->volume_icon_timer, VOLUME_ICON_TIMEOUT_MS);
  } else {
    data->volume_icon_timer = app_timer_register(VOLUME_ICON_TIMEOUT_MS,
                                                 prv_handle_volume_icon_timer, data);
  }
  prv_trigger_cassette_icon_switch(bitmap, true);
}

static void prv_change_volume(bool volume_is_up) {
  if (!music_is_command_supported(MusicCommandVolumeUp) ||
      !music_is_command_supported(MusicCommandVolumeDown)) {
    return;
  }

  MusicAppData *data = app_state_get_user_data();
  prv_show_volume_image(volume_is_up ? &data->image_volume_up : &data->image_volume_down);
  music_command_send(volume_is_up ? MusicCommandVolumeUp : MusicCommandVolumeDown);
}

static Animation* prv_create_layer_upwards_animation(Layer *layer, int16_t offset) {
  GPoint target = GPoint(0, -layer->bounds.size.h - offset);
  GPoint origin = GPoint(0, -offset);
  Animation *animation = property_animation_get_animation(
      property_animation_create_bounds_origin(layer, &origin, &target));
  animation_set_duration(animation, 4 * ANIMATION_FRAME_MS);
  animation_set_curve(animation, AnimationCurveEaseIn);
  return animation;
}

static Animation* prv_create_upwards_animation(MusicAppData *data) {
  return animation_spawn_create(
      prv_create_layer_upwards_animation(&data->artist_text_layer.layer, ARTIST_BOUNDS_OFFSET),
      prv_create_layer_upwards_animation(&data->title_text_layer.layer, TITLE_BOUNDS_OFFSET),
      prv_create_layer_upwards_animation(&data->position_text_layer.layer, TIME_BOUNDS_OFFSET),
      prv_create_layer_upwards_animation(&data->length_text_layer.layer, TIME_BOUNDS_OFFSET),
      NULL);
}

static const PropertyAnimationImplementation s_frame_layer_implementation = {
    .base = {
        .update = (AnimationUpdateImplementation) property_animation_update_grect,
    },
    .accessors = {
        .setter = { .grect = (const GRectSetter) layer_set_frame_by_value, },
        .getter = { .grect = (const GRectGetter) layer_get_frame_by_value, },
    },
};

static Animation *prv_create_layer_bounceback_animation(Layer *layer, GRect origin) {
  GRect target = origin;
  origin.origin.y -= BOUNCEBACK_OFFSET;
  Animation *animation = property_animation_get_animation(
      property_animation_create(&s_frame_layer_implementation, layer, &origin, &target));
  animation_set_duration(animation, 4 * ANIMATION_FRAME_MS);
  animation_set_curve(animation, AnimationCurveEaseOut);
  return animation;
}

static Animation *prv_create_bounceback_animation(MusicAppData *data) {
  const GRect time_rect = prv_time_rect();
  return animation_spawn_create(
      prv_create_layer_bounceback_animation(&data->artist_text_layer.layer, prv_artist_rect()),
      prv_create_layer_bounceback_animation(&data->title_text_layer.layer, prv_title_rect()),
      prv_create_layer_bounceback_animation(&data->position_text_layer.layer, time_rect),
      prv_create_layer_bounceback_animation(&data->length_text_layer.layer, time_rect),
      NULL);
}

static void prv_update_track_progress(MusicAppData *data);
static void prv_update_now_playing(MusicAppData *data);

static void prv_flip_animated_text(Animation *animation, bool finished, void *context) {
  MusicAppData *data = context;
  data->pause_track_pos_updates = false;
  prv_update_track_progress(data);
  music_get_now_playing(data->title_buffer, data->artist_buffer, NULL);
  // Restore the layers to their original bounds for the next part of the animation.
  data->title_text_layer.layer.bounds.origin.y = -TITLE_BOUNDS_OFFSET;
  data->artist_text_layer.layer.bounds.origin.y = -ARTIST_BOUNDS_OFFSET;
  data->position_text_layer.layer.bounds.origin.y = -TIME_BOUNDS_OFFSET;
  data->length_text_layer.layer.bounds.origin.y = -TIME_BOUNDS_OFFSET;
}

static inline bool prv_should_animate_casssette(void) {
  return music_get_playback_state() != MusicPlayStatePaused;
}

static Animation *prv_create_cassette_animation(MusicAppData *data) {
  const MusicAppSizeConfig *config = prv_config();
  const GRect cassette_rect = prv_cassette_rect();
  GPoint left_target = GPoint(-cassette_rect.size.w - cassette_rect.origin.x, 0);
  Animation *cassette_left = property_animation_get_animation(
      property_animation_create_bounds_origin(&data->cassette_container, &GPointZero,
                                              &left_target));
  Animation *cassette_right = property_animation_get_animation(
      property_animation_create_bounds_origin(&data->cassette_container,
          &GPoint(config->cassette_animation_x, 0), &GPoint(-4, 0)));
  Animation *cassette_bounceback = property_animation_get_animation(
      property_animation_create_bounds_origin(&data->cassette_container, &GPoint(-4, 0),
                                              &GPointZero));
  animation_set_duration(cassette_left, 4 * ANIMATION_FRAME_MS);
  animation_set_curve(cassette_left, AnimationCurveEaseIn);
  animation_set_duration(cassette_right, config->cassette_animation_time);
  animation_set_curve(cassette_left, AnimationCurveLinear);
  animation_set_duration(cassette_bounceback, 4 * ANIMATION_FRAME_MS);
  animation_set_curve(cassette_bounceback, AnimationCurveEaseOut);
  Animation *sequence = animation_sequence_create(cassette_left, cassette_right,
                                                  cassette_bounceback, NULL);
  if (!prv_should_animate_casssette()) {
    animation_set_play_count(sequence, 0);
  }
  return sequence;
}

static void prv_transition_stopped(Animation *animation, bool finished, void *context) {
  MusicAppData *data = context;
  data->transition = NULL;
  if (!finished) {
    return;
  }
  // Now-playing updates that arrive while the transition is scheduled are
  // dropped; re-check so the latest track is not lost.
  prv_update_now_playing(data);
}

static void prv_trigger_track_change_animation(MusicAppData *data) {
  // Animation structure:
  // - Master animation
  //   - Cassette animation
  //     - Move to left
  //     - Move in from right
  //     - Bounceback
  //   - Upwards animation
  //     - Per-layer animations
  //   - (flip text)
  //   - Bounceback animation
  //     - Per-layer animations

  if (animation_is_scheduled(data->transition)) {
    return;
  }
  data->pause_track_pos_updates = true;
  Animation *scroll_up = prv_create_upwards_animation(data);
  Animation *bounceback = prv_create_bounceback_animation(data);
  animation_set_handlers(scroll_up, (AnimationHandlers) {
      .stopped = prv_flip_animated_text,
  }, data);

  Animation *complete;
  complete = animation_sequence_create(scroll_up, bounceback, NULL);
  animation_set_handlers(complete, (AnimationHandlers) {
      .stopped = prv_transition_stopped,
  }, data);
  data->transition = complete;
  animation_schedule(complete);
}

static void prv_update_icon(Animation *animation, bool finished, void *context) {
  MusicAppData *data = app_state_get_user_data();
  GBitmap *bitmap = context;
  bitmap_layer_set_bitmap(&data->cassette_layer, bitmap);
  data->cassette_layer.layer.bounds.origin.y = 0;
}

static void prv_trigger_cassette_icon_switch(GBitmap *new_bitmap, bool animated) {
  MusicAppData *data = app_state_get_user_data();

  if (!animated) {
    bitmap_layer_set_bitmap(&data->cassette_layer, new_bitmap);
    data->cassette_current_icon = new_bitmap;
    return;
  }
  // Never animate an icon to itself. We can't use the current value of the bitmap layer itself,
  // because that will cause false positives if an icon change is triggered, but a revert
  // is triggered before the first half of the icon animation completes (currently 107 ms).
  if (new_bitmap == data->cassette_current_icon) {
    return;
  }

  GRect cassette_rect = prv_cassette_rect();
  Animation *disappear_animation = property_animation_get_animation(
      property_animation_create_bounds_origin(&data->cassette_layer.layer, &GPointZero,
                                              &GPoint(0, -cassette_rect.size.h)));
  animation_set_duration(disappear_animation, 3 * ANIMATION_FRAME_MS);
  animation_set_curve(disappear_animation, AnimationCurveEaseIn);

  GRect origin = cassette_rect;
  origin.origin.y -= BOUNCEBACK_OFFSET;

  Animation *bounceback_animation = property_animation_get_animation(
      property_animation_create(&s_frame_layer_implementation, &data->cassette_layer.layer,
                                &origin, &cassette_rect));
  animation_set_duration(bounceback_animation, 4 * ANIMATION_FRAME_MS);
  animation_set_curve(bounceback_animation, AnimationCurveEaseOut);

  animation_set_handlers(disappear_animation, (AnimationHandlers) {
      .stopped = prv_update_icon,
  }, new_bitmap);

  Animation *sequence = animation_sequence_create(disappear_animation, bounceback_animation, NULL);

  data->cassette_current_icon = new_bitmap;
  animation_schedule(sequence);
}

static void prv_skipping_click_config_provider(void *data);
static void prv_volume_click_config_provider(void *data);
static void prv_disable_progress(void *context) {
  MusicAppData *data = context;
  data->temporarily_show_progress = false;
  data->temporarily_show_progress_timer = NULL;
  prv_update_layout(data);
  prv_update_track_progress(data);
  prv_set_pos_update_timer(data, music_get_playback_state());
}

static void prv_show_progress_bar_temporarily(AccelAxisType axis, int32_t direction) {
  MusicAppData *data = app_state_get_user_data();
  data->temporarily_show_progress = true;
  if (data->temporarily_show_progress_timer) {
    app_timer_reschedule(data->temporarily_show_progress_timer, 5000);
  } else {
    data->temporarily_show_progress_timer = app_timer_register(5000, prv_disable_progress, data);
  }
  prv_update_layout(data);
  prv_update_track_progress(data);
  prv_set_pos_update_timer(data, music_get_playback_state());
}

static void prv_update_cassette_icon(MusicAppData *data, bool animated) {
  if (music_get_playback_state() == MusicPlayStatePaused) {
    prv_trigger_cassette_icon_switch(&data->image_pause, animated);
  } else {
    prv_trigger_cassette_icon_switch(&data->image_cassette, animated);
  }
}

static void prv_update_ui_state_skipping(MusicAppData *data, bool animated) {
  action_bar_layer_set_click_config_provider(&data->action_bar,
                                             prv_skipping_click_config_provider);
  action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_FORWARD,
                                     &data->icon_skip_forward, animated);
  action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_BACKWARD,
                                     &data->icon_skip_backward, animated);
  const bool show_volume_controls = shell_prefs_get_music_show_volume_controls();
  if (music_get_playback_state() == MusicPlayStatePaused) {
    action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_ID_SELECT, &data->icon_play,
                                       animated);
  } else if (show_volume_controls) {
    action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_ID_SELECT, &data->icon_ellipsis,
                                       animated);
  } else {
    action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_ID_SELECT, &data->icon_pause,
                                       animated);
  }
}

static void prv_update_ui_state_volume(MusicAppData *data, bool animated) {
  if (data->action_bar_state == ActionBarStateVolume) {
    action_bar_layer_set_click_config_provider(&data->action_bar,
                                               prv_volume_click_config_provider);
  }
  action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_ID_UP, &data->icon_volume_up,
                                     animated);
  action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_ID_DOWN, &data->icon_volume_down,
                                     animated);
  GBitmap const *select_bitmap;
  switch (music_get_playback_state()) {
    case MusicPlayStatePlaying: select_bitmap = &data->icon_pause; break;
    case MusicPlayStatePaused: select_bitmap = &data->icon_play; break;
    default: select_bitmap = &data->icon_play_pause; break;
  }
  action_bar_layer_set_icon_animated(&data->action_bar, BUTTON_ID_SELECT, select_bitmap, animated);
}

static void prv_update_ui_state(MusicAppData *data, bool animated) {
  if (data->action_bar_state == ActionBarStateSkip) {
    prv_update_ui_state_skipping(data, animated);
  } else {
    prv_update_ui_state_volume(data, animated);
  }

  if (music_get_playback_state() != data->current_play_state) {
    data->current_play_state = music_get_playback_state();
    prv_update_cassette_icon(data, animated);
  }
}

static void prv_set_action_bar_state(MusicAppData *data, enum ActionBarState state) {
  data->action_bar_state = state;
  prv_update_ui_state(data, true);
}

static void prv_action_bar_revert(void *context) {
  MusicAppData *data = context;
  data->action_bar_revert_timer = NULL;
  prv_set_action_bar_state(data, ActionBarStateSkip);
}

static void reset_action_bar_revert_timer(MusicAppData *data) {
  if (data->action_bar_revert_timer) {
    app_timer_reschedule(data->action_bar_revert_timer, ACTION_BAR_TIMEOUT_MS);
  }
}

static void prv_skip_click_handler(ClickRecognizerRef recognizer, void *context) {
  // no animations on tintin
  Animation *animation = prv_create_cassette_animation(context);
  animation_schedule(animation);
  if (click_recognizer_get_button_id(recognizer) == BUTTON_BACKWARD) {
    music_command_send(MusicCommandPreviousTrack);
  } else {
    music_command_send(MusicCommandNextTrack);
  }
}

static void prv_volume_click_handler(ClickRecognizerRef recognizer, void *context) {
  reset_action_bar_revert_timer(context);
  // TODO: absolute volume + volume indicator, when that information is available.
  bool is_volume_up = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_UP);
  prv_change_volume(is_volume_up);

  // Trigger haptic feedback only on repeat
  if (click_number_of_clicks_counted(recognizer) >= 2) {
    prv_do_haptic_feedback_vibe(context);
  }
}

static void prv_ellipsis_click_handler(ClickRecognizerRef recognizer, void *context) {
  if (!shell_prefs_get_music_show_volume_controls()) {
    music_command_send(MusicCommandTogglePlayPause);
    return;
  }

  MusicAppData *data = context;
  data->action_bar_revert_timer = app_timer_register(ACTION_BAR_TIMEOUT_MS, prv_action_bar_revert,
                                                     data);
  prv_set_action_bar_state(data, ActionBarStateVolume);
}

static void prv_toggle_playing(void) {
  music_command_send(MusicCommandTogglePlayPause);
}

static void prv_play_pause_click_handler(ClickRecognizerRef recognizer, void *context) {
  reset_action_bar_revert_timer(context);
  prv_toggle_playing();
}

static void prv_handle_volume_repeat(void *context) {
  MusicAppData *data = context;
  if (!data->volume_repeat_timer) {
    return;
  }
  data->volume_repeat_timer = app_timer_register(VOLUME_REPEAT_INTERVAL_MS,
                                                 prv_handle_volume_repeat, data);
  prv_change_volume(data->volume_is_up);
  prv_do_haptic_feedback_vibe(context);
}

static void prv_volume_long_click_start_handler(ClickRecognizerRef recognizer, void *context) {
  const bool volume_is_up = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_UP);
  prv_change_volume(volume_is_up);
  prv_set_action_bar_state(context, ActionBarStateLongPress);
  MusicAppData *data = context;
  data->volume_is_up = volume_is_up;
  data->volume_repeat_timer = app_timer_register(VOLUME_REPEAT_INTERVAL_MS,
                                                 prv_handle_volume_repeat, data);
  prv_do_haptic_feedback_vibe(data);
}

static void prv_volume_long_click_end_handler(ClickRecognizerRef recognizer, void *context) {
  MusicAppData *data = context;
  prv_set_action_bar_state(data, ActionBarStateSkip);
  app_timer_cancel(data->volume_repeat_timer);
  data->volume_repeat_timer = NULL;
}

static void prv_play_pause_long_click_start_handler(ClickRecognizerRef recognizer, void *context) {
  if (!shell_prefs_get_music_show_volume_controls()) {
    return;
  }

  prv_toggle_playing();
  prv_set_action_bar_state(context, ActionBarStateLongPress);
  prv_do_haptic_feedback_vibe(context);
}

static void prv_play_pause_long_click_end_handler(ClickRecognizerRef recognizer, void *context) {
  prv_set_action_bar_state(context, ActionBarStateSkip);
}

static void prv_skipping_click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_skip_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_skip_click_handler);
  const bool show_volume_controls = shell_prefs_get_music_show_volume_controls();
  if (music_get_playback_state() == MusicPlayStatePaused) {
    window_single_click_subscribe(BUTTON_ID_SELECT, prv_play_pause_click_handler);
  } else if (show_volume_controls) {
    window_single_click_subscribe(BUTTON_ID_SELECT, prv_ellipsis_click_handler);
  } else {
    window_single_click_subscribe(BUTTON_ID_SELECT, prv_play_pause_click_handler);
  }
  if (show_volume_controls) {
    window_long_click_subscribe(BUTTON_ID_UP, 0, prv_volume_long_click_start_handler,
                                prv_volume_long_click_end_handler);
    window_long_click_subscribe(BUTTON_ID_DOWN, 0, prv_volume_long_click_start_handler,
                                prv_volume_long_click_end_handler);
    window_long_click_subscribe(BUTTON_ID_SELECT, 0, prv_play_pause_long_click_start_handler,
                                prv_play_pause_long_click_end_handler);
  }
}

static void prv_volume_click_config_provider(void *context) {
  window_single_repeating_click_subscribe(BUTTON_ID_UP, VOLUME_REPEAT_INTERVAL_MS,
                                          prv_volume_click_handler);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, VOLUME_REPEAT_INTERVAL_MS,
                                          prv_volume_click_handler);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_play_pause_click_handler);
}

static void prv_update_layout(MusicAppData *data) {
  const bool show_progress_bar = shell_prefs_get_music_show_progress_bar() || data->temporarily_show_progress;
  bool hide_layer = !show_progress_bar || !music_is_progress_reporting_supported();
#if MUSIC_ROUND_MEDIA_LAYOUT
  // The unified round layout draws its own bezel arc and times on the backdrop; the stock bar and
  // time labels stay hidden permanently, and the backdrop repaints when visibility changes.
  hide_layer = true;
  layer_mark_dirty(&data->album_art_layer);
#endif
  layer_set_hidden(&data->track_pos_bar.layer, hide_layer);
  layer_set_hidden(&data->position_text_layer.layer, hide_layer);
  layer_set_hidden(&data->length_text_layer.layer, hide_layer);
}

static void prv_unload_no_music_window(Window *window) {
  MusicNoMusicWindow *music_window = (MusicNoMusicWindow *)window;
  gbitmap_deinit(&music_window->bitmap);
  bitmap_layer_deinit(&music_window->bitmap_layer);
  text_layer_deinit(&music_window->text_layer);
  i18n_free_all(music_window);
}

static void prv_handle_no_music_back(ClickRecognizerRef recognizer, void *context) {
  app_window_stack_pop_all(true);
}

static void prv_no_music_window_click_config(void *context) {
  window_single_click_subscribe(BUTTON_ID_BACK, prv_handle_no_music_back);
}

static MusicNoMusicWindow *prv_create_no_music_window(void) {
  MusicNoMusicWindow *window = app_malloc_check(sizeof(MusicNoMusicWindow));
  window_init(&window->window, WINDOW_NAME("NoMusicWindow"));
  window_set_background_color(&window->window, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  window_set_window_handlers(&window->window, &(WindowHandlers) {
      .unload = prv_unload_no_music_window
  });

  const MusicAppSizeConfig *config = prv_config();

  gbitmap_init_with_resource(&window->bitmap, RESOURCE_ID_MUSIC_IMAGE_NO_MUSIC);
  const GSize NO_MUSIC_IMAGE_SIZE = window->bitmap.bounds.size;
  const GRect NO_MUSIC_IMAGE_RECT = GRect(config->no_music_img_pos.x,
                                          config->no_music_img_pos.y,
                                          NO_MUSIC_IMAGE_SIZE.w, NO_MUSIC_IMAGE_SIZE.h);
  bitmap_layer_init(&window->bitmap_layer, &NO_MUSIC_IMAGE_RECT);
  bitmap_layer_set_bitmap(&window->bitmap_layer, &window->bitmap);
  bitmap_layer_set_compositing_mode(&window->bitmap_layer, GCompOpSet);

  const GRect NO_MUSIC_TEXT_RECT = GRect(0, config->no_music_text_field.origin_y,
                                         DISP_COLS, config->no_music_text_field.size_h);

  text_layer_init_with_parameters(&window->text_layer,
                                  &NO_MUSIC_TEXT_RECT,
                                  i18n_get("START PLAYBACK\nON YOUR PHONE", window),
                                  fonts_get_system_font(config->no_music_font_key),
                                  GColorBlack, GColorClear, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->window.layer, &window->bitmap_layer.layer);
  layer_add_child(&window->window.layer, &window->text_layer.layer);
  window_set_click_config_provider(&window->window, prv_no_music_window_click_config);
  return window;
}

static void prv_push_no_music_window(MusicAppData *data) {
  if (data->no_music_window) {
    return;
  }
  data->no_music_window = prv_create_no_music_window();
  app_window_stack_push(&data->no_music_window->window, false);
}

static void prv_pop_no_music_window(MusicAppData *data) {
  if (data->no_music_window) {
    app_window_stack_remove(&data->no_music_window->window, true);
    app_free(data->no_music_window);
    data->no_music_window = NULL;
  }
}

static void prv_update_now_playing(MusicAppData *data) {
  prv_update_layout(data);

  char artist_buffer[MUSIC_BUFFER_LENGTH];
  char title_buffer[MUSIC_BUFFER_LENGTH];
  music_get_now_playing(title_buffer, artist_buffer, NULL);

  if (music_needs_user_to_start_playback_on_phone()) {
    prv_push_no_music_window(data);
  } else {
    prv_pop_no_music_window(data);
  }

  bool title_changed = strncmp(data->title_buffer, title_buffer, MUSIC_BUFFER_LENGTH) != 0;
  bool artist_changed = strncmp(data->artist_buffer, artist_buffer, MUSIC_BUFFER_LENGTH) != 0;
  if (title_changed) {
    // Force the marquee to re-measure the new title on the next appearance pass.
    data->title_marquee_on = false;
  }
  if (title_changed || artist_changed) {
    // The slide animation bounces the artist/title layers back to their stock positions, which
    // fights the media layout, so there just swap the text. Also skip it when there's nothing
    // to animate from.
    if (prv_use_media_layout(data) ||
        (data->artist_buffer[0] == 0 && data->title_buffer[0] == 0)) {
      strncpy(data->artist_buffer, artist_buffer, MUSIC_BUFFER_LENGTH);
      strncpy(data->title_buffer, title_buffer, MUSIC_BUFFER_LENGTH);
      // It is sufficient to mark one layer as dirty.
      layer_mark_dirty(&data->title_text_layer.layer);
    } else if (title_changed) {
      prv_trigger_track_change_animation(data);
    }
  }
  // Re-request art whenever the service reports a new track (generation bumps on title/artist/album
  // change); the service has already dropped the old art by this point.
  const uint8_t generation = music_get_now_playing_generation();
  if (generation != data->last_art_generation) {
    data->last_art_generation = generation;
    prv_maybe_request_album_art();
  }
  prv_apply_art_appearance(data);
  prv_update_layout(data);
}

static void prv_copy_time_period(char *buffer, size_t n, uint32_t period_s) {
  uint32_t hours = period_s / SECONDS_PER_HOUR;
  uint32_t minutes = (period_s % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE;
  uint32_t seconds = period_s % SECONDS_PER_MINUTE;
#pragma GCC diagnostic ignored "-Wformat-truncation"
  if (hours > 0) {
    snprintf(buffer, n, "%"PRIu32":%02"PRIu32":%02"PRIu32, hours, minutes, seconds);
  } else {
    snprintf(buffer, n, "%"PRIu32":%02"PRIu32, minutes, seconds);
  }
}

static void prv_update_track_progress(MusicAppData *data) {
  if (data->pause_track_pos_updates) {
    return;
  }

  if (!data->temporarily_show_progress && !shell_prefs_get_music_show_progress_bar()){
    return;
  }

  if (!music_is_progress_reporting_supported()) {
    progress_layer_set_progress(&data->track_pos_bar, 0);
  } else {
    unsigned int percent = 0;
    if (data->track_length > 0) {
      percent = MIN((data->track_pos * 100) / data->track_length, 100);
    }
    progress_layer_set_progress(&data->track_pos_bar, percent);
    prv_copy_time_period(data->position_buffer, sizeof(data->position_buffer),
                         data->track_pos / 1000);
    prv_copy_time_period(data->length_buffer, sizeof(data->length_buffer),
                         data->track_length / 1000);
  }
#if MUSIC_ROUND_MEDIA_LAYOUT
  // The arc and times draw on the backdrop layer; refresh it with each progress update.
  layer_mark_dirty(&data->album_art_layer);
#endif
}

static void prv_update_pos(void) {
  MusicAppData *data = app_state_get_user_data();
  music_get_pos(&data->track_pos, &data->track_length);
  prv_update_track_progress(data);
}

static void prv_handle_tick_time(struct tm *time, TimeUnits units_changed) {
  if (music_get_playback_state() == MusicPlayStatePlaying) {
    prv_update_pos();
  }
}

static void prv_set_pos_update_timer(MusicAppData* data, MusicPlayState playstate) {
  if (!music_is_progress_reporting_supported() || (!shell_prefs_get_music_show_progress_bar() && !data->temporarily_show_progress)) {
    tick_timer_service_unsubscribe();
    return;
  }
  switch (playstate) {
    case MusicPlayStatePlaying:
      // We need to update the progress bar every second.
      tick_timer_service_subscribe(SECOND_UNIT, prv_handle_tick_time);
      break;
    default:
      // We're no longer updating the progress bar; unsubscribe.
      tick_timer_service_unsubscribe();
  }
}

static void prv_configure_music_text_layer(
    TextLayer *text_layer, char* text_buffer, const GRect *rect, int16_t y_offset,
    GTextAlignment align, GFont font) {
  text_layer_init_with_parameters(text_layer, rect, text_buffer, font,
                                  GColorBlack, GColorClear, align, GTextOverflowModeFill);
  layer_set_bounds(&text_layer->layer, &GRect(0, -y_offset,
                                              rect->size.w, rect->size.h + y_offset));
}

// The album-art square fills the width from the top down to just above the times row. On rect
// screens it stops left of the action bar; on round screens it spans the full display and the
// action bar overlaps its right edge, so the round cover reaches edge to edge.
static int16_t prv_art_width(void) {
  return PBL_IF_RECT_ELSE(DISP_COLS - ACTION_BAR_WIDTH, DISP_COLS);
}

static GRect prv_album_art_rect(void) {
  // Rect: a square band from the top down to just above the times row. Round: the cover fills the
  // whole display (it is already fetched at the full width) and the circle masks it.
  const int16_t art_h = PBL_IF_RECT_ELSE(prv_config()->time_field.origin_y - 2, DISP_ROWS);
  return GRect(0, 0, prv_art_width(), art_h);
}

// In album-art mode the track title moves down beside the tape (to the tape's right on rect
// screens, to its left on round ones), where the artist normally never was — so it reads on the
// plain background below the cover. Kept tight so as much of the title shows as possible.
static GRect prv_art_title_rect(void) {
#if PBL_RECT
  const int16_t h = 28;
  const MusicAppSizeConfig *config = prv_config();
  const GRect tape = prv_cassette_rect();
  // Clear the tape frame plus a gap: the widest state icon (volume) fills the whole frame.
  const int16_t x = tape.origin.x + tape.size.w + 2;
  const int16_t right = DISP_COLS - ACTION_BAR_WIDTH - 7;
  // Single line, vertically centred in the band between the progress bar and the bottom of screen,
  // nudged up slightly since the glyph sits a touch low within the line box.
  const int16_t bar_bottom = config->track_field.origin_y + config->track_field.size_h;
  const int16_t y = bar_bottom + (DISP_ROWS - bar_bottom - h) / 2 - 5;
#else
  // Round: the centrepiece block across the middle of the circle. Clamp to the visible chord at
  // the block's lowest row so a full-height title never starts behind the bezel.
  const int16_t h = ART_ROUND_TITLE_H;
  const int16_t y = ART_ROUND_TITLE_Y;
  const int16_t radius = DISP_COLS / 2;
  const int32_t dy = (y + h - 2) - radius;
  const int16_t half_chord =
      integer_sqrt((int64_t)radius * radius - (int64_t)dy * dy);
  const int16_t x = MAX(radius - half_chord + 4, ART_ROUND_TEXT_MARGIN);
  const int16_t right = DISP_COLS - x;
#endif
  return GRect(x, y, right - x, h);
}

// Artist name, centred over the cover. Rect: just above the track-info row; round: the top of the
// centred text stack in the lower half.
static GRect prv_art_artist_rect(void) {
#if PBL_RECT
  const int16_t content_w = DISP_COLS - ACTION_BAR_WIDTH;
  const int16_t art_bottom = prv_config()->time_field.origin_y - 2;
  return GRect(0, art_bottom - 26, content_w, 26);
#else
  return GRect(ART_ROUND_TEXT_MARGIN, ART_ROUND_ARTIST_Y,
               DISP_COLS - 2 * ART_ROUND_TEXT_MARGIN, 24);
#endif
}

// Draw text with a 1px black outline (8 offset copies) then white on top, so it pops over the art.
static void prv_draw_outlined_text(GContext *ctx, const char *text, GFont font, GRect box,
                                   GTextOverflowMode overflow, GTextAlignment align) {
  static const GPoint k_off[] = {
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1},
  };
  graphics_context_set_text_color(ctx, GColorBlack);
  for (unsigned i = 0; i < sizeof(k_off) / sizeof(k_off[0]); ++i) {
    GRect b = box;
    b.origin.x += k_off[i].x;
    b.origin.y += k_off[i].y;
    graphics_draw_text(ctx, text, font, b, overflow, align, NULL);
  }
  graphics_context_set_text_color(ctx, GColorWhite);
  graphics_draw_text(ctx, text, font, box, overflow, align, NULL);
}

// Artist layer: outlined + centred over the art; stock render (black, its normal spot) otherwise.
static void prv_artist_update_proc(Layer *layer, GContext *ctx) {
  MusicAppData *data = app_state_get_user_data();
  const TextLayer *tl = (const TextLayer *)layer;
  if (!data->has_album_art) {
    if (data->orig_text_update_proc) {
      data->orig_text_update_proc(layer, ctx);
    }
    return;
  }
  if (tl->text && tl->text[0]) {
    prv_draw_outlined_text(ctx, tl->text, tl->font, layer->bounds, tl->overflow_mode,
                           tl->text_alignment);
  }
}

// Title layer: glance-style scrolling single line over art; stock render otherwise.
static void prv_title_update_proc(Layer *layer, GContext *ctx) {
  MusicAppData *data = app_state_get_user_data();
  const TextLayer *tl = (const TextLayer *)layer;
#if !MUSIC_ROUND_MEDIA_LAYOUT
  if (!data->has_album_art) {
    if (data->orig_text_update_proc) {
      data->orig_text_update_proc(layer, ctx);
    }
    return;
  }
#endif
  if (!tl->text || !tl->text[0]) {
    return;
  }
#if PBL_RECT
  GRect b = layer->bounds;
  b.origin.x = -data->title_marquee_offset;
  graphics_context_set_text_color(ctx, tl->text_color);
  graphics_draw_text(ctx, tl->text, tl->font, b, GTextOverflowModeFill, GTextAlignmentLeft, NULL);
#else
  // Unified round layout: the title wraps to up to three centred lines (ellipsised beyond) and
  // the block floats vertically centred in its region; outlined over art, plain otherwise.
  GRect box = layer->bounds;
  const GSize used = graphics_text_layout_get_max_used_size(
      ctx, tl->text, tl->font, box, GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  // The -4 backs out the glyph box's dead space above the caps so the INK centres in the region
  // (which is itself centred on the display), lining a one-line title up with the action bar's
  // middle icon.
  const int16_t off = MAX((box.size.h - used.h) / 2 - 4, 0);
  box.origin.y += off;
  box.size.h -= off;
  if (data->has_album_art) {
    prv_draw_outlined_text(ctx, tl->text, tl->font, box, GTextOverflowModeTrailingEllipsis,
                           GTextAlignmentCenter);
  } else {
    graphics_context_set_text_color(ctx, tl->text_color);
    graphics_draw_text(ctx, tl->text, tl->font, box, GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentCenter, NULL);
  }
#endif
}

#if MUSIC_ROUND_MEDIA_LAYOUT
//! Fill part of the bezel arc band. Degrees run clockwise from 12 o'clock and the range may pass
//! through 360 (the fill is split there).
static void prv_fill_bezel_arc(GContext *ctx, const GRect *bounds, int32_t from_deg,
                               int32_t to_deg) {
  if (to_deg <= from_deg) {
    return;
  }
  if (to_deg <= 360) {
    graphics_fill_radial(ctx, *bounds, GOvalScaleModeFitCircle, ART_ROUND_RING_THICKNESS,
                         DEG_TO_TRIGANGLE(from_deg), DEG_TO_TRIGANGLE(to_deg));
  } else {
    graphics_fill_radial(ctx, *bounds, GOvalScaleModeFitCircle, ART_ROUND_RING_THICKNESS,
                         DEG_TO_TRIGANGLE(from_deg), TRIG_MAX_ANGLE);
    graphics_fill_radial(ctx, *bounds, GOvalScaleModeFitCircle, ART_ROUND_RING_THICKNESS, 0,
                         DEG_TO_TRIGANGLE(to_deg - 360));
  }
}

// Progress arc along the bezel (its gap hugging the action bar) plus a small centred
// elapsed/total line: the round replacements for the stock bar and time labels.
static void prv_draw_round_progress(GContext *ctx, const GRect *bounds) {
  MusicAppData *data = app_state_get_user_data();
  if (!(data->temporarily_show_progress || shell_prefs_get_music_show_progress_bar()) ||
      !music_is_progress_reporting_supported()) {
    return;
  }
  graphics_context_set_fill_color(ctx, GColorBlack);
  prv_fill_bezel_arc(ctx, bounds, ART_ROUND_ARC_START_DEG, ART_ROUND_ARC_END_DEG);
  if (data->track_length > 0) {
    const int32_t sweep = ART_ROUND_ARC_END_DEG - ART_ROUND_ARC_START_DEG;
    const int32_t end = ART_ROUND_ARC_START_DEG +
        MIN((int32_t)(((int64_t)sweep * data->track_pos) / data->track_length), sweep);
    graphics_context_set_fill_color(ctx, GColorRed);
    prv_fill_bezel_arc(ctx, bounds, ART_ROUND_ARC_START_DEG, end);
  }
  if (data->position_buffer[0] && data->length_buffer[0]) {
    char times[24];
    snprintf(times, sizeof(times), "%s / %s", data->position_buffer, data->length_buffer);
    const GRect box = GRect(ART_ROUND_TEXT_MARGIN, ART_ROUND_TIMES_Y,
                            DISP_COLS - 2 * ART_ROUND_TEXT_MARGIN, 22);
    GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
    if (data->has_album_art) {
      prv_draw_outlined_text(ctx, times, font, box, GTextOverflowModeFill, GTextAlignmentCenter);
    } else {
      graphics_context_set_text_color(ctx, GColorBlack);
      graphics_draw_text(ctx, times, font, box, GTextOverflowModeFill, GTextAlignmentCenter,
                         NULL);
    }
  }
}
#endif

static void prv_album_art_update_proc(Layer *layer, GContext *ctx) {
  // Hold the art locked for the whole draw so the service can't free it mid-blit.
  const GBitmap *art = music_album_art_lock();
  if (art && shell_prefs_get_music_show_album_art()) {
    // The phone renders the whole cover to the content width, so it fills the square; centre it in
    // case the box is a touch shorter than the cover.
    const GSize sz = art->bounds.size;
    const GRect b = layer->bounds;
    const GRect dst = GRect((b.size.w - sz.w) / 2, (b.size.h - sz.h) / 2, sz.w, sz.h);
    graphics_context_set_compositing_mode(ctx, GCompOpAssign);
    graphics_draw_bitmap_in_rect(ctx, art, &dst);
  }
  music_album_art_unlock();
#if MUSIC_ROUND_MEDIA_LAYOUT
  // The layer doubles as the round backdrop: the arc and times draw with or without a cover.
  prv_draw_round_progress(ctx, &layer->bounds);
#endif
}

//! Request album art for the current track, but only when it would actually be shown and we don't
//! already have art for this track (keyed off the now-playing generation, not "is any art present",
//! so a track change re-fetches).
static void prv_maybe_request_album_art(void) {
#if MUSIC_ALBUM_ART_SUPPORTED
  if (!shell_prefs_get_music_show_album_art() ||
      !imaging_is_type_supported(ImagingImageTypeAlbumArt) ||
      !music_has_now_playing() || music_album_art_is_current()) {
    return;
  }
  // The watch picks the size (its art-square width) and asks for the current track by name, so the
  // phone returns art that matches what we're showing.
  const int16_t side = prv_art_width();
  char title[MUSIC_BUFFER_LENGTH];
  char artist[MUSIC_BUFFER_LENGTH];
  music_get_now_playing(title, artist, NULL);
  imaging_request_album_art(music_get_now_playing_generation(), ImagingFormat4BitPalette,
                            side, side, title, artist);
#endif
}

// Launcher-glance-style title scroll (same motion as the app-glance subtitles, whose pacing was
// design-tuned): pause at the start, scroll just far enough to reveal the end at a moderate pace,
// pause there, rewind rapidly. Runs a few cycles per track, then rests showing the start.
#define TITLE_SCROLL_TICK_MS 33            // frame interval while the text is moving
#define TITLE_SCROLL_MS_PER_PX 20          // forward pace
#define TITLE_SCROLL_REWIND_MS_PER_PX 2    // rewind pace
#define TITLE_SCROLL_PAUSE_START_MS 600
#define TITLE_SCROLL_PAUSE_END_MS 750
#define TITLE_SCROLL_CYCLES 3

static void prv_title_marquee_stop(MusicAppData *data) {
  if (data->title_marquee_timer) {
    app_timer_cancel(data->title_marquee_timer);
    data->title_marquee_timer = NULL;
  }
}

#if !MUSIC_ROUND_MEDIA_LAYOUT
// Advance the pause / scroll / pause / rewind cycle. Redraws only when the offset changes, and
// sleeps through the pauses in one go instead of ticking. (The unified round layout wraps the
// title instead of scrolling it, so the cycle only exists on rect.)
static void prv_title_marquee_cb(void *context) {
  MusicAppData *data = context;
  data->title_marquee_timer = NULL;
  if (!data->has_album_art || data->title_marquee_span == 0) {
    data->title_marquee_offset = 0;
    layer_mark_dirty(&data->title_text_layer.layer);
    return;
  }
  const uint32_t fwd_ms = (uint32_t)data->title_marquee_span * TITLE_SCROLL_MS_PER_PX;
  const uint32_t rewind_ms = (uint32_t)data->title_marquee_span * TITLE_SCROLL_REWIND_MS_PER_PX;
  const uint32_t cycle_ms =
      TITLE_SCROLL_PAUSE_START_MS + fwd_ms + TITLE_SCROLL_PAUSE_END_MS + rewind_ms;
  if (data->title_marquee_elapsed_ms / cycle_ms >= TITLE_SCROLL_CYCLES) {
    data->title_marquee_offset = 0;  // done: rest showing the start
    layer_mark_dirty(&data->title_text_layer.layer);
    return;
  }
  const uint32_t t = data->title_marquee_elapsed_ms % cycle_ms;

  int16_t offset;
  uint32_t sleep_ms;
  if (t < TITLE_SCROLL_PAUSE_START_MS) {
    offset = 0;
    sleep_ms = TITLE_SCROLL_PAUSE_START_MS - t;
  } else if (t < TITLE_SCROLL_PAUSE_START_MS + fwd_ms) {
    offset = (t - TITLE_SCROLL_PAUSE_START_MS) / TITLE_SCROLL_MS_PER_PX;
    sleep_ms = TITLE_SCROLL_TICK_MS;
  } else if (t < TITLE_SCROLL_PAUSE_START_MS + fwd_ms + TITLE_SCROLL_PAUSE_END_MS) {
    offset = data->title_marquee_span;
    sleep_ms = TITLE_SCROLL_PAUSE_START_MS + fwd_ms + TITLE_SCROLL_PAUSE_END_MS - t;
  } else {
    offset = (cycle_ms - t) / TITLE_SCROLL_REWIND_MS_PER_PX;
    sleep_ms = TITLE_SCROLL_TICK_MS;
  }
  if (offset != data->title_marquee_offset) {
    data->title_marquee_offset = offset;
    layer_mark_dirty(&data->title_text_layer.layer);
  }
  data->title_marquee_elapsed_ms += sleep_ms;
  data->title_marquee_timer = app_timer_register(sleep_ms, prv_title_marquee_cb, data);
}
#endif

// Lay the title out on one line beside the tape and, if it overflows, arm the scroll cycle.
static void prv_title_marquee_setup(MusicAppData *data) {
  prv_title_marquee_stop(data);
  const GRect vis = prv_art_title_rect();
  // Explicit system font (not the theme Subtitle) so measured width matches what's drawn.
  text_layer_set_font(&data->title_text_layer, fonts_get_system_font(TITLE_SCROLL_FONT_KEY));
  layer_set_frame(&data->title_text_layer.layer, &vis);
  layer_set_clips(&data->title_text_layer.layer, true);
  data->title_marquee_offset = 0;
  data->title_marquee_elapsed_ms = 0;
#if MUSIC_ROUND_MEDIA_LAYOUT
  // Round: no scrolling — the title wraps to up to three centred lines in its block.
  text_layer_set_text_alignment(&data->title_text_layer, GTextAlignmentCenter);
  layer_set_bounds(&data->title_text_layer.layer, &GRect(0, 0, vis.size.w, vis.size.h));
  data->title_marquee_span = 0;
#else
  text_layer_set_text_alignment(&data->title_text_layer, GTextAlignmentLeft);
  // Wide, one-line bounds: measure here and keep it wide so the scroll proc never wraps a copy.
  layer_set_bounds(&data->title_text_layer.layer, &GRect(0, 0, DISP_COLS * 3, vis.size.h));
  const GSize text_size = app_text_layer_get_content_size(&data->title_text_layer);
  // The layout measurer can report a slightly wider box than graphics_draw_text inks; back that
  // out (~2px per glyph) only for the overflow decision so a barely-fitting title doesn't scroll.
  // The scroll distance uses the full measured width so the end pause always reveals the tail.
  const int16_t ink_w = text_size.w - 2 * (int16_t)strlen(data->title_buffer);
  if (ink_w > vis.size.w) {
    data->title_marquee_span = text_size.w - vis.size.w;
    data->title_marquee_timer =
        app_timer_register(TITLE_SCROLL_TICK_MS, prv_title_marquee_cb, data);
  } else {
    data->title_marquee_span = 0;
  }
#endif
  layer_mark_dirty(&data->title_text_layer.layer);
}

// Restore the title to its stock (wrapping, animated) layout for the no-art screen.
static void prv_title_restore(MusicAppData *data) {
  prv_title_marquee_stop(data);
  data->title_marquee_span = 0;
  const GRect title_rect = prv_title_rect();
  text_layer_set_font(&data->title_text_layer,
                      system_theme_get_font_for_default_size(TextStyleFont_Subtitle));
  text_layer_set_overflow_mode(&data->title_text_layer, GTextOverflowModeFill);
  text_layer_set_text_alignment(&data->title_text_layer,
                                PBL_IF_RECT_ELSE(GTextAlignmentLeft, GTextAlignmentRight));
  layer_set_frame(&data->title_text_layer.layer, &title_rect);
  layer_set_bounds(&data->title_text_layer.layer,
                   &GRect(0, -TITLE_BOUNDS_OFFSET, title_rect.size.w,
                          title_rect.size.h + TITLE_BOUNDS_OFFSET));
}

// Place the artist as a centred, outlined single line over the bottom of the cover.
static void prv_artist_setup_art(MusicAppData *data) {
  const GRect ar = prv_art_artist_rect();
  text_layer_set_font(&data->artist_text_layer, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
  text_layer_set_text_alignment(&data->artist_text_layer, GTextAlignmentCenter);
  text_layer_set_overflow_mode(&data->artist_text_layer, GTextOverflowModeTrailingEllipsis);
  layer_set_frame(&data->artist_text_layer.layer, &ar);
  layer_set_bounds(&data->artist_text_layer.layer, &GRect(0, 0, ar.size.w, ar.size.h));
}

// Restore the artist to its stock layout for the no-art screen.
static void prv_artist_restore(MusicAppData *data) {
  const GRect ar = prv_artist_rect();
  text_layer_set_font(&data->artist_text_layer,
                      system_theme_get_font_for_default_size(TextStyleFont_Header));
  text_layer_set_overflow_mode(&data->artist_text_layer, GTextOverflowModeFill);
  text_layer_set_text_alignment(&data->artist_text_layer,
                                PBL_IF_RECT_ELSE(GTextAlignmentLeft, GTextAlignmentRight));
  layer_set_frame(&data->artist_text_layer.layer, &ar);
  layer_set_bounds(&data->artist_text_layer.layer,
                   &GRect(0, -ARTIST_BOUNDS_OFFSET, ar.size.w, ar.size.h + ARTIST_BOUNDS_OFFSET));
}

//! Reflect the current art state: show/hide the cover, lay out the artist (outlined, over the art)
//! and the scrolling title, and swap the clock to the outlined big style so it stays legible over
//! the cover. A full-window repaint clears the moved layers' old pixels.
static void prv_apply_art_appearance(MusicAppData *data) {
  const GBitmap *art = music_album_art_lock();
  data->has_album_art = (art != NULL) && shell_prefs_get_music_show_album_art();
  music_album_art_unlock();

  layer_set_hidden(&data->album_art_layer, !prv_use_media_layout(data));
  if (prv_use_media_layout(data)) {
    // Rebuild the media layout only on the transition in (or on a track change, which clears the
    // flag) — re-running every event would restart the scroll and spawn overlapping timers.
    if (!data->title_marquee_on) {
      // The stock track-change slide may be mid-flight (it runs when the previous track had no
      // art). It animates the text layers back to their stock frames and would clobber the art
      // layout when the cover lands during the transition. Finish it instantly: unscheduling
      // fires its handlers (text flip, pos-update resume), then park the time labels where its
      // bounceback would have left them; the art setup below owns the title/artist frames.
      if (animation_is_scheduled(data->transition)) {
        animation_unschedule(data->transition);
        const GRect time_rect = prv_time_rect();
        layer_set_frame(&data->position_text_layer.layer, &time_rect);
        layer_set_frame(&data->length_text_layer.layer, &time_rect);
      }
      prv_artist_setup_art(data);
      prv_title_marquee_setup(data);
      data->title_marquee_on = true;
    }
  } else if (data->title_marquee_on) {
    prv_artist_restore(data);
    prv_title_restore(data);
    data->title_marquee_on = false;
  }
  // The state icons (tape / pause / volume) differ in size. In the media layout, centre them in
  // the tape frame so a state swap keeps the same visual centre; the stock layout keeps its
  // original corner pinning.
  bitmap_layer_set_alignment(&data->cassette_layer,
                             prv_use_media_layout(data)
                                 ? GAlignCenter
                                 : PBL_IF_RECT_ELSE(GAlignTopLeft, GAlignTopRight));
#if MUSIC_ROUND_MEDIA_LAYOUT
  // The bezel arc + drawn times replace the stock bar and time labels permanently on round.
  prv_update_layout(data);
#endif
  // The big clock belongs to the media layout. Over art it's white with a black outline (matching
  // the artist) so the time reads over the cover; in the media layout without art it's plain big
  // black. The stock (rectangular, no-art) layout keeps the normal status-bar clock.
  StatusBarLayerMode clock_mode;
  if (!prv_use_media_layout(data)) {
    clock_mode = StatusBarLayerModeClock;
  } else if (data->has_album_art) {
    clock_mode = StatusBarLayerModeClockLargeBoldOutlined;
  } else {
    clock_mode = StatusBarLayerModeClockLargeBold;
  }
  status_bar_layer_set_colors(&data->status_layer, GColorClear,
                              data->has_album_art ? GColorWhite : GColorBlack);
  status_bar_layer_set_mode(&data->status_layer, clock_mode);
  layer_mark_dirty(&data->window.layer);
}

static void prv_init_ui(Window *window) {
  MusicAppData *data = window_get_user_data(window);

  window_set_background_color(window, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));

  const GSize WINDOW_SIZE = window->layer.bounds.size;

  const GTextAlignment ARTIST_TITLE_TEXT_ALIGNMENT = PBL_IF_RECT_ELSE(GTextAlignmentLeft,
                                                                      GTextAlignmentRight);

  const MusicAppSizeConfig *config = prv_config();

  const GRect artist_rect = prv_artist_rect();
  const GRect title_rect = prv_title_rect();
  const GRect time_rect = prv_time_rect();
  const GRect cassette_rect = prv_cassette_rect();
  const GRect track_rect = prv_track_rect();

  // Album-art square across the top, behind the text. Hidden until art is actually shown.
  const GRect art_rect = prv_album_art_rect();
  layer_init(&data->album_art_layer, &art_rect);
  layer_set_update_proc(&data->album_art_layer, prv_album_art_update_proc);
  layer_set_hidden(&data->album_art_layer, true);
  layer_add_child(&data->window.layer, &data->album_art_layer);

  prv_configure_music_text_layer(&data->artist_text_layer, data->artist_buffer,
                                 &artist_rect, ARTIST_BOUNDS_OFFSET,
                                 ARTIST_TITLE_TEXT_ALIGNMENT,
                                 system_theme_get_font_for_default_size(TextStyleFont_Header));
  layer_add_child(&data->window.layer, &data->artist_text_layer.layer);

  prv_configure_music_text_layer(&data->position_text_layer, data->position_buffer, &time_rect,
                                 TIME_BOUNDS_OFFSET, GTextAlignmentLeft,
                                 fonts_get_system_font(config->music_time_font_key));
  layer_add_child(&data->window.layer, &data->position_text_layer.layer);

  prv_configure_music_text_layer(&data->length_text_layer, data->length_buffer, &time_rect,
                                 TIME_BOUNDS_OFFSET, GTextAlignmentRight,
                                 fonts_get_system_font(config->music_time_font_key));
  layer_add_child(&data->window.layer, &data->length_text_layer.layer);

  prv_configure_music_text_layer(&data->title_text_layer, data->title_buffer, &title_rect,
                                 TITLE_BOUNDS_OFFSET, ARTIST_TITLE_TEXT_ALIGNMENT,
                                 system_theme_get_font_for_default_size(TextStyleFont_Subtitle));
  text_layer_set_line_spacing_delta(&data->title_text_layer, -2);

  layer_add_child(&data->window.layer, &data->title_text_layer.layer);

  // Over album art we render the artist (outlined) and title (scrolling) ourselves; off art these
  // proc wrappers defer to the stock TextLayer render captured here.
  data->orig_text_update_proc = data->title_text_layer.layer.update_proc;
  layer_set_update_proc(&data->artist_text_layer.layer, prv_artist_update_proc);
  layer_set_update_proc(&data->title_text_layer.layer, prv_title_update_proc);

  const int16_t horizontal_margin = config->horizontal_margin;
  layer_init(&data->cassette_container, &GRect(0, WINDOW_SIZE.h - horizontal_margin - 24,
                                               WINDOW_SIZE.w - ACTION_BAR_WIDTH, 24));
  layer_add_child(&data->window.layer, &data->cassette_container);
  layer_set_clips(&data->cassette_container, false);

  bitmap_layer_init(&data->cassette_layer, &cassette_rect);
  bitmap_layer_set_bitmap(&data->cassette_layer, &data->image_cassette);
  data->cassette_current_icon = &data->image_cassette;
  const GAlign CASSETTE_LAYER_ALIGNMENT = PBL_IF_RECT_ELSE(GAlignTopLeft, GAlignTopRight);
  bitmap_layer_set_alignment(&data->cassette_layer, CASSETTE_LAYER_ALIGNMENT);
  bitmap_layer_set_compositing_mode(&data->cassette_layer, GCompOpSet);
  layer_add_child(&data->cassette_container, &data->cassette_layer.layer);

#if MUSIC_ROUND_MEDIA_LAYOUT
  // Unified round media layout: centred text stack across the widest band and the state icon
  // anchoring the bottom of the circle, in both the art and plain-background states.
  prv_artist_setup_art(data);
  const GRect round_title_rect = prv_art_title_rect();
  layer_set_frame(&data->title_text_layer.layer, &round_title_rect);
  text_layer_set_font(&data->title_text_layer, fonts_get_system_font(TITLE_SCROLL_FONT_KEY));
  text_layer_set_text_alignment(&data->title_text_layer, GTextAlignmentCenter);
  GRect tape_frame = GRect(0, WINDOW_SIZE.h - horizontal_margin - 24,
                           WINDOW_SIZE.w - ACTION_BAR_WIDTH, 24);
  tape_frame.origin.x += DISP_COLS / 2 - (cassette_rect.origin.x + cassette_rect.size.w / 2);
  tape_frame.origin.y = ART_ROUND_ICON_CENTER_Y - cassette_rect.origin.y - cassette_rect.size.h / 2;
  layer_set_frame(&data->cassette_container, &tape_frame);
  bitmap_layer_set_alignment(&data->cassette_layer, GAlignCenter);
#endif

  progress_layer_init(&data->track_pos_bar, &track_rect);
  progress_layer_set_background_color(&data->track_pos_bar,
                                      PBL_IF_COLOR_ELSE(GColorBlack, GColorWhite));
  progress_layer_set_foreground_color(&data->track_pos_bar,
                                      PBL_IF_COLOR_ELSE(GColorRed, GColorBlack));
  progress_layer_set_corner_radius(&data->track_pos_bar, config->track_corner_radius);
  layer_add_child(&window->layer, (Layer *)&data->track_pos_bar);

  ActionBarLayer *action_bar = &data->action_bar;
  data->action_bar_state = ActionBarStateSkip;
  action_bar_layer_init(action_bar);
  action_bar_layer_set_context(action_bar, data);
  action_bar_layer_add_to_window(action_bar, window);

  StatusBarLayer *status_layer = &data->status_layer;
  status_bar_layer_init(status_layer);
  GRect status_layer_frame = status_layer->layer.frame;
  const int16_t STATUS_BAR_LAYER_WIDTH = PBL_IF_RECT_ELSE(WINDOW_SIZE.w - ACTION_BAR_WIDTH,
                                                          WINDOW_SIZE.w);
  status_layer_frame.size.w = STATUS_BAR_LAYER_WIDTH;
  layer_set_frame(&status_layer->layer, &status_layer_frame);
  status_bar_layer_set_colors(&data->status_layer, GColorClear, GColorBlack);
  layer_add_child(&data->window.layer, &status_layer->layer);

  music_get_pos(&data->track_pos, &data->track_length);

  data->score = vibe_score_create_with_resource(RESOURCE_ID_VIBE_SCORE_HAPTIC_FEEDBACK);

  prv_update_now_playing(data);
  prv_update_layout(data);
  prv_update_track_progress(data);
  prv_update_ui_state(data, false);
}

static void prv_push_window(MusicAppData *data) {
  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Music"));
  window_set_user_data(window, data);
  window_set_status_bar_icon(window, (GBitmap*)&s_status_icon_music_bitmap);

  const bool animated = true;
  app_window_stack_push(window, animated);
  prv_init_ui(window);
}

// Key string must match PREF_KEY_MUSIC_SHOW_ALBUM_ART in shell/normal/prefs.c.
#define MUSIC_SHOW_ALBUM_ART_PREF_KEY "musicShowAlbumArt"

static void prv_pref_change_handler(PebbleEvent *event, void *context) {
  const char *key = event->pref_change.key;
  if (!key || strcmp(key, MUSIC_SHOW_ALBUM_ART_PREF_KEY) != 0) {
    return;
  }
  MusicAppData *data = app_state_get_user_data();
  // Turning it on may need art we skipped fetching earlier; turning it off flips straight back to
  // the stock layout. prv_apply_art_appearance recomputes has_album_art from the new setting.
  prv_maybe_request_album_art();
  prv_apply_art_appearance(data);
}

static void prv_music_event_handler(PebbleEvent *event, void *context) {
  MusicAppData *data = app_state_get_user_data();
  switch (event->media.type) {
    case PebbleMediaEventTypeNowPlayingChanged:
      prv_update_now_playing(data);
      return;
    case PebbleMediaEventTypePlaybackStateChanged: {
      prv_set_pos_update_timer(data, event->media.playback_state);
      prv_update_ui_state(data, true);
      return;
    }
    case PebbleMediaEventTypeAlbumArtUpdated:
      prv_apply_art_appearance(data);
      return;
    case PebbleMediaEventTypeVolumeChanged:
    case PebbleMediaEventTypeServerConnected:
    case PebbleMediaEventTypeServerDisconnected:
    case PebbleMediaEventTypeTrackPosChanged:
      music_get_pos(&data->track_pos, &data->track_length);
      prv_update_track_progress(data);
      prv_update_layout(data);
      return;
    default: return;
  }
}

////////////////////
// App boilerplate

static void prv_handle_init(void) {
  MusicAppData *data = app_malloc_check(sizeof(MusicAppData));
  *data = (MusicAppData){};
  app_state_set_user_data(data);

  data->event_info = (EventServiceInfo){
    .type = PEBBLE_MEDIA_EVENT,
    .handler = prv_music_event_handler,
  };

  data->pref_change_event_info = (EventServiceInfo){
    .type = PEBBLE_PREF_CHANGE_EVENT,
    .handler = prv_pref_change_handler,
  };
  event_service_client_subscribe(&data->pref_change_event_info);

  // TODO: Once we have some sort of system-wide "needs bluetooth" assertion, invoke that here.

  data->current_play_state = MusicPlayStateInvalid;

  gbitmap_init_with_resource(&data->icon_skip_backward, RESOURCE_ID_MUSIC_ICON_SKIP_BACKWARD);
  gbitmap_init_with_resource(&data->icon_skip_forward, RESOURCE_ID_MUSIC_ICON_SKIP_FORWARD);
  gbitmap_init_with_resource(&data->icon_ellipsis, RESOURCE_ID_MUSIC_ICON_ELLIPSIS);
  gbitmap_init_with_resource(&data->icon_play, RESOURCE_ID_MUSIC_ICON_PLAY);
  gbitmap_init_with_resource(&data->icon_pause, RESOURCE_ID_MUSIC_ICON_PAUSE);
  gbitmap_init_with_resource(&data->icon_play_pause, RESOURCE_ID_MUSIC_ICON_PLAY_PAUSE);
  gbitmap_init_with_resource(&data->icon_volume_up, RESOURCE_ID_MUSIC_ICON_VOLUME_UP);
  gbitmap_init_with_resource(&data->icon_volume_down, RESOURCE_ID_MUSIC_ICON_VOLUME_DOWN);
  gbitmap_init_with_resource(&data->image_cassette, RESOURCE_ID_MUSIC_LARGE_CASSETTE);
  gbitmap_init_with_resource(&data->image_pause, RESOURCE_ID_MUSIC_LARGE_PAUSED);
  gbitmap_init_with_resource(&data->image_volume_up, RESOURCE_ID_MUSIC_LARGE_VOLUME_UP);
  gbitmap_init_with_resource(&data->image_volume_down, RESOURCE_ID_MUSIC_LARGE_VOLUME_DOWN);

  event_service_client_subscribe(&data->event_info);
  prv_push_window(data);

  // Overall reduce the latency at the expense of some power...
  music_request_reduced_latency(true);

  // Give us a super responsive initial period:
  music_request_low_latency_for_period(5000);

  prv_set_pos_update_timer(data, music_get_playback_state());
  if (music_is_progress_reporting_supported() && !shell_prefs_get_music_show_progress_bar()) {
    accel_tap_service_subscribe(prv_show_progress_bar_temporarily);
  }
}

static void prv_handle_deinit(void) {
  tick_timer_service_unsubscribe();
  accel_tap_service_unsubscribe();
  music_request_reduced_latency(false);

  MusicAppData *data = app_state_get_user_data();
  event_service_client_unsubscribe(&data->pref_change_event_info);
  prv_title_marquee_stop(data);
  if (data->temporarily_show_progress_timer) {
    app_timer_cancel(data->temporarily_show_progress_timer);
  }
  // Detach the action bar so its touch-nav snapshot does not keep routing taps after the app exits.
  action_bar_layer_remove_from_window(&data->action_bar);
  i18n_free_all(data);
}

static void prv_main(void) {
  prv_handle_init();

  app_event_loop();

  prv_handle_deinit();
}

const PebbleProcessMd* music_app_get_info(void) {
  // [INTL] The app name should come from a standard app resource, so it's localizable.
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &prv_main,
      // UUID: 1f03293d-47af-4f28-b960-f2b02a6dd757
      .uuid = {0x1f, 0x03, 0x29, 0x3d, 0x47, 0xaf, 0x4f, 0x28,
               0xb9, 0x60, 0xf2, 0xb0, 0x2a, 0x6d, 0xd7, 0x57},
    },
    .name = i18n_noop("Music"),
    .icon_resource_id = RESOURCE_ID_AUDIO_CASSETTE_TINY,
  };
  return (const PebbleProcessMd*) &s_app_info;
}

