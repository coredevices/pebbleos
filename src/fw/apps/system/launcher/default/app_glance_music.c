/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_music.h"

#include "app_glance_structured.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/music.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/string.h"
#include "pbl/util/struct.h"

#include <stdio.h>

// We need enough space for the track artist and title (so 2 * MUSIC_BUFFER_LENGTH from music.h),
// the delimiter string " - " (3), and 1 for the null terminator
#define TRACK_TEXT_BUFFER_SIZE ((MUSIC_BUFFER_LENGTH * 2) + 3 + 1)

typedef struct LauncherAppGlanceMusic {
  char title[APP_NAME_SIZE_BYTES];
  char subtitle[TRACK_TEXT_BUFFER_SIZE];
  KinoReel *icon;
  uint32_t icon_resource_id;
  uint32_t default_icon_resource_id;
  EventServiceInfo music_event_info;
} LauncherAppGlanceMusic;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceMusic *music_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(music_glance, icon, NULL);
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceMusic *music_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(music_glance, title, NULL);
}

static void prv_music_glance_subtitle_dynamic_text_node_update(
    PBL_UNUSED GContext *ctx, PBL_UNUSED GTextNode *node, PBL_UNUSED const GRect *box,
    PBL_UNUSED const GTextNodeDrawConfig *config, PBL_UNUSED bool render, char *buffer, size_t buffer_size,
    void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceMusic *music_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (music_glance) {
    strncpy(buffer, music_glance->subtitle, buffer_size);
    buffer[buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  return launcher_app_glance_structured_create_subtitle_text_node(
      structured_glance, prv_music_glance_subtitle_dynamic_text_node_update);
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceMusic *music_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (music_glance) {
    event_service_client_unsubscribe(&music_glance->music_event_info);
    kino_reel_destroy(music_glance->icon);
  }
  app_free(music_glance);
}

static void prv_set_glance_icon(LauncherAppGlanceMusic *music_glance,
                                uint32_t new_icon_resource_id) {
  if (music_glance->icon_resource_id == new_icon_resource_id) {
    // Nothing to do, bail out
    return;
  }

  // Destroy the existing icon
  kino_reel_destroy(music_glance->icon);

  // Set the new icon and record its resource ID
  // TODO PBL-38539: Switch from using a regular resource ID to using a TimelineResourceId
  music_glance->icon = kino_reel_create_with_resource(new_icon_resource_id);
  PBL_ASSERTN(music_glance->icon);
  music_glance->icon_resource_id = new_icon_resource_id;
}

static bool prv_should_display_music_state(MusicPlayState play_state,
                                           uint32_t last_updated_time_elapsed_ms) {
  const uint32_t music_last_updated_display_threshold_ms = 30 * MS_PER_SECOND * SECONDS_PER_MINUTE;

  switch (play_state) {
    case MusicPlayStatePlaying:
    case MusicPlayStateForwarding:
    case MusicPlayStateRewinding:
      return true;
    case MusicPlayStatePaused:
      // We won't display the music state if the music is paused and it hasn't changed in a while
      return (last_updated_time_elapsed_ms < music_last_updated_display_threshold_ms);
    case MusicPlayStateUnknown:
    case MusicPlayStateInvalid:
      return false;
  }
  WTF;
}

static void prv_update_glance_for_music_state(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceMusic *music_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  PBL_ASSERTN(music_glance);

  // Zero out the glance's subtitle buffer
  const size_t music_glance_subtitle_size = sizeof(music_glance->subtitle);
  memset(music_glance->subtitle, 0, music_glance_subtitle_size);

  // Default to showing the default icon
  uint32_t new_icon_resource_id = music_glance->default_icon_resource_id;

  // Determine if we should display the current music state
  const MusicPlayState play_state = music_get_playback_state();
  const uint32_t last_updated_time_elapsed_ms = music_get_ms_since_pos_last_updated();
  const bool should_display_music_state =
      prv_should_display_music_state(play_state, last_updated_time_elapsed_ms);

  if (should_display_music_state) {
    // Get the artist and title strings for the music playing or paused
    char artist_buffer[MUSIC_BUFFER_LENGTH] = {};
    char title_buffer[MUSIC_BUFFER_LENGTH] = {};
    music_get_now_playing(title_buffer, artist_buffer, NULL /* album_buffer */);

    // Only populate the glance with music info if we have both an artist string and a title string
    if (!IS_EMPTY_STRING(artist_buffer) && !IS_EMPTY_STRING(title_buffer)) {
      // Use the strings to fill the subtitle buffer
      snprintf(music_glance->subtitle, music_glance_subtitle_size, "%s - %s", artist_buffer,
               title_buffer);

      // Choose the icon we should display; note that we'll use the default icon we set above if we
      // don't have an icon for the current play state
      if (play_state == MusicPlayStatePlaying) {
        new_icon_resource_id = RESOURCE_ID_MUSIC_APP_GLANCE_PLAY;
      } else if (play_state == MusicPlayStatePaused) {
        new_icon_resource_id = RESOURCE_ID_MUSIC_APP_GLANCE_PAUSE;
      }
    }
  }

  // Update the glance icon
  prv_set_glance_icon(music_glance, new_icon_resource_id);

  // Broadcast to the service that we changed the glance
  launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
}

static void prv_music_event_handler(PebbleEvent *event, void *context) {
  switch (event->media.type) {
    case PebbleMediaEventTypeNowPlayingChanged:
    case PebbleMediaEventTypePlaybackStateChanged:
    case PebbleMediaEventTypeServerConnected:
    case PebbleMediaEventTypeServerDisconnected:
      prv_update_glance_for_music_state(context);
      return;
    case PebbleMediaEventTypeVolumeChanged:
    case PebbleMediaEventTypeTrackPosChanged:
    case PebbleMediaEventTypeAlbumArtUpdated:
      return;
  }
  WTF;
}

static const LauncherAppGlanceStructuredImpl s_music_structured_glance_impl = {
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_music_create(const AppMenuNode *node) {
  PBL_ASSERTN(node);

  LauncherAppGlanceMusic *music_glance = app_zalloc_check(sizeof(*music_glance));

  // Copy the name of the Music app as the title
  const size_t title_size = sizeof(music_glance->title);
  strncpy(music_glance->title, node->name, title_size);
  music_glance->title[title_size - 1] = '\0';

  // Save the default icon resource ID for the Music app
  music_glance->default_icon_resource_id = node->icon_resource_id;

  const bool should_consider_slices = false;
  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_music_structured_glance_impl,
                                            should_consider_slices, music_glance);
  PBL_ASSERTN(structured_glance);

  // Get the first state of the glance
  prv_update_glance_for_music_state(structured_glance);

  // Subscribe to music events for updating the glance
  music_glance->music_event_info = (EventServiceInfo) {
    .type = PEBBLE_MEDIA_EVENT,
    .handler = prv_music_event_handler,
    .context = structured_glance,
  };
  event_service_client_subscribe(&music_glance->music_event_info);

  return &structured_glance->glance;
}
