/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/music_internal.h"

#include "applib/graphics/gtypes.h"
#include "apps/system/music.h"
#include "pbl/services/imaging.h"
#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/kernel/mutex.h"
#include "pbl/kernel/types.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"

PBL_LOG_MODULE_DEFINE(service_music, CONFIG_SERVICE_MUSIC_LOG_LEVEL);

//! @file
//! This module implements the music service. It provides an abstraction layer on top of the
//! various underlying music metadata and control services: the Pebble Protocol
//! music endpoint (see music_endpoint.c) and Apple Media Service (see ams.c).
//! This module also caches the last known metadata and media player state.
//! @note Only one underlying backend is supported at a time. If a second backend tries to "connect"
//! it is ignored.

#define MUSIC_NORMAL_PLAYBACK_RATE_PERCENT ((int32_t) 100)

//! Cache of the most recently received now playing data. Note that this is read and written from
//! multiple threads, so access is protected by the mutex member.
struct MusicServiceContext {
  struct pbl_mutex mutex;

  //! The connected server that provides media metadata and accepts control commands
  const MusicServerImplementation *implementation;

  //! The volume setting of the current player
  uint8_t player_volume_percent;

  char player_name[MUSIC_BUFFER_LENGTH];

  char title[MUSIC_BUFFER_LENGTH];
  char artist[MUSIC_BUFFER_LENGTH];
  char album[MUSIC_BUFFER_LENGTH];

  uint32_t track_length_ms;

  //! Position that was last communicated to Pebble by the server.
  //! @note This is not necessarily the actual position. See music_get_pos()
  uint32_t track_pos_ms;

  //! The time when track_pos_ms was last updated.
  RtcTicks track_pos_updated_at;

  //! The current playback rate in percent units.
  //! Example values:
  //!  100 = normal playback rate
  //!  0   = paused
  //!  200 = 2x playback rate (Apple's Podcast app can vary the playback rate)
  //! -100 = backwards at normal rate
  int32_t playback_rate_percent;

  //! The current playback state
  MusicPlayState playback_state;

  //! @see music_skip_seeks_within_track
  bool skip_seeks_within_track;

  //! Album art for the current track, or NULL if none. Owned by the service; freed with
  //! kernel_free. Deliberately kept (up to ~34 KB of kernel heap on the largest displays) even
  //! while the Music app is closed, so reopening it shows the cover instantly; a track change,
  //! NoArt reply or server disconnect replaces or frees it.
  GBitmap *album_art;

  //! The now_playing_generation the current album art response was for. A track change bumps
  //! now_playing_generation without clearing the art (so the previous art stays on screen until the
  //! new one arrives); comparing the two tells the Music app whether it still needs to request art
  //! for the current track. Set on every response (art or NoArt), so a NoArt reply also stops the
  //! re-request. @see music_album_art_is_current
  uint8_t album_art_generation;

  //! Generation token, bumped whenever the track (title/artist) changes. Used to discard album art
  //! that arrives after a track change. @see music_get_now_playing_generation
  uint8_t now_playing_generation;
} s_music_ctx;

// Album art arrives via the generic image-fetch service; store it against the requesting token.
static void prv_imaging_album_art_received(uint8_t token, GBitmap *bitmap) {
  music_set_album_art(bitmap, token);
}

void music_init(void) {
  pbl_mutex_init(&s_music_ctx.mutex);
  imaging_register_handler(ImagingImageTypeAlbumArt, prv_imaging_album_art_received);
}

//! Length to keep from (src, src_length) so the copy fits the buffer without splitting a UTF-8
//! sequence. Backing off mid-sequence keeps the stored string valid UTF-8, which lets the phone
//! prefix-match the title it gets back in an album-art request exactly (see the imaging service).
static size_t prv_utf8_crop_length(const char *src, size_t src_length) {
  size_t cropped = MIN(MUSIC_BUFFER_LENGTH - 1, src ? src_length : 0);
  if (src && cropped < src_length) {
    // We cut: if it landed inside a multi-byte sequence, drop the partial trailing bytes.
    while (cropped > 0 && ((uint8_t)src[cropped] & 0xC0) == 0x80) {
      cropped--;
    }
  }
  return cropped;
}

static void copy_and_truncate(char *dest, const char *src, size_t src_length) {
  size_t cropped_length = prv_utf8_crop_length(src, src_length);
  if (src) {
    memcpy(dest, src, cropped_length);
  }
  dest[cropped_length] = 0;
}

//! @return true if the truncated form of (src, src_length) differs from the null-terminated dest.
static bool prv_str_differs(const char *dest, const char *src, size_t src_length) {
  size_t cropped_length = prv_utf8_crop_length(src, src_length);
  return (strlen(dest) != cropped_length) || (memcmp(dest, src, cropped_length) != 0);
}

//! Free the currently-stored album art. Caller must hold the mutex.
static void prv_free_album_art_locked(void) {
  if (s_music_ctx.album_art) {
    kernel_free(s_music_ctx.album_art->addr);
    kernel_free(s_music_ctx.album_art->palette);
    kernel_free(s_music_ctx.album_art);
    s_music_ctx.album_art = NULL;
  }
}

static void prv_put_now_playing_changed_event(void) {
  PebbleEvent e = {
    .type = PEBBLE_MEDIA_EVENT,
    .media.type = PebbleMediaEventTypeNowPlayingChanged
  };
  event_put(&e);
}

bool music_set_connected_server(const MusicServerImplementation *implementation, bool connected) {
  enum {
    Disconnected = -1,
    None = 0,
    Connected = 1,
  } change_type = None;

  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  if (connected) {
    if (s_music_ctx.implementation == NULL) {
      change_type = Connected;
      s_music_ctx.implementation = implementation;
      PBL_LOG_INFO("Music server connected: %s", implementation->debug_name);
    } else {
      PBL_LOG_ERR("Server <0x%p> connected, but another <0x%p> is already registered",
              implementation, s_music_ctx.implementation);
    }

  } else {
    if (s_music_ctx.implementation == implementation) {
      // Previously registered server got disconnected
      change_type = Disconnected;
      s_music_ctx.implementation = NULL;
      PBL_LOG_INFO("Music server disconnected: %s", implementation->debug_name);
    } else {
      PBL_LOG_ERR("Unknown server <%p> disconnected", implementation);
    }
  }

  if (change_type != None) {
    // Upon connect and disconnect, reset the cached data:

    music_update_player_volume_percent(0);
    // Taking short-cut here, music_update_now_playing already puts NowPlayingChanged event, no
    // need to put it again by calling music_update_player_name:
    s_music_ctx.player_name[0] = 0;
    // now_playing no longer drops art on a track change (see music_update_now_playing), so clear it
    // explicitly here: a connect/disconnect must not leave the previous session's art on screen.
    prv_free_album_art_locked();
    music_update_now_playing(NULL, 0, NULL, 0, NULL, 0);
    music_update_track_duration(0);
    const MusicPlayerStateUpdate state = {
      .playback_state = MusicPlayStateUnknown,
      .playback_rate_percent = 0,
      .elapsed_time_ms = 0,
    };
    music_update_player_playback_state(&state);

    PebbleEvent event = {
      .type = PEBBLE_MEDIA_EVENT,
      .media = {
        .type = (change_type == Connected) ? PebbleMediaEventTypeServerConnected :
                                             PebbleMediaEventTypeServerDisconnected,
      },
    };
    event_put(&event);
  }

  pbl_mutex_unlock(&s_music_ctx.mutex);

  return (change_type != None);
}

const char * music_get_connected_server_debug_name(void) {
  const char *debug_name = NULL;
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  if (s_music_ctx.implementation) {
    debug_name = s_music_ctx.implementation->debug_name;
  }
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return debug_name;
}

void music_update_now_playing(const char *title, size_t title_length,
                              const char *artist, size_t artist_length,
                              const char *album, size_t album_length) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  // A change to the title, artist or album means we're on a different track, so any album art we're
  // holding (or receiving) is now stale. Bump the generation so in-flight chunks for the old track
  // are rejected and the Music app re-requests art. (Album is included so same-title/same-artist
  // tracks from different albums, or podcast episodes, still refresh their art.)
  //
  // We deliberately keep the previous art on screen until the new track's art (or a NoArt reply)
  // arrives, rather than dropping it here. Phones (e.g. Android) often send several now-playing
  // updates per skip as metadata fields populate one-by-one, which would otherwise blank the art to
  // black and flash it back once per update. music_set_album_art()/NoArt handle the swap.
  const bool track_changed = prv_str_differs(s_music_ctx.title, title, title_length) ||
                             prv_str_differs(s_music_ctx.artist, artist, artist_length) ||
                             prv_str_differs(s_music_ctx.album, album, album_length);
  if (track_changed) {
    s_music_ctx.now_playing_generation++;
  }

  copy_and_truncate(s_music_ctx.title, title, title_length);
  copy_and_truncate(s_music_ctx.artist, artist, artist_length);
  copy_and_truncate(s_music_ctx.album, album, album_length);

  pbl_mutex_unlock(&s_music_ctx.mutex);

  prv_put_now_playing_changed_event();
}

static void prv_update_string_and_put_event(const char *value, size_t value_length, off_t offset) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  char *buffer = ((char *) &s_music_ctx) + offset;
  copy_and_truncate(buffer, value, value_length);
  pbl_mutex_unlock(&s_music_ctx.mutex);
  prv_put_now_playing_changed_event();
}

void music_update_player_name(const char *player_name, size_t player_name_length) {
  // TODO: actually do something with this
  off_t o = offsetof(__typeof__(s_music_ctx), player_name);
  prv_update_string_and_put_event(player_name, player_name_length, o);
}

void music_update_track_title(const char *title, size_t title_length) {
  off_t o = offsetof(__typeof__(s_music_ctx), title);
  prv_update_string_and_put_event(title, title_length, o);
}

void music_update_track_artist(const char *artist, size_t artist_length) {
  off_t o = offsetof(__typeof__(s_music_ctx), artist);
  prv_update_string_and_put_event(artist, artist_length, o);
}

void music_update_track_album(const char *album, size_t album_length) {
  off_t o = offsetof(__typeof__(s_music_ctx), album);
  prv_update_string_and_put_event(album, album_length, o);
}

static void prv_put_pos_changed_event(void) {
  PebbleEvent e = {
    .type = PEBBLE_MEDIA_EVENT,
    .media.type = PebbleMediaEventTypeTrackPosChanged,
  };
  event_put(&e);
}

void music_update_track_position(uint32_t track_pos_ms) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  s_music_ctx.track_pos_ms = track_pos_ms;
  s_music_ctx.track_pos_updated_at = rtc_get_ticks();

  pbl_mutex_unlock(&s_music_ctx.mutex);

  prv_put_pos_changed_event();
}

void music_update_track_duration(uint32_t track_duration_ms) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  s_music_ctx.track_length_ms = track_duration_ms;

  pbl_mutex_unlock(&s_music_ctx.mutex);

  prv_put_pos_changed_event();
}

void music_get_now_playing(char *title, char *artist, char *album) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  if (title) {
    strcpy(title, s_music_ctx.title);
  }
  if (artist) {
    strcpy(artist, s_music_ctx.artist);
  }
  if (album) {
    strcpy(album, s_music_ctx.album);
  }

  pbl_mutex_unlock(&s_music_ctx.mutex);
}

bool music_get_player_name(char *player_name_out) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  const bool has_player_name = (s_music_ctx.player_name[0] != 0);

  if (player_name_out) {
    strcpy(player_name_out, s_music_ctx.player_name);
  }

  pbl_mutex_unlock(&s_music_ctx.mutex);

  return has_player_name;
}

bool music_has_now_playing(void) {
  bool has_now_playing = false;
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  if (s_music_ctx.title[0] != 0 ||
      s_music_ctx.artist[0] != 0) {
    has_now_playing = true;
  }
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return has_now_playing;
}

uint32_t music_get_ms_since_pos_last_updated(void) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  const RtcTicks time_elapsed_ticks = rtc_get_ticks() - s_music_ctx.track_pos_updated_at;
  const uint32_t time_elapsed_ms = pbl_ticks_to_ms(time_elapsed_ticks);
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return time_elapsed_ms;
}

void music_get_pos(uint32_t *track_pos_ms, uint32_t *track_length_ms) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  const int32_t time_elapsed_ms = music_get_ms_since_pos_last_updated();
  const int32_t track_time_elapsed =
      (time_elapsed_ms * s_music_ctx.playback_rate_percent) / MUSIC_NORMAL_PLAYBACK_RATE_PERCENT;
  const int32_t pos_ms = s_music_ctx.track_pos_ms + track_time_elapsed;
  const int32_t length_ms = s_music_ctx.track_length_ms;

  *track_pos_ms = CLIP(pos_ms, 0, length_ms);
  *track_length_ms = length_ms;

  pbl_mutex_unlock(&s_music_ctx.mutex);
}

int32_t music_get_playback_rate_percent(void) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  int32_t playback_rate_percent = s_music_ctx.playback_rate_percent;
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return playback_rate_percent;
}

uint8_t music_get_volume_percent(void) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  int32_t player_volume_percent = s_music_ctx.player_volume_percent;
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return player_volume_percent;
}

static void prv_put_state_changed_event(MusicPlayState playback_state) {
  PebbleEvent event = {
    .type = PEBBLE_MEDIA_EVENT,
    .media = {
      .type = PebbleMediaEventTypePlaybackStateChanged,
      .playback_state = playback_state,
    },
  };
  event_put(&event);
}

void music_update_player_playback_state(const MusicPlayerStateUpdate *state) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  s_music_ctx.playback_state = state->playback_state;
  s_music_ctx.playback_rate_percent = state->playback_rate_percent;
  s_music_ctx.track_pos_ms = state->elapsed_time_ms;
  s_music_ctx.track_pos_updated_at = rtc_get_ticks();
  s_music_ctx.skip_seeks_within_track = state->skip_seeks_within_track;
  pbl_mutex_unlock(&s_music_ctx.mutex);

  prv_put_state_changed_event(state->playback_state);
  prv_put_pos_changed_event();
}

void music_update_player_volume_percent(uint8_t volume_percent) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  s_music_ctx.player_volume_percent = volume_percent;
  pbl_mutex_unlock(&s_music_ctx.mutex);

  PebbleEvent event = {
    .type = PEBBLE_MEDIA_EVENT,
    .media = {
      .type = PebbleMediaEventTypeVolumeChanged,
      .volume_percent = volume_percent,
    },
  };
  event_put(&event);
}

MusicPlayState music_get_playback_state(void) {
  if (!music_is_playback_state_reporting_supported()) {
    return MusicPlayStateUnknown;
  }
  MusicPlayState result;
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  result = s_music_ctx.playback_state;
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return result;
}

static void * prv_implementation_function_for_offset(off_t offset) {
  typedef void (*FuncPtr)(void);
  FuncPtr func_ptr = NULL;
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  if (s_music_ctx.implementation) {
    func_ptr = *(FuncPtr *) (((const uint8_t *)s_music_ctx.implementation) + offset);
  }
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return func_ptr;
}

void music_command_send(MusicCommand command) {
  const off_t o = offsetof(__typeof__(*s_music_ctx.implementation), command_send);
  void (*command_send)(MusicCommand) = prv_implementation_function_for_offset(o);
  if (command_send) {
    command_send(command);
  }
}

void music_request_reduced_latency(bool reduced_latency) {
  const off_t o = offsetof(__typeof__(*s_music_ctx.implementation), request_reduced_latency);
  void (*request_reduced_latency)(bool) = prv_implementation_function_for_offset(o);
  if (request_reduced_latency) {
    request_reduced_latency(reduced_latency);
  }
}

void music_request_low_latency_for_period(uint32_t period_ms) {
  const off_t o = offsetof(__typeof__(*s_music_ctx.implementation), request_low_latency_for_period);
  void (*request_low_latency_for_period)(uint32_t) = prv_implementation_function_for_offset(o);
  if (request_low_latency_for_period) {
    request_low_latency_for_period(period_ms);
  }
}

bool music_skip_seeks_within_track(void) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  const bool seeks = s_music_ctx.skip_seeks_within_track;
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return seeks;
}

bool music_is_command_supported(MusicCommand command) {
  const off_t o = offsetof(__typeof__(*s_music_ctx.implementation),
                           is_command_supported);
  bool (*func_ptr)(MusicCommand) = prv_implementation_function_for_offset(o);
  if (!func_ptr) {
    return false;
  }
  return func_ptr(command);
}

static bool prv_call_implementation_bool_return_void_args(off_t offset) {
  bool (*func_ptr)(void) = prv_implementation_function_for_offset(offset);
  if (!func_ptr) {
    return false;  // defaults to false when there is no "connected" implementation
  }
  return func_ptr();
}

bool music_needs_user_to_start_playback_on_phone(void) {
  const off_t o = offsetof(__typeof__(*s_music_ctx.implementation),
                           needs_user_to_start_playback_on_phone);
  return prv_call_implementation_bool_return_void_args(o);
}

static bool prv_is_capability_supported(MusicServerCapability capability) {
  const off_t o = offsetof(__typeof__(*s_music_ctx.implementation), get_capability_bitset);
  MusicServerCapability (*func_ptr)(void) = prv_implementation_function_for_offset(o);
  if (!func_ptr) {
    return false;
  }
  return (func_ptr() & capability);
}

bool music_is_playback_state_reporting_supported(void) {
  return prv_is_capability_supported(MusicServerCapabilityPlaybackStateReporting);
}

bool music_is_progress_reporting_supported(void) {
  // Check capability and that track length is greater than 0
  uint32_t track_length_ms;
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  track_length_ms = s_music_ctx.track_length_ms;
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return (prv_is_capability_supported(MusicServerCapabilityProgressReporting) && track_length_ms);
}

bool music_is_volume_reporting_supported(void) {
  return prv_is_capability_supported(MusicServerCapabilityVolumeReporting);
}

uint8_t music_get_now_playing_generation(void) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  uint8_t generation = s_music_ctx.now_playing_generation;
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return generation;
}

static void prv_put_album_art_updated_event(void) {
  PebbleEvent e = {
    .type = PEBBLE_MEDIA_EVENT,
    .media.type = PebbleMediaEventTypeAlbumArtUpdated,
  };
  event_put(&e);
}

void music_set_album_art(GBitmap *bitmap, uint8_t token) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);

  if (token != s_music_ctx.now_playing_generation) {
    // The track changed while the art was in flight; it's for the wrong song. Drop it.
    if (bitmap) {
      kernel_free(bitmap->addr);
      kernel_free(bitmap->palette);
      kernel_free(bitmap);
    }
    pbl_mutex_unlock(&s_music_ctx.mutex);
    return;
  }

  prv_free_album_art_locked();
  s_music_ctx.album_art = bitmap;
  // Record that we've now got a response (art, or NULL for NoArt) for this track, so the Music app
  // stops re-requesting until the track changes again.
  s_music_ctx.album_art_generation = token;

  pbl_mutex_unlock(&s_music_ctx.mutex);

  prv_put_album_art_updated_event();
}

bool music_album_art_is_current(void) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  const bool is_current = (s_music_ctx.album_art_generation == s_music_ctx.now_playing_generation);
  pbl_mutex_unlock(&s_music_ctx.mutex);
  return is_current;
}

const GBitmap *music_album_art_lock(void) {
  pbl_mutex_lock(&s_music_ctx.mutex, PBL_FOREVER);
  // Held until music_album_art_unlock so the bitmap can't be freed mid-draw. The recursive mutex is
  // released by the matching unlock call.
  return s_music_ctx.album_art;
}

void music_album_art_unlock(void) {
  pbl_mutex_unlock(&s_music_ctx.mutex);
}

void command_print_now_playing(void) {
  char title[MUSIC_BUFFER_LENGTH];
  char artist[MUSIC_BUFFER_LENGTH];
  char album[MUSIC_BUFFER_LENGTH];

  music_get_now_playing(title, artist, album);

  char buffer[128];
  dbgserial_putstr_fmt(buffer, 128, "title=%s; artist=%s; album=%s", title, artist, album);
}

