/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MUSIC_BUFFER_LENGTH 64

typedef enum {
  MusicPlayStateUnknown,
  MusicPlayStatePlaying,
  MusicPlayStatePaused,
  MusicPlayStateForwarding,
  MusicPlayStateRewinding,
  MusicPlayStateInvalid = 0xFF,
} MusicPlayState;

typedef enum {
  MusicCommandPlay,
  MusicCommandPause,
  MusicCommandTogglePlayPause,
  MusicCommandNextTrack,
  MusicCommandPreviousTrack,
  MusicCommandVolumeUp,
  MusicCommandVolumeDown,
  MusicCommandAdvanceRepeatMode,
  MusicCommandAdvanceShuffleMode,
  MusicCommandSkipForward,
  MusicCommandSkipBackward,
  MusicCommandLike,
  MusicCommandDislike,
  MusicCommandBookmark,

  NumMusicCommand,
} MusicCommand;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface to Music app

//! Copy out the current now playing fields into the parameters. We'll assume you've provided
//! buffers that are at least MUSIC_BUFFER_LENGTH in size.
void music_get_now_playing(char *title, char *artist, char *album);

//! @return True if the music service has Now Playing metadata.
bool music_has_now_playing(void);

//! Copy out the name of the current player. We'll assume you've provided
//! buffers that are at least MUSIC_BUFFER_LENGTH in size.
//! @return True if the name was copied successfully, or false if there was no name available.
bool music_get_player_name(char *player_name_out);

//! @return The milliseconds since the track position was last updated.
uint32_t music_get_ms_since_pos_last_updated(void);

//! Retrieve the position in the current track in the given pointers (which must not be null).
void music_get_pos(uint32_t *track_pos_ms, uint32_t *track_length_ms);

//! @return The current playback rate percentage.
int32_t music_get_playback_rate_percent(void);

//! @return The volume percentage.
uint8_t music_get_volume_percent(void);

//! Retrieve the current playback state.
MusicPlayState music_get_playback_state(void);

//! @return True if the service supports reporting of the player's playback state.
//! @see music_get_playback_state
bool music_is_playback_state_reporting_supported(void);

//! @return True if the service support reporting of the playback progress.
//! @see music_get_pos
bool music_is_progress_reporting_supported(void);

//! @return True if the service supports reporting of the current volume.
//! @see music_get_volume_percent
bool music_is_volume_reporting_supported(void);

//! Sends the command to the server. Commands are "unreliable", they are sent at "best effort".
//! @param command The command to send.
//! @see music_is_command_supported
void music_command_send(MusicCommand command);

//! @param command The command to test.
//! @return True if the command is supported by the connected server.
bool music_is_command_supported(MusicCommand command);

//! @return True if MusicCommandNextTrack / MusicCommandPreviousTrack will seek within the current
//! track (podcasts, audiobooks) rather than change track. The commands to send are the same either
//! way; this only says what they will do, so the Music app can show matching icons.
bool music_skip_seeks_within_track(void);

//! @return True if playback needs to be started manually by the user from the phone.
bool music_needs_user_to_start_playback_on_phone(void);

//! Puts the underlying connection in a reduced latency mode, for better responsiveness.
void music_request_reduced_latency(bool reduced_latency);

//! Puts the underlying connection in a low latency mode, for the best responsiveness.
void music_request_low_latency_for_period(uint32_t period_seconds);

//! For testing purposes.
const char * music_get_connected_server_debug_name(void);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Album art

struct GBitmap;


//! @return An 8-bit generation token that changes whenever the current track changes (title, artist
//! or album). The Music app re-requests art when it changes; servers echo it in album-art transfers
//! so the service can discard art that arrived after a track change. @see music_set_album_art
uint8_t music_get_now_playing_generation(void);

//! @return True if the album art currently held matches the current track (i.e. we've received a
//! response — art or "no art" — for this generation). The previous track's art stays on screen
//! until the new track's art arrives, so callers must use this rather than "is any art present" to
//! decide whether to (re-)request art for the current track.
bool music_album_art_is_current(void);

//! Borrow the current album art for drawing. Returns NULL if there is no art for the current track.
//! The returned bitmap is owned by the music service and remains valid until music_album_art_unlock
//! is called; the caller MUST call music_album_art_unlock when done, and MUST NOT retain the pointer
//! past that point.
const struct GBitmap *music_album_art_lock(void);

//! Release the album art borrowed with music_album_art_lock.
void music_album_art_unlock(void);
