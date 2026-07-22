/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/music_endpoint.h"
#include "pbl/services/music_endpoint_types.h"

#include "applib/graphics/gtypes.h"
#include "comm/ble/kernel_le_client/ams/ams.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/comm_session/session_remote_os.h"
#include "pbl/services/music_internal.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"

#include <string.h>

PBL_LOG_MODULE_DECLARE(service_music, CONFIG_SERVICE_MUSIC_LOG_LEVEL);

static const uint16_t MUSIC_CTRL_ENDPOINT = 0x20;

static bool s_connected;
static bool s_progress_reporting_supported = true;

static void prv_send_music_command_to_handset(MusicEndpointCmdID cmd) {
  CommSession *session = comm_session_get_system_session();
  if (!session) {
    PBL_LOG_ERR("No system session");
    return;
  }
  comm_session_send_data(session, MUSIC_CTRL_ENDPOINT,
                         (const uint8_t *)&cmd, 1, COMM_SESSION_DEFAULT_TIMEOUT);
}

static const uint8_t* prv_read_ptr_and_length_from_buffer(const uint8_t *iter,
                                                          const uint8_t *iter_end,
                                                          const char** out_str,
                                                          size_t *out_length) {
  if (!out_str || !out_length) {
    return NULL;
  }

  *out_length = *iter;
  *out_str = (const char*) iter + 1;

  iter += 1 + *out_length;
  if (iter > iter_end) {
    PBL_LOG_WRN("Invalid music message");
    return NULL;
  }
  return iter;
}

static void prv_update_now_playing_info(CommSession *session, const uint8_t* msg, size_t length) {
  // Read all the lengths from the message so we know how to break it up.
  const uint8_t* read_iter = msg;
  const char* artist_ptr;
  size_t artist_length;

  read_iter = prv_read_ptr_and_length_from_buffer(read_iter, msg + length,
                                                  &artist_ptr, &artist_length);
  if (!read_iter) {
    return;
  }

  const char* album_ptr;
  size_t album_length;
  read_iter = prv_read_ptr_and_length_from_buffer(read_iter, msg + length,
                                                  &album_ptr, &album_length);
  if (!read_iter) {
    return;
  }

  const char* title_ptr;
  size_t title_length;
  read_iter = prv_read_ptr_and_length_from_buffer(read_iter, msg + length,
                                              &title_ptr, &title_length);
  if (!read_iter) {
    return;
  }

  music_update_now_playing(title_ptr, title_length,
                           artist_ptr, artist_length,
                           album_ptr, album_length);

  if (comm_session_has_capability(session, CommSessionExtendedMusicService)) {
    if (read_iter + sizeof(uint32_t) <= msg + length) {
      uint32_t track_duration_ms = *(uint32_t *)read_iter;
      music_update_track_duration(track_duration_ms);
    }
    // TODO: Do something with this info
    // read_iter += 4;
    // if (read_iter + sizeof(uint16_t) <= msg + length) {
    //   uint16_t num_tracks = *(uint16_t *)read_iter;
    // }

    // TODO: Do something with this info
    // read_iter += 2;
    // if (read_iter + sizeof(uint16_t) <= msg + length) {
    //   uint16_t idx_curr_track = *(uint16_t *)read_iter;
    // }
  }
}

static void prv_update_play_state_info(CommSession *session, const uint8_t* msg, size_t length) {
  if (length < sizeof(MusicEndpointPlayStateInfo)) {
    return;
  }
  MusicEndpointPlayStateInfo *play_state_info = (MusicEndpointPlayStateInfo*) msg;
  MusicPlayerStateUpdate player_state_update;

  switch (play_state_info->play_state) {
    case MusicEndpointPlaybackStatePaused:
      player_state_update.playback_state = MusicPlayStatePaused;
      break;
    case MusicEndpointPlaybackStatePlaying:
      player_state_update.playback_state = MusicPlayStatePlaying;
      break;
    case MusicEndpointPlaybackStateRewinding:
      player_state_update.playback_state = MusicPlayStateRewinding;
      break;
    case MusicEndpointPlaybackStateForwarding:
      player_state_update.playback_state = MusicPlayStateForwarding;
      break;
    case MusicEndpointPlaybackStateUnknown:
      player_state_update.playback_state = MusicPlayStateUnknown;
      break;
    default:
      player_state_update.playback_state = MusicPlayStateInvalid;
  }
  player_state_update.playback_rate_percent = play_state_info->play_rate;
  s_progress_reporting_supported = (play_state_info->track_pos_ms >= 0);
  player_state_update.elapsed_time_ms = MAX(play_state_info->track_pos_ms, 0);
  // TODO: Do something with this info
  // play_state_info->play_shuffle_mode;
  // play_state_info->play_repeat_mode;

  music_update_player_playback_state(&player_state_update);
}

static void prv_update_volume_info(CommSession *session, const uint8_t* msg, size_t length) {
  if (length < sizeof(uint8_t)) {
    return;
  }
  music_update_player_volume_percent((uint8_t)*msg);
}

static void prv_update_player_info(CommSession *session, const uint8_t* msg, size_t length) {
  // Read all the lengths from the message so we know how to break it up.
  const uint8_t* read_iter = msg;

  const char* player_package_ptr;
  size_t player_package_length;
  read_iter = prv_read_ptr_and_length_from_buffer(read_iter, msg + length,
                                                  &player_package_ptr, &player_package_length);
  if (!read_iter) {
    return;
  }

  const char* player_name_ptr;
  size_t player_name_length;
  read_iter = prv_read_ptr_and_length_from_buffer(read_iter, msg + length,
                                                  &player_name_ptr, &player_name_length);
  if (!read_iter) {
    return;
  }
  // Not doing anything with player package name
  music_update_player_name(player_name_ptr, player_name_length);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Album art reassembly
//
// The phone renders album art to the watch's native dimensions as a 4-bpp palettized image and
// sends it in MusicEndpointCmdIDAlbumArtResponse chunks. We reassemble the pixel data into a kernel
// heap buffer and, once complete, wrap it in a GBitmap that ownership of which is handed to the
// music service. This callback always runs on KernelMain, so the static reassembly state needs no
// additional locking.

// Full-screen 4-bpp on the largest supported display (260x260) is ~34 KB. Cap generously and reject
// anything larger to bound kernel heap use against a malformed or hostile phone. (The kernel heap's
// largest contiguous block is ~44 KB, so full-screen 8-bit at 66 KB does not fit; art stays 4-bpp
// palettized with a per-image 16-colour palette.)
#define MUSIC_ALBUM_ART_MAX_BYTES (40 * 1024)
#define MUSIC_ALBUM_ART_MAX_DIM (300)
#define MUSIC_ALBUM_ART_PALETTE_ENTRIES (16)

static struct {
  bool active;
  uint8_t token;
  uint16_t width;
  uint16_t height;
  uint16_t row_size_bytes;
  uint32_t total_bytes;
  uint32_t received_bytes;
  uint8_t *pixels;
  GColor *palette;
} s_art_rx;

static void prv_art_rx_reset(void) {
  kernel_free(s_art_rx.pixels);
  kernel_free(s_art_rx.palette);
  s_art_rx = (__typeof__(s_art_rx)) { 0 };
}

static void prv_handle_album_art_chunk(const uint8_t *msg, size_t length) {
  if (length < sizeof(MusicEndpointAlbumArtChunkHeader)) {
    return;
  }
  const MusicEndpointAlbumArtChunkHeader *hdr = (const MusicEndpointAlbumArtChunkHeader *)msg;
  const uint8_t *cursor = msg + sizeof(*hdr);
  const uint8_t *msg_end = msg + length;

  // Ignore chunks that aren't for the track we currently have (art that raced with a track change).
  // Don't tear down an in-progress transfer for the current track: just drop the stale chunk.
  if (hdr->token != music_get_now_playing_generation()) {
    return;
  }

  if (hdr->flags & MusicAlbumArtFlagNoArt) {
    // Phone has no art for this track: clear any displayed art and fall back to text-only.
    prv_art_rx_reset();
    music_set_album_art(NULL, hdr->token);
    return;
  }

  if (hdr->flags & MusicAlbumArtFlagFirst) {
    prv_art_rx_reset();
    if (cursor + 5 > msg_end) {
      return;
    }
    const uint16_t width = cursor[0] | (cursor[1] << 8);
    const uint16_t height = cursor[2] | (cursor[3] << 8);
    const uint8_t palette_count = cursor[4];
    cursor += 5;
    if (width == 0 || height == 0 || width > MUSIC_ALBUM_ART_MAX_DIM ||
        height > MUSIC_ALBUM_ART_MAX_DIM || palette_count == 0 ||
        palette_count > MUSIC_ALBUM_ART_PALETTE_ENTRIES) {
      return;
    }
    if (cursor + palette_count > msg_end) {
      return;
    }
    const uint16_t row_size = gbitmap_format_get_row_size_bytes(width, GBitmapFormat4BitPalette);
    const uint32_t total = (uint32_t)row_size * height;
    if (total == 0 || total > MUSIC_ALBUM_ART_MAX_BYTES) {
      return;
    }
    s_art_rx.pixels = kernel_zalloc(total);
    s_art_rx.palette = kernel_zalloc(MUSIC_ALBUM_ART_PALETTE_ENTRIES * sizeof(GColor));
    if (!s_art_rx.pixels || !s_art_rx.palette) {
      prv_art_rx_reset();
      return;
    }
    for (uint8_t i = 0; i < palette_count; ++i) {
      s_art_rx.palette[i] = (GColor) { .argb = cursor[i] };
    }
    cursor += palette_count;
    s_art_rx.active = true;
    s_art_rx.token = hdr->token;
    s_art_rx.width = width;
    s_art_rx.height = height;
    s_art_rx.row_size_bytes = row_size;
    s_art_rx.total_bytes = total;
    s_art_rx.received_bytes = 0;
  }

  if (!s_art_rx.active || s_art_rx.token != hdr->token) {
    // Continuation chunk without a valid in-progress transfer.
    prv_art_rx_reset();
    return;
  }

  // The transport is reliable and ordered, so require chunks to be contiguous and in order. This
  // makes the received-byte count exact (no duplicate/overlapping chunks silently leaving holes),
  // and the declared length must match the actual pixel payload.
  const size_t avail = (cursor <= msg_end) ? (size_t)(msg_end - cursor) : 0;
  if (hdr->offset != s_art_rx.received_bytes || hdr->chunk_len != avail ||
      (uint32_t)hdr->offset + hdr->chunk_len > s_art_rx.total_bytes) {
    prv_art_rx_reset();
    return;
  }
  memcpy(s_art_rx.pixels + hdr->offset, cursor, hdr->chunk_len);
  s_art_rx.received_bytes += hdr->chunk_len;

  if (hdr->flags & MusicAlbumArtFlagLast) {
    if (s_art_rx.received_bytes != s_art_rx.total_bytes) {
      // Incomplete: don't publish a partial image.
      prv_art_rx_reset();
      return;
    }
    GBitmap *bmp = kernel_zalloc(sizeof(GBitmap));
    if (!bmp) {
      prv_art_rx_reset();
      return;
    }
    bmp->addr = s_art_rx.pixels;
    bmp->row_size_bytes = s_art_rx.row_size_bytes;
    bmp->info.format = GBitmapFormat4BitPalette;
    bmp->info.version = GBITMAP_VERSION_CURRENT;
    bmp->bounds = (GRect) { { 0, 0 }, { s_art_rx.width, s_art_rx.height } };
    bmp->palette = s_art_rx.palette;
    // Ownership of the pixel and palette buffers moves into the bitmap.
    const uint8_t token = s_art_rx.token;
    s_art_rx.pixels = NULL;
    s_art_rx.palette = NULL;
    prv_art_rx_reset();
    music_set_album_art(bmp, token);
  }
}

void music_protocol_msg_callback(CommSession *session, const uint8_t* msg, size_t length) {
  if (!s_connected) {
    return;
  }
  --length;

  switch (*(msg++)) {
    case MusicEndpointCmdIDNowPlayingInfoResponse:
      prv_update_now_playing_info(session, msg, length);
      break;
    case MusicEndpointCmdIDPlayStateInfoResponse:
      prv_update_play_state_info(session, msg, length);
      break;
    case MusicEndpointCmdIDVolumeInfoResponse:
      prv_update_volume_info(session, msg, length);
      break;
    case MusicEndpointCmdIDPlayerInfoResponse:
      prv_update_player_info(session, msg, length);
      break;
    case MusicEndpointCmdIDAlbumArtResponse:
      prv_handle_album_art_chunk(msg, length);
      break;
    default:
      PBL_LOG_DBG("Invalid command 0x%"PRIx8, msg[0]);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// MusicServerImplementation

static MusicEndpointCmdID prv_pp_command_for_music_command(MusicCommand command) {
  switch (command) {
    case MusicCommandPlay:
      return MusicEndpointCmdIDPlay;
    case MusicCommandPause:
      return MusicEndpointCmdIDPause;
    case MusicCommandTogglePlayPause:
      return MusicEndpointCmdIDTogglePlayPause;
    case MusicCommandNextTrack:
      return MusicEndpointCmdIDNextTrack;
    case MusicCommandPreviousTrack:
      return MusicEndpointCmdIDPreviousTrack;
    case MusicCommandVolumeUp:
      return MusicEndpointCmdIDVolumeUp;
    case MusicCommandVolumeDown:
      return MusicEndpointCmdIDVolumeDown;

    case MusicCommandAdvanceRepeatMode:
    case MusicCommandAdvanceShuffleMode:
    case MusicCommandSkipForward:
    case MusicCommandSkipBackward:
    default:
      return MusicEndpointCmdIDInvalid;
  }
}

static bool prv_music_is_command_supported(MusicCommand command) {
  return (prv_pp_command_for_music_command(command) != MusicEndpointCmdIDInvalid);
}

static void prv_music_command_send(MusicCommand command) {
  const MusicEndpointCmdID pp_command = prv_pp_command_for_music_command(command);
  if (pp_command == MusicEndpointCmdIDInvalid) {
    return;
  }

  prv_send_music_command_to_handset(pp_command);
}

static MusicServerCapability prv_music_get_capability_bitset(void) {
  if (comm_session_has_capability(comm_session_get_system_session(),
                                  CommSessionExtendedMusicService)) {
    if (s_progress_reporting_supported) {
      return (MusicServerCapabilityPlaybackStateReporting |
              MusicServerCapabilityProgressReporting |
              MusicServerCapabilityVolumeReporting);
    } else {
      return (MusicServerCapabilityPlaybackStateReporting | MusicServerCapabilityVolumeReporting);
    }
  } else {
    return MusicServerCapabilityNone;
  }
}

static bool prv_music_needs_user_to_start_playback_on_phone(void) {
  return false;  // On Android, we can initiate playback from Pebble.
}

static void prv_music_request_reduced_latency(bool reduced_latency) {
  const ResponseTimeState state = reduced_latency ? ResponseTimeMiddle : ResponseTimeMax;
  comm_session_set_responsiveness(comm_session_get_system_session(),
                                  BtConsumerMusicServiceIndefinite, state,
                                  MAX_PERIOD_RUN_FOREVER);
}

static void prv_music_request_low_latency_for_period(uint32_t period_ms) {
  comm_session_set_responsiveness(comm_session_get_system_session(),
                                  BtConsumerMusicServiceMomentary,
                                  ResponseTimeMin,
                                  period_ms / MS_PER_SECOND);
}

static bool prv_music_is_album_art_supported(void) {
  CommSession *session = comm_session_get_system_session();
  return session && comm_session_has_capability(session, CommSessionMusicAlbumArtSupport);
}

static void prv_music_request_album_art(uint8_t token) {
  CommSession *session = comm_session_get_system_session();
  if (!session || !comm_session_has_capability(session, CommSessionMusicAlbumArtSupport)) {
    return;
  }
  const uint8_t payload[] = { MusicEndpointCmdIDGetAlbumArt, token };
  comm_session_send_data(session, MUSIC_CTRL_ENDPOINT, payload, sizeof(payload),
                         COMM_SESSION_DEFAULT_TIMEOUT);
}

static const MusicServerImplementation s_pp_music_implementation = {
  .debug_name = "PP",
  .is_command_supported = &prv_music_is_command_supported,
  .command_send = &prv_music_command_send,
  .needs_user_to_start_playback_on_phone = prv_music_needs_user_to_start_playback_on_phone,
  .get_capability_bitset = prv_music_get_capability_bitset,
  .request_reduced_latency = prv_music_request_reduced_latency,
  .request_low_latency_for_period = prv_music_request_low_latency_for_period,
  .is_album_art_supported = prv_music_is_album_art_supported,
  .request_album_art = prv_music_request_album_art,
};

////////////////////////////////////////////////////////////////////////////////////////////////////

static void prv_set_connected(bool connected) {
  if (s_connected == connected) {
    return;  // Expected to happen because this is called with `false` for any OS
  }
  if (music_set_connected_server(&s_pp_music_implementation, connected)) {
    s_connected = connected;
  } else {
    s_connected = false;
  }
  if (s_connected) {
    // Request initial state:
    prv_send_music_command_to_handset(MusicEndpointCmdIDGetAllInfo);
  } else {
    // Drop any in-progress album-art transfer so its buffer isn't leaked across a disconnect. Runs
    // on the same task as the chunk receiver, so touching s_art_rx here is safe.
    prv_art_rx_reset();
  }
}

void music_endpoint_handle_mobile_app_info_event(const PebbleRemoteAppInfoEvent *app_info_event) {
  if (app_info_event->os != RemoteOSAndroid) {
    // Only on Android we use Pebble Protocol for music metadata and control.
    return;
  }
  
  ams_music_disconnect();
  prv_set_connected(true);
}

void music_endpoint_handle_mobile_app_event(const PebbleCommSessionEvent *app_event) {
  if (!app_event->is_open && app_event->is_system) {
    // Pebble mobile app went away, communicate the disconnection to the upper layers:
    prv_set_connected(false);
  }
}
