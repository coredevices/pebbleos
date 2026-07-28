/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/music_endpoint.h"
#include "pbl/services/music_endpoint_types.h"

#include "comm/ble/kernel_le_client/ams/ams.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/comm_session/session_remote_os.h"
#include "pbl/services/music_internal.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"

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

static const MusicServerImplementation s_pp_music_implementation = {
  .debug_name = "PP",
  .is_command_supported = &prv_music_is_command_supported,
  .command_send = &prv_music_command_send,
  .needs_user_to_start_playback_on_phone = prv_music_needs_user_to_start_playback_on_phone,
  .get_capability_bitset = prv_music_get_capability_bitset,
  .request_reduced_latency = prv_music_request_reduced_latency,
  .request_low_latency_for_period = prv_music_request_low_latency_for_period,
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
