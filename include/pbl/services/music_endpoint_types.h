/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include "pbl/util/attributes.h"

typedef enum {
  // Watch -> Phone
  MusicEndpointCmdIDTogglePlayPause = 0x1,
  MusicEndpointCmdIDPause = 0x2,
  MusicEndpointCmdIDPlay = 0x3,
  MusicEndpointCmdIDNextTrack = 0x4,
  MusicEndpointCmdIDPreviousTrack = 0x5,
  MusicEndpointCmdIDVolumeUp = 0x6,
  MusicEndpointCmdIDVolumeDown = 0x7,
  MusicEndpointCmdIDGetAllInfo = 0x8,
  MusicEndpointCmdIDGetAlbumArt = 0x9,

  // Phone -> Watch
  MusicEndpointCmdIDNowPlayingInfoResponse = 0x10,
  MusicEndpointCmdIDPlayStateInfoResponse = 0x11,
  MusicEndpointCmdIDVolumeInfoResponse = 0x12,
  MusicEndpointCmdIDPlayerInfoResponse = 0x13,
  MusicEndpointCmdIDAlbumArtResponse = 0x14,

  MusicEndpointCmdIDInvalid = 0xff,
} MusicEndpointCmdID;

//! Flags byte in a MusicEndpointCmdIDAlbumArtResponse chunk.
typedef enum {
  //! This chunk is the first of the transfer; the album-art header (dimensions
  //! and palette) precedes the pixel data.
  MusicAlbumArtFlagFirst = (1 << 0),
  //! This chunk is the last of the transfer.
  MusicAlbumArtFlagLast = (1 << 1),
  //! The phone has no album art for the current track. No pixel data follows;
  //! the watch should fall back to the text-only now-playing screen.
  MusicAlbumArtFlagNoArt = (1 << 2),
} MusicAlbumArtFlags;

//! Header carried by the first chunk of a MusicEndpointCmdIDAlbumArtResponse.
//! The phone renders the art to the watch's native dimensions (known from the
//! watch type) as a GBitmapFormat4BitPalette image: `palette_count` GColor8
//! entries followed by 4-bpp pixel indices, rows tightly packed to
//! ceil(width / 2) bytes.
typedef struct PACKED {
  uint8_t token;      //!< Echo of the requesting GetAlbumArt token.
  uint8_t flags;      //!< MusicAlbumArtFlags bitset.
  uint32_t offset;    //!< Byte offset of this chunk's pixels into the index stream.
  uint16_t chunk_len; //!< Number of pixel-index bytes in this chunk.
  // First chunk only, before the pixel data:
  //   uint16_t width;
  //   uint16_t height;
  //   uint8_t  palette_count;   // 1..16
  //   uint8_t  palette[palette_count];  // GColor8 entries
} MusicEndpointAlbumArtChunkHeader;

typedef enum {
  MusicEndpointPlaybackStatePaused = 0,
  MusicEndpointPlaybackStatePlaying = 1,
  MusicEndpointPlaybackStateRewinding = 2,
  MusicEndpointPlaybackStateForwarding = 3,
  MusicEndpointPlaybackStateUnknown = 4,
} MusicEndpointPlaybackState;

typedef enum {
  MusicEndpointShuffleModeUnknown = 0,
  MusicEndpointShuffleModeOff = 1,
  MusicEndpointShuffleModeOn = 2,
} MusicEndpointShuffleMode;

typedef enum {
  MusicEndpointRepeatModeUnknown = 0,
  MusicEndpointRepeatModeOff = 1,
  MusicEndpointRepeatModeOne = 2,
  MusicEndpointRepeatModeAll = 3,
} MusicEndpointRepeatMode;

typedef struct PACKED {
  uint8_t play_state;
  int32_t track_pos_ms;
  int32_t play_rate;
  uint8_t play_shuffle_mode;
  uint8_t play_repeat_mode;
} MusicEndpointPlayStateInfo;
