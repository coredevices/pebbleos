/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/audio_endpoint.h"
#include "pbl/services/voice_endpoint.h"
#include "pbl/util/attributes.h"
#include "util/generic_attribute.h"

// Shared message definitions with unit test

typedef enum {
  MsgIdSessionSetup = 0x01,
  MsgIdDictationResult = 0x02,
  MsgIdNLPResult = 0x03,
  MsgIdRecordingListRequest = 0x10,
  MsgIdRecordingListResponse = 0x11,
  MsgIdRecordingDeleteRequest = 0x12,
  MsgIdRecordingDeleteResponse = 0x13,
  MsgIdRecordingPlaybackRequest = 0x14,
  MsgIdRecordingPlaybackResponse = 0x15,
  MsgIdRecordingTranscribeRequest = 0x16,
  MsgIdRecordingTranscribeResponse = 0x17,
} MsgId;

// Attribute ID definitions
typedef enum {
  VEAttributeIdInvalid = 0x00,
  VEAttributeIdAudioTransferInfoSpeex = 0x01,
  VEAttributeIdTranscription = 0x02,
  VEAttributeIdAppUuid = 0x03,
  VEAttributeIdReminder = 0x04,
  VEAttributeIdTimestamp = 0x05,
} VEAttributeId;

// Sent and received by watch. Result is only sent by phone.

typedef union PACKED {
  struct {
    uint32_t app_initiated:1;
  };
  uint32_t all;
} VEFlags;

typedef struct PACKED {
  MsgId msg_id:8;
  VEFlags flags;
  VoiceEndpointSessionType session_type:8;
  AudioEndpointSessionId session_id;
  GenericAttributeList attr_list;
} SessionSetupMsg;

typedef struct PACKED {
  MsgId msg_id:8;
  VEFlags flags;
  VoiceEndpointSessionType session_type:8;
  VoiceEndpointResult result:8;
} SessionSetupResultMsg;

typedef struct PACKED {
  MsgId msg_id:8;
  VEFlags flags;
  AudioEndpointSessionId session_id;
  VoiceEndpointResult result:8;
  GenericAttributeList attr_list;
} VoiceSessionResultMsg;

typedef enum {
  VoiceRecordingEndpointResultSuccess = 0x00,
  VoiceRecordingEndpointResultNotFound = 0x01,
  VoiceRecordingEndpointResultBusy = 0x02,
  VoiceRecordingEndpointResultInvalidRequest = 0x03,
  VoiceRecordingEndpointResultUnsupported = 0x04,
  VoiceRecordingEndpointResultFailed = 0x05,
} VoiceRecordingEndpointResult;

typedef enum {
  VoiceRecordingPlaybackActionPlay = 0x00,
  VoiceRecordingPlaybackActionStop = 0x01,
} VoiceRecordingPlaybackAction;

typedef struct PACKED {
  uint8_t msg_id;
  VEFlags flags;
  uint8_t transaction_id;
  uint16_t offset;
  uint8_t limit;
} VoiceRecordingListRequest;

typedef struct PACKED {
  uint16_t id;
  uint32_t size_bytes;
  uint32_t duration_ms;
  uint32_t created;
  Uuid app_uuid;
} VoiceRecordingProtocolInfo;

typedef struct PACKED {
  uint8_t msg_id;
  VEFlags flags;
  uint8_t transaction_id;
  uint8_t result;
  uint8_t has_more;
  uint8_t count;
  VoiceRecordingProtocolInfo recordings[];
} VoiceRecordingListResponse;

typedef struct PACKED {
  uint8_t msg_id;
  VEFlags flags;
  uint8_t transaction_id;
  uint16_t recording_id;
} VoiceRecordingIdRequest;

typedef struct PACKED {
  uint8_t msg_id;
  VEFlags flags;
  uint8_t transaction_id;
  uint16_t recording_id;
  uint8_t action;
} VoiceRecordingPlaybackRequest;

typedef struct PACKED {
  uint8_t msg_id;
  VEFlags flags;
  uint8_t transaction_id;
  uint8_t result;
} VoiceRecordingCommandResponse;
