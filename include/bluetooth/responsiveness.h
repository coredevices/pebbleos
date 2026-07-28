/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>
#include <inttypes.h>

#include <bluetooth/bluetooth_types.h>

#include "bluetooth/gap_le_connect.h"
#include "pbl/util/attributes.h"

typedef enum {
  BtConsumerNone = 0,
  // Every sub-module has its own consumer name. We try to enter & exit
  // from low latency states within the same module
  BtConsumerApp,
  BtConsumerLePairing,
  BtConsumerLeServiceDiscovery,
  BtConsumerMusicServiceIndefinite,
  BtConsumerMusicServiceMomentary,
  BtConsumerPpAppFetch,
  BtConsumerPpAppMessage,
  BtConsumerPpAudioEndpoint,
  BtConsumerPpGetBytes,
  BtConsumerPpLogDump,
  BtConsumerPpPutBytes,
  BtConsumerPpScreenshot,
  BtConsumerPpVoiceEndpoint,
  BtConsumerPrompt,
  BtConsumerTimelineActionMenu,
  BtConsumerPRF,
  BtConsumerPebblePairingServiceRemoteDevice,
  BtConsumerHidRemote,
  BtConsumerUnitTests, // For unit testing
  NumBtConsumer,
} BtConsumer;

typedef enum {
  ResponseTimeInvalid = -1,
  ResponseTimeMax = 0, // lowest throughput, most friendly power profile
  ResponseTimeMiddle,
  ResponseTimeMin, // highest throughput, least friendly power profile
  NumResponseTimeState,
} ResponseTimeState;

//! Callback to call when the requested response time has been negotiated and granted.
typedef void (*ResponsivenessGrantedHandler)(void);

// Longest duration we want to be in Min latency for different modules
#define MIN_LATENCY_MODE_TIMEOUT_AUDIO_SECS                (10)
#define MIN_LATENCY_MODE_TIMEOUT_APP_FETCH_SECS            (5)
#define MIN_LATENCY_MODE_TIMEOUT_APP_MESSAGE_SECS          (10)
#define MIN_LATENCY_MODE_TIMEOUT_CD_SECS                   (10)
#define MIN_LATENCY_MODE_TIMEOUT_HID_SECS                  (60)
#define MIN_LATENCY_MODE_TIMEOUT_PROTOCOL_RECV_SECS        (60)
#define MIN_LATENCY_MODE_TIMEOUT_PUT_BYTES_SECS            (60)
#define MIN_LATENCY_MODE_TIMEOUT_SCREENSHOT_SECS           (5)
#define MIN_LATENCY_MODE_TIMEOUT_TIMELINE_ACTION_MENU_SECS (10)
#define MIN_LATENCY_MODE_TIMEOUT_VOICE_SECS                (10)

//! Connection Parameters Update Request Packet
typedef struct PACKED { // PACKED since this struct is serialized
  uint16_t interval_min_1_25ms;
  uint16_t interval_max_1_25ms;
  uint16_t slave_latency_events;
  uint16_t supervision_timeout_10ms;
} BleConnectionParamsUpdateReq;

bool bt_driver_le_connection_parameter_update(
    const BTDeviceInternal *addr, const BleConnectionParamsUpdateReq *req);
