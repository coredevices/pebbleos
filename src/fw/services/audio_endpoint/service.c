/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/audio_endpoint.h"
#include "pbl/services/audio_endpoint_private.h"

#include "comm/bt_lock.h"
#include "pbl/services/comm_session/session_send_buffer.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/circular_buffer.h"

PBL_LOG_MODULE_DEFINE(service_audio_endpoint, CONFIG_SERVICE_AUDIO_ENDPOINT_LOG_LEVEL);

#define AUDIO_ENDPOINT (10000)

#define ACTIVE_MODE_TIMEOUT      (10000)
#define ACTIVE_MODE_START_BUFFER (100)

_Static_assert(ACTIVE_MODE_TIMEOUT > ACTIVE_MODE_START_BUFFER,
  "ACTIVE_MODE_TIMEOUT must be greater than ACTIVE_MODE_START_BUFFER");

typedef struct {
  AudioEndpointSessionId id;
  AudioEndpointStopTransferCallback stop_transfer;
  TimerID active_mode_trigger;
} AudioEndpointSession;

static AudioEndpointSessionId s_session_id = AUDIO_ENDPOINT_SESSION_INVALID_ID;
static AudioEndpointSession s_session;
static uint32_t s_dropped_frames;

static void prv_session_deinit(bool call_stop_handler) {
  bt_lock();
  if (call_stop_handler && s_session.stop_transfer) {
    s_session.stop_transfer(s_session.id);
  }

  if (s_session.active_mode_trigger != TIMER_INVALID_ID) {
    new_timer_delete(s_session.active_mode_trigger);
    s_session.active_mode_trigger = TIMER_INVALID_ID;
    CommSession *comm_session = comm_session_get_system_session();
    comm_session_set_responsiveness(
        comm_session, BtConsumerPpAudioEndpoint, ResponseTimeMax, 0);
  }

  s_session.id = AUDIO_ENDPOINT_SESSION_INVALID_ID;
  s_session.stop_transfer = NULL;
  bt_unlock();

  if (s_dropped_frames > 0) {
    PBL_LOG_WRN("Dropped %"PRIu32" frames during audio transfer", s_dropped_frames);
  }
}

void audio_endpoint_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t size) {
  MsgId msg_id = data[0];
  if (size >= sizeof(StopTransferMsg) && msg_id == MsgIdStopTransfer) {
    StopTransferMsg *msg = (StopTransferMsg *)data;

    if (msg->session_id == s_session.id) {
      prv_session_deinit(true /* call_stop_handler */);
    } else {
      PBL_LOG_WRN("Received mismatching session id: %u vs %u",
              msg->session_id, s_session.id);
    }
  }
}

static void prv_start_active_mode(void *data) {
  CommSession *comm_session = comm_session_get_system_session();
  comm_session_set_responsiveness_ext(comm_session, BtConsumerPpAudioEndpoint, ResponseTimeMin,
                                      MIN_LATENCY_MODE_TIMEOUT_AUDIO_SECS,
                                      NULL /* granted_handler */);
}

//! Never block on bt_lock from the timer task (comm_session_set_responsiveness_ext
//! takes it); defer to KernelBG.
static void prv_active_mode_timer_cb(void *data) {
  system_task_add_callback(prv_start_active_mode, NULL);
}

AudioEndpointSessionId audio_endpoint_setup_transfer(AudioEndpointStopTransferCallback stop_transfer) {

  if (s_session.id != AUDIO_ENDPOINT_SESSION_INVALID_ID) {
    return AUDIO_ENDPOINT_SESSION_INVALID_ID;
  }

  bt_lock();

  s_session.id = ++s_session_id;
  s_session.stop_transfer = stop_transfer;
  s_session.active_mode_trigger = new_timer_create();
  s_dropped_frames = 0;

  // restart active mode before it expires, this way it will never be off during the transfer
  new_timer_start(s_session.active_mode_trigger, ACTIVE_MODE_TIMEOUT - ACTIVE_MODE_START_BUFFER,
      prv_active_mode_timer_cb, NULL, TIMER_START_FLAG_REPEATING);

  bt_unlock();

  prv_start_active_mode(NULL);

  return s_session.id;
}

void audio_endpoint_add_frame(AudioEndpointSessionId session_id, uint8_t *frame,
    uint8_t frame_size) {
  PBL_ASSERTN(session_id != AUDIO_ENDPOINT_SESSION_INVALID_ID);

  if (s_session.id != session_id) {
    return;
  }

  CommSession *comm_session = comm_session_get_system_session();
  SendBuffer *sb = comm_session_send_buffer_begin_write(comm_session, AUDIO_ENDPOINT,
                                                        sizeof(DataTransferMsg) + frame_size + 1,
                                                        0 /* timeout_ms, never block */);
  if (!sb) {
    s_dropped_frames++;
    PBL_LOG_DBG("Dropping a frame...");
    return;
  }

  uint8_t header[sizeof(DataTransferMsg) + sizeof(uint8_t) /* frame_size */];
  DataTransferMsg *msg = (DataTransferMsg *) header;
  *msg = (const DataTransferMsg) {
    .msg_id = MsgIdDataTransfer,
    .session_id = session_id,
    .frame_count = 1,
  };
  msg->frames[0] = frame_size;

  comm_session_send_buffer_write(sb, header, sizeof(header));
  comm_session_send_buffer_write(sb, frame, frame_size);
  comm_session_send_buffer_end_write(sb);
}

void audio_endpoint_cancel_transfer(AudioEndpointSessionId session_id) {
  PBL_ASSERTN(session_id != AUDIO_ENDPOINT_SESSION_INVALID_ID);

  if (s_session.id != session_id) {
    return;
  }

  prv_session_deinit(false /* call_stop_handler */);
}

void audio_endpoint_stop_transfer(AudioEndpointSessionId session_id) {
  PBL_ASSERTN(session_id != AUDIO_ENDPOINT_SESSION_INVALID_ID);

  if (s_session.id != session_id) {
    return;
  }

  StopTransferMsg msg = (const StopTransferMsg) {
    .msg_id = MsgIdStopTransfer,
    .session_id = session_id,
  };

  prv_session_deinit(false /* call_stop_handler */);

  comm_session_send_data(comm_session_get_system_session(), AUDIO_ENDPOINT, (const uint8_t *) &msg,
                         sizeof(msg), COMM_SESSION_DEFAULT_TIMEOUT);
}
