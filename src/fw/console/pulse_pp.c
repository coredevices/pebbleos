/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/bt_driver_comm.h>

#include "comm/bt_lock.h"

#include "console/pulse_protocol_impl.h"

#include "kernel/event_loop.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"

#include "pbl/services/comm_session/session_transport.h"

#include "system/passert.h"
#include <pbl/logging/logging.h>

#include "pbl/util/attributes.h"
#include "pbl/util/math.h"

#include <string.h>

#define PULSE_PP_OPCODE_DATA (1)
#define PULSE_PP_OPCODE_OPEN (2)
#define PULSE_PP_OPCODE_CLOSE (3)

#define PULSE_PP_OPCODE_UNKNOWN (255)

typedef struct PACKED PulsePPPacket {
  uint8_t opcode;
  uint8_t data[0];
} PulsePPPacket;

typedef struct PACKED PulsePPCallbackPacket {
  size_t packet_length;
  PulsePPPacket packet;
} PulsePPCallbackPacket;

typedef struct {
  CommSession *session;
} PULSETransport;

//! The CommSession that the PULSE transport is managing.
//! Currently there's only one for the System session.
static PULSETransport s_transport;

static void prv_send_next(Transport *transport) {
  CommSession *session = s_transport.session;
  PBL_ASSERTN(session);

  size_t bytes_remaining = comm_session_send_queue_get_length(session);
  size_t mss = pulse_reliable_max_send_size() - sizeof(PulsePPPacket);

  while (bytes_remaining) {
    bt_unlock();
    PulsePPPacket *resp = (PulsePPPacket*) pulse_reliable_send_begin(PULSE2_PEBBLE_PROTOCOL);
    bt_lock();

    if (resp) {
      resp->opcode = PULSE_PP_OPCODE_DATA;

      const size_t bytes_to_copy = MIN(bytes_remaining, mss);
      comm_session_send_queue_copy(session, 0 /* start_offset */,
                                   bytes_to_copy, &resp->data[0]);
      pulse_reliable_send(resp, bytes_to_copy + sizeof(PulsePPPacket));
      comm_session_send_queue_consume(session, bytes_to_copy);

      bytes_remaining -= bytes_to_copy;
    } else {
      // Reliable transport went down while waiting to send.
      // The CommSession has already been closed so simply return without doing
      // anything further.
      break;
    }
  }
}

static void prv_reset(Transport *transport) {
  PBL_LOG_WRN("Unimplemented");
}

static void prv_granted_kernel_main_cb(void *ctx) {
  ResponsivenessGrantedHandler granted_handler = ctx;
  granted_handler();
}

static void prv_set_connection_responsiveness(
    Transport *transport, BtConsumer consumer, ResponseTimeState state, uint16_t max_period_secs,
    ResponsivenessGrantedHandler granted_handler) {
  if (granted_handler) {
    launcher_task_add_callback(prv_granted_kernel_main_cb, granted_handler);
  }
}

static CommSessionTransportType prv_get_type(struct Transport *transport) {
  return CommSessionTransportType_PULSE;
}

static void prv_send_job(void *data) {
  CommSession *session = (CommSession *)data;
  bt_driver_run_send_next_job(session, true);
}

static bool prv_schedule_send_next_job(CommSession *session) {
  launcher_task_add_callback(prv_send_job, session);
  return true;
}

static bool prv_is_current_task_schedule_task(struct Transport *transport) {
  return launcher_task_is_current_task();
}

//! Defined in session.c
extern void comm_session_set_capabilities(
    CommSession *session, CommSessionCapability capability_flags);

bool pulse_transport_is_connected(void) {
  return (s_transport.session != NULL);
}

// -----------------------------------------------------------------------------------------
void pulse_transport_set_connected(bool is_connected) {
  if (pulse_transport_is_connected() == is_connected) {
    return;
  }

  static const TransportImplementation s_pulse_transport_implementation = {
    .send_next = prv_send_next,
    .reset = prv_reset,
    .set_connection_responsiveness = prv_set_connection_responsiveness,
    .get_type = prv_get_type,
    .schedule = prv_schedule_send_next_job,
    .is_current_task_schedule_task = prv_is_current_task_schedule_task,
  };

  bool send_event = true;

  if (is_connected) {
    s_transport.session = comm_session_open((Transport *) &s_transport,
                                            &s_pulse_transport_implementation,
                                            TransportDestinationHybrid);
    if (!s_transport.session) {
      PBL_LOG_ERR("CommSession couldn't be opened");
      send_event = false;
    }

    // Give it the appropriate capabilities
    const CommSessionCapability capabilities = CommSessionRunState |
                                               CommSessionInfiniteLogDumping |
                                               CommSessionVoiceApiSupport |
                                               CommSessionAppMessage8kSupport |
                                               CommSessionWeatherAppSupport |
                                               CommSessionExtendedNotificationService;
    comm_session_set_capabilities(s_transport.session, capabilities);
  } else {
    comm_session_close(s_transport.session, CommSessionCloseReason_UnderlyingDisconnection);
    s_transport.session = NULL;
  }

  if (send_event) {
    PebbleEvent e = {
      .type = PEBBLE_BT_CONNECTION_EVENT,
      .bluetooth = {
        .connection = {
          .state = (s_transport.session) ? PebbleBluetoothConnectionEventStateConnected
          : PebbleBluetoothConnectionEventStateDisconnected
        }
      }
    };
    event_put(&e);
  }
}

static void prv_pulse_pp_transport_set_connected(bool connected) {
  bt_lock();
  pulse_transport_set_connected(connected);
  bt_unlock();
}

static void prv_pulse_pp_handle_data(void *data, size_t length) {
  bt_lock();

  if (!s_transport.session) {
    PBL_LOG_ERR("Received PULSE serial data, but session not connected!");
    goto unlock;
  }
  comm_session_receive_router_write(s_transport.session, data, length);

unlock:
  bt_unlock();
}

static void prv_pulse_pp_send_cb(void *data) {
  PulsePPCallbackPacket *cb_data = data;
  uint8_t *resp = pulse_reliable_send_begin(PULSE2_PEBBLE_PROTOCOL);
  memcpy(resp, &cb_data->packet, cb_data->packet_length);
  pulse_reliable_send(resp, cb_data->packet_length);
  kernel_free(data);
}

static void prv_pulse_pp_send(uint8_t opcode, uint8_t *data, size_t data_length) {
  size_t packet_length = sizeof(PulsePPCallbackPacket) + data_length;
  PulsePPCallbackPacket *cb_data = kernel_malloc_check(packet_length);
  cb_data->packet_length = sizeof(PulsePPPacket) + data_length;
  cb_data->packet.opcode = opcode;

  if (data) {
    memcpy(&cb_data->packet.data[0], data, data_length);
  }

  launcher_task_add_callback(prv_pulse_pp_send_cb, cb_data);
}

void pulse_pp_transport_open_handler(void) {
  return;
}

void pulse_pp_transport_closed_handler(void) {
  prv_pulse_pp_transport_set_connected(false);
}

void pulse_pp_transport_handle_received_data(void *data, size_t length) {
  PulsePPPacket *packet = data;

  switch (packet->opcode) {
    case PULSE_PP_OPCODE_DATA:
      prv_pulse_pp_handle_data(&packet->data[0], length - sizeof(PulsePPPacket));
      break;
    case PULSE_PP_OPCODE_OPEN:
      prv_pulse_pp_send(PULSE_PP_OPCODE_OPEN, NULL, 0);
      prv_pulse_pp_transport_set_connected(true);
      break;
    case PULSE_PP_OPCODE_CLOSE:
      prv_pulse_pp_transport_set_connected(false);
      prv_pulse_pp_send(PULSE_PP_OPCODE_CLOSE, NULL, 0);
      break;
    default:
      prv_pulse_pp_send(PULSE_PP_OPCODE_UNKNOWN, &packet->opcode, sizeof(packet->opcode));
      break;
  }
}
