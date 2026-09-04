/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/event_loop.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"

#include "pbl/services/comm_session/session_transport.h"

#include "system/passert.h"
#include <pbl/logging/logging.h>

#include "comm/bt_lock.h"

#include <pbl/drivers/qemu/qemu_serial.h>
#include <pbl/drivers/qemu/qemu_serial_private.h>

#include "pbl/util/math.h"

#include <bluetooth/qemu_transport.h>

typedef struct {
  CommSession *session;
} QemuTransport;

// -----------------------------------------------------------------------------------------
// Static Variables (protected by bt_lock)

//! The CommSession that the QEMU transport is managing.
//! Currently there's only one for the System session.
static QemuTransport s_transport;

//! Emulated bluetooth connection state
static bool s_emulated_session_connected;

// -----------------------------------------------------------------------------------------
// bt_lock() is held by caller
static void prv_send_next(Transport *transport) {
  CommSession *session = s_transport.session;
  PBL_ASSERTN(session);
  size_t bytes_remaining = comm_session_send_queue_get_length(session);
  if (bytes_remaining == 0) {
    return;
  }

  const size_t temp_buffer_size = MIN(bytes_remaining, QEMU_MAX_DATA_LEN);
  uint8_t *temp_buffer = kernel_malloc_check(temp_buffer_size);
  while (bytes_remaining) {
    const size_t bytes_to_copy = MIN(bytes_remaining, temp_buffer_size);
    comm_session_send_queue_copy(session, 0 /* start_offset */, bytes_to_copy, temp_buffer);
    qemu_serial_send(QemuProtocol_SPP, temp_buffer, bytes_to_copy);
    comm_session_send_queue_consume(session, bytes_to_copy);
    bytes_remaining -= bytes_to_copy;
  }
  kernel_free(temp_buffer);
}

// -----------------------------------------------------------------------------------------
// bt_lock() is held by caller
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
  // PutBytes calls this every single packet — at INFO it pushes ~76 bytes/packet
  // into the 1200-byte advanced_logging ring buffer, which fills in ~16 packets
  // and forces inline flash-logging flushes on KernelMain.  That starves
  // SystemTask's PutBytes callback (it's competing for s_flash_write_mutex) and
  // wedges the install with no watchdog reboot.  CLAUDE.md is explicit that
  // high-frequency paths must not log at INFO; this is one of them.
  PBL_LOG_DBG("Consumer %d: requesting change to %d for %" PRIu16 "seconds",
          consumer, state, max_period_secs);

  // it's qemu, our request to bump the speed is always granted!
  if (granted_handler) {
    launcher_task_add_callback(prv_granted_kernel_main_cb, granted_handler);
  }
}

static CommSessionTransportType prv_get_type(struct Transport *transport) {
  return CommSessionTransportType_QEMU;
}

//! Copy of the same function in session.c
static void prv_put_comm_session_event(bool is_open, bool is_system) {
  PebbleEvent event = {
    .type = PEBBLE_COMM_SESSION_EVENT,
    .bluetooth.comm_session_event.is_open = is_open,
    .bluetooth.comm_session_event
    .is_system = is_system,
  };
  event_put(&event);
}

//! Defined in session.c
extern void comm_session_set_capabilities(
    CommSession *session, CommSessionCapability capability_flags);

// -----------------------------------------------------------------------------------------
void qemu_transport_set_connected(bool is_connected) {
  bt_lock();

  const bool transport_is_connected = (s_transport.session) && s_emulated_session_connected;
  if (transport_is_connected == is_connected) {
    bt_unlock();
    return;
  }

  static const TransportImplementation s_qemu_transport_implementation = {
    .send_next = prv_send_next,
    .reset = prv_reset,
    .set_connection_responsiveness = prv_set_connection_responsiveness,
    .get_type = prv_get_type,
  };

  bool send_event = true;

  if (is_connected && !s_transport.session) {
    PBL_LOG_DBG("Opening new QemuTransport CommSession");
    s_transport.session = comm_session_open((Transport *) &s_transport,
                                            &s_qemu_transport_implementation,
                                            TransportDestinationHybrid);
    if (!s_transport.session) {
      PBL_LOG_ERR("CommSession couldn't be opened");
      send_event = false;
    }

    // Give it the appropriate capabilities
    const CommSessionCapability capabilities = CommSessionRunState |
                                               CommSessionInfiniteLogDumping |
                                               CommSessionVoiceApiSupport |
                                               CommSessionAppMessage8kSupport;
    comm_session_set_capabilities(s_transport.session, capabilities);

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

    s_emulated_session_connected = is_connected;
  }

  if (s_emulated_session_connected != is_connected) {
    PBL_LOG_DBG("Toggling emulated session connection state --> %s", is_connected ? "connecting" : "disconnecting");

    // Only send PEBBLE_COMM_SESSION_EVENT without opening or terminating a session
    // Apps will get notified in both case but the rest of the firmware will still
    // communicate normally in case of disconnection
    prv_put_comm_session_event(is_connected, true);
    prv_put_comm_session_event(is_connected, false);

    s_emulated_session_connected = is_connected;
  }

  bt_unlock();
}

void qemu_transport_close_session() {
  if (!s_transport.session) return;

  bt_lock();

  comm_session_close(s_transport.session, CommSessionCloseReason_UnderlyingDisconnection);
  s_transport.session = NULL;

  PebbleEvent e = {
    .type = PEBBLE_BT_CONNECTION_EVENT,
    .bluetooth = {
      .connection = {
        .state = PebbleBluetoothConnectionEventStateDisconnected
      }
    }
  };
  event_put(&e);

  bt_unlock();
}

// -----------------------------------------------------------------------------------------
bool qemu_transport_is_connected(void) {
  return (s_transport.session != NULL) && s_emulated_session_connected;
}

// -----------------------------------------------------------------------------------------
// Handle incoming Qemu-SPP packet data
void qemu_transport_handle_received_data(const uint8_t *data, uint32_t length) {
  bt_lock();
  if (!s_transport.session) {
    PBL_LOG_ERR("Received QEMU serial data, but session not connected!");
    goto unlock;
  }
  comm_session_receive_router_write(s_transport.session, data, length);
unlock:
  bt_unlock();
}
