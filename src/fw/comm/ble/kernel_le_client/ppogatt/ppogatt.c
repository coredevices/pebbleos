/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "ppogatt.h"
#include "ppogatt_internal.h"

#include "comm/ble/gap_le_connection.h"
#include "comm/ble/gatt_client_operations.h"
#include "comm/bt_lock.h"

#include <bluetooth/bt_driver_ppog_reversed.h>
#include <bluetooth/gatt.h>

#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/comm_session/session_transport.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/system_task.h"

#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/likely.h"
#include "pbl/util/list.h"
#include "pbl/util/math.h"

#include <inttypes.h>

#include <pbl/drivers/rtc.h>

//! See https://pebbletechnology.atlassian.net/wiki/pages/viewpage.action?pageId=22511665
//! for detailed information regarding the PPoGATT protocol state machine.

typedef enum {
  StateDisconnectedReadingMeta,
  StateDisconnectedAwaitingMetaRetry,
  StateDisconnectedSubscribingData,
  //! Reversed role only: waiting for the phone to send the initial ResetRequest.
  StateConnectedClosedAwaitingResetRequest,
  StateConnectedClosedAwaitingResetCompleteSelfInitiatedReset,
  StateConnectedClosedAwaitingResetCompleteSelfInitiatedResetStalled,
  StateConnectedClosedAwaitingResetCompleteRemoteInitiatedReset,
  StateConnectedOpen,
} State;

typedef enum {
  DeleteReason_DuplicateServer = CommSessionCloseReason_TransportSpecificBegin,
  DeleteReason_ServiceRemoved,
  DeleteReason_InvalidateAllReferences,
  DeleteReason_ResetSelfInitiated,
  DeleteReason_ResetRemoteInitiated,
  DeleteReason_CloseCalled,
  DeleteReason_DestroyCalled,
  DeleteReason_SubscribeFailure,
  DeleteReason_MetaDataReadFailure,
  DeleteReason_MetaDataInvalid,
  DeleteReason_CouldntOpenCommSession,
  DeleteReasonCount,
} DeleteReason;

_Static_assert(DeleteReasonCount <= CommSessionCloseReason_TransportSpecificEnd + 1, "");

typedef enum {
  AckTimeoutState_Inactive = 0,
  AckTimeoutState_Active = 1,
  AckTimeoutState_TimedOut = AckTimeoutState_Active + PPOGATT_TIMEOUT_TICKS,
} AckTimeoutState;

typedef struct PPoGATTClient {
  ListNode node;
  State state;
  uint8_t version;
  PPoGATTRole role;

  // TODO: Save some memory and point to app metadata instead?
  Uuid app_uuid;

  struct {
    BLECharacteristic meta;
    BLECharacteristic data;
  } characteristics;

  //! State for the reversed role (watch hosts the GATT service).
  struct {
    uint16_t conn_handle;
    GAPLEConnection *connection;
  } rev;

  //! Stuffs that deals with inbound data
  struct {
    uint8_t next_expected_data_sn;
  } in;

  //! Stuffs that deals with outbound data
  struct {
    union {
      PPoGATTPacket reset_packet_to_send;
      //! Set to 0 if there is no reset packet to send
      uint8_t reset_packet_byte;
    };
    union {
      PPoGATTPacket ack_packet_to_send;
      //! Set to 0 if there is no ack packet to send
      uint8_t ack_packet_byte;
    };

    uint16_t payload_sizes[PPOGATT_SN_MOD_DIV];
    uint8_t tx_window_size;
    uint8_t rx_window_size;

    AckTimeoutState ack_timeout_state;

    //! Number of consecutive timeouts so far
    uint8_t timeouts_counter;

    uint8_t next_expected_ack_sn;
    uint8_t next_data_sn;

    bool send_rx_ack_now; //! True if we want to flush the Ack immediately!
    uint8_t outstanding_rx_ack_count; //! Count of how many data packets we have yet to Ack
  } out;

  //! Number of consecutive resets so far
  uint8_t resets_counter;

  //! Number of meta read retry attempts
  uint8_t meta_read_retries;

  bool disconnect_requested; //! True if the client requested a disconnect

  //! Guards prv_send_next_packets against re-entry while bt_lock is dropped.
  bool send_in_progress;

  //! Number of consecutive send attempts that failed for lack of BT stack
  //! buffers (reversed role only); 0 when not stalled.
  uint16_t send_retry_count;

  TimerID rx_ack_timer;   //! Timer to ensure Acks for data are dispatched regularly

  //! Timer to retry sending after the BT stack ran out of buffers
  //! (reversed role only; NimBLE has no "buffers freed" event to wait for)
  TimerID send_retry_timer;

  //! Whether the PPoGATT server transports "System", "App" or "Hybrid" PP sessions.
  TransportDestination destination;

  //! The CommSession associated with the client.
  //! @note Each PPoGATT client (transport) is responsible for managing the CommSession's lifecycle,
  //! by calling comm_session_open / comm_session_close at the appropriate times.
  CommSession *session;

  //! Tick count when this client was created, for measuring handshake duration
  RtcTicks created_ticks;
} PPoGATTClient;

// -------------------------------------------------------------------------------------------------
// Static variables

static PPoGATTClient *s_ppogatt_head;

static RegularTimerInfo s_ack_timer;

//! Last timer value. It rolls over after 3, back to 0.
//! This way the time-outs can be crammed into 2 bits per packet.
static uint8_t s_timer_ticks;

static uint8_t s_disconnect_counter;

//! Caps rediscovery on stale meta handles to once per BLE connection.
static bool s_rediscovery_requested_this_connection;

//! The connection currently using reversed PPoG, or NULL if none. Forward
//! PPoG yields to reversed on this connection.
static GAPLEConnection *s_reversed_active_conn;

// -------------------------------------------------------------------------------------------------
// Function Prototypes

static void prv_send_next_packets(PPoGATTClient *client);
static void prv_start_reset(PPoGATTClient *client);
static void prv_request_meta_rediscovery(PPoGATTClient *client);

extern BTErrno gatt_client_discovery_rediscover_all(const BTDeviceInternal *device);

//! Gets the GAPLEConnection associated with the characteristic reference.
//! @return The connection or NULL in case it could not be found.
//! @note The caller MUST own bt_lock()
extern GAPLEConnection *gatt_client_characteristic_get_connection(BLECharacteristic characteristic);

//! Gets the att_handle and connection for a characteristic.
//! @return The att_handle or 0 if not found. Connection is returned via out parameter.
//! @note The caller MUST own bt_lock()
extern uint16_t gatt_client_characteristic_get_handle_and_connection(
    BLECharacteristic characteristic_ref, GAPLEConnection **connection_out);


// -------------------------------------------------------------------------------------------------
void ppogatt_reset_disconnect_counter(void) {
    s_disconnect_counter = 0;
}

static bool prv_client_supports_enhanced_throughput_features(const PPoGATTClient *client) {
  // In PPoGATT V1, two features were added to allow for enhanced throughput:
  // 1) Negotiable RX/TX in-flight windows - This let's the phone put more data out over the
  //                                         air and not block waiting for an Ack
  // 2) Coalesced ACKing - Since 1) makes the window size larger, it's beneficial to flush Acks less
  //                       frequently. This reduces the strain on the BT controller scheduler and
  //                       frees up more slots for outbound data packets. As long as we send an Ack
  //                       before the in-flight window fills, the phone can keep pushing data.
  //                       If very little data is in flight, flushing Acks periodically will have no
  //                       impact on throughput
  return (client->version >= 1);
}

// -------------------------------------------------------------------------------------------------

static GAPLEConnection *prv_get_connection(const PPoGATTClient *client) {
  if (client->role == PPoGATTRoleReversed) {
    return client->rev.connection;
  }
  return gatt_client_characteristic_get_connection(client->characteristics.meta);
}

static void prv_set_connection_responsiveness(
    Transport *transport, BtConsumer consumer, ResponseTimeState state, uint16_t max_period_secs,
    ResponsivenessGrantedHandler granted_handler) {
  PPoGATTClient *client = (PPoGATTClient *) transport;
  GAPLEConnection *connection = prv_get_connection(client);
  conn_mgr_set_ble_conn_response_time_ext(connection, consumer, state, max_period_secs,
                                          granted_handler);
}

static const Uuid *prv_get_uuid(Transport *transport) {
  PPoGATTClient *client = (PPoGATTClient *) transport;
  return &client->app_uuid;
}

static CommSessionTransportType prv_get_type(struct Transport *transport) {
  return CommSessionTransportType_PPoGATT;
}

static const TransportImplementation s_ppogatt_transport_implementation = {
  .send_next = &ppogatt_send_next,
  .close = &ppogatt_close,
  .reset = &ppogatt_reset,
  .set_connection_responsiveness = prv_set_connection_responsiveness,
  .get_uuid = prv_get_uuid,
  .get_type = prv_get_type,
};

// -------------------------------------------------------------------------------------------------

static void prv_send_next_packets_async(PPoGATTClient *client) {
  // Go through comm_session, because this will skip scheduling a callback to send_next if one is
  // already scheduled, to avoid spamming the KernelBG queue and doing unnecessary work.
  comm_session_send_next(client->session);
}

// -------------------------------------------------------------------------------------------------

static uint32_t prv_sn_distance(uint8_t sn_begin_incl, uint32_t sn_end_excl) {
  return ((uint32_t) PPOGATT_SN_MOD_DIV + sn_end_excl - sn_begin_incl) % PPOGATT_SN_MOD_DIV;
}

//! @return Number of packets in flight, *excluding* packets that are pending retransmission.
static uint32_t prv_num_packets_in_flight(const PPoGATTClient *client) {
  return prv_sn_distance(client->out.next_expected_ack_sn, client->out.next_data_sn);
}

static uint32_t prv_next_sn(uint32_t current_sn) {
  return (current_sn + 1) % PPOGATT_SN_MOD_DIV;
}

static uint32_t prv_prev_sn(uint32_t sn) {
  return ((PPOGATT_SN_MOD_DIV + sn - 1) % PPOGATT_SN_MOD_DIV);
}

// -------------------------------------------------------------------------------------------------

static uint16_t prv_get_payload_size_for_sn(const PPoGATTClient *client, uint32_t sn) {
  return client->out.payload_sizes[sn];
}

static bool prv_is_packet_with_sn_awaiting_ack(const PPoGATTClient *client, uint32_t sn) {
  return (prv_get_payload_size_for_sn(client, sn) != 0);
}

static uint16_t prv_total_num_bytes_awaiting_ack_up_to(const PPoGATTClient *client,
                                                       uint32_t sn_end_excl) {
  uint16_t num_bytes = 0;
  for (uint32_t sn = client->out.next_expected_ack_sn;
       sn != sn_end_excl; sn = prv_next_sn(sn)) {
    num_bytes += prv_get_payload_size_for_sn(client, sn);
  }
  return num_bytes;
}

static uint16_t prv_total_num_bytes_awaiting_ack(const PPoGATTClient *client) {
  return prv_total_num_bytes_awaiting_ack_up_to(client, client->out.next_data_sn);
}

static void prv_set_payload_size_for_sn(PPoGATTClient *client, uint32_t sn, uint16_t payload_size) {
  client->out.payload_sizes[sn] = payload_size;
}

static void prv_clear_payload_sizes_up_to(PPoGATTClient *client,
                                          uint32_t sn_end_excl) {
  for (uint32_t sn = client->out.next_expected_ack_sn;
       sn != sn_end_excl; sn = prv_next_sn(sn)) {
    prv_set_payload_size_for_sn(client, sn, 0);
  }
}

// -------------------------------------------------------------------------------------------------
// Ack Time-out related things.
// The effective timeout duration will be between 2 and 3 seconds, depending on when in the second
// the timeout is set (RegularTimer is used).

static void prv_reset_ack_timeout(PPoGATTClient *client) {
  client->out.ack_timeout_state = AckTimeoutState_Active;
}

static void prv_roll_back(PPoGATTClient *client, uint32_t sn) {
  if (++client->out.timeouts_counter >= PPOGATT_TIMEOUT_COUNT_MAX) {
    PBL_LOG_ERR("Resetting because max timeouts reached...");
    prv_start_reset(client);
    return;
  }

  PBL_LOG_WRN("Rolling back from (%u, %u) to %"PRIu32,
          client->out.next_data_sn, client->out.next_expected_ack_sn, sn);

  // Go back and send again:
  // No need to worry about the timeouts of these packets hitting, because prv_check_timeouts uses
  // next_data_sn and next_expected_ack_sn to determine which packets can time-out.
  client->out.next_data_sn = sn;
  client->out.next_expected_ack_sn = sn;
  prv_reset_ack_timeout(client);

  // Don't send from Timer task
  prv_send_next_packets_async(client);
}

static bool prv_has_timeout(const PPoGATTClient *client) {
  return (client->out.ack_timeout_state != AckTimeoutState_Inactive &&
          client->out.ack_timeout_state >= AckTimeoutState_TimedOut);
}

static void prv_increment_timeout_counter_if_necessary(PPoGATTClient *client) {
  if (client->out.ack_timeout_state >= AckTimeoutState_Active) {
    client->out.ack_timeout_state++;
  }
}

static void prv_check_timeouts(PPoGATTClient *client) {
  if (client->state == StateConnectedClosedAwaitingResetCompleteSelfInitiatedReset ||
      client->state == StateConnectedClosedAwaitingResetCompleteRemoteInitiatedReset) {
    if (prv_has_timeout(client)) {
      // We've timed out waiting for a reset to be completed, start over:
      PBL_LOG_WRN("Timed out waiting for Reset Complete, Resetting again...");
      prv_start_reset(client);
    }
    return;
  } else if (client->state == StateConnectedClosedAwaitingResetCompleteSelfInitiatedResetStalled) {
    return;
  }

  uint8_t sn = client->out.next_expected_ack_sn;
  if (prv_has_timeout(client)) {
    prv_roll_back(client, sn);
    // Return, because all packets after the timed-out one have been "rolled back" now,
    // no point in continuing.
    return;
  }
}

static void prv_ack_timeout_kernelmain_cb(void *unused) {
  bt_lock();
  {
    PPoGATTClient *client = s_ppogatt_head;
    while (client) {
      prv_increment_timeout_counter_if_necessary(client);
      prv_check_timeouts(client);
      client = (PPoGATTClient *) client->node.next;
    }
  }
  bt_unlock();
}

//! Never block on bt_lock from the timer task (it stalls every other timer in
//! the system); bounce to KernelMain, where the rest of the PPoG state machine runs.
static void prv_timer_callback(void *unused) {
  launcher_task_add_callback(prv_ack_timeout_kernelmain_cb, NULL);
}

// -------------------------------------------------------------------------------------------------

static PPoGATTClient *prv_create_client(TimerID rx_ack_timer, TimerID send_retry_timer) {
  PPoGATTClient *client = kernel_malloc(sizeof(PPoGATTClient));
  if (!client) {
    return NULL;
  }
  *client = (PPoGATTClient){};
  client->app_uuid = UUID_INVALID;
  client->rx_ack_timer = rx_ack_timer;
  client->send_retry_timer = send_retry_timer;
  client->created_ticks = rtc_get_ticks();
  s_ppogatt_head = (PPoGATTClient *) list_prepend((ListNode *)s_ppogatt_head, &client->node);
  if (!regular_timer_is_scheduled(&s_ack_timer)) {
    s_ack_timer.cb = prv_timer_callback;
    regular_timer_add_multisecond_callback(&s_ack_timer, PPOGATT_TIMEOUT_TICK_INTERVAL_SECS);
  }
  return client;
}

// -------------------------------------------------------------------------------------------------

static void prv_delete_client(PPoGATTClient *client, bool is_disconnected, DeleteReason reason) {
  const uint32_t elapsed_ms = (rtc_get_ticks() - client->created_ticks) * 1000 / RTC_TICKS_HZ;
  PBL_LOG_INFO("PPoGATT client deleted: role=%u state=%u reason=%u disconnected=%u after %"PRIu32"ms",
          client->role, client->state, reason, is_disconnected, elapsed_ms);
  // Unsubscribe from the phone's Data characteristic (forward role only):
  if (client->role == PPoGATTRoleForward &&
      client->state > StateDisconnectedSubscribingData && !is_disconnected) {
    BLECharacteristic data_char = client->characteristics.data;
    // Release bt_lock before calling into NimBLE to avoid deadlock with ble_hs_mutex.
    bt_unlock();
    gatt_client_subscriptions_subscribe(data_char, BLESubscriptionNone, GAPLEClientKernel);
    bt_lock();
  }

  if (client->state == StateConnectedOpen) {
    comm_session_close(client->session, (CommSessionCloseReason)reason);
  }

  if (client->role == PPoGATTRoleReversed && s_reversed_active_conn == client->rev.connection) {
    s_reversed_active_conn = NULL;
  }

  list_remove(&client->node, (ListNode **) &s_ppogatt_head, NULL);
  new_timer_delete(client->rx_ack_timer);
  new_timer_delete(client->send_retry_timer);
  kernel_free(client);

  if (s_ppogatt_head == NULL) {
    regular_timer_remove_callback(&s_ack_timer);
  }
}

// -------------------------------------------------------------------------------------------------

static bool prv_characteristic_filter_callback(ListNode *found_node, void *data) {
  const PPoGATTClient *client = (const PPoGATTClient *) found_node;
  const BLECharacteristic characteristic = (const BLECharacteristic) data;
  return (client->characteristics.data == characteristic ||
          client->characteristics.meta == characteristic);
}

static PPoGATTClient * prv_find_client_with_characteristic(BLECharacteristic characteristic,
                                                           bool *is_data) {
  PPoGATTClient *client =
        (PPoGATTClient *) list_find((ListNode *)s_ppogatt_head, prv_characteristic_filter_callback,
                                    (void *)(uintptr_t) characteristic);
  if (client && is_data) {
    *is_data = (client->characteristics.data == characteristic);
  }
  return client;
}

// -------------------------------------------------------------------------------------------------

static bool prv_uuid_filter_callback(ListNode *found_node, void *data) {
  const PPoGATTClient *client = (const PPoGATTClient *) found_node;
  const Uuid *uuid = (const Uuid *) data;
  return uuid_equal(&client->app_uuid, uuid);
}

static PPoGATTClient * prv_find_client_with_uuid(const Uuid *uuid) {
  return (PPoGATTClient *) list_find((ListNode *)s_ppogatt_head,
                                     prv_uuid_filter_callback, (void *) uuid);
}

// -------------------------------------------------------------------------------------------------

static bool prv_reversed_conn_filter_callback(ListNode *found_node, void *data) {
  const PPoGATTClient *client = (const PPoGATTClient *) found_node;
  const uint16_t conn_handle = (uint16_t)(uintptr_t) data;
  return (client->role == PPoGATTRoleReversed && client->rev.conn_handle == conn_handle);
}

static PPoGATTClient *prv_find_reversed_client_for_conn(uint16_t conn_handle) {
  return (PPoGATTClient *) list_find((ListNode *) s_ppogatt_head,
                                     prv_reversed_conn_filter_callback,
                                     (void *)(uintptr_t) conn_handle);
}

static bool prv_connection_filter_callback(ListNode *found_node, void *data) {
  const PPoGATTClient *client = (const PPoGATTClient *) found_node;
  const GAPLEConnection *connection = (const GAPLEConnection *) data;
  if (client->role == PPoGATTRoleReversed) {
    return (client->rev.connection == connection);
  }
  GAPLEConnection *forward_conn =
      gatt_client_characteristic_get_connection(client->characteristics.meta);
  return (forward_conn == connection);
}

static PPoGATTClient *prv_find_client_for_connection(GAPLEConnection *connection) {
  return (PPoGATTClient *) list_find((ListNode *) s_ppogatt_head,
                                     prv_connection_filter_callback, (void *) connection);
}

// -------------------------------------------------------------------------------------------------

static bool prv_client_filter_callback(ListNode *found_node, void *data) {
  return ((const PPoGATTClient *) found_node == (const PPoGATTClient *) data);
}

static bool prv_is_client_valid(const PPoGATTClient *client) {
  return (list_find((ListNode *)s_ppogatt_head,
                    prv_client_filter_callback, (void *) client) != NULL);
}

// -------------------------------------------------------------------------------------------------

static uint16_t prv_get_max_payload_size(const PPoGATTClient *client) {
  BTDeviceInternal device;
  if (client->role == PPoGATTRoleReversed) {
    if (!client->rev.connection) {
      return 0;
    }
    device = client->rev.connection->device;
  } else {
    device = gatt_client_characteristic_get_device(client->characteristics.data);
  }
  const uint16_t mtu = gap_le_connection_get_gatt_mtu(&device);
  if (mtu < GATT_MTU_MINIMUM) {
    // Device got disconnected in the mean time
    return 0;
  }
  const uint16_t num_bytes_overhead = 3 /* ATT header */ + sizeof(PPoGATTPacket);
  return (mtu - num_bytes_overhead);
}

// -------------------------------------------------------------------------------------------------

static void prv_enter_awaiting_reset_complete(PPoGATTClient *client, bool self_initiated) {
  if (client->state == StateConnectedOpen) {
    // No need to consume the remaining bytes in the SendBuffer, it's CommSession's responsibility
    // to clean up the SendBuffer.
    DeleteReason reason =
        self_initiated ? DeleteReason_ResetSelfInitiated : DeleteReason_ResetRemoteInitiated;
    comm_session_close(client->session, (CommSessionCloseReason)reason);
    client->session = NULL;
  }
  client->in.next_expected_data_sn = 0;
  // FIXME: Use SN for RR / RC (https://pebbletechnology.atlassian.net/browse/PBL-12424)
  client->out = (__typeof__(client->out)) {};

  if (prv_client_supports_enhanced_throughput_features(client)) {
    // Set our desired window sizes
    //
    // Note: as of PBL-38806 (which is in Android 4.0), the Android app will negotiate the MTU size
    // before starting up a PPoG session so we can use this info to dynamically change the window
    // size. The iOS app has no control over when the MTU size is negotiated (though it seems to be
    // negotiated in time) but if we were to use PPoG V1 on iOS it's something we should check

    if (prv_get_max_payload_size(client) < GATT_MTU_MINIMUM) {
      // If a device does not support large a large MTU/payload size, its throughput is severely
      // limited by the window size leading. This prevents us from handling throughput sensitive
      // operations (such as dictation) in time and results in dropped packets. To improve this,
      // negotiate a larger TX Window size so we can get a better data rate.
      client->out.tx_window_size = (PPOGATT_SN_MOD_DIV - 1);
    } else {
      // TODO: For larger MTU sizes, we wind up getting throttled by default_kernel_sender.c
      // because it limits kernel heap space allocated to ~1kB. We may be able to improve App
      // Message throughput by fiddling with this value but at the same time we run a higher risk
      // of blowing up the Dialog Heap if a lot of payloads get queued up on the BT chip. When time
      // allows, we should fiddle with this setting to see if we can optimize outbound throughput
      client->out.tx_window_size = PPOGATT_V0_WINDOW_SIZE;
    }

    const uint8_t desired_rx_window =
        MIN(PPOGATT_V1_DESIRED_RX_WINDOW_SIZE, PPOGATT_SN_MOD_DIV - 1);
    client->out.rx_window_size = desired_rx_window;
  } else {
    client->out.tx_window_size = client->out.rx_window_size = PPOGATT_V0_WINDOW_SIZE;
  }

  if (self_initiated) {
    client->out.reset_packet_to_send.type = PPoGATTPacketTypeResetRequest;
    client->state = StateConnectedClosedAwaitingResetCompleteSelfInitiatedReset;
  } else {
    client->out.reset_packet_to_send.type = PPoGATTPacketTypeResetComplete;
    client->state = StateConnectedClosedAwaitingResetCompleteRemoteInitiatedReset;
  }
  prv_send_next_packets(client);

  // Set a timeout within which we expect to receive the "Reset Complete" message.
  prv_reset_ack_timeout(client);
}

// -------------------------------------------------------------------------------------------------

static void prv_start_reset(PPoGATTClient *client) {
  if (++client->resets_counter >= PPOGATT_RESET_COUNT_MAX) {
    if (client->disconnect_requested) {
      return;
    }

    if (++s_disconnect_counter > PPOGATT_DISCONNECT_COUNT_MAX) {
      // If we have disconnected too many times, do not disconnect and leave the client in a
      // "stalled" state, so that we have the option to "unstall" by sending a remote reset
      client->state = StateConnectedClosedAwaitingResetCompleteSelfInitiatedResetStalled;
      PBL_LOG_WRN("Reset request stalled, not disconnecting");
      return;
    }

    PBL_LOG_ERR("Disconnecting because max resets reached...");

    // Record the time of this disconnect request

    bt_lock();
    GAPLEConnection *connection = prv_get_connection(client);
    PBL_ASSERTN(connection != NULL);
    bt_unlock();

    bt_driver_gap_le_disconnect(&connection->device);
    client->disconnect_requested = true;
  } else {
    prv_enter_awaiting_reset_complete(client, true /* self_initiated */);
  }
}

// -------------------------------------------------------------------------------------------------

static void prv_handle_reset_request(PPoGATTClient *client) {
  if (client->state == StateConnectedClosedAwaitingResetCompleteSelfInitiatedReset) {
    // Already in self-initated reset procedure, client should ignore the request from the server.
    PBL_LOG_INFO("Ignoring reset request because local client already requested.");
    return;
  }
  if (client->state == StateConnectedClosedAwaitingResetCompleteRemoteInitiatedReset) {
    // Already in remote-initiated reset procedure, server retrying?
    // See https://pebbletechnology.atlassian.net/browse/PBL-12424
    PBL_LOG_INFO("Ignoring reset request because remote server already requested.");
    return;
  }
  prv_enter_awaiting_reset_complete(client, false /* self_initiated */);
}

// -------------------------------------------------------------------------------------------------

static void prv_handle_reset_complete(PPoGATTClient *client, const PPoGATTPacket *packet,
                                      uint16_t payload_length) {
  CommSession *session = comm_session_open((Transport *) client,
                                           &s_ppogatt_transport_implementation,
                                           client->destination);
  if (!session) {
    prv_delete_client(client, false /* is_disconnected */, DeleteReason_CouldntOpenCommSession);
    return;
  }

  // Possibly successful disconnect?
  if (s_disconnect_counter) {
  }
  ppogatt_reset_disconnect_counter();
  client->resets_counter = 0;


  if (LIKELY(client->state == StateConnectedClosedAwaitingResetCompleteSelfInitiatedReset)) {
    client->out.reset_packet_to_send = (const PPoGATTPacket) {
      .sn = 0,
      .type = PPoGATTPacketTypeResetComplete,
    };
    prv_send_next_packets(client);
  }
  client->state = StateConnectedOpen;
  client->session = session;

  PBL_ANALYTICS_SET_UNSIGNED(ppog_reversed, (client->role == PPoGATTRoleReversed) ? 1 : 0);

  if (prv_client_supports_enhanced_throughput_features(client)) {
    if (payload_length < sizeof(PPoGATTResetCompleteClientIDPayloadV1)) {
      PBL_LOG_WRN("Unexpected PPoGatt Reset Complete Payload Size: %"PRIu16,
              payload_length);
      // Be defensive, and use the original window size
      client->out.tx_window_size = client->out.rx_window_size = PPOGATT_V0_WINDOW_SIZE;
    } else {
      PPoGATTResetCompleteClientIDPayloadV1 *payload =
          (PPoGATTResetCompleteClientIDPayloadV1 *)&packet->payload[0];
      PBL_LOG_DBG("PPoGATT Remote RxWindow: %d TxWindow %d",
              payload->ppogatt_max_rx_window, payload->ppogatt_max_tx_window);

      client->out.tx_window_size = MIN(client->out.tx_window_size, payload->ppogatt_max_rx_window);
      client->out.rx_window_size = MIN(client->out.rx_window_size, payload->ppogatt_max_tx_window);
    }
  }

  {
    const uint32_t elapsed_ms =
        (rtc_get_ticks() - client->created_ticks) * 1000 / RTC_TICKS_HZ;
    PBL_LOG_INFO("PPoGATT Session is opened (%s, Vers: %d TXW: %d RXW: %d) after %"PRIu32"ms!",
                 (client->role == PPoGATTRoleReversed) ? "reversed" : "forward",
                 client->version, client->out.tx_window_size, client->out.rx_window_size, elapsed_ms);
  }
}

// -------------------------------------------------------------------------------------------------

static void prv_handle_ack(PPoGATTClient *client, uint32_t sn) {
  if (prv_is_packet_with_sn_awaiting_ack(client, sn)) {
    client->out.timeouts_counter = 0;
    client->out.ack_timeout_state = AckTimeoutState_Inactive;

    // Ack'd one of the packets in flight
    const uint32_t next_sn = prv_next_sn(sn);
    const uint16_t num_bytes_acked = prv_total_num_bytes_awaiting_ack_up_to(client, next_sn);
    comm_session_send_queue_consume(client->session, num_bytes_acked);

    // If next_data_sn is before the Ack'd sn, the packet pending retransmission has just been
    // Ack'd. We can determine whether or not a packet is pending retransmission by checking if the
    // payload size for next_data_sn is not 0. This means the packet has been enqueued to get sent
    if (prv_is_packet_with_sn_awaiting_ack(client, client->out.next_data_sn)) {
      client->out.next_data_sn = next_sn;
    }

    // Clear up the payload size(s) for Ack'd packets.
    // See comment with the prv_get_payload_size_for_sn() call in prv_send_next_packets()
    prv_clear_payload_sizes_up_to(client, next_sn);

    client->out.next_expected_ack_sn = next_sn;

    if (prv_get_payload_size_for_sn(client, next_sn) != 0) { // Still awaiting ACKs
      prv_reset_ack_timeout(client);
    }

    prv_send_next_packets(client);
  } else if (sn == prv_prev_sn(client->out.next_expected_ack_sn)) {
    // Data we had sent got dropped causing the other side to re-ACK the last data it had received.
    // Don't roll back directly to avoid creating an Sorcerer's Apprentice bug
    // https://en.wikipedia.org/wiki/Sorcerer%27s_Apprentice_Syndrome
    // We'll rely on the ACK timeout for the next data packet to fire and roll back.
    PBL_LOG_WRN("Received retransmitted Ack for sn:%"PRIu32". Ignoring it.", sn);
  } else {
    PBL_LOG_ERR("Ack'd packet out of range %"PRIu32", [%u-%u].",
            sn, client->out.next_expected_ack_sn, client->out.next_data_sn);
    prv_start_reset(client);
  }
}

// -------------------------------------------------------------------------------------------------

static void prv_handle_data(PPoGATTClient *client,
                            const PPoGATTPacket *packet, uint16_t payload_length) {
  if (client->in.next_expected_data_sn == packet->sn) {
    client->out.ack_packet_to_send = (const PPoGATTPacket) {
      .sn = client->in.next_expected_data_sn,
      .type = PPoGATTPacketTypeAck,
    };
    prv_send_next_packets(client);

    client->in.next_expected_data_sn = prv_next_sn(client->in.next_expected_data_sn);
    comm_session_receive_router_write(client->session, packet->payload, payload_length);
//    PBL_LOG_DBG("Got PP data (sn=%u, %u bytes)", packet->sn, payload_length);
  } else {
    PBL_LOG_DBG("packet->sn != next_expected_data_sn (%u != %u)",
            packet->sn, client->in.next_expected_data_sn);
    // Rely on the server retransmitting on Ack time-out
  }
}

// -------------------------------------------------------------------------------------------------

static void prv_handle_data_notification(PPoGATTClient *client,
                                         const uint8_t *value, uint16_t value_length) {
//  PBL_LOG_DBG("IN:");
//  PBL_HEXDUMP(LOG_LEVEL_DEBUG, value, value_length);

  if (UNLIKELY(value_length == 0)) {
    PBL_LOG_ERR("Zero length packet");
    return;
  }
  const PPoGATTPacket *packet = (const PPoGATTPacket *) value;
  if (UNLIKELY(packet->type >= PPoGATTPacketTypeInvalidRangeStart)) {
    PBL_LOG_ERR("Invalid type %u", packet->type);
    return;
  }
  if (UNLIKELY(packet->type) == PPoGATTPacketTypeResetRequest) {
    PBL_LOG_INFO("Got reset request!");
    prv_handle_reset_request(client);
    return;
  }
  if (LIKELY(client->state == StateConnectedOpen)) {
    if (LIKELY(packet->type == PPoGATTPacketTypeData)) {
      prv_handle_data(client, packet, value_length - sizeof(PPoGATTPacket));
    } else if (LIKELY(packet->type == PPoGATTPacketTypeAck)) {
      prv_handle_ack(client, packet->sn);
    } else if (UNLIKELY(packet->type == PPoGATTPacketTypeResetComplete)) {
      PBL_LOG_ERR("Got reset complete while open!?");
    }
  } else if (client->state == StateConnectedClosedAwaitingResetCompleteSelfInitiatedReset ||
             client->state == StateConnectedClosedAwaitingResetCompleteSelfInitiatedResetStalled ||
             client->state == StateConnectedClosedAwaitingResetCompleteRemoteInitiatedReset) {
    if (LIKELY(packet->type == PPoGATTPacketTypeResetComplete)) {
      prv_handle_reset_complete(client, packet, value_length - sizeof(PPoGATTPacket));
    } else {
      PBL_LOG_DBG("Resetting, ignoring data/ack packets (%u)", packet->type);
    }
  } else {
    PBL_LOG_DBG("Ignoring all packets in state %u", client->state);
  }
}

// -------------------------------------------------------------------------------------------------

static void prv_rediscover_kernelbg_cb(void *data) {
  BTDeviceInternal *device = (BTDeviceInternal *)data;
  const BTErrno e = gatt_client_discovery_rediscover_all(device);
  kernel_free(device);
  if (e != BTErrnoOK) {
    PBL_LOG_ERR("PPoGATT couldn't restart discovery: %d", (int)e);
  }
}

static void prv_request_meta_rediscovery(PPoGATTClient *client) {
  bt_lock_assert_held(true);

  GAPLEConnection *connection =
      gatt_client_characteristic_get_connection(client->characteristics.meta);
  if (!connection) {
    return;
  }

  BTDeviceInternal *device = kernel_malloc(sizeof(BTDeviceInternal));
  if (!device) {
    return;
  }
  *device = connection->device;
  system_task_add_callback(prv_rediscover_kernelbg_cb, device);
}

static void prv_retry_meta_read(PPoGATTClient *client) {
  bt_lock_assert_held(true);

  if (!client || client->state != StateDisconnectedAwaitingMetaRetry) {
    return;
  }

  PBL_LOG_INFO("Retrying PPoGATT meta read (attempt %u/%u)",
          client->meta_read_retries + 1, PPOGATT_META_READ_RETRY_COUNT_MAX);

  client->state = StateDisconnectedReadingMeta;
  BLECharacteristic meta = client->characteristics.meta;

  // Release bt_lock before calling gatt_client_op_read to avoid recursive lock deadlock.
  // gatt_client_op_read manages bt_lock internally.
  bt_unlock();

  BTErrno result = gatt_client_op_read(meta, GAPLEClientKernel);

  // Re-acquire bt_lock before accessing client state again
  bt_lock();

  if (result != BTErrnoOK) {
    // Read failed to start, delete the client
    PBL_LOG_ERR("Failed to initiate meta read retry");
    prv_delete_client(client, false /* is_disconnected */, DeleteReason_MetaDataReadFailure);
  }
}

static void prv_meta_read_retry_kernelmain_cb(void *data) {
  PPoGATTClient *client = (PPoGATTClient *)data;
  bt_lock();
  {
    // make sure we didn't disconnect in between
    if (prv_is_client_valid(client)) {
      prv_retry_meta_read(client);
    }
  }
  bt_unlock();
}

//! Never block on bt_lock from the timer task; bounce to KernelMain
//! (prv_retry_meta_read also calls into NimBLE, which doesn't belong on the timer task).
static void prv_meta_read_retry_timer_cb(void *data) {
  launcher_task_add_callback(prv_meta_read_retry_kernelmain_cb, data);
}

// -------------------------------------------------------------------------------------------------

static void prv_handle_meta_read(PPoGATTClient *client, const uint8_t *value,
                                 size_t value_length, BLEGATTError error) {
  const uint32_t elapsed_ms = (rtc_get_ticks() - client->created_ticks) * 1000 / RTC_TICKS_HZ;
  PBL_ASSERTN(client->state == StateDisconnectedReadingMeta);
  if (error != BLEGATTErrorSuccess) {
    PBL_LOG_ERR("PPoGATT meta read failed: err=0x%x after %"PRIu32"ms (retry %u/%u)",
            error, elapsed_ms, client->meta_read_retries, PPOGATT_META_READ_RETRY_COUNT_MAX);
    // GATT read failed - this is retriable since the mobile app may not be ready yet
    goto handle_retriable_error;
  }
  PBL_LOG_INFO("PPoGATT meta read success after %"PRIu32"ms, len=%u", elapsed_ms,
               (unsigned int)value_length);
  if (value_length < sizeof(PPoGATTMetaV0)) {
    goto handle_error;
  }
  const PPoGATTMetaV0 *meta = (const PPoGATTMetaV0 *) value;
  if (meta->ppogatt_min_version > PPOGATT_MAX_VERSION
      /* || meta->ppogatt_max_version < PPOGATT_MIN_VERSION  // always true at the moment */) {
    goto handle_error;
  }
  if (uuid_is_invalid(&meta->app_uuid)) {
    PBL_LOG_ERR("Invalid UUID");
    goto handle_error;
  }
#ifdef CONFIG_RECOVERY_FW
  if (!uuid_is_system(&meta->app_uuid)) {
    PBL_LOG_ERR("Found PPoGATT server from non-Pebble app, not connecting in PRF..");
    goto handle_error;
  }
#endif

  // Use the highest version that both ends support:
  client->version = MIN(meta->ppogatt_max_version, PPOGATT_MAX_VERSION);

  // Parse additional v1 metadata fields:
  PPoGATTSessionType session_type = PPoGATTSessionType_InferredFromUuid;
  if (/*meta->ppogatt_max_version >= 0x01 &&*/ value_length >= sizeof(PPoGATTMetaV1)) {
    const PPoGATTMetaV1 *meta_v1 = (const PPoGATTMetaV1 *)value;
    if (meta_v1->pp_session_type >= PPoGATTSessionTypeCount) {
      PBL_LOG_ERR("Invalid session type %u", meta_v1->pp_session_type);
      goto handle_error;
    }
    session_type = meta_v1->pp_session_type;
  }

  // Save data needed across the bt_lock release
  BLECharacteristic data_char = client->characteristics.data;
  Uuid app_uuid = meta->app_uuid;

  // Release bt_lock before calling gatt_client_subscriptions_subscribe, which
  // eventually calls into NimBLE. Holding bt_lock when calling into NimBLE
  // creates a lock ordering deadlock with ble_hs_mutex (NimBLE store callbacks
  // acquire bt_lock while holding ble_hs_mutex).
  // See prv_retry_meta_read for the same pattern.
  bt_unlock();

  BTErrno e = gatt_client_subscriptions_subscribe(data_char,
                                                  BLESubscriptionNotifications,
                                                  GAPLEClientKernel);

  // Re-acquire bt_lock before accessing client state
  bt_lock();

  // Re-validate client pointer after releasing lock, as it may have been removed
  bool is_data_unused;
  client = prv_find_client_with_characteristic(data_char, &is_data_unused);
  if (!client) {
    return;
  }

  if (e == BTErrnoOK) {
    // Delete any existing client with this UUID, last one wins.
    // iOS behavior is a bit strange when it comes to service persistence. When an app crashes or
    // gets killed through Xcode, the service records persist. When the app is relaunched again,
    // a new service will get added again. The old one remains when it was killed through Xcode
    // before. The old one seems to go away *after* the new one gets added in the crash scenario.
    PPoGATTClient *existing_client = prv_find_client_with_uuid(&app_uuid);
    if (existing_client) {
      PBL_LOG_ERR("Found PPoGATT server with same UUID. Keeping only the last one.");
      prv_delete_client(existing_client, true /* is_disconnected */, DeleteReason_DuplicateServer);
    }
    client->state = StateDisconnectedSubscribingData;
    client->app_uuid = app_uuid;
    // Reset retry counter on success
    client->meta_read_retries = 0;

    if (session_type == PPoGATTSessionType_Hybrid) {
      client->destination = TransportDestinationHybrid;
    } else {  // (session_type == PPoGATTSessionType_InferredFromUuid)
      const bool is_system = uuid_is_system(&client->app_uuid);
      client->destination = is_system ? TransportDestinationSystem : TransportDestinationApp;
    }
    return;
  }

handle_retriable_error:
  // GATT read failed - schedule a retry if we haven't exceeded the max retry count
  if (++client->meta_read_retries < PPOGATT_META_READ_RETRY_COUNT_MAX) {
    PBL_LOG_WRN("PPoGATT meta read failed (err=%x), scheduling retry %u/%u",
            error, client->meta_read_retries, PPOGATT_META_READ_RETRY_COUNT_MAX);
    client->state = StateDisconnectedAwaitingMetaRetry;
    new_timer_start(client->rx_ack_timer, PPOGATT_META_READ_RETRY_DELAY_MS,
                    prv_meta_read_retry_timer_cb, client, 0);
    return;
  }
  PBL_LOG_ERR("PPoGATT meta read failed after %u retries",
          PPOGATT_META_READ_RETRY_COUNT_MAX);

  // Cached handle may be stale; try once with fresh discovery.
  if (!s_rediscovery_requested_this_connection) {
    PBL_LOG_INFO("Triggering GATT rediscovery for fresh PPoGATT handles");
    s_rediscovery_requested_this_connection = true;
    prv_request_meta_rediscovery(client);
  }

handle_error:
  PBL_LOG_ERR("Failed handling PPoGATT meta: len=%u ver=%x err=%x",
          (unsigned int) value_length, value_length ? value[0] : ~0, error);
  prv_delete_client(client, false /* is_disconnected */, DeleteReason_MetaDataInvalid);
}


// -------------------------------------------------------------------------------------------------

void ppogatt_create(void) {
  bt_lock();
  {
    PBL_ASSERT_TASK(PebbleTask_KernelMain);
    PBL_ASSERTN(!s_ppogatt_head);
    s_timer_ticks = 0;
    s_rediscovery_requested_this_connection = false;
  }
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------

void ppogatt_handle_service_removed(
    BLECharacteristic *characteristics, uint8_t num_characteristics) {
  bt_lock();
  {
    bool client_removed = false;

    // Delete existing clients:
    PPoGATTClient *client = s_ppogatt_head;
    while (client) {
      PPoGATTClient *next = (PPoGATTClient *) client->node.next;
      for (int i = 0; i < num_characteristics; i++) {
        if (client->characteristics.meta == characteristics[i] ||
            client->characteristics.data == characteristics[i]) {
          client_removed = true;
          prv_delete_client(client, true, DeleteReason_ServiceRemoved);
          break;
        }
      }
      client = next;
    }

    // PBL-42768 - In the logs in this ticket it looks to me like we missed that the service
    // was removed. Add some diagnostic logging to hopefully reveal more info on a failure
    if (!client_removed) {
      BLECharacteristic meta = 0;
      BLECharacteristic data = 0;

      // assume one client
      PPoGATTClient *client = s_ppogatt_head;
      if (client != NULL) {
        meta = client->characteristics.meta;
        data = client->characteristics.data;
      }

      BLECharacteristic char1 = num_characteristics > 0 ? characteristics[0] : 0;
      BLECharacteristic char2 = num_characteristics > 1 ? characteristics[1] : 0;

      PBL_LOG_WRN("No ppog client removed? 0x%x 0x%x vs 0x%x 0x%x",
              (int)meta, (int)data, (int)char1, (int)char2);
    }
  }
  bt_unlock();
}

void ppogatt_invalidate_all_references(void) {
  bt_lock();
  {
    PPoGATTClient *client = s_ppogatt_head;
    while (client) {
      PPoGATTClient *next = (PPoGATTClient *) client->node.next;
      prv_delete_client(client, true /* is_disconnected */, DeleteReason_InvalidateAllReferences);
      client = next;
    }
  }
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------
// Reversed PPoG (watch hosts the GATT service)

static bool prv_reversed_start(GAPLEConnection *connection, uint16_t conn_handle,
                               TimerID rx_ack_timer, TimerID send_retry_timer) {
  bt_lock_assert_held(true);

  // Reversed wins over any client already bound to this connection.
  PPoGATTClient *existing = prv_find_client_for_connection(connection);
  if (existing) {
    PBL_LOG_INFO("Reversed PPoG taking over from existing client (role=%u)", existing->role);
    prv_delete_client(existing, false /* is_disconnected */, DeleteReason_DuplicateServer);
  }

  PPoGATTClient *client = prv_create_client(rx_ack_timer, send_retry_timer);
  if (!client) {
    return false;
  }
  client->role = PPoGATTRoleReversed;
  client->rev.conn_handle = conn_handle;
  client->rev.connection = connection;
  client->app_uuid = UUID_INVALID;
  client->version = PPOGATT_MAX_VERSION;
  client->destination = TransportDestinationHybrid;

  s_reversed_active_conn = connection;

  // The handshake is phone-initiated: wait for the phone's ResetRequest.
  client->state = StateConnectedClosedAwaitingResetRequest;
  return true;
}

void ppogatt_reversed_handle_subscribed(const BTDeviceInternal *device, uint16_t conn_handle) {
  // new_timer_create() must be called outside bt_lock (see ppogatt_handle_service_discovered).
  TimerID rx_ack_timer = new_timer_create();
  PBL_ASSERTN(rx_ack_timer);
  TimerID send_retry_timer = new_timer_create();
  PBL_ASSERTN(send_retry_timer);

  bool started = false;
  bt_lock();
  {
    GAPLEConnection *connection = gap_le_connection_by_device(device);
    if (!connection) {
      PBL_LOG_WRN("Reversed PPoG subscribed: no connection for device");
      goto unlock;
    }
    // Backstop for the driver-side check: never talk PP on an unencrypted link.
    if (!connection->is_encrypted) {
      PBL_LOG_WRN("Reversed PPoG subscribed: connection not encrypted; ignoring");
      goto unlock;
    }
    PBL_LOG_INFO("Reversed PPoG subscribed (conn_handle=%u)", conn_handle);
    started = prv_reversed_start(connection, conn_handle, rx_ack_timer, send_retry_timer);
  }
unlock:
  bt_unlock();
  if (!started) {
    new_timer_delete(rx_ack_timer);
    new_timer_delete(send_retry_timer);
  }
}

void ppogatt_reversed_handle_unsubscribed(uint16_t conn_handle) {
  bt_lock();
  {
    PPoGATTClient *client = prv_find_reversed_client_for_conn(conn_handle);
    if (client) {
      PBL_LOG_INFO("Reversed PPoG unsubscribed (conn_handle=%u)", conn_handle);
      prv_delete_client(client, false /* is_disconnected */, DeleteReason_CloseCalled);
    }
  }
  bt_unlock();
}

void ppogatt_reversed_handle_data(uint16_t conn_handle, const uint8_t *buf, size_t len) {
  bt_lock();
  {
    PPoGATTClient *client = prv_find_reversed_client_for_conn(conn_handle);
    if (!client) {
      PBL_LOG_DBG("Reversed PPoG data for unknown conn_handle=%u", conn_handle);
      goto unlock;
    }
    prv_handle_data_notification(client, buf, (uint16_t) len);
  }
unlock:
  bt_unlock();
}

// The bt_driver_cb_* callbacks below defer to KernelMain: the NimBLE host-task
// stack is too small to run the PPoG state machine directly, and KernelBG
// callbacks may block waiting for Pebble Protocol send progress (e.g. the BT
// log dump), which would starve inbound ACK processing and deadlock the
// transport. KernelMain is where forward-mode RX and send_next jobs already
// run.

typedef struct {
  BTDeviceInternal device;
  uint16_t conn_handle;
} ReversedSubscribedCtx;

typedef struct {
  uint16_t conn_handle;
} ReversedUnsubscribedCtx;

typedef struct {
  uint16_t conn_handle;
  uint16_t len;
  uint8_t *buf;  // owned, freed by the KernelMain cb.
} ReversedDataCtx;

static void prv_reversed_subscribed_kernelmain_cb(void *data) {
  ReversedSubscribedCtx *ctx = (ReversedSubscribedCtx *)data;
  ppogatt_reversed_handle_subscribed(&ctx->device, ctx->conn_handle);
  kernel_free(ctx);
}

static void prv_reversed_unsubscribed_kernelmain_cb(void *data) {
  ReversedUnsubscribedCtx *ctx = (ReversedUnsubscribedCtx *)data;
  ppogatt_reversed_handle_unsubscribed(ctx->conn_handle);
  kernel_free(ctx);
}

static void prv_reversed_data_kernelmain_cb(void *data) {
  ReversedDataCtx *ctx = (ReversedDataCtx *)data;
  ppogatt_reversed_handle_data(ctx->conn_handle, ctx->buf, ctx->len);
  kernel_free(ctx->buf);
  kernel_free(ctx);
}

void bt_driver_cb_ppog_reversed_subscribed(const BTDeviceInternal *device, uint16_t conn_handle) {
  ReversedSubscribedCtx *ctx = kernel_malloc(sizeof(*ctx));
  if (!ctx) {
    PBL_LOG_ERR("Reversed PPoG subscribed: out of memory");
    return;
  }
  ctx->device = *device;
  ctx->conn_handle = conn_handle;
  launcher_task_add_callback(prv_reversed_subscribed_kernelmain_cb, ctx);
}

void bt_driver_cb_ppog_reversed_unsubscribed(uint16_t conn_handle) {
  ReversedUnsubscribedCtx *ctx = kernel_malloc(sizeof(*ctx));
  if (!ctx) {
    PBL_LOG_ERR("Reversed PPoG unsubscribed: out of memory");
    return;
  }
  ctx->conn_handle = conn_handle;
  launcher_task_add_callback(prv_reversed_unsubscribed_kernelmain_cb, ctx);
}

void bt_driver_cb_ppog_reversed_data_written(uint16_t conn_handle,
                                             uint8_t *buf, uint16_t len) {
  ReversedDataCtx *ctx = kernel_malloc(sizeof(*ctx));
  if (!ctx) {
    PBL_LOG_ERR("Reversed PPoG data: out of memory");
    kernel_free(buf);
    return;
  }
  ctx->conn_handle = conn_handle;
  ctx->len = len;
  ctx->buf = buf;
  launcher_task_add_callback(prv_reversed_data_kernelmain_cb, ctx);
}

// -------------------------------------------------------------------------------------------------

void ppogatt_handle_service_discovered(BLECharacteristic *characteristics) {
  PBL_LOG_INFO("PPoGATT service discovered, starting handshake");

  // Create timers outside of bt_lock to avoid deadlock with NimbleHost.
  // new_timer_create() acquires TaskTimerManager mutex, which may be held by NimbleHost
  // when it's trying to acquire bt_lock, leading to a lock ordering deadlock.
  TimerID rx_ack_timer = new_timer_create();
  PBL_ASSERTN(rx_ack_timer);
  TimerID send_retry_timer = new_timer_create();
  PBL_ASSERTN(send_retry_timer);

  bt_lock();
  {
    // Reversed PPoG has priority: ignore the forward service if reversed is
    // active on this connection.
    GAPLEConnection *meta_conn =
        gatt_client_characteristic_get_connection(characteristics[PPoGATTCharacteristicMeta]);
    if (meta_conn && s_reversed_active_conn == meta_conn) {
      bt_unlock();
      new_timer_delete(rx_ack_timer);
      new_timer_delete(send_retry_timer);
      PBL_LOG_DBG("Reversed PPoG active, skipping forward");
      return;
    }

    // Create new clients:
    PPoGATTClient *client = prv_create_client(rx_ack_timer, send_retry_timer);
    if (!client) {
      bt_unlock();
      new_timer_delete(rx_ack_timer);
      new_timer_delete(send_retry_timer);
      return;
    }
    client->role = PPoGATTRoleForward;
    BLECharacteristic meta = characteristics[PPoGATTCharacteristicMeta];
    client->characteristics.meta = characteristics[PPoGATTCharacteristicMeta];
    client->characteristics.data = characteristics[PPoGATTCharacteristicData];

    PBL_LOG_DBG("PPoGATT reading meta characteristic");

    // Release bt_lock before calling gatt_client_op_read to avoid recursive lock deadlock.
    // gatt_client_op_read manages bt_lock internally.
    bt_unlock();

    BTErrno result = gatt_client_op_read(meta, GAPLEClientKernel);

    // Re-acquire bt_lock before accessing client state again
    bt_lock();

    if (result != BTErrnoOK) {
      PBL_LOG_ERR("PPoGATT meta read initiation failed: err=0x%x", result);
      // Read failed, probably disconnected or insufficient resources
      prv_delete_client(client, false /* is_disconnected */, DeleteReason_MetaDataReadFailure);
    }
  }
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------

bool ppogatt_can_handle_characteristic(BLECharacteristic characteristic) {
  bt_lock();
  bool can_handle = (prv_find_client_with_characteristic(characteristic, NULL) != NULL);
  bt_unlock();
  return can_handle;
}

// -------------------------------------------------------------------------------------------------

void ppogatt_handle_subscribe(BLECharacteristic characteristic,
                              BLESubscription subscription_type, BLEGATTError error) {
  bt_lock();
  {
    const bool is_subscribed = (subscription_type != BLESubscriptionNone);
    PPoGATTClient *client = prv_find_client_with_characteristic(characteristic, NULL);
    if (!client && is_subscribed) {
      PBL_LOG_ERR("PPoGATT Client could be found, unsubscribing");
      // Attempt to unsubscribe to avoid wasting bandwidth.
      // Release bt_lock before calling into NimBLE to avoid deadlock with ble_hs_mutex.
      bt_unlock();
      gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionNone, GAPLEClientKernel);
      bt_lock();
      goto unlock;
    }
    PBL_ASSERTN(client->state == StateDisconnectedSubscribingData);
    if (error) {
      const uint32_t elapsed_ms =
          (rtc_get_ticks() - client->created_ticks) * 1000 / RTC_TICKS_HZ;
      PBL_LOG_ERR("PPoGATT subscribe failed: err=0x%x after %"PRIu32"ms", error, elapsed_ms);
      prv_delete_client(client, false /* is_disconnected */, DeleteReason_SubscribeFailure);
      goto unlock;
    }
    if (!is_subscribed) {
      // Unsubscribed due to removed client
      goto unlock;
    }
    {
      const uint32_t elapsed_ms =
          (rtc_get_ticks() - client->created_ticks) * 1000 / RTC_TICKS_HZ;
      PBL_LOG_INFO("PPoGATT subscribed to Data after %"PRIu32"ms, sending ResetRequest",
              elapsed_ms);
    }
    prv_start_reset(client);
  }
unlock:
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------

void ppogatt_handle_read_or_notification(BLECharacteristic characteristic, const uint8_t *value,
                                         size_t value_length, BLEGATTError error) {
  bt_lock();
  {
    bool is_data = false;
    PPoGATTClient *client = prv_find_client_with_characteristic(characteristic, &is_data);
    if (!client) {
      PBL_LOG_DBG("Got notification/read for unknown client");
      goto unlock;
    }
    if (is_data) {
      prv_handle_data_notification(client, value, value_length);
    } else {
      prv_handle_meta_read(client, value, value_length, error);
    }
  }
unlock:
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------

static PPoGATTPacket * prv_lazily_allocate_packet_if_needed(const PPoGATTClient *client,
                                                            PPoGATTPacket **heap_packet_in_out) {
  PPoGATTPacket *packet = *heap_packet_in_out;
  // Lazily allocate a buffer on the heap:
  if (!packet) {
    const uint16_t max_payload_size = prv_get_max_payload_size(client);
    if (max_payload_size == 0) {
      return NULL;
    }
    packet = (PPoGATTPacket *) kernel_malloc_check(sizeof(PPoGATTPacket) + max_payload_size);
    *heap_packet_in_out = packet;
  }
  return packet;
}

// -------------------------------------------------------------------------------------------------

static const PPoGATTPacket * prv_prepare_next_reset_packet(const PPoGATTClient *client,
                                                      PPoGATTPacket **heap_packet_in_out,
                                                      uint16_t *payload_size_out) {
  PPoGATTPacket *packet = prv_lazily_allocate_packet_if_needed(client, heap_packet_in_out);
  if (!packet) {
    PBL_LOG_WRN("Couldn't allocate reset packet: role=%u state=%u type=%u",
                client->role, client->state,
                (unsigned) client->out.reset_packet_to_send.type);
    return NULL;
  }

  if (client->out.reset_packet_to_send.type == PPoGATTPacketTypeResetRequest) {
    // Reset Request packet:
    *packet = (const PPoGATTPacket) {
      .type = PPoGATTPacketTypeResetRequest,
      .sn = 0,
    };
    PPoGATTResetRequestClientIDPayload *client_id_payload =
          (PPoGATTResetRequestClientIDPayload *) packet->payload;
    *client_id_payload = (const PPoGATTResetRequestClientIDPayload) {
      .ppogatt_version = client->version,
    };
    memcpy(client_id_payload->serial_number, mfg_get_serial_number(),
           sizeof(client_id_payload->serial_number));
    *payload_size_out = sizeof(*client_id_payload);
    return packet;
  } else { // PPoGATTPacketTypeResetComplete
    if (prv_client_supports_enhanced_throughput_features(client)) {
      *packet = (const PPoGATTPacket) {
        .sn = 0,
        .type = PPoGATTPacketTypeResetComplete,
      };
      PPoGATTResetCompleteClientIDPayloadV1 *client_id_payload =
          (PPoGATTResetCompleteClientIDPayloadV1 *) packet->payload;
      *client_id_payload = (const PPoGATTResetCompleteClientIDPayloadV1) {
        .ppogatt_max_rx_window = client->out.rx_window_size,
        .ppogatt_max_tx_window = client->out.tx_window_size,
      };
      *payload_size_out = sizeof(*client_id_payload);
      return packet;
    } else {
      // Reset Complete packet (zero payload size):
      *payload_size_out = 0;
      return &client->out.reset_packet_to_send;
    }
  }
}

// -------------------------------------------------------------------------------------------------

static void prv_rx_ack_kernelmain_cb(void *data) {
  PPoGATTClient *client = (PPoGATTClient *)data;
  bt_lock();
  {
    // make sure we didn't disconnect in between
    if (prv_is_client_valid(client)) {
      client->out.send_rx_ack_now = true;
      prv_send_next_packets_async(client);
    }
  }
  bt_unlock();
}

//! Never block on bt_lock from the timer task; bounce to KernelMain.
static void rx_ack_timer_cb(void *data) {
  launcher_task_add_callback(prv_rx_ack_kernelmain_cb, data);
}

//! Pumps prv_send_next_packets directly (not via prv_send_next_packets_async)
//! because the session may not exist yet: a Reset Complete send can hit
//! ENOMEM during the handshake.
static void prv_send_retry_kernelmain_cb(void *data) {
  PPoGATTClient *client = (PPoGATTClient *)data;
  bt_lock();
  {
    // make sure we didn't disconnect in between
    if (prv_is_client_valid(client)) {
      prv_send_next_packets(client);
    }
  }
  bt_unlock();
}

//! Never send from the timer task (NimBLE calls don't belong on it); bounce
//! to KernelMain, where the rest of the PPoG state machine runs.
static void prv_send_retry_timer_cb(void *data) {
  launcher_task_add_callback(prv_send_retry_kernelmain_cb, data);
}

static const PPoGATTPacket * prv_prepare_next_packet(PPoGATTClient *client,
                                                     PPoGATTPacket **heap_packet_in_out,
                                                     uint16_t *payload_size_out) {
  if (client->out.reset_packet_byte != 0) {
    return prv_prepare_next_reset_packet(client, heap_packet_in_out, payload_size_out);
  } else if (client->out.ack_packet_byte != 0) {

    if (!prv_client_supports_enhanced_throughput_features(client)) {
      client->out.send_rx_ack_now = true;
    } else {
      client->out.outstanding_rx_ack_count++;
      if (client->out.outstanding_rx_ack_count >= (client->out.rx_window_size / 2)) {
        // we want to ACK data before the other side is blocking waiting for an ACK
        client->out.send_rx_ack_now = true;
      }
    }

    if (client->out.send_rx_ack_now) {
      if (new_timer_scheduled(client->rx_ack_timer, NULL)) {
        new_timer_stop(client->rx_ack_timer);
      }

      // Ack packet (zero payload size):
      *payload_size_out = 0;
      return &client->out.ack_packet_to_send;
    }

    if (!new_timer_scheduled(client->rx_ack_timer, NULL)) {
      new_timer_start(client->rx_ack_timer, PPOGATT_MAX_DATA_ACK_LATENCY_MS,
                      rx_ack_timer_cb, client, 0);
    }
    // We will defer sending the Ack for now, fallthrough and send data instead
  }

  // Data packets:
  if (client->state != StateConnectedOpen) {
    return NULL;
  };
  if (prv_num_packets_in_flight(client) >= client->out.tx_window_size) {
    // Max number of data packets in flight, try again when we got some of them Ack'd.
    return NULL;
  }
  uint16_t read_space = comm_session_send_queue_get_length(client->session);
  if (read_space == 0) {
    return NULL;
  }

  const uint16_t max_payload_size = prv_get_max_payload_size(client);
  if (!max_payload_size) {
    return NULL;
  }

  // Bytes that are awaiting an Ack, have already been handed to Bluetopia, but are still
  // sitting in the send buffer, until they are Ack'd in case we need to retransmit them.
  uint16_t offset = prv_total_num_bytes_awaiting_ack(client);

  // If retransmitting, we need to use the same fragmentation as the previous transmission.
  // The payload_sizes field will still contain the previously used size, unless it was zero'ed
  // out because it got Ack'd.
  uint16_t payload_size = prv_get_payload_size_for_sn(client, client->out.next_data_sn);
  if (payload_size == 0) {
    PBL_ASSERTN(read_space >= offset);
    payload_size = read_space - offset;

    if (payload_size == 0) {
      // No data to send
      return NULL;
    }

    // Cap to the size that the GATT MTU allows:
    payload_size = MIN(payload_size, max_payload_size);
  }

  PPoGATTPacket *packet = prv_lazily_allocate_packet_if_needed(client, heap_packet_in_out);
  if (!packet) {
    return NULL;
  }
  packet->type = PPoGATTPacketTypeData;
  packet->sn = client->out.next_data_sn;
  PBL_ASSERTN(comm_session_send_queue_copy(client->session, offset,
                                           payload_size, packet->payload));
  *payload_size_out = payload_size;
  return packet;
}

// -------------------------------------------------------------------------------------------------

static void prv_finalize_queued_packet(PPoGATTClient *client, uint16_t payload_size) {
  if (client->out.reset_packet_byte != 0) {
    client->out.reset_packet_byte = 0;
  } else if (client->out.send_rx_ack_now && client->out.ack_packet_byte != 0) {
    client->out.ack_packet_byte = 0;
    client->out.send_rx_ack_now = false;
    client->out.outstanding_rx_ack_count = 0;
  } else { // we are sending a data packet
    const uint32_t sn = client->out.next_data_sn;
    prv_set_payload_size_for_sn(client, sn, payload_size);
    if (client->out.ack_timeout_state == AckTimeoutState_Inactive) {
      prv_reset_ack_timeout(client); // Enable timeout if we don't already have it set
    }
    client->out.next_data_sn = prv_next_sn(sn);
  }
}

// -------------------------------------------------------------------------------------------------

static void prv_send_next_packets(PPoGATTClient *client) {
  if (client->send_in_progress) {
    // Re-entered while the outer call dropped bt_lock; reschedule so queued
    // data still gets flushed after the outer call exits.
    if (client->session) {
      comm_session_send_next(client->session);
    }
    return;
  }
  client->send_in_progress = true;

  uint16_t payload_size = 0;
  const PPoGATTPacket *packet = NULL;
  PPoGATTPacket *heap_packet = NULL;

  // Cap the number of times we loop here, to avoid blocking the task for too long.
  uint8_t loop_count = 0;
  while ((packet = prv_prepare_next_packet(client, &heap_packet, &payload_size))) {
    ++loop_count;

    if (client->role == PPoGATTRoleReversed) {
      const uint16_t total_len = sizeof(PPoGATTPacket) + payload_size;
      // Release bt_lock before calling into NimBLE to avoid deadlock with ble_hs_mutex.
      const bool lock_was_held = bt_lock_is_held();
      if (lock_was_held) {
        bt_unlock();
      }
      const BTErrno e = bt_driver_ppog_reversed_notify(client->rev.conn_handle,
                                                       (const uint8_t *) packet, total_len);
      if (lock_was_held) {
        bt_lock();
      }
      if (e == BTErrnoNotEnoughResources) {
        // Out of mbufs (e.g. inbound flood during a firmware update). NimBLE
        // has no "buffers freed" event, so retry after a short delay.
        if (client->send_retry_count++ == 0) {
          PBL_LOG_WRN("Reversed PPoG out of buffers, retrying");
        }
        if (!new_timer_scheduled(client->send_retry_timer, NULL)) {
          new_timer_start(client->send_retry_timer, PPOGATT_SEND_RETRY_DELAY_MS,
                          prv_send_retry_timer_cb, client, 0);
        }
        break;
      } else if (e != BTErrnoOK) {
        PBL_LOG_ERR("Reversed PPoG notify failed %i", e);
        break;
      }
      if (client->send_retry_count) {
        PBL_LOG_INFO("Reversed PPoG send recovered after %u retries", client->send_retry_count);
        client->send_retry_count = 0;
      }
      prv_finalize_queued_packet(client, payload_size);

      const uint8_t max_loop_count = 10;
      if (loop_count > max_loop_count) {
        prv_send_next_packets_async(client);
        break;
      }
      continue;
    }

    // Get the connection and handle while holding bt_lock
    GAPLEConnection *connection;
    uint16_t att_handle;
    bool lock_was_held = bt_lock_is_held();

    if (lock_was_held) {
      att_handle = gatt_client_characteristic_get_handle_and_connection(
          client->characteristics.data, &connection);
      if (!att_handle) {
        // Invalid characteristic, bail out
        break;
      }
      // Release bt_lock BEFORE calling into NimBLE to avoid deadlock.
      // See comment in gatt_client_op_write_without_response.
      bt_unlock();
    } else {
      // If bt_lock wasn't held, we can call gatt_client_op_write_without_response directly
      // (it will manage the lock itself)
      const BTErrno e = gatt_client_op_write_without_response(client->characteristics.data,
                                                              (const uint8_t *) packet,
                                                              sizeof(PPoGATTPacket) + payload_size,
                                                              GAPLEClientKernel);
      if (e == BTErrnoNotEnoughResources) {
        // Need to wait for "Buffer Empty" event (see ppogatt_handle_buffer_empty)
        break;
      } else if (e != BTErrnoOK) {
        // Most likely the LE connection got busted, don't think retrying will help.
        PBL_LOG_ERR("Write failed %i", e);
        break;
      } else {
        // Packet successfully queued
        prv_finalize_queued_packet(client, payload_size);
      }

      const uint8_t max_loop_count = 10;
      if (loop_count > max_loop_count) {
        prv_send_next_packets_async(client);
        break;
      }
      continue;
    }

    // Call into NimBLE without holding bt_lock
    const BTErrno e = bt_driver_gatt_write_without_response(connection, (const uint8_t *) packet,
                                                            sizeof(PPoGATTPacket) + payload_size,
                                                            att_handle);

    // Re-acquire bt_lock before accessing client state
    bt_lock();

    if (e == BTErrnoNotEnoughResources) {
      // Need to wait for "Buffer Empty" event (see ppogatt_handle_buffer_empty)
      break;
    } else if (e != BTErrnoOK) {
      // Most likely the LE connection got busted, don't think retrying will help.
      PBL_LOG_ERR("Write failed %i", e);
      break;
    } else {
//     PBL_LOG_DBG("OUT:");
//     PBL_HEXDUMP(LOG_LEVEL_DEBUG, (const uint8_t *) packet, sizeof(PPoGATTPacket) + payload_size);

      // Packet successfully queued
      prv_finalize_queued_packet(client, payload_size);
    }

    const uint8_t max_loop_count = 10;
    if (loop_count > max_loop_count) {
      // If more bytes left to send (but loop_count became >= 10),
      // schedule a callback to process them later to avoid blocking the task for too long:
      prv_send_next_packets_async(client);
      break;
    }
  }

  kernel_free(heap_packet);

  client->send_in_progress = false;
}

// -------------------------------------------------------------------------------------------------

void ppogatt_handle_buffer_empty(void) {
  bt_lock();
  {
    // FIXME: How to avoid one client using up all the buffer space all the time?
    PPoGATTClient *client = s_ppogatt_head;
    while (client) {
      prv_send_next_packets(client);
      client = (PPoGATTClient *) client->node.next;
    }
  }
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------

// To be called by comm_session after enqueuing new data into the SendBuffer
// bt_lock() must be held before calling
void ppogatt_send_next(Transport *transport) {
  bt_lock_assert_held(true);
  PPoGATTClient *client = (PPoGATTClient *) transport;
  if (!prv_is_client_valid(client)) {
    // Client became invalid in the mean time
    return;
  }
  prv_send_next_packets(client);
}

// -------------------------------------------------------------------------------------------------

void ppogatt_close(struct Transport *transport) {
  bt_lock_assert_held(true);
  PPoGATTClient *client = (PPoGATTClient *) transport;
  prv_delete_client(client, false /* is_disconnected */, DeleteReason_CloseCalled);
}

// -------------------------------------------------------------------------------------------------

void ppogatt_reset(struct Transport *transport) {
  PPoGATTClient *client = (PPoGATTClient *) transport;
  bt_lock();
  {
    if (!prv_is_client_valid(client)) {
      // Client became invalid in the mean time
      goto unlock;
    }
    prv_start_reset(client);
  }
unlock:
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------

void ppogatt_destroy(void) {
  bool self_initiated_disconnect = false;
  bt_lock();
  {
    PPoGATTClient *client = s_ppogatt_head;
    while (client) {
      PPoGATTClient *next = (PPoGATTClient *) client->node.next;
      self_initiated_disconnect = self_initiated_disconnect || client->disconnect_requested;
      prv_delete_client(client, true /* is_disconnected */, DeleteReason_DestroyCalled);
      client = next;
    }
    s_rediscovery_requested_this_connection = false;
  }
  bt_unlock();

  // Only reset the disconnect counter if this was an external disconnect (e.g. RF interference,
  // phone out of range). If we initiated the disconnect ourselves due to max resets, keep the
  // counter so the stall protection can kick in after repeated failures.
  if (!self_initiated_disconnect) {
    ppogatt_reset_disconnect_counter();
  }
}

// -------------------------------------------------------------------------------------------------
// For Unit Testing

Transport *ppogatt_client_for_uuid(const Uuid *uuid) {
  return (Transport *) prv_find_client_with_uuid(uuid);
}

TransportDestination ppogatt_get_destination(Transport *transport) {
  return ((PPoGATTClient *)transport)->destination;
}

bool ppogatt_has_client_for_uuid(const Uuid *uuid) {
  return (prv_find_client_with_uuid(uuid) != NULL);
}

uint32_t ppogatt_client_count(void) {
  return list_count((ListNode *) s_ppogatt_head);
}

void ppogatt_trigger_rx_ack_send_timeout(void) {
  PPoGATTClient *client = s_ppogatt_head;
  while (client) {
    prv_rx_ack_kernelmain_cb(client);
    client = (PPoGATTClient *) client->node.next;
  }
}
