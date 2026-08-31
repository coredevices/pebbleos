/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/comm_session/session.h"
#include "pbl/services/comm_session/session_analytics.h"

#include "pbl/util/uuid.h"

// -------------------------------------------------------------------------------------------------
// Types and functions that a transport should use to manage the session:

//! Opaque type (can be anything)
typedef struct Transport Transport;

//! Pointer to function implementing the sending of data that is enqueued in the send buffer
typedef void (*TransportSendNext)(Transport *transport);

//! Pointer to function implementing the closing of the transport
//! @note This is called by session.c in case there is a conflict: multiple transports for the
//! 'system' destination. In this case, the older one will be closed. The transport MUST call
//! comm_session_close() before returning from this call.
typedef void (*TransportClose)(Transport *transport);

//! Pointer to function implementing the resetting of the transport.
typedef void (*TransportReset)(Transport *transport);

//! Pointer to function which calls the appropriate connection speed API
//! exported by bt_conn_mgr
typedef void (*TransportSetConnectionResponsiveness)(Transport *transport,
                                                     BtConsumer consumer,
                                                     ResponseTimeState state,
                                                     uint16_t max_period_secs,
                                                     ResponsivenessGrantedHandler granted_handler);

//! Pointer to function which returns the UUID of the app that the transport connects to.
typedef const Uuid *(*TransportGetUUID)(Transport *transport);

typedef CommSessionTransportType (*TransportGetType)(Transport *transport);

//! Pointer to function that schedules a callback to send data over the transport.
typedef bool (*TransportSchedule)(CommSession *session);

typedef bool (*TransportScheduleTask)(Transport *transport);

//! Set of function pointers that the session can use to call back to the transport
typedef struct TransportImplementation {
  //! Pointer to function of that will trigger the transport to send out any newly enqueued data
  //! from the send buffer. bt_lock() is held when this call is made. The implementation must be
  //! able to handle send_next() getting called but having no data in the send buffer. (This is to
  //! allow some implementations to flush out other types of data during the call)
  TransportSendNext send_next;

  TransportClose close;
  TransportReset reset;
  TransportSetConnectionResponsiveness set_connection_responsiveness;

  //! This field is allowed to be NULL if the transport is not UUID-aware.
  TransportGetUUID get_uuid;

  TransportGetType get_type;

  //! Pointer to function that schedules a callback to send data over the transport.
  //! When left NULL, bt_driver_comm_schedule_send_next_job() will be used instead.
  //! @note When providing a function, .schedule_task must be provided as well!
  TransportSchedule schedule;
  TransportScheduleTask is_current_task_schedule_task;
} TransportImplementation;

//! The "destination" of the transport
typedef enum TransportDestination {
  //! The transport carries Pebble Protocol solely for the "system", for example:
  //! iSPP/iAP with Pebble iOS App.
  TransportDestinationSystem,

  //! The transport carries Pebble Protocol solely for a Pebble app, for example:
  //! iSPP/iAP with 3rd party native iOS App and PebbleKit iOS.
  TransportDestinationApp,

  //! The transport carries Pebble Protocol for both the "system" and "app", for example:
  //! Plain SPP with Pebble Android App.
  TransportDestinationHybrid,
} TransportDestination;

// -------------------------------------------------------------------------------------------------
// Open & Close

//! Called by a transport to open/create a Pebble Protocol session for it.
//! @param transport Opaque reference to the underlying serial transport
//! @param implementation Function pointers implementing the transport (e.g. to send data)
//! @param destination Whether the transport carries Pebble Protocol for the "system", a 3rd
//! party app, or both (see \ref TransportDestination).
//! @return True if the session was opened successfully, false if not
//! bt_lock() is expected to be taken by the caller!
CommSession * comm_session_open(Transport *transport, const TransportImplementation *implementation,
                       TransportDestination destination);

//! Called by the transport to indicate that the session associated with the given transport needs
//! to be closed and cleaned up.
//! bt_lock() is expected to be taken by the caller!
//! @param session The session to close.
//! @param reason For analytics tracking.
void comm_session_close(CommSession *session, CommSessionCloseReason reason);

// -------------------------------------------------------------------------------------------------
// Receiving

//! Called by the transport to copy received data from a given buffer into the receive buffer.
//! @note bt_lock() is expected to be taken by the caller!
void comm_session_receive_router_write(CommSession *session,
                                       const uint8_t *data, size_t data_size);

// -------------------------------------------------------------------------------------------------
// Sending

//! @note bt_lock() is expected to be taken by the caller!
//! @return The total size in bytes, of all the messages in the queue.
size_t comm_session_send_queue_get_length(const CommSession *session);

//! Copies bytes from the send buffer into another buffer.
//! @param session The session whose send queue to copy from
//! @param start_offset The offset into the send buffer
//! @param length The number of bytes to copy
//! @param[out] data_out Pointer to the buffer into which to copy the data
//! @return The number of bytes copied
//! @note To avoid making a copy, consider using comm_session_send_queue_get_read_pointer().
//! @note The caller must ensure there is enough data available, for example by getting the length
//! by calling comm_session_send_queue_get_length().
//! @note bt_lock() is expected to be taken by the caller!
size_t comm_session_send_queue_copy(CommSession *session, uint32_t start_offset,
                                    size_t length, uint8_t *data_out);

//! Gets a read pointer and the number of bytes that can be read from the read pointer.
//! @note Internally, a non-contiguous buffer is used, so it is possible that there is more data
//! to read. To access the entire contents, call this function and comm_session_send_queue_consume()
//! repeatedly until it returns zero.
//! @param session The session whose send queue to read from
//! @param data_out Pointer to the pointer to assign the read pointer to.
//! @return The number of bytes that can be read starting at the read pointer.
size_t comm_session_send_queue_get_read_pointer(const CommSession *session,
                                                const uint8_t **data_out);

//! @note bt_lock() is expected to be taken by the caller!
void comm_session_send_queue_consume(CommSession *session, size_t length);

//! Schedule a KernelBG callback to the send_next function of the transport, if needed.
//! In case a callback is already pending, this function is a no-op.
//! If, by the time the callback executes, the send buffer is empty, no callback to send_next will
//! be made either.
//! @note bt_lock() is expected to be taken by the caller!
void comm_session_send_next(CommSession *session);
