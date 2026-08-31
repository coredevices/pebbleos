/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/bt_driver_comm.h>

#include "pbl/services/comm_session/session.h"
#include "pbl/services/comm_session/session_analytics.h"
#include "pbl/services/comm_session/session_internal.h"
#include "pbl/services/comm_session/session_transport.h"

#include "applib/app_comm.h"
#include "comm/ble/kernel_le_client/app_launch/app_launch.h"
#include "comm/bt_lock.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/comm_session/session_remote_version.h"
#include "pbl/services/comm_session/session_send_buffer.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"

#include "process_management/app_manager.h"

#include "pbl/services/system_task.h"
#include "pbl/services/data_logging/dls_private.h"

#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "syscall/syscall_internal.h"

#include <stdint.h>

PBL_LOG_MODULE_DEFINE(service_comm_session, CONFIG_SERVICE_COMM_SESSION_LOG_LEVEL);

// -------------------------------------------------------------------------------------------------
// Static variables

//! The list of open Pebble Protocol sessions.
//! @note bt_lock() must be held when accessing this list.
static CommSession *s_session_head;

// -------------------------------------------------------------------------------------------------
// Defined in session_send_buffer.c

extern SendBuffer * comm_session_send_buffer_create(TransportDestination destination);
extern void comm_session_send_buffer_destroy(SendBuffer *sb);

// -------------------------------------------------------------------------------------------------
// Defined in session_receive_router.c
extern void comm_session_receive_router_cleanup(CommSession *session);

// -------------------------------------------------------------------------------------------------
// Defined in session_send_queue.c
extern void comm_session_send_queue_cleanup(CommSession *session);

// -------------------------------------------------------------------------------------------------

static void prv_put_comm_session_event(bool is_open, bool is_system) {
  PebbleEvent event = {
    .type = PEBBLE_COMM_SESSION_EVENT,
    .bluetooth.comm_session_event.is_open = is_open,
    .bluetooth.comm_session_event
    .is_system = is_system,
  };
  event_put(&event);
}

// -------------------------------------------------------------------------------------------------
// Extern'd interface for session_send_buffer.c and session_remote_version.c
// -------------------------------------------------------------------------------------------------

//! bt_lock() is expected to be taken by the caller!
bool comm_session_is_valid(const CommSession *session) {
  return list_contains((ListNode *) s_session_head, &session->node);
}

// -------------------------------------------------------------------------------------------------
//! Extern'd interface for protocol.c
// -------------------------------------------------------------------------------------------------

bool comm_session_has_capability(CommSession *session, CommSessionCapability capability) {
  bool rv = false;
  bt_lock();
  if (comm_session_is_valid(session)) {
    rv = (session->protocol_capabilities & capability) != 0;
  }
  bt_unlock();

  return rv;
}

CommSessionCapability comm_session_get_capabilities(CommSession *session) {
  CommSessionCapability capabilities = 0;
  bt_lock();
  if (comm_session_is_valid(session)) {
    capabilities = session->protocol_capabilities;
  }
  bt_unlock();
  return capabilities;
}

void comm_session_set_capabilities(CommSession *session, CommSessionCapability capability_flags) {
  bt_lock();
  if (comm_session_is_valid(session)) {
    session->protocol_capabilities = capability_flags;
  }
  bt_unlock();

  if (comm_session_is_system(session)) {
    const PebbleProtocolCapabilities capabilities = { .flags = capability_flags };
    bt_persistent_storage_set_cached_system_capabilities(&capabilities);
  }
}


//! Resets the session (close and attempt re-opening the session)
//! @note If the underlying transport is iAP, this will end up closing all the sessions on top of
//! the transport, since we don't really have the ability to close a single iAP session.
void comm_session_reset(CommSession *session) {
  bt_lock();
  {
    if (!comm_session_is_valid(session)) {
      PBL_LOG_WRN("Already closed!");
      goto unlock;
    }
    session->transport_imp->reset(session->transport);
  }
unlock:
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------
// Interfaces towards Transport (reading from the send buffer to actually transmit the data):
// -------------------------------------------------------------------------------------------------

static const Uuid *prv_get_uuid(const CommSession *session) {
  if (session->transport_imp->get_uuid) {
    return session->transport_imp->get_uuid(session->transport);
  }
  return NULL;
}

static const char *prv_string_for_destination(TransportDestination destination) {
  switch (destination) {
    case TransportDestinationSystem: return "S";
    case TransportDestinationApp: return "A";
    case TransportDestinationHybrid: return "H";
    default:
      WTF;
      return NULL;
  }
}

static void prv_log_session_event(CommSession *session, bool is_open) {
  char uuid_string[UUID_STRING_BUFFER_LENGTH];
  uuid_to_string(prv_get_uuid(session), uuid_string);
  PBL_LOG_INFO("Session event: is_open=%d, destination=%s, app_uuid=%s",
          is_open, prv_string_for_destination(session->destination), uuid_string);
}

static bool prv_is_transport_type(Transport *transport,
                                  const TransportImplementation *implementation,
                                  CommSessionTransportType expected_transport_type) {
  CommSessionTransportType transport_type = implementation->get_type(transport);
  return transport_type == expected_transport_type;
}

//! bt_lock() is expected to be taken by the caller!
CommSession * comm_session_open(Transport *transport, const TransportImplementation *implementation,
                       TransportDestination destination) {

  const bool is_system = (destination != TransportDestinationApp);
  if (is_system) {
    CommSession *existing_system_session = comm_session_get_system_session();
    if (existing_system_session) {
      // Allow PULSE transport to be opened alongside any other transport
      // Actually using PULSE at the same time as another transport may cause
      // undesirable behaviour however.
      if (!prv_is_transport_type(existing_system_session->transport,
                                         existing_system_session->transport_imp,
                                         CommSessionTransportType_PULSE)
           && !prv_is_transport_type(transport, implementation, CommSessionTransportType_PULSE)) {
        if (!existing_system_session->transport_imp->close) {
          // iAP sessions cannot be closed from the watch' side :(
          PBL_LOG_ERR("System session already exists and cannot be closed");
          return NULL;
        }
        // Last system session to connect wins:
        // This is to work-around a race condition that happens when iOS still has the PPoGATT service
        // registered (the app has crashed / jettisoned) and iSPP is connected but the system session
        // is running over PPoGATT. If the app launches again, it will have no state of what was the
        // previously used transport was, prior to getting killed. Often, iAP ends up winning.
        // However, to the firmware, PPoGATT still appears connected, so we'd end up here.
        PBL_LOG_INFO("System session already exists, closing it now");
        existing_system_session->transport_imp->close(existing_system_session->transport);
      }
    }
  }

  CommSession *session = kernel_malloc(sizeof(CommSession));
  if (!session) {
    PBL_LOG_ERR("Not enough memory for new CommSession");
    return NULL;
  }
  *session = (const CommSession) {
    .transport = transport,
    .transport_imp = implementation,
    .destination = destination,
  };

  s_session_head = (CommSession *) list_prepend((ListNode *) s_session_head, &session->node);

  prv_log_session_event(session, true /* is_open */);

  // Request capabilities for both the Pebble app and 3rd party companion apps:
  session_remote_version_start_requests(session);

  comm_session_analytics_open_session(session);

  prv_put_comm_session_event(true, is_system);

  if (is_system && (session->destination == TransportDestinationHybrid)) {
    // For Android, if the app is connected, PebbleKit should be
    // working as well
    prv_put_comm_session_event(true, false);
  }

  return session;
}

// -------------------------------------------------------------------------------------------------

//! bt_lock() is expected to be taken by the caller!
void comm_session_close(CommSession *session, CommSessionCloseReason reason) {
  PBL_ASSERTN(comm_session_is_valid(session));

  prv_log_session_event(session, false /* is_open */);

  comm_session_analytics_close_session(session, reason);

  const bool is_system = (session->destination != TransportDestinationApp);
  if (is_system) {

    // Only relevant for iOS + BLE, otherwise this is a no-op:
    app_launch_trigger();

    // TODO: PBL-1771: find a more graceful way to handle this
#ifndef CONFIG_RECOVERY_FW
    system_task_add_callback(dls_private_handle_disconnect, NULL);
#endif
  }

  prv_put_comm_session_event(false, is_system);

  if (is_system && (session->destination == TransportDestinationHybrid)) {
    prv_put_comm_session_event(true, false);
  }

  // Cleanup:
  comm_session_receive_router_cleanup(session);
  comm_session_send_queue_cleanup(session);
  list_remove(&session->node, (ListNode **) &s_session_head, NULL);
  kernel_free(session);
}

void comm_session_set_responsiveness(
    CommSession *session, BtConsumer consumer, ResponseTimeState state,
    uint16_t max_period_secs) {
  comm_session_set_responsiveness_ext(session, consumer, state, max_period_secs, NULL);
}

void comm_session_set_responsiveness_ext(CommSession *session, BtConsumer consumer,
                                         ResponseTimeState state, uint16_t max_period_secs,
                                         ResponsivenessGrantedHandler granted_handler) {
  if (session) {
    bt_lock();
    if (comm_session_is_valid(session)) {
      session->transport_imp->set_connection_responsiveness(session->transport, consumer,
                                                            state, max_period_secs,
                                                            granted_handler);
    }
    bt_unlock();
  }
}

// -------------------------------------------------------------------------------------------------

bool comm_session_is_current_task_send_next_task(CommSession *session) {
  if (session->transport_imp->schedule) {
    return session->transport_imp->is_current_task_schedule_task(session->transport);
  }
  return bt_driver_comm_is_current_task_send_next_task();
}

void prv_send_next(CommSession *session, bool is_callback) {
  bt_lock();
  {
    if (!comm_session_is_valid(session)) {
      // Session closed in the mean time
      goto unlock;
    }
    // Flip the flag before the send_next callback, so it can schedule again if needed.
    // Only flip the flag, if this called as a thread callback, to avoid getting more
    // of these callbacks scheduled.
    if (is_callback) {
      session->is_send_next_call_pending = false;
    }

    // Kick the transport to send out the next bytes it has prepared.  It's possible these bytes
    // are not in the send queue (i.e PPoGATT Acks) so we leave it up to the transport to check
    // that
    session->transport_imp->send_next(session->transport);
  }
unlock:
  bt_unlock();
}

void bt_driver_run_send_next_job(CommSession *session, bool is_callback) {
  prv_send_next(session, is_callback);
}

//! bt_lock() is expected to be taken by the caller!
void comm_session_send_next(CommSession *session) {
  if (session->is_send_next_call_pending) {
    return;
  }

  TransportSchedule schedule_func = session->transport_imp->schedule;
  if (!schedule_func) {
    schedule_func = bt_driver_comm_schedule_send_next_job;
  }

  if (schedule_func(session)) {
    session->is_send_next_call_pending = true;
  } else {
    PBL_LOG_ERR("Failed to schedule comm_session_send_next callback");
  }
}

//! extern'd for session_send_buffer.c
void comm_session_send_next_immediately(CommSession *session) {
  prv_send_next(session, false /* is_callback */);
}

//! For unit test
//! bt_lock() is expected to be taken by the caller!
bool comm_session_send_next_is_scheduled(CommSession *session) {
  return session->is_send_next_call_pending;
}

// -------------------------------------------------------------------------------------------------
// Interface towards the system / subsystems that need to receive and send data
// -------------------------------------------------------------------------------------------------

bool comm_session_send_data(CommSession *session, uint16_t endpoint_id,
                            const uint8_t *data, size_t length, uint32_t timeout_ms) {
  if (!session) {
    return false;
  }
  SendBuffer *sb = comm_session_send_buffer_begin_write(session, endpoint_id, length, timeout_ms);
  if (!sb) {
    PBL_LOG_WRN("Could not acquire send buffer for %x", endpoint_id);
    return false;
  }
  comm_session_send_buffer_write(sb, data, length);
  comm_session_send_buffer_end_write(sb);
  return true;
}

// -------------------------------------------------------------------------------------------------

typedef struct {
  const Uuid *app_uuid;
  CommSession *fallback_session;
} FindByAppUUIDContext;

static bool prv_find_session_by_app_uuid_comparator(ListNode *found_node, void *data) {
  CommSession *session = (CommSession *)found_node;
  FindByAppUUIDContext *ctx = data;
  const Uuid *session_uuid = prv_get_uuid(session);
  if (uuid_equal(session_uuid, ctx->app_uuid)) {
    // Match on UUID found!
    return true;
  }
  // If there is no valid UUID, it means we don't know what app UUID is associated with the
  // transport, consider it as a fallback option:
  const bool is_unknown_app_session = (session->destination == TransportDestinationApp &&
                                       uuid_is_invalid(session_uuid));
  const bool is_hybrid_session = (session->destination == TransportDestinationHybrid);
  if (is_hybrid_session || is_unknown_app_session) {
    // On Android + SPP, we can expect one Hybrid session, so we assume that the found session is
    // the hybrid one.
    // On iOS + iAP, we can expect at most one App session, so we assume that the found session is
    // the app one.
    if (ctx->fallback_session) {
      PBL_LOG_ERR("Fallback session already set!?");
    }
    ctx->fallback_session = session;
  }
  return false;
}

static CommSession *prv_get_app_session(void) {
  const Uuid *app_uuid = &app_manager_get_current_app_md()->uuid;
  if (uuid_is_system(app_uuid) || uuid_is_invalid(app_uuid)) {
    return NULL;
  }
  FindByAppUUIDContext ctx = (FindByAppUUIDContext) {
    .app_uuid = app_uuid,
  };
  // Try most specific first:
  CommSession *session = (CommSession *) list_find((ListNode *) s_session_head,
                                                   prv_find_session_by_app_uuid_comparator, &ctx);
  if (!session) {
    return ctx.fallback_session;
  }
  return session;
}

static bool prv_find_session_is_system_filter(ListNode *found_node, void *data) {
  CommSession *session = (CommSession *) found_node;
  const TransportDestination destination = session->destination;
  return (destination == TransportDestinationSystem || destination == TransportDestinationHybrid)
          && !prv_is_transport_type(session->transport, session->transport_imp,
                                    CommSessionTransportType_QEMU)
          && !prv_is_transport_type(session->transport, session->transport_imp,
                                    CommSessionTransportType_PULSE);
}

static bool prv_find_session_is_type_filter(ListNode *found_node, void *data) {
  CommSession *session = (CommSession *) found_node;
  CommSessionTransportType required_session_type = (CommSessionTransportType) data;
  return prv_is_transport_type(session->transport, session->transport_imp, required_session_type);
}

static CommSession *prv_find_session_by_type(CommSessionTransportType session_type) {
  return (CommSession *) list_find((ListNode *) s_session_head,
                                   prv_find_session_is_type_filter, (void*)session_type);
}

static CommSession *prv_get_system_session(void) {
  // Attempt to explicitly find and return a session that isn't QEMU or PULSE
  CommSession *session = (CommSession *) list_find((ListNode *) s_session_head,
                                                   prv_find_session_is_system_filter, NULL);
  if (session) {
    return session;
  }

  // If we don't find one, try to find a PULSE session
  session = prv_find_session_by_type(CommSessionTransportType_PULSE);
  if (session) {
    return session;
  }

  // If we don't find one, try to find a QEMU session as a last resort
  return prv_find_session_by_type(CommSessionTransportType_QEMU);
}

static CommSession *prv_get_session_by_type(CommSessionType type) {
  switch (type) {
    case CommSessionTypeSystem:
      return prv_get_system_session();
    case CommSessionTypeApp:
      return prv_get_app_session();
    case CommSessionTypeInvalid:
    default:
      return NULL;
  }
}

const Uuid *comm_session_get_uuid(const CommSession *session) {
  bt_lock_assert_held(true);
  return prv_get_uuid(session);
}

CommSession *comm_session_get_by_type(CommSessionType type) {
  CommSession *session;
  bt_lock();
  {
    session = prv_get_session_by_type(type);
  }
  bt_unlock();
  return session;
}

CommSession *comm_session_get_system_session(void) {
  return comm_session_get_by_type(CommSessionTypeSystem);
}

CommSession *comm_session_get_current_app_session(void) {
  if (app_manager_get_current_app_md()->allow_js) {
    return comm_session_get_system_session();
  }
  return comm_session_get_by_type(CommSessionTypeApp);
}

void comm_session_sanitize_app_session(CommSession **session_in_out) {
  CommSession *permitted_session = comm_session_get_current_app_session();
  if (!permitted_session) {
    // No session connected that can serve the currently running app
    *session_in_out = NULL;
    return;
  }
  if (*session_in_out == NULL) {
    // NULL means "auto select" the session
    *session_in_out = permitted_session;
    return;
  }
  if (*session_in_out != permitted_session) {
    // Don't allow the app to send data to any arbitrary session, this can happen if the session
    // got disconnected in the mean time.
    *session_in_out = NULL;
    return;
  }
}

// -------------------------------------------------------------------------------------------------

CommSessionType comm_session_get_type(const CommSession *session) {
  CommSessionType type = CommSessionTypeInvalid;
  bt_lock();
  {
    if (comm_session_is_valid(session)) {
      type = (session->destination == TransportDestinationApp) ? CommSessionTypeApp :
                                                                 CommSessionTypeSystem;
    }
  }
  bt_unlock();
  return type;
}

bool comm_session_is_system(CommSession* session) {
  return (comm_session_get_type(session) == CommSessionTypeSystem);
}

// -------------------------------------------------------------------------------------------------

//! Must (only) be called when going out of airplane mode (enabling Bluetooth).
void comm_session_init(void) {
  PBL_ASSERTN(s_session_head == NULL);
}

//! Must (only) be called when going into airplane mode (disabling Bluetooth).
void comm_session_deinit(void) {
  // If this assert fires, it means a Transport has not cleaned up properly after itself by closing
  // all the CommSessions it has opened.
  PBL_ASSERTN(s_session_head == NULL);
}

DEFINE_SYSCALL(void, sys_app_comm_set_responsiveness, SniffInterval interval) {

  CommSession *comm_session = comm_session_get_current_app_session();
  switch (interval) {
    case SNIFF_INTERVAL_REDUCED:
      comm_session_set_responsiveness(comm_session, BtConsumerApp,
                                      ResponseTimeMiddle, MAX_PERIOD_RUN_FOREVER);
      return;
    case SNIFF_INTERVAL_NORMAL:
      comm_session_set_responsiveness(comm_session, BtConsumerApp,
                                      ResponseTimeMax, 0);
      return;
  }
  PBL_LOG_WRN("Invalid sniff interval");
  syscall_failed();
}

DEFINE_SYSCALL(bool, sys_system_pp_has_capability, CommSessionCapability capability) {
  CommSession *session = comm_session_get_system_session();
  return comm_session_has_capability(session, capability);
}
