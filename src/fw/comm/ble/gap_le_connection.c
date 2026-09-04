/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gap_le_connection.h"

#include "comm/bt_conn_mgr.h"
#include "comm/bt_lock.h"

#include "kernel/pbl_malloc.h"

#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "system/passert.h"

#include "pbl/util/list.h"

#include <pbl/btutil/bt_device.h>

//! About this module
//! -----------------
//! - Book-keeping of connection-related state for GAP and GATT.
//! - gap_le_connection.c registers connections with this module.
//! - Passive. Does not initiate (dis)connections.

// -------------------------------------------------------------------------------------------------

// Defined in gatt_service_changed.c
extern void gatt_service_changed_server_cleanup_by_connection(struct GAPLEConnection *connection);

// Defined in gatt_client_discovery.c
extern void gatt_client_discovery_cleanup_by_connection(GAPLEConnection *connection,
                                                        BTErrno reason);
extern void gatt_client_cleanup_discovery_jobs(GAPLEConnection *connection);

// Defined in gap_le_connect_params.c
extern void gap_le_connect_params_setup_connection(GAPLEConnection *connection, TimerID timer);
extern void gap_le_connect_params_cleanup_by_connection(GAPLEConnection *connection);

// -------------------------------------------------------------------------------------------------
// Static Variables -- MUST be protected with bt_lock/unlock!

static GAPLEConnection *s_connections;

static bool s_le_connection_module_initialized = false;

// -------------------------------------------------------------------------------------------------
// Internal helpers

static bool prv_list_filter_by_gatt_id(ListNode *found_node, void *data) {
  const unsigned int connection_id = (uintptr_t) data;
  const GAPLEConnection *connection = (const GAPLEConnection *) found_node;
  return (connection->gatt_connection_id == connection_id);
}

static GAPLEConnection * prv_find_connection_by_gatt_id(uintptr_t connection_id) {
  return (GAPLEConnection *) list_find(&s_connections->node,
                                       prv_list_filter_by_gatt_id,
                                       (void *) connection_id);
}

static bool prv_list_filter_for_addr(ListNode *found_node, void *data) {
  const BTDeviceAddress *addr = (const BTDeviceAddress *) data;
  const GAPLEConnection *connection = (const GAPLEConnection *) found_node;
  return bt_device_address_equal(&connection->device.address, addr);
}

static GAPLEConnection * prv_find_connection_by_addr(const BTDeviceAddress *addr) {
  return (GAPLEConnection *) list_find(&s_connections->node,
                                       prv_list_filter_for_addr,
                                       (void *) addr);
}

static bool prv_list_filter_for_device(ListNode *found_node, void *data) {
  const BTDeviceInternal *device = (const BTDeviceInternal *) data;
  const GAPLEConnection *connection = (const GAPLEConnection *) found_node;
  return bt_device_equal(&connection->device.opaque, &device->opaque);
}

static GAPLEConnection * prv_find_connection(const BTDeviceInternal *device) {
  return (GAPLEConnection *) list_find(&s_connections->node,
                                       prv_list_filter_for_device,
                                       (void *) device);
}

// -------------------------------------------------------------------------------------------------

static bool prv_find_connection_by_irk_filter(GAPLEConnection *connection, void *data) {
  const SMIdentityResolvingKey *irk = (const SMIdentityResolvingKey *)data;
  if (!connection->irk) {
    return false;
  }
  return (0 == memcmp(irk, connection->irk, sizeof(*irk)));
}

//! bt_lock() is expected to be taken by the caller
GAPLEConnection *gap_le_connection_find_by_irk(const SMIdentityResolvingKey *irk) {
  return gap_le_connection_find(prv_find_connection_by_irk_filter, (void *)irk);
}

// -------------------------------------------------------------------------------------------------

//! bt_lock() is expected to be taken by the caller
void gap_le_connection_set_irk(GAPLEConnection *connection, const SMIdentityResolvingKey *irk) {
  if (connection->irk) {
    kernel_free(connection->irk);
  }
  SMIdentityResolvingKey *irk_copy = NULL;
  if (irk) {
    irk_copy = kernel_zalloc_check(sizeof(*irk_copy));
    memcpy(irk_copy, irk, sizeof(*irk_copy));
  }
  connection->irk = irk_copy;
}

// -------------------------------------------------------------------------------------------------

GAPLEConnection *gap_le_connection_add(const BTDeviceInternal *device,
                                       const SMIdentityResolvingKey *irk,
                                       bool local_is_master,
                                       TimerID param_watchdog_timer) {
  bt_lock_assert_held(true /* is_held */);
  PBL_ASSERTN(!gap_le_connection_is_connected(device));

  GAPLEConnection *connection = kernel_zalloc_check(sizeof(GAPLEConnection));

  *connection = (const GAPLEConnection) {
    .device = *device,
    .local_is_master = local_is_master,
    .conn_mgr_info = bt_conn_mgr_info_init(),
    .bonding_id = BT_BONDING_ID_INVALID,
    .ticks_since_connection = rtc_get_ticks(),
    .is_remote_device_managing_connection_parameters = false,
    .connection_parameter_sets = NULL,
  };
  gap_le_connection_set_irk(connection, irk);

  s_connections = (GAPLEConnection *) list_prepend(&s_connections->node,
                                                   &connection->node);

  gap_le_connect_params_setup_connection(connection, param_watchdog_timer);

  return connection;
}

// -------------------------------------------------------------------------------------------------

void prv_destroy_connection(GAPLEConnection *connection) {
  gatt_service_changed_server_cleanup_by_connection(connection);
  gap_le_connect_params_cleanup_by_connection(connection);
  gatt_client_discovery_cleanup_by_connection(connection, BTErrnoServiceDiscoveryDisconnected);
  gatt_client_subscriptions_cleanup_by_connection(connection, false /* should_unsubscribe */);
  gatt_client_cleanup_discovery_jobs(connection);

  list_remove(&connection->node, (ListNode **) &s_connections, NULL);
  bt_conn_mgr_info_deinit(&connection->conn_mgr_info);
  kernel_free(connection->connection_parameter_sets);
  kernel_free(connection->pairing_state);
  kernel_free(connection->device_name);
  kernel_free(connection->irk);
  kernel_free(connection);
}

void gap_le_connection_remove(const BTDeviceInternal *device) {
  bt_lock();
  {
    GAPLEConnection *connection = prv_find_connection(device);

    // Verify that:
    //  the reason we can't find a connection is because we have deinitialized everything
    //  we only have connections stored after the module has been initialized
    PBL_ASSERTN((connection != NULL) == s_le_connection_module_initialized);

    if (connection) {
      prv_destroy_connection(connection);
    }
  }
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------

bool gap_le_connection_is_connected(const BTDeviceInternal *device) {
  bt_lock();
  const bool connected = (prv_find_connection(device) != NULL);
  bt_unlock();
  return connected;
}

// -------------------------------------------------------------------------------------------------

bool gap_le_connection_is_encrypted(const BTDeviceInternal *device) {
  bool encrypted = false;
  bt_lock();
  GAPLEConnection *connection = prv_find_connection(device);
  if (connection != NULL) {
    encrypted = connection->is_encrypted;
  }
  bt_unlock();
  return encrypted;
}

// -------------------------------------------------------------------------------------------------

uint16_t gap_le_connection_get_gatt_mtu(const BTDeviceInternal *device) {
  bt_lock();
  const GAPLEConnection *connection = prv_find_connection(device);
  const uint16_t mtu = connection ? connection->gatt_mtu : 0;
  bt_unlock();
  return mtu;
}

// -------------------------------------------------------------------------------------------------

void gap_le_connection_init(void) {
  s_le_connection_module_initialized = true;
}

// -------------------------------------------------------------------------------------------------

void gap_le_connection_deinit(void) {
  bt_lock();
  {
    GAPLEConnection *connection = s_connections;
    while (connection) {
      GAPLEConnection *next_connection =
                                      (GAPLEConnection *) connection->node.next;
      prv_destroy_connection(connection);
      connection = next_connection;
    }
    s_connections = NULL;
    s_le_connection_module_initialized = false;
  }
  bt_unlock();
}

// -------------------------------------------------------------------------------------------------
// The call below require the caller to own the bt_lock while calling the
// function and for as long as the result is being used / accessed.

// -------------------------------------------------------------------------------------------------

GAPLEConnection *gap_le_connection_any(void) {
  return s_connections;
}

static bool prv_valid_conn_filter(ListNode *found_node, void *data) {
  GAPLEConnection *searching_for = (GAPLEConnection *)data;
  GAPLEConnection *conn = (GAPLEConnection *)found_node;
  return (searching_for == conn);
}

bool gap_le_connection_is_valid(const GAPLEConnection *conn) {
  return (list_find(&s_connections->node, prv_valid_conn_filter, (void *)conn) != NULL);
}

//! @note !!! To access the returned context bt_lock MUST be held!!!
GAPLEConnection *gap_le_connection_by_device(const BTDeviceInternal *device) {
  return prv_find_connection(device);
}

// -------------------------------------------------------------------------------------------------

//! @note !!! To access the returned context bt_lock MUST be held!!!
GAPLEConnection *gap_le_connection_by_addr(const BTDeviceAddress *addr) {
  return prv_find_connection_by_addr(addr);
}

// -------------------------------------------------------------------------------------------------

//! @note !!! To access the returned context bt_lock MUST be held!!!
GAPLEConnection *gap_le_connection_by_gatt_id(unsigned int connection_id) {
  return prv_find_connection_by_gatt_id(connection_id);
}

// -------------------------------------------------------------------------------------------------

//! @note !!! To access the returned context bt_lock MUST be held!!!
GAPLEConnection *gap_le_connection_find(GAPLEConnectionFindCallback filter,
                                        void *data) {
  return (GAPLEConnection *) list_find(&s_connections->node,
                                       (ListFilterCallback) filter,
                                       data);
}

// -------------------------------------------------------------------------------------------------

//! @note !!! To access the returned context bt_lock MUST be held!!!
void gap_le_connection_for_each(GAPLEConnectionForEachCallback cb, void *data) {
  GAPLEConnection *connection = s_connections;
  while (connection) {
    cb(connection, data);
    connection = (GAPLEConnection *) connection->node.next;
  }
}

// -------------------------------------------------------------------------------------------------

void gap_le_connection_set_gateway(GAPLEConnection *connection, bool is_gateway) {
  connection->is_gateway = is_gateway;

  // TODO: update bonding `is_gateway` flag
  // bt_persistent_storage_...
}

// -------------------------------------------------------------------------------------------------

static bool prv_find_gateway(GAPLEConnection *connection, void *data) {
  return connection->is_gateway;
}

GAPLEConnection *gap_le_connection_get_gateway(void) {
  return gap_le_connection_find(prv_find_gateway, NULL);
}

// -------------------------------------------------------------------------------------------------

static bool prv_find_connection_with_bonding_id(GAPLEConnection *connection, void *data) {
  BTBondingID bonding_id = (BTBondingID)(uintptr_t)data;
  return (connection->bonding_id == bonding_id);
}

void gap_le_connection_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
  if (op != BtPersistBondingOpWillDelete) {
    return;
  }
  // Clean up the bonding_id field for the bonding that just got removed:
  bt_lock();
  GAPLEConnection *connection = gap_le_connection_find(prv_find_connection_with_bonding_id,
                                                       (void *)(uintptr_t)bonding);
  if (connection) {
    connection->bonding_id = BT_BONDING_ID_INVALID;
  }
  bt_unlock();
}

void gap_le_connection_copy_device_name(
    const GAPLEConnection *connection, char *name_out, size_t name_out_len) {
  bt_lock();
  {
    if (!gap_le_connection_is_valid(connection)) {
      goto unlock;
    }
    if (connection->device_name != NULL) {
      strncpy(name_out, connection->device_name, name_out_len);
    }
    name_out[name_out_len - 1] = '\0';
  }
unlock:
  bt_unlock();
}
