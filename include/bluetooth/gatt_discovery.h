/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/gatt_service_types.h>

typedef struct GAPLEConnection GAPLEConnection;
typedef struct GATTService GATTService;
typedef struct GATTServiceNode GATTServiceNode;

BTErrno bt_driver_gatt_start_discovery_range(
    const GAPLEConnection *connection, const ATTHandleRange *data);
BTErrno bt_driver_gatt_stop_discovery(GAPLEConnection *connection);

//! It's possible we are disconnected or the stack gets torn down while in the
//! middle of a discovery. This routine gets invoked if the connection gets
//! torn down or goes away so that the implementation can clean up any tracking
//! it has waiting for a discovery to complete
void bt_driver_gatt_handle_discovery_abandoned(void);

//! gatt_service_discovery callbacks
//! cb returns true iff the driver completed, false if a discovery retry was initiated
extern bool bt_driver_cb_gatt_client_discovery_complete(GAPLEConnection *connection, BTErrno errno);
extern void bt_driver_cb_gatt_client_discovery_handle_indication(
    GAPLEConnection *connection, GATTService *service_discovered, BTErrno error);
