/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "prefs_sync.h"

#include "applib/event_service_client.h"
#include "kernel/events.h"
#include "pbl/services/blob_db/api.h"
#include "pbl/services/blob_db/settings_blob_db.h"
#include "pbl/services/blob_db/sync.h"
#include <pbl/logging/logging.h>

//! Prefs Sync using BlobDB
//!
//! Settings are now synced via BlobDB with database ID 0x0F (BlobDBIdSettings).
//! 
//! The whitelist filtering and sync logic are implemented in:
//!   services/blob_db/settings_blob_db.c
//!
//! This module simply triggers BlobDB sync when the phone connects.

static bool s_sync_initialized = false;
static bool s_is_connected = false;
static EventServiceInfo s_connection_event_info;
static EventServiceInfo s_capabilities_event_info;

//! Try to start settings sync if conditions are met
static void prv_try_start_sync(void) {
  if (!s_is_connected) {
    return;
  }

  if (!settings_blob_db_phone_supports_sync()) {
    PBL_LOG_DBG("Phone doesn't support settings sync");
    return;
  }

  blob_db_sync_db(BlobDBIdSettings);
}

//! Connection state change callback
static void prv_connection_handler(PebbleEvent *event, void *context) {
  bool connected = event->bluetooth.comm_session_event.is_open;
  s_is_connected = connected;

  if (connected) {
    // Try to sync - capabilities may already be cached from previous connection
    prv_try_start_sync();
  }
}

//! Capabilities changed callback - triggers sync when phone reports settings_sync_support
static void prv_capabilities_handler(PebbleEvent *event, void *context) {
  // Check if the settings_sync_support capability changed
  if (event->capabilities.flags_diff.settings_sync_support) {
    prv_try_start_sync();
  }
}

// Public API

void prefs_sync_init(void) {
  if (s_sync_initialized) {
    return;
  }

  // Subscribe to connection events
  s_connection_event_info = (EventServiceInfo) {
    .type = PEBBLE_BT_CONNECTION_DEBOUNCED_EVENT,
    .handler = prv_connection_handler,
  };
  event_service_client_subscribe(&s_connection_event_info);

  // Subscribe to capabilities changed events - this fires when phone sends version response
  s_capabilities_event_info = (EventServiceInfo) {
    .type = PEBBLE_CAPABILITIES_CHANGED_EVENT,
    .handler = prv_capabilities_handler,
  };
  event_service_client_subscribe(&s_capabilities_event_info);

  // Start with disconnected state - will be updated when we receive connection events
  s_is_connected = false;

  s_sync_initialized = true;
}

void prefs_sync_deinit(void) {
  if (!s_sync_initialized) {
    return;
  }

  // Unsubscribe from events
  event_service_client_unsubscribe(&s_connection_event_info);
  event_service_client_unsubscribe(&s_capabilities_event_info);

  s_sync_initialized = false;
  s_is_connected = false;
}

void prefs_sync_trigger(void) {
  if (!s_sync_initialized) {
    PBL_LOG_WRN("Prefs sync not initialized");
    return;
  }
  
  if (!s_is_connected) {
    PBL_LOG_WRN("Not connected to phone, cannot sync");
    return;
  }
  
  // Check if the phone supports settings sync
  if (!settings_blob_db_phone_supports_sync()) {
    PBL_LOG_WRN("Phone doesn't support settings sync");
    return;
  }
  
  PBL_LOG_DBG("Manually triggering settings sync via BlobDB");
  
  blob_db_sync_db(BlobDBIdSettings);
}
