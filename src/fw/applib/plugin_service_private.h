/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "event_service_client.h"
#include "plugin_service.h"

// We dynamically allocate one of these for every service we subscribe to
typedef struct {
  ListNode  list_node;
  uint16_t  service_index;                    // index of the service
  PluginServiceHandler handler;               // handler for this service
} PluginServiceEntry;


typedef struct __attribute__((packed)) PluginServiceState {
  bool subscribed_to_app_event_service : 1;   // Set on first plugin_service_subscribe by this app
  EventServiceInfo event_service_info;
  ListNode subscribed_services;               // Linked list of PluginServiceEntrys
} PluginServiceState;

void plugin_service_state_init(PluginServiceState *state);

