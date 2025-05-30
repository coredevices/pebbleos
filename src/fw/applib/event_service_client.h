/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "kernel/events.h"

typedef void (*EventServiceEventHandler)(PebbleEvent *e, void *context);

typedef struct __attribute__((packed)) {
  ListNode list_node;
  PebbleEventType type;
  EventServiceEventHandler handler;
  void *context;
} EventServiceInfo;

void event_service_client_subscribe(EventServiceInfo * service_info);
void event_service_client_unsubscribe(EventServiceInfo * service_info);
void event_service_client_handle_event(PebbleEvent *e);
bool event_service_filter(ListNode *node, void *tp);
