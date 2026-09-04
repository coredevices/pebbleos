/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/event_service_client.h"
#include "logging/logging_private.h"
#include "applib/accel_service_private.h"
#include "applib/compass_service_private.h"
#include "applib/plugin_service_private.h"
#include "applib/backlight_service_private.h"
#include "applib/battery_state_service_private.h"
#include "applib/connection_service_private.h"
#include "applib/health_service_private.h"
#include "applib/tick_timer_service_private.h"
#include "applib/tick_timer_service.h"
#include "pbl/util/heap.h"

#include <stdbool.h>

struct _reent;

typedef struct MemorySegment MemorySegment;

//! Allocate worker state in the worker task's RAM segment
bool worker_state_configure(MemorySegment *worker_state_segment);

//! Reset ourselves to a blank slate
void worker_state_init(void);

//! Clean up after ourselves nicely. Note that this may not be called if the app crashes.
void worker_state_deinit(void);

Heap *worker_state_get_heap(void);

AccelServiceState *worker_state_get_accel_state(void);

CompassServiceConfig **worker_state_get_compass_config(void);

EventServiceInfo *worker_state_get_event_service_state(void);

PluginServiceState *worker_state_get_plugin_service(void);

LogState *worker_state_get_log_state(void);

BatteryStateServiceState *worker_state_get_battery_state_service_state(void);

BacklightServiceState *worker_state_get_backlight_service_state(void);

TickTimerServiceState *worker_state_get_tick_timer_service_state(void);

ConnectionServiceState *worker_state_get_connection_service_state(void);

struct tm *worker_state_get_gmtime_tm(void);
struct tm *worker_state_get_localtime_tm(void);
char *worker_state_get_localtime_zone(void);

void *worker_state_get_rand_ptr(void);

HealthServiceState *worker_state_get_health_service_state(void);
