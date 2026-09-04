/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/animation_private.h"
#include "logging/logging_private.h"
#include "applib/tick_timer_service_private.h"
#include "applib/touch_service_private.h"
#include "applib/compass_service_private.h"
#include "applib/battery_state_service_private.h"
#include "applib/connection_service_private.h"

void kernel_applib_init(void);

AnimationState *kernel_applib_get_animation_state(void);

LogState *kernel_applib_get_log_state(void);

void kernel_applib_release_log_state(LogState *state);

CompassServiceConfig **kernel_applib_get_compass_config(void);

EventServiceInfo* kernel_applib_get_event_service_state(void);

TickTimerServiceState *kernel_applib_get_tick_timer_service_state(void);

TouchServiceState *kernel_applib_get_touch_service_state(void);

ConnectionServiceState* kernel_applib_get_connection_service_state(void);

BatteryStateServiceState* kernel_applib_get_battery_state_service_state(void);

struct Layer;
typedef struct Layer Layer;

Layer** kernel_applib_get_layer_tree_stack(void);
