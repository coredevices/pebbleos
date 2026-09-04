/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "tick_timer_service.h"
#include "tick_timer_service_private.h"

#include "event_service_client.h"

#include "pbl/services/analytics/analytics.h"
#include "pbl/services/clock.h"
#include "pbl/services/event_service.h"
#include "pbl/services/tick_timer.h"
#include "kernel/events.h"
#include "kernel/kernel_applib_state.h"

#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"

#include "syscall/syscall.h"
#include "system/passert.h"

// ----------------------------------------------------------------------------------------------------
static TickTimerServiceState* prv_get_state(PebbleTask task) {
  if (task == PebbleTask_Unknown) {
    task = pebble_task_get_current();
  }

  if (task == PebbleTask_App) {
    return app_state_get_tick_timer_service_state();
  } else if (task == PebbleTask_Worker) {
    return worker_state_get_tick_timer_service_state();
  } else if (task == PebbleTask_KernelMain) {
    return kernel_applib_get_tick_timer_service_state();
  } else {
    WTF;
  }
}


static void do_handle(PebbleEvent *e, void *context) {
  TickTimerServiceState *state = prv_get_state(PebbleTask_Unknown);
  PBL_ASSERTN(state->handler != NULL);

  TimeUnits units_changed = 0;
  struct tm currtime;
  sys_localtime_r(&e->clock_tick.tick_time, &currtime);

  if (!state->first_tick) {
    if (state->last_time.tm_sec != currtime.tm_sec) {
      units_changed |= SECOND_UNIT;
    }
    if (state->last_time.tm_min != currtime.tm_min) {
      units_changed |= MINUTE_UNIT;
    }
    if (state->last_time.tm_hour != currtime.tm_hour) {
      units_changed |= HOUR_UNIT;
    }
    if (state->last_time.tm_mday != currtime.tm_mday) {
      units_changed |= DAY_UNIT;
    }
    if (state->last_time.tm_mon != currtime.tm_mon) {
      units_changed |= MONTH_UNIT;
    }
    if (state->last_time.tm_year != currtime.tm_year) {
      units_changed |= YEAR_UNIT;
    }
  }

  bool is_24h = clock_is_24h_style();
  bool format_changed = (is_24h != state->last_is_24h);
  state->last_is_24h = is_24h;

  if (((state->tick_units & units_changed) != 0) || state->first_tick || format_changed) {
    state->handler(&currtime, units_changed);
  }

  state->last_time = currtime;
  state->first_tick = false;
}

void tick_timer_service_init(void) {
  TickTimerServiceState *state = prv_get_state(PebbleTask_Unknown);
  state->handler = NULL;
  event_service_init(PEBBLE_TICK_EVENT, &tick_timer_add_subscriber, &tick_timer_remove_subscriber);
}

void tick_timer_service_subscribe(TimeUnits tick_units, TickHandler handler) {
  TickTimerServiceState *state = prv_get_state(PebbleTask_Unknown);
  state->handler = handler;
  state->tick_units = tick_units;
  state->first_tick = true;
  event_service_client_subscribe(&state->tick_service_info);
  if (pebble_task_get_current() == PebbleTask_App && sys_app_is_watchface()) {
    PBL_ANALYTICS_SET_UNSIGNED(app_tick_timer_second_subscribed,
                               (tick_units & SECOND_UNIT) ? 1 : 0);
  }
  // TODO: make an effort to get this closer to the "actual" second tick
}

void tick_timer_service_unsubscribe(void) {
  TickTimerServiceState *state = prv_get_state(PebbleTask_Unknown);
  event_service_client_unsubscribe(&state->tick_service_info);
  state->handler = NULL;
  if (pebble_task_get_current() == PebbleTask_App && sys_app_is_watchface()) {
    PBL_ANALYTICS_SET_UNSIGNED(app_tick_timer_second_subscribed, 0);
  }
}


void tick_timer_service_state_init(TickTimerServiceState *state) {
  *state = (TickTimerServiceState) {
    .tick_service_info = {
      .type = PEBBLE_TICK_EVENT,
      .handler = &do_handle,
    },
  };
}
