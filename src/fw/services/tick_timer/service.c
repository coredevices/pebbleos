/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/tick_timer.h"

#include "kernel/events.h"
#include <pbl/drivers/rtc.h>
#include "pbl/services/regular_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

PBL_LOG_MODULE_DEFINE(service_tick_timer, CONFIG_SERVICE_TICK_TIMER_LOG_LEVEL);

static uint16_t s_num_subscribers;

static void timer_tick_event_publisher(void* data) {
  PebbleEvent e = {
    .type = PEBBLE_TICK_EVENT,
    .clock_tick.tick_time = rtc_get_time(),
  };

  event_put(&e);
}

static RegularTimerInfo s_tick_timer_info = {
  .cb = &timer_tick_event_publisher
};

void tick_timer_add_subscriber(PebbleTask task) {
  ++s_num_subscribers;
  if (s_num_subscribers == 1) {
    PBL_LOG_DBG("starting tick timer");
    regular_timer_add_seconds_callback(&s_tick_timer_info);
  }
}

void tick_timer_remove_subscriber(PebbleTask task) {
  PBL_ASSERTN(s_num_subscribers > 0);
  --s_num_subscribers;
  if (s_num_subscribers == 0) {
    PBL_LOG_DBG("stopping tick timer");
    regular_timer_remove_callback(&s_tick_timer_info);
  }
}
