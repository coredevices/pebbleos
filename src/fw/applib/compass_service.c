/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef CONFIG_MAG
#error "Use fw/applib/compass_service_stub.c instead if we don't have a magnetometer"
#endif

#include "compass_service.h"
#include "compass_service_private.h"

#include "applib/app_timer.h"
#include "applib/event_service_client.h"
#include "pbl/util/trig.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "system/passert.h"

#include "kernel/kernel_applib_state.h"
#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"

#define PEEK_TIMEOUT_MS      11 * 1000

static CompassServiceConfig **prv_get_config(PebbleTask task) {
  CompassServiceConfig **config = NULL;

  if (task == PebbleTask_Unknown) {
    task = pebble_task_get_current();
  }

  if (task == PebbleTask_App) {
    config = app_state_get_compass_config();
  } else if (task == PebbleTask_Worker) {
    config = worker_state_get_compass_config();
  } else if (task == PebbleTask_KernelMain) {
    config = kernel_applib_get_compass_config();
  } else {
    WTF;
  }

  if (*config == NULL) {
    // Note that config will never be NULL after grabbing it from an app,
    // worker, or kernel state.  However, the value pointed to it may be
    // NULL.
    *config = task_zalloc_check(sizeof(CompassServiceConfig));
  }

  return config;
}

static void prv_do_data_handle(PebbleEvent *e, void *context) {
  PebbleCompassDataEvent *m = &e->compass_data;

  CompassHeadingData data = {
    .is_declination_valid = false,
    .compass_status = m->calib_status,
    .magnetic_heading = m->magnetic_heading,
    .true_heading = m->magnetic_heading
  };

  CompassServiceConfig *config = *prv_get_config(PebbleTask_Unknown);
  if (config->compass_cb != NULL) {
    if (ABS(config->last_angle - data.magnetic_heading) > config->compass_filter) {
      config->compass_cb(data);
      config->last_angle = data.magnetic_heading;
    }
  }
}

// Callback for disabling compass service on timeout
static void prv_peek_timeout_callback(void *data) {
  compass_service_unsubscribe();
}

int compass_service_peek(CompassHeadingData *data) {
  CompassServiceConfig *config = *prv_get_config(PebbleTask_Unknown);
  if ((config->peek_timer == NULL) && !sys_ecompass_service_subscribed()) {
    // If we haven't initialized the compass yet by subscribing, do that now.
    compass_service_subscribe(NULL);
  }

  sys_ecompass_get_last_heading(data);

  if (data->is_declination_valid) {
    data->true_heading += config->heading_declination;
  }

  // 11 second timer to turn off compass, reset timeout every peek
  if (config->peek_timer == NULL) {
    config->peek_timer = app_timer_register(PEEK_TIMEOUT_MS,
        prv_peek_timeout_callback, NULL);
  } else {
    app_timer_reschedule(config->peek_timer, PEEK_TIMEOUT_MS);
  }

  return (0);
}

int compass_service_set_heading_filter(CompassHeading filter) {
  if ((filter < 0) || (filter > (TRIG_MAX_ANGLE / 2))) {
    return (-1);
  }

  CompassServiceConfig *config = *prv_get_config(PebbleTask_Unknown);
  config->compass_filter = filter;
  return (0);
}

void compass_service_subscribe(CompassHeadingHandler handler) {
  CompassServiceConfig *config = *prv_get_config(PebbleTask_Unknown);

  *config = (const CompassServiceConfig){ 0 };
  config->compass_cb = handler;

  config->info = (EventServiceInfo) {
    .type = PEBBLE_COMPASS_DATA_EVENT,
    .handler = &prv_do_data_handle
  };

  event_service_client_subscribe(&config->info);
}

void compass_service_unsubscribe(void) {
  CompassServiceConfig **config = prv_get_config(PebbleTask_Unknown);
  event_service_client_unsubscribe(&(*config)->info);
  task_free(*config);
  *config = NULL;
}
