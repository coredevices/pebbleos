/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "accel_service.h"

#include "accel_service_private.h"
#include "applib/applib_malloc.auto.h"
#include "event_service_client.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"
#include "pbl/services/accel_manager.h"
#include "syscall/syscall.h"
#include "system/passert.h"

#include "FreeRTOS.h"

static bool prv_is_session_task(void) {
  PebbleTask task = pebble_task_get_current();
  return (task == PebbleTask_KernelMain || task == PebbleTask_KernelBackground ||
          task == PebbleTask_App);
}

// ------------------------------------------------------------------------------------------
// Assert that the current task is allowed to create/delete a session
static void prv_assert_session_task(void) {
  PBL_ASSERTN(prv_is_session_task());
}


// --------------------------------------------------------------------------------------------
// Return the session ref for the given task. This should ONLY be used by 3rd party tasks
// (app or worker).
AccelServiceState * accel_service_private_get_session(PebbleTask task) {
  if (task == PebbleTask_Unknown) {
    task = pebble_task_get_current();
  }

  if (task == PebbleTask_App) {
    return app_state_get_accel_state();
  } else if (task == PebbleTask_Worker) {
    return worker_state_get_accel_state();
  } else {
    WTF;
  }
}

void accel_service_cleanup_task_session(PebbleTask task) {
  AccelServiceState *state = accel_service_private_get_session(task);
  if (state->manager_state) {
    sys_accel_manager_data_unsubscribe(state->manager_state);
    state->manager_state = NULL;
  }
}


// ----------------------------------------------------------------------------------------------
// Event service handler for tap events
static void prv_do_shake_handle(PebbleEvent *e, void *context) {
  PebbleTask task = pebble_task_get_current();
  AccelServiceState *state = (AccelServiceState *)accel_service_private_get_session(task);
  PBL_ASSERTN(state->shake_handler != NULL);

  state->shake_handler((AccelAxisType)e->accel_tap.axis, e->accel_tap.direction);
}


// ----------------------------------------------------------------------------------------------
static void prv_do_double_tap_handle(PebbleEvent *e, void *context) {
  PebbleTask task = pebble_task_get_current();
  AccelServiceState *state = (AccelServiceState *)accel_service_private_get_session(task);
  PBL_ASSERTN(state->double_tap_handler != NULL);
  // only kernel clients can subscribe to double tap right now, so just increment double tap count
  // device analytic here
  state->double_tap_handler((AccelAxisType)e->accel_tap.axis, e->accel_tap.direction);
}


// -----------------------------------------------------------------------------------------------
// Handle a chunk of data received for a data subscription. Called by prv_do_data_handle.
static uint32_t prv_do_data_handle_chunk(AccelServiceState *state, uint16_t time_interval_ms) {
  uint32_t num_samples = 0;

  uint64_t timestamp_ms;
  num_samples = sys_accel_manager_get_num_samples(state->manager_state, &timestamp_ms);
  if (num_samples < state->samples_per_update) {
    return 0;
  }

  uint32_t time_since_last_sample =
      (state->prev_timestamp_ms != 0) ? timestamp_ms - state->prev_timestamp_ms : 0;
  state->prev_timestamp_ms = timestamp_ms;

  PBL_LOG_VERBOSE("got %d samples for task %d at %ld (%lu ms delta)",
                  (int)num_samples, (int)pebble_task_get_current(), (uint32_t)timestamp_ms,
                  time_since_last_sample);

  for (unsigned int i=0; i<num_samples; i++) {
    PBL_LOG_VERBOSE("  => x:%d, y:%d, z:%d", state->raw_data[i].x, state->raw_data[i].y, state->raw_data[i].z);
  }

  if (state->raw_data_handler_deprecated) {
    state->raw_data_handler_deprecated(state->raw_data, num_samples);

  } else if (state->raw_data_handler) {
    state->raw_data_handler(state->raw_data, num_samples, timestamp_ms);

  } else {
    AccelData data[num_samples];
    for (uint32_t i = 0; i < num_samples; i++) {
      data[i] = (AccelData) {
        .x = state->raw_data[i].x,
        .y = state->raw_data[i].y,
        .z = state->raw_data[i].z,
        .timestamp = timestamp_ms,
        .did_vibrate = sys_vibe_history_was_vibrating(timestamp_ms)
      };
      timestamp_ms += time_interval_ms;
    }
    state->data_handler(data, num_samples);
  }

  // Tell accel_manager that it can put more data in now
  bool success = sys_accel_manager_consume_samples(state->manager_state, num_samples);
  PBL_ASSERTN(success);
  return num_samples;
}


// ---------------------------------------------------------------------------------------------
// Called by sys_accel_manager when we have data available for this subscriber
static void prv_do_data_handle(void *context) {
  AccelServiceState *state = (AccelServiceState *)context;

  if (state->manager_state == NULL) {
    if (state->deferred_free) {
      PBL_LOG_DBG("Deferred free");
      kernel_free(state);
    }
    // event queue is handled kernel-side, so an event may fire after we've unsubscribed
    return;
  }

  PBL_ASSERTN(state->data_handler != NULL || state->raw_data_handler != NULL
              || state->raw_data_handler_deprecated != NULL);

  uint16_t time_interval_ms = 1000 / state->sampling_rate;

  // Process in chunks to limit the amount of stack space we use up.
  uint32_t num_processed;
  do {
    num_processed = prv_do_data_handle_chunk(state, time_interval_ms);
  } while (num_processed);

}


// -----------------------------------------------------------------------------------------------
int accel_service_set_sampling_rate(AccelSamplingRate rate) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  return accel_session_set_sampling_rate(session, rate);
}


// ----------------------------------------------------------------------------------------------
int accel_service_set_samples_per_update(uint32_t samples_per_update) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  return accel_session_set_samples_per_update(session, samples_per_update);
}


// ----------------------------------------------------------------------------------------------
static void prv_shared_subscribe(AccelServiceState *state, AccelSamplingRate sampling_rate,
                                 uint32_t samples_per_update, PebbleTask handler_task) {
  state->manager_state = sys_accel_manager_data_subscribe(
      sampling_rate, prv_do_data_handle, state, handler_task);

  accel_session_set_samples_per_update((AccelServiceState *)state, samples_per_update);
}


// ----------------------------------------------------------------------------------------------
void accel_data_service_subscribe(uint32_t samples_per_update, AccelDataHandler handler) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  accel_session_data_subscribe(session, samples_per_update, handler);
}


// ----------------------------------------------------------------------------------------------
void accel_raw_data_service_subscribe(uint32_t samples_per_update, AccelRawDataHandler handler) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  accel_session_raw_data_subscribe(session, ACCEL_SAMPLING_25HZ, samples_per_update, handler);
}


// ----------------------------------------------------------------------------------------------
void accel_data_service_subscribe__deprecated(uint32_t samples_per_update, AccelRawDataHandler__deprecated handler) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  AccelServiceState *state = (AccelServiceState *)session;

  state->raw_data_handler_deprecated = handler;
  state->raw_data_handler = NULL;
  state->data_handler = NULL;

  prv_shared_subscribe(state, ACCEL_SAMPLING_25HZ, samples_per_update, pebble_task_get_current());
}


// ----------------------------------------------------------------------------------------------
void accel_data_service_unsubscribe(void) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  accel_session_data_unsubscribe(session);
}


// ----------------------------------------------------------------------------------------------
void accel_tap_service_subscribe(AccelTapHandler handler) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  accel_session_shake_subscribe(session, handler);
}


// ----------------------------------------------------------------------------------------------
void accel_tap_service_unsubscribe(void) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  accel_session_shake_unsubscribe(session);
}


// ----------------------------------------------------------------------------------------------
void accel_double_tap_service_subscribe(AccelTapHandler handler) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  accel_session_double_tap_subscribe(session, handler);
}


// ----------------------------------------------------------------------------------------------
void accel_double_tap_service_unsubscribe(void) {
  AccelServiceState * session = accel_service_private_get_session(PebbleTask_Unknown);
  accel_session_double_tap_unsubscribe(session);
}


// ----------------------------------------------------------------------------------------------
int accel_service_peek(AccelData *accel_data) {
  AccelServiceState *state = accel_service_private_get_session(PebbleTask_Unknown);

  int rc = sys_accel_manager_peek(accel_data);

  PBL_LOG_VERBOSE("peek data x:%d, y:%d, z:%d", accel_data->x, accel_data->y, accel_data->z);
  if (rc != 0 || state->raw_data_handler_deprecated || state->raw_data_handler) {
    // No timestamp info needed
    return rc;
  }

  accel_data->did_vibrate = (sys_vibe_get_vibe_strength() != 0);
  return rc;
}


// ----------------------------------------------------------------------------------------------
void accel_service_state_init(AccelServiceState *state) {
  *state = (AccelServiceState) {
    .sampling_rate = ACCEL_DEFAULT_SAMPLING_RATE,
    .accel_shake_info = {
        .type = PEBBLE_ACCEL_SHAKE_EVENT,
        .handler = &prv_do_shake_handle,
    },
    .accel_double_tap_info = {
        .type = PEBBLE_ACCEL_DOUBLE_TAP_EVENT,
        .handler = &prv_do_double_tap_handle,
    }
  };
}


// ----------------------------------------------------------------------------------------------
// Event service handler for shake events
static void prv_session_do_shake_handle(PebbleEvent *e, void *context) {
  AccelServiceState *state = context;
  if (state->shake_handler != NULL) {
    state->shake_handler((AccelAxisType)e->accel_tap.axis, e->accel_tap.direction);
  }
}


// ----------------------------------------------------------------------------------------------
// Event service handler for double tap events
static void prv_session_do_double_tap_handle(PebbleEvent *e, void *context) {
  AccelServiceState *state = context;
  if (state->double_tap_handler != NULL) {
    state->double_tap_handler((AccelAxisType)e->accel_tap.axis, e->accel_tap.direction);
  }
}


// -----------------------------------------------------------------------------------------------
AccelServiceState * accel_session_create(void) {
  prv_assert_session_task();
  AccelServiceState *state = kernel_malloc_check(sizeof(AccelServiceState));

  *state = (AccelServiceState) {
    .sampling_rate = ACCEL_DEFAULT_SAMPLING_RATE,
    .accel_shake_info = {
        .type = PEBBLE_ACCEL_SHAKE_EVENT,
        .handler = &prv_session_do_shake_handle,
        .context = state,
    },
    .accel_double_tap_info = {
        .type = PEBBLE_ACCEL_DOUBLE_TAP_EVENT,
        .handler = &prv_session_do_double_tap_handle,
        .context = state,
    },
  };
  return state;
}


// -----------------------------------------------------------------------------------------------
void accel_session_delete(AccelServiceState * session) {
  prv_assert_session_task();

  // we better have unsubscribed at this point
  PBL_ASSERTN(session->manager_state == NULL);

  // A deferred free means one lingering event was posted. We will free the session once the event
  // gets drained in 'prv_do_data_handle'
  if (!session->deferred_free) {
    kernel_free(session);
  }
}


// ----------------------------------------------------------------------------------------------
void accel_session_shake_subscribe(AccelServiceState * session, AccelTapHandler handler) {
  AccelServiceState *state = (AccelServiceState *)session;
  state->shake_handler = handler;
  event_service_client_subscribe(&state->accel_shake_info);
}


// ----------------------------------------------------------------------------------------------
void accel_session_shake_unsubscribe(AccelServiceState *state) {
  event_service_client_unsubscribe(&state->accel_shake_info);
  state->shake_handler = NULL;
}


// -----------------------------------------------------------------------------------------------
void accel_session_double_tap_subscribe(AccelServiceState *state, AccelTapHandler handler) {
  state->double_tap_handler = handler;
  event_service_client_subscribe(&state->accel_double_tap_info);
}


// -----------------------------------------------------------------------------------------------
void accel_session_double_tap_unsubscribe(AccelServiceState *state) {
  event_service_client_unsubscribe(&state->accel_double_tap_info);
  state->double_tap_handler = NULL;
}


// -----------------------------------------------------------------------------------------------
void accel_session_data_subscribe(AccelServiceState *state, uint32_t samples_per_update,
                                  AccelDataHandler handler) {
  state->data_handler = handler;
  state->raw_data_handler = NULL;
  state->raw_data_handler_deprecated = NULL;

  prv_shared_subscribe(state, ACCEL_SAMPLING_25HZ, samples_per_update, pebble_task_get_current());
}


// -----------------------------------------------------------------------------------------------
void accel_session_raw_data_subscribe(
    AccelServiceState *state, AccelSamplingRate sampling_rate, uint32_t samples_per_update,
    AccelRawDataHandler handler) {
  state->raw_data_handler = handler;
  state->raw_data_handler_deprecated = NULL;
  state->data_handler = NULL;

  prv_shared_subscribe(state, sampling_rate, samples_per_update, pebble_task_get_current());
}


// -----------------------------------------------------------------------------------------------
void accel_session_data_unsubscribe(AccelServiceState *state) {
  if (!state->manager_state) {
    return;
  }
  if (sys_accel_manager_data_unsubscribe(state->manager_state)) {
    // There is a pending event posted. Only session tasks allocate memory for their state in the
    // first place so only free the memory if this is true
    state->deferred_free = prv_is_session_task();
  }

  applib_free(state->raw_data);
  state->manager_state = NULL;
  state->raw_data = NULL;
  state->data_handler = NULL;
  state->raw_data_handler = NULL;
  state->raw_data_handler_deprecated = NULL;
}


// -----------------------------------------------------------------------------------------------
int accel_session_set_sampling_rate(AccelServiceState *state, AccelSamplingRate rate) {
  if (!state->manager_state || (!state->data_handler && !state->raw_data_handler
      && !state->raw_data_handler_deprecated)) {
    return -1;
  }
  state->sampling_rate = rate;
  return sys_accel_manager_set_sampling_rate(state->manager_state, rate);
}


// -----------------------------------------------------------------------------------------------
int accel_session_set_samples_per_update(AccelServiceState *state, uint32_t samples_per_update) {

  uint32_t max_samples_per_update = sys_accel_manager_get_max_samples_per_update();
  if (samples_per_update > max_samples_per_update) {
    APP_LOG(LOG_LEVEL_WARNING, "%d samples per update requested, max is %d",
            (int)samples_per_update, (int)max_samples_per_update);
    samples_per_update = max_samples_per_update;
  }
  if (!state->manager_state
      || (samples_per_update > 0 && !state->data_handler && !state->raw_data_handler
          && !state->raw_data_handler_deprecated)) {
    return -1;
  }
  AccelRawData *old_buf = state->raw_data;

  // This is a packed array of simple types and therefore shouldn't have compatibility padding
  state->raw_data = applib_malloc(samples_per_update * sizeof(AccelRawData));
  if (!state->raw_data) {
    APP_LOG(LOG_LEVEL_ERROR, "Not enough memory to subscribe");
    state->raw_data = old_buf;
    return -1;
  }
  state->samples_per_update = samples_per_update;
  int result = sys_accel_manager_set_sample_buffer(state->manager_state, state->raw_data,
                                                   samples_per_update);
  applib_free(old_buf);
  return result;
}
