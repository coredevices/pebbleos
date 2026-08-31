/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "unobstructed_area_service.h"
#include "unobstructed_area_service_private.h"

#include "applib/app.h"
#include "applib/graphics/framebuffer.h"
#include "kernel/events.h"
#include "process_state/app_state/app_state.h"
#include "system/passert.h"

static void prv_handle_unobstructed_area_event(PebbleEvent *event, void *context);
static void prv_origin_y_to_area(int16_t origin_y, GRect *area_out);

void unobstructed_area_service_init(UnobstructedAreaState *state, int16_t current_y) {
  PBL_ASSERTN(state);
  *state = (UnobstructedAreaState) {};
  prv_origin_y_to_area(current_y, &state->area);
  state->event_info = (EventServiceInfo) {
    .type = PEBBLE_UNOBSTRUCTED_AREA_EVENT,
    .handler = prv_handle_unobstructed_area_event,
    .context = state,
  };
  event_service_client_subscribe(&state->event_info);
}

void unobstructed_area_service_deinit(UnobstructedAreaState *state) {
  PBL_ASSERTN(state);
  event_service_client_unsubscribe(&state->event_info);
}

static void prv_put_area_event(UnobstructedAreaEventType type, int16_t current_y,
                               int16_t final_y, AnimationProgress progress) {
  PebbleEvent event = {
    .type = PEBBLE_UNOBSTRUCTED_AREA_EVENT,
    .unobstructed_area = {
      .type = type,
      .current_y = current_y,
      .final_y = final_y,
      .progress = progress,
    },
  };
  event_put(&event);
}

static void prv_clip_area(GRect *area_out) {
  PBL_ASSERTN(area_out);
  const GRect display_frame = { .size = framebuffer_get_size(app_state_get_framebuffer()) };
  grect_clip(area_out, &display_frame);
}

//! Currently, the unobstructed area is derived from the origin of the obstruction.
//! This is equivalent to the height of the unobstructed area.
static void prv_origin_y_to_area(int16_t origin_y, GRect *area_out) {
  PBL_ASSERTN(area_out);
  *area_out = (GRect) {
    .size = { DISP_COLS, origin_y },
  };
  prv_clip_area(area_out);
}

void unobstructed_area_service_will_change(int16_t current_y, int16_t final_y) {
  prv_put_area_event(UnobstructedAreaEventType_WillChange, current_y, final_y,
                     ANIMATION_NORMALIZED_MIN);
}

void unobstructed_area_service_change(int16_t current_y, int16_t final_y,
                                      AnimationProgress progress) {
  prv_put_area_event(UnobstructedAreaEventType_Change, current_y, final_y, progress);
}

void unobstructed_area_service_did_change(int16_t final_y) {
  prv_put_area_event(UnobstructedAreaEventType_DidChange, final_y, final_y,
                     ANIMATION_NORMALIZED_MAX);
}

static void prv_save_event_area(UnobstructedAreaState *state, PebbleEvent *event) {
  GRect area;
  prv_origin_y_to_area(event->unobstructed_area.current_y, &area);
  state->area = area;
  prv_clip_area(&state->area);
}

static void prv_call_will_change(UnobstructedAreaState *state, PebbleEvent *event) {
  if (state->is_changing) {
    return;
  }
  // Always call the will handler even if the app restarts or starts out of sync with
  // the animation and has had its app state reinitialized. Set `is_changing` to keep track.
  state->is_changing = true;
  if (state->handlers.will_change) {
    GRect final_area;
    prv_origin_y_to_area(event->unobstructed_area.final_y, &final_area);
    state->handlers.will_change(final_area, state->context);
  }
}

static void prv_handle_will_change_event(PebbleEvent *event, void *context) {
  UnobstructedAreaState *state = context;
  // It is the producer's responsibility not to overlap unobstructed area changes.
  PBL_ASSERTN(!state->is_changing);
  prv_call_will_change(state, event);
}

static void prv_handle_change_event(PebbleEvent *event, void *context) {
  UnobstructedAreaState *state = context;
  prv_call_will_change(state, event);
  if (state->handlers.change) {
    state->handlers.change(event->unobstructed_area.progress, state->context);
  }
}

static void prv_handle_did_change_event(PebbleEvent *event, void *context) {
  UnobstructedAreaState *state = context;
  prv_call_will_change(state, event);
  state->is_changing = false;
  if (state->handlers.did_change) {
    state->handlers.did_change(state->context);
  }
}

static void prv_handle_unobstructed_area_event(PebbleEvent *event, void *context) {
  UnobstructedAreaState *state = context;
  GRect previous_area = state->area;
  prv_save_event_area(state, event);
  if (!grect_equal(&previous_area, &state->area)) {
    app_request_render();
  }
  switch (event->unobstructed_area.type) {
    case UnobstructedAreaEventType_WillChange:
      prv_handle_will_change_event(event, context);
      break;
    case UnobstructedAreaEventType_Change:
      prv_handle_change_event(event, context);
      break;
    case UnobstructedAreaEventType_DidChange:
      prv_handle_did_change_event(event, context);
      break;
  }
}

void unobstructed_area_service_subscribe(UnobstructedAreaState *state,
                                         const UnobstructedAreaHandlers *handlers,
                                         void *context) {
  PBL_ASSERTN(state && handlers);
  state->handlers = *handlers;
  state->context = context;
  state->is_subscribed = true;
}

void unobstructed_area_service_unsubscribe(UnobstructedAreaState *state) {
  PBL_ASSERTN(state);
  state->handlers = (UnobstructedAreaHandlers) {};
  state->context = NULL;
  state->is_subscribed = false;
}

void unobstructed_area_service_get_area(UnobstructedAreaState *state, GRect *area_out) {
  if (state && area_out) {
    state->has_requested_area = true;
    *area_out = state->area;
  }
}

bool unobstructed_area_service_is_subscribed(UnobstructedAreaState *state) {
  return state ? state->is_subscribed : false;
}

bool unobstructed_area_service_has_requested_area(UnobstructedAreaState *state) {
  return state ? state->has_requested_area : false;
}

void app_unobstructed_area_service_subscribe(UnobstructedAreaHandlers handlers, void *context) {
  unobstructed_area_service_subscribe(app_state_get_unobstructed_area_state(), &handlers, context);
}

void app_unobstructed_area_service_unsubscribe(void) {
  unobstructed_area_service_unsubscribe(app_state_get_unobstructed_area_state());
}
