/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "fake_events.h"
#include "kernel/pbl_malloc.h"

#include "projdefs.h"

static PebbleEvent s_last_pebble_event;
static uint32_t s_fake_event_count = 0;
static FakeEventCallback s_fake_event_cb = NULL;

WEAK void **fake_event_get_buffer(PebbleEvent *event) {
  switch (event->type) {
    case PEBBLE_BLE_GATT_CLIENT_EVENT:
      if (event->bluetooth.le.gatt_client.subtype == PebbleBLEGATTClientEventTypeServiceChange) {
        return (void **)(&event->bluetooth.le.gatt_client_service.info);
      }
      break;

    default:
      break; // Nothing to do!
  }
  return NULL;
}

void event_put(PebbleEvent* event) {
  fake_event_clear_last();
  s_last_pebble_event = *event;
  ++s_fake_event_count;
  if (s_fake_event_cb) {
    s_fake_event_cb(event);
  }
}

bool event_put_isr(PebbleEvent* event) {
  return false;
}

struct pbl_msgq *event_kernel_to_kernel_event_queue(void) {
  return NULL;
}

void event_queue_cleanup_and_reset(struct pbl_msgq *queue) {
}

void fake_event_init(void) {
  fake_event_reset_count();
  fake_event_clear_last();
}

PebbleEvent fake_event_get_last(void) {
  return s_last_pebble_event;
}

void fake_event_clear_last(void) {
  void **buf = fake_event_get_buffer(&s_last_pebble_event);
  if (buf && *buf) {
    kernel_free(*buf);
    *buf = NULL;
  }

  s_last_pebble_event = (PebbleEvent) {};
}

void fake_event_reset_count(void) {
  s_fake_event_count = 0;
}

uint32_t fake_event_get_count(void) {
  return s_fake_event_count;
}

void fake_event_set_callback(FakeEventCallback cb) {
  s_fake_event_cb = cb;
}
