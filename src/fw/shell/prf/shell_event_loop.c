/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "shell/shell_event_loop.h"

#include "popups/bluetooth_pairing_ui.h"

#include "pbl/services/idle_watchdog.h"

void shell_event_loop_init(void) {
  prf_idle_watchdog_init();
  prf_idle_watchdog_start();
}

void shell_event_loop_handle_event(PebbleEvent *e) {
  switch(e->type) {
  case PEBBLE_BT_PAIRING_EVENT:
    bluetooth_pairing_ui_handle_event(&e->bluetooth.pair);
    return;

  default:
    return;
  }
}

