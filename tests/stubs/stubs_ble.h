/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once 

#include "stubs_bluetooth_pairing_ui.h"
#include "stubs_events.h"
#include "stubs_hexdump.h"

#include <inttypes.h>
#include <stdbool.h>

bool gaps_init(void) {
  return true;
}

bool gaps_deinit(uint32_t stack_id) {
  return true;
}

void comm_handle_paired_devices_changed(void) {
}
