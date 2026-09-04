/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "ble_scan.h"

#include "applib/app_logging.h"
#include "applib/applib_malloc.auto.h"

#include "process_state/app_state/app_state.h"
#include "comm/ble/gap_le_scan.h"

#include "kernel/events.h"

#include "syscall/syscall.h"


void ble_scan_handle_event(PebbleEvent *e) {
  BLEAppState *ble_app_state = app_state_get_ble_app_state();
  if (!ble_app_state->scan_handler) {
    return;
  }

  // Use the same buffer size as the kernel itself:
  uint8_t *buffer = (uint8_t *) applib_malloc(GAP_LE_SCAN_REPORTS_BUFFER_SIZE);
  if (!buffer) {
    APP_LOG(LOG_LEVEL_ERROR, "Need %u bytes of heap for ble_scan_start()",
            GAP_LE_SCAN_REPORTS_BUFFER_SIZE);
    return;
  }
  uint16_t size = GAP_LE_SCAN_REPORTS_BUFFER_SIZE;
  sys_ble_consume_scan_results(buffer, &size);

  if (size == 0) {
    goto finally;
  }

  // Walk all the reports in the buffer:
  const uint8_t *cursor = buffer;
  while (cursor < buffer + size) {
    const GAPLERawAdReport *report = (GAPLERawAdReport *)cursor;

    const BTDeviceInternal device = (const BTDeviceInternal) {
      .address = report->address.address,
      .is_classic = false,
      .is_random_address = report->is_random_address,
    };

    // Call the scan handler for each report:
    ble_app_state->scan_handler(device.opaque, report->rssi, &report->payload);

    const size_t report_length = sizeof(GAPLERawAdReport) +
                                    report->payload.ad_data_length +
                                    report->payload.scan_resp_data_length;
    cursor += report_length;
  }

finally:
  applib_free(buffer);
}

BTErrno ble_scan_start(BLEScanHandler handler) {
  if (!handler) {
    return (BTErrnoInvalidParameter);
  }
  BLEAppState *ble_app_state = app_state_get_ble_app_state();
  if (ble_app_state->scan_handler) {
    return (BTErrnoInvalidState);
  }
  const bool result = sys_ble_scan_start();
  if (!result) {
    return BTErrnoOther;
  }
  ble_app_state->scan_handler = handler;
  event_service_client_subscribe(&ble_app_state->scan_service_info);
  return BTErrnoOK;
}

BTErrno ble_scan_stop(void) {
  BLEAppState *ble_app_state = app_state_get_ble_app_state();
  if (!ble_app_state->scan_handler) {
    return (BTErrnoInvalidState);
  }
  const bool result = sys_ble_scan_stop();
  if (!result) {
    return BTErrnoOther;
  }
  event_service_client_unsubscribe(&ble_app_state->scan_service_info);
  ble_app_state->scan_handler = NULL;
  return BTErrnoOK;
}

bool ble_scan_is_scanning(void) {
  return sys_ble_scan_is_scanning();
}
