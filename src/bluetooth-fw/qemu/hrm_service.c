/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/hrm_service.h>

bool bt_driver_is_hrm_service_supported(void) {
  return false;
}

void bt_driver_hrm_service_enable(bool enable) {
}

void bt_driver_hrm_service_handle_measurement(const BleHrmServiceMeasurement *measurement,
                                              const BTDeviceInternal *permitted_devices,
                                              size_t num_permitted_devices) {
}
