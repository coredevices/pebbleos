/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/bt_driver_ppog_reversed.h>
#include <bluetooth/gatt.h>

BTErrno bt_driver_gatt_write_without_response(GAPLEConnection *connection,
                                              const uint8_t *value,
                                              size_t value_length,
                                              uint16_t att_handle) {
  return 0;
}

BTErrno bt_driver_gatt_write(GAPLEConnection *connection,
                             const uint8_t *value,
                             size_t value_length,
                             uint16_t att_handle,
                             void *context) {
  return 0;
}

BTErrno bt_driver_gatt_read(GAPLEConnection *connection,
                            uint16_t att_handle,
                            void *context) {
  return 0;
}

BTErrno bt_driver_ppog_reversed_notify(uint16_t conn_handle, const uint8_t *buf, uint16_t len) {
  return BTErrnoInvalidState;
}
