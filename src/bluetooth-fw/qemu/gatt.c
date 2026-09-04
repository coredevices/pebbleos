/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/gatt.h>

void bt_driver_gatt_respond_read_subscription(uint32_t transaction_id, uint16_t response_code) {
}

void bt_driver_gatt_send_changed_indication(const BTDeviceInternal *device,
                                            const ATTHandleRange *data) {
}
