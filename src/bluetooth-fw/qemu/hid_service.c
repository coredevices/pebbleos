/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/hid_service.h>

bool bt_driver_is_hid_service_supported(void) {
  return false;
}

bool bt_driver_hid_service_send_consumer_report(uint8_t bits) {
  return false;
}

bool bt_driver_hid_service_is_subscribed(void) {
  return false;
}

bool bt_driver_hid_service_send_consumer_usage(uint16_t usage) {
  return false;
}

bool bt_driver_hid_service_usage_is_subscribed(void) {
  return false;
}
