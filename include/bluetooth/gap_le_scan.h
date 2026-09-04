/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "comm/ble/gap_le_scan.h"

//! Returns true on success, false on failure
bool bt_driver_start_le_scan(bool active_scan, bool use_white_list_filter, bool filter_dups,
                         uint16_t scan_interval_ms, uint16_t scan_window_ms);

//! Returns true on success, false on failure
bool bt_driver_stop_le_scan(void);

extern void bt_driver_cb_le_scan_handle_report(const GAPLERawAdReport *data, int length);
