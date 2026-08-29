/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

#include "util/time/time.h"

#define BLE_HRM_REMINDER_POPUP_DELAY_MINS (2 * MINUTES_PER_HOUR)

typedef struct GAPLEConnection GAPLEConnection;

typedef struct BLEHRMSharingRequest BLEHRMSharingRequest;

//! Called by the ble_hrm_sharing_popup upon the user's action to grant or decline the sharing.
//! @note Also cleans up the sharing_request object.
void ble_hrm_handle_sharing_request_response(bool is_granted,
                                             BLEHRMSharingRequest *sharing_request);

bool ble_hrm_is_supported_and_enabled(void);

bool ble_hrm_is_sharing_to_connection(const GAPLEConnection *connection);

bool ble_hrm_is_sharing(void);

void ble_hrm_revoke_sharing_permission_for_connection(GAPLEConnection *connection);

void ble_hrm_revoke_all(void);

void ble_hrm_handle_ble_hrm_sharing_enabled(bool is_enabled);

void ble_hrm_handle_disconnection(GAPLEConnection *connection);

void ble_hrm_init(void);

void ble_hrm_deinit(void);
