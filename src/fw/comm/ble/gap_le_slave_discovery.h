/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! @file gap_le_slave_discovery.h
//! This sub-module is responsible for advertising explicitely for device
//! discovery purposes. The advertisement will contain the device name,
//! transmit power level (to be able to order devices by estimated proximity),
//! Pebble Service UUID and discoverability flags.
//! Advertising devices will implicitely become the slave when being connected
//! to, so the "slave" part in the file name is redundant, but kept for
//! the sake of completeness.

#include <stdbool.h>

//! @return True is Pebble is currently explicitely discoverable as BLE slave
//! or false if not.
bool gap_le_slave_is_discoverable(void);

//! @param discoverable True to make Pebble currently explicitely discoverable
//! as BLE slave. Initially, Pebble will advertise at a relatively high rate for
//! a few seconds. After this, the rate will drop to save battery life.
void gap_le_slave_set_discoverable(bool discoverable);

//! Initializes the gap_le_slave_discovery module.
void gap_le_slave_discovery_init(void);

//! De-Initializes the gap_le_slave_discovery module.
void gap_le_slave_discovery_deinit(void);
