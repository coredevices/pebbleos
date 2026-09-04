/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! @file gap_le_slave_reconnect.h
//! Sub-system that will start advertising for reconnection, whenever there is a
//! bonded master device AND the local device is not already connected as slave.
//! The interface of the sub-system is merely a set of handlers to respond to
//! changes in slave connectivity and bonding.

//! Stops advertising for reconnection. For example, for when a connection to a
//! master gets established (only one master allowed in BT 4.0)
void gap_le_slave_reconnect_stop(void);

//! Start advertising for reconnection, but only if there is a bonded master
//! device. Otherwise, this is a no-op. In case the sub-system is already
//! advertising for reconnection, this function is a no-op.
//! Events for which this function should be called:
//! - When a connection to a master is lost
//! - When the list of bonded devices changes
//! - When Bluetooth is turned on
void gap_le_slave_reconnect_start(void);

#ifdef CONFIG_HRM

//! Start advertising for reconnection using a payload containing the Heart Rate Service UUID.
//! It will automatically stop after 60 seconds, in case gap_le_slave_reconnect_hrm_stop() is not
//! called sooner.
void gap_le_slave_reconnect_hrm_restart(void);

//! Stop advertising for reconnection using a payload containing the Heart Rate Service UUID.
//! This is a no-op when not advertising for HRM reconnection.
void gap_le_slave_reconnect_hrm_stop(void);

#endif
