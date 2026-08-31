/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! Handler called by qemu_serial driver when we receive a QemuProtocol_Battery message
//!  over the qemu serial connection.
void qemu_battery_msg_callack(const uint8_t *data, uint32_t len);

//! Returns the exact percent last set via `pebble emu-battery --percent N`.
//! Lets battery_state.c bypass the lossy voltage-curve roundtrip on QEMU.
uint8_t qemu_battery_get_percent(void);
