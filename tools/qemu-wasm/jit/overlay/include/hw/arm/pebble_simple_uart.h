/*
 * Pebble Simple UART — public API
 *
 * Functions for integrating pebble_control with the simple UART.
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_PEBBLE_SIMPLE_UART_H
#define HW_ARM_PEBBLE_SIMPLE_UART_H

#include "hw/core/qdev.h"
#include "chardev/char-fe.h"
#include "qemu/main-loop.h"

/* Write handler callback type — intercepts firmware UART TX */
typedef int (*PblUartWriteHandler)(void *opaque, const uint8_t *buf, int len);

/* Set a write handler that intercepts firmware TX data.
 * When set, TX bytes go to handler instead of chardev. */
void pbl_uart_set_write_handler(DeviceState *dev, void *opaque,
                                PblUartWriteHandler handler);

/* Get the UART's RX handler functions (for forwarding data to firmware).
 * These match the IOCanReadHandler/IOReadHandler/IOEventHandler signatures. */
void pbl_uart_get_rcv_handlers(DeviceState *dev,
                               IOCanReadHandler **can_read,
                               IOReadHandler **read,
                               IOEventHandler **event);

/* Inject RX data into the UART as if it came from the chardev. */
void pbl_uart_inject_rx(DeviceState *dev, const uint8_t *buf, int size);

#endif /* HW_ARM_PEBBLE_SIMPLE_UART_H */
