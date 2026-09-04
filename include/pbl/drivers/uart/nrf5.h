/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/uart.h>

#include "board/board.h"

#include <stdbool.h>
#include <stdint.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <nrfx_uarte.h>
#pragma GCC diagnostic pop

typedef struct UARTState {
  bool initialized;
  UARTRXInterruptHandler rx_irq_handler;
  UARTTXInterruptHandler tx_irq_handler;
  bool rx_int_enabled;
  bool tx_int_enabled;
  bool rx_done_pending;
  uint8_t *rx_dma_buffer;
  uint32_t rx_dma_length;
  uint32_t rx_dma_index;
  uint32_t rx_prod_index;
  uint32_t rx_cons_index;
  uint32_t rx_cons_pos;
  uint32_t tx_cache_buffer[8];
  uint32_t rx_cache_buffer[8];
  } UARTDeviceState;

typedef const struct UARTDevice {
  UARTDeviceState *state;
  bool half_duplex;
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  uint32_t rts_gpio;
  uint32_t cts_gpio;
  nrfx_uarte_t periph;
  nrfx_timer_t counter;
} UARTDevice;

// thinly wrapped by the IRQ handler in board_*.c
void uart_irq_handler(UARTDevice *dev);
