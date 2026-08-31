/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "dbgserial_input.h"

#include "board/board.h"
#include <pbl/drivers/uart.h>

#if !defined(CONFIG_RELEASE) || defined(CONFIG_MFG)

static DbgSerialCharacterCallback s_character_callback;

//! We DMA into this buffer as a circular buffer
#define DMA_BUFFER_LENGTH (200)
static uint8_t s_dma_buffer[DMA_BUFFER_LENGTH] __attribute__((aligned(4)));
static bool s_dma_enabled = false;

static bool prv_uart_irq_handler(UARTDevice *dev, uint8_t data, const UARTRXErrorFlags *err_flags) {
  bool should_context_switch = false;
  if (s_character_callback) {
    s_character_callback(data, &should_context_switch);
  }
  return should_context_switch;
}

void dbgserial_input_init(void) {
  // set up the USART interrupt on RX
  uart_set_rx_interrupt_handler(DBG_UART, prv_uart_irq_handler);
  uart_set_rx_interrupt_enabled(DBG_UART, true);
}

void dbgserial_register_character_callback(DbgSerialCharacterCallback callback) {
  s_character_callback = callback;
}

void dbgserial_set_rx_dma_enabled(bool enabled) {
  if (enabled == s_dma_enabled) {
    return;
  }
  s_dma_enabled = enabled;
  if (enabled) {
    uart_start_rx_dma(DBG_UART, s_dma_buffer, DMA_BUFFER_LENGTH);
  } else {
    uart_stop_rx_dma(DBG_UART);
  }
}

void dbgserial_set_input_enabled(bool enabled) {
  uart_set_rx_interrupt_enabled(DBG_UART, enabled);
}

#else
void dbgserial_input_init(void) {}

void dbgserial_register_character_callback(DbgSerialCharacterCallback callback) {}

void dbgserial_set_rx_dma_enabled(bool enabled) {}

void dbgserial_set_input_enabled(bool enabled) {}
#endif
