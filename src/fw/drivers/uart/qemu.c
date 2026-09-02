/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/uart.h>
#include <pbl/drivers/uart/qemu.h>

#include "board/board.h"
#include "system/passert.h"

#define REG32(addr) (*(volatile uint32_t *)(addr))

// QEMU pebble-simple-uart register offsets
#define UART_DATA    0x00
#define UART_STATE   0x04
#define UART_CTRL    0x08
#define UART_INT     0x0C

// STATE register bits
#define STATE_TX_READY  (1 << 0)
#define STATE_RX_READY  (1 << 1)

// CTRL register bits (must match QEMU pebble-simple-uart)
#define CTRL_TX_IE  (1 << 0)
#define CTRL_RX_IE  (1 << 1)

// INT register bits (must match QEMU pebble-simple-uart)
#define INT_TX_PENDING  (1 << 0)
#define INT_RX_PENDING  (1 << 1)

void uart_init(UARTDevice *dev) {
  // Clear any pending interrupts
  REG32(dev->base_addr + UART_INT) = INT_RX_PENDING | INT_TX_PENDING;
  // Disable interrupts initially
  REG32(dev->base_addr + UART_CTRL) = 0;
  // Enable this UART's IRQ in the NVIC
  NVIC_SetPriority(dev->irqn, dev->irq_priority);
  NVIC_EnableIRQ(dev->irqn);
}

void uart_init_open_drain(UARTDevice *dev) {
  uart_init(dev);
}

void uart_init_tx_only(UARTDevice *dev) {
  uart_init(dev);
}

void uart_init_rx_only(UARTDevice *dev) {
  uart_init(dev);
}

void uart_deinit(UARTDevice *dev) {
  REG32(dev->base_addr + UART_CTRL) = 0;
  REG32(dev->base_addr + UART_INT) = INT_RX_PENDING | INT_TX_PENDING;
}

void uart_set_baud_rate(UARTDevice *dev, uint32_t baud_rate) {
  // QEMU UART does not have a real baud rate; ignore
  (void)dev;
  (void)baud_rate;
}

void uart_write_byte(UARTDevice *dev, uint8_t data) {
  // Wait for TX ready
  while (!(REG32(dev->base_addr + UART_STATE) & STATE_TX_READY)) {
    // spin
  }
  REG32(dev->base_addr + UART_DATA) = data;
}

uint8_t uart_read_byte(UARTDevice *dev) {
  return (uint8_t)(REG32(dev->base_addr + UART_DATA) & 0xFF);
}

bool uart_is_rx_ready(UARTDevice *dev) {
  return (REG32(dev->base_addr + UART_STATE) & STATE_RX_READY) != 0;
}

bool uart_has_rx_overrun(UARTDevice *dev) {
  (void)dev;
  return false;
}

bool uart_has_rx_framing_error(UARTDevice *dev) {
  (void)dev;
  return false;
}

bool uart_is_tx_ready(UARTDevice *dev) {
  return (REG32(dev->base_addr + UART_STATE) & STATE_TX_READY) != 0;
}

bool uart_is_tx_complete(UARTDevice *dev) {
  return (REG32(dev->base_addr + UART_STATE) & STATE_TX_READY) != 0;
}

void uart_wait_for_tx_complete(UARTDevice *dev) {
  while (!uart_is_tx_complete(dev)) {
    // spin
  }
}

UARTRXErrorFlags uart_has_errored_out(UARTDevice *dev) {
  (void)dev;
  UARTRXErrorFlags flags = {};
  return flags;
}

void uart_set_rx_interrupt_handler(UARTDevice *dev, UARTRXInterruptHandler irq_handler) {
  dev->state->rx_irq_handler = irq_handler;
}

void uart_set_tx_interrupt_handler(UARTDevice *dev, UARTTXInterruptHandler irq_handler) {
  dev->state->tx_irq_handler = irq_handler;
}

void uart_set_rx_interrupt_enabled(UARTDevice *dev, bool enabled) {
  // Order matters: the QEMU pebble-simple-uart is level-sensitive on RX, so
  // as long as CTRL_RX_IE is set and rx_count > 0 the IRQ keeps re-firing.
  // The ISR uses dev->state->rx_int_enabled to decide whether to drain bytes.
  //
  // If we set the flag false BEFORE clearing CTRL_RX_IE, an incoming IRQ
  // re-enters the handler, sees the flag false, skips the drain, returns —
  // and immediately re-fires because CTRL_RX_IE is still set and rx_count is
  // still > 0.  KernelMain only executes one instruction per re-entry cycle
  // (just enough to crawl forward) and the install effectively wedges.
  //
  // For disable: clear hardware first so no more IRQs fire, THEN drop the
  // flag.  For enable: set the flag first so the handler will drain when the
  // hardware IRQ asserts.
  uint32_t ctrl = REG32(dev->base_addr + UART_CTRL);
  if (enabled) {
    dev->state->rx_int_enabled = true;
    REG32(dev->base_addr + UART_CTRL) = ctrl | CTRL_RX_IE;
  } else {
    REG32(dev->base_addr + UART_CTRL) = ctrl & ~CTRL_RX_IE;
    dev->state->rx_int_enabled = false;
  }
}

void uart_set_tx_interrupt_enabled(UARTDevice *dev, bool enabled) {
  dev->state->tx_int_enabled = enabled;
  uint32_t ctrl = REG32(dev->base_addr + UART_CTRL);
  if (enabled) {
    ctrl |= CTRL_TX_IE;
  } else {
    ctrl &= ~CTRL_TX_IE;
  }
  REG32(dev->base_addr + UART_CTRL) = ctrl;
}

void uart_clear_all_interrupt_flags(UARTDevice *dev) {
  REG32(dev->base_addr + UART_INT) = INT_RX_PENDING | INT_TX_PENDING;
}

void uart_start_rx_dma(UARTDevice *dev, void *buffer, uint32_t length) {
  // DMA not supported on QEMU UART
  (void)dev;
  (void)buffer;
  (void)length;
}

void uart_stop_rx_dma(UARTDevice *dev) {
  (void)dev;
}

void uart_clear_rx_dma_buffer(UARTDevice *dev) {
  (void)dev;
}

// Called from the IRQ handler trampoline defined via IRQ_MAP in the board file
void uart_irq_handler(UARTDevice *dev) {
  uint32_t int_status = REG32(dev->base_addr + UART_INT);

  if (int_status & INT_RX_PENDING) {
    REG32(dev->base_addr + UART_INT) = INT_RX_PENDING;
    if (dev->state->rx_irq_handler && dev->state->rx_int_enabled) {
      // Stop if the handler disables RX interrupts (e.g. ISR buffer full);
      // remaining bytes stay in the UART FIFO for the next interrupt.
      while (dev->state->rx_int_enabled &&
             (REG32(dev->base_addr + UART_STATE) & STATE_RX_READY)) {
        uint8_t byte = (uint8_t)(REG32(dev->base_addr + UART_DATA) & 0xFF);
        UARTRXErrorFlags err = {};
        dev->state->rx_irq_handler(dev, byte, &err);
      }
    }
  }

  if (int_status & INT_TX_PENDING) {
    REG32(dev->base_addr + UART_INT) = INT_TX_PENDING;
    if (dev->state->tx_irq_handler && dev->state->tx_int_enabled) {
      dev->state->tx_irq_handler(dev);
    }
  }

}
