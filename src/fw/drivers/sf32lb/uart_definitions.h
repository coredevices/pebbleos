#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board/board.h"
#include "drivers/uart.h"

typedef struct UARTState {
  bool initialized;
  UARTRXInterruptHandler rx_irq_handler;
  UARTTXInterruptHandler tx_irq_handler;
  bool rx_int_enabled;
  bool tx_int_enabled;
  uint8_t *rx_dma_buffer;
  uint32_t rx_dma_length;
  uint32_t rx_dma_index;
} UARTDeviceState;

typedef const struct UARTDevice {
  UARTDeviceState *state;
  bool half_duplex;
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  UART_HandleTypeDef *periph;
  DMA_HandleTypeDef *rx_dma;
  uint8_t irq_priority;
} UARTDevice;

// thinly wrapped by the IRQ handler in board_*.c
void uart_irq_handler(UARTDevice *dev);
