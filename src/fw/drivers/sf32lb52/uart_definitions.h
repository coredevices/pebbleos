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
  UART_HandleTypeDef huart;
  DMA_HandleTypeDef hdma;
} UARTDeviceState;

typedef const struct UARTDevice {
  UARTDeviceState *state;
  Pinmux rx;
  Pinmux tx;
  IRQn_Type irqn;
  uint8_t irq_priority;
  IRQn_Type dma_irqn;
  uint8_t dma_irq_priority;
} UARTDevice;

// thinly wrapped by the IRQ handler in board_*.c
void uart_irq_handler(UARTDevice *dev);
void uart_dma_irq_handler(UARTDevice *dev);
