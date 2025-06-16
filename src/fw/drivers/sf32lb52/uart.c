/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "uart_definitions.h"

#include "drivers/uart.h"
#include "system/passert.h"

#include "FreeRTOS.h"
#include "bf0_hal_dma.h"
#include "bf0_hal_uart.h"

#include "util/misc.h"

static void prv_init(UARTDevice *dev, uint32_t mode) {
  HAL_StatusTypeDef ret;

  dev->state->huart.Init.Mode = mode;
  dev->state->dev = dev;
  ret = HAL_UART_Init(&dev->state->huart);
  PBL_ASSERTN(ret == HAL_OK);

  switch (mode) {
    case UART_MODE_TX_RX:
      HAL_PIN_Set(dev->tx.pad, dev->tx.func, dev->tx.flags, 1);
      HAL_PIN_Set(dev->rx.pad, dev->rx.func, dev->rx.flags, 1);
      break;
    case UART_MODE_TX:
      HAL_PIN_Set(dev->tx.pad, dev->tx.func, dev->tx.flags, 1);
      break;
    case UART_MODE_RX:
      HAL_PIN_Set(dev->rx.pad, dev->rx.func, dev->rx.flags, 1);
      break;
    default:
      WTF;
      break;
  }

  dev->state->initialized = true;

  if (dev->state->hdma.Instance != NULL) {
    __HAL_LINKDMA(&dev->state->huart, hdmarx, dev->state->hdma);

    NVIC_SetPriority(dev->dma_irqn, dev->dma_irq_priority);
    HAL_NVIC_EnableIRQ(dev->dma_irqn);

    __HAL_UART_ENABLE_IT(&dev->state->huart, UART_IT_IDLE);
  }
}

void uart_init(UARTDevice *dev) { prv_init(dev, UART_MODE_TX_RX); }

void uart_init_open_drain(UARTDevice *dev) { WTF; }

void uart_init_tx_only(UARTDevice *dev) { prv_init(dev, UART_MODE_TX); }

void uart_init_rx_only(UARTDevice *dev) { prv_init(dev, UART_MODE_RX); }

void uart_deinit(UARTDevice *dev) { HAL_UART_DeInit(&dev->state->huart); }

void uart_set_baud_rate(UARTDevice *dev, uint32_t baud_rate) {
  HAL_StatusTypeDef ret;

  PBL_ASSERTN(dev->state->initialized);

  HAL_UART_DeInit(&dev->state->huart);

  dev->state->huart.Init.BaudRate = baud_rate;
  ret = HAL_UART_Init(&dev->state->huart);
  PBL_ASSERTN(ret == HAL_OK);
}

// Read / Write APIs
////////////////////////////////////////////////////////////////////////////////

void uart_write_byte(UARTDevice *dev, uint8_t data) {
  HAL_UART_Transmit(&dev->state->huart, &data, 1, HAL_MAX_DELAY);
}

uint8_t uart_read_byte(UARTDevice *dev) {
  HAL_StatusTypeDef ret;
  uint8_t data;

  ret = HAL_UART_Receive(&dev->state->huart, &data, 1, HAL_MAX_DELAY);
  // PBL_ASSERTN(ret == HAL_OK);

  return data;
}

bool uart_is_rx_ready(UARTDevice *dev) {
  return READ_REG(dev->state->huart.Instance->ISR) & USART_ISR_RXNE;
}

bool uart_has_rx_overrun(UARTDevice *dev) {
  return READ_REG(dev->state->huart.Instance->ISR) & USART_ISR_ORE;
}

bool uart_has_rx_framing_error(UARTDevice *dev) {
  return READ_REG(dev->state->huart.Instance->ISR) & USART_ISR_FE;
}

bool uart_is_tx_ready(UARTDevice *dev) {
  return READ_REG(dev->state->huart.Instance->ISR) & USART_ISR_TXE;
}

bool uart_is_tx_complete(UARTDevice *dev) {
  return READ_REG(dev->state->huart.Instance->ISR) & USART_ISR_TC;
}

void uart_wait_for_tx_complete(UARTDevice *dev) {
  while (!uart_is_tx_complete(dev)) continue;
}

// Interrupts
////////////////////////////////////////////////////////////////////////////////

static void prv_set_interrupt_enabled(UARTDevice *dev, bool enabled) {
  if (enabled) {
    PBL_ASSERTN(dev->state->tx_irq_handler || dev->state->rx_irq_handler);
    NVIC_SetPriority(dev->irqn, dev->irq_priority);
    HAL_NVIC_EnableIRQ(dev->irqn);
  } else {
    HAL_NVIC_DisableIRQ(dev->irqn);
  }
}

void uart_set_rx_interrupt_handler(UARTDevice *dev, UARTRXInterruptHandler irq_handler) {
  PBL_ASSERTN(dev->state->initialized);
  dev->state->rx_irq_handler = irq_handler;
}

void uart_set_tx_interrupt_handler(UARTDevice *dev, UARTTXInterruptHandler irq_handler) {
  PBL_ASSERTN(dev->state->initialized);
  dev->state->tx_irq_handler = irq_handler;
}

void uart_set_rx_interrupt_enabled(UARTDevice *dev, bool enabled) {
  PBL_ASSERTN(dev->state->initialized);
  if (enabled) {
    dev->state->rx_int_enabled = true;
    SET_BIT(dev->state->huart.Instance->CR1, USART_CR1_RXNEIE);
    prv_set_interrupt_enabled(dev, true);
  } else {
    // disable interrupt if TX is also disabled
    prv_set_interrupt_enabled(dev, dev->state->tx_int_enabled);
    CLEAR_BIT(dev->state->huart.Instance->CR1, USART_CR1_RXNEIE);
    dev->state->rx_int_enabled = false;
  }
}

void uart_set_tx_interrupt_enabled(UARTDevice *dev, bool enabled) {
  PBL_ASSERTN(dev->state->initialized);
  if (enabled) {
    dev->state->tx_int_enabled = true;
    SET_BIT(dev->state->huart.Instance->CR1, USART_CR1_TXEIE);
    prv_set_interrupt_enabled(dev, true);
  } else {
    // disable interrupt if RX is also disabled
    prv_set_interrupt_enabled(dev, dev->state->rx_int_enabled);
    CLEAR_BIT(dev->state->huart.Instance->CR1, USART_CR1_TXEIE);
    dev->state->tx_int_enabled = false;
  }
}

void uart_irq_handler(UARTDevice *dev) {
  PBL_ASSERTN(dev->state->initialized);
  bool should_context_switch = false;
  uint32_t idx;

  if (dev->state->rx_irq_handler && dev->state->rx_int_enabled) {
    const UARTRXErrorFlags err_flags = {
        .overrun_error = uart_has_rx_overrun(dev),
        .framing_error = uart_has_rx_framing_error(dev),
    };
    // DMA
    if (dev->state->rx_dma_buffer && (__HAL_UART_GET_FLAG(&dev->state->huart, UART_FLAG_IDLE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&dev->state->huart, UART_IT_IDLE) != RESET)) {
      // process bytes from the DMA buffer
      const uint32_t dma_length = dev->state->rx_dma_length;
      const uint32_t recv_total_index = dma_length - __HAL_DMA_GET_COUNTER(&dev->state->hdma);
      int32_t recv_len = recv_total_index - dev->state->rx_dma_index;
      if (recv_len < 0) {
        recv_len += dma_length;
      }

      idx = dev->state->rx_dma_index;
      for (int32_t i = 0; i < recv_len; i++) {
        uint8_t data;
        data = dev->state->rx_dma_buffer[idx];
        if (dev->state->rx_irq_handler(dev, data, &err_flags)) {
          should_context_switch = true;
        }
        idx++;
        if (idx >= dma_length) {
          idx = 0;
        }
      }
      dev->state->rx_dma_index = recv_total_index;
      if (dev->state->rx_dma_index >= dma_length) {
        dev->state->rx_dma_index = 0;
      }
      uart_clear_all_interrupt_flags(dev);
      __HAL_UART_CLEAR_IDLEFLAG(&dev->state->huart);
    } else {
      const bool has_byte = uart_is_rx_ready(dev);
      // read the data register regardless to clear the error flags
      const uint8_t data = uart_read_byte(dev);
      if (has_byte) {
        if (dev->state->rx_irq_handler(dev, data, &err_flags)) {
          should_context_switch = true;
        }
      }
    }
  }
  if (dev->state->tx_irq_handler && dev->state->tx_int_enabled && uart_is_tx_ready(dev)) {
    if (dev->state->tx_irq_handler(dev)) {
      should_context_switch = true;
    }
  }
  portEND_SWITCHING_ISR(should_context_switch);
}

void uart_clear_all_interrupt_flags(UARTDevice *dev) {
  UART_HandleTypeDef *uart = &dev->state->huart;
  if (__HAL_UART_GET_FLAG(uart, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_OREFLAG(uart);
  }
  if (__HAL_UART_GET_FLAG(uart, UART_FLAG_NE) != RESET) {
    __HAL_UART_CLEAR_NEFLAG(uart);
  }
  if (__HAL_UART_GET_FLAG(uart, UART_FLAG_FE) != RESET) {
    __HAL_UART_CLEAR_FEFLAG(uart);
  }
  if (__HAL_UART_GET_FLAG(uart, UART_FLAG_PE) != RESET) {
    __HAL_UART_CLEAR_PEFLAG(uart);
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  size_t recv_len;
  size_t recv_total_index;
  uint32_t idx;
  bool should_context_switch = false;

  UARTDeviceState *state = container_of(huart, UARTDeviceState, huart);
  UARTDevice *dev = (UARTDevice *)state->dev;

  recv_total_index = state->rx_dma_length - __HAL_DMA_GET_COUNTER(&state->hdma);
  if (recv_total_index < state->rx_dma_index)
    recv_len = state->rx_dma_length + recv_total_index - state->rx_dma_index;
  else
    recv_len = recv_total_index - state->rx_dma_index;

  idx = state->rx_dma_index;    
  state->rx_dma_index = recv_total_index;
  if (recv_len) {
    for (size_t i = 0; i < recv_len; i++) {
      uint8_t data;
      data = state->rx_dma_buffer[idx];
      if (state->rx_irq_handler(dev, data, NULL)) {
        should_context_switch = true;
      }
      idx++;
      if (idx >= state->rx_dma_length) {
          idx = 0;
      }
    }
  }
  portEND_SWITCHING_ISR(should_context_switch);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_UART_RxHalfCpltCallback(huart);
}

// DMA
////////////////////////////////////////////////////////////////////////////////

void uart_dma_irq_handler(UARTDevice *dev) {
  HAL_DMA_IRQHandler(&dev->state->hdma);
}

void uart_start_rx_dma(UARTDevice *dev, void *buffer, uint32_t length) {
  dev->state->rx_dma_buffer = buffer;
  dev->state->rx_dma_length = length;
  dev->state->rx_dma_index = 0;
  __HAL_UART_ENABLE_IT(&dev->state->huart, UART_IT_IDLE);
  HAL_UART_DmaTransmit(&dev->state->huart, buffer, length, DMA_PERIPH_TO_MEMORY);
}

void uart_stop_rx_dma(UARTDevice *dev) {
  dev->state->rx_dma_buffer = NULL;
  dev->state->rx_dma_length = 0;
  HAL_UART_DMAPause(&dev->state->huart);
}

void uart_clear_rx_dma_buffer(UARTDevice *dev) {
  dev->state->rx_dma_index = dev->state->rx_dma_length - __HAL_DMA_GET_COUNTER(&dev->state->hdma);
}
