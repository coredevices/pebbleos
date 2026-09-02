/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/uart/sf32lb.h>

#include <pbl/drivers/uart.h>
#include "pbl/soc/sf32lb/sleep.h"
#include "system/passert.h"

#include "bf0_hal_dma.h"
#include "bf0_hal_uart.h"

#include "pbl/util/misc.h"

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

    HAL_NVIC_SetPriority(dev->dma_irqn, dev->dma_irq_priority, 0);
    HAL_NVIC_EnableIRQ(dev->dma_irqn);

    __HAL_UART_ENABLE_IT(&dev->state->huart, UART_IT_IDLE);
  }
}

void uart_init(UARTDevice *dev) { prv_init(dev, UART_MODE_TX_RX); }

void uart_init_open_drain(UARTDevice *dev) { WTF; }

void uart_init_tx_only(UARTDevice *dev) { prv_init(dev, UART_MODE_TX); }

void uart_init_rx_only(UARTDevice *dev) { prv_init(dev, UART_MODE_RX); }

void uart_deinit(UARTDevice *dev) {
  HAL_UART_DeInit(&dev->state->huart);
}

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
  return __HAL_UART_GETC(&dev->state->huart);
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
    HAL_NVIC_SetPriority(dev->irqn, dev->irq_priority, 0);
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
    soc_sf32lb_sleep_block(SOC_SF32LB_DEEPSLEEP);
    dev->state->rx_int_enabled = true;
    SET_BIT(dev->state->huart.Instance->CR1, USART_CR1_RXNEIE);
    prv_set_interrupt_enabled(dev, true);
  } else {
    // disable interrupt if TX is also disabled
    prv_set_interrupt_enabled(dev, dev->state->tx_int_enabled);
    CLEAR_BIT(dev->state->huart.Instance->CR1, USART_CR1_RXNEIE);
    dev->state->rx_int_enabled = false;
    soc_sf32lb_sleep_release(SOC_SF32LB_DEEPSLEEP);
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
        dev->state->rx_irq_handler(dev, data, &err_flags);
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
        dev->state->rx_irq_handler(dev, data, &err_flags);
      }
    }
  }
  if (dev->state->tx_irq_handler && dev->state->tx_int_enabled && uart_is_tx_ready(dev)) {
    dev->state->tx_irq_handler(dev);
  }
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
      state->rx_irq_handler(dev, data, NULL);
      idx++;
      if (idx >= state->rx_dma_length) {
          idx = 0;
      }
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_UART_RxHalfCpltCallback(huart);
}

// DMA
////////////////////////////////////////////////////////////////////////////////

void uart_dma_irq_handler(UARTDevice *dev) {
  HAL_DMA_IRQHandler(&dev->state->hdma);
}

// FIXME(SF32LB52): the IRQ paths above read `rx_dma_buffer[idx]` straight
// without invalidating D-cache, so the CPU could pick up stale pre-DMA bytes.
// There is no active SF32LB52 caller today (dbgserial_set_rx_dma_enabled is a
// no-op for CONFIG_SOC_SF32LB52), so this hasn't been wired up. Before
// enabling, the buffer needs to be cache-line aligned/sized and the callbacks
// must dcache_invalidate the freshly-DMA'd range.
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
