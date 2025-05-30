/*
 * Copyright 2024 Google LLC
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

#include "drivers/uart.h"

#include "FreeRTOS.h"
#include "bf0_hal.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/periph_config.h"
#include "system/passert.h"
#include "uart_definitions.h"

typedef enum {
  UART_FullDuplex = 0,
  UART_TxOnly,
  UART_RxOnly,
} UARTInitMode_t;

typedef struct {
  pin_function tx_pin_fun;
  pin_function rx_pin_fun;
} UARTPinFunction_t;

static IRQn_Type get_uart_irqn(UART_HandleTypeDef *uart) {
  IRQn_Type irqn;
  if (uart->Instance == USART1) {
    irqn = USART1_IRQn;
  } else if (uart->Instance == USART2) {
    irqn = USART2_IRQn;
  } else if (uart->Instance == USART3) {
    irqn = USART3_IRQn;
  }
#ifdef USART4
  else if (uart->Instance == USART4) {
    irqn = USART4_IRQn;
  }
#endif
#ifdef USART5
  else if (uart->Instance == USART5) {
    irqn = USART5_IRQn;
  }
#endif
#ifdef USART6
  else if (uart->Instance == USART6) {
    irqn = USART6_IRQn;
  }
#endif
  else {
    WTF;
  }
  return irqn;
}

static IRQn_Type get_dma_channel_irqn(DMA_HandleTypeDef *dma) {
  IRQn_Type irqn;
  if (dma->Instance == DMA1_Channel1) {
    irqn = DMAC1_CH1_IRQn;
  } else if (dma->Instance == DMA1_Channel2) {
    irqn = DMAC1_CH2_IRQn;
  } else if (dma->Instance == DMA1_Channel3) {
    irqn = DMAC1_CH3_IRQn;
  } else if (dma->Instance == DMA1_Channel4) {
    irqn = DMAC1_CH4_IRQn;
  } else if (dma->Instance == DMA1_Channel5) {
    irqn = DMAC1_CH5_IRQn;
  } else if (dma->Instance == DMA1_Channel6) {
    irqn = DMAC1_CH6_IRQn;
  } else if (dma->Instance == DMA1_Channel7) {
    irqn = DMAC1_CH7_IRQn;
  } else if (dma->Instance == DMA2_Channel1) {
    irqn = DMAC2_CH1_IRQn;
  } else if (dma->Instance == DMA2_Channel2) {
    irqn = DMAC2_CH2_IRQn;
  } else if (dma->Instance == DMA2_Channel3) {
    irqn = DMAC2_CH3_IRQn;
  } else if (dma->Instance == DMA2_Channel4) {
    irqn = DMAC2_CH4_IRQn;
  } else if (dma->Instance == DMA2_Channel5) {
    irqn = DMAC2_CH5_IRQn;
  } else if (dma->Instance == DMA2_Channel6) {
    irqn = DMAC2_CH6_IRQn;
  } else if (dma->Instance == DMA2_Channel7) {
    irqn = DMAC2_CH7_IRQn;
  } else {
    WTF;
  }
  return irqn;
}

static UARTPinFunction_t get_uart_pin_fun(UART_HandleTypeDef *uart) {
  UARTPinFunction_t pin_fun = {0};
  if (uart->Instance == USART1) {
    pin_fun.tx_pin_fun = USART1_TXD;
    pin_fun.rx_pin_fun = USART1_RXD;
  } else if (uart->Instance == USART2) {
    pin_fun.tx_pin_fun = USART2_TXD;
    pin_fun.rx_pin_fun = USART2_RXD;
  } else if (uart->Instance == USART3) {
    pin_fun.tx_pin_fun = USART3_TXD;
    pin_fun.rx_pin_fun = USART3_RXD;
  }
#ifdef USART4
  else if (uart->Instance == USART4) {
    pin_fun.tx_pin_fun = USART4_TXD;
    pin_fun.rx_pin_fun = USART4_RXD;
  }
#endif
#ifdef USART5
  else if (uart->Instance == USART5) {
    pin_fun.tx_pin_fun = USART5_TXD;
    pin_fun.rx_pin_fun = USART5_RXD;
  }
#endif
#ifdef USART6
  else if (uart->Instance == USART6) {
    pin_fun.tx_pin_fun = USART6_TXD;
    pin_fun.rx_pin_fun = USART6_RXD;
  }
#endif
  else {
    WTF;
  }
  return pin_fun;
}

static void prv_init(UARTDevice *dev, UARTInitMode_t mode) {
  if (mode == UART_FullDuplex) {
    dev->periph->Init.Mode = UART_MODE_TX_RX;
  } else {
    dev->periph->Init.Mode = (mode == UART_TxOnly) ? UART_MODE_TX : UART_MODE_RX;
  }
  if (HAL_UART_Init(dev->periph) != HAL_OK) {
    // Initialization Error
    WTF;
  }

  UARTPinFunction_t pin_fun = get_uart_pin_fun(dev->periph);
  switch (mode) {
    case UART_FullDuplex:
      HAL_PIN_Set(dev->tx_gpio, pin_fun.tx_pin_fun, PIN_PULLUP, 1);
      HAL_PIN_Set(dev->rx_gpio, pin_fun.rx_pin_fun, PIN_PULLUP, 1);
      break;
    case UART_TxOnly:
      HAL_PIN_Set(dev->tx_gpio, pin_fun.tx_pin_fun, PIN_PULLUP, 1);
      break;
    case UART_RxOnly:
      HAL_PIN_Set(dev->rx_gpio, pin_fun.rx_pin_fun, PIN_PULLUP, 1);
      break;
    default:
      break;
  }
  dev->state->initialized = true;
  if (dev->rx_dma) {
    // initialize the DMA request
    __HAL_LINKDMA(dev->periph, hdmarx, *dev->rx_dma);
    IRQn_Type dma_irqn = get_dma_channel_irqn(dev->rx_dma);
    NVIC_SetPriority(dma_irqn, dev->irq_priority);
    HAL_NVIC_EnableIRQ(dma_irqn);
    __HAL_UART_ENABLE_IT(dev->periph, UART_IT_IDLE);
  }
}

void uart_init(UARTDevice *dev) { prv_init(dev, UART_FullDuplex); }

void uart_init_open_drain(UARTDevice *dev) { WTF; }

void uart_init_tx_only(UARTDevice *dev) { prv_init(dev, UART_TxOnly); }

void uart_init_rx_only(UARTDevice *dev) { prv_init(dev, UART_RxOnly); }

void uart_deinit(UARTDevice *dev) { HAL_UART_DeInit(dev->periph); }

void uart_set_baud_rate(UARTDevice *dev, uint32_t baud_rate) {
  PBL_ASSERTN(dev->state->initialized);
  HAL_UART_DeInit(dev->periph);
  dev->periph->Init.BaudRate = baud_rate;
  if (HAL_UART_Init(dev->periph) != HAL_OK) {
    // Initialization Error
    WTF;
  }
}

// Read / Write APIs
////////////////////////////////////////////////////////////////////////////////

void uart_write_byte(UARTDevice *dev, uint8_t data) {
  HAL_UART_Transmit(dev->periph, &data, 1, 1000);
}

uint8_t uart_read_byte(UARTDevice *dev) {
  uint8_t data;
  HAL_UART_Receive(dev->periph, &data, 1, 1000);
  return data;
}

bool uart_is_rx_ready(UARTDevice *dev) {
  return READ_REG(dev->periph->Instance->ISR) & USART_ISR_RXNE;
}

bool uart_has_rx_overrun(UARTDevice *dev) {
  return READ_REG(dev->periph->Instance->ISR) & USART_ISR_ORE;
}

bool uart_has_rx_framing_error(UARTDevice *dev) {
  return READ_REG(dev->periph->Instance->ISR) & USART_ISR_FE;
}

bool uart_is_tx_ready(UARTDevice *dev) {
  return READ_REG(dev->periph->Instance->ISR) & USART_ISR_TXE;
}

bool uart_is_tx_complete(UARTDevice *dev) {
  return READ_REG(dev->periph->Instance->ISR) & USART_ISR_TC;
}

void uart_wait_for_tx_complete(UARTDevice *dev) {
  while (!uart_is_tx_complete(dev)) continue;
}

// Interrupts
////////////////////////////////////////////////////////////////////////////////

static void prv_set_interrupt_enabled(UARTDevice *dev, bool enabled) {
  if (enabled) {
    PBL_ASSERTN(dev->state->tx_irq_handler || dev->state->rx_irq_handler);
    // enable the interrupt
    IRQn_Type uart_irqn = get_uart_irqn(dev->periph);
    NVIC_SetPriority(uart_irqn, dev->irq_priority);
    HAL_NVIC_EnableIRQ(uart_irqn);
  } else {
    // disable the interrupt
    HAL_NVIC_DisableIRQ(get_uart_irqn(dev->periph));
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
    SET_BIT(dev->periph->Instance->CR1, USART_CR1_RXNEIE);
    prv_set_interrupt_enabled(dev, true);
  } else {
    // disable interrupt if TX is also disabled
    prv_set_interrupt_enabled(dev, dev->state->tx_int_enabled);
    CLEAR_BIT(dev->periph->Instance->CR1, USART_CR1_RXNEIE);
    dev->state->rx_int_enabled = false;
  }
}

void uart_set_tx_interrupt_enabled(UARTDevice *dev, bool enabled) {
  PBL_ASSERTN(dev->state->initialized);
  if (enabled) {
    dev->state->tx_int_enabled = true;
    SET_BIT(dev->periph->Instance->CR1, USART_CR1_TXEIE);
    prv_set_interrupt_enabled(dev, true);
  } else {
    // disable interrupt if RX is also disabled
    prv_set_interrupt_enabled(dev, dev->state->rx_int_enabled);
    CLEAR_BIT(dev->periph->Instance->CR1, USART_CR1_TXEIE);
    dev->state->tx_int_enabled = false;
  }
}

void uart_irq_handler(UARTDevice *dev) {
  PBL_ASSERTN(dev->state->initialized);
  bool should_context_switch = false;
  if (dev->state->rx_irq_handler && dev->state->rx_int_enabled) {
    const UARTRXErrorFlags err_flags = {
        .overrun_error = uart_has_rx_overrun(dev),
        .framing_error = uart_has_rx_framing_error(dev),
    };
    // DMA
    if (dev->state->rx_dma_buffer) {
      // process bytes from the DMA buffer
      const uint32_t dma_length = dev->state->rx_dma_length;
      const uint32_t recv_total_index = dma_length - __HAL_DMA_GET_COUNTER(dev->rx_dma);
      int32_t recv_len = recv_total_index - dev->state->rx_dma_index;
      if (recv_len < 0) {
        recv_len += dma_length;
      }

      for (int32_t i = 0; i < recv_len; i++) {
        const uint8_t data = dev->state->rx_dma_buffer[dev->state->rx_dma_index + i];
        if (dev->state->rx_irq_handler(dev, data, &err_flags)) {
          should_context_switch = true;
        }
      }
      dev->state->rx_dma_index = recv_total_index;
      if (dev->state->rx_dma_index >= dma_length) {
        dev->state->rx_dma_index = 0;
        HAL_UART_DmaTransmit(dev->periph, dev->state->rx_dma_buffer, dma_length, DMA_PERIPH_TO_MEMORY);
      }
      uart_clear_all_interrupt_flags(dev);
      __HAL_UART_CLEAR_IDLEFLAG(dev->periph);
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
  __HAL_UART_CLEAR_OREFLAG(dev->periph);
}

// DMA
////////////////////////////////////////////////////////////////////////////////

void uart_start_rx_dma(UARTDevice *dev, void *buffer, uint32_t length) {
  dev->state->rx_dma_buffer = buffer;
  dev->state->rx_dma_length = length;
  dev->state->rx_dma_index = 0;
  HAL_UART_DmaTransmit(dev->periph, buffer, length, DMA_PERIPH_TO_MEMORY);
}

void uart_stop_rx_dma(UARTDevice *dev) {
  dev->state->rx_dma_buffer = NULL;
  dev->state->rx_dma_length = 0;
  HAL_UART_DMAPause(dev->periph);
}

void uart_clear_rx_dma_buffer(UARTDevice *dev) {
  dev->state->rx_dma_index = dev->state->rx_dma_length - __HAL_DMA_GET_COUNTER(dev->rx_dma);
}
