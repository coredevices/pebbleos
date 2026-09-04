/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/uart/nrf5.h>
#include <pbl/drivers/uart.h>

#include <pbl/logging/logging.h>
#include "system/passert.h"

#include <nrfx_uarte.h>
#ifdef NRF_PPI_BASE
#include <nrfx_ppi.h>
#else
#include <nrfx_dppi.h>
#endif

PBL_LOG_MODULE_DEFINE(driver_uart_nrf5, CONFIG_DRIVER_UART_LOG_LEVEL);


// UART: 8n1, duplex

static void _uart_event_handler(const nrfx_uarte_event_t *event, void *ctx);

static void _timer_event_handler(nrf_timer_event_t event_type, void *ctx) { }

void uart_init(UARTDevice *dev) {
  nrfx_uarte_config_t config = {
    .txd_pin = dev->tx_gpio,
    .rxd_pin = dev->rx_gpio,
    .rts_pin = dev->rts_gpio,
    .cts_pin = dev->cts_gpio,
    .p_context = (void *)dev,
    .tx_cache = { .p_buffer = (uint8_t *) dev->state->tx_cache_buffer, .length = sizeof(dev->state->tx_cache_buffer) },
    .rx_cache = { .p_buffer = (uint8_t *) dev->state->rx_cache_buffer, .length = sizeof(dev->state->rx_cache_buffer) },
    .baudrate = NRF_UARTE_BAUDRATE_1000000,
    .config = {
      .hwfc = NRF_UARTE_HWFC_DISABLED,
      .parity = NRF_UARTE_PARITY_EXCLUDED,
      .stop = NRF_UARTE_STOP_ONE
    },
    .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,
  };

  nrfx_err_t err = nrfx_uarte_init(&dev->periph, &config, _uart_event_handler);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  
  /* Roughly patterned off of https://devzone.nordicsemi.com/f/nordic-q-a/28420/uarte-in-circular-mode */
  nrfx_timer_config_t tconfig = {
    .frequency = 1000000, /* dummy */
    .mode = NRF_TIMER_MODE_COUNTER,
    .bit_width = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
    .p_context = NULL,
  };
  err = nrfx_timer_init(&dev->counter, &tconfig, _timer_event_handler);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  
  
#ifdef NRF_PPI_BASE
  nrf_ppi_channel_t rxdrdy_count_channel;
  nrf_ppi_channel_t endrx_clear_channel;

  err = nrfx_ppi_channel_alloc(&rxdrdy_count_channel);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  nrfx_ppi_channel_assign(rxdrdy_count_channel, nrfx_uarte_event_address_get(&dev->periph, NRF_UARTE_EVENT_RXDRDY), nrfx_timer_task_address_get(&dev->counter, NRF_TIMER_TASK_COUNT));
  nrfx_ppi_channel_enable(rxdrdy_count_channel);

  err = nrfx_ppi_channel_alloc(&endrx_clear_channel);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  nrfx_ppi_channel_assign(endrx_clear_channel, nrfx_uarte_event_address_get(&dev->periph, NRF_UARTE_EVENT_ENDRX), nrfx_timer_task_address_get(&dev->counter, NRF_TIMER_TASK_CLEAR));
  nrfx_ppi_channel_enable(endrx_clear_channel);
#else
  uint8_t rxdrdy_count_channel;
  uint8_t endrx_clear_channel;

  err = nrfx_dppi_channel_alloc(&rxdrdy_count_channel);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  NRF_DPPI_ENDPOINT_SETUP(nrfx_uarte_event_address_get(&dev->periph, NRF_UARTE_EVENT_RXDRDY), rxdrdy_count_channel);
  NRF_DPPI_ENDPOINT_SETUP(nrfx_timer_task_address_get(&dev->counter, NRF_TIMER_TASK_COUNT), rxdrdy_count_channel);
  nrfx_dppi_channel_enable(rxdrdy_count_channel);

  err = nrfx_dppi_channel_alloc(&endrx_clear_channel);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  NRF_DPPI_ENDPOINT_SETUP(nrfx_uarte_event_address_get(&dev->periph, NRF_UARTE_EVENT_ENDRX), endrx_clear_channel);
  NRF_DPPI_ENDPOINT_SETUP(nrfx_timer_task_address_get(&dev->counter, NRF_TIMER_TASK_CLEAR), endrx_clear_channel);
  nrfx_dppi_channel_enable(endrx_clear_channel);
#endif
  

  dev->state->initialized = true;
}

void uart_init_open_drain(UARTDevice *dev) {
  WTF; /* unimplemented, for now */
}

void uart_init_tx_only(UARTDevice *dev) {
  nrfx_uarte_config_t config = {
    .txd_pin = dev->tx_gpio,
    .rxd_pin = NRF_UARTE_PSEL_DISCONNECTED,
    .rts_pin = NRF_UARTE_PSEL_DISCONNECTED,
    .cts_pin = NRF_UARTE_PSEL_DISCONNECTED,
    .p_context = (void *)dev,
    .tx_cache = { .p_buffer = (uint8_t *) dev->state->tx_cache_buffer, .length = sizeof(dev->state->tx_cache_buffer) },
    .baudrate = NRF_UARTE_BAUDRATE_1000000,
    .config = {
      .hwfc = NRF_UARTE_HWFC_DISABLED,
      .parity = NRF_UARTE_PARITY_EXCLUDED,
      .stop = NRF_UARTE_STOP_ONE
    },
    .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,
  };

  nrfx_err_t err = nrfx_uarte_init(&dev->periph, &config, _uart_event_handler);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  dev->state->initialized = true;
}

void uart_init_rx_only(UARTDevice *dev) {
  WTF; /* unimplemented, for now */
}

void uart_deinit(UARTDevice *dev) {
  nrfx_uarte_uninit(&dev->periph);
}

void uart_set_baud_rate(UARTDevice *dev, uint32_t baud_rate) {
  nrf_uarte_baudrate_t baud_cfg =
#define MKBAUD(b) (baud_rate == b) ? NRF_UARTE_BAUDRATE_##b :
    MKBAUD(1200)
    MKBAUD(2400)
    MKBAUD(4800)
    MKBAUD(9600)
    MKBAUD(14400)
    MKBAUD(19200)
    MKBAUD(28800)
    MKBAUD(31250)
    MKBAUD(38400)
    MKBAUD(56000)
    MKBAUD(57600)
    MKBAUD(76800)
    MKBAUD(115200)
    MKBAUD(230400)
    MKBAUD(250000)
    MKBAUD(460800)
    MKBAUD(921600)
    MKBAUD(1000000)
    -1;
  if (baud_cfg == (nrf_uarte_baudrate_t)-1)
    WTF;
  nrfx_uarte_config_t config = {
    .txd_pin = dev->tx_gpio,
    .rxd_pin = dev->rx_gpio,
    .rts_pin = dev->rts_gpio,
    .cts_pin = dev->cts_gpio,
    .p_context = (void *)dev,
    .tx_cache = { .p_buffer = (uint8_t *) dev->state->tx_cache_buffer, .length = sizeof(dev->state->tx_cache_buffer) },
    //.rx_cache = { .p_buffer = (uint8_t *) dev->state->rx_cache_buffer, .length = sizeof(dev->state->rx_cache_buffer) },
    .baudrate = baud_cfg,
    .config = {
      .hwfc = NRF_UARTE_HWFC_DISABLED,
      .parity = NRF_UARTE_PARITY_EXCLUDED,
      .stop = NRF_UARTE_STOP_ONE
    },
    .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,
  };

  nrfx_err_t err = nrfx_uarte_reconfigure(&dev->periph, &config);
  if (err != NRFX_SUCCESS)
    WTF;
}


// Read / Write APIs
////////////////////////////////////////////////////////////////////////////////

void uart_write_byte(UARTDevice *dev, uint8_t data) {
  /* XXX: nRF5 can run either a PIO UART or a DMA, but not tx-as-PIO /
   * rx-as-DMA.  we could create our own linked TX buffer, but it is not
   * really performance critical for now.  so for now we will do a blocking
   * send on every byte.  */
  nrfx_uarte_tx(&dev->periph, &data, 1, NRFX_UARTE_TX_BLOCKING);
}

uint8_t uart_read_byte(UARTDevice *dev) {
  /* NYI for now, only accessory uses it */
  WTF;
  return 0;
}

UARTRXErrorFlags uart_has_errored_out(UARTDevice *dev) {
#if 0
  uint16_t errors = dev->periph->SR;
  UARTRXErrorFlags flags = {
    .parity_error = (errors & USART_FLAG_PE) != 0,
    .overrun_error = (errors & USART_FLAG_ORE) != 0,
    .framing_error = (errors & USART_FLAG_FE) != 0,
    .noise_detected = (errors & USART_FLAG_NE) != 0,
  };
#endif
  UARTRXErrorFlags flags = {};
  return flags;
}

bool uart_is_rx_ready(UARTDevice *dev) {
  // return dev->periph->SR & USART_SR_RXNE;
  /* NYI: used only in dialog_bootrom.c */
  WTF;
  return false;
}

bool uart_has_rx_overrun(UARTDevice *dev) {
  // return dev->periph->SR & USART_SR_ORE;
  WTF; /* NYI: used only in has_errored_out */
  return false;
}

bool uart_has_rx_framing_error(UARTDevice *dev) {
  // return dev->periph->SR & USART_SR_FE;
  WTF; /* NYU: used only in has_errored_out */
  return false;
}

bool uart_is_tx_ready(UARTDevice *dev) {
  // return dev->periph->SR & USART_SR_TXE;
  /* NYI for now, only accessory uses it */
  WTF;
  return false;
}

bool uart_is_tx_complete(UARTDevice *dev) {
  return true;
  //return nrfx_uarte_tx_in_progress(&dev->periph);
}

void uart_wait_for_tx_complete(UARTDevice *dev) {
  while (!uart_is_tx_complete(dev)) continue;
}


void uart_set_rx_interrupt_handler(UARTDevice *dev, UARTRXInterruptHandler irq_handler) {
  PBL_ASSERTN(dev->state->initialized);
  dev->state->rx_irq_handler = irq_handler;
}

void uart_set_tx_interrupt_handler(UARTDevice *dev, UARTTXInterruptHandler irq_handler) {
  PBL_ASSERTN(dev->state->initialized);
  WTF; /* accessory only, for now */
  dev->state->tx_irq_handler = irq_handler;
}

void uart_set_rx_interrupt_enabled(UARTDevice *dev, bool enabled) {
  PBL_ASSERTN(dev->state->initialized);
  dev->state->rx_int_enabled = enabled;
}

void uart_set_tx_interrupt_enabled(UARTDevice *dev, bool enabled) {
  PBL_ASSERTN(dev->state->initialized);
  WTF;
}

void uart_clear_all_interrupt_flags(UARTDevice *dev) {
  WTF; /* only used internally? */
}


// DMA
////////////////////////////////////////////////////////////////////////////////

#define DMA_BUFFERS 4

#define GET_SUBBUF_P(dev, n) (dev->state->rx_dma_buffer + dev->state->rx_dma_length * (n))

static void _uart_event_handler(const nrfx_uarte_event_t *event, void *ctx) {
  UARTDevice *dev = (UARTDevice *)ctx;
  bool should_context_switch = false;

  switch (event->type) {
  case NRFX_UARTE_EVT_RX_BUF_REQUEST:
    dev->state->rx_dma_index = (dev->state->rx_dma_index + 1) % DMA_BUFFERS;
    nrfx_uarte_rx_buffer_set(&dev->periph, GET_SUBBUF_P(dev, dev->state->rx_dma_index), dev->state->rx_dma_length);
    break;
  case NRFX_UARTE_EVT_RX_BYTE:
    /* we'll catch this up in the ring buffer catchup below */
    break;
  case NRFX_UARTE_EVT_RX_DONE: {
    dev->state->rx_prod_index = (dev->state->rx_prod_index + 1) % DMA_BUFFERS;
    break;
  }
  default:
    break;
  }

  /* catch up on any completed buffers */
  for (; dev->state->rx_cons_index != dev->state->rx_prod_index; dev->state->rx_cons_index = (dev->state->rx_cons_index + 1) % DMA_BUFFERS) {
    uint8_t *buf = GET_SUBBUF_P(dev, dev->state->rx_cons_index);
    size_t ofs = dev->state->rx_cons_pos;
    dev->state->rx_cons_pos = 0;

    if (ofs == dev->state->rx_dma_length) /* already consumed */
      continue;

    const UARTRXErrorFlags err_flags = {}; /* ignored, for now */
    for (; ofs < dev->state->rx_dma_length; ofs++) {
      if (dev->state->rx_irq_handler && dev->state->rx_int_enabled) {
        should_context_switch |= dev->state->rx_irq_handler(dev, buf[ofs], &err_flags);
      }
    }
  }
  
  uint32_t curpos = nrfx_timer_capture(&dev->counter, NRF_TIMER_CC_CHANNEL0);
  if (dev->state->rx_cons_pos < curpos) { /* if it is greater, then we have wrapped and we will catch it on the completed buffer irq later */
    uint8_t *buf = GET_SUBBUF_P(dev, dev->state->rx_cons_index);

    const UARTRXErrorFlags err_flags = {}; /* ignored, for now */
    for (; dev->state->rx_cons_pos < curpos; dev->state->rx_cons_pos++) {
      if (dev->state->rx_irq_handler && dev->state->rx_int_enabled) {
        should_context_switch |= dev->state->rx_irq_handler(dev, buf[dev->state->rx_cons_pos], &err_flags);
      }
    }
  }
  
  portEND_SWITCHING_ISR(should_context_switch);
}

void uart_start_rx_dma(UARTDevice *dev, void *buffer, uint32_t length) {
  /* the nRF5 model of DMA is sort of annoying.  what we do is we split the
   * buffer in half, and double-buffer, swapping back and forth while
   * triggering the RXSTOP -> RXSTART shortcut; every time we get a RXDRDY,
   * we trigger a RXSTOP, eat the old buffer, and open the new buffer.  ugh!
   */
  PBL_ASSERTN((((uint32_t) buffer) & 3) == 0);
  dev->state->rx_dma_buffer = buffer;
  dev->state->rx_dma_length = length / DMA_BUFFERS;
  if (dev->state->rx_dma_length % 4)
    dev->state->rx_dma_length -= dev->state->rx_dma_length % 4;
  dev->state->rx_dma_index = 0;
  dev->state->rx_prod_index = 0;
  dev->state->rx_cons_index = 0;
  dev->state->rx_cons_pos = 0;

  nrfx_timer_enable(&dev->counter);
  nrfx_timer_clear(&dev->counter);
    
  nrfx_uarte_rxdrdy_enable(&dev->periph);
  nrfx_uarte_rx_buffer_set(&dev->periph, dev->state->rx_dma_buffer, dev->state->rx_dma_length);
  nrfx_uarte_rx_enable(&dev->periph, NRFX_UARTE_RX_ENABLE_CONT | NRFX_UARTE_RX_ENABLE_KEEP_FIFO_CONTENT);
}

void uart_stop_rx_dma(UARTDevice *dev) {
  nrfx_uarte_rx_abort(&dev->periph, true, true);
  nrfx_timer_disable(&dev->counter);
}

void uart_clear_rx_dma_buffer(UARTDevice *dev) {
}
