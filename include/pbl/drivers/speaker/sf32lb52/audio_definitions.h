/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "board/board.h"
#include <pbl/drivers/audio.h>
#include <pbl/util/circular_buffer.h>

#include <stdbool.h>
#include <stdint.h>

#define CFG_AUDIO_PLAYBACK_PIPE_SIZE          (1024)

// Circular buffer configuration
#define CIRCULAR_BUF_SIZE_MS       (128)
#define CIRCULAR_BUF_SIZE_SAMPLES  ((MIC_SAMPLE_RATE * CIRCULAR_BUF_SIZE_MS) / 1000)
#define CIRCULAR_BUF_SIZE_BYTES    (CIRCULAR_BUF_SIZE_SAMPLES * sizeof(int16_t))

typedef enum AUDIO_PLL_STATE_TAG
{
    AUDIO_PLL_CLOSED,
    AUDIO_PLL_OPEN,
    AUDIO_PLL_ENABLE,
} AUDIO_PLL_STATE;

typedef struct AudioState {
  AUDCODEC_HandleTypeDef audcodec;
  DMA_HandleTypeDef dac_dma_handle;
  AUDPRC_HandleTypeDef audprc;
  uint32_t slot_valid;
  uint8_t *queue_buf[HAL_AUDPRC_INSTANC_CNT];
  uint8_t *audec_queue_buf[HAL_AUDCODEC_INSTANC_CNT];
  AUDIO_PLL_STATE pll_state;
  uint32_t pll_samplerate;
  uint8_t tx_instanc;
  bool    tx_rbf_enable;
  uint16_t tx_buffer_size;
  uint8_t *circ_buffer_storage;
  CircularBuffer circ_buffer;
  AudioTransCB trans_cb;
  //! Set while a prv_audio_trans_bg refill callback is queued on the system
  //! task; the DMA ISR must not enqueue another until it has run.
  volatile bool callback_pending;
  uint8_t volume;
  //! Raw (unaligned) pointer returned by kernel_malloc for the AUDCODEC DAC
  //! DMA buffer. haudcodec->buf[] is bumped up to a cache-line boundary so
  //! dcache_flush() of one half can't touch the other half's lines.
  uint8_t *raw_dac_buffer;
} AudioDeviceState;

typedef const struct AudioDevice {
  AudioDeviceState *state;
  uint32_t irq_priority; 
  DMA_Channel_TypeDef *audprc_dma_channel;
  uint32_t audprc_dma_request;
  IRQn_Type audprc_dma_irq;
  DMA_Channel_TypeDef *audec_dma_channel;
  uint32_t audec_dma_request;
  IRQn_Type audec_dma_irq;
  OutputConfig pa_ctrl;
  const BoardPowerOps *power_ops;
  uint8_t data_format;
  uint8_t data_mode;
  uint32_t samplerate;
  uint32_t channels;
} AudioDevice;

extern void audprc_dma_iqr_handler(AudioDevice* audio_device);
extern void audec_dac0_dma_irq_handler(AudioDevice* audio_device);
