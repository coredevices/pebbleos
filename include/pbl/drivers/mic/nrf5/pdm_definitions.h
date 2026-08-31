/* SPDX-FileCopyrightText: 2025 Joshua Jun */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "board/board.h"
#include <pbl/drivers/mic.h>
#include "pbl/os/mutex.h"
#include "pbl/util/circular_buffer.h"

#include <stdbool.h>
#include <stdint.h>

// PDM Configuration
#define PDM_BUFFER_SIZE_SAMPLES    (320)
#define PDM_BUFFER_COUNT           (2)
#define PDM_GAIN_DEFAULT           (NRF_PDM_GAIN_DEFAULT)

// Circular buffer configuration
#define CIRCULAR_BUF_SIZE_MS       (320)
#define CIRCULAR_BUF_SIZE_SAMPLES  ((MIC_SAMPLE_RATE * CIRCULAR_BUF_SIZE_MS) / 1000)
#define CIRCULAR_BUF_SIZE_BYTES    (CIRCULAR_BUF_SIZE_SAMPLES * sizeof(int16_t))

// Minimum fallback size when the heap is fragmented and the full circular
// buffer alloc fails. Below this transcription quality degrades noticeably.
#define CIRCULAR_BUF_MIN_SIZE_MS       (128)
#define CIRCULAR_BUF_MIN_SIZE_SAMPLES  ((MIC_SAMPLE_RATE * CIRCULAR_BUF_MIN_SIZE_MS) / 1000)
#define CIRCULAR_BUF_MIN_SIZE_BYTES    (CIRCULAR_BUF_MIN_SIZE_SAMPLES * sizeof(int16_t))

// 32 ms shrink per retry gives ~7 attempts between 320 ms and 128 ms.
#define CIRCULAR_BUF_STEP_MS           (32)
#define CIRCULAR_BUF_STEP_SAMPLES      ((MIC_SAMPLE_RATE * CIRCULAR_BUF_STEP_MS) / 1000)
#define CIRCULAR_BUF_STEP_BYTES        (CIRCULAR_BUF_STEP_SAMPLES * sizeof(int16_t))

typedef struct {
  nrfx_pdm_config_t pdm_config;
  int16_t *pdm_buffers[PDM_BUFFER_COUNT];
  uint8_t current_buffer_idx;
  
  // User interface
  MicDataHandlerCB data_handler;
  void *handler_context;
  int16_t *audio_buffer;
  size_t audio_buffer_len;
  
  // Intermediate storage
  CircularBuffer circ_buffer;
  uint8_t *circ_buffer_storage;
  uint16_t circ_buffer_size;
  
  // State management
  PebbleRecursiveMutex *mutex;
  bool is_running;
  bool is_initialized;
  bool main_pending;
} MicDeviceState;

typedef const struct MicDevice {
  MicDeviceState *state;
  
  // Hardware configuration
  const nrfx_pdm_t pdm_instance;
  uint32_t clk_pin;
  uint32_t data_pin;
  uint32_t channels;
} MicDevice;
