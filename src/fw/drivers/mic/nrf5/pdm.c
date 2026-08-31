/* SPDX-FileCopyrightText: 2025 Joshua Jun */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/mic.h>
#include <pbl/drivers/mic/nrf5/pdm_definitions.h>

#include "board/board.h"
#include <pbl/drivers/clocksource.h>
#include "kernel/events.h"
#include "kernel/kernel_heap.h"
#include "kernel/pbl_malloc.h"
#include "kernel/util/sleep.h"
#include "pbl/os/mutex.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/circular_buffer.h"
#include "pbl/util/heap.h"
#include "util/time/time.h"

#include "nrfx_pdm.h"

PBL_LOG_MODULE_DEFINE(driver_mic_nrf5, CONFIG_DRIVER_MIC_LOG_LEVEL);

static void prv_pdm_event_handler(nrfx_pdm_evt_t const *p_evt);
static void prv_dispatch_samples_system_task(void *data);
static bool prv_allocate_buffers(MicDeviceState *state);
static void prv_free_buffers(MicDeviceState *state);

static bool prv_is_valid_buffer(MicDeviceState *state, int16_t *buffer) {
  for (int i = 0; i < PDM_BUFFER_COUNT; i++) {
    if (state->pdm_buffers[i] && buffer == state->pdm_buffers[i]) {
      return true;
    }
  }
  return false;
}

static bool prv_allocate_buffers(MicDeviceState *state) {
  // The kernel heap fragments over time, so a single 10 KB contiguous alloc
  // can fail even when plenty of memory is free. Shrink the request in 32 ms
  // steps until it fits or we hit the 128 ms floor.
  uint16_t try_size = CIRCULAR_BUF_SIZE_BYTES;
  uint8_t *storage = NULL;

  while (try_size >= CIRCULAR_BUF_MIN_SIZE_BYTES) {
    storage = kernel_malloc(try_size);
    if (storage) {
      break;
    }
    try_size -= CIRCULAR_BUF_STEP_BYTES;
  }

  if (!storage) {
    unsigned int used, free_bytes, max_free;
    heap_calc_totals(kernel_heap_get(), &used, &free_bytes, &max_free);
    PBL_LOG_ERR("Failed to allocate PDM circular buffer (min %u B, max_free %u B)",
                (unsigned)CIRCULAR_BUF_MIN_SIZE_BYTES, max_free);
    return false;
  }

  if (try_size < CIRCULAR_BUF_SIZE_BYTES) {
    unsigned int used, free_bytes, max_free;
    heap_calc_totals(kernel_heap_get(), &used, &free_bytes, &max_free);
    PBL_LOG_WRN("PDM circular buffer fell back to %u B (requested %u, max_free %u)",
                (unsigned)try_size, (unsigned)CIRCULAR_BUF_SIZE_BYTES, max_free);
  }

  state->circ_buffer_storage = storage;
  state->circ_buffer_size = try_size;

  // Allocate PDM buffers
  for (int i = 0; i < PDM_BUFFER_COUNT; i++) {
    size_t buffer_size = PDM_BUFFER_SIZE_SAMPLES * sizeof(int16_t);
    state->pdm_buffers[i] = kernel_malloc(buffer_size);
    if (!state->pdm_buffers[i]) {
      PBL_LOG_ERR("Failed to allocate PDM buffer %d", i);
      // Free any previously allocated buffers
      prv_free_buffers(state);
      return false;
    }
    // Clear the buffer
    memset(state->pdm_buffers[i], 0, buffer_size);
  }
  
  // Initialize circular buffer with allocated storage
  circular_buffer_init(&state->circ_buffer, state->circ_buffer_storage, try_size);
  
  return true;
}

static void prv_free_buffers(MicDeviceState *state) {
  // Free circular buffer storage
  if (state->circ_buffer_storage) {
    kernel_free(state->circ_buffer_storage);
    state->circ_buffer_storage = NULL;
  }
  state->circ_buffer_size = 0;
  
  // Free PDM buffers
  for (int i = 0; i < PDM_BUFFER_COUNT; i++) {
    if (state->pdm_buffers[i]) {
      kernel_free(state->pdm_buffers[i]);
      state->pdm_buffers[i] = NULL;
    }
  }
}

static void prv_process_pdm_buffer(MicDeviceState *state, int16_t *pdm_data) {
  // Ensure we're still running and have valid state
  if (!state->is_running) {
    PBL_LOG_DBG("prv_process_pdm_buffer: Not running, ignoring data");
    return;
  }
  
  // Ensure circular buffer storage is allocated
  if (!state->circ_buffer_storage) {
    PBL_LOG_DBG("prv_process_pdm_buffer: No circular buffer storage, ignoring data");
    return;
  }
  
  // Ensure we have valid audio buffer info
  if (!state->audio_buffer || state->audio_buffer_len == 0) {
    PBL_LOG_DBG("prv_process_pdm_buffer: No audio buffer configured, ignoring data");
    return;
  }
  
  // Write samples to circular buffer
  uint32_t samples_written = 0;
  for (int i = 0; i < PDM_BUFFER_SIZE_SAMPLES; i++) {
    if (!circular_buffer_write(&state->circ_buffer,
                              (const uint8_t *)&pdm_data[i],
                              sizeof(int16_t))) {
      break;  // Buffer is full, drop remaining samples
    }
    samples_written++;
  }

  // Monitor for buffer overruns (dropped samples)
  static uint32_t total_samples = 0, dropped_samples = 0;
  total_samples += PDM_BUFFER_SIZE_SAMPLES;
  dropped_samples += (PDM_BUFFER_SIZE_SAMPLES - samples_written);

  // Monitor buffer utilization
  uint16_t buffer_free = circular_buffer_get_write_space_remaining(&state->circ_buffer);
  uint16_t buffer_total = state->circ_buffer_size;
  uint16_t buffer_used = buffer_total - buffer_free;
  uint16_t buffer_utilization = buffer_total ? (buffer_used * 100) / buffer_total : 0;

  // Log dropout statistics periodically
  static uint32_t log_counter = 0;
  if (++log_counter >= 100) { // Every 100 buffers (~2 seconds)
    if (dropped_samples > 0) {
      // Calculate percentage using integer arithmetic (x10 for one decimal place)
      uint32_t percent_x10 = (dropped_samples * 1000) / total_samples;
      PBL_LOG_DBG("Audio dropouts: %"PRIu32"/%"PRIu32" samples dropped (%"PRIu32".%"PRIu32" percent), buffer util: %"PRIu16,
              dropped_samples, total_samples, percent_x10 / 10, percent_x10 % 10, buffer_utilization);
    } else {
      PBL_LOG_DBG("Audio buffer utilization: %"PRIu16" (%"PRIu16"/%"PRIu16" bytes)",
              buffer_utilization, buffer_used, buffer_total);
    }
    log_counter = 0;
    total_samples = dropped_samples = 0; // Reset counters
  }
  
  // Check if we have enough data for a complete frame
  size_t frame_size_bytes = state->audio_buffer_len * sizeof(int16_t);
  uint16_t available_data = circular_buffer_get_read_space_remaining(&state->circ_buffer);
  
  if (available_data >= frame_size_bytes && !state->main_pending) {
    state->main_pending = true;

    // Dispatch to low-priority system task instead of kernel event queue.
    // A drop is retried on the next PDM buffer event; losing samples beats
    // resetting the system over a full queue.
    bool should_context_switch = false;
    if (!system_task_add_callback_from_isr_droppable(prv_dispatch_samples_system_task, NULL,
                                                     &should_context_switch)) {
      state->main_pending = false;
    }
  }
}

static void prv_pdm_event_handler(nrfx_pdm_evt_t const *p_evt) {
  MicDeviceState *state = MIC->state;
  
  PBL_ASSERTN(state->is_initialized);
  
  // Don't assert on is_running during shutdown - the PDM might send final events
  if (!state->is_running) {
    PBL_LOG_DBG("prv_pdm_event_handler: Microphone stopped, ignoring event");
    return;
  }
  
  PBL_ASSERTN(p_evt->error == NRFX_PDM_NO_ERROR);
  
  if (p_evt->buffer_requested) {
    uint8_t next_buffer_idx = (state->current_buffer_idx + 1) % PDM_BUFFER_COUNT;
    nrfx_err_t err = nrfx_pdm_buffer_set(&MIC->pdm_instance, 
                                        state->pdm_buffers[next_buffer_idx], 
                                        PDM_BUFFER_SIZE_SAMPLES);
    if (err == NRFX_SUCCESS) {
      state->current_buffer_idx = next_buffer_idx;
    }
  }
  
  if (p_evt->buffer_released) {
    int16_t *pdm_data = (int16_t *)p_evt->buffer_released;
    
    if (pdm_data && prv_is_valid_buffer(state, pdm_data)) {
      prv_process_pdm_buffer(state, pdm_data);
    }
  }
}

void mic_init(const MicDevice *this) {
  PBL_ASSERTN(this);
  
  MicDeviceState *state = this->state;
  
  if (state && state->is_initialized) {
    return;
  }

  memset(state, 0, sizeof(MicDeviceState));
  
  // Initialize PDM configuration
  state->pdm_config = (nrfx_pdm_config_t)NRFX_PDM_DEFAULT_CONFIG(this->clk_pin, this->data_pin);
  state->pdm_config.clock_freq = NRF_PDM_FREQ_1280K;
  state->pdm_config.ratio = NRF_PDM_RATIO_80X;
  state->pdm_config.gain_l = BOARD_CONFIG.mic_config.gain;
  state->pdm_config.gain_r = BOARD_CONFIG.mic_config.gain;
  
  // Create mutex for thread safety
  state->mutex = mutex_create_recursive();
  PBL_ASSERTN(state->mutex);
  
  // Initialize PDM driver once during init
  nrfx_err_t err = nrfx_pdm_init(&this->pdm_instance, &state->pdm_config, prv_pdm_event_handler);
  if (err != NRFX_SUCCESS) {
    PBL_LOG_ERR("Failed to initialize PDM: %d", err);
    return;
  }
  
  state->is_initialized = true;
}

// Process at most this many frames per system task callback to allow
// other tasks (especially Bluetooth) to run and prevent send buffer overflow
// This needs to be high enough to keep up with real-time audio (~50 frames/sec)
// but low enough to allow BT to drain its send buffer between batches
#define MAX_FRAMES_PER_SYSTEM_TASK_CALLBACK 64

static void prv_dispatch_samples_system_task(void *data) {
  MicDeviceState *state = MIC->state;

  // Defensive check
  if (!state || !state->is_initialized) {
    return;
  }

  mutex_lock_recursive(state->mutex);

  // Process a limited number of frames to provide backpressure
  // This prevents overwhelming the Bluetooth send buffer
  if (state->is_running && state->data_handler && state->audio_buffer && state->circ_buffer_storage) {

    size_t frame_size_bytes = state->audio_buffer_len * sizeof(int16_t);
    int frames_processed = 0;

    while (state->is_running && state->data_handler && frames_processed < MAX_FRAMES_PER_SYSTEM_TASK_CALLBACK) {
      // Check if we have enough data for a complete frame
      uint16_t available_data = circular_buffer_get_read_space_remaining(&state->circ_buffer);

      if (available_data < frame_size_bytes) {
        break;  // Not enough data for another frame
      }

      // Copy one frame
      uint16_t bytes_copied = circular_buffer_copy(&state->circ_buffer,
          (uint8_t *)state->audio_buffer,
          frame_size_bytes);

      if (bytes_copied == frame_size_bytes) {
        // Call callback with the frame
        state->data_handler(state->audio_buffer, state->audio_buffer_len, state->handler_context);

        // Consume the frame we processed
        circular_buffer_consume(&state->circ_buffer, bytes_copied);

        frames_processed++;

        // Feed the system task watchdog periodically during long processing
        system_task_watchdog_feed();
      } else {
        break;  // Failed to copy, stop processing
      }
    }

    // If we still have data available after processing, reschedule immediately
    // to continue processing on the next system task cycle
    uint16_t remaining_data = circular_buffer_get_read_space_remaining(&state->circ_buffer);
    if (remaining_data >= frame_size_bytes && state->is_running && !state->main_pending) {
      state->main_pending = true;
      if (!system_task_add_callback(prv_dispatch_samples_system_task, NULL)) {
        state->main_pending = false;
      }
    } else {
      // Clear pending flag only if we're done processing
      state->main_pending = false;
    }
  } else {
    // Clear pending flag if we can't process
    state->main_pending = false;
  }

  mutex_unlock_recursive(state->mutex);
}

void mic_set_volume(const MicDevice *this, uint16_t volume) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);
  
  MicDeviceState *state = this->state;
  
  if (state->is_running) {
    PBL_LOG_WRN("Cannot set volume while microphone is running");
    return;
  }
  
  // Scale volume from 0-1024 range to nRF PDM gain range (0-80)
  // Volume 0 = minimum gain, 1024 = maximum gain
  uint16_t nrf_gain;
  if (volume == 0) {
    nrf_gain = NRF_PDM_GAIN_MINIMUM;
  } else if (volume >= 1024) {
    nrf_gain = NRF_PDM_GAIN_MAXIMUM;
  } else {
    // Linear scaling: volume * (max - min) / 1024 + min
    nrf_gain = (volume * (NRF_PDM_GAIN_MAXIMUM - NRF_PDM_GAIN_MINIMUM)) / 1024 + NRF_PDM_GAIN_MINIMUM;
  }
  
  state->pdm_config.gain_l = nrf_gain;
  state->pdm_config.gain_r = nrf_gain;
}

static bool prv_start_pdm_capture(const MicDevice *this) {
  MicDeviceState *state = this->state;
  
  // Clear buffers and set initial buffer
  for (int i = 0; i < PDM_BUFFER_COUNT; i++) {
    if (state->pdm_buffers[i]) {
      memset(state->pdm_buffers[i], 0, PDM_BUFFER_SIZE_SAMPLES * sizeof(int16_t));
    }
  }
  state->current_buffer_idx = 0;
  
  // Check if buffers are valid
  if (!state->pdm_buffers[0] || !state->pdm_buffers[1]) {
    PBL_LOG_ERR("Invalid PDM buffers: [0]=%p [1]=%p", 
            state->pdm_buffers[0], state->pdm_buffers[1]);
    return false;
  }
  
  // Try starting PDM first, then set buffer in event handler
  nrfx_err_t err = nrfx_pdm_start(&this->pdm_instance);
  if (err != NRFX_SUCCESS) {
    PBL_LOG_ERR("Failed to start PDM: %d", err);
    return false;
  }
  
  return true;
}

bool mic_start(const MicDevice *this, MicDataHandlerCB data_handler, void *context,
               int16_t *audio_buffer, size_t audio_buffer_len) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);
  PBL_ASSERTN(data_handler);
  PBL_ASSERTN(audio_buffer);
  PBL_ASSERTN(audio_buffer_len > 0);
  
  MicDeviceState *state = this->state;
  
  mutex_lock_recursive(state->mutex);
  
  if (state->is_running) {
    PBL_LOG_WRN("Microphone is already running");
    mutex_unlock_recursive(state->mutex);
    return false;
  }
  
  if (!state->is_initialized) {
    PBL_LOG_ERR("Microphone not initialized");
    mutex_unlock_recursive(state->mutex);
    return false;
  }
  
  // Allocate buffers dynamically. prv_allocate_buffers also initializes the
  // circular buffer with the actual (possibly shrunk) size.
  if (!prv_allocate_buffers(state)) {
    PBL_LOG_ERR("Failed to allocate microphone buffers");
    mutex_unlock_recursive(state->mutex);
    return false;
  }
  
  state->data_handler = data_handler;
  state->handler_context = context;
  state->audio_buffer = audio_buffer;
  state->audio_buffer_len = audio_buffer_len;
  state->main_pending = false;
  
  // Request high frequency crystal oscillator
  clocksource_hfxo_request();
  
  // Set is_running to true BEFORE starting PDM, since the event handler will be called immediately
  state->is_running = true;
  
  // Start PDM capture
  if (!prv_start_pdm_capture(this)) {
    state->is_running = false;  // Reset on failure    
    clocksource_hfxo_release();
    prv_free_buffers(state);
    mutex_unlock_recursive(state->mutex);
    return false;
  }

  mutex_unlock_recursive(state->mutex);
  return true;
}

void mic_stop(const MicDevice *this) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);
  
  MicDeviceState *state = this->state;
  
  mutex_lock_recursive(state->mutex);
  
  if (!state->is_running) {
    mutex_unlock_recursive(state->mutex);
    return;
  }
  
  // Mark as stopped first to prevent new buffer requests
  state->is_running = false;
  
  // Stop PDM capture
  nrfx_pdm_stop(&this->pdm_instance);
  
  // Give the PDM hardware a moment to finish any pending operations
  // This helps ensure no DMA operations are still accessing our buffers
  psleep(1); // 1ms delay to let hardware settle
  
  // Release high frequency oscillator
  clocksource_hfxo_release();
  
  // Free dynamically allocated buffers
  prv_free_buffers(state);
  
  // Clear state
  state->data_handler = NULL;
  state->handler_context = NULL;
  state->audio_buffer = NULL;
  state->audio_buffer_len = 0;
  state->main_pending = false;

  mutex_unlock_recursive(state->mutex);
}

#include "console/prompt.h"

// Console command stubs for Asterix (since we don't have accessory connector)
// These commands are defined in the console command table but Asterix doesn't need
// the full accessory-based microphone streaming functionality

void command_mic_start(char *timeout_str, char *sample_size_str, char *sample_rate_str, char *format_str) {
  prompt_send_response("Microphone console commands not supported on Asterix");
  prompt_send_response("Use the standard microphone API instead");
}

void command_mic_read(void) {
  prompt_send_response("Microphone read command not supported on Asterix");
  prompt_send_response("Use the standard microphone API instead");
}

bool mic_is_running(const MicDevice *this) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);
  
  return this->state->is_running;
}

uint32_t mic_get_channels(const MicDevice *this) {
  PBL_ASSERTN(this);
  return this->channels ? this->channels : 1;
}
