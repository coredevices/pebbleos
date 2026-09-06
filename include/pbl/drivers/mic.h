/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MIC_SAMPLE_RATE     (16000) //!< Microphone audio data sample rate
#define MIC_DEFAULT_VOLUME  (-1)

typedef const struct MicDevice MicDevice;

//! Microphone audio data handler callback. Called when buffer is full
typedef void (*MicDataHandlerCB)(int16_t *samples, size_t sample_count, void *context);

//! Initialize microphone driver. Should be called on boot
void mic_init(MicDevice *this);

//! Set the mic volume.  This must be called after mic_init, and not while the mic is running.
void mic_set_volume(MicDevice *this, uint16_t volume);

//! Start the microphone. The driver will fill the specified buffer with up to the specified size
//! each time it calls the audio data handler callback. audio_buffer_len should be specified as the
//! length of the buffer (number of 16-bit samples it can hold)
//! @return true if mic was started, false if mic was already running
bool mic_start(MicDevice *this, MicDataHandlerCB data_handler, void *context,
               int16_t *audio_buffer, size_t audio_buffer_len);

//! Stop the microphone. If buffer is not full, the remaining samples will be abandoned. No more
//! callbacks will be executed nor data copied into the buffer after this returns
void mic_stop(MicDevice *this);

//! Indicates whether the mic is running
bool mic_is_running(MicDevice *this);

//! Get the number of audio channels (1 for mono, 2 for stereo)
//! @return number of channels, or 1 if not specified
uint32_t mic_get_channels(MicDevice *this);
