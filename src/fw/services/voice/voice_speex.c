/* SPDX-FileCopyrightText: 2025 Joshua Jun */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/voice/voice_speex.h"

#include "board/board.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "kernel/pbl_malloc.h"
#include <pbl/logging/logging.h>
#include "pbl/services/audio_endpoint.h"
#include <pbl/drivers/mic.h>

#include "speex/speex.h"
#include "speex/speex_bits.h"
#include "speex/speex_header.h"
#include "speex/speex_stereo.h"

#include <string.h>
#include <inttypes.h>

PBL_LOG_MODULE_DECLARE(service_voice, CONFIG_SERVICE_VOICE_LOG_LEVEL);

// External Speex mode declarations
extern const SpeexMode speex_wb_mode;

// Speex bitstream version
#define SPEEX_BITSTREAM_VERSION 4

// Speex encoder state
typedef struct {
  void *enc_state;
  SpeexBits bits;
  SpeexStereoState stereo_state;
  uint32_t frame_size;       // Changed to uint32_t to match transfer info
  uint32_t sample_rate;      // Changed to uint32_t to match transfer info
  uint16_t bit_rate;         // Changed to uint16_t to match transfer info
  uint8_t bitstream_version; // Changed to uint8_t to match transfer info
  uint8_t channels;          // 1 for mono, 2 for stereo
  bool initialized;
  uint8_t *frame_buffer;
  size_t frame_buffer_size;
  uint8_t *encoded_buffer;
  size_t encoded_buffer_size;
} VoiceSpeexEncoder;

static VoiceSpeexEncoder s_encoder = {0};

// Speex configuration
#define SPEEX_SAMPLE_RATE 16000  // 16 kHz wideband
#define SPEEX_BIT_RATE 9800      // 9.8 kbps
#define SPEEX_QUALITY VOICE_SPEEX_QUALITY_DEFAULT
#define SPEEX_COMPLEXITY 1       // Complexity (1-10, lower for embedded)
#define SPEEX_ENCODED_BUFFER_SIZE 320  // Max encoded frame size
#define SPEEX_AUDIO_GAIN 2       // Audio gain multiplier

bool voice_speex_init(void) {
  if (s_encoder.initialized) {
    return true;
  }

  memset(&s_encoder, 0, sizeof(s_encoder));

  // Get channel count from mic device (default to mono if not specified)
  s_encoder.channels = (uint8_t)mic_get_channels(MIC);
  PBL_LOG_DBG("Mic channels: %"PRIu8, s_encoder.channels);

  // Initialize Speex encoder - use wideband mode for 16kHz sample rate
  const SpeexMode *mode = &speex_wb_mode;
  if (!mode) {
    PBL_LOG_ERR("Failed to get Speex wideband mode");
    return false;
  }

  s_encoder.enc_state = speex_encoder_init(mode);
  if (!s_encoder.enc_state) {
    PBL_LOG_ERR("Failed to initialize Speex encoder");
    return false;
  }

  // Initialize bits structure
  speex_bits_init(&s_encoder.bits);

  // Initialize stereo state if stereo
  if (s_encoder.channels == 2) {
    s_encoder.stereo_state = (SpeexStereoState)SPEEX_STEREO_STATE_INIT;
  }

  // Get frame size
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_GET_FRAME_SIZE, &s_encoder.frame_size);
  PBL_LOG_DBG("Initial frame size from Speex: %"PRIu32, s_encoder.frame_size);

  // Set encoder parameters
  int tmp = SPEEX_QUALITY;
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_SET_QUALITY, &tmp);

  tmp = SPEEX_COMPLEXITY;
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_SET_COMPLEXITY, &tmp);

  tmp = SPEEX_SAMPLE_RATE;
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_SET_SAMPLING_RATE, &tmp);
  PBL_LOG_DBG("Set sample rate to: %d", tmp);

  tmp = SPEEX_BIT_RATE;
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_SET_BITRATE, &tmp);
  PBL_LOG_DBG("Set bit rate to: %d", tmp);

  // Get actual parameters
  int actual_sample_rate, actual_bit_rate;
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_GET_SAMPLING_RATE, &actual_sample_rate);
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_GET_BITRATE, &actual_bit_rate);

  s_encoder.sample_rate = (uint32_t)actual_sample_rate;
  s_encoder.bit_rate = (uint16_t)actual_bit_rate;

  s_encoder.bitstream_version = SPEEX_BITSTREAM_VERSION;

  // Allocate frame buffer (16-bit samples, multiplied by channel count for stereo)
  s_encoder.frame_buffer_size = s_encoder.frame_size * sizeof(int16_t) * s_encoder.channels;
  s_encoder.frame_buffer = (uint8_t *)kernel_malloc_check(s_encoder.frame_buffer_size);

  // Allocate encoded buffer
  s_encoder.encoded_buffer_size = SPEEX_ENCODED_BUFFER_SIZE;
  s_encoder.encoded_buffer = kernel_malloc_check(s_encoder.encoded_buffer_size);

  s_encoder.initialized = true;

  PBL_LOG_DBG("Speex encoder initialized: sample_rate=%"PRIu32", bit_rate=%"PRIu16", frame_size=%"PRIu32", channels=%"PRIu8,
              s_encoder.sample_rate, s_encoder.bit_rate, s_encoder.frame_size, s_encoder.channels);

  // Verify sample rates match
  if (s_encoder.sample_rate != MIC_SAMPLE_RATE) {
    PBL_LOG_WRN("Speex sample rate (%"PRIu32") != Mic sample rate (%d)",
                s_encoder.sample_rate, MIC_SAMPLE_RATE);
  }

  return true;
}

void voice_speex_deinit(void) {
  if (!s_encoder.initialized) {
    return;
  }

  if (s_encoder.enc_state) {
    speex_encoder_destroy(s_encoder.enc_state);
    s_encoder.enc_state = NULL;
  }

  speex_bits_destroy(&s_encoder.bits);


  if (s_encoder.frame_buffer) {
    kernel_free(s_encoder.frame_buffer);
    s_encoder.frame_buffer = NULL;
  }

  if (s_encoder.encoded_buffer) {
    kernel_free(s_encoder.encoded_buffer);
    s_encoder.encoded_buffer = NULL;
  }

  memset(&s_encoder, 0, sizeof(s_encoder));
}

void voice_speex_get_transfer_info(AudioTransferInfoSpeex *info) {
  PBL_ASSERTN(s_encoder.initialized);
  PBL_ASSERTN(info);

  memset(info, 0, sizeof(AudioTransferInfoSpeex));
  strncpy(info->version, "1.2.1", sizeof(info->version) - 1);
  info->sample_rate = s_encoder.sample_rate;
  info->bit_rate = s_encoder.bit_rate;
  info->frame_size = (uint16_t)s_encoder.frame_size;  // Explicit cast to uint16_t
  info->bitstream_version = s_encoder.bitstream_version;
  
  PBL_LOG_DBG("Transfer info: sample_rate=%"PRIu32", bit_rate=%"PRIu16", frame_size=%"PRIu16", bitstream_version=%"PRIu8,
              info->sample_rate, info->bit_rate, info->frame_size, info->bitstream_version);
  
  // Additional validation
  if (info->sample_rate != 16000) {
    PBL_LOG_WRN("Unexpected sample rate in transfer info: %"PRIu32, info->sample_rate);
  }
}

int voice_speex_get_frame_size(void) {
  // Return total samples per frame (frame_size * channels for stereo)
  return s_encoder.initialized ? (int)(s_encoder.frame_size * s_encoder.channels) : 0;
}

int16_t *voice_speex_get_frame_buffer(void) {
  return s_encoder.initialized ? (int16_t *)s_encoder.frame_buffer : NULL;
}

size_t voice_speex_get_frame_buffer_size(void) {
  return s_encoder.initialized ? s_encoder.frame_buffer_size : 0;
}

int voice_speex_encode_frame(int16_t *samples, uint8_t *encoded_data, size_t max_encoded_size) {
  if (!s_encoder.initialized) {
    PBL_LOG_ERR("encode_frame called but Speex not initialized");
    return -1;
  }

  if (!samples || !encoded_data) {
    PBL_LOG_ERR("encode_frame called with invalid buffers");
    return -1;
  }

  uint32_t total_samples = s_encoder.frame_size * s_encoder.channels;

  // Apply gain boost to samples
  for (uint32_t i = 0; i < total_samples; i++) {
    int32_t boosted = (int32_t)samples[i] * SPEEX_AUDIO_GAIN;
    // Clamp to int16_t range to prevent overflow
    if (boosted > INT16_MAX) {
      boosted = INT16_MAX;
    } else if (boosted < INT16_MIN) {
      boosted = INT16_MIN;
    }
    samples[i] = (int16_t)boosted;
  }

  // Reset bits structure
  speex_bits_reset(&s_encoder.bits);

  if (s_encoder.channels == 2) {
    // For stereo: encode stereo info and convert to mono in-place
    speex_encode_stereo_int(samples, s_encoder.frame_size, &s_encoder.bits);
  }

  // Encode frame (for stereo, samples have been converted to mono in-place by speex_encode_stereo_int)
  speex_encode_int(s_encoder.enc_state, (spx_int16_t *)samples, &s_encoder.bits);

  // Write encoded data to buffer
  int encoded_bytes = speex_bits_write(&s_encoder.bits, (char *)encoded_data, max_encoded_size);
  
  if (encoded_bytes < 0) {
    PBL_LOG_ERR("Failed to write Speex encoded data (returned %d)", encoded_bytes);
    return -1;
  }

  PBL_LOG_VERBOSE("Encoded frame: input_samples=%"PRIu32", output_bytes=%d, frame_size=%"PRIu32", channels=%"PRIu8,
                  total_samples, encoded_bytes, s_encoder.frame_size, s_encoder.channels);

  return encoded_bytes;
}

bool voice_speex_is_initialized(void) {
  return s_encoder.initialized;
}

bool voice_speex_set_quality(int quality) {
  if (!s_encoder.initialized) {
    return false;
  }

  int tmp = quality;
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_SET_QUALITY, &tmp);

  // Keep the cached bit rate in sync: it is reported to the phone in the
  // session transfer info and stored in recording headers.
  int bit_rate;
  speex_encoder_ctl(s_encoder.enc_state, SPEEX_GET_BITRATE, &bit_rate);
  s_encoder.bit_rate = (uint16_t)bit_rate;

  PBL_LOG_DBG("Speex encoder quality set to %d (bit_rate=%" PRIu16 ")", quality,
              s_encoder.bit_rate);
  return true;
}

// Speex decoder state (used for on-device playback of recorded frames)
typedef struct {
  void *dec_state;
  SpeexBits bits;
  uint32_t frame_size;
} VoiceSpeexDecoder;

static VoiceSpeexDecoder s_decoder;

bool voice_speex_decoder_init(void) {
  if (s_decoder.dec_state) {
    return true;
  }

  s_decoder.dec_state = speex_decoder_init(&speex_wb_mode);
  if (!s_decoder.dec_state) {
    PBL_LOG_ERR("Failed to initialize Speex decoder");
    return false;
  }

  speex_bits_init(&s_decoder.bits);
  speex_decoder_ctl(s_decoder.dec_state, SPEEX_GET_FRAME_SIZE, &s_decoder.frame_size);

  // Enable perceptual enhancement for nicer playback.
  int enh = 1;
  speex_decoder_ctl(s_decoder.dec_state, SPEEX_SET_ENH, &enh);

  PBL_LOG_DBG("Speex decoder initialized: frame_size=%" PRIu32, s_decoder.frame_size);
  return true;
}

void voice_speex_decoder_deinit(void) {
  if (!s_decoder.dec_state) {
    return;
  }
  speex_decoder_destroy(s_decoder.dec_state);
  speex_bits_destroy(&s_decoder.bits);
  memset(&s_decoder, 0, sizeof(s_decoder));
}

int voice_speex_get_decoder_frame_size(void) {
  return s_decoder.dec_state ? (int)s_decoder.frame_size : 0;
}

int voice_speex_decode_frame(const uint8_t *encoded, int len, int16_t *out_pcm) {
  if (!s_decoder.dec_state || !encoded || !out_pcm || (len <= 0)) {
    return -1;
  }

  speex_bits_read_from(&s_decoder.bits, (const char *)encoded, len);
  // speex_decode_int returns 0 on success, negative on end-of-stream / corrupt frame.
  if (speex_decode_int(s_decoder.dec_state, &s_decoder.bits, (spx_int16_t *)out_pcm) != 0) {
    return -1;
  }
  return (int)s_decoder.frame_size;
}
