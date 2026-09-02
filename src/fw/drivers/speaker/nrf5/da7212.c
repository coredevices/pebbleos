/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/speaker/nrf5/da7212_definitions.h>

#include "board/board.h"
#include <pbl/drivers/audio.h>
#include <pbl/drivers/clocksource.h>
#include <pbl/drivers/i2c.h>
#include "kernel/pbl_malloc.h"
#include "kernel/util/sleep.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/circular_buffer.h"
#include "pbl/util/math.h"

#include "nrfx_i2s.h"

PBL_LOG_MODULE_DEFINE(driver_speaker_da7212, CONFIG_DRIVER_SPEAKER_LOG_LEVEL);

// Hold the codec + I2S warm for this long after audio_stop before powering
// down. Rapid play/pause spamming used to phaser the output until hard power
// cycle; deferring the SYSTEM_ACTIVE=0 keeps the LINE output continuous
// across pauses and the analog domain never gets disturbed.
#define AUDIO_IDLE_SHUTDOWN_MS 1000

typedef enum {
  AudioPwrCold,
  AudioPwrWarm,
  AudioPwrActive,
} AudioPwrState;

static AudioPwrState s_pwr_state = AudioPwrCold;
static TimerID s_idle_timer = TIMER_INVALID_ID;
// Serializes cold/warm/active transitions and their I/O so audio_start,
// audio_stop, and prv_idle_shutdown can't race across threads.
static PBL_MUTEX_DEFINE(s_audio_mutex);

static void prv_idle_shutdown(void *data);

// ---------------------------------------------------------------------------
// DA7212 codec registers used by the driver.
// ---------------------------------------------------------------------------
#define DA7212_PLL_STATUS            0x03
#define DA7212_CIF_CTRL              0x1D
#define DA7212_DIG_ROUTING_DAI       0x21
#define DA7212_SR                    0x22
#define DA7212_REFERENCES            0x23
#define DA7212_PLL_FRAC_TOP          0x24
#define DA7212_PLL_FRAC_BOT          0x25
#define DA7212_PLL_INTEGER           0x26
#define DA7212_PLL_CTRL              0x27
#define DA7212_DAI_CLK_MODE          0x28
#define DA7212_DAI_CTRL              0x29
#define DA7212_DIG_ROUTING_DAC       0x2A
#define DA7212_DAC_FILTERS5          0x40
#define DA7212_DAC_R_GAIN            0x46
#define DA7212_LINE_GAIN             0x4A
#define DA7212_MIXOUT_R_SELECT       0x4C
#define DA7212_SYSTEM_MODES_OUTPUT   0x51
#define DA7212_DAC_R_CTRL            0x6A
#define DA7212_LINE_CTRL             0x6D
#define DA7212_MIXOUT_R_CTRL         0x6F
#define DA7212_LDO_CTRL              0x90
#define DA7212_GAIN_RAMP_CTRL        0x92
#define DA7212_SYSTEM_ACTIVE         0xFD

// DAC_R_GAIN value corresponding to 0 dB per DA7212 datasheet.
#define DA7212_DAC_R_GAIN_0DB        0x6f
// Attenuation span, in 0.75 dB DAC_R_GAIN steps, that volume 1..100 maps
// onto (64 steps = 48 dB).
#define DA7212_DAC_GAIN_VOL_RANGE_STEPS 64

#define I2S_BUF_SAMPLES_STEREO       (NRF5_AUDIO_I2S_BUF_SAMPLES_MONO * 2)
#define I2S_BUF_SIZE_BYTES           (I2S_BUF_SAMPLES_STEREO * sizeof(int16_t))
// nrfx_i2s buffer_size is counted in 32-bit words.
#define I2S_BUF_SIZE_WORDS           (I2S_BUF_SIZE_BYTES / sizeof(uint32_t))

static void prv_i2s_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status);

// ---------------------------------------------------------------------------
// DA7212 helpers.
// ---------------------------------------------------------------------------

static void prv_codec_write(AudioDevice *dev, uint8_t reg, uint8_t value) {
  uint8_t data[2] = { reg, value };
  i2c_use(dev->codec);
  bool ok = i2c_write_block(dev->codec, sizeof(data), data);
  i2c_release(dev->codec);
  PBL_ASSERTN(ok);
}

static uint8_t prv_codec_read(AudioDevice *dev, uint8_t reg) {
  uint8_t value = 0;
  i2c_use(dev->codec);
  bool ok = i2c_read_register(dev->codec, reg, &value);
  i2c_release(dev->codec);
  PBL_ASSERTN(ok);
  return value;
}

// DAC_R_GAIN is 0.75 dB per step with 0x6F = 0 dB. Volume 100 maps to 0 dB
// (codes above 0x6F amplify and can clip full-scale samples); lower volumes
// attenuate linearly in dB, which tracks perceived loudness better than
// scaling the register code linearly.
static uint8_t prv_volume_to_dac_gain(uint8_t volume) {
  return DA7212_DAC_R_GAIN_0DB -
         (uint8_t)(((100 - volume) * DA7212_DAC_GAIN_VOL_RANGE_STEPS) / 100);
}

// Push the cached volume to the codec: soft-mute at 0, otherwise the mapped
// DAC gain with soft-mute cleared.
static void prv_apply_volume(AudioDevice *dev) {
  AudioDeviceState *state = dev->state;
  if (state->volume == 0) {
    prv_codec_write(dev, DA7212_DAC_FILTERS5, 0x80);
    return;
  }
  prv_codec_write(dev, DA7212_DAC_R_GAIN, prv_volume_to_dac_gain(state->volume));
  prv_codec_write(dev, DA7212_DAC_FILTERS5, 0x00);
}

// Bring the DA7212 out of standby, lock its PLL and configure the DAC/LINE
// path — but leave the DAI (BCLK/WCLK) disabled. The caller starts the nRF
// I2S peripheral in slave mode in between, then calls prv_codec_start_dai()
// below to have the codec begin driving BCLK/WCLK into the nRF.
//
// We run the codec as I2S master because the nRF's 32 MHz clock tree can't
// divide down to exactly 16 kHz LRCK, while the DA7212 PLL can synthesize
// exactly 12.288 MHz system clock from our 4 MHz MCK and emit a true 16 kHz
// WCLK when SR=0x05. Register values are taken from
// src/fw/apps/prf/mfg_mic_asterix.c (PLL setup) and
// src/fw/apps/prf/mfg_speaker_asterix.c (codec-master DAI mode).
static void prv_codec_prepare(AudioDevice *dev) {
  prv_codec_write(dev, DA7212_CIF_CTRL, 0x80);
  psleep(10);

  prv_codec_write(dev, DA7212_SYSTEM_ACTIVE, 0x01);
  prv_codec_write(dev, DA7212_REFERENCES, 0x08);
  psleep(30);

  prv_codec_write(dev, DA7212_LDO_CTRL, 0x80);

  // PLL: MCLK=4MHz (input divider 2), target system clock 12.288MHz for
  // SR=16kHz. VCO = 98.304MHz => integer=49, frac=1245 (0x4dd).
  prv_codec_write(dev, DA7212_PLL_FRAC_TOP, 0x04);
  prv_codec_write(dev, DA7212_PLL_FRAC_BOT, 0xdd);
  prv_codec_write(dev, DA7212_PLL_INTEGER, 0x31);
  prv_codec_write(dev, DA7212_PLL_CTRL, 0xC0);

  // DA7212 rev 3.6 13.29 workaround for 2-5MHz MCLK range.
  prv_codec_write(dev, 0xF0, 0x8B);
  prv_codec_write(dev, 0xF2, 0x03);
  prv_codec_write(dev, 0xF0, 0x00);
  psleep(40);

  PBL_ASSERT(prv_codec_read(dev, DA7212_PLL_STATUS) == 0x07,
             "DA7212 PLL not locked");

  // Gain ramp off: the mfg test uses a multi-second ramp for smooth
  // mic-capture playback, but for the speaker service we want audio as soon
  // as the stream opens.
  prv_codec_write(dev, DA7212_GAIN_RAMP_CTRL, 0x00);
  prv_codec_write(dev, DA7212_SR, 0x05);

  prv_codec_write(dev, DA7212_DIG_ROUTING_DAI, 0x32);
  prv_codec_write(dev, DA7212_DIG_ROUTING_DAC, 0xba);

  prv_codec_write(dev, DA7212_DAC_R_CTRL, 0x80);
  prv_codec_write(dev, DA7212_MIXOUT_R_SELECT, 0x08);
  // amp + mix enable, softmix off (softmix slow-ramps each routing change).
  prv_codec_write(dev, DA7212_MIXOUT_R_CTRL, 0x90);

  prv_codec_write(dev, DA7212_LINE_GAIN, 0x30);
  prv_codec_write(dev, DA7212_LINE_CTRL, 0x80);

  prv_codec_write(dev, DA7212_SYSTEM_MODES_OUTPUT, 0x89);

  // Apply the cached volume (gain + soft-mute state) now that the path is up
  // (DAI still disabled).
  prv_apply_volume(dev);
}

// Enable the DA7212 DAI as I2S master. This is the point at which BCLK/WCLK
// begin toggling on P0.12 / P0.07; the nRF I2S must already be armed in slave
// mode when this runs.
static void prv_codec_start_dai(AudioDevice *dev) {
  // DAI_CTRL: enable, 16-bit samples. Codec is still in slave-clock mode at
  // this point, so its DAI output pins remain high-Z.
  prv_codec_write(dev, DA7212_DAI_CTRL, 0x80);
  // DAI_CLK_MODE: master + BCLKS_PER_WCLK=001 (64 BCLK per WCLK frame). The
  // codec now drives BCLK and WCLK; nRF I2S slave picks them up.
  prv_codec_write(dev, DA7212_DAI_CLK_MODE, 0x81);
}

static void prv_codec_mute(AudioDevice *dev) {
  prv_codec_write(dev, DA7212_DAC_FILTERS5, 0x80);
}

static void prv_codec_power_down(AudioDevice *dev) {
  prv_codec_write(dev, DA7212_SYSTEM_ACTIVE, 0x00);
}

// ---------------------------------------------------------------------------
// Buffer management.
// ---------------------------------------------------------------------------

static bool prv_allocate_buffers(AudioDeviceState *state) {
  state->circ_buffer_storage = kernel_malloc(NRF5_AUDIO_CIRC_BUF_SIZE_BYTES);
  if (!state->circ_buffer_storage) {
    PBL_LOG_ERR("Failed to allocate audio circular buffer");
    return false;
  }

  for (int i = 0; i < NRF5_AUDIO_I2S_BUF_COUNT; i++) {
    state->i2s_bufs[i] = kernel_malloc(I2S_BUF_SIZE_BYTES);
    if (!state->i2s_bufs[i]) {
      PBL_LOG_ERR("Failed to allocate I2S buffer %d", i);
      for (int j = 0; j < i; j++) {
        kernel_free(state->i2s_bufs[j]);
        state->i2s_bufs[j] = NULL;
      }
      kernel_free(state->circ_buffer_storage);
      state->circ_buffer_storage = NULL;
      return false;
    }
    memset(state->i2s_bufs[i], 0, I2S_BUF_SIZE_BYTES);
  }

  circular_buffer_init(&state->circ_buffer, state->circ_buffer_storage,
                       NRF5_AUDIO_CIRC_BUF_SIZE_BYTES);
  return true;
}

static void prv_free_buffers(AudioDeviceState *state) {
  for (int i = 0; i < NRF5_AUDIO_I2S_BUF_COUNT; i++) {
    if (state->i2s_bufs[i]) {
      kernel_free(state->i2s_bufs[i]);
      state->i2s_bufs[i] = NULL;
    }
  }
  if (state->circ_buffer_storage) {
    kernel_free(state->circ_buffer_storage);
    state->circ_buffer_storage = NULL;
  }
}

// Fill a stereo I2S buffer by draining mono samples from the circular buffer
// and duplicating each to both channels. Any shortfall is padded with silence.
// Runs in the nrfx_i2s ISR.
static void prv_fill_i2s_buffer(AudioDeviceState *state, int16_t *out) {
  const uint32_t n_mono = NRF5_AUDIO_I2S_BUF_SAMPLES_MONO;
  const uint32_t mono_bytes = n_mono * sizeof(int16_t);

  uint16_t available = circular_buffer_get_read_space_remaining(&state->circ_buffer);
  uint16_t to_copy = (available > mono_bytes) ? mono_bytes : available;
  uint32_t samples_copied = 0;

  if (to_copy > 0) {
    uint16_t copied = circular_buffer_copy(&state->circ_buffer, out, to_copy);
    circular_buffer_consume(&state->circ_buffer, copied);
    samples_copied = copied / sizeof(int16_t);
  }

  // Zero the unfilled tail of the mono portion.
  for (uint32_t i = samples_copied; i < n_mono; i++) {
    out[i] = 0;
  }

  // Expand mono -> stereo in place, iterating back-to-front so reads from
  // out[i] happen before writes at the paired stereo slots overwrite them.
  for (int32_t i = (int32_t)n_mono - 1; i >= 0; i--) {
    int16_t s = out[i];
    out[2 * i] = s;
    out[2 * i + 1] = s;
  }
}

// ---------------------------------------------------------------------------
// trans_cb dispatch (runs on the system task).
// ---------------------------------------------------------------------------

static void prv_audio_trans_bg(void *data) {
  AudioDeviceState *state = (AudioDeviceState *)data;
  state->callback_pending = false;
  if (state->is_running && state->trans_cb) {
    uint32_t free_size = circular_buffer_get_write_space_remaining(&state->circ_buffer);
    state->trans_cb(&free_size);
  }
}

static void prv_maybe_request_refill_from_isr(AudioDeviceState *state) {
  if (!state->trans_cb || state->callback_pending || !state->is_running) {
    return;
  }
  uint16_t free_size = circular_buffer_get_write_space_remaining(&state->circ_buffer);
  if (free_size < NRF5_AUDIO_REFILL_THRESHOLD_BYTES) {
    return;
  }

  // A dropped refill is retried on the next I2S buffer event; a momentary
  // underrun beats resetting the system over a full queue.
  state->callback_pending = true;
  bool should_context_switch = false;
  if (!system_task_add_callback_from_isr_droppable(prv_audio_trans_bg, state,
                                                   &should_context_switch)) {
    state->callback_pending = false;
  }
}

// ---------------------------------------------------------------------------
// nrfx_i2s data handler.
// ---------------------------------------------------------------------------

// nrfx_i2s_data_handler_t takes no user data, so we stash the currently active
// device here while it's running.
static AudioDevice *s_active_device;

static void prv_i2s_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status) {
  AudioDevice *dev = s_active_device;
  if (!dev) {
    return;
  }
  AudioDeviceState *state = dev->state;

  if (status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
    int16_t *fill_buf = state->i2s_bufs[state->buf_idx];
    prv_fill_i2s_buffer(state, fill_buf);

    nrfx_i2s_buffers_t next = {
        .p_tx_buffer = (uint32_t *)fill_buf,
        .p_rx_buffer = NULL,
        .buffer_size = I2S_BUF_SIZE_WORDS,
    };
    (void)nrfx_i2s_next_buffers_set(&dev->i2s_instance, &next);
    state->buf_idx = (state->buf_idx + 1) % NRF5_AUDIO_I2S_BUF_COUNT;

    prv_maybe_request_refill_from_isr(state);
  }
}

// ---------------------------------------------------------------------------
// Public API.
// ---------------------------------------------------------------------------

void audio_init(AudioDevice *audio_device) {
  PBL_ASSERTN(audio_device);
  AudioDeviceState *state = audio_device->state;

  // Skip the memset on a warm restart — the buffers and live ISR state are
  // still in use and would be leaked / clobbered.
  if (s_pwr_state == AudioPwrCold) {
    memset(state, 0, sizeof(*state));
    state->volume = 100;
  }
}

void audio_start(AudioDevice *audio_device, AudioTransCB cb) {
  PBL_ASSERTN(audio_device);
  AudioDeviceState *state = audio_device->state;

  pbl_mutex_lock(&s_audio_mutex, PBL_FOREVER);

  if (state->is_running) {
    pbl_mutex_unlock(&s_audio_mutex);
    PBL_LOG_WRN("Audio already running");
    return;
  }

  // Cancel any pending deferred shutdown so we keep the codec up.
  if (s_idle_timer != TIMER_INVALID_ID) {
    new_timer_stop(s_idle_timer);
  }

  if (s_pwr_state == AudioPwrWarm) {
    // Codec, DAI, and I2S have been running continuously, streaming zeros
    // from the ISR's auto-fill. Just resume writes and the next refill will
    // land real audio.
    state->trans_cb = cb;
    state->callback_pending = false;
    state->is_running = true;
    s_pwr_state = AudioPwrActive;
    pbl_mutex_unlock(&s_audio_mutex);
    PBL_LOG_DBG("Audio started (warm)");
    return;
  }

  if (!prv_allocate_buffers(state)) {
    pbl_mutex_unlock(&s_audio_mutex);
    return;
  }

  state->trans_cb = cb;
  state->callback_pending = false;
  state->buf_idx = 0;

  s_active_device = audio_device;

  if (audio_device->power_ops && audio_device->power_ops->power_up) {
    audio_device->power_ops->power_up();
  }

  // The DA7212 PLL references our MCK, which must be stable before we try to
  // lock it. HFXO is also needed by the I2S peripheral for MCK generation.
  clocksource_hfxo_request();

  nrfx_i2s_config_t cfg = NRFX_I2S_DEFAULT_CONFIG(
      audio_device->sck_pin, audio_device->lrck_pin, audio_device->mck_pin,
      audio_device->sdout_pin, audio_device->sdin_pin);
  cfg.irq_priority = audio_device->irq_priority;
  cfg.channels = NRF_I2S_CHANNELS_STEREO;
  cfg.sample_width = NRF_I2S_SWIDTH_16BIT;
  cfg.format = NRF_I2S_FORMAT_I2S;
  cfg.alignment = NRF_I2S_ALIGN_LEFT;
  // nRF is the I2S slave: SCK/LRCK come from the DA7212 (codec is master).
  // MCK is still generated by nRF at 4 MHz and fed into the codec PLL; the
  // codec divides it down to exactly 16 kHz WCLK internally, avoiding the
  // ~2% mismatch we'd get from nRF-as-master with any available MDIV/ratio.
  cfg.mode = NRF_I2S_MODE_SLAVE;
  cfg.mck_setup = NRF_I2S_MCK_32MDIV8;
  // Ratio is unused for LRCK in slave mode; pick a value that nrfx accepts
  // for 16-bit stereo (validate_config only enforces this in master mode).
  cfg.ratio = NRF_I2S_RATIO_256X;

  nrfx_err_t err = nrfx_i2s_init(&audio_device->i2s_instance, &cfg,
                                 prv_i2s_data_handler);
  PBL_ASSERT(err == NRFX_SUCCESS, "nrfx_i2s_init failed: %d", err);

  // Prime the first buffer with silence; real samples arrive via audio_write.
  memset(state->i2s_bufs[0], 0, I2S_BUF_SIZE_BYTES);
  nrfx_i2s_buffers_t initial = {
      .p_tx_buffer = (uint32_t *)state->i2s_bufs[0],
      .p_rx_buffer = NULL,
      .buffer_size = I2S_BUF_SIZE_WORDS,
  };
  state->buf_idx = 1;

  state->is_running = true;

  // Arm the nRF I2S in slave mode. This also starts MCK toggling on the mck
  // pin — required before the codec PLL can lock. The peripheral will sit
  // idle until the codec starts driving BCLK/WCLK in phase 2.
  err = nrfx_i2s_start(&audio_device->i2s_instance, &initial, 0);
  PBL_ASSERT(err == NRFX_SUCCESS, "nrfx_i2s_start failed: %d", err);

  // Codec phase 1: PLL lock against the now-running MCK + full path config.
  // DAI output stays disabled so P0.12/P0.07 remain high-Z (nRF has them
  // configured as inputs in slave mode).
  prv_codec_prepare(audio_device);

  // Codec phase 2: enable the DAI as master. Clocks start toggling now and
  // the nRF begins consuming the silence buffer.
  prv_codec_start_dai(audio_device);

  s_pwr_state = AudioPwrActive;
  pbl_mutex_unlock(&s_audio_mutex);

  PBL_LOG_DBG("Audio started");
}

uint32_t audio_write(AudioDevice *audio_device, void *write_buf, uint32_t size) {
  PBL_ASSERTN(audio_device);
  AudioDeviceState *state = audio_device->state;

  if (!state->is_running || !state->circ_buffer_storage) {
    return 0;
  }

  uint16_t free_size = circular_buffer_get_write_space_remaining(&state->circ_buffer);
  uint16_t to_write = (size > free_size) ? free_size : (uint16_t)size;
  if (to_write > 0) {
    (void)circular_buffer_write(&state->circ_buffer, write_buf, to_write);
  }
  return circular_buffer_get_write_space_remaining(&state->circ_buffer);
}

void audio_set_volume(AudioDevice *audio_device, int volume) {
  PBL_ASSERTN(audio_device);
  AudioDeviceState *state = audio_device->state;

  state->volume = (uint8_t)MIN(MAX(volume, 0), 100);

  if (s_pwr_state == AudioPwrCold) {
    // Codec is powered down (or audio_init hasn't run yet);
    // prv_codec_prepare() applies the cached value on the next start.
    return;
  }

  pbl_mutex_lock(&s_audio_mutex, PBL_FOREVER);
  if (s_pwr_state != AudioPwrCold) {
    prv_apply_volume(audio_device);
  }
  pbl_mutex_unlock(&s_audio_mutex);
}

void audio_stop(AudioDevice *audio_device) {
  PBL_ASSERTN(audio_device);
  AudioDeviceState *state = audio_device->state;

  pbl_mutex_lock(&s_audio_mutex, PBL_FOREVER);

  if (!state->is_running) {
    pbl_mutex_unlock(&s_audio_mutex);
    return;
  }

  // Warm pause: just stop feeding new audio. The ISR's prv_fill_i2s_buffer
  // sees an empty circular buffer and auto-zero-fills, so the codec keeps
  // receiving uninterrupted clocks and a continuous silent data stream — no
  // DAC mute, no I2S teardown, no LINE-output transitions. The actual
  // teardown happens later on the idle timer.
  state->is_running = false;
  state->trans_cb = NULL;
  s_pwr_state = AudioPwrWarm;

  if (s_idle_timer == TIMER_INVALID_ID) {
    s_idle_timer = new_timer_create();
  }
  if (s_idle_timer != TIMER_INVALID_ID) {
    new_timer_start(s_idle_timer, AUDIO_IDLE_SHUTDOWN_MS, prv_idle_shutdown,
                    (void *)(uintptr_t)audio_device, 0);
    pbl_mutex_unlock(&s_audio_mutex);
    PBL_LOG_DBG("Audio paused (warm)");
    return;
  }

  // Couldn't get a timer — fall through to immediate shutdown.
  prv_codec_mute(audio_device);

  nrfx_i2s_stop(&audio_device->i2s_instance);
  nrfx_i2s_uninit(&audio_device->i2s_instance);

  prv_codec_power_down(audio_device);

  clocksource_hfxo_release();

  if (audio_device->power_ops && audio_device->power_ops->power_down) {
    audio_device->power_ops->power_down();
  }

  s_active_device = NULL;
  prv_free_buffers(state);

  s_pwr_state = AudioPwrCold;
  pbl_mutex_unlock(&s_audio_mutex);

  PBL_LOG_DBG("Audio stopped");
}

static void prv_idle_shutdown(void *data) {
  AudioDevice *audio_device = (AudioDevice *)data;

  pbl_mutex_lock(&s_audio_mutex, PBL_FOREVER);

  if (s_pwr_state != AudioPwrWarm) {
    // audio_start beat us to the mutex, or we're already cold.
    pbl_mutex_unlock(&s_audio_mutex);
    return;
  }

  AudioDeviceState *state = audio_device->state;

  prv_codec_mute(audio_device);

  nrfx_i2s_stop(&audio_device->i2s_instance);
  nrfx_i2s_uninit(&audio_device->i2s_instance);

  prv_codec_power_down(audio_device);

  clocksource_hfxo_release();

  if (audio_device->power_ops && audio_device->power_ops->power_down) {
    audio_device->power_ops->power_down();
  }

  s_active_device = NULL;
  prv_free_buffers(state);

  s_pwr_state = AudioPwrCold;
  pbl_mutex_unlock(&s_audio_mutex);

  PBL_LOG_DBG("Audio fully stopped");
}
