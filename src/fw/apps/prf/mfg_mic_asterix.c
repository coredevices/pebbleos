/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mfg_sine_wave.h"

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/flash.h>
#include <pbl/drivers/clocksource.h>
#include "flash_region/flash_region.h"
#include "kernel/pbl_malloc.h"
#include "kernel/util/sleep.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"

#include <FreeRTOS.h>
#include <semphr.h>

#include "nrfx_i2s.h"
#include "nrfx_pdm.h"

#include "console/dbgserial.h"

#define DUMP_RECORDING_DBGSERIAL 0
#define PLAY_SINEWAVE 0

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
#define DA7212_TONE_GEN_CFG1         0xB4
#define DA7212_TONE_GEN_CYCLES       0xB6
#define DA7212_TONE_GEN_ON_PER       0xBB
#define DA7212_SYSTEM_ACTIVE         0xFD

#define RECORDING_MS      3000
#define SAMPLE_RATE_HZ    16000
#define SAMPLE_BITS       16
#define CAPTURE_MS        100
#define N_CHANNELS        2
#define N_SAMPLES         (N_CHANNELS * ((SAMPLE_RATE_HZ * CAPTURE_MS) / 1000))
#define SAMPLE_SIZE_BYTES (SAMPLE_BITS / 8)
#define BLOCK_SIZE        (N_SAMPLES * SAMPLE_SIZE_BYTES)

#define FLASH_START       FLASH_REGION_FIRMWARE_DEST_BEGIN
#define FLASH_END         FLASH_REGION_FIRMWARE_DEST_END

#if !PLAY_SINEWAVE
static int16_t s_buf[2][N_SAMPLES];
static int16_t *s_buf_rd;
static int16_t *s_buf_wr;
static uint8_t s_buf_idx;
static SemaphoreHandle_t s_data_ready;
static SemaphoreHandle_t s_need_data;
#endif
static nrfx_i2s_buffers_t s_i2s_bufs;

#if !PLAY_SINEWAVE
static const nrfx_pdm_t s_pdm = NRFX_PDM_INSTANCE(0);
static nrfx_pdm_config_t s_pdm_cfg =
  NRFX_PDM_DEFAULT_CONFIG(NRF_GPIO_PIN_MAP(1, 0), NRF_GPIO_PIN_MAP(0, 24));
#endif

static const nrfx_i2s_t s_i2s = NRFX_I2S_INSTANCE(0);
static nrfx_i2s_config_t s_i2s_cfg =
    NRFX_I2S_DEFAULT_CONFIG(NRF_GPIO_PIN_MAP(0, 12), NRF_GPIO_PIN_MAP(0, 7), NRF_GPIO_PIN_MAP(1, 9),
                            NRF_GPIO_PIN_MAP(0, 13), NRF_I2S_PIN_NOT_CONNECTED);

typedef struct {
  Window window;

  TextLayer title;
} AppData;

static void da7212_register_write(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  bool ret;

  i2c_use(I2C_DA7212);
  PBL_LOG_DBG("Writing DA7212 register 0x%02x with value 0x%02x", reg, value);

  ret = i2c_write_block(I2C_DA7212, 2, data);
  PBL_ASSERTN(ret);

  i2c_release(I2C_DA7212);
}

static uint8_t da7212_register_read(uint8_t reg) {
  uint8_t data;
  bool ret;

  i2c_use(I2C_DA7212);

  ret = i2c_read_register(I2C_DA7212, reg, &data);
  PBL_ASSERTN(ret);

  i2c_release(I2C_DA7212);

  return data;
}

static void prv_codec_setup(void) {
  // CIF_CTRL: soft reset
  da7212_register_write(DA7212_CIF_CTRL, 0x80);

  psleep(10);

  // SYSTEM_ACTIVE: wake-up
  da7212_register_write(DA7212_SYSTEM_ACTIVE, 0x01);

  // REFERENCES: enable master bias
  da7212_register_write(DA7212_REFERENCES, 0x08);

  psleep(30);

  // LDO_CTRL: enable LDO, 1.05V
  da7212_register_write(DA7212_LDO_CTRL, 0x80);

  // PLL: MCLK=4MHz (so input divider=2), we need 12.288MHz System Clock for SR=16KHz (see table 34)
  // VCO = System Clock * 8 = 98.304MHz
  // Feedback divider = VCO * Input Divider / MCLK
  //                  = 98.304MHz * 2 / 4MHz = 49.152
  // PLL_INTEGER = 49 (0x31)
  // PLL_FRAC = 0.152 * 2^13 = 1245 (0x4dd)
  // PLL_FRAC_TOP = 0x04
  // PLL_FRAC_BOT = 0xdd
  da7212_register_write(DA7212_PLL_FRAC_TOP, 0x04);
  da7212_register_write(DA7212_PLL_FRAC_BOT, 0xdd);
  da7212_register_write(DA7212_PLL_INTEGER, 0x31);

  // PLL_CTRL: enable + SRM, input clock range 2-10MHz
  da7212_register_write(DA7212_PLL_CTRL, 0xC0);

  // PLL: operate with a 2-5MHz MCLK (ref. DA7212 rev 3.6, 13.29)
  da7212_register_write(0xF0, 0x8B);
  da7212_register_write(0xF2, 0x03);
  da7212_register_write(0xF0, 0x00);

  psleep(40);

  PBL_ASSERT(da7212_register_read(DA7212_PLL_STATUS) == 0x07, "DA7212 PLL not locked");

  // GAIN_RAMP_CTRL: 1s
  da7212_register_write(DA7212_GAIN_RAMP_CTRL, 0x02);
  // SR: 16KHz
  da7212_register_write(DA7212_SR, 0x05);
  // DAI_CLK_MODE: slave
  da7212_register_write(DA7212_DAI_CLK_MODE, 0x00);
  // DAI_CTRL: enable, 16-bit
  da7212_register_write(DA7212_DAI_CTRL, 0x80);
  // DIG_ROUTING_DAI: DAI_R/L_SRC to DAI_R/L
  da7212_register_write(DA7212_DIG_ROUTING_DAI, 0x32);
  // DIG_ROUTING_DAC: DAC_R/L mono mix of R/L
  da7212_register_write(DA7212_DIG_ROUTING_DAC, 0xba);
  // DAC_R_GAIN: 0dB
  da7212_register_write(DA7212_DAC_R_GAIN, 0x6f);
  // DAC_R_CTRL: enable
  da7212_register_write(DA7212_DAC_R_CTRL, 0x80);
  // MIXOUT_R_SELECT: DAC_R
  da7212_register_write(DA7212_MIXOUT_R_SELECT, 0x08);
  // MIXOUT_R_CTRL: enable, softmix enable, amp enable
  da7212_register_write(DA7212_MIXOUT_R_CTRL, 0x98);
  // LINE_GAIN: 10dB
  da7212_register_write(DA7212_LINE_GAIN, 0x3a);
  // LINE_CTRL: enable
  da7212_register_write(DA7212_LINE_CTRL, 0x80);
}

static void prv_codec_standby(void) {
  da7212_register_write(DA7212_SYSTEM_ACTIVE, 0x00);
}

static void prv_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status) {
#if !PLAY_SINEWAVE
  PBL_ASSERT(!(status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED && p_released == NULL), "I2S buffers re-used");

  if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
    s_i2s_bufs.p_tx_buffer = (uint32_t *)s_buf[s_buf_idx];
    s_buf_idx = (s_buf_idx + 1) % 2;
    nrfx_i2s_next_buffers_set(&s_i2s, &s_i2s_bufs);
  }

  if (p_released != NULL && p_released->p_tx_buffer != NULL) {
    BaseType_t woken;
    s_buf_wr = (int16_t *)p_released->p_tx_buffer;
    xSemaphoreGiveFromISR(s_need_data, &woken);
    portYIELD_FROM_ISR(woken);
  }
#endif
}

#if !PLAY_SINEWAVE
static void prv_pdm_evt_handler(nrfx_pdm_evt_t const * p_evt) {
  PBL_ASSERT(p_evt->error == NRFX_PDM_NO_ERROR, "PDM overflow");

  if (p_evt->buffer_requested) {
    nrfx_pdm_buffer_set(&s_pdm, s_buf[s_buf_idx], N_SAMPLES);
    s_buf_idx = (s_buf_idx + 1) % 2;
  }

  if (p_evt->buffer_released) {
    BaseType_t woken;
    s_buf_rd = p_evt->buffer_released;
    xSemaphoreGiveFromISR(s_data_ready, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

static void prv_mic_capture(void) {
  uint32_t flash_addr;
  nrfx_err_t err;

  s_data_ready = xSemaphoreCreateBinary();

  clocksource_hfxo_request();

  s_pdm_cfg.mode = NRF_PDM_MODE_STEREO;
  // Sample rate of 16KHz (1280KHz / 80 = 16KHz)
  s_pdm_cfg.clock_freq = NRF_PDM_FREQ_1280K;
  s_pdm_cfg.ratio = NRF_PDM_RATIO_80X;
  s_pdm_cfg.gain_l = NRF_PDM_GAIN_MAXIMUM;
  s_pdm_cfg.gain_r = NRF_PDM_GAIN_MAXIMUM;

  err = nrfx_pdm_init(&s_pdm, &s_pdm_cfg, prv_pdm_evt_handler);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  flash_region_erase_optimal_range(FLASH_START, FLASH_START, FLASH_END, FLASH_END);

  err = nrfx_pdm_start(&s_pdm);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  flash_addr = FLASH_START;
  for (unsigned int i = 0U; i < RECORDING_MS / CAPTURE_MS; i++) {
    xSemaphoreTake(s_data_ready, portMAX_DELAY);

    flash_write_bytes((uint8_t *)s_buf_rd, flash_addr, BLOCK_SIZE);
    flash_addr += BLOCK_SIZE;
  }

  nrfx_pdm_stop(&s_pdm);
  nrfx_pdm_uninit(&s_pdm);

  clocksource_hfxo_release();

  vSemaphoreDelete(s_data_ready);

#if DUMP_RECORDING_DBGSERIAL
  flash_addr = FLASH_START;
  dbgserial_putstr("S");
  for (unsigned int i = 0U; i < RECORDING_MS / CAPTURE_MS; i++) {
    flash_read_bytes((uint8_t *)s_buf_rd, flash_addr, BLOCK_SIZE);
    flash_addr += BLOCK_SIZE;

    for (unsigned int j = 0U; j < N_SAMPLES; j++) {
      char buf[8];
      dbgserial_putstr_fmt(buf, sizeof(buf), "%d", s_buf_rd[j]);
    }
  }
  dbgserial_putstr("E");
#endif // DUMP_RECORDING_DBGSERIAL
}
#endif

#if !PLAY_SINEWAVE
static void prv_playback(void) {
  uint32_t flash_addr;
  nrfx_err_t err;

  s_need_data = xSemaphoreCreateBinary();

  clocksource_hfxo_request();

  // MCLK: 4MHz, sample rate: ~16KHz (4Mhz / 256 = 15625Hz)
  s_i2s_cfg.mck_setup = NRF_I2S_MCK_32MDIV8;
  s_i2s_cfg.ratio = NRF_I2S_RATIO_256X;
  s_i2s_cfg.channels = NRF_I2S_CHANNELS_STEREO;

  err = nrfx_i2s_init(&s_i2s, &s_i2s_cfg, prv_data_handler);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  flash_addr = FLASH_START;
  flash_read_bytes((uint8_t *)s_buf[0], flash_addr, BLOCK_SIZE);
  flash_addr += BLOCK_SIZE;

  s_buf_idx = 1U;
  s_buf_wr = s_buf[s_buf_idx];

  s_i2s_bufs.p_tx_buffer = (uint32_t *)s_buf[0];
  s_i2s_bufs.buffer_size = BLOCK_SIZE / 4;
  err = nrfx_i2s_start(&s_i2s, &s_i2s_bufs, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  prv_codec_setup();

  for (unsigned int i = 0U; i < (RECORDING_MS / CAPTURE_MS) - 1U; i++) {
    flash_read_bytes((uint8_t *)s_buf_wr, flash_addr, BLOCK_SIZE);
    flash_addr += BLOCK_SIZE;

    xSemaphoreTake(s_need_data, portMAX_DELAY);
  }

  prv_codec_standby();

  nrfx_i2s_stop(&s_i2s);
  nrfx_i2s_uninit(&s_i2s);

  clocksource_hfxo_release();

  vSemaphoreDelete(s_need_data);

  flash_region_erase_optimal_range(FLASH_START, FLASH_START, FLASH_END, FLASH_END);
}
#else
static void prv_playback(void) {
  nrfx_err_t err;

  clocksource_hfxo_request();

  // MCLK: 4MHz, sample rate: ~16KHz (4Mhz / 256 = 15625Hz)
  s_i2s_cfg.mck_setup = NRF_I2S_MCK_32MDIV8;
  s_i2s_cfg.ratio = NRF_I2S_RATIO_256X;
  s_i2s_cfg.channels = NRF_I2S_CHANNELS_STEREO;

  err = nrfx_i2s_init(&s_i2s, &s_i2s_cfg, prv_data_handler);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  s_i2s_bufs.p_tx_buffer = (uint32_t *)sine_wave;
  s_i2s_bufs.buffer_size = SINE_WAVE_TOTAL_SAMPLES / 2;
  err = nrfx_i2s_start(&s_i2s, &s_i2s_bufs, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  prv_codec_setup();
  psleep(3000);
  prv_codec_standby();

  nrfx_i2s_stop(&s_i2s);
  nrfx_i2s_uninit(&s_i2s);

  clocksource_hfxo_release();
}
#endif

static void prv_select_click_handler(ClickRecognizerRef recognizer, void *data) {
#if !PLAY_SINEWAVE
  prv_mic_capture();
#endif
  prv_playback();

  app_window_stack_pop(true);
}

static void prv_config_provider(void *data) {
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click_handler);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  window_set_click_config_provider(window, prv_config_provider);

  TextLayer *title = &data->title;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "MICROPHONE TEST");
  layer_add_child(&window->layer, &title->layer);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd *mfg_mic_asterix_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
      .common.main_func = &s_main,
      // UUID: 95ada1ce-04b3-46b0-8519-0b42260b5c39
      .common.uuid = {0x95, 0xad, 0xa1, 0xce, 0x04, 0xb3, 0x46, 0xb0,
                      0x85, 0x19, 0x0b, 0x42, 0x26, 0x0b, 0x5c, 0x39},
      .name = "MfgMicAsterix",
  };
  return (const PebbleProcessMd *)&s_app_info;
}
