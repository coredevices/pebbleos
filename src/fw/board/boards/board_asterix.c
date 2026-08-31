/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <nrfx_i2s.h>

#include "board/board.h"
#include <pbl/drivers/audio.h>
#include <pbl/drivers/flash/qspi_flash_definitions.h>
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/i2c/definitions.h>
#include <pbl/drivers/mic.h>
#include <pbl/drivers/mic/nrf5/pdm_definitions.h>
#include <pbl/drivers/speaker/nrf5/da7212_definitions.h>
#include <pbl/drivers/i2c/nrf5.h>
#include <pbl/drivers/uart/nrf5.h>
#include <pbl/drivers/pmic/npm1300.h>
#include <pbl/drivers/qspi_definitions.h>
#include <pbl/drivers/rtc.h>
#include "flash_region/flash_region.h"

// QSPI
#include <hal/nrf_clock.h>
#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <nrfx_twim.h>
#include <nrfx_pdm.h>

static QSPIPortState s_qspi_port_state;
static QSPIPort QSPI_PORT = {
    .state = &s_qspi_port_state,
    .clk_freq_hz = 8000000UL,
    .cs_gpio = NRF_GPIO_PIN_MAP(0, 17),
    .clk_gpio = NRF_GPIO_PIN_MAP(0, 19),
    .data_gpio =
        {
            NRF_GPIO_PIN_MAP(0, 20),
            NRF_GPIO_PIN_MAP(0, 21),
            NRF_GPIO_PIN_MAP(0, 22),
            NRF_GPIO_PIN_MAP(0, 23),
        },
};
QSPIPort *const QSPI = &QSPI_PORT;

static QSPIFlashState s_qspi_flash_state;
static QSPIFlash QSPI_FLASH_DEVICE = {
    .state = &s_qspi_flash_state,
    .qspi = &QSPI_PORT,
    .read_mode = QSPI_FLASH_READ_READ4IO,
    .write_mode = QSPI_FLASH_WRITE_PP4O,
};
QSPIFlash *const QSPI_FLASH = &QSPI_FLASH_DEVICE;
/* PERIPHERAL ID 43 */

static UARTDeviceState s_dbg_uart_state;
static UARTDevice DBG_UART_DEVICE = {
    .state = &s_dbg_uart_state,
    .tx_gpio = NRF_GPIO_PIN_MAP(0, 27),
    .rx_gpio = NRF_GPIO_PIN_MAP(0, 5),
    .rts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .cts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .periph = NRFX_UARTE_INSTANCE(0),
    .counter = NRFX_TIMER_INSTANCE(2),
};
UARTDevice *const DBG_UART = &DBG_UART_DEVICE;
IRQ_MAP_NRFX(UART0_UARTE0, nrfx_uarte_0_irq_handler);
/* PERIPHERAL ID 8 */

/* buttons */
IRQ_MAP_NRFX(TIMER1, nrfx_timer_1_irq_handler);
IRQ_MAP_NRFX(TIMER2, nrfx_timer_2_irq_handler);

/* display */
PwmState DISPLAY_EXTCOMIN_STATE;
IRQ_MAP_NRFX(SPIM3, nrfx_spim_3_irq_handler);

/* PERIPHERAL ID 10 */

/* EXTI */
IRQ_MAP_NRFX(GPIOTE, nrfx_gpiote_0_irq_handler);

/* nPM1300 */
static I2CBusState I2C_NPMC_IIC1_BUS_STATE = {};

static const I2CBusHal I2C_NPMC_IIC1_BUS_HAL = {
    .twim = NRFX_TWIM_INSTANCE(1),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_NPMC_IIC1_BUS = {
    .state = &I2C_NPMC_IIC1_BUS_STATE,
    .hal = &I2C_NPMC_IIC1_BUS_HAL,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 14),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 15),
        },
    .name = "I2C_NPMC_IIC1",
};
IRQ_MAP_NRFX(SPI1_SPIM1_SPIS1_TWI1_TWIM1_TWIS1, nrfx_twim_1_irq_handler);
/* PERIPHERAL ID 9 */

static const I2CSlavePort I2C_SLAVE_NPM1300 = {
    .bus = &I2C_NPMC_IIC1_BUS,
    .address = 0x6B << 1,
};

I2CSlavePort *const I2C_NPM1300 = &I2C_SLAVE_NPM1300;

/* peripheral I2C bus */
static I2CBusState I2C_IIC2_BUS_STATE = {};

static const I2CBusHal I2C_IIC2_BUS_HAL = {
    .twim = NRFX_TWIM_INSTANCE(0),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_IIC2_BUS = {
    .state = &I2C_IIC2_BUS_STATE,
    .hal = &I2C_IIC2_BUS_HAL,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 25),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 11),
        },
    .name = "I2C_IIC2",
};
IRQ_MAP_NRFX(SPI0_SPIM0_SPIS0_TWI0_TWIM0_TWIS0, nrfx_twim_0_irq_handler);

static const I2CSlavePort I2C_SLAVE_DRV2604 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x5A << 1,
};

I2CSlavePort *const I2C_DRV2604 = &I2C_SLAVE_DRV2604;

static const I2CSlavePort I2C_SLAVE_OPT3001 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x44 << 1,
};

I2CSlavePort *const I2C_OPT3001 = &I2C_SLAVE_OPT3001;

static const I2CSlavePort I2C_SLAVE_DA7212 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x1A << 1,
};

I2CSlavePort *const I2C_DA7212 = &I2C_SLAVE_DA7212;

static const I2CSlavePort I2C_SLAVE_MMC5603NJ = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x30 << 1,
};

I2CSlavePort *const I2C_MMC5603NJ = &I2C_SLAVE_MMC5603NJ;

static const I2CSlavePort I2C_SLAVE_BMP390 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x76 << 1,
};

I2CSlavePort *const I2C_BMP390 = &I2C_SLAVE_BMP390;

static LSM6DSOState s_lsm6dso_state;

static const LSM6DSOConfig s_lsm6dso_config = {
    .state = &s_lsm6dso_state,
    .i2c = {
        .bus = &I2C_IIC2_BUS,
        .address = 0x6A << 1,
    },
    .int1 = {
        .peripheral = NRFX_GPIOTE_INSTANCE(0),
        .channel = 7,
        .gpio_pin = NRF_GPIO_PIN_MAP(1, 13),
    },
    .int1_in = {
        .gpio = NRF5_GPIO_RESOURCE_EXISTS,
        .gpio_pin = NRF_GPIO_PIN_MAP(1, 13),
    },
    .axis_map = {
        [AXIS_X] = 1,
        [AXIS_Y] = 0,
        [AXIS_Z] = 2,
    },
    .axis_dir = {
        [AXIS_X] = 1,
        [AXIS_Y] = 1,
        [AXIS_Z] = 1,
    },
};

const LSM6DSOConfig *const LSM6DSO = &s_lsm6dso_config;

IRQ_MAP_NRFX(I2S, nrfx_i2s_0_irq_handler);

IRQ_MAP_NRFX(PDM, NRFX_PDM_INST_HANDLER_GET(0));

/* PERIPHERAL ID 11 */

/* Microphone */
static MicDeviceState s_mic_state_storage;
static MicDevice s_mic_device = {
  .state = &s_mic_state_storage,
  .pdm_instance = NRFX_PDM_INSTANCE(0),
  .clk_pin = NRF_GPIO_PIN_MAP(1, 0),   // P1.00 - PDM CLK
  .data_pin = NRF_GPIO_PIN_MAP(0, 24), // P0.24 - PDM DATA
  .channels = 1,
};
MicDevice * const MIC = &s_mic_device;

/* Speaker / audio output (DA7212 codec over I2S) */
static AudioDeviceState s_audio_state_storage;
static void prv_audio_power_up(void) {
  NPM1300_OPS.dischg_limit_ma_set(NPM1300_DISCHG_LIMIT_MA_MAX);
}
static void prv_audio_power_down(void) {
  NPM1300_OPS.dischg_limit_ma_set(NPM1300_CONFIG.dischg_limit_ma);
}
static const BoardPowerOps s_audio_power_ops = {
  .power_up = prv_audio_power_up,
  .power_down = prv_audio_power_down,
};
static const AudioDevice s_audio_device = {
  .state = &s_audio_state_storage,
  .i2s_instance = NRFX_I2S_INSTANCE(0),
  .sck_pin = NRF_GPIO_PIN_MAP(0, 12),   // P0.12 - I2S SCK  -> DA7212 BCLK
  .lrck_pin = NRF_GPIO_PIN_MAP(0, 7),   // P0.07 - I2S LRCK -> DA7212 WCLK
  .mck_pin = NRF_GPIO_PIN_MAP(1, 9),    // P1.09 - I2S MCK  -> DA7212 MCLK
  .sdout_pin = NRF_GPIO_PIN_MAP(0, 13), // P0.13 - I2S SDOUT -> DA7212 DATA_IN
  .sdin_pin = NRF_I2S_PIN_NOT_CONNECTED, // codec DATA_OUT unused for playback
  .irq_priority = 5,
  .codec = &I2C_SLAVE_DA7212,
  .power_ops = &s_audio_power_ops,
  .samplerate = 16000,
};
AudioDevice * const AUDIO = (AudioDevice *)&s_audio_device;

/* sensor SPI bus */

/* asterix shares SPI with flash, which we don't support */

PwmState BACKLIGHT_PWM_STATE;
IRQ_MAP_NRFX(PWM0, nrfx_pwm_0_irq_handler);

IRQ_MAP_NRFX(RTC1, rtc_irq_handler);

const Npm1300Config NPM1300_CONFIG = {
  // 128mA = ~1C (rapid charge)
  .chg_current_ma = 128,
  .dischg_limit_ma = 200,
  .term_current_pct = 10,
  .thermistor_beta = 3380,
  .ntc_hot_celsius = 45,
};

void board_early_init(void) {
  PBL_LOG_ERR("asterix early init");

  NRF_NVMC->ICACHECNF |= NVMC_ICACHECNF_CACHEEN_Msk;

  nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL);
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
  /* TODO: Add timeout, report failure if LFCLK does not start. For now,
   * WDT should trigger a reboot. Calibrated RC may be used as a fallback,
   * provided we can adjust BLE SCA settings at runtime.
   */
  while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
  }
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
}

void board_init(void) {
  i2c_init(&I2C_NPMC_IIC1_BUS);
  i2c_init(&I2C_IIC2_BUS);

  uint8_t da7212_powerdown[] = { 0xFD /* SYSTEM_ACTIVE */, 0 };
  i2c_use(I2C_DA7212);
  i2c_write_block(I2C_DA7212, 2, da7212_powerdown);
  i2c_release(I2C_DA7212);
}
