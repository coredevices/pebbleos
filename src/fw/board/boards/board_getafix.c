/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include "board/splash.h"
#include <pbl/drivers/sf32lb52/debounced_button_definitions.h>
#include "system/passert.h"

static UARTDeviceState s_dbg_uart_state = {
  .huart = {
    .Instance = USART1,
    .Init = {
      .BaudRate = 1000000,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
    },
  },
  .hdma = {
    .Instance = DMA1_Channel1,
    .Init = {
      .Request = DMA_REQUEST_5,
      .IrqPrio = 5,
    },
  },
};

static UARTDevice DBG_UART_DEVICE = {
    .state = &s_dbg_uart_state,
    .tx = {
        .pad = PAD_PA19,
        .func = USART1_TXD,
        .flags = PIN_NOPULL,
    },
    .rx = {
        .pad = PAD_PA18,
        .func = USART1_RXD,
        .flags = PIN_PULLUP,
    },
    .irqn = USART1_IRQn,
    .irq_priority = 5,
    .dma_irqn = DMAC1_CH1_IRQn,
    .dma_irq_priority = 5,
};

UARTDevice *const DBG_UART = &DBG_UART_DEVICE;

IRQ_MAP(USART1, uart_irq_handler, DBG_UART);
IRQ_MAP(DMAC1_CH1, uart_dma_irq_handler, DBG_UART);

#ifdef NIMBLE_HCI_SF32LB52_TRACE_BINARY
static UARTDeviceState s_hci_trace_uart_state = {
  .huart = {
    .Instance = USART3,
    .Init = {
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
    },
  },
};

static UARTDevice HCI_TRACE_UART_DEVICE = {
    .state = &s_hci_trace_uart_state,
    .tx = {
        .pad = PAD_PA27,
        .func = USART3_TXD,
        .flags = PIN_NOPULL,
    },
};
UARTDevice *const HCI_TRACE_UART = &HCI_TRACE_UART_DEVICE;
#endif // NIMBLE_HCI_SF32LB52_TRACE_BINARY

static QSPIPortState s_qspi_port_state = {
    .cfg = {
      .Instance = FLASH2,
      .line = HAL_FLASH_QMODE,
      .base = FLASH2_BASE_ADDR,
      .msize = 32,
      .SpiMode = SPI_MODE_NOR,
    },
    .dma = {
      .Instance = DMA1_Channel2,
      .dma_irq = DMAC1_CH2_IRQn,
      .request = DMA_REQUEST_1,
    },
    .t_enter_deep_us = 3,
    .t_exit_deep_us = 20,
};

static QSPIPort QSPI_PORT = {
    .state = &s_qspi_port_state,
    .clk_div = 0U,
};
QSPIPort *const QSPI = &QSPI_PORT;

static QSPIFlashState s_qspi_flash_state;
static QSPIFlash QSPI_FLASH_DEVICE = {
    .state = &s_qspi_flash_state,
    .qspi = &QSPI_PORT,
};
QSPIFlash *const QSPI_FLASH = &QSPI_FLASH_DEVICE;

static DisplayJDIState s_display_state = {
    .hlcdc = {
        .Instance = LCDC1,
        .Init = {
            .lcd_itf = LCDC_INTF_JDI_PARALLEL,
            .color_mode = LCDC_PIXEL_FORMAT_RGB332,
            .freq = 746268,  // HCK frequency
            .cfg = {
              .jdi = {
                .bank_col_head = 0,
                .valid_columns = PBL_DISPLAY_WIDTH,
                .bank_col_tail = 8,
                .bank_row_head = 0,
                .valid_rows = PBL_DISPLAY_HEIGHT,
                .bank_row_tail = 4,
                .enb_start_col = 6,
                .enb_end_col = 128,
            },
          },
        },
    },
};

static DisplayJDIDevice s_display = {
    .state = &s_display_state,
    .irqn = LCDC1_IRQn,
    .irq_priority = 5,
    .vcom = {
        .lptim = hwp_lptim2,
        .freq_hz = 60U,
    },
    .pinmux = {
        .xrst = {
            .pad = PAD_PA40,
            .func = LCDC1_JDI_XRST,
            .flags = PIN_NOPULL,
            },
        .vst = {
            .pad = PAD_PA08,
            .func = LCDC1_JDI_VST,
            .flags = PIN_NOPULL,
            },
        .vck = {
            .pad = PAD_PA39,
            .func = LCDC1_JDI_VCK,
            .flags = PIN_NOPULL,
            },
        .enb = {
            .pad = PAD_PA07,
            .func = LCDC1_JDI_ENB,
            .flags = PIN_NOPULL,
            },
        .hst = {
            .pad = PAD_PA06,
            .func = LCDC1_JDI_HST,
            .flags = PIN_NOPULL,
            },
        .hck = {
            .pad = PAD_PA41,
            .func = LCDC1_JDI_HCK,
            .flags = PIN_NOPULL,
            },
        .r1 = {
            .pad = PAD_PA05,
            .func = LCDC1_JDI_R1,
            .flags = PIN_NOPULL,
            },
        .r2 = {
            .pad = PAD_PA42,
            .func = LCDC1_JDI_R2,
            .flags = PIN_NOPULL,
            },
        .g1 = {
            .pad = PAD_PA04,
            .func = LCDC1_JDI_G1,
            .flags = PIN_NOPULL,
            },
        .g2 = {
            .pad = PAD_PA43,
            .func = LCDC1_JDI_G2,
            .flags = PIN_NOPULL,
            },
        .b1 = {
            .pad = PAD_PA03,
            .func = LCDC1_JDI_B1,
            .flags = PIN_NOPULL,
            },
        .b2 = {
            .pad = PAD_PA02,
            .func = LCDC1_JDI_B2,
            .flags = PIN_NOPULL,
            },
        .vcom_frp = {
            .pad = PAD_PA24,
            .func = PBR_LPTIM2_OUT,
            .flags = PIN_NOPULL,
        },
        .xfrp = {
            .pad = PAD_PA25,
            .func = PBR_LPTIM2_INV_OUT,
            .flags = PIN_NOPULL,
        },
    },
    .vddp = {hwp_gpio1, 0, true},
    .vlcd = {hwp_gpio1, 9, true},
    .splash = {
        .data = splash_bits,
        .width = splash_width,
        .height = splash_height,
    },
};

DisplayJDIDevice *const DISPLAY = &s_display;
IRQ_MAP(LCDC1, display_jdi_irq_handler, DISPLAY);

const LedControllerAW9364E AW9364E = {
    .gpio = {
        .gpio = hwp_gpio1,
        .gpio_pin = 1,
        .active_high = true,
    },
};

static I2CBusHalState s_i2c_bus_hal_state_1 = {
    .hdl = {
        .Instance = I2C1,
        .Init = {
            .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
            .ClockSpeed = 400000,
            .GeneralCallMode = I2C_GENERALCALL_DISABLE,
        },
        .Mode = HAL_I2C_MODE_MASTER,
        .core = CORE_ID_HCPU,
    },
};

static struct I2CBusHal s_i2c_bus_hal_1 = {
    .state = &s_i2c_bus_hal_state_1,
    .scl =
        {
            .pad = PAD_PA32,
            .func = I2C1_SCL,
            .flags = PIN_NOPULL,
        },
    .sda =
        {
            .pad = PAD_PA33,
            .func = I2C1_SDA,
            .flags = PIN_NOPULL,
        },
    .module = RCC_MOD_I2C1,
    .irqn = I2C1_IRQn,
    .irq_priority = 5,
};

static I2CBusState s_i2c_bus_state_1;

static I2CBus s_i2c_bus_1 = {
    .hal = &s_i2c_bus_hal_1,
    .name = "i2c1",
    .state = &s_i2c_bus_state_1,
};

I2CBus *const I2C1_BUS = &s_i2c_bus_1;

IRQ_MAP(I2C1, i2c_irq_handler, I2C1_BUS);

static LIS2DW12State s_lis2dw12_state;

static const LIS2DW12Config s_lis2dw12_config = {
    .state = &s_lis2dw12_state,
    .i2c = {
        .bus = &s_i2c_bus_1,
        .address = 0x19,
    },
    .int1 = {
      .peripheral = hwp_gpio1,
      .gpio_pin = 26,
    },
    .int1_in = {
      .gpio = hwp_gpio1,
      .gpio_pin = 26,
    },
    .axis_map = {
        [AXIS_X] = 0,
        [AXIS_Y] = 1,
        [AXIS_Z] = 2,
    },
    .axis_dir = {
        [AXIS_X] = -1,
        [AXIS_Y] = 1,
        [AXIS_Z] = 1,
    },
};

const LIS2DW12Config *const LIS2DW12 = &s_lis2dw12_config;

static const I2CSlavePort s_i2c_mmc5603nj = {
    .bus = &s_i2c_bus_1,
    .address = 0x30,
};

I2CSlavePort *const I2C_MMC5603NJ = &s_i2c_mmc5603nj;

static I2CBusHalState s_i2c_bus_hal_state_2 = {
    .hdl = {
        .Instance = I2C2,
        .Init = {
            .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
            .ClockSpeed = 400000,
            .GeneralCallMode = I2C_GENERALCALL_DISABLE,
        },
        .Mode = HAL_I2C_MODE_MASTER,
        .core = CORE_ID_HCPU,
    },
};

static struct I2CBusHal s_i2c_bus_hal_2 = {
    .state = &s_i2c_bus_hal_state_2,
    .scl =
        {
            .pad = PAD_PA11,
            .func = I2C2_SCL,
            .flags = PIN_NOPULL,
        },
    .sda =
        {
            .pad = PAD_PA10,
            .func = I2C2_SDA,
            .flags = PIN_NOPULL,
        },
    .module = RCC_MOD_I2C2,
    .irqn = I2C2_IRQn,
    .irq_priority = 5,
};

static I2CBusState s_i2c_bus_state_2;

static I2CBus s_i2c_bus_2 = {
    .hal = &s_i2c_bus_hal_2,
    .name = "i2c2",
    .state = &s_i2c_bus_state_2,
};

I2CBus *const I2C2_BUS = &s_i2c_bus_2;
IRQ_MAP(I2C2, i2c_irq_handler, I2C2_BUS);

static const I2CSlavePort s_i2c_cst816 = {
    .bus = &s_i2c_bus_2,
    .address = 0x15,
};

static const I2CSlavePort s_i2c_cst816_boot = {
    .bus = &s_i2c_bus_2,
    .address = 0x6A,
};

static const TouchSensor s_touch_cst816 = {
    .i2c = &s_i2c_cst816,
    .i2c_boot = &s_i2c_cst816_boot,
    .int_exti = {
        .peripheral = hwp_gpio1,
        .gpio_pin = 38,
        .pull = GPIO_PuPd_UP,
    },
    .reset = {
        .gpio = hwp_gpio1,
        .gpio_pin = 28,
        .active_high = false,
    },
};

const TouchSensor *CST816 = &s_touch_cst816;

static I2CBusHalState s_i2c_bus_hal_state_3 = {
    .hdl = {
        .Instance = I2C3,
        .Init = {
            .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
            .ClockSpeed = 400000,
            .GeneralCallMode = I2C_GENERALCALL_DISABLE,
        },
        .Mode = HAL_I2C_MODE_MASTER,
        .core = CORE_ID_HCPU,
    },
};

static struct I2CBusHal s_i2c_bus_hal_3 = {
    .state = &s_i2c_bus_hal_state_3,
    .scl =
        {
            .pad = PAD_PA31,
            .func = I2C3_SCL,
            .flags = PIN_NOPULL,
        },
    .sda =
        {
            .pad = PAD_PA30,
            .func = I2C3_SDA,
            .flags = PIN_NOPULL,
        },
    .module = RCC_MOD_I2C3,
    .irqn = I2C3_IRQn,
    .irq_priority = 5,
};

static I2CBusState s_i2c_bus_state_3;

static I2CBus s_i2c_bus_3 = {
    .hal = &s_i2c_bus_hal_3,
    .name = "i2c3",
    .state = &s_i2c_bus_state_3,
};

I2CBus *const I2C3_BUS = &s_i2c_bus_3;
IRQ_MAP(I2C3, i2c_irq_handler, I2C3_BUS);

static const I2CSlavePort s_i2c_npm1300 = {
    .bus = &s_i2c_bus_3,
    .address = 0x6B,
};

I2CSlavePort *const I2C_NPM1300 = &s_i2c_npm1300;

static const I2CSlavePort s_i2c_w1160 = {
    .bus = &s_i2c_bus_3,
    .address = 0x48,
};

I2CSlavePort *const I2C_W1160 = &s_i2c_w1160;

#ifdef CONFIG_BOARD_GETAFIX_DVT2
static const I2CSlavePort s_i2c_aw86225 = {
    .bus = &s_i2c_bus_3,
    .address = 0x58,
};
  
I2CSlavePort *const I2C_AW86225 = &s_i2c_aw86225;

static const AW86225Config s_aw86225_config = {
    .lra_frequency_hz = 240,
    .lra_frequency_tolerance_hz = 20,
};

const AW86225Config *const AW86225 = &s_aw86225_config;
#else
static const I2CSlavePort s_i2c_aw8623x = {
    .bus = &s_i2c_bus_3,
    .address = 0x5a,
};

I2CSlavePort *const I2C_AW8623X = &s_i2c_aw8623x;
#endif

const BoardConfigActuator BOARD_CONFIG_VIBE = {
    .ctl = {hwp_gpio1, 20, false},
};

const Npm1300Config NPM1300_CONFIG = {
  // 70mA = 1C (max limit from datasheet)
  .chg_current_ma = 70,
  .dischg_limit_ma = 200,
  .term_current_pct = 10,
  .thermistor_beta = 3380,
  .ntc_hot_celsius = 45,
  .vbus_current_lim0 = 500,
  .vbus_current_startup = 500,
};

const BoardConfigPower BOARD_CONFIG_POWER = {
  .pmic_int = {
    .peripheral = hwp_gpio1,
    .gpio_pin = 44,
  },
  .low_power_threshold = 5U,
  .battery_capacity_hours = 150U,
};

const BoardConfig BOARD_CONFIG = {
  .backlight_on_percent = 25,
  .ambient_light_dark_threshold = 800,
  .ambient_k_delta_threshold = 100,
  // Bench-calibrated on 1 DVT2 unit, fit above 4500 lux; readings below
  // ~4000 lux deviated from the fit on that unit and need a re-measure.
  .ambient_light_lux_dark_offset = 0,
  .ambient_light_lux_num = 50,
  .ambient_light_lux_den = 517,
};

const BoardConfigButton BOARD_CONFIG_BUTTON = {
  .buttons = {
    [BUTTON_ID_BACK]   = { "Back",   hwp_gpio1, 34, GPIO_PuPd_NOPULL, true },
    [BUTTON_ID_UP]     = { "Up",     hwp_gpio1, 35, GPIO_PuPd_UP, false},
    [BUTTON_ID_SELECT] = { "Select", hwp_gpio1, 36, GPIO_PuPd_UP, false},
    [BUTTON_ID_DOWN]   = { "Down",   hwp_gpio1, 37, GPIO_PuPd_UP, false},
  },
  .timer = GPTIM2,
  .timer_irqn = GPTIM2_IRQn,
};
IRQ_MAP(GPTIM2, debounced_button_irq_handler, GPTIM2);

static MicDeviceState mic_state = {
  .hdma = {
  .Instance = DMA1_Channel5,
    .Init = {
       .Request = DMA_REQUEST_36,
       .IrqPrio = 5,
    },
  },
};
static const MicDevice mic_device = {
    .state = &mic_state,
    .pdm_instance = hwp_pdm1,
    .clk_gpio = {
        .pad = PAD_PA22,
        .func = PDM1_CLK,
        .flags = PIN_NOPULL, 
    },
    .data_gpio = {
        .pad = PAD_PA23,
        .func = PDM1_DATA,
        .flags = PIN_PULLDOWN,
    },
    .pdm_dma_irq = DMAC1_CH5_IRQn,
    .pdm_irq = PDM1_IRQn,
    .pdm_irq_priority = 5,
#ifdef CONFIG_MFG
    // MFG mic test needs stereo capture to verify both microphones
    .channels = 2,
#else
    .channels = 1,
#endif
    .sample_rate = 16000,
    .channel_depth = 16,
};
const MicDevice* MIC = &mic_device;
IRQ_MAP(PDM1, pdm1_data_handler, MIC);
IRQ_MAP(DMAC1_CH5, pdm1_l_dma_handler, MIC);

uint32_t BSP_GetOtpBase(void) {
  return MPI2_MEM_BASE;
}

void board_early_init(void) {

}

void board_init(void) {
  i2c_init(I2C1_BUS);
  i2c_init(I2C2_BUS);
  i2c_init(I2C3_BUS);

  mic_init(MIC);
}
