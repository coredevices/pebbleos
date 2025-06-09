/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "board/board.h"
#include "system/passert.h"

#include "bf0_hal.h"
#include "bf0_hal_efuse.h"
#include "bf0_hal_lcpu_config.h"
#include "bf0_hal_rcc.h"
#include "bf0_hal_pmu.h"

#define HCPU_FREQ_MHZ 240

#define USING_UART1_CONSOLE
#ifdef USING_UART1_CONSOLE
#define UART_INST USART1
#define UART_TX PAD_PA19
#define UART_TX_FUNC USART1_TXD
#define UART_RX PAD_PA18
#define UART_RX_FUNC USART1_RXD
#define UART_DMAREQ DMA_REQUEST_5
#define UART_IRQ_NUM USART1_IRQn
#else
#define UART_INST USART3
#define UART_TX PAD_PA20
#define UART_TX_FUNC USART3_TXD
#define UART_RX PAD_PA27
#define UART_RX_FUNC USART3_RXD
#define UART_DMAREQ DMA_REQUEST_27
#define UART_IRQ_NUM USART3_IRQn
#endif

static UARTDeviceState s_dbg_uart_state = {
  .huart = {
    .Instance = UART_INST,
    .Init = {
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
      .Request = UART_DMAREQ,
      .IrqPrio = 5,
    },
  },
};

static UARTDevice DBG_UART_DEVICE = {
    .state = &s_dbg_uart_state,
    .tx = {
        .pad = UART_TX,
        .func = UART_TX_FUNC,
        .flags = PIN_NOPULL,
    },
    .rx = {
        .pad = UART_RX,
        .func = UART_RX_FUNC,
        .flags = PIN_PULLUP,
    },
    .irqn = UART_IRQ_NUM,
    .irq_priority = 5,
    .dma_irqn = DMAC1_CH1_IRQn,
    .dma_irq_priority = 5,
};

UARTDevice *const DBG_UART = &DBG_UART_DEVICE;

#ifdef USING_UART1_CONSOLE
IRQ_MAP(USART1, uart_irq_handler, DBG_UART);
#else
IRQ_MAP(USART3, uart_irq_handler, DBG_UART);
#endif

IRQ_MAP(DMAC1_CH1, uart_dma_irq_handler, DBG_UART);


// Raw UART --------------------------------------------------------------
#undef UART_INST 
#undef UART_TX 
#undef UART_TX_FUNC
#undef UART_RX 
#undef UART_RX_FUNC
#undef UART_DMAREQ 
#undef UART_IRQ_NUM

#ifndef USING_UART1_CONSOLE
#define UART_INST USART1
#define UART_TX PAD_PA19
#define UART_TX_FUNC USART1_TXD
#define UART_RX PAD_PA18
#define UART_RX_FUNC USART1_RXD
#define UART_DMAREQ DMA_REQUEST_5
#define UART_IRQ_NUM USART1_IRQn
#else
#define UART_INST USART3
#define UART_TX PAD_PA20
#define UART_TX_FUNC USART3_TXD
#define UART_RX PAD_PA27
#define UART_RX_FUNC USART3_RXD
#define UART_DMAREQ DMA_REQUEST_27
#define UART_IRQ_NUM USART3_IRQn
#endif

static UARTDeviceState s_raw_uart_state = {
  .huart = {
    .Instance = UART_INST,
    .Init = {
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
      .Request = UART_DMAREQ,
      .IrqPrio = 5,
    },
  },
};

static UARTDevice RAW_UART_DEVICE = {
    .state = &s_raw_uart_state,
    .tx = {
        .pad = UART_TX,
        .func = UART_TX_FUNC,
        .flags = PIN_NOPULL,
    },
    .rx = {
        .pad = UART_RX,
        .func = UART_RX_FUNC,
        .flags = PIN_PULLUP,
    },
    .irqn = UART_IRQ_NUM,
    .irq_priority = 5,
    .dma_irqn = DMAC1_CH1_IRQn,
    .dma_irq_priority = 5,
};
UARTDevice *const RAW_UART = &RAW_UART_DEVICE;

#ifndef USING_UART1_CONSOLE
IRQ_MAP(USART1, uart_irq_handler, RAW_UART);
#else
IRQ_MAP(USART3, uart_irq_handler, RAW_UART);
#endif

/**********************************************************************************************/


static QSPIPortState s_qspi_port_state;
static QSPIPort QSPI_PORT = {
    .state = &s_qspi_port_state,
    .cfg = {
      .Instance = FLASH2,
      .line = HAL_FLASH_QMODE,
      .base = FLASH2_BASE_ADDR,
      .msize = 16,
      .SpiMode = SPI_MODE_NOR,
    },
    .clk_div = 5U,
    .dma = {
      .Instance = DMA1_Channel2,
      .dma_irq = DMAC1_CH2_IRQn,
      .request = DMA_REQUEST_1,
    },
};
QSPIPort *const QSPI = &QSPI_PORT;

static QSPIFlashState s_qspi_flash_state;
static QSPIFlash QSPI_FLASH_DEVICE = {
    .state = &s_qspi_flash_state,
    .qspi = &QSPI_PORT,
};
QSPIFlash *const QSPI_FLASH = &QSPI_FLASH_DEVICE;

const BoardConfigPower BOARD_CONFIG_POWER = {
  .low_power_threshold = 5U,
  .battery_capacity_hours = 100U,
};

const BoardConfig BOARD_CONFIG = {
  .backlight_on_percent = 100U,
};

uint32_t BSP_GetOtpBase(void) {
  return MPI2_MEM_BASE;
}

void board_early_init(void) {
  HAL_StatusTypeDef ret;

  if (HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS) == RCC_SYSCLK_HRC48) {
    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
  }

  HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

  // Halt LCPU first to avoid LCPU in running state
  HAL_HPAON_WakeCore(CORE_ID_LCPU);
  HAL_RCC_Reset_and_Halt_LCPU(1);

  // Load system configuration from EFUSE
  BSP_System_Config();

  HAL_HPAON_StartGTimer();
  HAL_PMU_EnableRC32K(1);
  HAL_PMU_LpCLockSelect(PMU_LPCLK_RC32);

  HAL_PMU_EnableDLL(1);

  HAL_PMU_EnableXTAL32();
  ret = HAL_PMU_LXTReady();
  PBL_ASSERTN(ret == HAL_OK);

  HAL_RTC_ENABLE_LXT();

  HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HXT48);

  HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();

  HAL_RCC_HCPU_ConfigHCLK(HCPU_FREQ_MHZ);

  // Reset sysclk used by HAL_Delay_us
  HAL_Delay_us(0);

  ret = HAL_RCC_CalibrateRC48();
  PBL_ASSERTN(ret == HAL_OK);

  HAL_RCC_Init();
  HAL_PMU_Init();

  __HAL_SYSCFG_CLEAR_SECURITY();
  HAL_EFUSE_Init();
}

void board_init(void) {}
