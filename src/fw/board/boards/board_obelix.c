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
#include "drivers/sf32lb/uart_definitions.h"

#define HCPU_FREQ_MHZ 240

#define USING_UART1

#ifdef USING_UART1
#define UART_INST USART1
#define UART_TX PAD_PA19
#define UART_RX PAD_PA18
#define UART_DMAREQ DMA_REQUEST_5
#else
#define UART_INST USART3
#define UART_TX PAD_PA20
#define UART_RX PAD_PA27
#define UART_DMAREQ DMA_REQUEST_27
#endif

static UARTDeviceState s_dbg_uart_state;

static UART_HandleTypeDef s_dbg_uart_handle = {
    .Instance = UART_INST,
    .Init =
        {
            .WordLength = UART_WORDLENGTH_8B,
            .StopBits = UART_STOPBITS_1,
            .Parity = UART_PARITY_NONE,
            .HwFlowCtl = UART_HWCONTROL_NONE,
            .OverSampling = UART_OVERSAMPLING_16,
        },
};

static DMA_HandleTypeDef s_dbg_uart_rx_dma_handle = {
    .Instance = DMA1_Channel1,
    .Init.Request = UART_DMAREQ,
};

static UARTDevice DBG_UART_DEVICE = {
    .state = &s_dbg_uart_state,
    .tx_gpio = UART_TX,
    .rx_gpio = UART_RX,
    .periph = &s_dbg_uart_handle,
    .rx_dma = &s_dbg_uart_rx_dma_handle,
    .irq_priority = 5,
};

UARTDevice *const DBG_UART = &DBG_UART_DEVICE;

#ifdef USING_UART1
IRQ_MAP(USART1, uart_irq_handler, DBG_UART);
#else
IRQ_MAP(USART3, uart_irq_handler, DBG_UART);
#endif

void DMAC1_CH1_IRQHandler(void) { HAL_DMA_IRQHandler(&s_dbg_uart_rx_dma_handle); }

const BoardConfigPower BOARD_CONFIG_POWER = {
  .low_power_threshold = 5U,
  .battery_capacity_hours = 100U,
};

const BoardConfig BOARD_CONFIG = {
  .backlight_on_percent = 100U,
};

void HAL_PreInit(void) {
  /* not switch back to XT48 if other clock source has been selected already */
  if (RCC_SYSCLK_HRC48 == HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS)) {
    // To avoid somebody cancel request.
    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
  }

  HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

  if (PM_STANDBY_BOOT != SystemPowerOnModeGet()) {
    // Halt LCPU first to avoid LCPU in running state
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_RCC_Reset_and_Halt_LCPU(1);
    // get system configure from EFUSE
    BSP_System_Config();
    HAL_HPAON_StartGTimer();
    HAL_PMU_EnableRC32K(1);
    HAL_PMU_LpCLockSelect(PMU_LPCLK_RC32);

    HAL_PMU_EnableDLL(1);

    HAL_PMU_EnableXTAL32();
    if (HAL_PMU_LXTReady() != HAL_OK) HAL_ASSERT(0);
    // RTC/GTIME/LPTIME Using same low power clock source
    HAL_RTC_ENABLE_LXT();

    {
      uint8_t is_enable_lxt = 1;
      uint32_t wdt_staus = 0xFF;
      uint32_t wdt_time = 0;
      uint16_t wdt_clk = 32768;
      uint8_t is_lcpu_rccal = 1;
      HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_XTAL_ENABLED, &is_enable_lxt, 1);
      HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_STATUS, &wdt_staus, 4);
      HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_TIME, &wdt_time, 4);
      HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_CLK_FEQ, &wdt_clk, 2);
      HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_RC_CAL_IN_L, &is_lcpu_rccal, 1);
      HAL_PMU_SetWdt((uint32_t)hwp_wdt2);  // Add reboot cause for watchdog2
    }

    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HXT48);

    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
  }

  HAL_RCC_HCPU_ConfigHCLK(HCPU_FREQ_MHZ);

  // Reset sysclk used by HAL_Delay_us
  HAL_Delay_us(0);
}

void board_early_init(void) {
  HAL_StatusTypeDef status;

#if defined(HAL_V2D_GPU_MODULE_ENABLED)
  HAL_RCC_ResetModule(RCC_MOD_GPU);
#endif

  HAL_PreInit();

#ifdef SOC_BF0_HCPU
  if (PM_STANDBY_BOOT != SystemPowerOnModeGet()) {
    // Except Standby mode, all other boot mode need to re-calibrate RC48
    status = HAL_RCC_CalibrateRC48();
    HAL_ASSERT(HAL_OK == status);
  }
#endif /* SOC_BF0_HCPU */

  HAL_RCC_Init();

#ifdef SOC_BF0_HCPU
  if (PM_STANDBY_BOOT != SystemPowerOnModeGet()) {
    HAL_PMU_Init();
  }
#endif /* SOC_BF0_HCPU */

#ifdef SOC_BF0_HCPU
  /* init AES_ACC as normal mode */
  __HAL_SYSCFG_CLEAR_SECURITY();
  HAL_EFUSE_Init();
#endif
}

void board_init(void) {}
