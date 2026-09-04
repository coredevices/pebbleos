# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_compile_definitions(
  USE_HAL_DRIVER
  SOC_BF0_HCPU
  HAL_TICK_PER_SECOND=1024
  HAL_NMI_HANLDER_OVERRIDED
)
pbl_link_options(-Wl,--undefined=HAL_GetTick)
if(CONFIG_HAL_SIFLI_SF32LB52)
  pbl_compile_definitions(
    SF32LB52X
  )
endif()
set(micro_sources
  SiFli-SDK/drivers/hal/bf0_hal.c
  SiFli-SDK/drivers/hal/bf0_hal_adc.c
  SiFli-SDK/drivers/hal/bf0_hal_cortex.c
  SiFli-SDK/drivers/hal/bf0_hal_dma.c
  SiFli-SDK/drivers/hal/bf0_hal_efuse.c
  SiFli-SDK/drivers/hal/bf0_hal_hlp.c
  SiFli-SDK/drivers/hal/bf0_hal_pmu.c
  SiFli-SDK/drivers/hal/bf0_hal_rcc.c
  SiFli-SDK/drivers/hal/bf0_sys_cfg.c
)
if(CONFIG_HAL_SIFLI_AES OR CONFIG_HAL_SIFLI_HASH)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_aes.c
  )
endif()
if(CONFIG_HAL_SIFLI_AON)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_hpaon.c
    SiFli-SDK/drivers/hal/bf0_hal_lpaon.c
  )
endif()
if(CONFIG_HAL_SIFLI_ATIM)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_tim_ex.c
  )
endif()
if(CONFIG_HAL_SIFLI_AUDCODEC)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_audcodec_m.c
  )
endif()
if(CONFIG_HAL_SIFLI_AUDPRC)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_audprc.c
  )
endif()
if(CONFIG_HAL_SIFLI_BUSMON)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_busmon.c
  )
endif()
if(CONFIG_HAL_SIFLI_CRC)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_crc.c
  )
endif()
if(CONFIG_HAL_SIFLI_EPIC)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_epic.c
  )
endif()
if(CONFIG_HAL_SIFLI_EXTDMA)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_ext_dma.c
  )
endif()
if(CONFIG_HAL_SIFLI_EZIP)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_ezip.c
  )
endif()
if(CONFIG_HAL_SIFLI_GPIO)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_gpio.c
  )
endif()
if(CONFIG_HAL_SIFLI_GPT)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_tim.c
  )
endif()
if(CONFIG_HAL_SIFLI_HCD)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_hcd.c
  )
endif()
if(CONFIG_HAL_SIFLI_I2C)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_i2c.c
  )
endif()
if(CONFIG_HAL_SIFLI_I2S)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_i2s.c
  )
endif()
if(CONFIG_HAL_SIFLI_LCD)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_lcdc.c
  )
endif()
if(CONFIG_HAL_SIFLI_LCPU_CONFIG)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_lcpu_config.c
  )
endif()
if(CONFIG_HAL_SIFLI_LCPU_PATCH)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_patch.c
  )
endif()
if(CONFIG_HAL_SIFLI_LPTIM)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_lptim.c
  )
endif()
if(CONFIG_HAL_SIFLI_LRC_CAL)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_lrc_cal.c
  )
endif()
if(CONFIG_HAL_SIFLI_MAILBOX)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_mailbox.c
  )
endif()
if(CONFIG_HAL_SIFLI_MPI)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_mpi.c
    SiFli-SDK/drivers/hal/bf0_hal_mpi_ex.c
    SiFli-SDK/drivers/hal/flash_table.c
  )
endif()
if(CONFIG_HAL_SIFLI_PCD)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_pcd.c
  )
endif()
if(CONFIG_HAL_SIFLI_PDM)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_pdm.c
  )
endif()
if(CONFIG_HAL_SIFLI_PINMUX)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_pinmux.c
  )
endif()
if(CONFIG_HAL_SIFLI_PTC)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_ptc.c
  )
endif()
if(CONFIG_HAL_SIFLI_RNG)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_rng.c
  )
endif()
if(CONFIG_HAL_SIFLI_RTC)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_rtc.c
  )
endif()
if(CONFIG_HAL_SIFLI_SD)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_sdmmc.c
    SiFli-SDK/drivers/hal/bf0_hal_sd_ex.c
  )
endif()
if(CONFIG_HAL_SIFLI_SDHCI)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_sdhci.c
  )
endif()
if(CONFIG_HAL_SIFLI_SECU)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_secu.c
  )
endif()
if(CONFIG_HAL_SIFLI_SPI)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_spi.c
  )
endif()
if(CONFIG_HAL_SIFLI_TSEN)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_tsen.c
  )
endif()
if(CONFIG_HAL_SIFLI_UART)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_uart.c
  )
endif()
if(CONFIG_HAL_SIFLI_WDT)
  list(APPEND micro_sources
    SiFli-SDK/drivers/hal/bf0_hal_wdt.c
  )
endif()
if(CONFIG_HAL_SIFLI_IPC_QUEUE)
  list(APPEND micro_sources
    SiFli-SDK/middleware/ipc_queue/common/circular_buf.c
    SiFli-SDK/middleware/ipc_queue/common/ipc_hw.c
    SiFli-SDK/middleware/ipc_queue/common/ipc_queue.c
  )
endif()
if(CONFIG_HAL_SIFLI_SF32LB52)
  list(APPEND micro_sources
    SiFli-SDK/drivers/cmsis/sf32lb52x/bf0_pin_const.c
    sf32lb52/system_bf0_ap.c
  )
  if(CONFIG_HAL_SIFLI_IPC_QUEUE)
    list(APPEND micro_sources
      SiFli-SDK/middleware/ipc_queue/porting/sf32lb52x/hcpu/ipc_hw_port.c
    )
  endif()
  if(CONFIG_HAL_SIFLI_LCPU_CONFIG)
    list(APPEND micro_sources
      SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_config_type.c
    )
  endif()
  if(CONFIG_HAL_SIFLI_LCPU)
    list(APPEND micro_sources
      SiFli-SDK/drivers/cmsis/sf32lb52x/bf0_lcpu_init.c
      SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_patch.c
      ../nonfree/pebbleos-nonfree/sf32lb52/lcpu_patch_rev_b.c
      SiFli-SDK/drivers/cmsis/sf32lb52x/bt_rf_fulcal.c
      sf32lb52/lcpu_config.c
    )
  endif()
endif()
set(micro_includes
  SiFli-SDK/customer/boards/include
  SiFli-SDK/drivers/Include
  SiFli-SDK/drivers/cmsis/Include
  SiFli-SDK/external/CMSIS/Include
  SiFli-SDK/middleware/include
  SiFli-SDK/middleware/ipc_queue/common
  SiFli-SDK/middleware/ipc_queue/include
)
if(CONFIG_HAL_SIFLI_SF32LB52)
  list(APPEND micro_includes
    sf32lb52
    SiFli-SDK/drivers/cmsis/sf32lb52x
    SiFli-SDK/middleware/ipc_queue/porting/sf32lb52x/hcpu
  )
endif()
set(micro_defines)
if(CONFIG_BOARD_OBELIX OR CONFIG_BOARD_GETAFIX)
  list(APPEND micro_defines
    LXT_DISABLE
  )
endif()
pbl_include_directories(${micro_includes})
pbl_library_named(hal_sifli)
pbl_library_kind(stlib)
pbl_library_sources(${micro_sources})
pbl_library_compile_definitions(${micro_defines})
pbl_library_compile_options(-Wno-unused-value -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable -Wno-sign-compare -Wno-type-limits)
