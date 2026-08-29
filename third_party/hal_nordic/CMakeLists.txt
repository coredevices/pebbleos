# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_HAL_NORDIC_NRF52840)
  pbl_compile_definitions(
    NRF52840_XXAA
    NRF_CONFIG_NFCT_PINS_AS_GPIOS
    CONFIG_GPIO_AS_PINRESET
    NRFX_I2S_ENABLED=1
    NRFX_I2S0_ENABLED=1
    NRFX_GPIOTE_ENABLED=1
    NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS=4
    NRFX_PDM_ENABLED=1
    NRFX_PPI_ENABLED=1
    NRFX_PWM_ENABLED=1
    NRFX_PWM0_ENABLED=1
    NRFX_SPIM_ENABLED=1
    NRFX_SPIM3_ENABLED=1
    NRFX_TIMER_ENABLED=1
    NRFX_TIMER1_ENABLED=1
    NRFX_TIMER2_ENABLED=1
    NRFX_TWIM_ENABLED=1
    NRFX_TWIM0_ENABLED=1
    NRFX_TWIM1_ENABLED=1
    NRFX_UARTE_ENABLED=1
    NRFX_UARTE0_ENABLED=1
  )
endif()
file(GLOB_RECURSE micro_sources CONFIGURE_DEPENDS nrfx/*.c)
pbl_filter_sources(micro_sources EXCLUDE **/system_nrf*.c **/startup_nrf_common.c **/nrfx_twi_twim.c)
set(micro_includes
  .
  nrfx
  nrfx/hal
  nrfx/helpers
  nrfx/mdk
  nrfx/drivers/include
  nrfx/templates
)
pbl_include_directories(${micro_includes})
pbl_library_named(hal_nordic)
pbl_library_kind(stlib)
pbl_library_sources(${micro_sources})
pbl_library_compile_options(-Wno-unused-variable)
