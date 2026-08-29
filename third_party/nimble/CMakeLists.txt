# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(nimble mynewt-nimble)

set(includes
  ${nimble}/nimble/include
  ${nimble}/nimble/host/include
  ${nimble}/nimble/host/services/ans/include
  ${nimble}/nimble/host/services/bas/include
  ${nimble}/nimble/host/services/bleuart/include
  ${nimble}/nimble/host/services/gap/include
  ${nimble}/nimble/host/services/gatt/include
  ${nimble}/nimble/host/services/ias/include
  ${nimble}/nimble/host/services/dis/include
  ${nimble}/nimble/host/services/lls/include
  ${nimble}/nimble/host/services/tps/include
  ${nimble}/nimble/host/util/include
  ${nimble}/nimble/transport/include
  ${nimble}/nimble/transport/common/hci_h4/include
  ${nimble}/porting/nimble/include
  port/include
)

file(GLOB sources CONFIGURE_DEPENDS
  ${nimble}/porting/nimble/src/*.c
  ${nimble}/nimble/src/*.c
  ${nimble}/nimble/host/src/*.c
  ${nimble}/nimble/host/util/src/*.c
  ${nimble}/nimble/host/services/bas/src/*.c
  ${nimble}/nimble/host/services/gap/src/*.c
  ${nimble}/nimble/host/services/gatt/src/*.c
  ${nimble}/nimble/host/services/dis/src/*.c
  ${nimble}/nimble/transport/src/*.c
  ${nimble}/nimble/transport/common/hci_h4/src/*.c
  port/src/npl_os_pebble.c
)
pbl_filter_sources(sources EXCLUDE
  # has a duplicate define, and is not needed anyway
  ${nimble}/nimble/host/src/ble_gatts_lcl.c
  ${nimble}/porting/nimble/src/hal_timer.c
)

set(defines "")

if(CONFIG_SOC_NRF52)
  file(GLOB controller_sources CONFIGURE_DEPENDS
    ${nimble}/nimble/controller/src/*.c
    ${nimble}/nimble/drivers/nrf5x/src/*.c
    ${nimble}/nimble/drivers/nrf5x/src/nrf52/*.c
  )
  list(APPEND sources ${controller_sources}
    ${nimble}/porting/nimble/src/hal_timer.c
    port/src/nrf52.c
  )
  list(PREPEND includes
    port/include/nrf52
    ${nimble}/nimble/controller/include
    ${nimble}/nimble/drivers/nrf5x/include
  )
  list(APPEND defines FREERTOS=1 PEBBLEOS NRF52_SERIES NIMBLE_CFG_CONTROLLER=1)
  pbl_compile_definitions(NIMBLE_CFG_CONTROLLER=1)
elseif(CONFIG_SOC_SF32LB52)
  list(APPEND sources
    transport/hci_sf32lb52.c
    ../nonfree/pebbleos-nonfree/sf32lb52/lcpu_52x.c
  )
  list(PREPEND includes port/include/sf32lb52)

  if(CONFIG_NIMBLE_HCI_SF32LB52_TRACE_BINARY)
    pbl_compile_definitions(NIMBLE_HCI_SF32LB52_TRACE NIMBLE_HCI_SF32LB52_TRACE_BINARY)
  elseif(CONFIG_NIMBLE_HCI_SF32LB52_TRACE_LOG)
    pbl_compile_definitions(NIMBLE_HCI_SF32LB52_TRACE NIMBLE_HCI_SF32LB52_TRACE_LOG)
  endif()
endif()

pbl_include_directories(${includes})

pbl_library_named(nimble)
pbl_library_sources(${sources})
pbl_library_compile_definitions(${defines})
pbl_library_compile_options(
  -Wno-pointer-arith
  -Wno-unused-function
  -Wno-sign-compare
  -Wno-old-style-declaration
  -Wimplicit-fallthrough=1
)
