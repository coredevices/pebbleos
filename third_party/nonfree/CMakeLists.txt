# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(nonfree pebbleos-nonfree)

if(CONFIG_NRF_FUEL_GAUGE)
  if(CONFIG_CORTEX_M4)
    set(fuel_gauge_lib ${nonfree}/npm1300/nrf_fuel_gauge/lib/cortex-m4/soft-float)
  elseif(CONFIG_CORTEX_M33)
    set(fuel_gauge_lib ${nonfree}/npm1300/nrf_fuel_gauge/lib/cortex-m33/soft-float)
  endif()
  pbl_include_directories(
    ${nonfree}/npm1300/nrf_fuel_gauge/include
    ${nonfree}/npm1300/models
  )
  pbl_library_import(nrf_fuel_gauge ${fuel_gauge_lib})
endif()

if(CONFIG_CST816_FW)
  if(CONFIG_BOARD_OBELIX)
    pbl_include_directories(${nonfree}/cst816/obelix)
  elseif(CONFIG_BOARD_GETAFIX)
    pbl_include_directories(${nonfree}/cst816/getafix)
  else()
    message(FATAL_ERROR "no CST816 firmware for board ${PBL_BOARD}")
  endif()
endif()

if(CONFIG_GH3X2X_ALGO)
  set(gh3x2x ${nonfree}/gh3x2x)
  pbl_include_directories(
    ${gh3x2x}/algo_lib/algo_params/HR/04_EXCLUSIVE
    ${gh3x2x}/demo_code/demo_algo_code/goodix_algo_application/inc
    ${gh3x2x}/demo_code/demo_algo_code/goodix_algo_call/inc
    ${gh3x2x}/demo_code/demo_algo_code/goodix_algo_call/inc/hr_exc
    ${gh3x2x}/demo_code/demo_algo_code/goodix_algo_call/inc/spo2_pre_exc
    ${gh3x2x}/demo_code/demo_kernel_code/driver/inc
    ${gh3x2x}/demo_code/demo_kernel_code/kernel
    ${gh3x2x}/demo_code/demo_kernel_code/module/gh_agc
    ${gh3x2x}/demo_code/demo_kernel_code/module/gh_ecg
    ${gh3x2x}/demo_code/demo_kernel_code/module/gh_other
    ${gh3x2x}/demo_code/demo_kernel_code/module/gh_protocol
    ${gh3x2x}/demo_code/demo_kernel_code/module/gh_soft_adt
    ${gh3x2x}/demo_code/demo_mp_code/inc
  )
  pbl_library_import(star-mc1_arm-gcc14.2.1_softfp+Os_common_dl ${gh3x2x}/algo_lib/COMMON_DL)
  pbl_library_import(star-mc1_arm-gcc14.2.1_softfp+Os_common_dsp ${gh3x2x}/algo_lib/COMMON_DSP)
  pbl_library_import("GitInfo_star-mc1-arm-none-eabi-gcc1421(softfp+Os)" ${gh3x2x}/algo_lib/HR)
  pbl_library_import(star-mc1_arm-gcc14.2.1_softfp+Os_hrv ${gh3x2x}/algo_lib/HRV)
  pbl_library_import(star-mc1_arm-gcc14.2.1_softfp+Os_nadt ${gh3x2x}/algo_lib/NADT)
  pbl_library_import(star-mc1_arm-gcc14.2.1_softfp+Os_spo2 ${gh3x2x}/algo_lib/SPO2)
  pbl_library_import(arm-none-eabi-gcc-softfp-Os-gh_common ${gh3x2x}/drv_lib)

  file(GLOB_RECURSE sources CONFIGURE_DEPENDS
    ${gh3x2x}/algo_lib/*.c
    ${gh3x2x}/demo_code/*.c
  )
  pbl_library_named(gh3x2x)
  pbl_library_sources(${sources})
  pbl_library_compile_options(-Wno-sign-compare)
endif()
