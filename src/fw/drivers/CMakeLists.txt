# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_add_subdirectory_ifdef(CONFIG_SPEAKER speaker)
pbl_add_subdirectory_ifdef(CONFIG_AMBIENT_LIGHT ambient)
pbl_add_subdirectory_ifdef(CONFIG_TEMPERATURE temperature)
pbl_add_subdirectory_ifdef(CONFIG_BATTERY battery)
add_subdirectory(imu)
pbl_add_subdirectory_ifdef(CONFIG_DISPLAY display)
pbl_add_subdirectory_ifdef(CONFIG_EXTI exti)
pbl_add_subdirectory_ifdef(CONFIG_FLASH flash)
pbl_add_subdirectory_ifdef(CONFIG_GPIO gpio)
pbl_add_subdirectory_ifdef(CONFIG_I2C i2c)
pbl_add_subdirectory_ifdef(CONFIG_BACKLIGHT backlight)
pbl_add_subdirectory_ifdef(CONFIG_PRESSURE pressure)
pbl_add_subdirectory_ifdef(CONFIG_MIC mic)
pbl_add_subdirectory_ifdef(CONFIG_OTP otp)
pbl_add_subdirectory_ifdef(CONFIG_PMIC pmic)
pbl_add_subdirectory_ifdef(CONFIG_PWM pwm)
pbl_add_subdirectory_ifdef(CONFIG_RNG rng)
pbl_add_subdirectory_ifdef(CONFIG_RTC rtc)
pbl_add_subdirectory_ifdef(CONFIG_TOUCH touch)
pbl_add_subdirectory_ifdef(CONFIG_UART uart)
pbl_add_subdirectory_ifdef(CONFIG_VIBE vibe)
pbl_add_subdirectory_ifdef(CONFIG_WATCHDOG watchdog)
pbl_add_subdirectory_ifdef(CONFIG_HRM hrm)
if(CONFIG_MPU_TYPE_ARMV8M)
  set(mpu_arch_source "mpu/mpu_armv8m.c")
else()
  set(mpu_arch_source "mpu/mpu_armv7m.c")
endif()
pbl_library_named(driver_mpu)
pbl_library_sources(
  mpu.c
  ${mpu_arch_source}
)
pbl_library_named(driver_task_watchdog)
pbl_library_sources(task_watchdog.c)
if(CONFIG_QEMU)
  pbl_library_named(driver_button)
  pbl_library_sources(qemu/qemu_button.c)
  pbl_library_named(driver_mcu)
  pbl_library_sources(stubs/mcu.c)
  pbl_library_named(driver_qemu_serial)
  pbl_library_sources(
    qemu/qemu_serial.c
    qemu/qemu_serial_util.c
  )
  pbl_library_named(driver_qemu_settings)
  pbl_library_sources(qemu/qemu_settings.c)
  pbl_library_named(driver_qemu_stubs)
  pbl_library_sources(
    stubs/ambient_light.c
    qemu/qemu_vibe.c
    stubs/otp.c
  )
  if(CONFIG_MAG)
    pbl_library_named(driver_mag3110)
    pbl_library_sources(imu/mag3110/mag3110.c)
  endif()
elseif(CONFIG_SOC_NRF52)
  pbl_library_named(driver_button)
  pbl_library_sources(
    nrf5/button.c
    nrf5/debounced_button.c
  )
  pbl_library_named(driver_mcu)
  pbl_library_sources(stubs/mcu.c)
  pbl_library_named(driver_qspi)
  pbl_library_sources(nrf5/qspi.c)
  pbl_library_named(driver_nrf5_hfxo)
  pbl_library_sources(nrf5/hfxo.c)
elseif(CONFIG_SOC_SF32LB52)
  pbl_library_named(driver_button)
  pbl_library_sources(
    sf32lb52/button.c
    sf32lb52/debounced_button.c
  )
  pbl_library_named(driver_mcu)
  pbl_library_sources(sf32lb52/mcu.c)
  pbl_library_named(driver_qspi)
  pbl_library_sources(sf32lb52/qspi.c)
  pbl_library_named(driver_rc10k)
  pbl_library_sources(sf32lb52/rc10k.c)
endif()
