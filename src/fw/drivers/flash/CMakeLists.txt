# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  cd_flash_driver.c
  flash_api.c
  flash_crc.c
  flash_erase.c
)
if(CONFIG_FLASH_QEMU)
  list(APPEND sources ../qemu/qemu_flash_hal.c)
elseif(CONFIG_FLASH_GD25LQ255E)
  list(APPEND sources gd25lq255e.c)
elseif(CONFIG_FLASH_GD25Q256E)
  list(APPEND sources gd25q256e.c)
endif()
pbl_library()
pbl_library_sources(${sources})
