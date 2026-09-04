# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources battery_common.c)
if(CONFIG_BATTERY_QEMU)
  list(APPEND sources ../qemu/qemu_battery.c)
endif()
pbl_library()
pbl_library_sources(${sources})
