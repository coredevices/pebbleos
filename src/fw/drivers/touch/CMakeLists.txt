# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_TOUCH_QEMU)
  list(APPEND sources ../qemu/qemu_touch.c)
elseif(CONFIG_TOUCH_CST816)
  list(APPEND sources cst816/cst816.c)
endif()
pbl_library()
pbl_library_sources(${sources})
