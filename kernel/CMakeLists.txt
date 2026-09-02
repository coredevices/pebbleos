# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

file(GLOB sources CONFIGURE_DEPENDS *.c)

pbl_library_named(kernel)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_linker_sources(rom-start kobj_init.ld)

if(CONFIG_KERNEL_BACKEND_FREERTOS)
  pbl_include_directories(freertos/include)
  add_subdirectory(freertos)
endif()
