# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_include_directories(mbedtls/include include)
pbl_compile_definitions(MBEDTLS_CONFIG_FILE="pebble_mbedtls_config.h")

file(GLOB sources CONFIGURE_DEPENDS mbedtls/library/*.c)

pbl_library_named(mbedtls)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_library_include_directories(mbedtls/library)
pbl_library_compile_options(-Wno-unused-but-set-variable)
