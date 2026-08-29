# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources QR-Code-generator/c/qrcodegen.c)
set(includes QR-Code-generator/c)
pbl_include_directories(${includes})
pbl_library_named(qr_code_generator)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_library_compile_options(-Wno-type-limits)
