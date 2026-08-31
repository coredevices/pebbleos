# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# DUMA guards the tests' allocations against overruns and use-after-free.

add_library(duma STATIC duma.c print.c sem_inc.c)
# Disabling a warning -Werror is tripping over.
target_compile_options(duma PRIVATE -Wno-unused-but-set-variable)
if(APPLE)
  target_compile_definitions(duma PRIVATE DUMA_SO_PREFER_GETENV)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_compile_definitions(duma PRIVATE _POSIX_C_SOURCE)
endif()
