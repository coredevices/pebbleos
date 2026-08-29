# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Compiler and linker flags shared by every firmware object.

add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=c11>)

add_compile_options(
  -Wall
  -Wextra
  -Werror
  -Wpointer-arith
  -Wno-unused-parameter
  -Wno-missing-field-initializers
  -Wno-address-of-packed-member
)

set(pbl_arch_flags
  -fvar-tracking-assignments
  -mthumb
  -ffreestanding
  -ffunction-sections
  # let --gc-sections drop unreferenced const/data objects too
  -fdata-sections
  -fbuiltin
  -fno-builtin-itoa
)

if(CONFIG_DEBUG_INFO)
  # -g3 keeps macro definitions; -gdwarf-4 the more detailed debug info.
  list(APPEND pbl_arch_flags -g3 -gdwarf-4)
endif()

if(CONFIG_COMPILER_SAVE_TEMPS)
  list(APPEND pbl_arch_flags -save-temps=obj)
endif()

if(CONFIG_LTO)
  list(APPEND pbl_arch_flags
    -flto
    -flto-partition=balanced
    --param lto-partitions=128
    -fuse-linker-plugin
    -fno-if-conversion
    -fno-caller-saves
    -fira-region=mixed
    -finline-functions
    -fconserve-stack
    --param inline-unit-growth=1
    --param max-inline-insns-auto=1
    --param max-cse-path-length=1000
    --param max-grow-copy-bb-insns=1
    -fno-hoist-adjacent-loads
    -fno-optimize-sibling-calls
    -fno-schedule-insns2
  )
endif()

set(pbl_cpu_fpu "")
if(CONFIG_SOC_NRF52)
  list(APPEND pbl_arch_flags -mcpu=cortex-m4)
  set(pbl_cpu_fpu fpv4-sp-d16)
elseif(CONFIG_SOC_SF32LB52)
  list(APPEND pbl_arch_flags -mcpu=star-mc1)
  set(pbl_cpu_fpu fpv5-sp-d16)
elseif(CONFIG_QEMU AND CONFIG_CORTEX_M4)
  list(APPEND pbl_arch_flags -mcpu=cortex-m4 -Dsniprintf=snprintf -D_USE_LONG_TIME_T)
elseif(CONFIG_QEMU AND CONFIG_CORTEX_M33)
  list(APPEND pbl_arch_flags -mcpu=cortex-m33+nofp+nodsp -Dsniprintf=snprintf -D_USE_LONG_TIME_T)
endif()

# QEMU does not have an FPU. Without -mfloat-abi=softfp no FPU instructions
# are emitted, and __SOFTFP__=1 gets defined (a misleading name, but that
# is what the toolchain does).
if(CONFIG_QEMU)
  set(pbl_cpu_fpu "")
endif()

if(pbl_cpu_fpu)
  list(APPEND pbl_arch_flags -mfloat-abi=softfp -mfpu=${pbl_cpu_fpu})
endif()

# Reproducibility: strip the absolute source-root prefix from every
# embedded path, so binaries do not depend on where the tree sits.
# -ffile-prefix-map covers debug info and __FILE__; -fdebug-prefix-map is
# a subset, listed for toolchains predating -ffile-prefix-map.
list(APPEND pbl_arch_flags
  -ffile-prefix-map=${PBL_BASE}=.
  -fdebug-prefix-map=${PBL_BASE}=.
)

if(CONFIG_RELEASE)
  set(pbl_optimize -Os)
  message(STATUS "Optimization: release (-Os)")
elseif(CONFIG_NO_OPTIMIZATIONS)
  set(pbl_optimize -O0)
  message(STATUS "Optimization: none (-O0)")
elseif(CONFIG_DEBUG_OPTIMIZATIONS)
  set(pbl_optimize -Og)
  message(STATUS "Optimization: debug (-Og)")
else()
  set(pbl_optimize -Os)
  message(STATUS "Optimization: size (-Os)")
endif()

add_compile_options(${pbl_arch_flags} ${pbl_optimize})
add_link_options(-Wl,--warn-common ${pbl_arch_flags} ${pbl_optimize})

# Kconfig reaches every compilation unit, headers included.
# SHELL: keeps the flag and its argument together; CMake would
# otherwise fold the repeated -include options into one.
add_compile_options("SHELL:-include ${PBL_AUTOCONF_H}")

# time.h shims the firmware needs ahead of the toolchain's.
include_directories(${PBL_BASE}/src/fw/util/time)

# MAX_FONT_GLYPH_SIZE comes from the SDK platform description.
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -c
    "import sys; sys.path.insert(0, 'tools'); from pebble_sdk_platform import pebble_platforms; print(pebble_platforms['${PBL_PLATFORM_NAME}']['MAX_FONT_GLYPH_SIZE'])"
  WORKING_DIRECTORY ${PBL_BASE}
  OUTPUT_VARIABLE PBL_MAX_FONT_GLYPH_SIZE
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY
)
add_compile_definitions(MAX_FONT_GLYPH_SIZE=${PBL_MAX_FONT_GLYPH_SIZE})

# Stationary mode is for shipping watch firmware only.
if(NOT CONFIG_RECOVERY_FW AND NOT CONFIG_QEMU AND NOT CONFIG_SHELL_SDK)
  add_compile_definitions(STATIONARY_MODE)
endif()

add_compile_definitions(FIRMWARE_OFFSET=${CONFIG_FIRMWARE_OFFSET})
