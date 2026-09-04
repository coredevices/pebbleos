# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The unit test build.
#
# Every test is a standalone executable: the clar harness generated from
# the test file, the test itself, and the handful of firmware sources it
# exercises, compiled against the fakes and stubs under tests/. Tests
# declare themselves with pbl_clar_test(); nothing is built until
# pbl_test_finalize() runs, so a source needed by several tests with the
# same compile flags is compiled once and shared, the way the waf build
# this replaces did.

define_property(GLOBAL PROPERTY PBL_TESTS
  BRIEF_DOCS "Registered clar tests, materialized by pbl_test_finalize()")
define_property(GLOBAL PROPERTY PBL_TEST_OBJECTS
  BRIEF_DOCS "Product objects already compiled, keyed by flags and source")

set(PBL_TEST_PLATFORMS default obelix gabbro asterix)

# The tests build for the host with clang, at -O0 with symbols, and
# promote nothing to an error: they exercise code paths the firmware
# build never sees.
set(PBL_TEST_C_FLAGS
  -std=c11
  -Wall
  -Werror
  -Wno-error=unused-variable
  -Wno-error=unused-function
  -Wno-error=missing-braces
  -Wno-error=unused-const-variable
  -Wno-error=address-of-packed-member
  -Wno-enum-conversion
  -g3
  -gdwarf-4
  -O0
  -fdata-sections
  -ffunction-sections
  -fno-common
  -ffp-contract=off
  -fexcess-precision=standard
  # clang errors on the true == true assertions some tests compile.
  -Wno-tautological-compare
  -Wno-error
)

# Headers every test sees, in the order the compiler must find them:
# src/fw/util/time first, since its time.h deliberately shadows the
# host's, then the overrides, fakes and stubs ahead of the firmware.
set(PBL_TEST_INCLUDES_HEAD
  src/fw/util/time
  include
)
set(PBL_TEST_INCLUDES_TAIL
  tests/overrides/default
  tests/stubs
  tests/fakes
  tests/test_includes
  tests
  include
  kernel/native/include
  kernel/native/arch/posix/include
  subsys
  src/core
  src/fw
  src/boot
  src/fw/applib/vendor/tinflate
  src/fw/applib/vendor/uPNG
  third_party/nanopb/nanopb
  third_party/tinymt/TinyMT/tinymt
)

function(pbl_test_init)
  set_property(GLOBAL PROPERTY PBL_TESTS "")
  set_property(GLOBAL PROPERTY PBL_TEST_OBJECTS "")

  set(PBL_TEST_OBJ_DIR ${CMAKE_BINARY_DIR}/objects CACHE INTERNAL "")
  set(PBL_TEST_IMAGE_DIR ${CMAKE_BINARY_DIR}/test_images CACHE INTERNAL "")
  # Where a failing render drops what it produced, for the eye to judge.
  set(PBL_TEST_FAILED_DIR ${CMAKE_BINARY_DIR}/failed CACHE INTERNAL "")
  file(MAKE_DIRECTORY ${PBL_TEST_FAILED_DIR})
endfunction()

# Tests are named after their source file; a test that is not built at
# all is listed here rather than deleted, so it can be picked back up.
function(pbl_test_broken)
  set_property(GLOBAL APPEND PROPERTY PBL_TEST_BROKEN ${ARGN})
endfunction()

# The SDK platform each test platform stands in for, and the board whose
# headers it borrows. Tests load no defconfig, so the symbols the
# firmware sources expect are injected here.
function(_pbl_test_platform_defines platform out_defines out_bitdepth)
  if(platform STREQUAL "asterix")
    set(bitdepth 1)
    set(defines CONFIG_FLASH_GD25LQ255E=1 CONFIG_PLATFORM_FLINT=1 CONFIG_BOARD_ASTERIX=1)
  elseif(platform STREQUAL "obelix")
    set(bitdepth 8)
    set(defines CONFIG_FLASH_GD25Q256E=1 CONFIG_PLATFORM_EMERY=1 CONFIG_BOARD_OBELIX=1)
  elseif(platform STREQUAL "gabbro")
    # The round-display getafix board is the gabbro platform's closest
    # real-board analog.
    set(bitdepth 8)
    set(defines CONFIG_FLASH_GD25Q256E=1 CONFIG_PLATFORM_GABBRO=1 CONFIG_BOARD_GETAFIX=1)
  else()
    message(FATAL_ERROR "Unknown test platform '${platform}'")
  endif()

  # app_manager.c sizes app segments from these; the values are the ones
  # the old sdk_memory_limits.auto.h test override carried.
  list(APPEND defines
    "PLATFORM_NAME=\"${platform}\""
    CONFIG_SCREEN_COLOR_DEPTH_BITS=${bitdepth}
    CONFIG_APP_RAM_2X_SEGMENT_SIZE=23900
    CONFIG_APP_RAM_3X_SEGMENT_SIZE=65536
    CONFIG_APP_RAM_4X_SEGMENT_SIZE=65536
  )
  set(${out_defines} ${defines} PARENT_SCOPE)
  set(${out_bitdepth} ${bitdepth} PARENT_SCOPE)
endfunction()

# Declare a test. <name> names both the test and, unless SOURCE says
# otherwise, the file it is built from.
#
#   SOURCE     the test's own .c file
#   SOURCES    the firmware sources under test, relative to the repository
#              root; @PLATFORM@ and @BITDEPTH@ expand per platform
#   PLATFORMS  which platforms to build it for (default: 'default',
#              which is obelix without the platform-specific fixtures)
#   DEFINES    extra preprocessor defines
#   INCLUDES   extra header directories
#   OVERRIDES  directories under tests/overrides/ that come first on the
#              header search path
#   LIBS       extra libraries to link
#   DEPENDS    targets that have to be built first, for a test that needs
#              a generated header
#   TEST_IMAGES  the test renders against the image fixtures
function(pbl_clar_test name)
  cmake_parse_arguments(ARG "TEST_IMAGES" "SOURCE"
    "SOURCES;PLATFORMS;DEFINES;INCLUDES;OVERRIDES;LIBS;DEPENDS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "pbl_clar_test(${name}): unknown arguments ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  get_property(broken GLOBAL PROPERTY PBL_TEST_BROKEN)
  if(name IN_LIST broken)
    return()
  endif()

  if(NOT ARG_SOURCE)
    set(ARG_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${name}.c)
  elseif(NOT IS_ABSOLUTE ${ARG_SOURCE})
    set(ARG_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${ARG_SOURCE})
  endif()
  if(NOT ARG_PLATFORMS)
    set(ARG_PLATFORMS default)
  endif()

  foreach(platform ${ARG_PLATFORMS})
    if(NOT platform IN_LIST PBL_TEST_PLATFORMS)
      message(FATAL_ERROR "pbl_clar_test(${name}): unknown platform '${platform}'")
    endif()
    if(platform STREQUAL "default")
      set(id ${name})
    else()
      set(id ${name}_${platform})
    endif()

    get_property(ids GLOBAL PROPERTY PBL_TESTS)
    if(id IN_LIST ids)
      message(FATAL_ERROR "pbl_clar_test(${name}): '${id}' declared twice")
    endif()
    set_property(GLOBAL APPEND PROPERTY PBL_TESTS ${id})

    set_property(GLOBAL PROPERTY PBL_TEST_${id}_NAME ${name})
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_PLATFORM ${platform})
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_DIR ${CMAKE_CURRENT_SOURCE_DIR})
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_BINDIR ${CMAKE_CURRENT_BINARY_DIR})
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_SOURCE ${ARG_SOURCE})
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_SOURCES "${ARG_SOURCES}")
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_DEFINES "${ARG_DEFINES}")
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_INCLUDES "${ARG_INCLUDES}")
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_OVERRIDES "${ARG_OVERRIDES}")
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_LIBS "${ARG_LIBS}")
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_DEPENDS "${ARG_DEPENDS}")
    set_property(GLOBAL PROPERTY PBL_TEST_${id}_IMAGES "${ARG_TEST_IMAGES}")
  endforeach()
endfunction()

# Compile one firmware source with one set of flags, or hand back the
# object a previous test already had built with exactly those flags.
function(_pbl_test_object out source key flags)
  file(RELATIVE_PATH rel ${PBL_BASE} ${source})
  string(REPLACE "/" "__" flat ${rel})
  set(object ${PBL_TEST_OBJ_DIR}/${key}/${flat}.o)
  set(${out} ${object} PARENT_SCOPE)

  get_property(built GLOBAL PROPERTY PBL_TEST_OBJECTS)
  if(object IN_LIST built)
    return()
  endif()
  set_property(GLOBAL APPEND PROPERTY PBL_TEST_OBJECTS ${object})

  add_custom_command(
    OUTPUT ${object}
    COMMAND ${CMAKE_C_COMPILER_LAUNCHER} ${CMAKE_C_COMPILER} ${flags}
            -MD -MF ${object}.d -c ${source} -o ${object}
    DEPFILE ${object}.d
    DEPENDS ${source} pbl_generated_headers
    COMMENT "Building ${rel}"
    VERBATIM
  )
endfunction()

function(pbl_test_finalize)
  get_property(ids GLOBAL PROPERTY PBL_TESTS)
  list(LENGTH ids count)
  message(STATUS "Unit tests: ${count}")

  file(MAKE_DIRECTORY ${PBL_TEST_OBJ_DIR})
  foreach(id ${ids})
    _pbl_test_add(${id})
  endforeach()
endfunction()

function(_pbl_test_add id)
  foreach(field NAME PLATFORM DIR BINDIR SOURCE SOURCES DEFINES INCLUDES OVERRIDES LIBS DEPENDS IMAGES)
    get_property(${field} GLOBAL PROPERTY PBL_TEST_${id}_${field})
  endforeach()

  if(PLATFORM STREQUAL "default")
    # The default platform is obelix, told apart so tests can pick the
    # platform-neutral fixtures.
    set(platform obelix)
    set(platform_default 1)
    set(binary runme)
  else()
    set(platform ${PLATFORM})
    set(platform_default 0)
    set(binary runme_${PLATFORM})
  endif()
  _pbl_test_platform_defines(${platform} platform_defines bitdepth)

  file(RELATIVE_PATH reldir ${PBL_BASE} ${DIR})
  set(test_dir ${BINDIR}/${id})

  # Header search path: overrides first, then the shared list, with the
  # platform's resource headers and anything the test asked for.
  set(head "")
  foreach(dir ${PBL_TEST_INCLUDES_HEAD})
    list(APPEND head ${PBL_BASE}/${dir})
  endforeach()
  set(rest "")
  foreach(dir ${OVERRIDES})
    list(APPEND rest ${PBL_BASE}/tests/overrides/${dir})
  endforeach()
  foreach(dir ${PBL_TEST_INCLUDES_TAIL})
    list(APPEND rest ${PBL_BASE}/${dir})
  endforeach()
  list(APPEND rest ${PBL_BASE}/tests/overrides/default/resources/${platform})
  foreach(dir ${INCLUDES})
    if(IS_ABSOLUTE ${dir})
      list(APPEND rest ${dir})
    else()
      list(APPEND rest ${PBL_BASE}/${dir})
    endif()
  endforeach()
  list(APPEND rest ${PBL_IDL_INCLUDE_DIR} ${PBL_BASE}/include/pbl)
  set(includes ${head} ${rest})

  set(defines ${PBL_TEST_DEFINES} ${DEFINES} UNITTEST
      PLATFORM_DEFAULT=${platform_default} ${platform_defines})
  if(IMAGES)
    list(APPEND defines
      "TEST_IMAGES_PATH=\"${PBL_TEST_IMAGE_DIR}\""
      "TEST_OUTPUT_PATH=\"${PBL_TEST_FAILED_DIR}\""
      "PBI2PNG_EXE=\"${PBL_BASE}/tools/pbi2png.py\"")
  endif()

  # display.h and its per-platform sibling are force-included so that the
  # firmware's #ifdefs resolve without a per-platform reconfigure.
  if(platform STREQUAL "gabbro")
    set(display qemu_gabbro)
  else()
    set(display ${platform})
  endif()
  set(options -Wno-unused-command-line-argument
              -include${PBL_BASE}/src/fw/board/displays/display_${display}.h)

  # Everything that changes the generated code, and nothing that does
  # not: two tests agreeing on all of it share their objects.
  set(flags ${PBL_TEST_C_FLAGS} ${options})
  foreach(define ${PBL_TEST_DEFINES} ${defines})
    list(APPEND flags -D${define})
  endforeach()
  foreach(dir ${includes})
    list(APPEND flags -I${dir})
  endforeach()
  string(MD5 key "${flags}")

  set(objects "")
  foreach(source ${SOURCES})
    string(REPLACE "@PLATFORM@" "${platform}" source "${source}")
    string(REPLACE "@BITDEPTH@" "${bitdepth}" source "${source}")
    if(NOT IS_ABSOLUTE ${source})
      set(source ${PBL_BASE}/${source})
    endif()
    if(NOT EXISTS ${source})
      message(FATAL_ERROR "${id}: source ${source} not found")
    endif()
    _pbl_test_object(object ${source} ${key} "${flags}")
    list(APPEND objects ${object})
  endforeach()

  # clar turns the test's suite functions into a main().
  add_custom_command(
    OUTPUT ${test_dir}/clar_main.c ${test_dir}/clar.h
    COMMAND ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/clar/clar.py
            --file=${SOURCE} --clar-path=${PBL_BASE}/tools/clar ${test_dir}
    DEPENDS ${SOURCE} ${PBL_BASE}/tools/clar/clar.py ${PBL_BASE}/tools/clar/_clar.py
    COMMENT "Generating clar harness for ${id}"
    VERBATIM
  )

  set_source_files_properties(${objects} PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)
  add_executable(${id} ${SOURCE} ${test_dir}/clar_main.c ${objects})
  set_target_properties(${id} PROPERTIES
    OUTPUT_NAME ${binary}
    RUNTIME_OUTPUT_DIRECTORY ${test_dir})
  # clar.h lands next to the generated main.
  target_include_directories(${id} PRIVATE ${head} ${test_dir} ${rest})
  target_compile_options(${id} PRIVATE ${options})
  target_compile_definitions(${id} PRIVATE ${defines})
  # DUMA catches memory corruption; a handful of tests trip over it. It
  # has to come before any system library: it overrides malloc, and once
  # the linker has bound that to libc it stops looking.
  set(libs ${LIBS} libutil libbtutil)
  if(NOT "DUMA_DISABLED" IN_LIST defines)
    list(APPEND libs duma pthread)
  endif()
  target_link_libraries(${id} PRIVATE ${libs} m)
  if(IMAGES)
    add_dependencies(${id} pbl_test_images)
  endif()
  if(DEPENDS)
    add_dependencies(${id} ${DEPENDS})
  endif()

  add_test(NAME ${id} COMMAND $<TARGET_FILE:${id}> WORKING_DIRECTORY ${test_dir})
  set_tests_properties(${id} PROPERTIES LABELS "${reldir};${platform}")
  if(IMAGES)
    set_tests_properties(${id} PROPERTIES FIXTURES_REQUIRED pbl_test_failed_dir)
  endif()
endfunction()
