# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The PebbleOS build model, borrowed from Zephyr:
#
#   - every directory that contributes code declares a library with
#     pbl_library() and feeds it with pbl_library_sources();
#   - all registered libraries end up in the single firmware image, so
#     nothing has to list them;
#   - include directories and defines registered with
#     pbl_include_directories() / pbl_compile_definitions() are visible to
#     every library, declared once next to the headers they expose.

# Global registries, read by pbl_link_firmware().
define_property(GLOBAL PROPERTY PBL_OBJECT_LIBS
  BRIEF_DOCS "Libraries whose every object is linked into the firmware")
define_property(GLOBAL PROPERTY PBL_STATIC_LIBS
  BRIEF_DOCS "Libraries the linker pulls objects from on demand")

function(pbl_init)
  add_library(pbl_interface INTERFACE)
  set_property(GLOBAL PROPERTY PBL_OBJECT_LIBS "")
  set_property(GLOBAL PROPERTY PBL_STATIC_LIBS "")
endfunction()

# Name a library after its directory, e.g. src/fw/services/timeline ->
# src__fw__services__timeline. Mirrors the waf model this replaces.
function(pbl_library_default_name out)
  file(RELATIVE_PATH rel ${PBL_BASE} ${CMAKE_CURRENT_SOURCE_DIR})
  string(REPLACE "/" "__" name ${rel})
  set(${out} ${name} PARENT_SCOPE)
endfunction()

# Declare the library the following pbl_library_*() calls apply to. The
# target itself is only created once it gets its first source file, so a
# directory whose sources are all configured out contributes nothing.
macro(pbl_library)
  pbl_library_default_name(_pbl_lib_name)
  pbl_library_named(${_pbl_lib_name})
endmacro()

macro(pbl_library_named name)
  set(PBL_CURRENT_LIBRARY ${name})
  set(PBL_CURRENT_LIBRARY_KIND objects)
endmacro()

# objects: every object file is linked (the default).
# stlib:   a static library the linker pulls from on demand. Use it for
#          third-party code that ships more than the firmware references.
macro(pbl_library_kind kind)
  if(TARGET ${PBL_CURRENT_LIBRARY})
    message(FATAL_ERROR
      "pbl_library_kind(): ${PBL_CURRENT_LIBRARY} already has sources")
  endif()
  set(PBL_CURRENT_LIBRARY_KIND ${kind})
endmacro()

function(pbl_library_ensure)
  if(NOT DEFINED PBL_CURRENT_LIBRARY)
    message(FATAL_ERROR "No library declared; call pbl_library() first")
  endif()
  if(TARGET ${PBL_CURRENT_LIBRARY})
    return()
  endif()
  if(PBL_CURRENT_LIBRARY_KIND STREQUAL "objects")
    add_library(${PBL_CURRENT_LIBRARY} OBJECT "")
    set_property(GLOBAL APPEND PROPERTY PBL_OBJECT_LIBS ${PBL_CURRENT_LIBRARY})
  elseif(PBL_CURRENT_LIBRARY_KIND STREQUAL "stlib")
    add_library(${PBL_CURRENT_LIBRARY} STATIC "")
    set_property(GLOBAL APPEND PROPERTY PBL_STATIC_LIBS ${PBL_CURRENT_LIBRARY})
  else()
    message(FATAL_ERROR
      "${PBL_CURRENT_LIBRARY}: unknown kind '${PBL_CURRENT_LIBRARY_KIND}'")
  endif()
  target_link_libraries(${PBL_CURRENT_LIBRARY} PRIVATE pbl_interface)
  # Generated headers are not tracked per library; every library waits for
  # all of them.
  add_dependencies(${PBL_CURRENT_LIBRARY} pbl_generated_headers)
endfunction()

function(pbl_library_sources)
  if(NOT ARGN)
    return()
  endif()
  pbl_library_ensure()
  target_sources(${PBL_CURRENT_LIBRARY} PRIVATE ${ARGN})
  # Asserts and log messages name the file they came from. __FILE_NAME__
  # would name the header a macro was expanded in, so each object gets the
  # name of the source it was built from instead.
  foreach(source ${ARGN})
    get_filename_component(name ${source} NAME)
    set_property(SOURCE ${source} TARGET_DIRECTORY ${PBL_CURRENT_LIBRARY}
      APPEND PROPERTY COMPILE_DEFINITIONS "__FILE_NAME_LEGACY__=\"${name}\"")
  endforeach()
endfunction()

function(pbl_library_sources_ifdef feature)
  if(${feature})
    pbl_library_sources(${ARGN})
  endif()
endfunction()

function(pbl_library_sources_ifndef feature)
  if(NOT ${feature})
    pbl_library_sources(${ARGN})
  endif()
endfunction()

# Drop sources matching one of the given patterns, the way the waf
# ant_glob() excl= lists did. A pattern is either a glob relative to the
# current directory ("prompt.c", "voice/*.c", "**/system_nrf*.c") or a
# directory whose whole subtree goes away ("vendor", "ble_hrm/**").
function(pbl_filter_sources var)
  cmake_parse_arguments(ARG "" "" "EXCLUDE" ${ARGN})
  set(sources ${${var}})
  foreach(pattern ${ARG_EXCLUDE})
    if(pattern MATCHES "^(.*)/\\*\\*$" OR NOT pattern MATCHES "[*.]")
      # A directory: everything below it goes.
      string(REGEX REPLACE "/\\*\\*$" "" dir ${pattern})
      list(FILTER sources EXCLUDE REGEX "/${dir}/")
    elseif(pattern MATCHES "^\\*\\*/(.*)$")
      file(GLOB_RECURSE excluded CONFIGURE_DEPENDS ${CMAKE_MATCH_1})
      if(excluded)
        list(REMOVE_ITEM sources ${excluded})
      endif()
    else()
      file(GLOB excluded CONFIGURE_DEPENDS ${pattern})
      if(excluded)
        list(REMOVE_ITEM sources ${excluded})
      endif()
    endif()
  endforeach()
  set(${var} ${sources} PARENT_SCOPE)
endfunction()

# Sources picked up by pattern. The globs are re-evaluated whenever the
# build is regenerated.
function(pbl_library_sources_glob)
  cmake_parse_arguments(ARG "RECURSE" "" "EXCLUDE" ${ARGN})
  if(ARG_RECURSE)
    file(GLOB_RECURSE sources CONFIGURE_DEPENDS ${ARG_UNPARSED_ARGUMENTS})
  else()
    file(GLOB sources CONFIGURE_DEPENDS ${ARG_UNPARSED_ARGUMENTS})
  endif()
  pbl_filter_sources(sources EXCLUDE ${ARG_EXCLUDE})
  pbl_library_sources(${sources})
endfunction()

function(pbl_library_include_directories)
  pbl_library_ensure()
  target_include_directories(${PBL_CURRENT_LIBRARY} PRIVATE ${ARGN})
endfunction()

function(pbl_library_compile_definitions)
  pbl_library_ensure()
  target_compile_definitions(${PBL_CURRENT_LIBRARY} PRIVATE ${ARGN})
endfunction()

function(pbl_library_compile_options)
  pbl_library_ensure()
  target_compile_options(${PBL_CURRENT_LIBRARY} PRIVATE ${ARGN})
endfunction()

# A vendor-supplied archive: lib<name>.a somewhere under the given paths.
function(pbl_library_import name)
  set(found NOTFOUND)
  foreach(dir ${ARGN})
    if(NOT IS_ABSOLUTE ${dir})
      set(dir ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    endif()
    if(EXISTS ${dir}/lib${name}.a)
      set(found ${dir}/lib${name}.a)
      break()
    endif()
  endforeach()
  if(NOT found)
    message(FATAL_ERROR "pbl_library_import(): lib${name}.a not found in ${ARGN}")
  endif()
  # Vendor archives are named after their build flags, which makes for
  # names CMake will not take as targets.
  string(MAKE_C_IDENTIFIER ${name} target)
  add_library(${target} STATIC IMPORTED GLOBAL)
  set_target_properties(${target} PROPERTIES IMPORTED_LOCATION ${found})
  set_property(GLOBAL APPEND PROPERTY PBL_STATIC_LIBS ${target})
endfunction()

# Headers and defines every library sees. Relative paths resolve against
# the calling directory; the matching build directory is added too, so
# generated headers are found next to their hand-written neighbours.
function(pbl_include_directories)
  foreach(dir ${ARGN})
    if(IS_ABSOLUTE ${dir})
      target_include_directories(pbl_interface INTERFACE ${dir})
      continue()
    endif()
    set(src ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    if(NOT IS_DIRECTORY ${src})
      message(FATAL_ERROR "pbl_include_directories(): ${src} does not exist")
    endif()
    set(bin ${CMAKE_CURRENT_BINARY_DIR}/${dir})
    file(MAKE_DIRECTORY ${bin})
    target_include_directories(pbl_interface INTERFACE ${bin} ${src})
  endforeach()
endfunction()

function(pbl_compile_definitions)
  target_compile_definitions(pbl_interface INTERFACE ${ARGN})
endfunction()

function(pbl_compile_options)
  target_compile_options(pbl_interface INTERFACE ${ARGN})
endfunction()

# Link options the firmware image needs, contributed from anywhere in the
# tree (add_link_options() would only reach the calling directory).
define_property(GLOBAL PROPERTY PBL_LINK_OPTIONS
  BRIEF_DOCS "Extra options for the firmware link")

function(pbl_link_options)
  set_property(GLOBAL APPEND PROPERTY PBL_LINK_OPTIONS ${ARGN})
endfunction()

function(pbl_add_subdirectory_ifdef feature)
  if(${feature})
    foreach(dir ${ARGN})
      add_subdirectory(${dir})
    endforeach()
  endif()
endfunction()

function(pbl_include_directories_ifdef feature)
  if(${feature})
    pbl_include_directories(${ARGN})
  endif()
endfunction()

function(pbl_compile_definitions_ifdef feature)
  if(${feature})
    pbl_compile_definitions(${ARGN})
  endif()
endfunction()
