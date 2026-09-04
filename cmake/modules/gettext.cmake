# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Translatable string extraction. Every firmware area contributes a .pot
# file; they are merged into the firmware's catalog, which `pbl make_lang`
# turns into per-language catalogs.

find_program(XGETTEXT xgettext REQUIRED)
find_program(MSGCAT msgcat REQUIRED)

# gettext >= 0.22 shells out to git for a reproducible POT-Creation-Date,
# which warns once per source file on out-of-tree build paths.
execute_process(COMMAND ${XGETTEXT} --help OUTPUT_VARIABLE xgettext_help)
if(xgettext_help MATCHES "--no-git")
  set(XGETTEXT_NO_GIT --no-git)
else()
  set(XGETTEXT_NO_GIT "")
endif()

set(PBL_GETTEXT_KEYWORDS
  i18n_noop
  i18n_get
  i18n_get_with_buffer
  sys_i18n_get_with_buffer
  i18n_ctx_noop:1c,2
  i18n_ctx_get:1c,2
  i18n_ctx_get_with_buffer:1c,2
)

define_property(GLOBAL PROPERTY PBL_SERVICES_POT
  BRIEF_DOCS "Per-service .pot files merged into services.pot")

define_property(GLOBAL PROPERTY PBL_POT_TARGET
  BRIEF_DOCS "The target that builds the firmware's merged .pot")

# A .pot is produced in one directory and merged in another, and a custom
# command's rule is only reachable from the directory that added it. Every
# .pot therefore also gets a target, named after its path so that any
# directory can derive it; consumers depend on the target as well as on the
# file, the target to reach the rule and the file to keep rebuild tracking.
function(pbl_pot_target output var)
  file(RELATIVE_PATH relative ${PROJECT_BINARY_DIR} ${output})
  string(REGEX REPLACE "[^A-Za-z0-9]" "_" relative ${relative})
  set(${var} pbl_pot_${relative} PARENT_SCOPE)
endfunction()

function(pbl_gettext output)
  set(sources ${ARGN})
  list(SORT sources)
  set(keyword_args "")
  foreach(keyword ${PBL_GETTEXT_KEYWORDS})
    list(APPEND keyword_args --keyword=${keyword})
  endforeach()

  # The source list is long enough to overflow a command line.
  set(list_file ${output}.files)
  string(REPLACE ";" "\n" file_list "${sources}")
  file(CONFIGURE OUTPUT ${list_file} CONTENT "${file_list}\n" @ONLY)

  add_custom_command(
    OUTPUT ${output}
    COMMAND ${XGETTEXT} ${XGETTEXT_NO_GIT} -c/ -k --from-code=UTF-8
            --language=C ${keyword_args} -o ${output} --files-from=${list_file}
    DEPENDS ${sources} ${list_file}
    WORKING_DIRECTORY ${PBL_BASE}
    COMMENT "Extracting strings into ${output}"
    VERBATIM
  )

  pbl_pot_target(${output} target)
  add_custom_target(${target} DEPENDS ${output})
endfunction()

function(pbl_msgcat output)
  set(depends ${ARGN})
  foreach(input ${ARGN})
    pbl_pot_target(${input} dependency)
    if(TARGET ${dependency})
      list(APPEND depends ${dependency})
    endif()
  endforeach()

  add_custom_command(
    OUTPUT ${output}
    COMMAND ${MSGCAT} ${ARGN} -o ${output}
    DEPENDS ${depends}
    COMMENT "Merging catalogs into ${output}"
    VERBATIM
  )

  pbl_pot_target(${output} target)
  add_custom_target(${target} DEPENDS ${output})
endfunction()

# The firmware's merged catalog, built along with the firmware itself.
function(pbl_firmware_pot output)
  pbl_msgcat(${output} ${ARGN})
  pbl_pot_target(${output} target)
  set_property(GLOBAL PROPERTY PBL_POT_TARGET ${target})
endfunction()

# A service's translatable strings, merged into services.pot.
function(pbl_service_gettext name)
  set(output ${CMAKE_CURRENT_BINARY_DIR}/services_${name}.pot)
  set(sources "")
  foreach(source ${ARGN})
    if(NOT IS_ABSOLUTE ${source})
      set(source ${CMAKE_CURRENT_SOURCE_DIR}/${source})
    endif()
    list(APPEND sources ${source})
  endforeach()
  pbl_gettext(${output} ${sources})
  set_property(GLOBAL APPEND PROPERTY PBL_SERVICES_POT ${output})
endfunction()
