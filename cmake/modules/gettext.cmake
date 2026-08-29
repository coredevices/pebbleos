# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Translatable string extraction. Every firmware area contributes a .pot
# file; they are merged into build/pebbleos.pot, which `./pbl make_lang`
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
endfunction()

function(pbl_msgcat output)
  add_custom_command(
    OUTPUT ${output}
    COMMAND ${MSGCAT} ${ARGN} -o ${output}
    DEPENDS ${ARGN}
    COMMENT "Merging catalogs into ${output}"
    VERBATIM
  )
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
