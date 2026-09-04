# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The firmware image: the final link, the hex/bin images derived from it,
# and the bundling and QEMU targets built on top.

set(PBL_FIRMWARE_PY ${PBL_BASE}/tools/cmake/firmware.py)
set(PBL_GENERATE_PY ${PBL_BASE}/tools/cmake/generate.py)

function(pbl_link_firmware)
  get_property(object_libs GLOBAL PROPERTY PBL_OBJECT_LIBS)
  get_property(static_libs GLOBAL PROPERTY PBL_STATIC_LIBS)

  pbl_linker_script(ldscript ldscript_depends)

  add_executable(pebbleos ${PBL_RESOURCE_SOURCES} ${PBL_FIRMWARE_SOURCES})
  set_target_properties(pebbleos PROPERTIES
    OUTPUT_NAME pebbleos
    SUFFIX .elf
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
  )
  add_dependencies(pebbleos pbl_generated_headers)

  foreach(lib ${object_libs})
    target_sources(pebbleos PRIVATE $<TARGET_OBJECTS:${lib}>)
  endforeach()

  # A link group resolves the dependencies between the static libraries
  # without having to order them by hand.
  target_link_libraries(pebbleos PRIVATE
    pbl_interface
    -Wl,--start-group ${static_libs} ${PBL_LIBC_LIBS} -Wl,--end-group
  )

  # The map is tens of megabytes and costs about a fifth of the link, so
  # it is only written when something is going to read it.
  set(map_options "")
  if(CONFIG_LINKER_MAP)
    set(map_options -Wl,--cref -Wl,-Map=pebbleos.map)
  endif()

  target_link_options(pebbleos PRIVATE
    ${map_options}
    -Wl,--gc-sections
    -Wl,--build-id=sha1
    -Wl,--sort-section=alignment
    -Wl,--print-memory-usage
    ${PBL_LIBC_LINK_FLAGS}
    # Every allocation goes through the firmware's own heaps.
    -Wl,--wrap=malloc -Wl,--undefined=__wrap_malloc
    -Wl,--wrap=realloc -Wl,--undefined=__wrap_realloc
    -Wl,--wrap=calloc -Wl,--undefined=__wrap_calloc
    -Wl,--wrap=free -Wl,--undefined=__wrap_free
    -T${ldscript}
  )
  get_property(link_options GLOBAL PROPERTY PBL_LINK_OPTIONS)
  target_link_options(pebbleos PRIVATE ${link_options})
  set_property(TARGET pebbleos APPEND PROPERTY LINK_DEPENDS ${ldscript})
  add_custom_target(pbl_linker_script DEPENDS ${ldscript})
  add_dependencies(pebbleos pbl_linker_script)

  set(elf ${PROJECT_BINARY_DIR}/pebbleos.elf)
  set(hex ${PROJECT_BINARY_DIR}/pebbleos.hex)
  set(bin ${PROJECT_BINARY_DIR}/pebbleos.bin)
  set(objcopy_args -S -R .stack -R .priv_bss -R .bss -R .retained)

  if(CONFIG_PBLBOOT)
    # The bootloader expects an image header ahead of the firmware.
    set(nohdr_hex ${PROJECT_BINARY_DIR}/pebbleos.nohdr.hex)
    set(nohdr_bin ${PROJECT_BINARY_DIR}/pebbleos.nohdr.bin)
    add_custom_command(OUTPUT ${nohdr_hex}
      COMMAND ${CMAKE_OBJCOPY} ${objcopy_args} -O ihex ${elf} ${nohdr_hex}
      DEPENDS pebbleos VERBATIM)
    add_custom_command(OUTPUT ${nohdr_bin}
      COMMAND ${CMAKE_OBJCOPY} ${objcopy_args} -O binary ${elf} ${nohdr_bin}
      DEPENDS pebbleos VERBATIM)
    add_custom_command(OUTPUT ${hex}
      COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_GENERATE_PY} pblboot-header
              --input ${nohdr_hex} --output ${hex} --offset ${CONFIG_FIRMWARE_OFFSET}
      DEPENDS ${nohdr_hex} ${PBL_GENERATE_PY}
      WORKING_DIRECTORY ${PBL_BASE} VERBATIM)
    add_custom_command(OUTPUT ${bin}
      COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_GENERATE_PY} pblboot-header
              --input ${nohdr_bin} --output ${bin} --offset ${CONFIG_FIRMWARE_OFFSET}
      DEPENDS ${nohdr_bin} ${PBL_GENERATE_PY}
      WORKING_DIRECTORY ${PBL_BASE} VERBATIM)
  else()
    add_custom_command(OUTPUT ${hex}
      COMMAND ${CMAKE_OBJCOPY} ${objcopy_args} -O ihex ${elf} ${hex}
      DEPENDS pebbleos VERBATIM)
    add_custom_command(OUTPUT ${bin}
      COMMAND ${CMAKE_OBJCOPY} ${objcopy_args} -O binary ${elf} ${bin}
      DEPENDS pebbleos VERBATIM)
  endif()

  set(artifacts ${hex} ${bin})

  # Hashed log strings: the dictionary the console and the bundle use to
  # turn hashes back into messages.
  set(loghash ${PROJECT_BINARY_DIR}/src/fw/loghash_dict.json)
  if(CONFIG_LOG_HASHED)
    set(fw_loghash ${PROJECT_BINARY_DIR}/pebbleos_loghash_dict.json)
    add_custom_command(
      OUTPUT ${fw_loghash}
      COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_GENERATE_PY} loghash
              --elf ${elf} --output ${fw_loghash}
      DEPENDS pebbleos ${PBL_GENERATE_PY}
      WORKING_DIRECTORY ${PBL_BASE}
      COMMENT "Checking and hashing log strings"
      VERBATIM
    )
    add_custom_command(
      OUTPUT ${loghash}
      COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_GENERATE_PY} loghash-merge
              --output ${loghash} ${fw_loghash}
      DEPENDS ${fw_loghash} ${PBL_GENERATE_PY}
      WORKING_DIRECTORY ${PBL_BASE}
      VERBATIM
    )
    list(APPEND artifacts ${loghash})
  endif()

  if(PBL_PBPACK)
    list(APPEND artifacts ${PBL_PBPACK} ${PBL_LAYOUTS})
  endif()
  add_custom_target(pbl_firmware ALL DEPENDS ${artifacts})

  # The merged catalog is a target of its own, declared where its recipe is.
  get_property(pot GLOBAL PROPERTY PBL_POT_TARGET)
  if(pot)
    add_dependencies(pbl_firmware ${pot})
  endif()

  # --- Bundling -----------------------------------------------------------

  set(bundle_args --config ${PBL_DOTCONFIG} --firmware ${bin}
                  --board ${PBL_BOARD_NORMALIZED} --outdir ${PROJECT_BINARY_DIR}
                  --slot ${PBL_SLOT})
  if(PBL_PBPACK)
    list(APPEND bundle_args --resources ${PBL_PBPACK} --layouts ${PBL_LAYOUTS})
  endif()
  if(CONFIG_LOG_HASHED)
    list(APPEND bundle_args --loghash ${loghash})
  endif()
  add_custom_target(bundle
    COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_FIRMWARE_PY} bundle ${bundle_args}
    DEPENDS pbl_firmware
    WORKING_DIRECTORY ${PBL_BASE}
    COMMENT "Bundling firmware"
    VERBATIM
  )

  # --- QEMU flash images --------------------------------------------------

  if(CONFIG_QEMU)
    add_custom_target(qemu_image_micro
      COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_FIRMWARE_PY} qemu-image-micro
              --input ${hex} --output ${PROJECT_BINARY_DIR}/qemu_micro_flash.bin
      DEPENDS pbl_firmware
      WORKING_DIRECTORY ${PBL_BASE}
      VERBATIM
    )
    if(PBL_PBPACK)
      add_custom_target(qemu_image_spi
        COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_FIRMWARE_PY} qemu-image-spi
                --config ${PBL_DOTCONFIG} --pbpack ${PBL_PBPACK}
                --output ${PROJECT_BINARY_DIR}/qemu_spi_flash.bin
        DEPENDS pbl_firmware
        WORKING_DIRECTORY ${PBL_BASE}
        VERBATIM
      )
    endif()
  endif()
endfunction()
