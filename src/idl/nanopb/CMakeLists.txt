# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The protobuf schemas the firmware speaks, compiled by nanopb.

find_program(NANOPB_GENERATOR nanopb_generator REQUIRED)

file(GLOB protos CONFIGURE_DEPENDS *.proto)

set(sources "")
set(headers "")
foreach(proto ${protos})
  get_filename_component(name ${proto} NAME_WE)
  set(generated_c ${CMAKE_CURRENT_BINARY_DIR}/${name}.pb.c)
  set(generated_h ${CMAKE_CURRENT_BINARY_DIR}/${name}.pb.h)
  # .options files next to a schema tune the generated C types.
  file(GLOB options CONFIGURE_DEPENDS ${name}.options)
  add_custom_command(
    OUTPUT ${generated_c} ${generated_h}
    COMMAND ${NANOPB_GENERATOR} -I ${CMAKE_CURRENT_SOURCE_DIR}
            -D ${CMAKE_CURRENT_BINARY_DIR} ${proto}
    # A schema may import any of its neighbours.
    DEPENDS ${protos} ${options}
    COMMENT "Generating ${name}.pb.c"
    VERBATIM
  )
  list(APPEND sources ${generated_c})
  list(APPEND headers ${generated_h})
endforeach()

add_custom_target(pbl_proto_schemas DEPENDS ${headers})
add_dependencies(pbl_generated_headers pbl_proto_schemas)

pbl_library_named(proto_schemas)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})
