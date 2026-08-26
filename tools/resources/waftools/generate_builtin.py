# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import generate_c_byte_array
from resources.types.resource_ball import ResourceBall
from resources.types.resource_definition import StorageType
from waflib import Task, TaskGen


class generate_builtin(Task.Task):
    def run(self):
        resource_ball = ResourceBall.load(self.inputs[0].abspath())
        resource_objects = [
            reso
            for reso in resource_ball.resource_objects
            if reso.definition.storage == StorageType.builtin
        ]

        with open(self.outputs[0].abspath(), "w") as f:
            fw_bld_node = self.generator.bld.bldnode.find_node("src/fw")
            f.write(
                f'#include "{self.resource_id_header.path_from(fw_bld_node)}"\n'
            )
            f.write('#include "resource/resource_storage.h"\n')
            f.write('#include "resource/resource_storage_builtin.h"\n\n')

            def var_name(reso):
                return f"{reso.definition.name}_builtin_bytes"

            # Write the blobs of data:
            for reso in resource_objects:
                # some resources require 8-byte aligned addresses
                # to simplify the handling we align all resources
                f.write("__attribute__ ((aligned (8)))\n")
                generate_c_byte_array.write(f, reso.data, var_name(reso))

            f.write("\n")

            f.write(
                f"const uint32_t g_num_builtin_resources = {len(resource_objects)};\n"
            )
            f.write("const BuiltInResourceData g_builtin_resources[] = {\n")

            for reso in resource_objects:
                f.write(
                    "  {{ RESOURCE_ID_{resource_id}, {var_name}, sizeof({var_name}) }},\n".format(
                        resource_id=reso.definition.name, var_name=var_name(reso)
                    )
                )

            f.write("};\n")


@TaskGen.feature("generate_builtin")
@TaskGen.before_method("process_source", "process_rule")
def process_generate_builtin(self):
    task = self.create_task("generate_builtin", self.resource_ball, self.builtin_target)
    task.resource_id_header = self.resource_id_header
