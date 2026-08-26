# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from resources.types.resource_ball import ResourceBall
from waflib import Task, TaskGen


def get_font_keys_from_resource_ball(resource_ball_node):
    resource_ball = ResourceBall.load(resource_ball_node.abspath())
    definitions = (o.definition for o in resource_ball.resource_objects)

    font_keys = []
    for d in definitions:
        if d.type == "font":
            font_keys.append(d.name)
            font_keys.extend(d.aliases)

    return font_keys


class generate_font_header(Task.Task):
    def run(self):
        font_keys = get_font_keys_from_resource_ball(self.inputs[0])

        with open(self.outputs[0].abspath(), "w") as output_file:
            output_file.write("#pragma once\n\n")

            output_file.writelines(f'#define FONT_KEY_{key} "RESOURCE_ID_{key}"\n' for key in font_keys)

            # See PBL-9335. We removed this define as it's no longer a complete font. It looked the
            # same as Gothic 14, so going forward use that visual lookalike instead.
            output_file.write(
                '#define FONT_KEY_FONT_FALLBACK "RESOURCE_ID_GOTHIC_14"\n'
            )


class generate_font_table(Task.Task):
    def run(self):
        font_keys = get_font_keys_from_resource_ball(self.inputs[0])

        with open(self.outputs[0].abspath(), "w") as output_file:
            output_file.write("""
static const struct {
  const char *key_name;
  ResourceId resource_id;
  ResourceId extension_id;
} s_font_resource_keys[] = {
""")

            output_file.writelines(f"  {{ FONT_KEY_{key}, "
                    f"RESOURCE_ID_{key}, "
                    f"RESOURCE_ID_{key}_EXTENDED }},\n" for key in font_keys)

            output_file.write("};\n")


@TaskGen.feature("generate_fonts")
@TaskGen.before_method("process_source", "process_rule")
def process_generate_fonts(self):
    task = self.create_task(
        "generate_font_header", self.resource_ball, self.font_key_header
    )

    if self.font_key_table is not None:
        task = self.create_task(
            "generate_font_table", self.resource_ball, self.font_key_table
        )
