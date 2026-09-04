# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from waflib import Task, TaskGen

from resources import generators


class generate_resource_id_header(Task.Task):
    def run(self):
        self.outputs[0].parent.mkdir()
        try:
            generators.build_resource_id_header(
                self.inputs[0].abspath(),
                self.outputs[0].abspath(),
                use_extern=getattr(self, "use_extern", False),
                use_define=getattr(self, "use_define", False),
                published_media=getattr(self, "published_media", []),
            )
        except ValueError as e:
            self.generator.bld.fatal(
                f"{e} Check your generate_resource_id_header arguments."
            )


class generate_resource_id_definitions(Task.Task):
    def run(self):
        self.outputs[0].parent.mkdir()
        generators.build_resource_id_definitions(
            self.inputs[0].abspath(),
            self.outputs[0].abspath(),
            published_media=getattr(self, "published_media", []),
        )


@TaskGen.feature("generate_resource_id_header")
@TaskGen.before_method("process_source", "process_rule")
def process_generate_resource_id_header(self):
    task = self.create_task(
        "generate_resource_id_header",
        self.resource_ball,
        self.resource_id_header_target,
    )
    task.use_extern = getattr(self, "use_extern", False)
    task.use_define = getattr(self, "use_define", False)
    task.published_media = getattr(self, "published_media", [])


@TaskGen.feature("generate_resource_id_definitions")
@TaskGen.before_method("process_source", "process_rule")
def create_resource_id_definitions_task(self):
    task = self.create_task(
        "generate_resource_id_definitions",
        self.resource_ball,
        self.resource_id_definitions_target,
    )
    task.published_media = getattr(self, "published_media", [])
