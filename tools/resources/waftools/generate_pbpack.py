# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from pbpack import ResourcePack
from resources.types.resource_ball import ResourceBall
from resources.types.resource_definition import StorageType
from waflib import Task, TaskGen


class generate_pbpack(Task.Task):
    def run(self):
        resource_ball = ResourceBall.load(self.inputs[0].abspath())
        resource_objects = [
            reso
            for reso in resource_ball.resource_objects
            if reso.definition.storage == StorageType.pbpack
        ]

        pack = ResourcePack(self.is_system)

        for r in resource_objects:
            pack.add_resource(r.data)

        with open(self.outputs[0].abspath(), "wb") as f:
            pack.serialize(f)


@TaskGen.feature("generate_pbpack")
@TaskGen.before_method("process_source", "process_rule")
def process_generate_pbpack(self):
    task = self.create_task("generate_pbpack", self.resource_ball, self.pbpack_target)
    task.is_system = getattr(self, "is_system", False)
