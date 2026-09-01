# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from waflib import Task, TaskGen

from resources import generators


class generate_pbpack(Task.Task):
    def run(self):
        generators.build_pbpack(
            self.inputs[0].abspath(), self.outputs[0].abspath(), self.is_system
        )


@TaskGen.feature("generate_pbpack")
@TaskGen.before_method("process_source", "process_rule")
def process_generate_pbpack(self):
    task = self.create_task("generate_pbpack", self.resource_ball, self.pbpack_target)
    task.is_system = getattr(self, "is_system", False)
