# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import json
from subprocess import PIPE, Popen

from resources.resource_map.resource_generator import ResourceGenerator
from resources.types.resource_object import ResourceObject


class JsResourceGenerator(ResourceGenerator):
    type = "js"

    @staticmethod
    def generate_object(task, definition):
        node_command = task.generator.env.NODE

        # FIXME:
        # We use this file from both our firmware build (waf version 1.8.x) and from our sdk build
        # (waf version 1.7.x) and the functionality of Configure::find_program has changed between
        # these two release. On waf 1.7.x, it returns a list, where in waf 1.8.x it gives us a
        # string. Flatten it into a string regardless of version. To fix this we should just
        # update our SDK waf as it's redistributed alongside this file.
        if len(node_command) == 1:
            node_command = node_command[0]

        script_node = task.generator.env.JS_TOOLING_SCRIPT
        bytecode = task.outputs[0].change_ext(".bytecode")
        bytecode.parent.mkdir()
        memory_usage_output = task.generator.bld.path.get_bld().make_node(
            f"{task.generator.env.PLATFORM_NAME}_snapshot_size.json"
        )

        cmd = [
            node_command,
            script_node.abspath(),
            task.inputs[0].abspath(),
            bytecode.abspath(),
            memory_usage_output.abspath(),
        ]

        proc = Popen(cmd, stdout=PIPE, stderr=PIPE, encoding="utf-8")
        out, err = proc.communicate()

        if proc.returncode != 0:
            task.generator.bld.fatal(
                f"JS compilation failed.\nSTDOUT: {out}\nSTDERR: {err}"
            )

        # Save bytecode computed size and max size for SDK memory report
        if task.generator.env.PLATFORM_NAME in task.generator.bld.all_envs:
            env = task.generator.bld.all_envs[task.generator.env.PLATFORM_NAME]

            with open(memory_usage_output.abspath(), "r") as f:
                content = json.load(f)
            env.SNAPSHOT_SIZE = content["size"]
            env.SNAPSHOT_MAX = content["max"]

        with open(bytecode.abspath(), "rb") as f:
            data = f.read()
        return ResourceObject(definition, data)
