# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from sdk_helpers import get_node_from_abspath
from waflib import Context, Task
from waflib.TaskGen import before_method, feature


@feature("headers")
@before_method("make_lib_bundle")
def process_headers(task_gen):
    """
    Process all of the headers specified in the wscript file, as well as the headers generated
    during the build process for the resource ids and message keys, as needed.

    Keyword arguments:
    includes -- A list of header files to copy

    :param task_gen: the task generator instance
    :return: None
    """
    header_nodes = task_gen.to_nodes(task_gen.includes)

    for platform in task_gen.env.TARGET_PLATFORMS:
        env = task_gen.bld.all_envs[platform]
        header_nodes.append(
            get_node_from_abspath(task_gen.bld, env["RESOURCE_ID_HEADER"])
        )

    # Add .h file containing app message keys
    if "MESSAGE_KEYS_HEADER" in dict(task_gen.env):
        header_nodes.append(
            get_node_from_abspath(task_gen.bld, task_gen.env["MESSAGE_KEYS_HEADER"])
        )

    # Copy header files to build/include/<libname> to provide naming collision protection in
    # #includes
    lib_name = str(task_gen.env.PROJECT_INFO["name"])
    lib_include_node = (
        task_gen.bld.path.get_bld().make_node("include").make_node(lib_name)
    )
    target_nodes = []
    for header in header_nodes:
        base_node = (
            task_gen.bld.path.get_bld()
            if header.is_child_of(task_gen.bld.path.get_bld())
            else task_gen.bld.path
        )
        if header.is_child_of(base_node.find_node("include")):
            header_path = header.path_from(base_node.find_node("include"))
        else:
            header_path = header.path_from(base_node)
        target_node = lib_include_node.make_node(header_path)
        target_node.parent.mkdir()
        target_nodes.append(target_node)
    task_gen.includes = target_nodes
    task_gen.create_task("copy_headers", src=header_nodes, tgt=target_nodes)


class copy_headers(Task.Task):
    """
    Task class to copy specified headers from a source location to a target location
    """

    def run(self):
        """
        This method executes when the copy headers task runs
        :return: N/A
        """
        bld = self.generator.bld

        if len(self.inputs) != len(self.outputs):
            bld.fatal(
                f"Number of input headers ({len(self.inputs)}) does not match number of target headers ({len(self.outputs)})"
            )

        for i in range(len(self.inputs)):
            bld.cmd_and_log(
                f'cp "{self.inputs[i].abspath()}" "{self.outputs[i].abspath()}"',
                quiet=Context.BOTH,
            )
