# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os

# binutils and memory_reports live in the SDK's common/tools, which sdk_paths
# puts on the path; importing it first is what makes them resolvable.
import sdk_paths  # noqa: F401
from binutils import size
from memory_reports import (
    app_appstore_resource_memory_error,
    app_memory_report,
    app_resource_memory_error,  # noqa: F401
    simple_memory_report,
)
from sdk_helpers import is_sdk_2x  # noqa: F401
from waflib import Logs, Task
from waflib.TaskGen import after_method, feature


class memory_usage_report(Task.Task):
    """
    Task class to print a memory usage report for the specified binary and resources, if any
    """

    def run(self):
        """
        This method executes when the memory usage report task runs
        :return: None
        """
        bin_type = self.bin_type
        platform = self.generator.env.PLATFORM_NAME

        bin_path = self.inputs[0].abspath()
        resources_path = self.inputs[1].abspath() if len(self.inputs) > 1 else None
        max_ram, max_resources, max_appstore_resources = self.max_sizes

        # Handle zero-size binaries (more common with packages)
        ram_size = sum(size(bin_path)) if size(bin_path) != 0 else 0

        resource_size = os.stat(resources_path).st_size if resources_path else None
        if resource_size and max_resources and max_appstore_resources:
            if resource_size > max_resources:
                Logs.pprint(
                    *app_appstore_resource_memory_error(
                        platform, resource_size, max_resources
                    )
                )
                return -1
            elif resource_size > max_appstore_resources:
                Logs.pprint(
                    *app_appstore_resource_memory_error(
                        platform, resource_size, max_appstore_resources
                    )
                )

        if max_ram:
            # resource_size and max_appstore_resources are optional
            free_ram = max_ram - ram_size
            Logs.pprint(
                *app_memory_report(
                    platform,
                    bin_type,
                    ram_size,
                    max_ram,
                    free_ram,
                    resource_size,
                    max_appstore_resources,
                )
            )
        else:
            # resource_size is optional
            Logs.pprint(*simple_memory_report(platform, ram_size, resource_size))


@feature("memory_usage")
@after_method("cprogram", "cstlib")
def generate_memory_usage_report(task_gen):
    """
    Generates and prints a report of the project's memory usage (binary + resources, if applicable).

    Keyword arguments:
    app -- The path to the app elf file, if this is an app being evaluated
    worker - The path to the worker elf file, if this is a worker being evaluated
    lib - The path to the library archive file, if this is a library being evaluated
    resources - The path to the resource pack or resource ball, if resources exist for this bin_type

    :param task_gen: the task generator instance
    :return: None
    """
    app, worker, lib, resources = (
        getattr(task_gen, attr, None) for attr in ("app", "worker", "lib", "resources")
    )

    max_resources = task_gen.env.PLATFORM["MAX_RESOURCES_SIZE"]
    max_resources_appstore = task_gen.env.PLATFORM["MAX_RESOURCES_SIZE_APPSTORE"]
    app_max_ram = task_gen.env.PLATFORM["MAX_APP_MEMORY_SIZE"] if app else None
    worker_max_ram = task_gen.env.PLATFORM["MAX_WORKER_MEMORY_SIZE"] if worker else None

    if app:
        app_node = task_gen.path.find_or_declare(app)
        app_task = task_gen.create_task("memory_usage_report", [app_node, resources])
        app_task.max_sizes = (app_max_ram, max_resources, max_resources_appstore)
        app_task.bin_type = "app"
    if worker:
        worker_node = task_gen.path.find_or_declare(worker)
        worker_task = task_gen.create_task("memory_usage_report", worker_node)
        worker_task.max_sizes = (worker_max_ram, None, None)
        worker_task.bin_type = "worker"
    if lib:
        lib_task = task_gen.create_task(
            "memory_usage_report",
            [task_gen.to_nodes(lib)[0], task_gen.to_nodes(resources)[0]],
        )
        lib_task.max_sizes = (None, None, None)
        lib_task.bin_type = "lib"
