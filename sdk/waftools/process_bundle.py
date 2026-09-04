# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os

import mkbundle
from pebble_package import LibraryPackage
from process_elf import generate_bin_file
from resources.types.resource_ball import ResourceBall
from waflib import Task
from waflib.TaskGen import feature


class lib_package(Task.Task):
    """
    Task class to generate a library bundle for distribution
    """

    def run(self):
        """
        This method executes when the package task runs
        :return: N/A
        """
        bld = self.generator.bld
        build_dir = bld.bldnode

        includes = {
            include.path_from(build_dir.find_node("include")): include.abspath()
            for include in getattr(self, "includes", [])
        }
        binaries = {
            binary.path_from(build_dir): binary.abspath()
            for binary in getattr(self, "binaries", [])
        }
        js = {
            js.path_from(build_dir.find_node("js")): js.abspath()
            for js in getattr(self, "js", [])
        }

        resource_definitions = []
        for resball in getattr(self, "resources", []):
            resource_definitions.extend(
                ResourceBall.load(resball.abspath()).get_all_declarations()
            )

        reso_list = []
        for definition in resource_definitions:
            if definition.target_platforms:
                platforms = list(
                    set(definition.target_platforms) & set(bld.env.TARGET_PLATFORMS)
                )
            else:
                platforms = bld.env.TARGET_PLATFORMS
            for platform in platforms:
                platform_path = build_dir.find_node(
                    bld.all_envs[platform].BUILD_DIR
                ).relpath()
                reso_list.append(
                    build_dir.find_node(
                        "{}.{}.reso".format(
                            os.path.join(
                                platform_path,
                                bld.path.find_node(definition.sources[0]).relpath(),
                            ),
                            str(definition.name),
                        )
                    )
                )
        resources = {
            os.path.join(
                resource.path_from(build_dir).split("/", 1)[0],
                resource.path_from(build_dir).split("/", 3)[3],
            ): resource.abspath()
            for resource in reso_list
        }

        package = LibraryPackage(self.outputs[0].abspath())
        package.add_files(
            includes=includes, binaries=binaries, resources=resources, js=js
        )
        package.pack()


class app_bundle(Task.Task):
    """
    Task class to generate an app bundle for distribution
    """

    def run(self):
        """
        This method executes when the bundle task runs
        :return: N/A
        """
        binaries = self.bin_files
        js_files = self.js_files
        outfile = self.outputs[0].abspath()

        mkbundle.make_watchapp_bundle(
            timestamp=self.generator.bld.env.TIMESTAMP,
            appinfo=self.generator.bld.path.get_bld()
            .find_node("appinfo.json")
            .abspath(),
            binaries=binaries,
            js=[js_file.abspath() for js_file in js_files],
            outfile=outfile,
        )


@feature("package")
def make_lib_bundle(task_gen):
    """
    Bundle the build artifacts into a distributable library package.

    Keyword arguments:
    js -- A list of javascript files to package into the resulting bundle
    includes -- A list of header files to package into library bundle

    :param task_gen: the task generator instance
    :return: None
    """
    js = task_gen.to_nodes(getattr(task_gen, "js", []))
    includes = task_gen.to_nodes(getattr(task_gen, "includes", []))
    resources = []
    binaries = []

    for platform in task_gen.bld.env.TARGET_PLATFORMS:
        bld_dir = task_gen.path.get_bld().find_or_declare(platform)
        env = task_gen.bld.all_envs[platform]

        resources.append(env.PROJECT_RESBALL)

        project_name = env.PROJECT_INFO["name"]
        if project_name.startswith("@"):
            scoped_name = project_name.rsplit("/", 1)
            binaries.append(
                bld_dir.find_or_declare(str(scoped_name[0])).find_or_declare(
                    f"lib{scoped_name[1]}.a"
                )
            )
        else:
            binaries.append(bld_dir.find_or_declare(f"lib{project_name}.a"))

    task = task_gen.create_task(
        "lib_package", [], task_gen.bld.path.make_node(task_gen.bld.env.BUNDLE_NAME)
    )
    task.js = js
    task.includes = includes
    task.resources = resources
    task.binaries = binaries
    task.dep_nodes = js + includes + resources + binaries


# PBL-40925 Use pebble_package.py instead of mkbundle.py
@feature("bundle")
def make_pbl_bundle(task_gen):
    """
    Bundle the build artifacts into a distributable package.

    Keyword arguments:
    js -- A list of javascript files to package into the resulting bundle
    binaries -- A list of the binaries for each platform to include in the bundle

    :param task_gen: the task generator instance
    :return: None
    """
    bin_files = []
    bundle_sources = []
    js_files = getattr(task_gen, "js", [])

    has_pkjs = bool(getattr(task_gen, "js", False))
    if has_pkjs:
        bundle_sources.extend(task_gen.to_nodes(task_gen.js))

    cached_env = task_gen.bld.env

    binaries = task_gen.binaries

    for binary in binaries:
        task_gen.bld.env = task_gen.bld.all_envs[binary["platform"]]

        platform_build_node = task_gen.bld.path.find_or_declare(
            task_gen.bld.env.BUILD_DIR
        )

        app_elf_file = task_gen.bld.path.get_bld().make_node(binary["app_elf"])
        if app_elf_file is None:
            raise RuntimeError("Must specify elf argument to bundle")

        worker_bin_file = None
        if "worker_elf" in binary:
            worker_elf_file = task_gen.bld.path.get_bld().make_node(
                binary["worker_elf"]
            )
            app_bin_file = generate_bin_file(
                task_gen, "app", app_elf_file, has_pkjs, has_worker=True
            )
            worker_bin_file = generate_bin_file(
                task_gen, "worker", worker_elf_file, has_pkjs, has_worker=True
            )
            bundle_sources.append(worker_bin_file)
        else:
            app_bin_file = generate_bin_file(
                task_gen, "app", app_elf_file, has_pkjs, has_worker=False
            )

        resources_pack = platform_build_node.make_node("app_resources.pbpack")

        bundle_sources.extend([app_bin_file, resources_pack])
        bin_files.append(
            {
                "watchapp": app_bin_file.abspath(),
                "resources": resources_pack.abspath(),
                "worker_bin": worker_bin_file.abspath() if worker_bin_file else None,
                "sdk_version": {
                    "major": task_gen.bld.env.SDK_VERSION_MAJOR,
                    "minor": task_gen.bld.env.SDK_VERSION_MINOR,
                },
                "subfolder": task_gen.bld.env.BUNDLE_BIN_DIR,
            }
        )
    task_gen.bld.env = cached_env

    bundle_output = task_gen.bld.path.get_bld().make_node(task_gen.bld.env.BUNDLE_NAME)
    task = task_gen.create_task("app_bundle", [], bundle_output)
    task.bin_files = bin_files
    task.js_files = js_files
    task.dep_nodes = bundle_sources
