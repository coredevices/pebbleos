# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import json
import os
from string import Template

from sdk_helpers import (  # noqa: F401
    find_sdk_component,
    get_node_from_abspath,
    process_package,
)
from waflib import Context, Logs, Task
from waflib.Errors import WafError
from waflib.TaskGen import before_method, feature


@feature("js")
@before_method("make_pbl_bundle", "make_lib_bundle")
def process_js(task_gen):
    """
    Merge the JS source files into a single JS file if enableMultiJS is set to 'true', otherwise,
    skip JS processing

    Keyword arguments:
    js -- A list of JS files to process for the build

    :param task_gen: the task generator instance
    :return: N/A
    """
    # Skip JS handling if there are no JS files
    js_nodes = task_gen.to_nodes(getattr(task_gen, "js", []))
    if not js_nodes:
        return

    # Create JS merge task if the project specifies "enableMultiJS: true"
    if task_gen.env.PROJECT_INFO.get("enableMultiJS", False):
        target_js = task_gen.bld.path.get_bld().make_node("pebble-js-app.js")
        target_js_map = target_js.change_ext(".js.map")
        task_gen.js = [target_js, target_js_map]

        merge_task = task_gen.create_task(
            "merge_js", src=js_nodes, tgt=[target_js, target_js_map]
        )
        merge_task.js_entry_file = task_gen.js_entry_file
        merge_task.js_build_type = "pkjs"
        merge_task.js_source_map_config = {"sourceMapFilename": target_js_map.name}
        return

    # Check for pebble-js-app.js if developer does not specify "enableMultiJS: true" in
    # the project
    if task_gen.env.BUILD_TYPE != "lib":
        for node in js_nodes:
            if "pebble-js-app.js" in node.abspath():
                break
        else:
            Logs.pprint(
                "CYAN",
                "WARNING: enableMultiJS is not enabled for this project and "
                "pebble-js-app.js does not exist",
            )

    # For apps without multiJS enabled and libs, copy JS files from src folder to build folder,
    # skipping any files already in the build folder
    js_nodes_to_copy = [js_node for js_node in js_nodes if not js_node.is_bld()]
    if not js_nodes_to_copy:
        task_gen.js = js_nodes
        return

    target_nodes = []
    for js in js_nodes_to_copy:
        if js.is_child_of(task_gen.bld.path.find_node("src")):
            js_path = js.path_from(task_gen.bld.path.find_node("src"))
        else:
            js_path = os.path.abspath(js.path_from(task_gen.bld.path))
        target_node = task_gen.bld.path.get_bld().make_node(js_path)
        target_node.parent.mkdir()
        target_nodes.append(target_node)
    task_gen.js = target_nodes + list(set(js_nodes) - set(js_nodes_to_copy))
    task_gen.create_task("copy_js", src=js_nodes_to_copy, tgt=target_nodes)


class copy_js(Task.Task):
    """
    Task class for copying source JS files to a target location
    """

    def run(self):
        """
        This method executes when the JS copy task runs
        :return: N/A
        """
        bld = self.generator.bld

        if len(self.inputs) != len(self.outputs):
            bld.fatal(
                f"Number of input JS files ({len(self.inputs)}) does not match number of target JS files ({len(self.outputs)})"
            )

        for i in range(len(self.inputs)):
            bld.cmd_and_log(
                f'cp "{self.inputs[i].abspath()}" "{self.outputs[i].abspath()}"',
                quiet=Context.BOTH,
            )


class merge_js(Task.Task):
    """
    Task class for merging all specified JS files into one `pebble-js-app.js` file
    """

    def run(self):
        """
        This method executes when the JS merge task runs
        :return: N/A
        """
        bld = self.generator.bld
        js_build_type = self.js_build_type

        # Check for a valid JS entry point among JS files
        js_nodes = self.inputs
        entry_point = bld.path.find_resource(self.js_entry_file)
        if entry_point not in js_nodes:
            bld.fatal(
                f"\n\nJS entry file '{self.js_entry_file}' not found in JS source files '{js_nodes}'. We expect to find "
                "a javascript file here that we will execute directly when your app launches."
                "\n\nIf you are an advanced user, you can supply the 'js_entry_file' "
                "parameter to 'pbl_bundle' in your wscript to change the default entry point."
                " Note that doing this will break CloudPebble compatibility."
            )
        target_js = self.outputs[0]

        entry = [entry_point.abspath()]

        if js_build_type == "pkjs":
            # NOTE: The order is critical here.
            # _pkjs_shared_additions.js MUST be the first in the `entry` array!
            entry.insert(0, "_pkjs_shared_additions.js")
        common_node = bld.root.find_node(self.generator.env.PEBBLE_SDK_COMMON)
        tools_webpack_node = common_node.find_node("tools").find_node("webpack")
        webpack_config_template_node = tools_webpack_node.find_node(
            "webpack-config.js.pytemplate"
        )
        with open(webpack_config_template_node.abspath()) as f:
            webpack_config_template_content = f.read()

        search_paths = [
            common_node.find_node("include").abspath(),
            tools_webpack_node.abspath(),
            bld.root.find_node(self.generator.env.NODE_PATH).abspath(),
            bld.path.get_bld().make_node("js").abspath(),
        ]

        pebble_packages = [
            str(lib["name"]) for lib in bld.env.LIB_JSON if "pebble" in lib
        ]
        aliases = {lib: f"{lib}/dist/js" for lib in pebble_packages}

        info_json_file = bld.path.find_node("package.json") or bld.path.find_node(
            "appinfo.json"
        )
        if info_json_file:
            aliases.update({"app_package.json": info_json_file.abspath()})

        config_file = bld.path.get_bld().make_node(
            f"webpack/{js_build_type}/webpack.config.js"
        )
        config_file.parent.mkdir()
        with open(config_file.abspath(), "w") as f:
            m = {
                "IS_SANDBOX": bool(self.env.SANDBOX),
                "ENTRY_FILENAMES": entry,
                "OUTPUT_PATH": target_js.parent.path_from(bld.path),
                "OUTPUT_FILENAME": target_js.name,
                "RESOLVE_ROOTS": search_paths,
                "RESOLVE_ALIASES": aliases,
                "SOURCE_MAP_CONFIG": getattr(self, "js_source_map_config", None),
            }
            f.write(
                Template(webpack_config_template_content).substitute(
                    {k: json.dumps(m[k], separators=(",\n", ": ")) for k in m}
                )
            )

        cmd = self.generator.env.WEBPACK + [
            "--config",
            config_file.path_from(bld.path),
            "--display-modules",
        ]
        try:
            out = bld.cmd_and_log(cmd, quiet=Context.BOTH, output=Context.STDOUT)
        except WafError as e:
            bld.fatal(f"JS bundling failed\n{e.stdout}\n{e.stderr}")
        else:
            if self.env.VERBOSE > 0:
                Logs.pprint("WHITE", out)

            # Strip local filesystem paths from the sourcemap to avoid
            # leaking private info (username, directory structure) in .pbw
            if len(self.outputs) > 1:
                target_map = self.outputs[1]
                map_path = target_map.abspath()
                if os.path.exists(map_path):
                    with open(map_path, "r") as f:
                        map_content = json.load(f)

                    project_path = bld.path.abspath()

                    def _clean_path(path):
                        if not os.path.isabs(path):
                            return path
                        if path.startswith(project_path):
                            return os.path.relpath(path, project_path)
                        return os.path.basename(path)

                    if "sources" in map_content:
                        map_content["sources"] = [
                            _clean_path(s) for s in map_content["sources"]
                        ]

                    with open(map_path, "w") as f:
                        json.dump(map_content, f, separators=(",", ":"))
