# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import json
import os
import sys

# waf loads this module from waflib/extras, which contains the file sdk_paths
extras_dir = os.path.dirname(os.path.abspath(__file__))
if extras_dir not in sys.path:
    sys.path.insert(0, extras_dir)

import sdk_paths  # noqa: F401
from generate_appinfo import generate_appinfo_c
from process_sdk_resources import generate_resources
from sdk_helpers import (
    configure_libraries,
    configure_platform,
    get_target_platforms,
    truncate_to_32_bytes,
    validate_message_keys_object,
)
from waflib.Configure import conf


def _extract_project_info(conf, info_json, json_filename):
    """
    Extract project info from "pebble" object, or copy configuration directly if read from
    appinfo.json
    :param conf: the ConfigurationContext
    :param info_json: the JSON blob contained in appinfo.json or package.json
    :return: JSON blob containing project information for build
    """
    if "pebble" in info_json:
        project_info = info_json["pebble"]
        validate_message_keys_object(conf, project_info, "package.json")

        project_info["name"] = info_json["name"]
        project_info["shortName"] = project_info["longName"] = project_info[
            "displayName"
        ]

        # Validate version specified in package.json to avoid issues later
        if not info_json["version"]:
            conf.fatal("Project is missing a version")
        version = _validate_version(conf, info_json["version"])
        project_info["versionLabel"] = version

        if isinstance(info_json["author"], str):
            project_info["companyName"] = (
                info_json["author"].split("(", 1)[0].split("<", 1)[0].strip()
            )
        elif isinstance(info_json["author"], dict) and "name" in info_json["author"]:
            project_info["companyName"] = info_json["author"]["name"]
        else:
            conf.fatal("Missing author name in project info")
    elif "package.json" == json_filename:
        try:
            with open(
                conf.path.get_src().find_node("appinfo.json").abspath(), "r"
            ) as f:
                info_json = json.load(f)
        except AttributeError:
            conf.fatal(
                "Could not find Pebble project info in package.json and no appinfo.json file"
                " exists"
            )
        project_info = info_json
        validate_message_keys_object(conf, project_info, "appinfo.json")
    else:
        project_info = info_json
        validate_message_keys_object(conf, project_info, "appinfo.json")
    return project_info


def _generate_appinfo_c_file(task):
    """
    This Task generates the appinfo.auto.c file that is included in binary metadata
    :param task: the instance of this task
    :return: N/A
    """
    info_json = dict(getattr(task.generator.env, task.vars[0]))
    info_json["shortName"] = truncate_to_32_bytes(info_json["shortName"])
    info_json["companyName"] = truncate_to_32_bytes(info_json["companyName"])
    current_platform = task.generator.env.PLATFORM_NAME
    generate_appinfo_c(info_json, task.outputs[0].abspath(), current_platform)


def _write_appinfo_json_file(task):
    """
    This task writes the content of the PROJECT_INFO environment variable to appinfo.json in the
    build directory. PROJECT_INFO is generated from reading in either a package.json file or an
    old-style appinfo.json file.
    :param task: the task instance
    :return: None
    """
    appinfo = dict(getattr(task.generator.env, task.vars[0]))
    capabilities = appinfo.get("capabilities", [])

    for lib in dict(task.generator.env).get("LIB_JSON", []):
        if "pebble" in lib:
            capabilities.extend(lib["pebble"].get("capabilities", []))
    appinfo["capabilities"] = list(set(capabilities))

    for key in task.env.BLOCK_MESSAGE_KEYS:
        del appinfo["appKeys"][key]

    if appinfo:
        with open(task.outputs[0].abspath(), "w") as f:
            json.dump(appinfo, f, indent=4)
    else:
        task.generator.bld.fatal(
            "Unable to find project info to populate appinfo.json file with"
        )


def _validate_version(ctx, original_version):
    """
    Validates the format of the version field in an app's project info
    to be compatible with the Pebble FW.
    :param ctx: the ConfigureContext object
    :param version: the version provided in project info (package.json/appinfo.json)
    :return: a MAJOR.MINOR version that is acceptable for Pebble FW
    """
    version = original_version.split(".")
    if len(version) > 3:
        ctx.fatal(
            "App versions must be of the format MAJOR or MAJOR.MINOR or MAJOR.MINOR.PATCH. "
            f"An invalid version of {original_version} was specified for the app. Try {version[0]}.{version[1]}.{version[2]} instead"
        )
    elif not (0 <= int(version[0]) <= 255):
        ctx.fatal(
            f"An invalid or out of range value of {version[0]} was specified for the major version of "
            "the app. The valid range is 0-255."
        )
    elif not (0 <= int(version[1]) <= 255):
        ctx.fatal(
            f"An invalid or out of range value of {version[1]} was specified for the minor version of "
            "the app. The valid range is 0-255."
        )
    # note: version[2] does not reach the Pebble FW (it is stripped off by generate_appinfo)
    # so has no upper limit. Just make sure it's an int.
    if len(version) == 3:
        int(version[2])

    return ".".join(version[:3])


def options(opt):
    """
    Specify the options available when invoking waf; uses OptParse
    :param opt: the OptionContext object
    :return: N/A
    """
    opt.load("pebble_sdk_common")
    opt.add_option(
        "-t",
        "--timestamp",
        dest="timestamp",
        help="Use a specific timestamp to label this package (ie, your repository's "
        "last commit time), defaults to time of build",
    )


def configure(conf):
    """
    Configure the build using information obtained from a JSON file
    :param conf: the ConfigureContext object
    :return: N/A
    """
    conf.load("pebble_sdk_common")

    # This overrides the default config in pebble_sdk_common.py
    if conf.options.timestamp:
        conf.env.TIMESTAMP = conf.options.timestamp
        conf.env.BUNDLE_NAME = f"app_{conf.env.TIMESTAMP}.pbw"
    else:
        conf.env.BUNDLE_NAME = f"{conf.path.name}.pbw"

    # Read in package.json for environment configuration, or fallback to appinfo.json for older
    # projects
    info_json_node = conf.path.get_src().find_node(
        "package.json"
    ) or conf.path.get_src().find_node("appinfo.json")
    if info_json_node is None:
        conf.fatal("Could not find package.json")
    with open(info_json_node.abspath(), "r") as f:
        info_json = json.load(f)

    project_info = _extract_project_info(conf, info_json, info_json_node.name)

    conf.env.PROJECT_INFO = project_info
    project_type = project_info.get("projectType", None)
    if project_type == "moddable":
        conf.env.BUILD_TYPE = "moddable"
    else:
        conf.env.BUILD_TYPE = "app"

    if getattr(conf.env.PROJECT_INFO, "enableMultiJS", False) and not conf.env.WEBPACK:
        conf.fatal(
            f"'enableMultiJS' is set to true, but unable to locate webpack module at {conf.env.NODE_PATH} "
            "Please set enableMultiJS to false, or reinstall the SDK."
        )

    if "resources" in project_info and "media" in project_info["resources"]:
        conf.env.RESOURCES_JSON = project_info["resources"]["media"]

        if "publishedMedia" in project_info["resources"]:
            conf.env.PUBLISHED_MEDIA_JSON = project_info["resources"]["publishedMedia"]

    conf.env.REQUESTED_PLATFORMS = project_info.get("targetPlatforms", [])
    conf.env.LIB_DIR = "node_modules"

    get_target_platforms(conf)

    # With new-style projects, check for libraries specified in package.json
    if "dependencies" in info_json:
        configure_libraries(conf, info_json["dependencies"])
    conf.load("process_message_keys")

    # base_env is set to a shallow copy of the current ConfigSet for this ConfigureContext
    base_env = conf.env

    for platform in conf.env.TARGET_PLATFORMS:
        # Create a deep copy of the `base_env` ConfigSet and set conf.env to a shallow copy of
        # the resultant ConfigSet
        conf.setenv(platform, base_env)
        configure_platform(conf, platform)

    # conf.env is set back to a shallow copy of the default ConfigSet stored in conf.all_envs['']
    conf.setenv("")


def build(bld):
    """
    This method is invoked from a project's wscript with the `ctx.load('pebble_sdk')` call and
    sets up all of the task generators for the SDK. After all of the build methods have run,
    the configured task generators will run, generating build tasks and managing dependencies.
    See https://waf.io/book/#_task_generators for more details on task generator setup.
    :param bld: the BuildContext object
    :return: N/A
    """
    bld.load("pebble_sdk_common")

    # cached_env is set to a shallow copy of the current ConfigSet for this BuildContext
    cached_env = bld.env

    for platform in bld.env.TARGET_PLATFORMS:
        # bld.env is set to a shallow copy of the ConfigSet labeled <platform>
        bld.env = bld.all_envs[platform]

        # Set the build group (set of TaskGens) to the group labeled <platform>
        if bld.env.USE_GROUPS:
            bld.set_group(bld.env.PLATFORM_NAME)

        # Generate an appinfo file specific to the current platform
        build_node = bld.path.get_bld().make_node(bld.env.BUILD_DIR)
        bld(
            rule=_generate_appinfo_c_file,
            target=build_node.make_node("appinfo.auto.c"),
            vars=["PROJECT_INFO"],
        )

        # Generate an appinfo.json file for the current platform to bundle in a PBW
        bld(
            rule=_write_appinfo_json_file,
            target=bld.path.get_bld().make_node("appinfo.json"),
            vars=["PROJECT_INFO"],
        )

        # Generate resources specific to the current platform
        resource_node = None
        if bld.env.RESOURCES_JSON:
            try:
                resource_node = bld.path.find_node("resources")
            except AttributeError:
                bld.fatal("Unable to locate resources at resources/")

        resource_path = resource_node.path_from(bld.path) if resource_node else None
        generate_resources(bld, resource_path)

    # bld.env is set back to a shallow copy of the original ConfigSet that was set when this `build`
    # method was invoked
    bld.env = cached_env


@conf
def pbl_program(self, *k, **kw):
    """
    This method is bound to the build context and is called by specifying `bld.pbl_program()`. We
    set the custom features `c`, `cprogram` and `pebble_cprogram` to run when this method is
    invoked.
    :param self: the BuildContext object
    :param k: none expected
    :param kw:
        source - the source C files to be built and linked
        target - the destination binary file for the compiled source
    :return: a task generator instance with keyword arguments specified
    """
    kw["bin_type"] = "app"
    kw["features"] = "c cprogram pebble_cprogram memory_usage"
    kw["app"] = kw["target"]
    kw["resources"] = self.path.find_or_declare(self.env.BUILD_DIR).make_node(
        "app_resources.pbpack"
    )
    return self(*k, **kw)


@conf
def pbl_worker(self, *k, **kw):
    """
    This method is bound to the build context and is called by specifying `bld.pbl_worker()`. We set
    the custom features `c`, `cprogram` and `pebble_cprogram` to run when this method is invoked.
    :param self: the BuildContext object
    :param k: none expected
    :param kw:
        source - the source C files to be built and linked
        target - the destination binary file for the compiled source
    :return: a task generator instance with keyword arguments specified
    """
    kw["bin_type"] = "worker"
    kw["features"] = "c cprogram pebble_cprogram memory_usage"
    kw["worker"] = kw["target"]
    return self(*k, **kw)
