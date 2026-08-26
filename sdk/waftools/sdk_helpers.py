# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import json
import os
import struct

from pebble_package import LibraryPackage
from pebble_sdk_platform import pebble_platforms
from pebble_sdk_version import set_env_sdk_version
from resources.types.resource_object import ResourceObject
from waflib import Logs


def _get_pbi_size(data):
    """
    This method takes resource data and determines the dimensions of the pbi
    :param data: the data contained in the pbi, starting at the header
    :return: tuple containing the width and height of the pbi
    """
    # Read the first byte at header offset 0x08 for width
    width = struct.unpack("<h", data[8:10])[0]

    # Read the next 2 bytes after the width to get the height
    height = struct.unpack("<h", data[10:12])[0]

    return width, height


def _get_pdc_size(data):
    """
    This method takes resource data and determines the dimensions of the PDC
    :param data: the data contained in the PDC, starting at the header
    :return: tuple containing the width and height of the PDC
    """
    # Read the first 2 bytes at header offset 0x06 for width
    width = struct.unpack(">I", data[6:8])[0]

    # Read the next 2 bytes after the width to get the height
    height = struct.unpack(">I", data[8:10])[0]

    return width, height


def _get_png_size(data):
    """
    This resource takes resource data and determines the dimensions of the PNG
    :param data: the data contained in the PNG, starting at the IHDR
    :return: tuple containing the width and height of the PNG
    """
    # Assert that this is the IHDR header
    assert data[:4] == b"IHDR"

    # Read the first 4 bytes after IHDR for width
    width = struct.unpack(">I", data[4:8])[0]

    # Read the next 4 bytes after the width to get the height
    height = struct.unpack(">I", data[8:12])[0]

    return width, height


def _get_supported_platforms(ctx):
    """
    This method returns all of the supported SDK platforms, based off of SDK requirements found on
    the filesystem
    :param ctx: the Context object
    :return: a list of the platforms that are supported for the given SDK
    """
    sdk_check_nodes = [
        "lib/libpebble.a",
        "pebble_app.ld.template",
        "tools",
        "include",
        "include/pebble.h",
    ]
    supported_platforms = os.listdir(ctx.env.PEBBLE_SDK_ROOT)

    invalid_platforms = []
    for platform in supported_platforms:
        pebble_sdk_platform = ctx.root.find_node(ctx.env.PEBBLE_SDK_ROOT).find_node(
            platform
        )
        for node in sdk_check_nodes:
            if pebble_sdk_platform.find_node(node) is None:
                if (
                    ctx.root.find_node(ctx.env.PEBBLE_SDK_COMMON).find_node(node)
                    is None
                ):
                    invalid_platforms.append(platform)
                    break
    for platform in invalid_platforms:
        supported_platforms.remove(platform)

    ctx.env.SUPPORTED_PLATFORMS = supported_platforms
    return supported_platforms


def append_to_attr(self, attr, new_values):
    """
    This helper method appends `new_values` to `attr` on the object `self`
    :param self: the object
    :param attr: the attribute to modify
    :param new_values: the value(s) to set on the attribute
    :return: N/A
    """
    values = self.to_list(getattr(self, attr, []))
    if not isinstance(new_values, list):
        new_values = [new_values]
    values.extend(new_values)
    setattr(self, attr, values)


def configure_libraries(ctx, libraries):
    dependencies = list(libraries.keys())
    lib_json = []
    lib_resources_json = {}

    index = 0
    while index < len(dependencies):
        info, resources, additional_deps = process_package(ctx, dependencies[index])
        lib_json.append(info)
        lib_resources_json[dependencies[index]] = resources
        dependencies.extend(additional_deps)
        index += 1

    # Store package.json info for each library and add resources to an environment variable for
    # dependency-checking
    ctx.env.LIB_JSON = lib_json
    if lib_resources_json:
        ctx.env.LIB_RESOURCES_JSON = lib_resources_json


def configure_platform(ctx, platform):
    """
    Configure a build for the <platform> specified
    :param ctx: the ConfigureContext
    :param platform: the hardware platform this build is being targeted for
    :return: N/A
    """
    pebble_sdk_root = get_node_from_abspath(ctx, ctx.env.PEBBLE_SDK_ROOT)
    ctx.env.PLATFORM = pebble_platforms[platform]
    ctx.env.PEBBLE_SDK_PLATFORM = pebble_sdk_root.find_node(str(platform)).abspath()
    ctx.env.PLATFORM_NAME = ctx.env.PLATFORM["NAME"]

    for attribute in ["DEFINES"]:  # Attributes with list values
        ctx.env.append_unique(attribute, ctx.env.PLATFORM[attribute])
    for attribute in ["BUILD_DIR", "BUNDLE_BIN_DIR"]:  # Attributes with a single value
        ctx.env[attribute] = ctx.env.PLATFORM[attribute]
    ctx.env.append_value("INCLUDES", ctx.env.BUILD_DIR)

    ctx.msg(f"Found Pebble SDK for {platform} in:", ctx.env.PEBBLE_SDK_PLATFORM)

    process_info = pebble_sdk_root.find_node(str(platform)).find_node(
        "include/pebble_process_info.h"
    )
    set_env_sdk_version(ctx, process_info)

    if is_sdk_2x(ctx.env.SDK_VERSION_MAJOR, ctx.env.SDK_VERSION_MINOR):
        ctx.env.append_value("DEFINES", "PBL_SDK_2")
    else:
        ctx.env.append_value("DEFINES", "PBL_SDK_3")

    ctx.load("pebble_sdk_gcc")


def find_sdk_component(ctx, env, component):
    """
    This method finds an SDK component, either in the platform SDK folder, or the 'common' folder
    :param ctx: the Context object
    :param env: the environment which contains platform SDK folder path for the current platform
    :param component: the SDK component being sought
    :return: the path to the SDK component being sought
    """
    return ctx.root.find_node(env.PEBBLE_SDK_PLATFORM).find_node(
        component
    ) or ctx.root.find_node(env.PEBBLE_SDK_COMMON).find_node(component)


def get_node_from_abspath(ctx, path):
    return ctx.root.make_node(path)


def get_target_platforms(ctx):
    """
    This method returns a list of target platforms for a build, by comparing the list of requested
    platforms to the list of supported platforms, returning all of the supported platforms if no
    specific platforms are requested
    :param ctx: the Context object
    :return: list of target platforms for the build
    """
    supported_platforms = _get_supported_platforms(ctx)
    if not ctx.env.REQUESTED_PLATFORMS:
        target_platforms = supported_platforms
    else:
        target_platforms = list(
            set(supported_platforms) & set(ctx.env.REQUESTED_PLATFORMS)
        )
    if not target_platforms:
        ctx.fatal(
            f"No valid targetPlatforms specified in appinfo.json. Valid options are {supported_platforms}"
        )

    ctx.env.TARGET_PLATFORMS = sorted([p for p in target_platforms], reverse=True)
    return target_platforms


def is_sdk_2x(major, minor):
    """
    This method checks if a <major>.<minor> API version are associated with a 2.x version of the SDK
    :param major: the major API version to check
    :param minor: the minor API version to check
    :return: boolean representing whether a 2.x SDK is being used or not
    """
    LAST_2X_MAJOR_VERSION = 5
    LAST_2X_MINOR_VERSION = 19
    return (major, minor) <= (LAST_2X_MAJOR_VERSION, LAST_2X_MINOR_VERSION)


def process_package(ctx, package, root_lib_node=None):
    """
    This method parses the package.json for a given package and returns relevant information
    :param ctx: the Context object
    :param root_lib_node: node containing the package to be processed, if not the standard LIB_DIR
    :param package: the package to parse information for
    :return:
    - a dictionary containing the contents of package.json
    - a dictionary containing the resources object for the package
    - a list of dependencies for this package
    """
    resources_json = {}

    if not root_lib_node:
        root_lib_node = ctx.path.find_node(ctx.env.LIB_DIR)
        if root_lib_node is None:
            ctx.fatal(f"Missing {ctx.env.LIB_DIR} directory")

    lib_node = root_lib_node.find_node(str(package))
    if lib_node is None:
        ctx.fatal(f"Missing library for {package!s} in {ctx.env.LIB_DIR}")
    else:
        libinfo_node = lib_node.find_node("package.json")
        if libinfo_node is None:
            ctx.fatal(f"Missing package.json for {package!s} library")
        else:
            if lib_node.find_node(ctx.env.LIB_DIR):
                error_str = (
                    "ERROR: Multiple versions of the same package are not supported by "
                    f"the Pebble SDK due to namespace issues during linking. Package '{package}' "
                    "contains the following duplicate and incompatible dependencies, "
                    "which may lead to additional build errors and/or unpredictable "
                    "runtime behavior:\n"
                )
                packages_str = ""
                for package in lib_node.find_node(ctx.env.LIB_DIR).ant_glob(
                    "**/package.json"
                ):
                    with open(package.abspath()) as f:
                        info = json.load(f)
                    if not dict(ctx.env.PROJECT_INFO).get("enableMultiJS", False):
                        if "pebble" not in info:
                            continue
                    packages_str += "      '{}': '{}'\n".format(
                        info["name"], info["version"]
                    )
                if packages_str:
                    Logs.pprint("RED", error_str + packages_str)

            with open(libinfo_node.abspath()) as f:
                libinfo = json.load(f)

            if "pebble" in libinfo:
                libinfo["path"] = lib_node.make_node("dist").path_from(ctx.path)
                if "resources" in libinfo["pebble"]:
                    if "media" in libinfo["pebble"]["resources"]:
                        resources_json = libinfo["pebble"]["resources"]["media"]

                # Extract package into "dist" folder
                dist_node = lib_node.find_node("dist.zip")
                if not dist_node:
                    ctx.fatal(
                        f"Missing dist.zip file for {package}. Are you sure this is a Pebble "
                        "library?"
                    )
                lib_package = LibraryPackage(dist_node.abspath())
                lib_package.unpack(libinfo["path"])
                lib_js_node = lib_node.find_node("dist/js")
                if lib_js_node:
                    libinfo["js_paths"] = [
                        lib_js.path_from(ctx.path)
                        for lib_js in lib_js_node.ant_glob(["**/*.js", "**/*.json"])
                    ]
            else:
                libinfo["js_paths"] = [
                    lib_js.path_from(ctx.path)
                    for lib_js in lib_node.ant_glob(
                        ["**/*.js", "**/*.json"], excl="**/*.min.js"
                    )
                ]

            dependencies = (
                list(libinfo["dependencies"].keys())
                if "dependencies" in libinfo
                else []
            )
            return libinfo, resources_json, dependencies


def truncate_to_32_bytes(name):
    """
    This method takes an input string and returns a 32-byte truncated string if the input string is
    longer than 32 bytes
    :param name: the string to truncate
    :return: the truncated string, if the input string was > 32 bytes, or else the original input
    string
    """
    return name[:30] + ".." if len(name) > 32 else name


def validate_message_keys_object(ctx, project_info, info_json_type):
    """
    Verify that the appropriately-named message key object is present in the project info file
    :param ctx: the ConfigureContext object
    :param project_info: JSON object containing project info
    :param info_json_type: string containing the name of the file used to extract project info
    :return: N/A
    """
    if "appKeys" in project_info and info_json_type == "package.json":
        ctx.fatal(
            "Project contains an invalid object `appKeys` in package.json. Please use "
            "`messageKeys` instead."
        )
    if "messageKeys" in project_info and info_json_type == "appinfo.json":
        ctx.fatal(
            "Project contains an invalid object `messageKeys` in appinfo.json. Please use "
            "`appKeys` instead."
        )


def validate_resource_not_larger_than(
    ctx, resource_file, dimensions=None, width=None, height=None
):
    """
    This method takes a resource file and determines whether the file's dimensions exceed the
    maximum allowed values provided.
    :param resource_file: the path to the resource file
    :param dimensions: tuple specifying max width and height
    :param width: number specifying max width
    :param height: number specifying max height
    :return: boolean for whether the resource is larger than the maximum allowed size
    """
    if not dimensions and not width and not height:
        raise TypeError(
            "Missing values for maximum width and/or height to validate against"
        )
    if dimensions:
        width, height = dimensions

    with open(resource_file, "rb") as f:
        if resource_file.endswith(".reso"):
            reso = ResourceObject.load(resource_file)
            if reso.definition.type == "bitmap":
                storage_format = reso.definition.storage_format
            else:
                storage_format = reso.definition.type

            if storage_format == "pbi":
                resource_size = _get_pbi_size(reso.data)
            elif storage_format == "png":
                resource_size = _get_png_size(reso.data[12:])
            elif storage_format == "raw":
                try:
                    assert reso.data[4:] == "PDCI"
                except AssertionError:
                    ctx.fatal(
                        f"Unsupported published resource type for {resource_file}"
                    )
                else:
                    resource_size = _get_pdc_size(reso.data[4:])
        else:
            data = f.read(24)
            if data[1:4] == b"PNG":
                resource_size = _get_png_size(data[12:])
            elif data[:4] == b"PDCI":
                resource_size = _get_pdc_size(data[4:])
            else:
                ctx.fatal(
                    f"Unsupported published resource type for {resource_file}"
                )

    if width and height:
        return resource_size <= (width, height)
    elif width:
        return resource_size[0] <= width
    elif height:
        return resource_size[1] <= height


def wrap_task_name_with_platform(self):
    """
    This method replaces the existing waf Task class's __str__ method with the original content
    of the __str__ method, as well as an additional "<platform> | " before the task information,
    if a platform is set.
    :param self: the task instance
    :return: the user-friendly string to print
    """
    src_str = " ".join([a.path_from(a.ctx.launch_node()) for a in self.inputs])
    tgt_str = " ".join([a.path_from(a.ctx.launch_node()) for a in self.outputs])
    sep = " -> " if self.outputs else ""
    name = self.__class__.__name__.replace("_task", "")

    # Modification to the original __str__ method
    if self.env.PLATFORM_NAME:
        name = self.env.PLATFORM_NAME + " | " + name

    return "%s: %s%s%s" % (name, src_str, sep, tgt_str)
