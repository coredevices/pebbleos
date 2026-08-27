#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0



import argparse
import os
import shutil
from functools import cmp_to_key
from os import path

import exports
from extract_comments import extract_comments
from extract_symbol_info import extract_symbol_info
from generate_app_header import make_app_header
from generate_app_sdk_version_header import generate_app_sdk_version_header
from generate_app_shim import make_app_shim_lib
from generate_fw_shim import make_fw_shims
from generate_json_api_description import make_json_api_description

# When this file is called by waf using `python generate_pebble_native_sdk_files.py ...`, we
# need to append the parent directory to the system PATH because relative imports won't work
try:
    from ..pebble_sdk_platform import pebble_platforms
except ImportError:
    os.sys.path.append(path.dirname(path.dirname(__file__)))
    from pebble_sdk_platform import pebble_platforms

SRC_DIR = "src"
INCLUDE_DIR = "include"
LIB_DIR = "lib"

PEBBLE_APP_H_TEXT = """\
#include "pebble_fonts.h"
#include "message_keys.auto.h"
#include "src/resource_ids.auto.h"

#define PBL_APP_INFO(...) _Pragma("message \\"\\n\\n \\
  *** PBL_APP_INFO has been replaced with appinfo.json\\n \\
  Try updating your project with `pebble convert-project`\\n \\
  Visit our developer guides to learn more about appinfo.json:\\n \\
  http://developer.getpebble.com/guides/pebble-apps/\\n \\""); \\
  _Pragma("GCC error \\"PBL_APP_INFO has been replaced with appinfo.json\\"");

#define PBL_APP_INFO_SIMPLE PBL_APP_INFO
"""


def generate_shim_files(
    shim_def_path,
    pbl_src_dir,
    pbl_output_src_dir,
    sdk_include_dir,
    sdk_lib_dir,
    platform_name,
    internal_sdk_build=False,
    build_shim_lib=True,
):
    if internal_sdk_build:
        try:
            pass
        except ValueError:
            os.sys.path.append(path.dirname(path.dirname(__file__)))

    try:
        platform_info = pebble_platforms.get(platform_name)
    except KeyError:
        raise RuntimeError(f"Unsupported platform: {platform_name}")

    frozen_revision = platform_info.get("FROZEN_AT_REVISION")
    files, exports_tree = exports.parse_export_file(
        shim_def_path,
        internal_sdk_build,
        frozen_revision=frozen_revision,
    )
    root_dir = os.path.dirname(pbl_src_dir)
    files = [
        os.path.join(pbl_src_dir, f)
        if os.path.exists(os.path.join(pbl_src_dir, f))
        else os.path.join(root_dir, f)
        for f in files
    ]

    functions = []
    stubbed_functions = []

    def collect_functions(e):
        if isinstance(e, exports.StubbedFunctionExport):
            stubbed_functions.append(e)
        elif e.type == "function":
            functions.append(e)

    exports.walk_tree(exports_tree, collect_functions)
    all_functions = functions + stubbed_functions
    types = []
    exports.walk_tree(
        exports_tree, lambda e: types.append(e) if e.type == "type" else None
    )
    defines = []
    exports.walk_tree(
        exports_tree, lambda e: defines.append(e) if e.type == "define" else None
    )
    groups = []
    exports.walk_tree(
        exports_tree,
        lambda e: groups.append(e) if isinstance(e, exports.Group) else None,
        include_groups=True,
    )

    compiler_flags = [f"-D{d}" for d in platform_info["DEFINES"]]

    freertos_port_name = "ARM_CM3" if platform_name == "aplite" else "ARM_CM4F"
    compiler_flags.extend(
        [
            f"-I{pbl_src_dir}/../third_party/freertos/FreeRTOS-Kernel/FreeRTOS/Source/{p}"
            for p in ["include", f"portable/GCC/{freertos_port_name}"]
        ]
    )

    extract_symbol_info(
        files,
        functions,
        types,
        defines,
        pbl_output_src_dir,
        internal_sdk_build=internal_sdk_build,
        compiler_flags=compiler_flags,
    )
    extract_comments(files, groups, defines)

    # Make sure we found all the exported items
    def check_complete(e):
        if not e.complete():
            raise RuntimeError(
                f"""Missing export: {e} {e.__dict__!s}.
Hint: Add appropriate headers to the \"files\" array in exported_symbols.json"""
            )

    exports.walk_tree(exports_tree, check_complete)

    pebble_app_h_text_to_inject = PEBBLE_APP_H_TEXT + "\n".join(
        platform_info["ADDITIONAL_TEXT_LINES_FOR_PEBBLE_H"]
    )
    if platform_info.get("HAS_MODDABLE_XS"):
        pebble_app_h_text_to_inject += '\n#include "xsffi.h"\n'

    # Build pebble.h and pebble_worker.h for our apps to include
    for type_name_prefix in [
        ("app", "pebble.h", pebble_app_h_text_to_inject),
        ("worker", "pebble_worker.h", None),
        ("worker_only", "doxygen/pebble_worker.h", None),
    ]:
        sdk_header_filename = path.join(sdk_include_dir, type_name_prefix[1])
        make_app_header(
            exports_tree, sdk_header_filename, type_name_prefix[0], type_name_prefix[2]
        )

    # On platforms with Moddable XS support, ship xsffi.h alongside the SDK so
    # apps that use the FFI bindings can include it directly.
    if platform_info.get("HAS_MODDABLE_XS"):
        repo_root = path.dirname(pbl_src_dir)
        xsffi_src = path.join(
            repo_root,
            "third_party",
            "moddable",
            "moddable",
            "xs",
            "includes",
            "xsffi.h",
        )
        shutil.copy(xsffi_src, path.join(sdk_include_dir, "xsffi.h"))

    def function_export_compare_func(x, y):
        def cmp(a, b):
            return (a > b) - (a < b)

        if x.added_revision != y.added_revision:
            return cmp(x.added_revision, y.added_revision)

        return cmp(x.sort_name, y.sort_name)

    sorted_functions = sorted(functions, key=cmp_to_key(function_export_compare_func))

    # Build libpebble.a for our apps to compile against
    if build_shim_lib:
        make_app_shim_lib(sorted_functions, sdk_lib_dir)

    # Build pebble.auto.c to build into our firmware
    make_fw_shims(sorted_functions, pbl_output_src_dir)

    # Build .json API description, used as input for static analysis tools:
    make_json_api_description(sorted_functions, pbl_output_src_dir)

    for filename, version_functions in (
        ("pebble_sdk_version.h", (f for f in all_functions if not f.worker_only)),
        ("pebble_worker_sdk_version.h", (f for f in all_functions if not f.app_only)),
    ):
        generate_app_sdk_version_header(
            path.join(sdk_include_dir, filename),
            version_functions,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Auto-generate the Pebble native SDK files"
    )
    parser.add_argument(
        "--sdk-dir",
        dest="sdk_dir",
        help="root of the SDK dir",
        metavar="SDKDIR",
        required=True,
    )
    parser.add_argument("config")
    parser.add_argument("src_dir")
    parser.add_argument("output_dir")
    parser.add_argument("platform_name")
    parser.add_argument(
        "--internal-sdk-build", action="store_true", help="build internal SDK"
    )

    options = parser.parse_args()

    shim_config = path.normpath(path.abspath(options.config))
    pbl_src_dir = path.normpath(path.abspath(options.src_dir))
    pbl_output_src_dir = path.normpath(path.abspath(options.output_dir))

    sdk_include_dir = path.join(path.abspath(options.sdk_dir), INCLUDE_DIR)
    sdk_lib_dir = path.join(path.abspath(options.sdk_dir), LIB_DIR)

    if not path.isdir(pbl_src_dir):
        raise RuntimeError(f"'{pbl_src_dir}' does not exist")

    for d in (sdk_include_dir, sdk_lib_dir):
        if not path.isdir(d):
            os.makedirs(d)

    shutil.copy(
        path.join(pbl_src_dir, "fw", "process_management", "pebble_process_info.h"),
        path.join(sdk_include_dir, "pebble_process_info.h"),
    )

    shutil.copy(
        path.join(pbl_src_dir, "fw/applib/graphics", "gcolor_definitions.h"),
        path.join(sdk_include_dir, "gcolor_definitions.h"),
    )

    # Copy unsupported function warnings header to SDK
    shutil.copy(
        path.join(pbl_src_dir, "fw", "applib", "pebble_warn_unsupported_functions.h"),
        path.join(sdk_include_dir, "pebble_warn_unsupported_functions.h"),
    )

    generate_shim_files(
        shim_config,
        pbl_src_dir,
        pbl_output_src_dir,
        sdk_include_dir,
        sdk_lib_dir,
        options.platform_name,
        internal_sdk_build=options.internal_sdk_build,
    )
