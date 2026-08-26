#!/usr/bin/env python
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Standalone SDK generation script.

Generates SDK files for one or more Pebble platforms without requiring
a full waf configure/build cycle.

Usage:
    python tools/build_sdk.py basalt            # Single platform
    python tools/build_sdk.py aplite basalt     # Multiple platforms
    python tools/build_sdk.py all               # All platforms
"""


import argparse
import json
import os
import re
import shutil
import sys
from os import path

REPO_ROOT = path.dirname(path.dirname(path.abspath(__file__)))
sys.path.insert(0, path.join(REPO_ROOT, "tools", "generate_native_sdk"))
sys.path.insert(0, path.join(REPO_ROOT, "tools"))

from generate_pebble_native_sdk_files import generate_shim_files
from pebble_sdk_platform import pebble_platforms

SHIM_DEF = path.join(REPO_ROOT, "tools", "generate_native_sdk", "exported_symbols.json")
SRC_DIR = path.join(REPO_ROOT, "src")
PROCESS_INFO_H = path.join(SRC_DIR, "fw", "process_management", "pebble_process_info.h")

# Regex matching: // sdk.major:0x5 .minor:0x4e -- ... (rev 81)
_REV_COMMENT_RE = re.compile(
    r"//\s*sdk\.major:(0x[0-9a-fA-F]+)\s*\.minor:(0x[0-9a-fA-F]+)\s*--.*\(rev\.?\s*(\d+)\)"
)


def _revision_to_sdk_version(frozen_revision):
    """Parse pebble_process_info.h comments to find the SDK major/minor for a
    given export revision number."""
    with open(PROCESS_INFO_H) as f:
        for line in f:
            m = _REV_COMMENT_RE.search(line)
            if m and int(m.group(3)) == frozen_revision:
                return m.group(1), m.group(2)  # major, minor as hex strings
    raise RuntimeError(
        f"Could not find SDK version for revision {frozen_revision} in {PROCESS_INFO_H}"
    )


def _patch_process_info_version(dest_path, frozen_revision):
    """Rewrite PROCESS_INFO_CURRENT_SDK_VERSION_{MAJOR,MINOR} in a copied
    pebble_process_info.h to match a frozen revision."""
    major, minor = _revision_to_sdk_version(frozen_revision)
    with open(dest_path) as f:
        text = f.read()
    text = re.sub(
        r"(#define PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR\s+)0x[0-9a-fA-F]+",
        rf"\g<1>{major}",
        text,
    )
    text = re.sub(
        r"(#define PROCESS_INFO_CURRENT_SDK_VERSION_MINOR\s+)0x[0-9a-fA-F]+",
        rf"\g<1>{minor}",
        text,
    )
    with open(dest_path, "w") as f:
        f.write(text)


def build_sdk_for_platform(platform_name, output_dir, internal_sdk_build):
    if platform_name not in pebble_platforms:
        raise SystemExit(
            "Unknown platform '{}'. Available: {}".format(
                platform_name, ", ".join(sorted(pebble_platforms.keys()))
            )
        )

    print(f"=== Building SDK for {platform_name} ===")

    platform_dir = path.join(output_dir, platform_name)
    sdk_include_dir = path.join(platform_dir, "include")
    sdk_lib_dir = path.join(platform_dir, "lib")
    pbl_output_src_dir = path.join(output_dir, "src")

    for d in (sdk_include_dir, sdk_lib_dir, path.join(pbl_output_src_dir, "fw")):
        if not path.isdir(d):
            os.makedirs(d)

    # Copy static headers (mirrors the __main__ block of
    # generate_pebble_native_sdk_files.py)
    platform_info = pebble_platforms[platform_name]
    dest_process_info = path.join(sdk_include_dir, "pebble_process_info.h")
    shutil.copy(PROCESS_INFO_H, dest_process_info)
    frozen_revision = platform_info.get("FROZEN_AT_REVISION")
    if frozen_revision is not None:
        _patch_process_info_version(dest_process_info, frozen_revision)
    shutil.copy(
        path.join(SRC_DIR, "fw", "applib", "graphics", "gcolor_definitions.h"),
        path.join(sdk_include_dir, "gcolor_definitions.h"),
    )
    shutil.copy(
        path.join(SRC_DIR, "fw", "applib", "pebble_warn_unsupported_functions.h"),
        path.join(sdk_include_dir, "pebble_warn_unsupported_functions.h"),
    )

    # Generate pebble_fonts.h from the font whitelist in exported_symbols.json
    with open(SHIM_DEF) as f:
        raw_fonts = json.load(f)["fonts"]
    with open(path.join(sdk_include_dir, "pebble_fonts.h"), "w") as f:
        f.write("#pragma once\n\n")
        for entry in raw_fonts:
            if isinstance(entry, dict):
                name = entry["name"]
                added = entry.get("addedRevision")
                if (
                    frozen_revision is not None
                    and added is not None
                    and added > frozen_revision
                ):
                    continue
            else:
                name = entry
            f.write(f'#define FONT_KEY_{name} "RESOURCE_ID_{name}"\n')

    is_frozen = frozen_revision is not None

    generate_shim_files(
        SHIM_DEF,
        SRC_DIR,
        pbl_output_src_dir,
        sdk_include_dir,
        sdk_lib_dir,
        platform_name,
        internal_sdk_build=internal_sdk_build,
        build_shim_lib=not is_frozen,
    )

    if is_frozen:
        print("    Skipped libpebble.a (frozen SDK, use pre-built library)")

    print(f"    Output: {platform_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate Pebble SDK files for one or more platforms "
        "without a full waf build."
    )
    parser.add_argument(
        "platforms",
        nargs="+",
        help="Platform name(s) to build, or 'all' for every platform.",
    )
    parser.add_argument(
        "--output-dir",
        default=path.join(REPO_ROOT, "build", "sdk"),
        help="Root output directory (default: build/sdk)",
    )
    parser.add_argument(
        "--internal-sdk-build",
        action="store_true",
        help="Enable internal SDK build",
    )
    args = parser.parse_args()

    if "all" in args.platforms:
        platforms = sorted(pebble_platforms.keys())
    else:
        platforms = args.platforms

    for p in platforms:
        build_sdk_for_platform(p, args.output_dir, args.internal_sdk_build)

    print(f"\nDone. SDK(s) generated in {args.output_dir}")


if __name__ == "__main__":
    main()
