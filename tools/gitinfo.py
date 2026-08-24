#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Git revision info for the firmware build.

Standalone (no waf): used by the build system, the post-link image tooling
and the bundle tooling. The version dict keys are part of the interface;
`git_version.auto.h` substitution and the bundle manifest depend on them.
"""

import argparse
import json
import re
import subprocess
import sys


def _git(args, cwd=None):
    return (
        subprocess.check_output(["git"] + args, cwd=cwd, stderr=subprocess.DEVNULL)
        .decode()
        .strip()
    )


def get_git_revision(cwd=None):
    commit = _git(["rev-parse", "--short", "HEAD"], cwd)
    timestamp = _git(["log", "-1", "--format=%ct", "HEAD"], cwd)

    try:
        tag = _git(["describe", "--dirty"], cwd)
    except subprocess.CalledProcessError:
        tag = "v9.9.9-dev"
        print(f"Git tag not found, using {tag}", file=sys.stderr)

    # Validate that git tag follows the required form:
    # See https://github.com/pebble/tintin/wiki/Firmware,-PRF-&-Bootloader-Versions
    # An optional fourth numeric component (e.g. v4.9.142.1) is accepted for
    # point releases; it is only exposed through TAG and PATCH_VERBOSE_STRING.
    # Note: version_regex.groups() returns sequence ('0', '0', '0', '0', 'suffix'):
    version_regex = re.search(
        r"^v(\d+)(?:\.(\d+))?(?:\.(\d+))?(?:\.(\d+))?(?:(?:-)(.+))?$", tag
    )
    if not version_regex:
        raise ValueError(f"Invalid tag: {tag}")

    # Get version numbers from version_regex.groups() sequence and replace None values with 0
    # e.g. v2-beta11 => ('2', None, None, None, 'beta11') => ('2', '0', '0')
    version = [x if x else "0" for x in version_regex.groups()]

    # Used for pebble_pipeline payload, generate a string that contains everything after minor.
    # Force include patch as 0 if it doesn't exist.
    patch_verbose = str(version[2])
    if version_regex.group(4):
        patch_verbose += "." + version[3]
    str_after_patch = version[4]
    if str_after_patch:
        patch_verbose += "-" + str_after_patch

    return {
        "TAG": tag,
        "COMMIT": commit,
        "TIMESTAMP": timestamp,
        "MAJOR_VERSION": version[0],
        "MINOR_VERSION": version[1],
        "PATCH_VERSION": version[2],
        "MAJOR_MINOR_PATCH_STRING": ".".join(version[0:3]),
        "PATCH_VERBOSE_STRING": patch_verbose,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Print firmware git revision info")
    parser.add_argument("-C", "--repo", default=None, help="Repository directory")
    parser.add_argument("--json", action="store_true", help="Output as JSON")
    args = parser.parse_args()

    revision = get_git_revision(cwd=args.repo)
    if args.json:
        print(json.dumps(revision, indent=2))
    else:
        for key, value in revision.items():
            print(f"{key}={value}")
