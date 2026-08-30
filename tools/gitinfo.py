# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""Firmware version information, derived from the git checkout."""

import re
import subprocess
import sys


def _git(*args):
    return subprocess.check_output(["git", *args], text=True).strip()


def _is_dirty():
    """Whether the working tree differs from HEAD.

    `git describe --dirty` answers this too, but it descends into every
    submodule, which on this tree costs ten times as much as the rest of
    the version lookup put together. A submodule moved to another commit
    still counts as dirty; uncommitted edits inside one do not.
    """
    return subprocess.call(
        ["git", "diff-index", "--quiet", "--ignore-submodules=dirty", "HEAD", "--"]
    ) != 0


def get_git_revision():
    # One process for both: the version rule runs on every build.
    timestamp, commit = _git("log", "-1", "--format=%ct%n%h", "HEAD").split("\n")

    try:
        tag = _git("describe")
        if _is_dirty():
            tag += "-dirty"
    except subprocess.CalledProcessError:
        tag = "v9.9.9-dev"
        print(f"Git tag not found, using {tag}", file=sys.stderr)

    # The tag must follow the documented form. See
    # https://github.com/pebble/tintin/wiki/Firmware,-PRF-&-Bootloader-Versions
    # A fourth numeric component (e.g. v4.9.142.1) is accepted for point
    # releases; it only shows up in TAG and PATCH_VERBOSE_STRING.
    match = re.search(r"^v(\d+)(?:\.(\d+))?(?:\.(\d+))?(?:\.(\d+))?(?:(?:-)(.+))?$", tag)
    if not match:
        raise ValueError(f"Invalid tag: {tag}")

    # e.g. v2-beta11 => ('2', None, None, None, 'beta11') => ('2', '0', '0')
    version = [x if x else "0" for x in match.groups()]

    # Everything after the minor version, with the patch forced in.
    patch_verbose = str(version[2])
    if match.group(4):
        patch_verbose += "." + version[3]
    if version[4]:
        patch_verbose += "-" + version[4]

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
