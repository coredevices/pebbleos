# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""build-info.json: neutral description of a configured build.

Written by the configure step, consumed by the standalone build tooling
(fw_image, bundling, QEMU images, the pbl CLI) so none of it needs to
parse waf's internal state. The producer does not have to be waf: any
build system that writes the same file can drive the same tools.
"""

import json
import os

FILENAME = "build-info.json"
FORMAT_VERSION = 1


def build_info_path(build_dir):
    return os.path.join(build_dir, FILENAME)


def write_build_info(build_dir, info):
    info = dict(info)
    info["format_version"] = FORMAT_VERSION
    with open(build_info_path(build_dir), "w") as f:
        json.dump(info, f, indent=2, sort_keys=True)
        f.write("\n")


def load_build_info(build_dir):
    path = build_info_path(build_dir)
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"{path} not found -- configure the build first "
            "(./pbl configure --board BOARD)"
        )
    with open(path) as f:
        info = json.load(f)
    version = info.get("format_version")
    if version != FORMAT_VERSION:
        raise ValueError(
            f"{path} has format_version {version}, expected {FORMAT_VERSION} "
            "-- re-run configure"
        )
    return info
