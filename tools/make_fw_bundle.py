#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Bundle a built firmware into a .pbz.

Wraps tools/mkbundle.py with the build-specific glue that used to live in
the root wscript: version info from git, the firmware size gate, resource
pack / loghash / layouts attachment and the output naming convention.
Driven by build/build-info.json; needs a completed build.
"""

import argparse
import os
import sys

import build_info
import gitinfo
import mkbundle

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _get_version_info():
    revision = gitinfo.get_git_revision(cwd=REPO_ROOT)
    if revision["TAG"] != "?":
        return revision["TAG"], int(revision["TIMESTAMP"]), revision["COMMIT"]
    return "dev", 0, ""


def _check_firmware_image_size(path, max_firmware_size):
    firmware_size = os.path.getsize(path)
    if firmware_size > max_firmware_size:
        sys.exit(
            f"Firmware is too large! Size is {firmware_size:#x} "
            f"should be less than {max_firmware_size:#x}"
        )


def make_bundle(build_dir, out_path=None):
    info = build_info.load_build_info(build_dir)
    artifacts = info["artifacts"]
    config = info["config"]

    fw_type = "recovery" if info["variant"] == "prf" else "normal"
    fw_bin = os.path.join(build_dir, artifacts["bin"])
    board = info["board_normalized"]

    version_string, version_ts, version_commit = _get_version_info()
    slot = info["slot"] if fw_type == "normal" else None

    if out_path is None:
        slot_suffix = "" if slot is None else f"_slot{slot}"
        out_path = os.path.join(
            build_dir, f"{fw_type}_{board}_{version_string}{slot_suffix}.pbz"
        )

    max_size = config.get("CONFIG_FW_MAX_SIZE")
    if not max_size:
        sys.exit("CONFIG_FW_MAX_SIZE not set, cannot check firmware size")
    _check_firmware_image_size(fw_bin, max_size)

    b = mkbundle.PebbleBundle()
    try:
        b.add_firmware(
            fw_bin, fw_type, version_ts, version_commit, board, version_string, slot
        )
    except mkbundle.MissingFileException as e:
        sys.exit(f"Error: Missing file {e.filename}, have you run ./pbl build yet?")

    if fw_type == "normal":
        b.add_resources(os.path.join(build_dir, artifacts["pbpack"]), version_ts)

    if not config.get("CONFIG_RELEASE") and config.get("CONFIG_LOG_HASHED"):
        b.add_loghash(os.path.join(build_dir, artifacts["loghash_dict"]))

    b.add_license(os.path.join(REPO_ROOT, "LICENSE"))

    if fw_type == "normal":
        layouts = os.path.join(build_dir, "resources", "layouts.json.auto")
        if os.path.isfile(layouts):
            b.add_layouts(layouts)

    b.write(out_path)
    print(f"Writing bundle to: {out_path}")
    return out_path


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-dir", required=True, help="Configured build directory")
    parser.add_argument(
        "-o",
        "--out",
        help="Output .pbz path (defaults to the "
        "conventional name in the build directory)",
    )
    args = parser.parse_args()

    make_bundle(args.build_dir, args.out)


if __name__ == "__main__":
    main()
