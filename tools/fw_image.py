#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Post-link firmware image pipeline.

Turns the linked ELF into the flashable images and log-hash dictionaries:

    pebbleos.elf -> pebbleos.hex / pebbleos.bin   (pblboot header when
                                                   CONFIG_PBLBOOT)
                 -> pebbleos_loghash_dict.json    (when CONFIG_LOG_HASHED)
                 -> src/fw/loghash_dict.json

Driven entirely by build/build-info.json; standalone from the build system
that produced the ELF.
"""

import argparse
import json
import os
import subprocess
import sys

import build_info
import gitinfo
import pblboot

# Debug/no-load sections stripped from the flashable images.
OBJCOPY_STRIP_ARGS = [
    "-S",
    "-R",
    ".stack",
    "-R",
    ".priv_bss",
    "-R",
    ".bss",
    "-R",
    ".retained",
]


def _objcopy(elf, out, fmt):
    subprocess.check_call(
        ["arm-none-eabi-objcopy"] + OBJCOPY_STRIP_ARGS + ["-O", fmt, elf, out]
    )


def _make_images(build_dir, elf, artifacts, config):
    hex_out = os.path.join(build_dir, artifacts["hex"])
    bin_out = os.path.join(build_dir, artifacts["bin"])

    if not config.get("CONFIG_PBLBOOT"):
        _objcopy(elf, hex_out, "ihex")
        _objcopy(elf, bin_out, "binary")
        return

    revision = gitinfo.get_git_revision(cwd=os.path.dirname(os.path.abspath(__file__)))
    priority = pblboot.boot_priority(revision["TAG"], int(revision["TIMESTAMP"]))
    offset = config["CONFIG_FIRMWARE_OFFSET"]

    nohdr_hex = os.path.splitext(hex_out)[0] + ".nohdr.hex"
    nohdr_bin = os.path.splitext(bin_out)[0] + ".nohdr.bin"
    _objcopy(elf, nohdr_hex, "ihex")
    _objcopy(elf, nohdr_bin, "binary")
    pblboot.insert_header_hex(nohdr_hex, hex_out, offset, priority)
    pblboot.insert_header_bin(nohdr_bin, bin_out, offset, priority)


def _make_loghash_dicts(build_dir, elf, artifacts):
    from log_hashing.check_elf_log_strings import check_dict_log_strings
    from log_hashing.newlogging import get_log_dict_from_file

    log_dict = get_log_dict_from_file(elf)
    if not log_dict:
        sys.exit(f"Unable to get log strings from {elf}")

    # Confirm that the log strings satisfy the rules
    output = check_dict_log_strings(log_dict)
    if output:
        sys.exit(output)

    for key in ("fw_loghash_dict", "loghash_dict"):
        path = os.path.join(build_dir, artifacts[key])
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(log_dict, f, indent=2, sort_keys=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-dir", required=True, help="Configured build directory")
    parser.add_argument("--elf", help="Firmware ELF (defaults to the configured one)")
    args = parser.parse_args()

    info = build_info.load_build_info(args.build_dir)
    artifacts = info["artifacts"]
    config = info["config"]
    elf = args.elf or os.path.join(args.build_dir, artifacts["elf"])

    _make_images(args.build_dir, elf, artifacts, config)
    if config.get("CONFIG_LOG_HASHED"):
        _make_loghash_dicts(args.build_dir, elf, artifacts)


if __name__ == "__main__":
    main()
