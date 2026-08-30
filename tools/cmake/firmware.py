#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Firmware image steps: size checks, bundling and QEMU flash images."""

import argparse
import os
import sys

import wafshim
from wafshim import REPO_ROOT

wafshim.setup_path()

import gitinfo

BYTES_PER_K = 1024

CYAN = "\033[36m"
RESET = "\033[0m"


def pprint(msg):
    print(f"{CYAN}{msg}{RESET}")


def read_config(path):
    config = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            config[key] = value
    return config


def enabled(config, key):
    return config.get(key) == "y"


def max_resources_size(config):
    if enabled(config, "CONFIG_SOC_NRF52"):
        return 1024 * BYTES_PER_K
    if enabled(config, "CONFIG_SOC_SF32LB52") or enabled(config, "CONFIG_QEMU"):
        return 2048 * BYTES_PER_K
    return 256 * BYTES_PER_K


def version_info():
    revision = gitinfo.get_git_revision()
    if revision["TAG"] != "?":
        return revision["TAG"], int(revision["TIMESTAMP"]), revision["COMMIT"]
    return "dev", 0, ""


def cmd_size_resources(args):
    config = read_config(args.config)
    maximum = max_resources_size(config)
    actual = os.path.getsize(args.pbpack)

    bar_width = 20
    filled = min(bar_width, round(bar_width * actual / maximum))
    bar = "#" * filled
    pprint(
        f"Resources: [{bar:<{bar_width}}] {100 * actual / maximum:6.2f}% "
        f"({actual}/{maximum} bytes)\n"
    )
    if actual > maximum:
        sys.exit(f"Resources are too large for target board {actual} > {maximum}")


def cmd_bundle(args):
    import mkbundle

    config = read_config(args.config)
    version_string, version_ts, version_commit = version_info()
    fw_type = "recovery" if enabled(config, "CONFIG_RECOVERY_FW") else "normal"
    slot = args.slot if fw_type == "normal" and args.slot != -1 else None

    bundle = mkbundle.PebbleBundle()
    try:
        bundle.add_firmware(
            args.firmware, fw_type, version_ts, version_commit, args.board,
            version_string, slot,
        )
    except mkbundle.MissingFileException as e:
        sys.exit(f"Error: Missing file {e.filename}, have you built the firmware yet?")

    if args.resources:
        bundle.add_resources(args.resources, version_ts)
    if not enabled(config, "CONFIG_RELEASE") and enabled(config, "CONFIG_LOG_HASHED"):
        bundle.add_loghash(args.loghash)
    bundle.add_license(os.path.join(REPO_ROOT, "LICENSE"))
    if fw_type == "normal" and args.layouts:
        bundle.add_layouts(args.layouts)

    out_file = os.path.join(
        args.outdir,
        "{}_{}_{}{}.pbz".format(
            fw_type, args.board, version_string,
            "" if slot is None else f"_slot{slot}",
        ),
    )
    bundle.write(out_file)
    pprint(f"Writing bundle to: {out_file}")


def cmd_qemu_image_micro(args):
    from intelhex import IntelHex

    pprint(f"Writing micro flash image to {args.output}")
    img = IntelHex(args.input)
    img.padding = 0xFF
    flash_end = ((img.maxaddr() + 511) // 512) * 512
    img.tobinfile(args.output, start=0x00000000, end=flash_end - 1)


def cmd_qemu_image_spi(args):
    config = read_config(args.config)
    if enabled(config, "CONFIG_QEMU"):
        # QEMU generic boards: resources at offset 0x620000 in 32MB flash
        resources_begin = 0x620000
        image_size = 0x2000000
    else:
        resources_begin = 0x280000
        image_size = 0x400000

    pprint(f"Writing SPI flash image to {args.output}")
    with open(args.pbpack, "rb") as f:
        resources = f.read()
    with open(args.output, "wb") as f:
        # Pad the region ahead of the system resources with 0xff
        f.write(b"\xff" * resources_begin)
        f.write(resources)
        f.write(b"\xff" * (image_size - resources_begin - len(resources)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("size-resources")
    p.add_argument("--config", required=True)
    p.add_argument("--pbpack", required=True)
    p.set_defaults(func=cmd_size_resources)

    p = sub.add_parser("bundle")
    p.add_argument("--config", required=True)
    p.add_argument("--firmware", required=True)
    p.add_argument("--board", required=True)
    p.add_argument("--outdir", required=True)
    p.add_argument("--slot", type=int, default=-1)
    p.add_argument("--resources")
    p.add_argument("--loghash")
    p.add_argument("--layouts")
    p.set_defaults(func=cmd_bundle)

    p = sub.add_parser("qemu-image-micro")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_qemu_image_micro)

    p = sub.add_parser("qemu-image-spi")
    p.add_argument("--config", required=True)
    p.add_argument("--pbpack", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_qemu_image_spi)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
