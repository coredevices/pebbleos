#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""QEMU flash images from built firmware artifacts.

micro: firmware hex -> qemu_micro_flash.bin (padded to a 512-byte boundary)
spi:   system resources pbpack placed at the board's resource offset in a
       0xff-padded SPI flash image (qemu_spi_flash.bin)

Driven by build/build-info.json; needs a completed build.
"""

import argparse
import os

import build_info
from intelhex import IntelHex


def make_micro_image(build_dir, artifacts):
    fw_hex = os.path.join(build_dir, artifacts["hex"])
    out_path = os.path.join(build_dir, "qemu_micro_flash.bin")
    print(f"Writing micro flash image to {out_path}")

    img = IntelHex(fw_hex)
    img.padding = 0xFF
    flash_end = ((img.maxaddr() + 511) // 512) * 512
    img.tobinfile(out_path, start=0x00000000, end=flash_end - 1)


def make_spi_image(build_dir, artifacts, config):
    if config.get("CONFIG_QEMU"):
        # QEMU generic boards: resources at offset 0x620000 in 32MB flash
        resources_begin = 0x620000
        image_size = 0x2000000
    else:
        resources_begin = 0x280000
        image_size = 0x400000

    out_path = os.path.join(build_dir, "qemu_spi_flash.bin")
    print(f"Writing SPI flash image to {out_path}")
    with open(os.path.join(build_dir, artifacts["pbpack"]), "rb") as f:
        res_img = f.read()

    with open(out_path, "wb") as f:
        # Pad the first section before system resources with FF's
        f.write(bytes([0xFF]) * resources_begin)
        f.write(res_img)
        # Pad with 0xFF up to image size
        f.write(bytes([0xFF]) * (image_size - resources_begin - len(res_img)))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-dir", required=True, help="Configured build directory")
    parser.add_argument(
        "images",
        nargs="*",
        choices=["micro", "spi"],
        help="Which images to build (default: both)",
    )
    args = parser.parse_args()
    if not args.images:
        args.images = ["micro", "spi"]

    info = build_info.load_build_info(args.build_dir)
    if "micro" in args.images:
        make_micro_image(args.build_dir, info["artifacts"])
    if "spi" in args.images:
        make_spi_image(args.build_dir, info["artifacts"], info["config"])


if __name__ == "__main__":
    main()
