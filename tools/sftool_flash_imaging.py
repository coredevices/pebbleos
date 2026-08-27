#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0
#
# Flash imaging for SF32LB52 (Obelix/Getafix): writes firmware or resources
# directly via sftool to the correct flash addresses. No PULSE descriptor is
# prepended, so recovery (PRF) stays in PBLBOOT format for the bootloader.
#
# Same CLI as pulse_flash_imaging / pulse_legacy_flash_imaging so waf can
# select this tool via _get_pulse_flash_tool() for MICRO_FAMILY == 'SF32LB52'.


import argparse
import subprocess
import sys

# SAFE_FIRMWARE (recovery/PRF) and SYSTEM_RESOURCES for SF32LB52 (gd25q256e layout)
SAFE_FIRMWARE_ADDR = 0x12A20000
SYSTEM_RESOURCES_ADDR = 0x12620000


def run_sftool(tty, path, address, chip="SF32LB52"):
    cmd = ["sftool", "-c", chip, "-p", tty, "write_flash", f"{path}@0x{address:x}"]
    r = subprocess.call(cmd)
    return r == 0


def main():
    parser = argparse.ArgumentParser(
        description="Load firmware or resources into flash via sftool (SF32LB52)."
    )
    parser.add_argument(
        "-t", "--tty", required=True, help="Serial port (e.g. /dev/ttyACM0)"
    )
    parser.add_argument("-c", "--chip", default="SF32LB52", help="Chip for sftool -c")
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="(ignored, for CLI compatibility)"
    )
    parser.add_argument(
        "-p", "--progress", action="store_true", help="(ignored, for CLI compatibility)"
    )
    subparsers = parser.add_subparsers(dest="which", required=True)

    fw = subparsers.add_parser(
        "firmware", help="load recovery firmware into SAFE_FIRMWARE"
    )
    fw.add_argument("file", help="path to PRF .bin")
    res = subparsers.add_parser(
        "resources", help="load resources into SYSTEM_RESOURCES"
    )
    res.add_argument("file", help="path to .pbpack")

    args = parser.parse_args()

    if args.which == "firmware":
        addr = SAFE_FIRMWARE_ADDR
        name = "recovery (SAFE_FIRMWARE)"
    else:
        addr = SYSTEM_RESOURCES_ADDR
        name = "resources (SYSTEM_RESOURCES)"

    print(f"Writing {name} to 0x{addr:x} via sftool...")
    if run_sftool(args.tty, args.file, addr, chip=args.chip):
        print("Success!")
    else:
        print("Fail!")
        sys.exit(1)


if __name__ == "__main__":
    main()
