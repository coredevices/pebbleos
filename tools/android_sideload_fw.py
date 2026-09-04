#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Sideload a firmware .pbz onto a Pebble via the Android companion app."""

import argparse
import glob
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REMOTE_PATH = "/data/local/tmp/firmware.pbz"
ACTION = "coredevices.pebble.SIDELOAD_FIRMWARE"
COMPONENT = "coredevices.coreapp/coredevices.pebble.firmware.FirmwareSideloadReceiver"


def newest_pbz():
    matches = glob.glob(os.path.join(REPO_ROOT, "build", "normal_*_slot0.pbz"))
    return max(matches, key=os.path.getmtime) if matches else None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("pbz", nargs="?", help="Path to .pbz (defaults to newest in build/)")
    args = p.parse_args()

    pbz = args.pbz or newest_pbz()
    if not pbz or not os.path.isfile(pbz):
        sys.exit("no .pbz found — pass one, or run `pbl build` first")

    print(f"sideloading {pbz}")
    subprocess.run(["adb", "push", pbz, REMOTE_PATH], check=True)
    subprocess.run(
        ["adb", "shell", "am", "broadcast",
         "-a", ACTION, "-n", COMPONENT, "--es", "path", REMOTE_PATH],
        check=True,
    )


if __name__ == "__main__":
    main()
