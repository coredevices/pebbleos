#!/usr/bin/env python
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Fix bundled library paths on macOS QEMU binaries."""

import argparse
import os
import subprocess
import sys

LIB_PATH_FIXES = [
    ("/usr/local/opt/pixman/lib/libpixman-1.0.dylib", "libpixman-1.0.dylib"),
    ("/usr/local/opt/sdl2/lib/libSDL2-2.0.0.dylib", "libSDL2-2.0.0.dylib"),
    ("/usr/local/opt/glib/lib/libgthread-2.0.0.dylib", "libgthread-2.0.0.dylib"),
    ("/usr/local/opt/glib/lib/libglib-2.0.0.dylib", "libglib-2.0.0.dylib"),
    ("/usr/local/opt/gettext/lib/libintl.8.dylib", "libintl.8.dylib"),
    ("/usr/local/opt/pcre2/lib/libpcre2-8.0.dylib", "libpcre2-8.0.dylib"),
]

CELLAR_PATTERNS = [
    ("/usr/local/Cellar/glib/", "libglib-2.0.0.dylib", "libglib-2.0.0.dylib"),
]


def fix_paths(qemu_bin: str, lib_dir: str | None = None, dry_run: bool = False) -> int:
    if sys.platform != "darwin":
        print("Error: this script only runs on macOS.", file=sys.stderr)
        return 1

    if lib_dir is None:
        lib_dir = os.path.join(os.path.dirname(qemu_bin), "..", "lib")

    if not os.path.isdir(lib_dir):
        print(f"Error: lib dir not found: {lib_dir}", file=sys.stderr)
        return 1

    def run(cmd):
        print("  $", " ".join(cmd))
        if not dry_run:
            subprocess.run(cmd, check=True)

    # Fix qemu binary
    print(f"Fixing binary: {qemu_bin}")
    otool_output = subprocess.check_output(["otool", "-L", qemu_bin], text=True)
    for old_path, lib_name in LIB_PATH_FIXES:
        if old_path in otool_output:
            new_path = "@executable_path/../lib/" + lib_name
            run(["install_name_tool", "-change", old_path, new_path, qemu_bin])

    # Fix bundled dylibs
    print(f"\nFixing dylibs in: {lib_dir}")
    for dylib in os.listdir(lib_dir):
        if not dylib.endswith(".dylib"):
            continue
        dylib_path = os.path.join(lib_dir, dylib)
        otool_output = subprocess.check_output(["otool", "-L", dylib_path], text=True)
        lines = otool_output.split("\n")

        # Fix library's own ID
        for old_path, lib_name in LIB_PATH_FIXES:
            if old_path in lines[0]:
                run(
                    ["install_name_tool", "-id", "@loader_path/" + lib_name, dylib_path]
                )
                break

        # Fix references to other bundled libraries
        for old_path, lib_name in LIB_PATH_FIXES:
            if old_path in otool_output:
                run(
                    [
                        "install_name_tool",
                        "-change",
                        old_path,
                        "@loader_path/" + lib_name,
                        dylib_path,
                    ]
                )

        # Fix Cellar paths
        for line in lines:
            for cellar_prefix, cellar_match, lib_name in CELLAR_PATTERNS:
                if cellar_prefix in line and cellar_match in line:
                    old_path = line.strip().split()[0]
                    run(
                        [
                            "install_name_tool",
                            "-change",
                            old_path,
                            "@loader_path/" + lib_name,
                            dylib_path,
                        ]
                    )

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Fix bundled dylib paths in a macOS QEMU installation."
    )
    parser.add_argument("qemu_bin", help="Path to the qemu-system-arm binary")
    parser.add_argument(
        "--lib-dir",
        default=None,
        help="Path to bundled lib dir (default: <qemu_bin>/../../lib)",
    )
    parser.add_argument(
        "--dry-run",
        "-n",
        action="store_true",
        help="Print install_name_tool commands without executing them",
    )
    args = parser.parse_args()

    try:
        sys.exit(fix_paths(args.qemu_bin, args.lib_dir, args.dry_run))
    except (subprocess.CalledProcessError, FileNotFoundError, OSError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
