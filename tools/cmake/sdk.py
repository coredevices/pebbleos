#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Assembles the shippable SDK, next to the shims the firmware build
already generated for it. See docs/development/sdk_export.md."""

import argparse
import json
import os
import shutil
import string
import subprocess
import sys
import zipfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SDK_ROOT = os.path.join(REPO_ROOT, "sdk")

# Copied to the SDK's common/tools, where sdk_paths puts them on the app
# build's import path.
SHARED_TOOLS = (
    "binutils.py",
    "bitmapgen.py",
    "font/__init__.py",
    "font/fontgen.py",
    "generate_appinfo.py",
    "generate_c_byte_array.py",
    "mkbundle.py",
    "pbpack.py",
    "pbpack_meta_data.py",
    "pebble_image_routines.py",
    "pebble_sdk_platform.py",
    "png2pblpng.py",
    "stm32_crc.py",
)

# .gitignore files only keep otherwise-empty directories in the checkout.
_IGNORED = shutil.ignore_patterns("__pycache__", "*.pyc", ".gitignore")


def _copy(src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)


def _copy_tree(src, dst):
    shutil.copytree(src, dst, ignore=_IGNORED, dirs_exist_ok=True)


def _copy_python_package(src, dst):
    """A package's modules, without __pycache__ or the docs sitting next to
    them."""
    def ignore(directory, names):
        return {n for n in names
                if n == "__pycache__"
                or not (n.endswith(".py")
                        or os.path.isdir(os.path.join(directory, n)))}

    shutil.copytree(src, dst, ignore=ignore, dirs_exist_ok=True)


def _substitute(src, dst, **values):
    with open(src) as f:
        text = f.read()
    for key, value in values.items():
        text = text.replace(f"@{key}@", value)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "w") as f:
        f.write(text)


def _check_templates(defaults):
    """The project files `pebble new-project` instantiates are
    string.Template sources; catch an unescaped '$' here rather than in an
    app developer's first build."""
    with open(os.path.join(defaults, "templates.json")) as f:
        templates = json.load(f)

    def walk(node):
        for value in node.values():
            if isinstance(value, dict):
                walk(value)
            elif isinstance(value, str):
                path = os.path.join(defaults, value)
                if not os.path.exists(path):
                    print(f"{value} is defined in templates.json but missing",
                          file=sys.stderr)
                    continue
                with open(path) as f:
                    try:
                        string.Template(f.read()).substitute()
                    except KeyError:
                        pass  # Expected: substitute() was given no arguments.
                    except ValueError as e:
                        raise SystemExit(
                            f"Template error in {path}:\n{e}\n"
                            "Hint: make sure to escape dollar signs! ($ => $$)")

    walk(templates)


def _build_waf(work_dir, output):
    """The waf app developers build with, carrying sdk/waftools as waflib
    extras. Its vendored wscript wants a clean directory of its own, so it
    gets a copy of the tree rather than the checkout."""
    shutil.rmtree(work_dir, ignore_errors=True)
    shutil.copytree(os.path.join(SDK_ROOT, "waf"), work_dir,
                    ignore=shutil.ignore_patterns("build", "waf", "waflib.zip",
                                                  "__pycache__", "*.pyc"))

    waftools = os.path.join(SDK_ROOT, "waftools")
    tools = sorted(os.path.join(waftools, name)
                   for name in os.listdir(waftools) if name.endswith(".py"))

    result = subprocess.run(
        [sys.executable, "waf-light", "distclean", "configure", "build",
         "--make-waf", "--tools=" + ",".join(tools)],
        cwd=work_dir, capture_output=True, text=True, check=False)
    if result.returncode:
        sys.stderr.write(result.stdout + result.stderr)
        raise SystemExit("failed to build the SDK's waf")

    _check_bundled_waftools(work_dir)
    _copy(os.path.join(work_dir, "waf"), os.path.join(output, "waf"))


def _check_bundled_waftools(work_dir):
    """waf runs everything it bundles through a minifier of its own; compile
    what came out so a mangled tool fails here and not in an app developer's
    first build."""
    with zipfile.ZipFile(os.path.join(work_dir, "zip", "waflib.zip")) as z:
        for name in sorted(z.namelist()):
            if not name.endswith(".py"):
                continue
            try:
                compile(z.read(name).decode(), name, "exec")
            except SyntaxError as e:
                raise SystemExit(f"the bundled {name} does not compile: {e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="SDK root to assemble")
    parser.add_argument("--platform", required=True, help="SDK platform name")
    parser.add_argument("--work-dir", required=True,
                        help="Scratch directory the bundled waf is built in")
    args = parser.parse_args()

    common = os.path.join(args.output, "common")
    platform_dir = os.path.join(args.output, args.platform)

    # Everything but the platform directory, which holds the shims the
    # firmware build generated.
    shutil.rmtree(common, ignore_errors=True)

    _copy(os.path.join(SDK_ROOT, "sdk_requirements.txt"),
          os.path.join(args.output, "requirements.txt"))
    _copy(os.path.join(SDK_ROOT, "sdk_package.json"),
          os.path.join(args.output, "package.json"))
    _copy(os.path.join(SDK_ROOT, "use_requirements.json"),
          os.path.join(args.output, "use_requirements.json"))

    _substitute(os.path.join(SDK_ROOT, "Doxyfile-SDK.template"),
                os.path.join(platform_dir, "Doxyfile-SDK.auto"),
                TINTIN_ROOT=REPO_ROOT, PLATFORM_PATH=platform_dir)

    _copy_tree(os.path.join(SDK_ROOT, "include"), os.path.join(common, "include"))
    _copy(os.path.join(SDK_ROOT, "pebble_app.ld.template"),
          os.path.join(common, "pebble_app.ld.template"))

    _check_templates(os.path.join(SDK_ROOT, "defaults"))
    _copy_tree(os.path.join(SDK_ROOT, "defaults"), os.path.join(common, "templates"))

    _copy_tree(os.path.join(SDK_ROOT, "tools"), os.path.join(common, "tools"))
    for tool in SHARED_TOOLS:
        _copy(os.path.join(REPO_ROOT, "tools", tool),
              os.path.join(common, "tools", tool))
    # The resource pipeline the app build shares with the firmware. It goes
    # over whole: resource_generator loads the per-type generators by name,
    # so a hand-picked subset breaks on whichever type nobody tried.
    _copy_python_package(os.path.join(REPO_ROOT, "tools", "resources"),
                         os.path.join(common, "waftools", "resources"))

    _build_waf(args.work_dir, args.output)


if __name__ == "__main__":
    main()
