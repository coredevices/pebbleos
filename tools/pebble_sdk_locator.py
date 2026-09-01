# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Locate and activate an installed PebbleOS SDK.

The SDK bundles the ARM toolchain, Pebble QEMU, and sftool. When an SDK
that satisfies the minimum version declared in ``SDK_VERSION`` is found
under one of the standard install locations, its ``env.sh`` is sourced and
the resulting ``PATH`` is exported into ``os.environ`` so every subsequent
``find_program`` and runtime shell-out picks the SDK binaries up.
"""

import os
import re
import shlex
import subprocess
from pathlib import Path

try:
    from waflib import Logs
except ImportError:
    # Allow reuse outside the waf runtime (e.g. the ./pbl dev CLI).
    class Logs:
        @staticmethod
        def warn(msg, *args):
            print(msg % args if args else msg)

        @staticmethod
        def pprint(color, msg):
            print(msg)


_VERSION_DIR_RE = re.compile(r"^pebbleos-sdk-(\d+)\.(\d+)\.(\d+)$")
_SEMVER_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)")


def _parse_version(text):
    m = _SEMVER_RE.match(text.strip())
    return tuple(int(x) for x in m.groups()) if m else None


def _read_min_version(repo_root):
    p = Path(repo_root) / "SDK_VERSION"
    if not p.is_file():
        return None
    return _parse_version(p.read_text())


def _versioned_candidates(base, min_ver):
    """All ``pebbleos-sdk-X.Y.Z`` dirs under base whose version >= min, highest first."""
    if not base.is_dir():
        return []
    found = []
    try:
        for entry in base.iterdir():
            m = _VERSION_DIR_RE.match(entry.name)
            if not m or not entry.is_dir():
                continue
            v = tuple(int(x) for x in m.groups())
            if v >= min_ver:
                found.append((v, entry))
    except OSError:
        return []
    found.sort(key=lambda x: x[0], reverse=True)
    return found


def _unversioned_candidate(base):
    d = base / "pebbleos-sdk"
    return d if d.is_dir() else None


def _find_sdk(repo_root):
    """Return ``(version_or_None, sdk_dir)`` for the preferred SDK install, or None."""
    min_ver = _read_min_version(repo_root)
    if min_ver is None:
        return None

    home = Path.home()
    opt = Path("/opt")

    # Search order per spec: home versioned -> home unversioned -> opt versioned -> opt unversioned.
    for v, d in _versioned_candidates(home, min_ver):
        return (v, d)
    d = _unversioned_candidate(home)
    if d is not None:
        return (None, d)
    for v, d in _versioned_candidates(opt, min_ver):
        return (v, d)
    d = _unversioned_candidate(opt)
    if d is not None:
        return (None, d)
    return None


def activate_sdk(repo_root):
    """Activate an installed PebbleOS SDK by sourcing its env.sh into PATH.

    Returns the activated SDK directory, or None if no usable SDK was found.
    """
    if os.environ.get("PEBBLEOS_SDK_ACTIVATED"):
        return Path(os.environ["PEBBLEOS_SDK_ACTIVATED"])

    found = _find_sdk(repo_root)
    if found is None:
        return None
    version, sdk_dir = found

    env_sh = sdk_dir / "env.sh"
    if not env_sh.is_file():
        Logs.warn(f"PebbleOS SDK at {sdk_dir} has no env.sh; ignoring")
        return None

    # bash, not sh: env.sh uses ${BASH_SOURCE:-$0} to find its own location;
    # under dash $0 resolves to the parent shell, which breaks path detection.
    try:
        new_path = subprocess.check_output(
            [
                "bash",
                "-c",
                f'. {shlex.quote(str(env_sh))} >/dev/null 2>&1 && printf %s "$PATH"',
            ],
            text=True,
        )
    except subprocess.CalledProcessError as e:
        Logs.warn(f"Failed to source {env_sh}: {e}")
        return None

    if not new_path:
        return None

    os.environ["PATH"] = new_path
    os.environ["PEBBLEOS_SDK_ACTIVATED"] = str(sdk_dir)

    label = ".".join(str(x) for x in version) if version else "unversioned"
    Logs.pprint("CYAN", f"Using PebbleOS SDK ({label}) at {sdk_dir}")
    return sdk_dir
