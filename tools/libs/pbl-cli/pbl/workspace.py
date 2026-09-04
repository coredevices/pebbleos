# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""The PebbleOS checkout the CLI is operating on.

The workspace is located by walking up from the current directory looking
for ``pbl.yml``, which also carries the directory layout and the list of
out-of-tree extension commands. Nothing about the checkout is baked into
the CLI itself.
"""

import importlib.util
import os
import sys
from dataclasses import dataclass

import yaml

from pbl import util

WORKSPACE_FILE = "pbl.yml"


class WorkspaceError(Exception):
    pass


@dataclass(frozen=True)
class ExtensionSpec:
    """An out-of-tree command declared in ``pbl.yml``."""

    name: str
    help: str
    path: str
    class_name: str

    def load(self):
        """Import the extension's file and instantiate its command class."""
        module_name = "pbl.commands.ext." + self.name.replace("-", "_")
        spec = importlib.util.spec_from_file_location(module_name, self.path)
        if spec is None or spec.loader is None:
            raise WorkspaceError(f"cannot load extension command from {self.path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as e:
            raise WorkspaceError(f"failed to load {self.path}: {e}") from e

        cls = getattr(module, self.class_name, None)
        if cls is None:
            raise WorkspaceError(
                f"{self.path} does not define a class named {self.class_name}"
            )
        return cls()


class Workspace:
    def __init__(self, topdir, data):
        self.topdir = topdir
        self._data = data or {}

    # --- discovery --------------------------------------------------------

    @classmethod
    def find(cls, start=None):
        """Walk up from ``start`` (default: the current directory) looking
        for the workspace file."""
        path = os.path.abspath(start or os.getcwd())
        while True:
            candidate = os.path.join(path, WORKSPACE_FILE)
            if os.path.isfile(candidate):
                return cls(path, cls._read(candidate))
            parent = os.path.dirname(path)
            if parent == path:
                raise WorkspaceError(
                    f"not inside a PebbleOS checkout: no {WORKSPACE_FILE} found "
                    "in this directory or any parent"
                )
            path = parent

    @staticmethod
    def _read(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        if data is None:
            return {}
        if not isinstance(data, dict):
            raise WorkspaceError(f"invalid {path}: expected a mapping")
        return data

    # --- layout -----------------------------------------------------------

    def _section(self, name):
        section = self._data.get(name, {})
        if not isinstance(section, dict):
            raise WorkspaceError(f"invalid {WORKSPACE_FILE}: {name} must be a mapping")
        return section

    def path(self, *parts):
        """Absolutize a workspace-relative path."""
        return os.path.join(self.topdir, *parts)

    def _dir(self, section, key, env, default):
        value = os.environ.get(env) or self._section(section).get(key, default)
        return os.path.abspath(os.path.join(self.topdir, value))

    @property
    def build_dir(self):
        return self._dir("build", "dir", "PBL_BUILD_DIR", "build")

    @property
    def test_dir(self):
        return self._dir("build", "test-dir", "PBL_TEST_DIR", "build-test")

    @property
    def lang_dir(self):
        return self._dir("lang", "dir", "PBL_LANG_DIR", "resources/normal/base/lang")

    # --- extensions -------------------------------------------------------

    def extensions(self):
        """The extension commands declared by ``pbl.yml``."""
        declared = self._data.get("commands", []) or []
        if not isinstance(declared, list):
            raise WorkspaceError(f"invalid {WORKSPACE_FILE}: commands must be a list")

        specs = []
        for entry in declared:
            if not isinstance(entry, dict) or "file" not in entry:
                raise WorkspaceError(
                    f"invalid {WORKSPACE_FILE}: every commands entry needs a file"
                )
            path = os.path.join(self.topdir, entry["file"])
            for command in entry.get("commands", []) or []:
                try:
                    name = command["name"]
                    class_name = command["class"]
                except (TypeError, KeyError) as e:
                    raise WorkspaceError(
                        f"invalid {WORKSPACE_FILE}: {entry['file']} commands need "
                        "a name and a class"
                    ) from e
                specs.append(
                    ExtensionSpec(
                        name=name,
                        help=command.get("help", f"{name} (extension command)"),
                        path=path,
                        class_name=class_name,
                    )
                )
        return specs

    # --- runtime ----------------------------------------------------------

    def activate(self):
        """Make the checkout's ``tools`` package importable and prefer an
        installed SDK's binaries (toolchain, QEMU, sftool)."""
        if self.topdir not in sys.path:
            sys.path.insert(0, self.topdir)

        try:
            from tools.pebble_sdk_locator import activate_sdk
        except ImportError:
            return
        try:
            activate_sdk(self.topdir)
        except OSError as e:
            util.wrn(f"could not activate the installed SDK: {e}")
