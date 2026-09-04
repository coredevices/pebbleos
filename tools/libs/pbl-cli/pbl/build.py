# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""What the CLI knows about a configured build directory.

Everything is read back from the build's own byproducts -- ``.config`` for
the Kconfig symbols and ``CMakeCache.txt`` for the board, the variant and
the project name -- so the CLI never has to be kept in sync with the build
by hand.
"""

import os

from pbl.errors import CommandContextError

CACHE_FILE = "CMakeCache.txt"
DOTCONFIG_FILE = ".config"

# CONFIG_PLATFORM_* -> the SDK platform name, mirroring cmake/modules/platform.cmake.
_PLATFORMS = ("emery", "flint", "gabbro")


def _parse_dotconfig(path):
    """Read a Kconfig ``.config`` into a dict of native Python values."""
    config = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if value == "y":
                value = True
            elif value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            else:
                try:
                    value = int(value, 0)
                except ValueError:
                    pass
            config[key] = value
    return config


def _parse_cmake_cache(path):
    """Read the ``KEY:TYPE=VALUE`` entries of a CMakeCache.txt."""
    cache = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(("#", "//")) or ":" not in line:
                continue
            entry, _, value = line.partition("=")
            key, _, _type = entry.partition(":")
            if key:
                cache[key] = value
    return cache


class Config:
    """The build's Kconfig symbols, plus the few facts derived from them.

    Missing keys read back as an empty list: the ConfigSet semantics the
    resource generators this CLI reuses were written against.
    """

    def __init__(self, data):
        self._data = data

    def __getattr__(self, key):
        return self._data.get(key, [])

    def __getitem__(self, key):
        return self._data.get(key, [])

    def __contains__(self, key):
        return key in self._data

    def get(self, key, default=None):
        return self._data.get(key, default)


class BuildDir:
    """A CMake build directory, configured or not."""

    def __init__(self, path, topdir):
        self.path = os.path.abspath(path)
        self.topdir = topdir
        self._config = None
        self._cache = None
        self._board = None

    def __str__(self):
        return self.path

    def join(self, *parts):
        return os.path.join(self.path, *parts)

    # --- state ------------------------------------------------------------

    @property
    def configured(self):
        return os.path.isfile(self.join(CACHE_FILE))

    def ensure_configured(self):
        if not self.configured:
            raise CommandContextError(
                f"{self.path} is not configured -- run 'pbl configure --board BOARD' first"
            )

    @property
    def cache(self):
        if self._cache is None:
            self.ensure_configured()
            self._cache = _parse_cmake_cache(self.join(CACHE_FILE))
        return self._cache

    @property
    def config(self):
        if self._config is None:
            self.ensure_configured()
            path = self.join(DOTCONFIG_FILE)
            if not os.path.isfile(path):
                raise CommandContextError(f"no {DOTCONFIG_FILE} in {self.path}")
            data = _parse_dotconfig(path)
            for platform in _PLATFORMS:
                if data.get(f"CONFIG_PLATFORM_{platform.upper()}"):
                    data["PLATFORM_NAME"] = platform
                    break
            self._config = Config(data)
        return self._config

    # --- what was configured ----------------------------------------------

    @property
    def board(self):
        """The board target as given to configure, e.g. ``obelix@pvt``."""
        board = self.cache.get("BOARD")
        if not board:
            raise CommandContextError(f"no board recorded in {self.join(CACHE_FILE)}")
        return board

    @property
    def board_spec(self):
        """The board's manifest: name, revision and supported runners."""
        if self._board is None:
            from tools import boards

            try:
                self._board = boards.parse_board(self.topdir, self.board)
            except (TypeError, ValueError) as e:
                raise CommandContextError(str(e)) from e
        return self._board

    @property
    def variant(self):
        return self.cache.get("VARIANT", "normal")

    @property
    def generator(self):
        return self.cache.get("CMAKE_GENERATOR", "Ninja")

    # --- artifacts --------------------------------------------------------

    @property
    def project(self):
        return self.cache.get("CMAKE_PROJECT_NAME", "pebbleos")

    @property
    def elf(self):
        return self.join(f"{self.project}.elf")

    @property
    def hex(self):
        return self.join(f"{self.project}.hex")

    @property
    def bin(self):
        return self.join(f"{self.project}.bin")

    @property
    def pbpack(self):
        return self.join("system_resources.pbpack")
