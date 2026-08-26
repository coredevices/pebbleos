# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Core binary-runner abstraction, modeled on Zephyr's west runners.

A runner knows how to talk to a connected device for a small set of commands
(flash/run/reset/erase/debug/debugserver). Frontends (the ./pbl CLI and the waf
build) build a :class:`RunnerConfig`, register runner-specific CLI arguments via
``do_add_parser()``, instantiate the selected runner with ``create()`` and
dispatch a command with ``run()``.
"""

import shlex
import subprocess
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import NamedTuple

# Commands a runner may implement.
COMMANDS = ("flash", "run", "reset", "erase", "debug", "debugserver")


class RunnerError(Exception):
    pass


class UnsupportedOperation(RunnerError):
    pass


@dataclass
class RunnerCaps:
    """Declares what a runner class can do."""

    commands: set = field(default_factory=lambda: {"flash"})
    # Whether flash() can also program system resources.
    flash_resources: bool = False

    def __post_init__(self):
        invalid = self.commands - set(COMMANDS)
        if invalid:
            raise ValueError(f"invalid runner commands: {sorted(invalid)}")


class RunnerConfig(NamedTuple):
    """Inputs a runner needs to execute a command, independent of the frontend."""

    board_dir: str  # boards/<board>
    soc: str | None = None  # CONFIG_SOC, e.g. "NRF52", "SF32LB52"
    hex_file: str | None = None
    elf_file: str | None = None
    resources_file: str | None = None
    dry_run: bool = False


class Runner(ABC):
    name = None

    def __init__(self, cfg):
        self.cfg = cfg

    @classmethod
    def capabilities(cls):
        return RunnerCaps()

    @classmethod
    def do_add_parser(cls, parser):
        """Add runner-specific CLI arguments. Override as needed."""

    @classmethod
    def add_parser(cls, parser):
        cls.do_add_parser(parser)

    @classmethod
    def create(cls, cfg, args):
        return cls.do_create(cfg, args)

    @classmethod
    def do_create(cls, cfg, args):
        return cls(cfg)

    def run(self, command):
        caps = self.capabilities()
        if command not in caps.commands:
            raise UnsupportedOperation(
                f"{self.name} runner does not support {command}"
            )
        if command == "flash" and self.cfg.resources_file and not caps.flash_resources:
            raise UnsupportedOperation(
                f"{self.name} runner does not support flashing resources"
            )
        self.do_run(command)

    @abstractmethod
    def do_run(self, command):
        """Concrete runner; run() delegates here once the command is validated."""

    def call(self, cmd):
        """Run a shell command, honoring dry-run."""
        if self.cfg.dry_run:
            print("[dry-run] " + cmd)
            return 0
        print(cmd)
        return subprocess.call(cmd, shell=True)

    @staticmethod
    def quote(*args):
        return " ".join(shlex.quote(str(a)) for a in args)
