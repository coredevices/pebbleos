# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Base class every ``pbl`` command derives from.

Modeled on Zephyr's west: a command declares its name and help text, adds
its own arguments in :meth:`do_add_parser` and does its work in
:meth:`do_run`. The core takes care of locating the workspace, the build
directory and the shared options.
"""

import argparse
import os
import shlex
import subprocess
from abc import ABC, abstractmethod

from pbl import util
from pbl.errors import CommandContextError, CommandError

__all__ = ["CommandContextError", "CommandError", "PblCommand", "global_options"]

# Command groups, in the order the top-level help lists them.
GROUPS = ("build", "device", "emulator", "i18n", "other")


def global_options():
    """Options every command accepts, before or after the command name.

    argparse would otherwise let a subparser's default overwrite what the
    top-level parser already saw, so nothing has a default here; the
    accessors on :class:`PblCommand` supply them.
    """
    parser = argparse.ArgumentParser(add_help=False)
    group = parser.add_argument_group("shared options")
    group.add_argument(
        "-b",
        "--build-dir",
        metavar="DIR",
        default=argparse.SUPPRESS,
        help="Build directory to use (default: the workspace's)",
    )
    group.add_argument(
        "--dry-run",
        action="store_true",
        default=argparse.SUPPRESS,
        help="Print the commands that would run instead of running them",
    )
    return parser


class PblCommand(ABC):
    #: Which section of the top-level help the command is listed under.
    group = "other"

    def __init__(self, name, help, description=None, accepts_unknown_args=False):
        self.name = name
        self.help = help
        self.description = description or help
        self.accepts_unknown_args = accepts_unknown_args
        self.parser = None
        self.workspace = None
        self.args = None

    # --- wiring -----------------------------------------------------------

    def add_parser(self, parser_adder):
        self.parser = self.do_add_parser(parser_adder)
        if self.parser is None:
            raise ValueError(f"{self.name}: do_add_parser() must return a parser")
        return self.parser

    def add_subparser(self, parser_adder, **kwargs):
        """Register this command's subparser with the shared options on it.

        No ``help`` is passed on: the top-level help lists the commands by
        group itself, and argparse would otherwise list them again.
        """
        kwargs.setdefault("description", self.description)
        kwargs.setdefault("parents", [])
        kwargs["parents"] = [global_options()] + list(kwargs["parents"])
        return parser_adder.add_parser(self.name, **kwargs)

    def run(self, args, unknown, workspace):
        if unknown and not self.accepts_unknown_args:
            self.parser.error(f"unrecognized arguments: {' '.join(unknown)}")
        self.workspace = workspace
        self.args = args
        return self.do_run(args, unknown) or 0

    @abstractmethod
    def do_add_parser(self, parser_adder):
        """Register the command's subparser and return it."""

    @abstractmethod
    def do_run(self, args, unknown):
        """Do the work. Return an exit status, or raise CommandError."""

    # --- context ----------------------------------------------------------

    @property
    def topdir(self):
        return self.workspace.topdir

    @property
    def dry_run(self):
        return getattr(self.args, "dry_run", False)

    def _build_dir(self, default):
        from pbl.build import BuildDir

        override = getattr(self.args, "build_dir", None)
        return BuildDir(override or default, self.topdir)

    def build_dir(self, configured=True):
        """The firmware build directory."""
        build = self._build_dir(self.workspace.build_dir)
        if configured:
            build.ensure_configured()
        return build

    def test_dir(self):
        """The unit tests' build directory, a CMake project of its own."""
        return self._build_dir(self.workspace.test_dir)

    # --- running things ---------------------------------------------------

    def run_cmd(self, cmd, cwd=None, **kwargs):
        """Run an argv list from the workspace root. Returns its status."""
        if self.dry_run:
            util.inf("[dry-run]", shlex.join(str(a) for a in cmd), color="yellow")
            return 0
        return subprocess.call(
            [str(a) for a in cmd], cwd=cwd or self.topdir, **kwargs
        )

    def run_shell(self, cmd, cwd=None, **kwargs):
        """Run a shell command string from the workspace root."""
        if self.dry_run:
            util.inf("[dry-run]", cmd, color="yellow")
            return 0
        return subprocess.call(cmd, shell=True, cwd=cwd or self.topdir, **kwargs)

    def check_cmd(self, cmd, msg=None, **kwargs):
        rc = self.run_cmd(cmd, **kwargs)
        if rc:
            raise CommandError(msg, returncode=rc)
        return rc

    def check_shell(self, cmd, msg=None, **kwargs):
        rc = self.run_shell(cmd, **kwargs)
        if rc:
            raise CommandError(msg, returncode=rc)
        return rc

    def cmake_build(self, build, *targets, msg=None):
        """Build the given targets (or the default one) in ``build``."""
        cmd = ["cmake", "--build", str(build)]
        for target in targets:
            cmd += ["--target", target]
        return self.check_cmd(cmd, msg)

    def script(self, *parts):
        """Absolute path to a helper script shipped in the checkout."""
        return os.path.join(self.topdir, *parts)

    # --- output -----------------------------------------------------------

    inf = staticmethod(util.inf)
    wrn = staticmethod(util.wrn)

    @staticmethod
    def die(msg, returncode=1):
        raise CommandError(msg, returncode=returncode)
