# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os

from pbl.command import PblCommand


class Configure(PblCommand):
    group = "build"

    def __init__(self):
        super().__init__(
            "configure",
            "Configure the firmware build directory",
            "Run CMake for a board. Anything this command does not recognize "
            "is passed straight to CMake, so Kconfig symbols can be overridden "
            "with -DCONFIG_FOO=y.",
            accepts_unknown_args=True,
        )

    def _boards(self):
        """The boards to offer, or None when they cannot be listed (which
        is not fatal: --board still reaches CMake)."""
        try:
            from tools import boards

            return boards.available_boards(self.workspace.topdir)
        except (AttributeError, ImportError, OSError, TypeError, ValueError):
            return None

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        available = self._boards()
        parser.add_argument(
            "--board",
            required=True,
            metavar="BOARD",
            choices=available,
            help="Board to build for"
            + (f" ({', '.join(available)})" if available else ""),
        )
        parser.add_argument(
            "--variant",
            default="normal",
            choices=["normal", "prf"],
            help="Firmware variant (default: normal)",
        )
        parser.add_argument(
            "-G",
            "--generator",
            default="Ninja",
            help="CMake generator (default: Ninja)",
        )
        return parser

    def do_run(self, args, unknown):
        build = self.build_dir(configured=False)
        if os.path.exists(build.join("menuconfig.conf")):
            self.wrn(f"applying the Kconfig changes from {build.join('menuconfig.conf')}")

        return self.run_cmd(
            [
                "cmake",
                "-B",
                str(build),
                "-S",
                self.topdir,
                "-G",
                args.generator,
                f"-DBOARD={args.board}",
                f"-DVARIANT={args.variant}",
            ]
            + unknown
        )
