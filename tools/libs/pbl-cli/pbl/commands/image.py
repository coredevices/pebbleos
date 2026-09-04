# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import shlex
import sys

from pbl.command import PblCommand


class _ImageCommand(PblCommand):
    """Writes one partition over the serial imaging protocol."""

    group = "device"
    partition = None

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument("--tty", help="tty for serial imaging")
        parser.add_argument(
            "--force-pulse",
            action="store_true",
            help="Force PULSE-based imaging even where sftool would be used",
        )
        return parser

    def _tool(self, build, args):
        """The imaging tool this build's SoC and protocol call for."""
        if build.config.CONFIG_SOC_SF32LB52 and not args.force_pulse:
            return "sftool_flash_imaging"
        if build.config.CONFIG_PULSE_EVERYWHERE or args.force_pulse:
            return "pulse_flash_imaging"
        return "pulse_legacy_flash_imaging"

    def _image(self, build, args, path):
        if not args.tty:
            self.die("no device to image: pass --tty")

        tool = self.script("tools", self._tool(build, args) + ".py")
        self.inf(f'writing "{path}" to {args.tty}')
        self.check_shell(
            f"{shlex.quote(sys.executable)} {tool} -t {shlex.quote(args.tty)} "
            f"-p {self.partition} {shlex.quote(path)}",
            "imaging failed",
        )


class ImageResources(_ImageCommand):
    partition = "resources"

    def __init__(self):
        super().__init__("image_resources", "Image the system resources over serial")

    def do_run(self, args, unknown):
        build = self.build_dir()
        self._image(build, args, build.pbpack)


class ImageRecovery(_ImageCommand):
    partition = "firmware"

    def __init__(self):
        super().__init__("image_recovery", "Image the recovery firmware over serial")

    def do_add_parser(self, parser_adder):
        parser = super().do_add_parser(parser_adder)
        parser.add_argument(
            "--file",
            type=os.path.abspath,
            help="Binary to write (default: the build's firmware)",
        )
        return parser

    def do_run(self, args, unknown):
        build = self.build_dir()
        self._image(build, args, args.file or build.bin)
