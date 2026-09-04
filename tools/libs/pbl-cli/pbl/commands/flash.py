# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

from pbl.command import PblCommand
from pbl.device import run_on_device, runner_options


class Flash(PblCommand):
    group = "device"

    def __init__(self):
        super().__init__("flash", "Flash the firmware to a connected device")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "--resources",
            action="store_true",
            help="Also flash the system resources alongside the firmware",
        )
        runner_options(parser)
        return parser

    def do_run(self, args, unknown):
        run_on_device(self.build_dir(), args, "flash", resources=args.resources)
