# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

from pbl.command import PblCommand
from pbl.device import run_on_device, runner_options


class _DeviceCommand(PblCommand):
    """A command that is one runner operation and nothing else."""

    group = "device"
    operation = None

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        runner_options(parser)
        return parser

    def do_run(self, args, unknown):
        run_on_device(self.build_dir(), args, self.operation)


class Reset(_DeviceCommand):
    operation = "reset"

    def __init__(self):
        super().__init__("reset", "Reset a connected device")


class Run(_DeviceCommand):
    operation = "run"

    def __init__(self):
        super().__init__("run", "Start or resume execution on a connected device")


class Bork(_DeviceCommand):
    operation = "erase"

    def __init__(self):
        super().__init__("bork", "Reset and wipe a connected device")


class OpenOcd(_DeviceCommand):
    operation = "debugserver"

    def __init__(self):
        super().__init__(
            "openocd",
            "Start a debug server and leave it running",
            "Start the debug server, resetting the board first to improve the "
            "chances of attaching successfully.",
        )
