# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import sys

from pbl.command import PblCommand
from pbl.device import run_on_device, runner_options
from pbl.emulator import GDB_PORT

# The port qemu_gdb_proxy.py listens on for gdb; it forwards to the emulator.
GDB_PROXY_PORT = 1233


class Debug(PblCommand):
    group = "device"

    def __init__(self):
        super().__init__("debug", "Attach gdb to the firmware")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        runner_options(parser)
        return parser

    def do_run(self, args, unknown):
        build = self.build_dir()
        if not build.config.CONFIG_QEMU:
            run_on_device(build, args, "debug")
            return

        import pexpect

        proxy = self.script("tools", "qemu", "qemu_gdb_proxy.py")
        cmd = (
            f"{sys.executable} {proxy} --port={GDB_PROXY_PORT} "
            f"--target=localhost:{GDB_PORT}"
        )
        if self.dry_run:
            self.inf("[dry-run]", cmd, color="yellow")
            return

        proc = pexpect.spawn(cmd, logfile=sys.stdout, encoding="utf-8")
        proc.expect(["Connected to target", pexpect.TIMEOUT], timeout=10)
        self._gdb(build.elf, GDB_PROXY_PORT)

    def _gdb(self, elf, port):
        from tools.gdb_driver import find_gdb_path

        gdb = find_gdb_path()
        if gdb is None:
            self.die("pebble-gdb not found")
        os.system(f'{gdb} {elf} --ex="target remote :{port}"')
