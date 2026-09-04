# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import shlex
import sys

from pbl.command import PblCommand
from pbl.emulator import CONSOLE_PORT

DEFAULT_BAUDRATE = 115200


class Console(PblCommand):
    group = "device"

    def __init__(self):
        super().__init__(
            "console",
            "Open the firmware console",
            "Attach to the firmware's log console, over serial on a real "
            "device and over TCP under the emulator.",
        )

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument("--tty", help="tty to use for the console")
        parser.add_argument(
            "--baudrate", type=int, help="Baudrate for the target's uart"
        )
        parser.add_argument(
            "--qemu-host",
            default=f"localhost:{CONSOLE_PORT}",
            help="host:port for the emulator's console (default: %(default)s)",
        )
        parser.add_argument(
            "--reconnect",
            action="store_true",
            help="Wrap the console in a keep-alive driver that reconnects",
        )
        return parser

    def do_run(self, args, unknown):
        build = self.build_dir()
        emulated = bool(build.config.CONFIG_QEMU)

        if emulated:
            tty = f"socket://{args.qemu_host}"
        elif args.tty:
            tty = args.tty
        else:
            self.die("no console to attach to: pass --tty")

        python = shlex.quote(sys.executable)
        if build.config.CONFIG_PULSE_EVERYWHERE:
            console = f"{python} {self.script('tools', 'pulse_console.py')} -t {tty}"
        else:
            miniterm = self.script("tools", "log_hashing", "miniterm_co.py")
            console = f"{python} {miniterm} {tty}"
            if not emulated:
                # Force RTS de-asserted: on some boards RTS resets the SoC.
                console += f" {args.baudrate or DEFAULT_BAUDRATE} --rts 0"

        if args.reconnect:
            keepalive = self.script("tools", "console_keepalive.py")
            console = f"{python} {keepalive} -t {tty} -- {console}"

        return self.run_shell(console)
