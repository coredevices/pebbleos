# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import contextlib
import errno
import os
import re
import socket
import subprocess
import sys

from .core import Runner, RunnerCaps, RunnerError

OPENOCD_TELNET_PORT = 4444
OPENOCD_GDB_PORT = 3333
OPENOCD_LOG = ".waf.openocd.log"


def _is_openocd_running():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("", OPENOCD_TELNET_PORT))
        s.close()
    except OSError as e:
        s.close()
        return e.errno == errno.EADDRINUSE
    return False


class OpenOcdRunner(Runner):
    name = "openocd"

    @classmethod
    def capabilities(cls):
        return RunnerCaps(
            commands={"flash", "run", "reset", "erase", "debug", "debugserver"}
        )

    @property
    def _cfg_file(self):
        return os.path.join(self.cfg.board_dir, "support", "openocd.cfg")

    def do_run(self, command):
        if command == "flash":
            self._run_command(
                f"init; reset halt; program {self.cfg.hex_file} reset;",
                expect=["Programming Finished", "Programming Finished", "shutdown"],
                enforce_expect=True,
            )
        elif command == "run":
            self._run_command("init; reset run;", expect=["found"])
        elif command == "reset":
            self._run_command("init; reset;", expect=["found"])
        elif command == "erase":
            self._run_command("init; reset halt;", ignore_fail=True)
            self._run_command("init; flash erase_sector 0 0 1;", ignore_fail=True)
        elif command == "debugserver":
            self._run_command("init; reset", shutdown=False)
        elif command == "debug":
            self._debug()

    def _debug(self):
        from tools.gdb_driver import find_gdb_path

        gdb = find_gdb_path()
        if gdb is None:
            raise RunnerError("pebble-gdb not found!")

        with self._daemon():
            cmd = f'{gdb} {self.cfg.elf_file} --init-command=".gdbinit" --ex="target remote :{OPENOCD_GDB_PORT}"'
            if self.cfg.dry_run:
                print("[dry-run] " + cmd)
            else:
                os.system(cmd)

    @contextlib.contextmanager
    def _daemon(self):
        if _is_openocd_running():
            yield
            return

        import pexpect

        proc = pexpect.spawn(
            "openocd", ["-f", self._cfg_file], encoding="utf-8", logfile=sys.stdout
        )
        result = proc.expect(["Listening on port", pexpect.TIMEOUT], timeout=10)
        if result != 0:
            raise RunnerError("Timed out connecting OpenOCD to development board...")
        try:
            yield
        finally:
            proc.close()

    def _run_command(
        self, cmd, ignore_fail=False, expect=(), shutdown=True, enforce_expect=False
    ):
        if _is_openocd_running():
            import telnetlib

            t = telnetlib.Telnet("", OPENOCD_TELNET_PORT)
            print(f"Sending commands to OpenOCD daemon:\n{cmd}\n...")
            t.write(b"%s\n" % cmd.encode())
            for regex in expect:
                idx, _match, _text = t.expect([regex.encode()], 40)
                if enforce_expect and idx == -1:
                    raise RunnerError(f"OpenOCD expectation '{regex}' unfulfilled")
            t.close()
            return

        if shutdown:
            if self.cfg.soc == "NRF52":
                # On nRF5, shut down the tracing modules in DEMCR, take the core
                # out of debug in DHCSR, and then shut down the AP to get back
                # down to baseline power.
                cmd = (
                    f"{cmd} ; mww 0xe000edfc 0x0; mww 0xe000edf0 0xa05f0000 ; "
                    "nrf52.dap dpreg 4 0 ; shutdown"
                )
            else:
                cmd = f"{cmd} ; shutdown"

        fail_handling = " || true " if ignore_fail else ""
        shell_cmd = f'openocd -f {self._cfg_file} -c "{cmd}" 2>&1 | tee {OPENOCD_LOG} {fail_handling}'
        if self.cfg.dry_run:
            print("[dry-run] " + shell_cmd)
            return

        subprocess.call(shell_cmd, shell=True)

        if enforce_expect:
            with open(OPENOCD_LOG, "r") as result_file:
                result = result_file.read()
            match_start = 0
            for regex in expect:
                expect_match = re.search(regex, result[match_start:])
                if not expect_match:
                    raise RunnerError(f"OpenOCD expectation '{regex}' unfulfilled")
                match_start = expect_match.end()
