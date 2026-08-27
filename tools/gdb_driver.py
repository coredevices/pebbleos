# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import logging
import subprocess
import time


def find_gdb_path():
    """Find the first arm gdb on our path"""
    prioritized_names = ["pebble-gdb", "arm-none-eabi-gdb-py", "arm-none-eabi-gdb"]
    for name in prioritized_names:
        try:
            which_all_cmd = "which %s" % name
            out = subprocess.check_output(which_all_cmd, shell=True, encoding="utf-8")
        except subprocess.CalledProcessError as e:
            if e.returncode == 1:
                continue  # `which` returns with 1 when nothing is found
            raise e
        path = out.splitlines()[0]
        logging.info(f"Found {name} at {path}")
        return path
    return None


class GDBDriver:
    def __init__(self, elf_path, gdb_path=None, server_port=1234):
        self.gdb_path = gdb_path or find_gdb_path()
        if not self.gdb_path:
            raise Exception(
                "pebble-gdb not found on your path, nor"
                " was it specified using the `gdb_path` argument"
            )
        self.elf_path = elf_path
        self.server_port = server_port
        self.pipe = None
        self.interface = GDBInterface(self)

    def _gdb_command(self):
        cmd = self.gdb_path
        cmd += " %s" % self.elf_path
        cmd += ' -ex="target remote :%u"' % self.server_port
        return cmd

    def start(self):
        if self.pipe:
            raise Exception("GDB Already running.")

        # Run GDB:
        cmd = self._gdb_command()
        try:
            self.pipe = subprocess.Popen(cmd, stdin=subprocess.PIPE, shell=True)
        except:
            logging.error("Failed to start GDB.\nCommand: `%s`" % cmd)
            return
        time.sleep(0.1)  # FIXME
        logging.info("GDB started.")

    def stop(self):
        if self.pipe:
            self.pipe.kill()
            logging.info("GDB stopped.")
            self.pipe = None

    def write_stdin(self, cmd):
        if not self.pipe:
            logging.error("GDB not running")
            return
        self.pipe.stdin.write(cmd)

    def send_signal(self, signal):
        self.pipe.send_signal(signal)


class GDBInterface:
    def __init__(self, gdb_driver):
        assert gdb_driver
        self.gdb_driver = gdb_driver

    def _send(self, cmd):
        self.gdb_driver.write_stdin(cmd)

    def _send_signal(self, signal):
        self.gdb_driver.send_signal(signal)

    def interrupt(self):
        self._send_signal(signal.SIGINT)

    def cont(self):
        self._send("c\n")

    def source(self, script_file_name):
        self._send("source %s\n" % script_file_name)

    def set(self, var_name, expr):
        self._send(f"set {var_name}={expr}\n")

    def disable_breakpoints(self):
        self._send("dis\n")

    def set_pagination(self, enabled):
        enabled_str = "on" if enabled else "off"
        self._send("set pagination %s\n" % enabled_str)
