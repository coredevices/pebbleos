# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

from .core import Runner, RunnerCaps, RunnerError

# Flash offset where system resources are written.
RESOURCES_OFFSET = 0x12620000


class SfToolRunner(Runner):
    name = "sftool"

    def __init__(self, cfg, tty=None):
        super().__init__(cfg)
        self.tty = tty

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={"flash", "erase"}, flash_resources=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument("--tty", help="Serial port for the sftool runner")

    @classmethod
    def do_create(cls, cfg, args):
        return cls(cfg, tty=getattr(args, "tty", None))

    def _sftool(self, command):
        if not self.tty:
            raise RunnerError("Port not specified, use --tty")
        self.call(f"sftool -c {self.cfg.soc} -p {self.tty} {command}")

    def do_run(self, command):
        if command == "flash":
            files = [self.cfg.hex_file]
            if self.cfg.resources_file:
                files.append(
                    f"{self.cfg.resources_file}@{RESOURCES_OFFSET:#x}"
                )
            self._sftool("write_flash " + " ".join(files))
        elif command == "erase":
            self._sftool("erase_flash")
