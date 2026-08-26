# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

from .core import Runner, RunnerCaps


class NrfUtilRunner(Runner):
    name = "nrfutil"

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={"flash", "run", "reset", "erase"})

    def _nrfutil(self, command):
        self.call("nrfutil device " + command)

    def do_run(self, command):
        if command == "flash":
            self._nrfutil(
                f"program --firmware {self.cfg.hex_file} "
                "--options chip_erase_mode=ERASE_RANGES_TOUCHED_BY_FIRMWARE"
            )
            self._nrfutil("reset")
        elif command in ("run", "reset"):
            self._nrfutil("reset")
        elif command == "erase":
            self._nrfutil("erase")
