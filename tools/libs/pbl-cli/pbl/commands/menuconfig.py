# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import shutil
import subprocess
import sys

from pbl.command import CommandContextError, PblCommand


class Menuconfig(PblCommand):
    group = "build"

    def __init__(self):
        super().__init__(
            "menuconfig",
            "Edit the configuration interactively",
            "Edit the build's Kconfig configuration, then reconfigure with it. "
            "The result is kept across reconfigurations; delete the build's "
            "menuconfig.conf to go back to the board defaults.",
        )

    def do_add_parser(self, parser_adder):
        return self.add_subparser(parser_adder)

    def do_run(self, args, unknown):
        from tools import boards

        build = self.build_dir()
        dotconfig = build.join(".config")
        if not os.path.isfile(dotconfig):
            raise CommandContextError(f"no .config in {build}")

        spec = build.board_spec
        env = dict(
            os.environ,
            srctree=self.topdir,
            BOARD=spec.name,
            BOARD_REVISION=spec.revision or boards.NO_REVISION,
            KCONFIG_CONFIG=dotconfig,
        )
        if self.dry_run:
            self.inf("[dry-run] menuconfig", dotconfig, color="yellow")
            return 0

        subprocess.run(
            [sys.executable, "-m", "menuconfig", os.path.join(self.topdir, "Kconfig")],
            env=env,
            check=False,
        )

        shutil.copyfile(dotconfig, build.join("menuconfig.conf"))
        return self.run_cmd(["cmake", str(build)])
