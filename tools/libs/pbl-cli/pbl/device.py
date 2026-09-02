# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Glue between the CLI and the binary runners.

Turns a configured build plus the parsed arguments into a runner ready to
execute one of the device commands.
"""

import os

from pbl import runners
from pbl.errors import CommandContextError


def runner_options(parser):
    """Add the runner selection flag and every runner's own arguments.

    Which runner is active is only known once a build directory has been
    read, so all of them contribute their arguments; the names have to stay
    unique across runners.
    """
    group = parser.add_argument_group("runner options")
    group.add_argument(
        "-r", "--runner", choices=runners.names(), help="Override the board's runner"
    )
    runners.register_args(group)


def make_runner(build, args, resources=False):
    """Create the runner for ``build``, honoring ``--runner``."""
    supported = build.board_spec.runners
    selected = getattr(args, "runner", None) or (supported[0] if supported else None)
    if not selected:
        raise CommandContextError(f"board {build.board} has no runner")
    if selected not in supported:
        raise CommandContextError(
            f"board {build.board} does not support the {selected} runner; "
            f"supported runners: {', '.join(supported) or 'none'}"
        )

    resources_file = None
    if resources and build.variant != "prf":
        resources_file = build.pbpack

    cfg = runners.RunnerConfig(
        board_dir=os.path.join(build.topdir, "boards", build.board_spec.name),
        soc=build.config.CONFIG_SOC or None,
        hex_file=build.hex,
        elf_file=build.elf,
        resources_file=resources_file,
        dry_run=getattr(args, "dry_run", False),
    )
    return runners.create(selected, cfg, args)


def run_on_device(build, args, command, resources=False):
    """Dispatch one runner command, reporting failures as command errors."""
    try:
        make_runner(build, args, resources=resources).run(command)
    except runners.RunnerError as e:
        raise CommandContextError(str(e)) from e
