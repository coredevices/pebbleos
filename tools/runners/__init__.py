# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

from .core import (
    RunnerCaps,
    RunnerConfig,
    RunnerError,
    UnsupportedOperation,
)
from .nrfutil import NrfUtilRunner
from .openocd import OpenOcdRunner
from .sftool import SfToolRunner

__all__ = [
    "RunnerCaps",
    "RunnerConfig",
    "RunnerError",
    "UnsupportedOperation",
    "create",
    "get",
    "names",
    "register_args",
]


RUNNERS = {
    runner.name: runner
    for runner in (
        OpenOcdRunner,
        SfToolRunner,
        NrfUtilRunner,
    )
}


def names():
    return sorted(RUNNERS)


def get(name):
    try:
        return RUNNERS[name]
    except KeyError:
        raise RunnerError(f"Unknown runner: {name}")


def create(name, cfg, args):
    return get(name).create(cfg, args)


def register_args(parser):
    """Register every runner's CLI arguments on a parser.

    The board (and therefore the active runner) is not known at parse time, so
    all runners contribute their arguments. Argument names must be unique across
    runners.
    """
    for runner in RUNNERS.values():
        runner.add_parser(parser)
