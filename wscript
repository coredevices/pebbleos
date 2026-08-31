# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""SDK packaging.

The firmware is built with CMake and the unit tests with CMake/CTest (see
docs/development/build_system.md and docs/development/testing.md); waf is
left with the shippable SDK, which bundles a waf of its own. See
docs/development/sdk_export.md.
"""

import os
import sys

from waflib import Logs
from waflib.Build import BuildContext

waf_dir = sys.path[0]
sys.path.append(os.path.join(waf_dir, 'tools'))

import tools.boards

# The firmware build owns 'build' and the unit tests own 'build-test'.
out = 'build-sdk'


def _available_boards():
    import waflib

    return tools.boards.available_boards(waflib.Context.run_dir or os.getcwd())


def options(opt):
    boards = _available_boards()
    opt.add_option('--board', action='store',
                   choices=boards,
                   help='Which board we are targeting '
                        f'({", ".join(boards)})')


def _platform_name(conf, board):
    """The SDK platform the board maps to, read straight from its
    defconfig: the SDK packaging step is the only thing here that needs
    it, and running Kconfig for one symbol is not worth it."""
    defconfig = os.path.join(conf.srcnode.abspath(), 'boards', board.name, 'defconfig')
    with open(defconfig) as f:
        for line in f:
            if line.startswith('CONFIG_PLATFORM_') and line.rstrip().endswith('=y'):
                symbol = line.split('=')[0]
                return symbol[len('CONFIG_PLATFORM_'):].lower()
    conf.fatal(f'No platform specified for {board.target}!')


def configure(conf):
    if not conf.options.board:
        conf.fatal('No board selected! '
                   'You must pass a --board argument when configuring.')

    try:
        board = tools.boards.parse_board(conf.srcnode.abspath(), conf.options.board)
    except (TypeError, ValueError) as e:
        conf.fatal(str(e))

    Logs.pprint('CYAN', f'Configuring the SDK for {board.target}')
    conf.env.PLATFORM_NAME = _platform_name(conf, board)

    # Confirm that requirements-*.txt and requirements-osx-brew.txt have been
    # satisfied.
    import tool_check
    tool_check.tool_check()


def build(bld):
    bld.fatal('Nothing to build here; the firmware is built with CMake. '
              'Run ./waf sdk to package the SDK.')


class SdkCommand(BuildContext):
    """packages the SDK into build-sdk/sdk"""
    cmd = 'sdk'
    fun = 'sdk'


def sdk(bld):
    """The shippable SDK; see docs/development/sdk_export.md."""
    bld.recurse('sdk')


# vim:filetype=python
