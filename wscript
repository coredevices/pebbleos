# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""Unit test build.

The firmware is built with CMake (see docs/development/build_system.md);
waf only builds and runs the host unit tests, out of its own build
directory.
"""

import os
import sys

from waflib import Logs
from waflib.Build import BuildContext

waf_dir = sys.path[0]
sys.path.append(os.path.join(waf_dir, 'tools'))
sys.path.append(os.path.join(waf_dir, 'tools/log_hashing'))
sys.path.append(os.path.join(waf_dir, 'tools/waf'))

import tools.boards
import tools.waf.pbl_build  # registers the pbl_library API

# The firmware build owns 'build'.
out = 'build-test'


def _available_boards():
    import waflib

    return tools.boards.available_boards(waflib.Context.run_dir or os.getcwd())


def options(opt):
    gr = opt.add_option_group('test options')
    gr.add_option('-D', '--debug_test', action='store_true',
        help='Execute tests within GDB. Use alongside -M.')
    gr.add_option('-M', '--match', dest='regex', default=None, action='store',
        help='Run regex match tests. Example: ./waf test -M "test.*resource.*"')
    gr.add_option('-L', '--list_tests', dest='list_tests', action='store_true',
        help='List all test names. Usually used in conjunction with -M. Example: '
             './waf test -M test_animation -L')
    gr.add_option('-T', '--test_name', dest='test_name', default=None, action='store',
        help='Run only the given test name. Usually used in conjunction with -M. Example: '
             './waf test -M test_animation -T unschedule')
    gr.add_option('-C', '--coverage', dest='coverage', action='store_true', help='Generate gcov test coverage data and use lcov to generate HTML report')
    gr.add_option('--show_output', action='store_true', help='show test output')
    gr.add_option('--no_run', action='store_true', help='Do not run the tests, just build them')
    gr.add_option('--no_images', action='store_true', help='skip generation of test images, '
                  'which are only required for some tests and can slow down build times')
    boards = _available_boards()
    opt.add_option('--board', action='store',
                   choices=boards,
                   help='Which board we are targeting '
                        f'({", ".join(boards)})')
    opt.add_option('--compile_commands', action='store_true',
                   help='Create a clang compile_commands.json')


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

    conf.env.PLATFORM_NAME = _platform_name(conf, board)

    conf.load('protoc')

    Logs.pprint('CYAN', 'Configuring unit test environment')
    # The tests build in their own environment, which the test tooling and
    # the host-side tools look up by name.
    conf.setenv('local', conf.env)

    # The waf clang tool likes to use llvm-ar as its ar tool, but that does not
    # work on our build servers. Fall back to boring old ar; this populates the
    # 'AR' env variable so later searches find this one.
    conf.find_program('ar')

    conf.load('clang')
    conf.load('pebble_test', tooldir='tools/waf')

    conf.env.CLAR_DIR = conf.path.make_node('tools/clar/').abspath()
    conf.env.CFLAGS = [ '-std=c11',
                        '-Wall',
                        '-Werror',
                        '-Wno-error=unused-variable',
                        '-Wno-error=unused-function',
                        '-Wno-error=missing-braces',
                        '-Wno-error=unused-const-variable',
                        '-Wno-error=address-of-packed-member',
                        '-Wno-enum-conversion',

                        '-g3',
                        '-gdwarf-4',
                        '-O0',
                        '-fdata-sections',
                        '-ffunction-sections',
                        '-fno-common',
                        '-ffp-contract=off',
                        '-fexcess-precision=standard' ]

    # Apple's ARM64 linker uses chained fixups, which require pointer-aligned
    # relocations. Packed structs with pointer members fail to link because the
    # packed layout can place pointers at non-aligned offsets. Disable chained
    # fixups to use classic relocations instead.
    if sys.platform == 'darwin':
        conf.env.append_value('LINKFLAGS', '-Wl,-no_fixup_chains')

    conf.env.append_value('DEFINES', 'CLAR_FIXTURE_PATH="' +
                                     conf.path.make_node('tests/fixtures/').abspath() + '"')

    conf.env.append_value('DEFINES', 'CONFIG_LOG=1')

    if conf.options.compile_commands:
        conf.load('clang_compilation_database', tooldir='tools/waf')

        if not os.path.lexists('compile_commands.json'):
            filename = 'compile_commands.json'
            source = conf.path.get_bld().make_node(filename)
            os.symlink(source.path_from(conf.path), filename)

    # Confirm that requirements-*.txt and requirements-osx-brew.txt have been
    # satisfied.
    import tool_check
    tool_check.tool_check()


def build(bld):
    bld.set_env(bld.all_envs['local'])
    bld.pbl_build_init()
    # FIXME: remove include/pbl once all modules use the prefix
    bld.pbl_include_directories('include', 'include/pbl')
    bld.load('file_name_c_define', tooldir='tools/waf')

    bld.recurse('third_party/nanopb')
    bld.recurse('src/idl')
    bld.recurse('lib')
    bld.recurse('tests')
    bld.recurse('tools')


class SdkCommand(BuildContext):
    """packages the SDK into build-test/sdk"""
    cmd = 'sdk'
    fun = 'sdk'


def sdk(bld):
    """The shippable SDK; see docs/development/sdk_export.md."""
    bld.recurse('sdk')


class test(BuildContext):
    """builds and runs the tests"""
    cmd = 'test'
    variant = 'test'


class build_pdc2png(BuildContext):
    """executes the pdc2png build"""
    cmd = 'build_pdc2png'
    variant = 'pdc2png'


# vim:filetype=python
