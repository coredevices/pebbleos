import os
import re
import subprocess
import sys
import datetime
import time

import waflib
from waflib import Logs
from waflib.Build import BuildContext
from waflib.Configure import conf
from waflib.TaskGen import before_method, feature
from waflib.Tools.ccroot import link_task


def _normalize_kconfig_override_args(argv):
    normalized = []
    for arg in argv:
        if arg.startswith('-DCONFIG_') and '=' in arg:
            normalized.append('--kconfig-override={}'.format(arg[2:]))
        else:
            normalized.append(arg)
    return normalized


sys.argv = _normalize_kconfig_override_args(sys.argv)

waf_dir = sys.path[0]
sys.path.append(os.path.join(waf_dir, 'tools'))
sys.path.append(os.path.join(waf_dir, 'tools/log_hashing'))
sys.path.append(os.path.join(waf_dir, 'sdk/tools/'))
sys.path.append(os.path.join(waf_dir, 'tools/waf'))

import tools.waf.generate_log_strings_json
import tools.waf.generate_timezone_data
import tools.waf.gitinfo
import tools.waf.boards
import tools.waf.ldscript
import tools.waf.objcopy
import tools.waf.pblboot
import tools.waf.pebble_sdk_gcc as pebble_sdk_gcc
import tools.runners as pebble_runners
from tools.waf.pebble_sdk_locator import activate_sdk
from tools.pebble_sdk_platform import pebble_platforms

from pebble_sdk_version import set_env_sdk_version

# Prefer an installed PebbleOS SDK's binaries (toolchain, QEMU, sftool) when
# present. Done at import time so it applies to every waf invocation.
activate_sdk(waflib.Context.run_dir or os.getcwd())

LOGHASH_OUT_PATH = 'src/fw/loghash_dict.json'


@conf
def get_pbz_node(ctx, fw_type, board_type, version_string, slot=None):
    return ctx.path.get_bld().make_node('{}_{}_{}{}.pbz'.format(
        fw_type, board_type, version_string, "" if slot is None else f"_slot{slot}"
    ))


@conf
def get_pbpack_node(ctx):
    return ctx.path.get_bld().make_node('system_resources.pbpack')


@conf
def get_pebbleos_node(ctx):
    return ctx.path.get_bld().make_node('pebbleos.bin')


@feature("c")
@before_method('apply_link')
def use_group_link(self):
    """
    Use a link group to resolve dependencies
    """
    if 'cprogram' in self.features and getattr(self, 'link_group', False):
        self.features.insert(0, "group_cprogram")


class group_cprogram(link_task):
    run_str = '${LINK_CC} ${LINKFLAGS} ${CCLNK_SRC_F}${SRC} ${CCLNK_TGT_F}${TGT[0].abspath()} ${RPATH_ST:RPATH} ${FRAMEWORKPATH_ST:FRAMEWORKPATH} ${FRAMEWORK_ST:FRAMEWORK} ${ARCH_ST:ARCH} -Wl,--start-group ${STLIB_MARKER} ${STLIBPATH_ST:STLIBPATH} ${STLIB_ST:STLIB} ${SHLIB_MARKER} ${LIBPATH_ST:LIBPATH} ${LIB_ST:LIB} -Wl,--end-group'
    ext_out=['.bin']
    vars=['LINKDEPS']
    inst_to='${BINDIR}'


def _available_boards():
    return tools.waf.boards.available_boards(waflib.Context.run_dir or os.getcwd())


def truncate(msg):
    if msg is None:
        return msg

    # Don't truncate exceptions thrown by waf itself
    if "Traceback " in msg:
        return msg

    truncate_length = 600
    if len(msg) > truncate_length:
        msg = msg[:truncate_length-4] + '...\n' + waflib.Logs.colors.NORMAL
    return msg


class _OptParserAdapter(object):
    """Adapts a waf (optparse) option container to the argparse-style
    add_argument() interface the runners' do_add_parser() expects."""

    def __init__(self, opt):
        self._opt = opt

    def add_argument(self, *flags, **kwargs):
        self._opt.add_option(*flags, **kwargs)


def options(opt):
    opt.load('pebble_arm_gcc', tooldir='tools/waf')
    opt.load('show_configure', tooldir='tools/waf')
    opt.load('kconfig', tooldir='tools/waf')

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
                        '({})'.format(', '.join(boards)))
    opt.add_option('--runner', default=None,
                   help="Override the board's default runner for flash/run/debug")
    opt.add_option('--resources', action='store_true',
                   help='Also flash system resources alongside the firmware')
    # Runner-specific arguments (e.g. --tty for sftool) are contributed by the
    # runners themselves, mirroring Zephyr's west do_add_parser().
    pebble_runners.register_args(_OptParserAdapter(opt))
    opt.add_option('--compile_commands', action='store_true', help='Create a clang compile_commands.json')
    opt.add_option('--onlysdk', action='store_true', help="only build the sdk")
    opt.add_option('--variant', action='store', default='normal',
                   choices=['normal', 'prf'],
                   help='Build variant: normal (default) or prf (recovery firmware)')

def configure(conf):
    if not conf.options.board:
        conf.fatal('No board selected! '
                   'You must pass a --board argument when configuring.')

    try:
        board = tools.waf.boards.parse_board(conf.srcnode.abspath(), conf.options.board)
    except ValueError as e:
        conf.fatal(str(e))

    # Has to be 'tools.waf.gettext' as unadorned 'gettext' will find the gettext
    # module in the standard library.
    conf.load('tools.waf.gettext')

    conf.load('kconfig', tooldir='tools/waf')

    # JS engine selection is driven entirely by CONFIG_MODDABLE_XS. Override
    # per-board with `-DCONFIG_MODDABLE_XS=y/n` at configure time.
    if conf.env.CONFIG_MODDABLE_XS:
        conf.env.JS_ENGINE = 'moddable'
    else:
        conf.env.JS_ENGINE = 'none'

    if not board.runners and not conf.env.CONFIG_QEMU:
        conf.fatal('Board {} does not define any supported runners'.format(
                   board.target))

    for runner in board.runners:
        if runner not in pebble_runners.names():
            conf.fatal('Board {} references unknown runner {}'.format(
                       board.target, runner))

    conf.env.SUPPORTED_RUNNERS = board.runners
    conf.env.RUNNER = board.runners[0] if board.runners else None

    # Set platform used for building the SDK
    if conf.env.CONFIG_PLATFORM_EMERY:
        conf.env.PLATFORM_NAME = 'emery'
        conf.env.MIN_SDK_VERSION = 3
    elif conf.env.CONFIG_PLATFORM_FLINT:
        conf.env.PLATFORM_NAME = 'flint'
        conf.env.MIN_SDK_VERSION = 2
    elif conf.env.CONFIG_PLATFORM_GABBRO:
        conf.env.PLATFORM_NAME = 'gabbro'
        conf.env.MIN_SDK_VERSION = 3
    else:
        conf.fatal('No platform specified for {}!'.format(board.target))

    # Save this for later
    conf.env.BOARD = board.target
    conf.env.BOARD_NAME = board.name
    conf.env.BOARD_REVISION = board.revision
    conf.env.BOARD_NORMALIZED = board.normalized

    conf.env.VARIANT = conf.options.variant
    if conf.env.VARIANT == 'prf':
        conf.env.JS_ENGINE = 'none'

    # PRF variant forces JS_ENGINE='none' above. If the board's defconfig had
    # CONFIG_MODDABLE_XS=y, autoconf.h was already written with the macro
    # defined — undefine it on the command line so source-level guards match
    # what we actually link.
    if conf.env.JS_ENGINE == 'none' and conf.env.CONFIG_MODDABLE_XS:
        conf.env.append_value('CFLAGS', ['-UCONFIG_MODDABLE_XS'])
        conf.env.CONFIG_MODDABLE_XS = None

    conf.find_program('node nodejs', var='NODE',
                      errmsg="Unable to locate the Node command. "
                             "Please check your Node installation and try again.")

    conf.load('protoc')

    conf.load('binary_header')

    platform = pebble_platforms[conf.env.PLATFORM_NAME]
    define = 'MAX_FONT_GLYPH_SIZE={}'.format(platform['MAX_FONT_GLYPH_SIZE'])
    conf.env.append_value('DEFINES', [define])

    # Used for pblboot image naming; -1 when the board has no slots.
    conf.env.SLOT = conf.env.CONFIG_FIRMWARE_SLOT if conf.env.CONFIG_PBLBOOT else -1

    # Save a baseline environment that we'll use for unit tests
    # Detach so operations against conf.env don't affect unit_test_env
    unit_test_env = conf.env.derive()
    unit_test_env.detach()

    # Save a baseline environment that we'll use for ARM environments
    base_env = conf.env

    Logs.pprint('CYAN', 'Configuring arm_firmware environment')
    conf.setenv('', base_env)
    conf.load('pebble_arm_gcc', tooldir='tools/waf')
    # Select the C library (see lib/c/Kconfig) once the arch flags are set:
    # picolibc-from-source is built for that exact multilib.
    conf.load('libc', tooldir='tools/waf')

    Logs.pprint('CYAN', 'Configuring unit test environment')
    conf.setenv('local', unit_test_env)

    # Strip CONFIG_* DEFINES mirrored from the configure-time board: each test
    # selects its own simulated platform (asterix / obelix / gabbro) and injects
    # the matching BOARD/PLATFORM/SCREEN_COLOR_DEPTH_BITS itself, so the
    # configure board's symbols would just collide with the per-test ones.
    conf.env.DEFINES = [d for d in conf.env.DEFINES
                        if not d.split('=', 1)[0].startswith('CONFIG_')]

    # if sys.platform.startswith('linux'):
        # libclang_path = subprocess.check_output(['llvm-config', '--libdir']).strip()
        # conf.env.append_value('INCLUDES', [os.path.join(libclang_path, 'clang/3.2/include/'),])

    # The waf clang tool likes to use llvm-ar as it's ar tool, but that doesn't work on our build
    # servers. Fall back to boring old ar. This will populate the 'AR' env variable so future
    # searches for what value to put into env['AR'] will find this one.
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

    # Reset LINKFLAGS so firmware-specific flags (e.g. --undefined=HAL_GetTick)
    # don't leak into the host test environment.
    conf.env.LINKFLAGS = []

    # Apple's ARM64 linker uses chained fixups which require pointer-aligned
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

    Logs.pprint('CYAN', 'Configuring stored apps environment')
    conf.setenv('stored_apps', base_env)
    process_info = conf.path.find_node('src/fw/process_management/pebble_process_info.h')
    set_env_sdk_version(conf, process_info)
    pebble_sdk_gcc.configure(conf)

    # Confirm that requirements-*.txt and requirements-osx-brew.txt have been satisfied.
    import tool_check
    tool_check.tool_check()


def stop_build_timer(ctx):
    t = datetime.datetime.utcnow() - ctx.pbl_build_start_time
    node = ctx.path.get_bld().make_node('build_time')
    with open(node.abspath(), 'w') as fout:
        fout.write(str(int(round(t.total_seconds()))))


def _link_firmware(bld, sources):
    fw_linkflags = ['-Wl,--cref',
                    '-Wl,-Map=pebbleos.map',
                    '-Wl,--gc-sections',
                    '-Wl,--undefined=uxTopUsedPriority',
                    '-Wl,--build-id=sha1',
                    '-Wl,--sort-section=alignment',
                    '-Wl,--print-memory-usage']

    # C library link flags (-nostdlib / -specs=...), selected by lib/c via
    # tools/waf/libc.py. malloc/free are always redirected to pbl_malloc.
    fw_linkflags.extend(bld.env.LIBC_LINKFLAGS)

    fw_linkflags.extend(['-Wl,--wrap=malloc',
                         '-Wl,--undefined=__wrap_malloc',
                         '-Wl,--wrap=realloc',
                         '-Wl,--undefined=__wrap_realloc',
                         '-Wl,--wrap=calloc',
                         '-Wl,--undefined=__wrap_calloc',
                         '-Wl,--wrap=free',
                         '-Wl,--undefined=__wrap_free'])

    uses = ['applib',
            'board',
            'bt_driver',
            'comm',
            'console',
            'debug',
            'drivers',
            'flash_region',
            'freertos',
            'fw_services',
            'gcc',
            'kernel',
            'logging',
            'mfg',
            'popups',
            'process_management',
            'process_state',
            'resource',
            'shell',
            'syscall',
            'system',
            'util',
            'proto_schemas',
            'libbtutil',
            'libos',
            'libutil',
            'nanopb',
            'pbl_includes',
            'soc',
            'speex',
            'startup',
            'tinymt32',
            'upng']
    uses.extend(bld.env.FW_APPS)
    # C library use targets (the assert hook, _sbrk, the nano printf shim),
    # selected by lib/c via tools/waf/libc.py.
    uses.extend(bld.env.LIBC_USE)

    # Used by pblboot image tools; the C define mirrors the historical name.
    bld.env.FIRMWARE_OFFSET = bld.env.CONFIG_FIRMWARE_OFFSET
    bld.env.append_value('DEFINES', [f'FIRMWARE_OFFSET={bld.env.CONFIG_FIRMWARE_OFFSET}'])

    # Build and link the firmware ELF
    elf_node = bld.path.get_bld().make_node('pebbleos.elf')
    x = bld.program(source=sources,
                use=uses,
                link_group=True,
                lib=bld.env.LIBC_LIBS,
                target=elf_node,
                includes='fonts',
                ldscript='src/fw/linker/pebbleos.ld',
                linkflags=fw_linkflags)

    x.env.append_value('LINKFLAGS', fw_linkflags)

    if bld.env.CONFIG_PBLBOOT:
        git_revision = tools.waf.gitinfo.get_git_revision(bld)
        bld.env.PBLBOOT_PRIORITY = str(tools.waf.pblboot.boot_priority(
            git_revision['TAG'], int(git_revision['TIMESTAMP'])))
        nohdr_hex_node = elf_node.change_ext('.nohdr.hex')
        bld(rule=tools.waf.objcopy.objcopy_hex, source=elf_node, target=nohdr_hex_node)
        hex_node = elf_node.change_ext('.hex')
        bld(rule=tools.waf.pblboot.insert_header_hex, source=nohdr_hex_node, target=hex_node)
        nohdr_bin_node = elf_node.change_ext('.nohdr.bin')
        bld(rule=tools.waf.objcopy.objcopy_bin, source=elf_node, target=nohdr_bin_node)
        bin_node = elf_node.change_ext('.bin')
        bld(rule=tools.waf.pblboot.insert_header_bin, source=nohdr_bin_node, target=bin_node)
    else:
        hex_node = elf_node.change_ext('.hex')
        bld(rule=tools.waf.objcopy.objcopy_hex, source=elf_node, target=hex_node)
        bin_node = elf_node.change_ext('.bin')
        bld(rule=tools.waf.objcopy.objcopy_bin, source=elf_node, target=bin_node)

    # Create the log_strings .elf and check the format specifier rules
    if bld.env.CONFIG_LOG_HASHED:
        fw_loghash_node = bld.path.get_bld().make_node('pebbleos_loghash_dict.json')
        bld(rule=tools.waf.generate_log_strings_json.wafrule,
            source=elf_node, target=fw_loghash_node, path=bld.path)
        bld.LOGHASH_DICTS.append(fw_loghash_node)


def _build_recovery(bld):
    sources = bld.path.ant_glob('src/fw/*.c')
    sources.extend(bld.path.ant_glob('src/fw/*.[sS]'))

    sources.append(bld.path.get_bld().make_node('src/fw/builtin_resources.auto.c'))

    _link_firmware(bld, sources)


def _build_normal(bld):
    # Generate timezone data
    olson_txt = bld.srcnode.make_node('resources/normal/base/tzdata/timezones_olson.txt')
    tzdata_bin = bld.bldnode.make_node('resources/normal/base/tzdata/tzdata.bin.reso')
    bld(rule=tools.waf.generate_timezone_data.wafrule,
        source=olson_txt,
        target=tzdata_bin)

    bld.DYNAMIC_RESOURCES.append(tzdata_bin)

    sources = bld.path.ant_glob('src/fw/*.c')
    sources.extend(bld.path.ant_glob('src/fw/*.[sS]'))

    # Collect translatable strings from the firmware-core sources. apps,
    # services and applib have their own .pot targets (merged below).
    gettexts = []
    gettexts.extend(bld.path.ant_glob('src/fw/**/*.c',
                                     excl=['apps/**', 'services/**', 'applib/**']))
    gettexts.extend(bld.path.ant_glob('src/fw/**/*.h'))
    gettexts.extend(bld.path.ant_glob('src/fw/**/*.def'))

    bld.gettext(source=gettexts, target=bld.path.get_bld().make_node('fw.pot'))
    bld.msgcat(
            source=[bld.path.get_bld().make_node('fw.pot'),
                    bld.path.get_bld().make_node('src/fw/services/services.pot'),
                    bld.path.get_bld().make_node('src/fw/applib/applib.pot'),
                    bld.path.get_bld().make_node('src/fw/apps/apps.pot')],
            target=bld.path.get_bld().make_node('pebbleos.pot'))

    sources.append(bld.path.get_bld().make_node('src/fw/pebble.auto.c'))
    sources.append(bld.path.get_bld().make_node('src/fw/resource/pfs_resource_table.auto.c'))
    sources.append(bld.path.get_bld().make_node('src/fw/resource/timeline_resource_table.auto.c'))
    sources.append(bld.path.get_bld().make_node('src/fw/builtin_resources.auto.c'))

    _link_firmware(bld, sources)


def _build_fw(bld):
    bld.env.FW_APPS = []

    # FIXME create applib_includes or something like that
    fw_includes_use=['pbl_includes',
                     'subsys_includes',
                     'freertos_includes',
                     'idl_includes',
                     'nanopb_includes']

    if bld.env.CONFIG_SOC_NRF52:
        fw_includes_use.append('hal_nordic')
    elif bld.env.CONFIG_SOC_SF32LB52:
        fw_includes_use.append('hal_sifli')

    bld(export_includes=['src/fw',
                         'src/fw/applib/vendor/uPNG',
                         'src/fw/applib/vendor/tinflate'],
        use=fw_includes_use,
        name='fw_includes')

    # Truncate the commit to fit in our versions struct. This may cause an ambiguous commit
    # hash, but it's better than killing the build because the commit doesn't fit.
    git_rev = tools.waf.gitinfo.get_git_revision(bld)
    git_rev['COMMIT'] = git_rev['COMMIT'][:7]
    git_rev['PATCH_VERBOSE_STRING']
    if len(git_rev['TAG']) > 31:
        Logs.warn('Git tag {} is too long, truncating'.format(git_rev['TAG']))
        git_rev['TAG'] = git_rev['TAG'][:31]

    bld(features='subst',
        source='src/fw/git_version.auto.h.in',
        target=bld.path.get_bld().make_node('src/fw/git_version.auto.h'),
        **git_rev)

    bld.recurse('subsys')
    bld.recurse('src/fw/startup')
    bld.recurse('src/fw/drivers')
    bld.recurse('src/fw/board')
    bld.recurse('src/fw/shell')
    bld.recurse('src/fw/services')
    bld.recurse('src/fw/applib')
    bld.recurse('soc')
    bld.recurse('src/fw/mfg')
    bld.recurse('src/fw/comm')
    bld.recurse('src/fw/console')
    bld.recurse('src/fw/debug')
    bld.recurse('src/fw/flash_region')
    bld.recurse('src/fw/kernel')
    bld.recurse('src/fw/popups')
    bld.recurse('src/fw/process_management')
    bld.recurse('src/fw/process_state')
    bld.recurse('src/fw/resource')
    bld.recurse('src/fw/syscall')
    bld.recurse('src/fw/system')
    bld.recurse('src/fw/util')
    bld.recurse('src/fw/apps/core')

    if bld.env.VARIANT == 'prf':
        bld.recurse('src/fw/apps/prf')
        _build_recovery(bld)
    else:
        bld.recurse('src/fw/apps')
        _build_normal(bld)


def build(bld):
    bld.DYNAMIC_RESOURCES = []
    bld.LOGHASH_DICTS = []

    # Start this timer here to include the time to generate tasks.
    bld.pbl_build_start_time = datetime.datetime.utcnow()
    bld.add_post_fun(stop_build_timer)

    # FIXME: remove include/pbl once all modules use prefix
    bld(export_includes=['include', 'include/pbl'], name='pbl_includes')

    if bld.variant == 'test':
        bld.set_env(bld.all_envs['local'])

    bld.load('file_name_c_define', tooldir='tools/waf')

    bld.recurse('third_party/nanopb')
    bld.recurse('src/idl')

    if bld.cmd == 'install':
        raise Exception("install isn't a supported command. Did you mean flash?")

    if bld.variant == 'pdc2png':
        bld.recurse('tools')
        return

    if bld.variant == 'tools':
        bld.recurse('tools')
        return

    if bld.variant == '':
        # Dependency for SDK
        bld.recurse('third_party/moddable')

    if bld.variant == '' and bld.env.VARIANT != 'prf':
        # sdk generation
        bld.recurse('sdk')

    if bld.options.onlysdk:
        # stop here, sdk generation is done
        return

    # Do not enable stationary mode in PRF or release firmware
    if (bld.env.VARIANT != 'prf' and not bld.env.CONFIG_QEMU and not bld.env.CONFIG_SHELL_SDK):
        bld.env.append_value('DEFINES', 'STATIONARY_MODE')

    if bld.variant == 'test':
        bld.recurse('third_party/nanopb')
        bld.recurse('lib')
        bld.recurse('src')
        bld.recurse('tests')
        bld.recurse('tools')
        return

    if bld.variant == '' and bld.env.VARIANT != 'prf':
        bld.recurse('apps/stored')

    bld.recurse('third_party')
    bld.recurse('lib')
    bld.recurse('src')
    _build_fw(bld)

    # Generate resources. Leave this until the end so we collect all the env['DYNAMIC_RESOURCES']
    # values that the other build steps added.
    bld.recurse('resources')

    bld.add_post_fun(size_resources)
    if bld.env.CONFIG_LOG_HASHED:
        bld.add_post_fun(merge_loghash_dicts)


def merge_loghash_dicts(bld):
    loghash_dict = bld.path.get_bld().make_node(LOGHASH_OUT_PATH)

    import log_hashing.newlogging
    log_hashing.newlogging.merge_loghash_dict_json_files(loghash_dict, bld.LOGHASH_DICTS)


class SizeResources(BuildContext):
    cmd = 'size_resources'
    fun = 'size_resources'


def size_resources(ctx):
    """prints size information of resources"""

    if ctx.env.VARIANT == 'prf':
        return

    pbpack_path = ctx.path.get_bld().find_node('system_resources.pbpack')
    if pbpack_path is None:
        ctx.fatal('No resource pbpack found')

    if ctx.env.CONFIG_SOC_NRF52:
        max_size = 1024 * 1024
    elif ctx.env.CONFIG_SOC_SF32LB52:
        max_size = 2048 * 1024
    elif ctx.env.CONFIG_QEMU:
        max_size = 2048 * 1024
    else:
        max_size = 256 * 1024

    pbpack_actual_size = os.path.getsize(pbpack_path.path_from(ctx.path))

    bar_width = 20
    filled = min(bar_width, round(bar_width * pbpack_actual_size / max_size))
    Logs.pprint('CYAN', 'Resources: [%-*s] %6.2f%% (%d/%d bytes)\n'
                % (bar_width, '#' * filled,
                   100 * pbpack_actual_size / max_size,
                   pbpack_actual_size, max_size))

    if pbpack_actual_size > max_size:
        ctx.fatal('Resources are too large for target board %d > %d'
                  % (pbpack_actual_size, max_size))


class test(BuildContext):
    """builds and runs the tests"""
    cmd = 'test'
    variant = 'test'



def docs(ctx):
    """builds the documentation out to build/doxygen"""
    ctx.exec_command('doxygen Doxyfile', stdout=None, stderr=None)


class DocsSdk(BuildContext):
    """builds the sdk documentation out to build/sdk/<platformname>/doxygen_sdk"""
    cmd = 'docs_sdk'
    fun = 'docs_sdk'


def docs_sdk(ctx):
    pebble_sdk = ctx.path.get_bld().make_node('sdk')
    supported_platforms = pebble_sdk.listdir()

    for platform in supported_platforms:
        doxyfile = pebble_sdk.find_node(platform).find_node('Doxyfile-SDK.auto')
        if doxyfile:
            ctx.exec_command('doxygen {}'.format(doxyfile.path_from(ctx.path)),
                             stdout=None, stderr=None)


def docs_all(ctx):
    """builds the documentation with all dependency graphs out to build/doxygen"""
    ctx.exec_command('doxygen Doxyfile-all-graphs', stdout=None, stderr=None)

# Bundle commands
#################################################


def _get_version_info(ctx):
    # FIXME: it's probably a better idea to lift board + version info from the .bin file... this can get out of sync!
    git_revision = tools.waf.gitinfo.get_git_revision(ctx)
    if git_revision['TAG'] != '?':
        version_string = git_revision['TAG']
        version_ts = int(git_revision['TIMESTAMP'])
        version_commit = git_revision['COMMIT']
    else:
        version_string = 'dev'
        version_ts = 0
        version_commit = ''
    return version_string, version_ts, version_commit


def _make_bundle(ctx, fw_bin_path, fw_type='normal', board=None, resource_path=None, write=True):
    import mkbundle

    if board is None:
        board = ctx.env.BOARD_NORMALIZED

    b = mkbundle.PebbleBundle()

    version_string, version_ts, version_commit = _get_version_info(ctx)
    slot = ctx.env.SLOT if fw_type == 'normal' and ctx.env.SLOT != -1 else None
    out_file = ctx.get_pbz_node(fw_type, ctx.env.BOARD_NORMALIZED, version_string, slot).path_from(ctx.path)

    try:
        _check_firmware_image_size(ctx, fw_bin_path)
        b.add_firmware(fw_bin_path, fw_type, version_ts, version_commit, board, version_string, slot)
    except FirmwareTooLargeException as e:
        ctx.fatal(str(e))
    except mkbundle.MissingFileException as e:
        ctx.fatal('Error: Missing file ' + e.filename + ', have you run ./waf build yet?')

    if resource_path is not None:
        b.add_resources(resource_path, version_ts)
    if not ctx.env.CONFIG_RELEASE and ctx.env.CONFIG_LOG_HASHED:
        loghash_dict = ctx.path.get_bld().make_node(LOGHASH_OUT_PATH).abspath()
        b.add_loghash(loghash_dict)

    # Add a LICENSE.txt file
    b.add_license('LICENSE')

    if fw_type == 'normal':
        layouts_node = ctx.path.get_bld().find_node('resources/layouts.json.auto')
        if layouts_node is not None:
            b.add_layouts(layouts_node.path_from(ctx.path))

    if write:
        b.write(out_file)
        waflib.Logs.pprint('CYAN', 'Writing bundle to: %s' % out_file)

    return b


class BundleCommand(BuildContext):
    cmd = 'bundle'
    fun = 'bundle'


def bundle(ctx):
    """bundles a firmware"""

    if ctx.env.VARIANT == 'prf':
        _make_bundle(ctx, ctx.get_pebbleos_node().path_from(ctx.path), fw_type='recovery')
    else:
        _make_bundle(ctx, ctx.get_pebbleos_node().path_from(ctx.path),
                     resource_path=ctx.get_pbpack_node().path_from(ctx.path))


# QEMU flash image commands
#################################################

class QemuImageMicroCommand(BuildContext):
    cmd = 'qemu_image_micro'
    fun = 'qemu_image_micro'


class QemuImageSpiCommand(BuildContext):
    cmd = 'qemu_image_spi'
    fun = 'qemu_image_spi'


def qemu_image_micro(ctx):
    """creates the micro-flash image for qemu"""
    from intelhex import IntelHex

    fw_hex = ctx.get_pebbleos_node().change_ext('.hex')
    micro_flash_node = ctx.path.get_bld().make_node('qemu_micro_flash.bin')
    micro_flash_path = micro_flash_node.path_from(ctx.path)
    Logs.pprint('CYAN', 'Writing micro flash image to {}'.format(micro_flash_path))

    img = IntelHex(fw_hex.path_from(ctx.path))
    img.padding = 0xff
    flash_end = ((img.maxaddr() + 511) // 512) * 512
    img.tobinfile(micro_flash_path, start=0x00000000, end=flash_end - 1)


def qemu_image_spi(ctx):
    """creates a SPI flash image for qemu"""
    if ctx.env.CONFIG_QEMU:
        # QEMU generic boards: resources at offset 0x620000 in 32MB flash
        resources_begin = 0x620000
        image_size = 0x2000000
    else:
        resources_begin = 0x280000
        image_size = 0x400000

    spi_flash_node = ctx.path.get_bld().make_node('qemu_spi_flash.bin')
    spi_flash_path = spi_flash_node.path_from(ctx.path)
    Logs.pprint('CYAN', 'Writing SPI flash image to {}'.format(spi_flash_path))
    with open(spi_flash_path, 'wb') as qemu_spi_img_file:
        # Pad the first section before system resources with FF's
        qemu_spi_img_file.write(bytes([0xff]) * resources_begin)

        # Write system resources:
        pbpack = ctx.get_pbpack_node()
        res_img = open(pbpack.path_from(ctx.path), 'rb').read()
        qemu_spi_img_file.write(res_img)

        # Pad with 0xFF up to image size
        tail_padding_size = image_size - resources_begin - len(res_img)
        qemu_spi_img_file.write(bytes([0xff]) * tail_padding_size)


# Flash commands
#################################################

class FirmwareTooLargeException(Exception):
    pass


def _check_firmware_image_size(ctx, path):
    BYTES_PER_K = 1024
    firmware_size = os.path.getsize(path)
    # Determine flash and bootloader size so we can calculate the max firmware size
    if ctx.env.CONFIG_SOC_NRF52:
        if ctx.env.VARIANT == 'prf' and not ctx.env.CONFIG_MFG:
            max_firmware_size = 512 * BYTES_PER_K
        else:
            # 1024k of flash and 32k bootloader
            max_firmware_size = (1024 - 32) * BYTES_PER_K
    elif ctx.env.CONFIG_SOC_SF32LB52:
        if ctx.env.VARIANT == 'prf' and not ctx.env.CONFIG_MFG:
            max_firmware_size = 576 * BYTES_PER_K
        else:
            # 3072k of flash
            max_firmware_size = 3072 * BYTES_PER_K
    elif ctx.env.CONFIG_QEMU:
        max_firmware_size = 4096 * BYTES_PER_K
    else:
        ctx.fatal('Cannot check firmware size against unknown micro family')

    if firmware_size > max_firmware_size:
        raise FirmwareTooLargeException('Firmware is too large! Size is 0x%x should be less than 0x%x' \
                                        % (firmware_size, max_firmware_size))

    return ('%d / %d bytes used (%d free)' %
            (firmware_size, max_firmware_size, (max_firmware_size - firmware_size)))


def _create_runner(ctx, want_resources=False):
    selected = ctx.options.runner or ctx.env.RUNNER
    supported = ctx.env.SUPPORTED_RUNNERS or ([ctx.env.RUNNER] if ctx.env.RUNNER else [])

    if not selected:
        ctx.fatal('No runner available for board {}'.format(ctx.env.BOARD))
    if selected not in supported:
        ctx.fatal('Board {} does not support runner {}. Supported runners: {}'.format(
                  ctx.env.BOARD, selected, ', '.join(supported) or 'none'))

    resources_file = None
    if want_resources and ctx.options.resources and ctx.env.VARIANT != 'prf':
        resources_file = ctx.get_pbpack_node().path_from(ctx.path)

    fw = ctx.get_pebbleos_node()
    cfg = pebble_runners.RunnerConfig(
        board_dir=os.path.join('boards', ctx.env.BOARD_NAME),
        soc=ctx.env.CONFIG_SOC,
        hex_file=fw.change_ext('.hex').path_from(ctx.path),
        elf_file=fw.change_ext('.elf').path_from(ctx.path),
        resources_file=resources_file,
    )

    try:
        return pebble_runners.create(selected, cfg, ctx.options)
    except pebble_runners.RunnerError as e:
        ctx.fatal(str(e))


class FlashCommand(BuildContext):
    """flashes the firmware to a connected device"""
    cmd = 'flash'
    fun = 'flash'


def flash(ctx):
    fw_bin = ctx.get_pebbleos_node()
    try:
        space_left = _check_firmware_image_size(ctx, fw_bin.path_from(ctx.path))
    except FirmwareTooLargeException as e:
        ctx.fatal(str(e))
    Logs.pprint('CYAN', 'FW: ' + space_left)

    runner = _create_runner(ctx, want_resources=True)
    try:
        runner.run('flash')
    except pebble_runners.RunnerError as e:
        ctx.fatal(str(e))


class RunCommand(BuildContext):
    """resets and runs the firmware on a connected device"""
    cmd = 'run'
    fun = 'run'


def run(ctx):
    try:
        _create_runner(ctx).run('run')
    except pebble_runners.RunnerError as e:
        ctx.fatal(str(e))


class DebugCommand(BuildContext):
    """attaches gdb to the target"""
    cmd = 'debug'
    fun = 'debug'


def debug(ctx):
    try:
        _create_runner(ctx).run('debug')
    except pebble_runners.RunnerError as e:
        ctx.fatal(str(e))


# Tool build commands
#################################################


class build_pdc2png(BuildContext):
    """executes the pdc2png build"""
    cmd = 'build_pdc2png'
    variant = 'pdc2png'


class build_tools(BuildContext):
    """build all tools in tools/ dir"""
    cmd = 'build_tools'
    variant = 'tools'

# vim:filetype=python
