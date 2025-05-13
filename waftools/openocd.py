# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import contextlib
import pexpect
import re
import string
import subprocess
import sys
import waflib

from waflib import Logs


JTAG_OPTIONS = {'olimex': 'source [find interface/ftdi/olimex-arm-usb-ocd-h.cfg]',
                'fixture': 'source [find interface/flossjtag-noeeprom.cfg]',
                'bb2': 'source waftools/openocd_bb2_ftdi.cfg',
                'bb2-legacy': 'source waftools/openocd_bb2_ft2232.cfg',
                'jtag_ftdi': 'source waftools/openocd_jtag_ftdi.cfg',
                'swd_ftdi': 'source waftools/openocd_swd_ftdi.cfg',
                'swd_jlink': 'source waftools/openocd_swd_jlink.cfg',
                'swd_tigard': 'source waftools/openocd_swd_tigard.cfg',
                'swd_stlink': 'source [find interface/stlink-v2.cfg]',
                'swd_cmsisdap': 'source waftools/openocd_swd_cmsisdap.cfg',
                }

OPENOCD_TELNET_PORT = 4444


@contextlib.contextmanager
def daemon(ctx, cfg_file, use_swd=False):
    if _is_openocd_running():
        yield
    else:
        expect_str = "Listening on port"
        proc = pexpect.spawn('openocd', ['-f', cfg_file], encoding='utf-8', logfile=sys.stdout)
        # Wait for OpenOCD to connect to the board:
        result = proc.expect([expect_str, pexpect.TIMEOUT], timeout=10)
        if result == 0:
            yield
        else:
            raise Exception("Timed out connecting OpenOCD to development board...")
        proc.close()


def _has_openocd(ctx):
    try:
        ctx.cmd_and_log(['which', 'openocd'], quiet=waflib.Context.BOTH)
        return True
    except:
        return False


def _is_openocd_running():
    import socket
    import errno
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(('', OPENOCD_TELNET_PORT))
        s.close()
    except socket.error as e:
        s.close()
        return e.errno == errno.EADDRINUSE
    return False


def run_command(ctx, cmd, ignore_fail=False, expect=[], timeout=40,
                shutdown=True, enforce_expect=False, cfg_file="openocd.cfg"):
    if _is_openocd_running():
        import telnetlib
        t = telnetlib.Telnet('', OPENOCD_TELNET_PORT)
        Logs.info("Sending commands to OpenOCD daemon:\n%s\n..." % cmd)
        t.write("%s\n" % cmd)
        for regex in expect:
            idx, match, text = t.expect([regex], timeout)
            if enforce_expect and idx == -1:
                # They'll see the full story in another window
                ctx.fatal("OpenOCD expectation '%s' unfulfilled" % regex)
        t.close()
    else:
        fail_handling = ' || true ' if ignore_fail else ''
        if shutdown:
            # append 'shutdown' to make openocd exit:
            cmd = "%s ; shutdown" % cmd
        ctx.exec_command('openocd -f %s -c "%s" 2>&1 | tee .waf.openocd.log %s' %
                         (cfg_file, cmd, fail_handling), stdout=None, stderr=None)
        if enforce_expect:
            # Read the result
            with open(".waf.openocd.log", "r") as result_file:
                result = result_file.read()
                match_start = 0
                for regex in expect:
                    expect_match = re.search(regex, result[match_start:])
                    if not expect_match:
                        ctx.fatal("OpenOCD expectation '%s' unfulfilled" % regex)
                    match_start = expect_match.end()



def _get_supported_interfaces(ctx):
    if not _has_openocd(ctx):
        return []
    # Ugh, openocd exits with status 1 when not specifying an interface...
    try:
        ctx.cmd_and_log(['openocd', '-c', '"interface_list"'],
                        quiet=waflib.Context.BOTH,
                        output=waflib.Context.STDERR)
    except Exception as e:
        # Ugh, openocd prints the output to stderr...
        out = e.stderr
    out_lines = out.splitlines()
    interfaces = []
    for line in out_lines:
        matches = re.search(r"\d+: (\w+)", line)
        if matches:
            interfaces.append(matches.groups()[0])
    return interfaces


def get_flavor(conf):
    """ Returns a if OpenOCD is Pebble flavor """

    try:
        version_string = conf.cmd_and_log(['openocd', '--version'],
                                          quiet=waflib.Context.BOTH,
                                          output=waflib.Context.STDERR)
        version_string = version_string.splitlines()[0]
        return 'pebble' in version_string
    except Exception:
        Logs.error("Couldn't parse openocd version")
        return (False, False)


def _get_reset_conf(conf, should_connect_assert_srst):
    if conf.env.MICRO_FAMILY.startswith('STM32'):
        options = ['trst_and_srst', 'srst_nogate']
        if should_connect_assert_srst:
            options.append('connect_assert_srst')
        return ' '.join(options)
    elif conf.env.MICRO_FAMILY.startswith('NRF52'):
        return 'none'
    # FIXME: remove this, SF32LB does not support OpenOCD
    # but build system requires OpenOCD!
    elif conf.env.MICRO_FAMILY.startswith('SF32LB'):
        return 'none'
    else:
        raise Exception("Unsupported microcontroller family: %s" % conf.env.MICRO_FAMILY)


def write_cfg(conf):
    jtag = conf.env.JTAG
    if jtag == 'bb2':
        if 'ftdi' not in _get_supported_interfaces(conf):
            jtag = 'bb2-legacy'
            Logs.warn('OpenOCD is not compiled with --enable-ftdi, falling'
                      ' back to legacy ft2232 driver.')

    if conf.env.MICRO_FAMILY == 'STM32F2':
        target = 'stm32f2x.cfg'
    elif conf.env.MICRO_FAMILY == 'STM32F4':
        target = 'stm32f4x.cfg'
    elif conf.env.MICRO_FAMILY == 'STM32F7':
        target = 'stm32f7x.cfg'
    elif conf.env.MICRO_FAMILY == 'NRF52840':
        target = 'nrf52.cfg'
    # FIXME: remove this, SF32LB does not support OpenOCD
    # but build system requires OpenOCD!
    elif conf.env.MICRO_FAMILY == 'SF32LB':
        target = 'nrf52.cfg'

    is_pebble_flavor = get_flavor(conf)

    reset_config = _get_reset_conf(conf, False)
    Logs.info("reset_config: %s" % reset_config)

    if not is_pebble_flavor:
        Logs.warn("openocd is not Pebble flavored!")

    openocd_cfg = OPENOCD_CFG_TEMPLATE.substitute(jtag=JTAG_OPTIONS[jtag],
                                                  target=target,
                                                  reset_config=reset_config)
    waflib.Utils.writef('./openocd.cfg', openocd_cfg)


OPENOCD_CFG_TEMPLATE = string.Template("""
# THIS IS A GENERATED FILE: See waftools/openocd.py for details

${jtag}
source [find target/${target}]

reset_config ${reset_config}

$$_TARGETNAME configure -rtos FreeRTOS
$$_TARGETNAME configure -event gdb-attach {
    echo "Halting target because GDB is attaching..."
    halt
}
$$_TARGETNAME configure -event gdb-detach {
    echo "Resuming target because GDB is detaching..."
    resume
}
""")
