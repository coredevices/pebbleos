# Copyright 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import re
import subprocess
import sys

from waflib import Logs
from waflib.Build import BuildContext
from waflib.Configure import conf

import tools.waf.boards


class HexInt(int):
    """Int that formats as the original hex literal, so DEFINES mirroring hex
    Kconfig symbols stays token-identical with the autoconf.h #define (which
    preserves the literal as written, e.g. 0x00000000; a non-identical macro
    redefinition is fatal under -Werror)."""

    def __new__(cls, literal):
        if isinstance(literal, str):
            value = int(literal, 16)
        else:
            value = int(literal)
            literal = hex(value)
        obj = super().__new__(cls, value)
        obj.literal = literal
        return obj

    def __getnewargs__(self):
        # Reconstruct from the literal so copies keep its exact spelling.
        return (self.literal,)

    def __str__(self):
        return self.literal

    def __format__(self, spec):
        if not spec:
            return self.literal
        return super().__format__(spec)


def options(opt):
    opt.add_option(
        "--kconfig-override",
        action="append",
        default=[],
        dest="kconfig_overrides",
        metavar="CONFIG_FOO=value",
        help=(
            "Override a Kconfig symbol at configure time. "
            "May be specified multiple times. The compact form "
            "-DCONFIG_FOO=y is also accepted."
        ),
    )


@conf
def load_kconfig(ctx, config_path):
    """Parse a .config file into a Python dictionary.

    Reads lines of the form CONFIG_FOO=val, skipping comments and blank lines.
    String values are unquoted. Returns a dict mapping config key names to
    string values.
    """
    config = {}
    with open(config_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            m = re.match(r"^(CONFIG_\w+)=(.*)$", line)
            if m:
                key = m.group(1)
                val = m.group(2)
                if val in ("y", "n"):
                    val = val == "y"
                elif val.startswith('"') and val.endswith('"'):
                    val = val[1:-1]
                elif val.startswith(("0x", "0X")):
                    val = HexInt(val)
                else:
                    try:
                        val = int(val)
                    except ValueError:
                        pass
                config[key] = val
    return config


def _parse_kconfig_override(conf, override):
    if "=" not in override:
        conf.fatal(f"Invalid Kconfig override '{override}': expected CONFIG_FOO=value")

    key, value = override.split("=", 1)
    if not re.match(r"^CONFIG_\w+$", key):
        conf.fatal(
            f"Invalid Kconfig override '{override}': symbol name must look like CONFIG_FOO"
        )

    return key[len("CONFIG_") :], value


def _apply_kconfig_overrides(conf, kconf):
    for override in conf.options.kconfig_overrides:
        name, value = _parse_kconfig_override(conf, override)
        sym = kconf.syms.get(name)
        if sym is None or not sym.nodes:
            conf.fatal(f"Undefined Kconfig symbol in override: CONFIG_{name}")

        sym.set_value(value)


def configure(conf):
    import kconfiglib

    srcdir = conf.srcnode.abspath()
    blddir = conf.bldnode.abspath()
    try:
        board = tools.waf.boards.parse_board(srcdir, conf.options.board)
    except (TypeError, ValueError) as e:
        conf.fatal(str(e))

    defconfig = os.path.join(srcdir, "boards", board.name, "defconfig")
    if not os.path.exists(defconfig):
        conf.fatal(f"Board defconfig not found: {defconfig}")
    revision_defconfig = os.path.join(
        srcdir, "boards", board.name, f"defconfig.{board.revision}"
    )

    os.environ["srctree"] = srcdir
    # The board is known from --board; export it so the root Kconfig can
    # source only the active board's Kconfig via `rsource "boards/$(BOARD)/..."`.
    os.environ["BOARD"] = board.name
    os.environ["BOARD_REVISION"] = board.revision or tools.waf.boards.NO_REVISION
    kconf = kconfiglib.Kconfig(os.path.join(srcdir, "Kconfig"))
    kconf.warn_assign_override = True
    kconf.warn_assign_redun = True
    kconf.warn_assign_undef = True
    kconf.load_config(defconfig)
    if board.revision and os.path.exists(revision_defconfig):
        kconf.load_config(revision_defconfig, replace=False)

    prj_conf = os.path.join(srcdir, "src", "fw", "prj.conf")
    if os.path.exists(prj_conf):
        kconf.load_config(prj_conf, replace=False)

    variant = conf.options.variant
    variant_conf = os.path.join(srcdir, "src", "fw", f"prj_{variant}.conf")
    if os.path.exists(variant_conf):
        kconf.load_config(variant_conf, replace=False)

    _apply_kconfig_overrides(conf, kconf)

    # Check for assigned values overridden by unsatisfied dependencies
    # (same pattern as Zephyr's check_assigned_sym_values)
    for sym in kconf.unique_defined_syms:
        if sym.choice or sym.user_value is None:
            continue

        if sym.type in (kconfiglib.BOOL, kconfiglib.TRISTATE):
            user_str = kconfiglib.TRI_TO_STR[sym.user_value]
        else:
            user_str = sym.user_value

        if user_str != sym.str_value:
            deps = kconfiglib.split_expr(sym.direct_dep, kconfiglib.AND)
            if sym.type in (kconfiglib.BOOL, kconfiglib.TRISTATE):
                mdeps = [d for d in deps if kconfiglib.expr_value(d) < sym.user_value]
            else:
                mdeps = [d for d in deps if kconfiglib.expr_value(d) == 0]
            dep_strs = [
                f"{kconfiglib.expr_str(d)} "
                f"(={kconfiglib.TRI_TO_STR[kconfiglib.expr_value(d)]})"
                for d in mdeps
            ]
            kconf.warnings.append(
                f"CONFIG_{sym.name}={user_str} was resolved to "
                f"{sym.str_value}. "
                f"Check these unsatisfied dependencies: "
                f"{', '.join(dep_strs)}"
            )

    if kconf.warnings:
        for warning in kconf.warnings:
            Logs.warn(f"Kconfig: {warning}")
        conf.fatal("Kconfig warnings found, aborting")

    config_path = os.path.join(blddir, ".config")
    kconf.write_config(config_path)

    autoconf_path = os.path.join(blddir, "autoconf.h")
    kconf.write_autoconf(autoconf_path)

    kconfig = conf.load_kconfig(config_path)
    for key, val in kconfig.items():
        conf.env[key] = val
    conf.env.AUTOCONF_H = autoconf_path
    conf.env.append_unique("CFLAGS", ["-include", autoconf_path])
    conf.env.append_unique("cfg_files", [config_path])

    # Mirror CONFIG_* symbols into env.DEFINES. The compiler already sees them
    # via -include autoconf.h, but waf's C preprocessor scanner only consults
    # DEFINES when evaluating `#if` / `#ifdef` while tracking header deps.
    # Without this, scanner-skipped branches hide transitive dependencies on
    # generated headers (e.g. font_resource_keys.auto.h) and waf schedules
    # compilation before those headers are produced.
    for key, val in kconfig.items():
        if isinstance(val, bool):
            if val:
                conf.env.append_value("DEFINES", f"{key}=1")
        elif isinstance(val, int):
            conf.env.append_value("DEFINES", f"{key}={val}")
        elif isinstance(val, str):
            conf.env.append_value("DEFINES", f'{key}="{val}"')
    msg = f"{len(kconfig)} symbols loaded from {board.target}"
    if os.path.exists(prj_conf):
        msg += " + prj.conf"
    if os.path.exists(variant_conf):
        msg += f" + prj_{variant}.conf"
    if board.revision and os.path.exists(revision_defconfig):
        msg += f" + defconfig.{board.revision}"
    if conf.options.kconfig_overrides:
        msg += f" + {len(conf.options.kconfig_overrides)} CLI override(s)"
    conf.msg("Kconfig", msg)


class menuconfig(BuildContext):
    """launch menuconfig to interactively configure the firmware"""

    cmd = "menuconfig"

    def execute(self):
        self.restore()
        if not self.all_envs:
            self.load_envs()
        srcdir = self.srcnode.abspath()
        blddir = self.bldnode.abspath()

        env = os.environ.copy()
        env["srctree"] = srcdir
        env["BOARD"] = self.env.BOARD_NAME
        env["BOARD_REVISION"] = self.env.BOARD_REVISION or tools.waf.boards.NO_REVISION
        env["KCONFIG_CONFIG"] = os.path.join(blddir, ".config")

        subprocess.run(
            [sys.executable, "-m", "menuconfig", os.path.join(srcdir, "Kconfig")],
            env=env,
            check=False,
        )
