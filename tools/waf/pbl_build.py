# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os

from waflib import Node, Utils
from waflib.Configure import conf
from waflib.TaskGen import before_method, feature

INTERFACE = "pbl_interface"


@conf
def pbl_build_init(bld):
    bld.pbl_libs = []
    bld.pbl_interface = bld(name=INTERFACE, export_includes=[], export_defines=[])


@conf
def pbl_include_directories(bld, *dirs):
    # Resolved here since export_includes strings are relative to the
    # interface generator, not to the caller. Like waf, a relative path
    # adds both its build-tree and source-tree nodes.
    inc = bld.pbl_interface.export_includes
    for d in dirs:
        if isinstance(d, Node.Node):
            inc.append(d)
            continue
        if bld.path.find_node(d) is None:
            bld.fatal(f"{bld.path}: include directory {d!r} not found")
        bld_node = bld.path.get_bld().make_node(d)
        bld_node.mkdir()
        inc.extend([bld_node, bld.path.make_node(d)])


@conf
def pbl_compile_definitions(bld, *defs):
    bld.pbl_interface.export_defines.extend(defs)


@conf
def pbl_library(bld, source, name=None, kind="objects", use=None, **kw):
    name = name or bld.path.path_from(bld.srcnode).replace(os.sep, "__")
    if name in bld.pbl_libs:
        bld.fatal(f"pbl library {name!r} defined twice")
    use = Utils.to_list(use or []) + [INTERFACE]
    if kind == "objects":
        tg = bld.objects(name=name, source=source, use=use, **kw)
    elif kind == "stlib":
        tg = bld.stlib(target=name, source=source, use=use, **kw)
    else:
        bld.fatal(f"pbl library {name!r}: unknown kind {kind!r}")
    bld.pbl_libs.append(name)
    return tg


@conf
def pbl_prebuilt_library(bld, name, paths):
    if name in bld.pbl_libs:
        bld.fatal(f"pbl library {name!r} defined twice")
    tg = bld.read_stlib(name, paths=paths)
    bld.pbl_libs.append(name)
    return tg


@conf
def pbl_library_ifdef(bld, cfg, *args, **kw):
    if bld.env[cfg]:
        return bld.pbl_library(*args, **kw)
    return None


@conf
def pbl_recurse_ifdef(bld, cfg, *dirs):
    if bld.env[cfg]:
        bld.recurse(list(dirs))


@conf
def pbl_program(bld, **kw):
    features = Utils.to_list(kw.pop("features", []))
    return bld.program(features=features + ["pbl_program"], link_group=True, **kw)


@feature("pbl_program")
@before_method("process_use")
def pbl_program_use(self):
    # Resolved at post time so wscript recursion order does not matter.
    self.use = Utils.to_list(getattr(self, "use", [])) + self.bld.pbl_libs + [INTERFACE]
