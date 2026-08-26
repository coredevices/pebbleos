# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from waflib import Errors, Task, Utils
from waflib.Configure import conf
from waflib.TaskGen import after, feature

# Hook points in the master linker script (src/fw/linker/pebbleos.ld). For
# each location a snippets-<location>.ld file is generated in build/linker,
# aggregating the fragments registered with bld.linker_sources().
LINKER_SNIPPET_LOCATIONS = (
    "memory",
    "rom-start",
    "ram-sections",
    "footer",
)


@conf
def linker_sources(self, location, *sources, sort_key="default"):
    """Register linker script fragments for inclusion at a named hook point
    of the master linker script. Paths are relative to the calling wscript.
    Fragments are included sorted by (sort_key, registration order)."""
    if location not in LINKER_SNIPPET_LOCATIONS:
        raise Errors.WafError(
            "unknown linker snippet location %r (must be one of %s)"
            % (location, ", ".join(LINKER_SNIPPET_LOCATIONS))
        )
    snippets = getattr(self, "linker_snippets", None)
    if snippets is None:
        snippets = self.linker_snippets = {}
    for src in sources:
        node = self.path.find_node(src)
        if node is None:
            raise Errors.WafError(
                "could not find linker fragment %r in %s" % (src, self.path)
            )
        snippets.setdefault(location, []).append((sort_key, node))


def _write_linker_snippets(bld):
    """Generate the snippets-<location>.ld aggregation files. Returns the
    directory node, the snippet nodes and the registered fragment nodes."""
    dirnode = bld.bldnode.make_node("linker")
    dirnode.mkdir()
    registered = getattr(bld, "linker_snippets", {})
    snippet_nodes = []
    fragment_nodes = []
    for location in LINKER_SNIPPET_LOCATIONS:
        entries = sorted(registered.get(location, []), key=lambda e: e[0])
        content = "".join('#include "%s"\n' % node.abspath() for _, node in entries)
        node = dirnode.make_node("snippets-%s.ld" % location)
        if not node.exists() or node.read() != content:
            node.write(content)
        snippet_nodes.append(node)
        fragment_nodes.extend(n for _, n in entries)
    return dirnode, snippet_nodes, fragment_nodes


class ldscript_cpp(Task.Task):
    """Preprocess a linker script with the C preprocessor before it's handed
    to the linker. This gives linker scripts access
    to Kconfig CONFIG_* defines (via autoconf.h), #include, #if, etc."""

    run_str = (
        "${CC} -x assembler-with-cpp -nostdinc -undef -E -P "
        "${DEFINES_ST:DEFINES} ${CPPPATH_ST:LDSCRIPT_INCPATHS} "
        "-include ${AUTOCONF_H} ${SRC} -o ${TGT}"
    )
    color = "PINK"


@after("apply_link")
@feature("cprogram", "cshlib")
def process_ldscript(self):
    if not getattr(self, "ldscript", None) or self.env.CC_NAME != "gcc":
        return

    def convert_to_node(node_or_path_str):
        if isinstance(node_or_path_str, str):
            return self.path.make_node(node_or_path_str)
        else:
            return node_or_path_str

    if isinstance(self.ldscript, str) or isinstance(self.ldscript, list):
        ldscripts = Utils.to_list(self.ldscript)
    else:  # Assume Nod3
        ldscripts = [self.ldscript]
    nodes = [convert_to_node(node) for node in ldscripts]

    incpaths = []
    dep_nodes = []
    if self.env.AUTOCONF_H:
        snippet_dir, snippet_nodes, fragment_nodes = _write_linker_snippets(self.bld)
        incpaths.append(snippet_dir.abspath())
        dep_nodes.extend(snippet_nodes)
        dep_nodes.extend(fragment_nodes)
        # Common fragments included by the master script and by SoC fragments.
        common_dir = self.bld.srcnode.find_node("src/fw/linker")
        if common_dir is not None:
            incpaths.append(common_dir.abspath())
            dep_nodes.extend(common_dir.ant_glob("**/*.ld"))

    for node in nodes:
        if not node:
            raise Errors.WafError("could not find %r" % self.ldscript)

        if self.env.AUTOCONF_H:
            preprocessed = node.parent.find_or_declare(node.name + ".pre")
            tsk = self.create_task("ldscript_cpp", node, preprocessed)
            tsk.env.append_value("LDSCRIPT_INCPATHS", incpaths)
            tsk.dep_nodes.append(self.bld.bldnode.find_or_declare("autoconf.h"))
            tsk.dep_nodes.extend(dep_nodes)
            node = preprocessed

        self.link_task.env.append_value("LINKFLAGS", "-T%s" % node.abspath())
        self.link_task.dep_nodes.append(node)
