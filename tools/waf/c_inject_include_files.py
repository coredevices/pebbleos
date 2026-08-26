# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
Adds `-include` flags for list of files to CFLAGS and ASFLAGS, by adding an
optional attribute `inject_include_files`.
"""

from waflib.Node import Nod3
from waflib.TaskGen import after_method, feature
from waflib.Utils import def_attrs, to_list


@feature("c", "asm")
@after_method("create_compiled_task")
def process_include_files(self):
    def_attrs(self, inject_include_files=None)
    if not self.inject_include_files:
        return

    include_flags = []
    for include_file in to_list(self.inject_include_files):
        if isinstance(include_file, Nod3):
            node = include_file
        elif isinstance(include_file, str):
            node = self.path.find_node(include_file)
            if not node:
                self.bld.fatal("%s does not exist." % include_file)
        else:
            self.bld.fatal("Expecting str or Nod3 in `inject_include_files` list")
        include_file_path = node.abspath()
        include_flags.append("-include%s" % include_file_path)

    self.env.append_unique("CFLAGS", include_flags)
    self.env.append_unique("ASFLAGS", include_flags)

    for s in self.source:
        self.bld.add_manual_dependency(s, node)
