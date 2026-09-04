# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from waflib import Errors, Utils
from waflib.TaskGen import after, feature


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

    if isinstance(self.ldscript, (str, list)):
        ldscripts = Utils.to_list(self.ldscript)
    else:  # Assume Nod3
        ldscripts = [self.ldscript]
    nodes = [convert_to_node(node) for node in ldscripts]

    for node in nodes:
        if not node:
            raise Errors.WafError(f"could not find {self.ldscript!r}")

        self.link_task.env.append_value("LINKFLAGS", f"-T{node.abspath()}")
        self.link_task.dep_nodes.append(node)
