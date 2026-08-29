# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Just enough of waf's Node/ConfigSet/BuildContext surface to drive the
resource generators from CMake.

The resource generators (tools/resources/**) are shared with the SDK,
which still builds with waf, so they keep taking waf-shaped arguments.
These shims let the same code run from a plain command line.
"""

import glob as globmod
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def setup_path():
    """Put the module search path in the state the generators expect."""
    for path in (os.path.join(REPO_ROOT, "tools"), REPO_ROOT):
        if path not in sys.path:
            sys.path.insert(0, path)


class Node:
    """A filesystem path with the handful of waf Node methods the resource
    generators call."""

    def __init__(self, path, bld_root=None, src_root=None):
        self.path = os.path.abspath(path)
        self.bld_root = bld_root
        self.src_root = src_root or REPO_ROOT

    def _derive(self, path):
        return Node(path, bld_root=self.bld_root, src_root=self.src_root)

    def abspath(self):
        return self.path

    @property
    def name(self):
        return os.path.basename(self.path)

    @property
    def parent(self):
        return self._derive(os.path.dirname(self.path))

    def suffix(self):
        return os.path.splitext(self.path)[1]

    def relpath(self):
        return os.path.relpath(self.path, self.src_root)

    def path_from(self, node):
        return os.path.relpath(self.path, node.abspath())

    def make_node(self, rel):
        if isinstance(rel, (list, tuple)):
            rel = os.path.join(*rel)
        return self._derive(os.path.join(self.path, rel))

    def find_node(self, rel):
        node = self.make_node(rel)
        return node if os.path.exists(node.abspath()) else None

    find_resource = find_node

    def get_bld(self):
        if self.bld_root is None:
            return self
        rel = os.path.relpath(self.path, self.src_root)
        return Node(os.path.join(self.bld_root, rel),
                    bld_root=self.bld_root, src_root=self.src_root)

    def change_ext(self, ext, old=None):
        base = self.path
        if old:
            base = base.removesuffix(old)
        else:
            base = os.path.splitext(base)[0]
        return self._derive(base + ext)

    def mkdir(self):
        os.makedirs(self.path, exist_ok=True)

    def exists(self):
        return os.path.exists(self.path)

    def ant_glob(self, pattern, **kwargs):
        matches = globmod.glob(os.path.join(self.path, pattern), recursive=True)
        return [self._derive(m) for m in sorted(matches)]

    def read(self, flags="r", encoding="ISO8859-1"):
        if "b" in flags:
            with open(self.path, "rb") as f:
                return f.read()
        with open(self.path, encoding=encoding) as f:
            return f.read()

    def write(self, data, flags="w", encoding="ISO8859-1"):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        if "b" in flags:
            with open(self.path, "wb") as f:
                f.write(data)
        else:
            with open(self.path, "w", encoding=encoding) as f:
                f.write(data)

    def __repr__(self):
        return f"<Node {self.path}>"


class Env(dict):
    """waf's ConfigSet: missing keys read back as an empty list."""

    def __getattr__(self, key):
        return self.get(key, [])

    def __setattr__(self, key, value):
        self[key] = value

    def __getitem__(self, key):
        return self.get(key, [])


class Bld:
    """The bits of a BuildContext the generators reach for."""

    def __init__(self, env, path, bld_root, variant=""):
        self.env = env
        self.all_envs = {}
        self.variant = variant
        self.bldnode = Node(bld_root, bld_root=bld_root)
        self.srcnode = Node(REPO_ROOT, bld_root=bld_root)
        self.path = Node(path, bld_root=bld_root)

    def fatal(self, msg):
        sys.exit(msg)


class TaskGen:
    def __init__(self, bld, path=None):
        self.bld = bld
        self.env = bld.env
        self.path = path or bld.path


class Task:
    """A single generator invocation: its inputs, outputs and environment."""

    def __init__(self, bld, inputs, outputs, path=None):
        self.inputs = list(inputs)
        self.outputs = list(outputs)
        self.env = bld.env
        self.generator = TaskGen(bld, path)
