# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""waf shim over tools/gitinfo.py, keeping the ctx-based signature."""

from tools.gitinfo import get_git_revision as _get_git_revision


def get_git_revision(ctx):
    return _get_git_revision(cwd=ctx.srcnode.abspath())
