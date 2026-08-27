# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import re
import sys

block_comment_re = re.compile(r"""(^//!.*$(\n)?)+""", flags=re.MULTILINE)

addtogroup_start_re = re.compile(
    r"""//!\s+@addtogroup\s+(?P<name>\S+)(\s+(?P<display_name>.+))?$"""
)
block_start_re = re.compile(r"""//!\s+@{""")
block_end_re = re.compile(r"""//!\s+@}""")

define_block_comment_re = re.compile(
    r"""(?P<block_comment>(^//!.*$(\n)?)+)#define\s+(?P<define_name>[A-Za-z0-9_]+)""",
    flags=re.MULTILINE,
)


def find_group(group_stack, groups):
    for g in groups:
        if g.group_stack() == group_stack:
            return g
    return None


def add_group_comment(group_comment, group_stack, groups):
    for g in groups:
        if g.group_stack() == group_stack:
            g.comment = group_comment.strip()
            break


def scan_file_content_for_groups(content, groups):
    group_stack = []

    in_group_description = False
    group_comment = ""

    for match in block_comment_re.finditer(content):
        comment_block = match.group(0).strip()
        for line in comment_block.splitlines():
            result = addtogroup_start_re.search(line)
            if result is not None:
                group_stack.append(result.group("name"))

                if result.group("display_name") is not None:
                    g = find_group(group_stack, groups)
                    if g is not None:
                        g.display_name = result.group("display_name")

                in_group_description = True
            elif block_start_re.search(line) is not None:
                in_group_description = False

                group_comment.strip()

                if len(group_comment) > 0:
                    g = find_group(group_stack, groups)
                    if g is not None:
                        g.comment = group_comment.strip()

                    group_comment = ""
            elif block_end_re.search(line) is not None:
                if len(group_stack) == 0:
                    raise RuntimeError("Unbalanced groups!")

                group_stack.pop()
            elif in_group_description:
                group_comment += line + "\n"

    if len(group_stack) != 0:
        raise RuntimeError("Unbalanced groups!")


def scan_file_content_for_defines(content, defines):
    for match in define_block_comment_re.finditer(content):
        for d in defines:
            if d.name == match.group("define_name"):
                d.comment = match.group("block_comment").strip()
                break


def parse_file(filename, groups, defines):
    with open(filename) as f:
        content = f.read()

    scan_file_content_for_groups(content, groups)
    scan_file_content_for_defines(content, defines)


def extract_comments(filenames, groups, defines):
    for f in filenames:
        parse_file(f, groups, defines)


def test_handle_macro():
    test_input = """
//! This is a documented MACRO
#define FOO(x) BAR(x);

//! This is a documented define
#define BRAD "awesome"

//! This is a multiline
//! documented define.
#define TEST "nosetests"
"""

    class TestDefine:
        def __init__(self, name):
            self.name = name
            self.comment = None

    defines = [
        TestDefine("FOO"),
        TestDefine("BRAD"),
        TestDefine("TEST"),
        TestDefine("Other"),
    ]

    scan_file_content_for_defines(test_input, defines)

    assert defines[0].comment == "//! This is a documented MACRO"
    assert defines[1].comment == "//! This is a documented define"
    assert defines[2].comment == "//! This is a multiline\n//! documented define."
    assert defines[3].comment is None


if __name__ == "__main__":
    parse_file(sys.argv[1], [], [])
