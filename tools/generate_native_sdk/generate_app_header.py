# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os

import exports


def writeline(f, line=""):
    f.write(line + "\n")


def strip_internal_comments(comment_string):
    """Takes a multiline comment string and strips out the parts of the comment after an @internal keyword"""
    result = []
    for line in comment_string.splitlines():
        if "@internal" in line:
            # Skip the rest
            break
        result.append(line)
    return "\n".join(result)


def strip_internal_subcomments(string):
    """Takes a multiline comment string and strips out the parts of the comment after an @internal keyword"""
    result = []
    in_internal_comment = False
    for line in string.splitlines():
        if "@internal" in line and line.lstrip().startswith("//!"):
            in_internal_comment = True
        elif in_internal_comment:
            if not line.lstrip().startswith("//!"):
                in_internal_comment = False
                result.append(line)
        else:
            result.append(line)
    return "\n".join(result)


def rename_full_definition(e):
    if hasattr(e, "impl_name"):
        return e.full_definition.replace(e.impl_name, e.name, 1)
    else:
        return e.full_definition


def make_app_header(exports_tree, output_filename, header_type, inject_text):
    output_filename_dir = os.path.dirname(output_filename)
    if not os.path.exists(output_filename_dir):
        os.makedirs(output_filename_dir)

    """ header_type can be either "app", "worker" or "both" """
    with open(output_filename, "w") as f:
        writeline(f, "#pragma once")
        writeline(f)
        if inject_text is not None:
            f.write(inject_text)
            writeline(f)
        writeline(f, "#include <locale.h>")
        writeline(f, "#include <stdlib.h>")
        writeline(f, "#include <stdint.h>")
        writeline(f, "#include <stdio.h>")
        writeline(f, "#include <stdbool.h>")
        writeline(f, "#include <string.h>")
        writeline(f, "#include <time.h>")
        writeline(f)
        writeline(f, '#include "pebble_warn_unsupported_functions.h"')
        if header_type == "app":
            writeline(f, '#include "pebble_sdk_version.h"')
        else:
            writeline(f, '#include "pebble_worker_sdk_version.h"')
        writeline(f)
        writeline(f, "#ifndef __FILE_NAME__")
        writeline(f, "#define __FILE_NAME__ __FILE__")
        writeline(f, "#endif")
        writeline(f)

        def format_export(e):
            skip = (
                (header_type == "app" and e.worker_only)
                or (header_type == "worker" and e.app_only)
                or (header_type == "worker_only" and not e.worker_only)
                or e.deprecated
            )
            if isinstance(e, exports.Group):
                if not skip:
                    line = f"//! @addtogroup {e.name}"
                    if e.display_name is not None:
                        line += f" {e.display_name}"
                    writeline(f, line)

                    if e.comment is not None:
                        writeline(f, strip_internal_comments(e.comment))

                    writeline(f, "//! @{")
                    writeline(f)
                format_export_list(e.exports)
                if not skip:
                    writeline(f, f"//! @}} // group {e.name}")
                    writeline(f)
                return
            elif e.type == "forward_struct":
                if skip:
                    return
                writeline(f, "struct " + e.name + ";")
                writeline(f, "typedef struct " + e.name + " " + e.name + ";")
                writeline(f)
            elif isinstance(e, exports.StubbedFunctionExport):
                if skip:
                    return
                if not e.removed and not e.skip_definition and not e.deprecated:
                    if e.stub_definition is not None:
                        writeline(f, e.stub_definition)
                    elif e.stub_return == "void":
                        writeline(f, f"#define {e.name}(...) do {{}} while(0)")
                    else:
                        writeline(f, f"#define {e.name}(...) ({e.stub_return})")
                    writeline(f)
            elif e.type == "function":
                if skip:
                    return
                if not e.removed and not e.skip_definition and not e.deprecated:
                    if e.comment is not None:
                        writeline(f, strip_internal_comments(e.comment))
                    writeline(f, rename_full_definition(e).strip() + ";")
                    writeline(f)
            elif e.type == "define":
                if skip:
                    return
                if e.comment is not None:
                    writeline(f, strip_internal_comments(e.comment))
                writeline(f, e.full_definition)
                writeline(f)
            elif e.type == "type":
                if skip:
                    return
                if e.comment is not None:
                    writeline(f, strip_internal_comments(e.comment))
                writeline(f, strip_internal_subcomments(e.full_definition + ";"))
                writeline(f)
            else:
                raise RuntimeError("Unknown type: %s", e.type)

            if not skip and e.include_after:
                for header in e.include_after:
                    writeline(f, f'#include "{header}"')
                writeline(f, "")  # space out these headers nicely.

        def format_export_list(export_list):
            for e in export_list:
                format_export(e)

        format_export_list(exports_tree)


import unittest


class TestStripInternalComments(unittest.TestCase):
    def test_non_internal(self):
        comment = "//! This is a comment\n//! It looks normal"
        self.assertEqual(strip_internal_comments(comment), comment)  # unchanged

    def test_internal_line(self):
        comment = "//! This is a comment\n//! @internal This is internal"
        self.assertEqual(strip_internal_comments(comment), "//! This is a comment")

    def test_internal_block(self):
        comment = (
            "//! This is a comment\n//! @internal This is internal//! More internal"
        )
        self.assertEqual(strip_internal_comments(comment), "//! This is a comment")

    def test_all_internal(self):
        comment = "//! @internal This is internal\n//! More internal"
        self.assertEqual(strip_internal_comments(comment), "")

    def test_trailing_newline_non_internal(self):
        comment = "//! This is a comment\n//! It looks normal\n"
        self.assertEqual(
            strip_internal_comments(comment),
            "//! This is a comment\n//! It looks normal",
        )

    def test_trailing_newline_internal_block(self):
        comment = (
            "//! This is a comment\n//! @internal This is internal//! More internal"
        )
        self.assertEqual(strip_internal_comments(comment), "//! This is a comment")


if __name__ == "__main__":
    unittest.main()
