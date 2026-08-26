# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import functools
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

import clang
from generate_native_sdk import parse_c_decl


def extract_exported_functions(node, functions=[], types=[], defines=[]):
    def update_matching_export(exports, node):
        spelling = parse_c_decl.get_node_spelling(node)
        for e in exports:
            impl_name = e.impl_name if hasattr(e, "impl_name") else ""

            definition_name = impl_name if impl_name else e.name
            if spelling == definition_name:
                # Found a matching node! Before we update our export make sure this attribute is larger
                # than the one we may already have. This is to handle the case where we have typedef and
                # a struct as part of the same definition, we want to make sure we get the outer typedef.
                definition = parse_c_decl.get_string_from_file(node.extent)
                if e.full_definition is None or len(definition) > len(
                    e.full_definition
                ):
                    if node.kind == clang.cindex.CursorKind.MACRO_DEFINITION:
                        e.full_definition = "#define " + definition
                    else:
                        e.full_definition = definition

            # Update the exports with comments / definition info from both the
            # 'implName' and 'name'. Keep whatever is longer and does not start
            # with @internal (meaning the whole docstring is internal).
            if spelling == e.name or (impl_name and spelling == impl_name):
                comment = parse_c_decl.get_comment_string_for_decl(node)
                if comment is not None and not comment.startswith("//! @internal"):
                    if e.comment is None or len(comment) > len(e.comment):
                        e.comment = comment


    if node.kind == clang.cindex.CursorKind.FUNCTION_DECL:
        update_matching_export(functions, node)

    elif (
        node.kind == clang.cindex.CursorKind.STRUCT_DECL
        or node.kind == clang.cindex.CursorKind.ENUM_DECL
        or node.kind == clang.cindex.CursorKind.TYPEDEF_DECL
    ):
        update_matching_export(types, node)

    elif node.kind == clang.cindex.CursorKind.MACRO_DEFINITION:
        update_matching_export(defines, node)


def extract_symbol_info(
    filenames,
    functions,
    types,
    defines,
    output_dir,
    internal_sdk_build=False,
    compiler_flags=None,
):

    # Parse all the headers at the same time since that is much faster than
    # parsing each one individually
    all_headers_file = os.path.join(output_dir, "all_sdk_headers.h")
    with open(all_headers_file, "w") as outfile:
        outfile.writelines('#include "%s"\n' % f for f in filenames)

    parse_c_decl.parse_file(
        all_headers_file,
        filenames,
        functools.partial(
            extract_exported_functions,
            functions=functions,
            types=types,
            defines=defines,
        ),
        internal_sdk_build=internal_sdk_build,
        compiler_flags=compiler_flags,
    )


if __name__ == "__main__":
    parse_c_decl.dump_tree = True

    class Export:
        def __init__(self, name):
            self.name = name

            self.full_definition = None
            self.comment = None

    # clang.cindex.Config.library_file = "/home/brad/src/llvmbuild/Debug+Asserts/lib/libclang.so"

    extract_symbol_info((sys.argv[1],), [], [], [])
