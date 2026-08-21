# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import glob
import logging
import os
import re
import shutil
import subprocess
import sys

dump_tree = False


def add_clang_compat_module_to_sys_path_if_needed():
    try:
        pass
    except:
        sys.path.append(os.path.join(os.path.dirname(__file__), "clang_compat"))
        logging.info("Importing clang python compatibility module")


add_clang_compat_module_to_sys_path_if_needed()
import clang.cindex


def get_homebrew_llvm_lib_path():
    try:
        o = subprocess.check_output(["brew", "ls", "llvm"])
    except subprocess.CalledProcessError:
        # No brew llvm installed
        return None

    # Brittleness alert! Grepping output of `brew info llvm` for llvm bin path:
    m = re.search(".*/llvm-config", o.decode("utf8"))
    if m:
        llvm_config_path = m.group(0)

        o = subprocess.check_output([llvm_config_path, "--libdir"])
        llvm_lib_path = o.decode("utf8").strip()

        # Make sure --enable-clang and --enable-python options were used:
        if os.path.exists(os.path.join(llvm_lib_path, "libclang.dylib")) and glob.glob(
            os.path.join(llvm_lib_path, "python*", "site-packages", "clang")
        ):
            return llvm_lib_path
        else:
            logging.info(
                "Found llvm from homebrew, but not installed with"
                " --with-clang --with-python"
            )


def load_library():
    try:
        libclang_lib = clang.cindex.conf.lib
    except clang.cindex.LibclangError:
        pass
    except:
        raise
    else:
        return

    if sys.platform == "darwin":
        libclang_path = get_homebrew_llvm_lib_path()
        if not libclang_path:
            # Try using Xcode's libclang:
            logging.info("llvm from homebrew not found, trying Xcode's instead")
            xcode_path = (
                subprocess.check_output(["xcode-select", "--print-path"])
                .decode("utf8")
                .strip()
            )
            libclang_path = os.path.join(
                xcode_path, "Toolchains/XcodeDefault.xctoolchain/usr/lib"
            )
        clang.cindex.conf.set_library_path(libclang_path)
    elif sys.platform == "linux2":
        libclang_path = (
            subprocess.check_output(["llvm-config", "--libdir"]).decode("utf8").strip()
        )
        clang.cindex.conf.set_library_path(libclang_path)

    libclang_lib = clang.cindex.conf.lib


def do_libclang_setup():
    load_library()

    functions = (
        (
            "clang_Cursor_getCommentRange",
            [clang.cindex.Cursor],
            clang.cindex.SourceRange,
        ),
    )

    for f in functions:
        clang.cindex.register_function(clang.cindex.conf.lib, f, False)


def is_node_kind_a_type_decl(kind):
    return (
        kind == clang.cindex.CursorKind.STRUCT_DECL
        or kind == clang.cindex.CursorKind.ENUM_DECL
        or kind == clang.cindex.CursorKind.TYPEDEF_DECL
    )


def get_node_spelling(node):
    return clang.cindex.conf.lib.clang_getCursorSpelling(node)


def get_comment_range(node):
    source_range = clang.cindex.conf.lib.clang_Cursor_getCommentRange(node)
    if source_range.start.file is None:
        return None

    return source_range


def get_comment_range_for_decl(node):
    source_range = get_comment_range(node)
    if source_range is None:
        if node.kind == clang.cindex.CursorKind.TYPEDEF_DECL:
            for child in node.get_children():
                if (
                    is_node_kind_a_type_decl(child.kind)
                    and len(get_node_spelling(child)) == 0
                ):
                    source_range = get_comment_range(child)

    return source_range


def get_comment_string_for_decl(node):
    comment_range = get_comment_range_for_decl(node)
    comment_string = get_string_from_file(comment_range)
    if comment_string is None:
        return None

    if "@addtogroup" in comment_string or re.search(r"@[{}]", comment_string):
        # This is actually a block comment, not a comment specifically for this type. Ignore it.
        return None

    return comment_string


def get_string_from_file(source_range):
    if source_range is None:
        return None

    source_range_file = source_range.start.file
    if source_range_file is None:
        return None

    with open(source_range_file.name, "rb") as f:
        f.seek(source_range.start.offset)
        return f.read(source_range.end.offset - source_range.start.offset).decode(
            "utf8"
        )


def dump_node(node, indent_level=0):
    spelling = node.spelling
    if node.kind == clang.cindex.CursorKind.MACRO_DEFINITION:
        spelling = get_node_spelling(node)

    print("%*s%s> %s" % (indent_level * 2, "", node.kind, spelling))
    print("%*sRange:   %s" % (4 + (indent_level * 2), "", str(node.extent)))
    print(
        "%*sComment: %s"
        % (4 + (indent_level * 2), "", str(get_comment_range_for_decl(node)))
    )


def return_true(node):
    return True


def for_each_node(node, func, level=0, filter_func=return_true):
    if not filter_func(node):
        return

    if dump_tree:
        # Skip over nodes that are added by clang internals
        if node.location.file is not None:
            dump_node(node, level)

    func(node)

    for child in node.get_children():
        for_each_node(child, func, level + 1, filter_func)


def extract_declarations(tu, filenames, func):
    matching_basenames = {os.path.basename(f) for f in filenames}

    def filename_filter_func(node):
        node_file = node.location.file
        if node_file is None:
            return True

        node_filename = node_file.name
        if node_filename is None:
            return True

        base_name = os.path.basename(node_filename)
        return base_name in matching_basenames

    for_each_node(tu.cursor, func, filter_func=filename_filter_func)


def parse_file(
    filename, filenames, func, internal_sdk_build=False, compiler_flags=None
):
    root_dir = os.path.join(os.path.dirname(__file__), "../..")
    src_dir = os.path.join(root_dir, "src")

    args = [
        "-I%s/core" % src_dir,
        "-I%s/include" % root_dir,
        "-I%s/subsys" % root_dir,
        "-I%s/fw" % src_dir,
        "-I%s/fw/applib/vendor/uPNG" % src_dir,
        "-I%s/fw/applib/vendor/tinflate" % src_dir,
        "-I%s/libc/include" % src_dir,
        "-I%s/../build/src/fw" % src_dir,
        "-DSDK",
        "-fno-builtin-itoa",
    ]

    # Add header search paths, recursing subdirs:
    for inc_sub_dir in ["fw/util"]:
        args += [inc_sub_dir]
        args += [
            "-I%s" % d for d in glob.glob(os.path.join(src_dir, "%s/*/" % inc_sub_dir))
        ]

    if internal_sdk_build:
        args.append("-DINTERNAL_SDK_BUILD")
    else:
        args.append("-DPUBLIC_SDK")

    args.extend(compiler_flags)

    # Check Clang for unsigned types being undefined
    # https://sourceware.org/ml/newlib/2014/msg00082.html
    # this workaround should be removed when fixed in newlib
    cmd = ["clang" if shutil.which("clang") else "gcc", "-dM", "-E", "-"]
    try:
        out = (
            subprocess.check_output(cmd, stdin=open("/dev/null")).decode("utf8").strip()
        )
        if not isinstance(out, str):
            out = out.decode(sys.stdout.encoding or "iso8859-1")
    except Exception as err:
        print("Could not run compiler type checking %r" % err)
        out = ""

    if "__UINT8_TYPE__" not in out:
        args.insert(0, r"-D__UINT8_TYPE__=unsigned __INT8_TYPE__")
        args.insert(0, r"-D__UINT16_TYPE__=unsigned __INT16_TYPE__")
        args.insert(0, r"-D__UINT32_TYPE__=unsigned __INT32_TYPE__")
        args.insert(0, r"-D__UINT64_TYPE__=unsigned __INT64_TYPE__")
        args.insert(0, r"-D__UINTPTR_TYPE__=unsigned __INTPTR_TYPE__")

    # Tools pull in time.h from arm toolchain instead of using our core/utils/time/time.h
    # with modified definition of struct tm, so disable accidental include of wrong time.h
    args.insert(0, r"-D_TIME_H_")

    # Use the libc headers from the ARM toolchain. -print-sysroot resolves
    # the real toolchain location even when the driver is reached through a
    # symlink (e.g. the nix devShell's flat bin/), unlike a path derived
    # from `which arm-none-eabi-gcc`.
    sysroot = (
        subprocess.check_output(["arm-none-eabi-gcc", "-print-sysroot"])
        .decode("utf8")
        .strip()
    )
    args.append("-I%s" % os.path.join(sysroot, "include"))

    # Find the arm-none-eabi-gcc libgcc path including stdbool.h
    cmd = ["arm-none-eabi-gcc"] + ["-E", "-v", "-xc", "-"]
    try:
        out = (
            subprocess.check_output(
                cmd, stdin=open("/dev/null"), stderr=subprocess.STDOUT
            )
            .decode("utf8")
            .strip()
            .splitlines()
        )
        if "#include <...> search starts here:" in out:
            libgcc_include_path = out[
                out.index("#include <...> search starts here:") + 1
            ].strip()
            args.append("-I%s" % libgcc_include_path)
    except Exception as err:
        print("Could not run arm-none-eabi-gcc path detection %r" % err)

    if not os.path.isfile(filename):
        raise Exception("Invalid filename: " + filename)

    args.append("-ffreestanding")
    index = clang.cindex.Index.create()
    tu = index.parse(
        filename,
        args=args,
        options=clang.cindex.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD,
    )

    extract_declarations(tu, filenames, func)

    for d in tu.diagnostics:
        if (
            d.severity >= clang.cindex.Diagnostic.Error
            and d.spelling != "conflicting types for 'itoa'"
        ):
            if d.severity == clang.cindex.Diagnostic.Error:
                error_str = "Error: %s" % d.__repr__()
            elif d.severity == clang.cindex.Diagnostic.Fatal:
                error_str = "Fatal: %s" % d.__repr__()

            class ParsingException(Exception):
                pass

            raise ParsingException(error_str)


do_libclang_setup()
