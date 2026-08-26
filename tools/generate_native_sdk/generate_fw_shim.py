# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os.path

FUNCTION_PTR_FILE = "pebble.auto.c"


def gen_function_pointer_array(functions):
    output = []

    # Include headers for the standard functions
    output.append("#include <stdlib.h>")
    output.append("#include <stdio.h>")
    output.append("#include <string.h>")
    output.append("")

    for f in functions:
        if not f.removed and (not f.skip_definition or f.impl_name != f.name):
            output.append("extern void %s();" % f.impl_name)

    output.append("")
    output.append("const void* const g_pbl_system_tbl[] = {")

    function_ptrs = []
    for f in functions:
        if not f.removed:
            function_ptrs.append("&%s" % f.impl_name)
        else:
            function_ptrs.append("0")

    output.extend("  %s," % f for f in function_ptrs)
    output.append("};")
    output.append("")

    return "\n".join(output)


def make_fw_shims(functions, pbl_output_src_dir):
    with open(os.path.join(pbl_output_src_dir, "fw", FUNCTION_PTR_FILE), "w") as fptr_c:
        fptr_c.write(gen_function_pointer_array(functions))
