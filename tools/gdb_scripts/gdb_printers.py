# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import uuid

try:
    import gdb
except ImportError:
    raise RuntimeError(
        "This file is a GDB script.\n"
        "It is not intended to be run outside of GDB.\n"
        "Hint: to load a script in GDB, use `source this_file.py`"
    )
import gdb.printing


class grectPrinter:
    """Print a GRect struct as a fragment of C code."""

    def __init__(self, val):
        self.val = val

    def to_string(self):
        code = (
            "(GRect) { .origin = { .x = %i, .y = %i }, .size = { .w = %i, .h = %i } }"
        )
        return code % (
            int(self.val["origin"]["x"]),
            int(self.val["origin"]["y"]),
            int(self.val["size"]["w"]),
            int(self.val["size"]["h"]),
        )


class gpathInfoPrinter:
    """Print a GPathInfo struct as a fragment of C code."""

    def __init__(self, val):
        self.val = val

    def to_string(self):
        points_code = ""
        num_points = int(self.val["num_points"])
        array_val = self.val["points"]
        for i in range(num_points):
            point_val = array_val[i]
            if points_code:
                points_code += ", "
            points_code += "{{ {:d}, {:d} }}".format(point_val["x"], point_val["y"])
        outer_code_fmt = "(GPathInfo) { .num_points = %i, .points = (GPoint[]) {%s} }"
        return outer_code_fmt % (num_points, points_code)


class UuidPrinter:
    """Print a UUID."""

    def __init__(self, val):
        data = bytes(int(val[f"byte{n:d}"]) for n in range(16))
        self.uuid = uuid.UUID(bytes=data)

    def to_string(self):
        return f"{{{self.uuid}}}"


pp = gdb.printing.RegexpCollectionPrettyPrinter("tintin")
pp.add_printer("GRect", "^GRect$", grectPrinter)
pp.add_printer("GPathInfo", "^GPathInfo$", gpathInfoPrinter)
pp.add_printer("Uuid", "^Uuid$", UuidPrinter)
# Register the pretty-printer globally
gdb.printing.register_pretty_printer(None, pp, replace=True)
