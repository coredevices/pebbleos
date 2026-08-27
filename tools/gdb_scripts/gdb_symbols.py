# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import re
from collections import defaultdict

import gdb

INFO_LINE_RE = re.compile(r"(.+\s)*(\S+)\s+(\S+);")
INFO_FILE_RE = re.compile(r"File ([^:]+):$")


def _do_info_line_match(line):
    m = INFO_LINE_RE.match(line)
    if not m:
        return None
    _type = m.group(2)
    if _type.startswith("0x"):
        # This is the address rather than the type
        _type = None
    symbol = m.group(3)
    if symbol.startswith("*"):
        symbol = symbol[1:]
        if _type is not None:
            _type += " *"

    paren_loc = symbol.find("[")
    if paren_loc > 0:
        symbol = symbol[:paren_loc]
    symbol = f"'{symbol}'"
    return (_type, symbol)


def _find_match_for_file(gdb_output, _file):
    matches = []
    in_file = False
    for line in gdb_output.split("\n"):
        if not in_file:
            # matching file
            m = INFO_FILE_RE.match(line)
            if m and m.group(1).endswith(_file):
                in_file = True
        else:
            if not line.strip():
                break  # Done the file
            result = _do_info_line_match(line)
            if result is not None:
                matches.append(result)
    if len(matches) == 0:
        return (None, None)
    if len(matches) > 1:
        raise RuntimeError("Error: Multiple statics by same name")
    return matches[0]


def _find_match(gdb_output, _file=None):
    if _file is not None:
        return _find_match_for_file(gdb_output, _file)
    matches = []
    for line in gdb_output.split("\n"):
        result = _do_info_line_match(line)
        if result is not None:
            matches.append(result)
    if len(matches) == 0:
        return (None, None)
    if len(matches) > 1:
        raise RuntimeError("Error: Multiple statics by same name")
    return matches[0]


def _run_info(symbol_name, _type):
    out = gdb.execute(f"info {_type} {symbol_name}\\b", False, True)
    return out


def get_static_variable(variable_name, _file=None, ref=False):
    if get_static_variable.cache[_file][variable_name]:
        return get_static_variable.cache[_file][variable_name]

    out = _run_info(variable_name, "variables")
    (_type, symbol) = _find_match(out, _file)
    if symbol is None:
        raise RuntimeError(f'Error: Symbol matching "{variable_name}" DNE.')
    if ref:
        symbol = "&" + symbol
        if _type is not None:
            _type += " *"
    if _type:
        ret = f"(({_type}){symbol})"
    ret = f"({symbol})"
    get_static_variable.cache[_file][variable_name] = ret
    return ret


get_static_variable.cache = defaultdict(lambda: defaultdict(dict))


def get_static_function(function_name):
    _run_info(function_name, "functions")
    # TODO: Figure out what we need to do to properly find matches here.
    raise RuntimeError("Not yet implemented.")
