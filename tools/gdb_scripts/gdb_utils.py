# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

try:
    import gdb
except ImportError:
    raise RuntimeError(
        "This file is a GDB script.\n"
        "It is not intended to be run outside of GDB.\n"
        "Hint: to load a script in GDB, use `source this_file.py`"
    )
import argparse
import re
from collections import namedtuple


class GdbArgumentParser(argparse.ArgumentParser):
    def parse_args(self, args=None, namespace=None):
        if args:
            args = gdb.string_to_argv(args)
        return argparse.ArgumentParser.parse_args(self, args, namespace)

    def exit(self, status=0, msg=None):
        raise gdb.GdbError(msg)


class AddressInfo(namedtuple("AddressInfo", "filename line addr")):
    def __new__(cls, filename, line, addr):
        return cls._make([filename, line, addr])

    def __str__(self):
        return f"{self.filename}:{self.line} (0x{self.addr:0>8x})"


def addr2line(addr_value):
    """Convenience function to return a string with code line for an address.
    The function takes an int or a gdb.Value that can be converted to an int.
    The format used is: `file_name:line_number (0xhex_address)`

    """
    addr = int(addr_value)
    s = gdb.find_pc_line(addr)
    filename = None
    line = None
    if s and s.symtab:
        filename = re.sub(r"^[./]+", "", s.symtab.filename)
        line = s.line
    if not filename:
        filename = "?"
    if not line:
        line = "?"
    return AddressInfo(filename, line, addr)


class Address(int):
    """Convenience subclass of `int` that accepts a hexadecimal string in its
    constructor. It also accepts a gdb.Value in its constructor, in which case
    the address of the value will be attempted to be used to create the object.
    Its `__repr__` prints its value formatted as hexadecimal.

    """

    ADDR_REGEX = re.compile(r"^\s*(0x[a-fA-F0-9]{7,8})")

    def __new__(cls, *args, **kwargs):
        if args:
            val = args[0]
            if isinstance(val, gdb.Value):
                if val.address:
                    # If the value has an address, use it:
                    val = str(val.address)
                else:
                    # Otherwise, attempt to use that the value as an int:
                    val = int(val)
            if isinstance(val, str):
                # GDB tends to append symbolic info to the string, even for
                # a gdb.Value that was acquired from the `address` attribute...
                match = Address.ADDR_REGEX.match(val)
                if match:
                    val = match.group(1)
                    return super().__new__(cls, val, base=16)
        return super().__new__(cls, *args, **kwargs)

    def __repr__(self):
        return f"0x{self:08x}"

    def __str__(self):
        return self.__repr__()


class ActionBreakpoint(gdb.Breakpoint):
    """
    Convenience wrapper around gdb.Breakpoint.

    The first argument to the constructor is a Callable, the ActionBreakpoint's
    attribute `action_callable` will be set to this object. The Callable must
    accept one argument, which will be set to the instance of the
    ActionBreakpoint instance itself.

    The second (optional) argument to the constructor is the name (str) of the
    symbol on which to set the breakpoint. If not specified, the __name__ of
    the callable is used as the name of the symbol, for convenience.

    ActionBreakpoint implements the method `def handle_break(self)` that is
    called when the breakpoint is hit. The base implementation just calls the
    `action_callable` attribute.

    Usage example:

    def window_stack_push(breakpoint):
        print "window_stack_push() called!"
    bp = ActionBreakpoint(window_stack_push)

    """

    def stop_handler(event):
        if isinstance(event, gdb.BreakpointEvent):
            for breakpoint in event.breakpoints:
                if isinstance(breakpoint, ActionBreakpoint):
                    breakpoint.handle_break()

    gdb.events.stop.connect(stop_handler)  # Register with gdb module

    def __init__(
        self, action_callable, symbol_name=None, addr=None, auto_continue=True
    ):
        if addr and symbol_name:
            raise RuntimeError(
                "Can't use arguments `symbol_name` and `addr` simultaneously!"
            )
        if addr:
            # When an address is specified, the expression must be prepended
            # with an `*` (not to be confused with a dereference...):
            # https://sourceware.org/gdb/onlinedocs/gdb/Specify-Location.html
            symbol_name = "*" + str(addr)
        if not symbol_name:
            symbol_name = action_callable.__name__
        super().__init__(symbol_name)
        self.action_callable = action_callable
        self.auto_continue = auto_continue

    def handle_break(self):
        self.action_callable(self)
        if self.auto_continue:
            gdb.execute("continue")


class MonkeyPatch(ActionBreakpoint):
    """
    Object that `overrides` an existing function in the program being
    debugged, using ActionBreakpoint and the `return` GDB command.

    The first argument is the callable that provides the return value as a GDB
    expression (str) or returns `None` in case the function returns void.
    The (optional) second argument is a string of the symbol of the function to
    monkey-patch. If not specified, the __name__ of the callable is used as the
    name of the symbol, for convenience.

    Usage example:

    def my_function_override(monkey_patch):
        return "(int) 123"
    patch = MonkeyPatch(my_function_override, "my_existing_function")

    """

    def handle_break(self):
        return_value_str = self.action_callable(self)
        gdb.write(f"Hit monkey patch {self}, returning `{return_value_str}`")
        if return_value_str:
            gdb.execute(f"return ({return_value_str})")
        else:
            gdb.execute("return")
        gdb.execute("continue")


GET_ARGS_RE = re.compile("^[_A-z]+[_A-z0-9]*")


def get_args():
    """
    Returns a dict of gdb.Value objects of the arguments in scope.
    """
    args = {}
    info_args_str = gdb.execute("info args", to_string=True)
    print(info_args_str)
    for line in info_args_str.splitlines():
        match = GET_ARGS_RE.search(line)
        var_name = match.group()
        args[var_name] = gdb.parse_and_eval(var_name)
    return args
