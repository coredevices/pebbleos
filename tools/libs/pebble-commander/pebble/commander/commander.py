# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import contextlib
import re
import threading
import tokenize
import types

from pebble import pulse2

from . import apps


class Pulse2ConnectionAdapter:
    """An adapter for the pulse2 API to look enough like pulse.Connection
    to make PebbleCommander work...ish.

    Prompt will break spectacularly if the firmware reboots or the link
    state otherwise changes. Commander itself needs to be modified to be
    link-state aware.
    """

    def __init__(self, interface):
        self.interface = interface
        self.logging = apps.StreamingLogs(interface)
        link = interface.get_link()
        self.prompt = apps.Prompt(link)
        self.flash = apps.FlashImaging(link)

    def close(self):
        self.interface.close()


class PebbleCommander:
    """Pebble Commander.
    Implements everything for interfacing with PULSE things.
    """

    def __init__(self, tty=None, interactive=False, capfile=None):
        if capfile is not None:
            interface = pulse2.Interface.open_dbgserial(
                url=tty, capture_stream=open(capfile, "wb")  # noqa: SIM115
            )
        else:
            interface = pulse2.Interface.open_dbgserial(url=tty)

        try:
            self.connection = Pulse2ConnectionAdapter(interface)
        except BaseException:
            interface.close()
            raise

        self.interactive = interactive

        self.log_listeners_lock = threading.Lock()
        self.log_listeners = []

        # Start the logging thread
        self.log_thread = threading.Thread(target=self._start_logging)
        self.log_thread.daemon = True
        self.log_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    @classmethod
    def command(cls, name=None):
        """Registers a command.
        `name` is the command name. If `name` is unspecified, name will be the function name
        with underscores converted to hyphens.

        The convention for `name` is to separate words with a hyphen. The function name
        will be the same as `name` with hyphens replaced with underscores.
        Example: `click-short` will result in a PebbleCommander.click_short function existing.

        `fn` should return an array of strings (or None), and take the current
        `PebbleCommander` as the first argument, and the rest of the argument strings
        as subsequent arguments. For errors, `fn` should throw an exception.

        # TODO: Probably make the return something structured instead of stringly typed.
        """

        def decorator(fn):
            # Story time:
            # <cory> Things are fine as long as you only read from `name`, but assigning to `name`
            #        creates a new local which shadows the outer scope's variable, even though it's
            #        only assigned later on in the block
            # <cory> You could work around this by doing something like `name_ = name` and using
            #        `name_` in the `decorator` scope
            cmdname = name
            if not cmdname:
                cmdname = fn.__name__.replace("_", "-")
            funcname = cmdname.replace("-", "_")
            if not re.match(tokenize.Name + "$", funcname):
                raise ValueError(f"command name {funcname} isn't a valid name")
            if hasattr(cls, funcname):
                raise ValueError(
                    f"function name {funcname} clashes with existing attribute"
                )
            fn.is_command = True
            fn.name = cmdname
            method = types.MethodType(fn, cls)
            setattr(cls, funcname, method)

            return fn

        return decorator

    def close(self):
        self.connection.close()

    def _start_logging(self):
        """Thread to handle logging messages."""
        while True:
            try:
                msg = self.connection.logging.receive()
            except pulse2.exceptions.SocketClosed:
                break
            with self.log_listeners_lock:
                # TODO: Buffer log messages if no listeners attached?
                for listener in self.log_listeners:
                    with contextlib.suppress(Exception):
                        listener(msg)

    def attach_log_listener(self, listener):
        """Attaches a listener for log messages.
        Function takes message and returns are ignored.
        """
        with self.log_listeners_lock:
            self.log_listeners.append(listener)

    def detach_log_listener(self, listener):
        """Removes a listener that was added with `attach_log_listener`"""
        with self.log_listeners_lock:
            self.log_listeners.remove(listener)

    def send_prompt_command(self, cmd):
        """Send a prompt command string.
        Unfortunately this is indeed stringly typed, a better solution is necessary.
        """
        return self.connection.prompt.command_and_response(cmd)

    def get_command(self, command):
        try:
            fn = getattr(self, command.replace("-", "_"))
            if fn.is_command:
                return fn
        except AttributeError:
            # Method doesn't exist, or isn't a command.
            pass

        return None
