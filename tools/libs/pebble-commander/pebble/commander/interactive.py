# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import contextlib
import shlex
import traceback

import prompt_toolkit
from log_hashing.logdehash import LogDehash

from .commander import PebbleCommander


class InteractivePebbleCommander:
    """Interactive Pebble Commander.
    Most/all UI implementations should either use this directly or sub-class it.
    """

    def __init__(self, loghash_path=None, tty=None, capfile=None):
        self.cmdr = PebbleCommander(tty=tty, interactive=True, capfile=capfile)
        if loghash_path is None:
            loghash_path = "build/src/fw/loghash_dict.json"
        self.dehasher = LogDehash(loghash_path)
        self.cmdr.attach_log_listener(self.log_listener)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __del__(self):
        self.close()

    def close(self):
        with contextlib.suppress(Exception):
            self.cmdr.close()

    def attach_prompt_toolkit(self):
        """Attaches prompt_toolkit things"""
        self.history = prompt_toolkit.history.InMemoryHistory()
        self.cli = prompt_toolkit.CommandLineInterface(
            application=prompt_toolkit.shortcuts.create_prompt_application(
                "> ", history=self.history
            ),
            eventloop=prompt_toolkit.shortcuts.create_eventloop(),
        )
        self.patch_context = self.cli.patch_stdout_context(raw=True)
        self.patch_context.__enter__()

    def log_listener(self, msg):
        """This is called on every incoming log message.
        `msg` is the raw log message class, without any dehashing.

        Subclasses should override this probably.
        """
        line_dict = self.dehasher.dehash(msg)
        line = self.dehasher.commander_format_line(line_dict)
        print(line)

    def dispatch_command(self, string):
        """Dispatches a command string.

        Subclasses should not override this.
        """
        args = shlex.split(string)
        # Starting with '!' passes the rest of the line directly to prompt.
        # Otherwise we try to run a command; if that fails, the line goes to prompt.
        if string.startswith("!"):
            string = string[1:]  # Chop off the '!' marker
        else:
            cmd = self.cmdr.get_command(args[0])
            if cmd:  # If we provide the command, run it.
                return cmd(*args[1:])

        return self.cmdr.send_prompt_command(string)

    def input_handle(self, string):
        """Handles an input line.
        Generally the flow is to handle any UI-specific commands, then pass on to
        dispatch_command.

        Subclasses should override this probably.
        """
        # Handle "quit" strings
        if string in ["exit", "q", "quit"]:
            return False

        try:
            resp = self.dispatch_command(string)
            if resp is not None:
                print("\x1b[1m" + "\n".join(resp) + "\x1b[m")
        except Exception:  # noqa: BLE001
            print("An error occurred!")
            traceback.print_exc()

        return True

    def get_command(self):
        """Get a command input line.
        If there is no line, return an empty string or None.
        This may block.

        Subclasses should override this probably.
        """
        if self.cli is None:
            self.attach_prompt_toolkit()
        doc = self.cli.run(reset_current_buffer=True)
        if doc:
            return doc.text
        else:
            return None

    def command_loop(self):
        """The main command loop.

        Subclasses could override this, but it's probably not useful to do.
        """
        while True:
            try:
                cmd = self.get_command()
                if cmd and not self.input_handle(cmd):
                    break
            except (KeyboardInterrupt, EOFError):
                break
