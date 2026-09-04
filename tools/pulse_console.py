# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import os
import signal
import threading

from log_hashing.logdehash import LogDehash
from pebble import commander, pulse2
from prompt_toolkit import PromptSession
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.patch_stdout import patch_stdout

PROMPT_STRING = "> "


def handle_prompt_command(interface, session):
    cmd = session.prompt(PROMPT_STRING)
    if not cmd:
        return

    link = interface.get_link()
    prompt = commander.apps.Prompt(link)

    try:
        for line in prompt.command_and_response(cmd):
            print(line)
    except commander.exceptions.CommandTimedOut:
        print(f"Command '{cmd}' timed out")
    finally:
        prompt.close()


def handle_log_messages(interface, dehasher):
    logging = commander.apps.StreamingLogs(interface)
    while True:
        try:
            msg = logging.receive(block=True)
        except pulse2.exceptions.SocketClosed:
            print("--- Connection closed ---", flush=True)
            # Wake the main thread out of its blocking prompt so the
            # process exits promptly instead of waiting for the user to
            # hit Enter.  The main loop catches KeyboardInterrupt and
            # exits cleanly; tools/console_keepalive.py will then reconnect.
            os.kill(os.getpid(), signal.SIGINT)
            break
        line_dict = dehasher.dehash(msg)
        # patch_stdout() in main() re-routes this print() above the live
        # prompt; the prompt + in-flight typing is redrawn correctly under
        # the new log line on every platform (no readline/libedit quirks).
        print(dehasher.commander_format_line(line_dict))


def start_logging_thread(*args):
    log_thread = threading.Thread(target=handle_log_messages, args=args)
    log_thread.daemon = True
    log_thread.start()


def generate_dehash_arguments():
    def yes_no_to_bool(arg):
        return arg == "yes"

    args = {
        "justify": "small",
        "color": True,
        "bold": -1,
        "print_core": False,
        "dict_path": os.environ.get(
            "PBL_CONSOLE_DICT_PATH", "build/src/fw/loghash_dict.json"
        ),
    }

    arglist = os.getenv("PBL_CONSOLE_ARGS")
    if arglist:
        for arg in arglist.split(","):
            if not arg:
                break
            key, value = arg.split("=")
            if key == "--justify":
                args["justify"] = value
            elif key == "--color":
                args["color"] = yes_no_to_bool(value)
            elif key == "--bold":
                args["bold"] = int(value)
            elif key == "--dict":
                args["dict_path"] = value
            elif key == "--core":
                args["print_core"] = yes_no_to_bool(value)
            else:
                raise RuntimeError(
                    "Unknown console argument '{}'. Choices are ({})".format(
                        key, ["--justify", "--color", "--bold", "--dict", "--core"]
                    )
                )

    return args


def main():
    parser = argparse.ArgumentParser(description="Pebble Console")
    parser.add_argument(
        "-t",
        "--tty",
        help="serial port (defaults to auto-detect)",
        metavar="TTY",
        default=None,
    )

    args = parser.parse_args()
    interface = pulse2.Interface.open_dbgserial(url=args.tty)

    dehasher = LogDehash(**generate_dehash_arguments())

    start_logging_thread(interface, dehasher)

    print(f"--- PULSE terminal on {args.tty} ---")
    print("--- Ctrl-C or Ctrl-D to exit ---")

    session = PromptSession(history=InMemoryHistory())

    try:
        with patch_stdout(raw=True):
            while True:
                handle_prompt_command(interface, session)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        interface.close()


if __name__ == "__main__":
    main()
