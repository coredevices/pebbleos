# SPDX-FileCopyrightText: 2026 Joshua Jun
# SPDX-License-Identifier: Apache-2.0

"""Keep-alive wrapper for the serial console.

The underlying console tools (`pulse_console.py`, `miniterm_co.py`) open
their transport exactly once at startup and exit with a SerialException
if it isn't reachable.  This wrapper waits for the transport to come up,
runs the inner console, and restarts it if the transport goes away.
Works with both QEMU TCP sockets (`socket://host:port`) and serial TTY
paths (`/dev/tty.*`, `/dev/ttyUSB*`, etc.).  Press Ctrl-C while the
wrapper is waiting to quit; Ctrl-C inside the console (or just exiting
it) when the transport is still healthy is treated as an intentional
exit.
"""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time

_SOCKET_PREFIX = "socket://"


def _is_socket_url(tty: str) -> bool:
    return tty.startswith(_SOCKET_PREFIX)


def _parse_socket_url(url: str) -> tuple[str, int]:
    host, port = url[len(_SOCKET_PREFIX) :].rsplit(":", 1)
    return host, int(port)


def _is_available(tty: str) -> bool:
    if _is_socket_url(tty):
        host, port = _parse_socket_url(tty)
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            return False
    return os.path.exists(tty)


def _wait_for_transport(tty: str, interval: float = 0.5) -> None:
    while not _is_available(tty):
        time.sleep(interval)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "-t",
        "--tty",
        required=True,
        help="socket URL (socket://host:port) or serial device path",
    )
    parser.add_argument(
        "inner_cmd",
        nargs=argparse.REMAINDER,
        help=(
            "inner console command and args (default: "
            "tools/pulse_console.py -t TTY). The default command is run if "
            "none is supplied."
        ),
    )
    args = parser.parse_args()

    inner_cmd = list(args.inner_cmd)
    # argparse.REMAINDER often keeps a leading '--' that the user used to
    # separate wrapper flags from the inner command; strip it.
    if inner_cmd and inner_cmd[0] == "--":
        inner_cmd.pop(0)
    if not inner_cmd:
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        inner_cmd = [
            sys.executable,
            os.path.join(repo_root, "tools", "pulse_console.py"),
            "-t",
            args.tty,
        ]

    try:
        while True:
            if not _is_available(args.tty):
                print(
                    f"--- Waiting for {args.tty} (Ctrl-C to quit) ---",
                    flush=True,
                )
                _wait_for_transport(args.tty)

            print(f"--- Connected to {args.tty} ---", flush=True)
            subprocess.run(inner_cmd, check=False)

            if _is_available(args.tty):
                # Transport is still healthy, so the user must have
                # intentionally exited the inner console; respect that.
                break
            print("--- Connection lost; reconnecting ---", flush=True)
    except KeyboardInterrupt:
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
