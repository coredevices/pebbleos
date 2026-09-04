# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import platform
import shlex
import time

from pbl import emulator
from pbl.command import CommandContextError, PblCommand

# The image the emulator boots from, and the one it keeps stored apps in.
MICRO_FLASH_TARGET = "qemu_image_micro"
SPI_FLASH_TARGET = "qemu_image_spi"
SPI_FLASH_IMAGE = "qemu_spi_flash.bin"

NO_DECORATION = "none"


class _QemuCommand(PblCommand):
    group = "emulator"

    def emulated_build(self):
        """The build directory, refusing boards QEMU cannot run."""
        build = self.build_dir()
        if not build.config.CONFIG_QEMU:
            raise CommandContextError(
                f"board {build.board} is not an emulated board"
            )
        return build


class Qemu(_QemuCommand):
    def __init__(self):
        super().__init__("qemu", "Launch the firmware under QEMU")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "--keep-flash-image",
            action="store_true",
            help="Keep the SPI flash image, and the apps and data in it, "
            "instead of rebuilding it",
        )
        parser.add_argument(
            "--decoration",
            metavar="NAME",
            help="SDL decoration to draw around the screen, or 'none' "
            "(default: the board's first)",
        )
        return parser

    def do_run(self, args, unknown):
        build = self.emulated_build()
        decoration = self._decoration(build, args.decoration)

        # The flash images are build artifacts. The micro flash is always
        # rebuilt; the SPI flash only unless the caller wants to keep it.
        self.cmake_build(build, MICRO_FLASH_TARGET, msg="QEMU micro flash image failed")
        spi_flash = build.join(SPI_FLASH_IMAGE)
        if not args.keep_flash_image or not os.path.isfile(spi_flash):
            self.cmake_build(build, SPI_FLASH_TARGET, msg="QEMU SPI flash image failed")

        return self.run_shell(self._command_line(build, decoration, spi_flash))

    def _decoration(self, build, requested):
        available = build.board_spec.qemu.get("decorations", [])
        if requested is None:
            return available[0] if available else None
        if requested == NO_DECORATION:
            return None
        if requested not in available:
            raise CommandContextError(
                f"board {build.board} has no '{requested}' decoration; "
                f"available: {', '.join(available + [NO_DECORATION])}"
            )
        return requested

    def _machine_args(self, build, spi_flash):
        machine = build.config.CONFIG_QEMU_MACHINE
        if not machine or machine == "unknown":
            raise CommandContextError(
                f"board {build.board} declares no QEMU machine"
            )

        if build.config.CONFIG_PLATFORM_EMERY or build.config.CONFIG_PLATFORM_FLINT:
            driver = "coreaudio" if platform.system() == "Darwin" else "sdl"
            machine_args = [
                "-machine", f"{machine},audiodev=snd0",
                "-audiodev", f"{driver},id=snd0",
            ]
        else:
            machine_args = ["-machine", machine]

        # The ELF is loaded as the kernel so QEMU sets the vector table up.
        return machine_args + [
            "-kernel", build.elf,
            "-drive", f"if=mtd,format=raw,file={spi_flash}",
        ]

    def _command_line(self, build, decoration, spi_flash):
        qemu = os.getenv("PEBBLE_QEMU_BIN")
        if not qemu or not (os.path.isfile(qemu) and os.access(qemu, os.X_OK)):
            qemu = "qemu-pebble"

        machine_args = self._machine_args(build, spi_flash)

        display = f"sdl,decoration={decoration}" if decoration else "sdl"
        machine_args += ["-display", f"{display},show-cursor=on"]

        # Both control sockets are recreated on every launch.
        monitor = build.join(emulator.MONITOR_SOCKET)
        qmp = build.join(emulator.QMP_SOCKET)
        if not self.dry_run:
            for socket in (monitor, qmp):
                if os.path.exists(socket):
                    os.unlink(socket)

        launch = [
            qemu,
            "-rtc", "base=localtime",
            "-monitor", "stdio",
            "-monitor", f"unix:{monitor},server=on,wait=off",
            "-qmp", f"unix:{qmp},server=on,wait=off",
            "-s",
            "-serial", "file:uart1.log",
            "-serial", f"tcp::{emulator.PEBBLE_TOOL_PORT},server=on,wait=off",
            "-serial", f"tcp::{emulator.CONSOLE_PORT},server=on,wait=off",
        ] + machine_args

        command = shlex.join(launch)
        self.inf("QEMU command:", command)
        return command


class Screenshot(_QemuCommand):
    def __init__(self):
        super().__init__("screenshot", "Capture the emulator's screen to a PNG")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "-o",
            "--output",
            type=os.path.abspath,
            help="Where to write the PNG (default: screenshot.png in the build)",
        )
        return parser

    def do_run(self, args, unknown):
        build = self.emulated_build()
        output = args.output or build.join("screenshot.png")
        if not output.lower().endswith(".png"):
            self.die("the screenshot's output path must end in .png")
        if os.path.exists(output):
            os.unlink(output)

        with emulator.Monitor(build.join(emulator.MONITOR_SOCKET)) as monitor:
            response = monitor.command(f"screendump {output} -f png")

        if not os.path.exists(output) or os.path.getsize(output) == 0:
            self.die(
                f"QEMU wrote no screenshot to {output}\nmonitor response:\n{response}"
            )
        self.inf("wrote", output)


class _InputCommand(_QemuCommand):
    """Injects touch events into the running emulator."""

    def qmp(self, build):
        return emulator.Qmp(build.join(emulator.QMP_SOCKET))


class Touch(_InputCommand):
    def __init__(self):
        super().__init__("touch", "Tap the emulator's screen at a pixel")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument("x", type=int, help="x coordinate in screen pixels")
        parser.add_argument("y", type=int, help="y coordinate in screen pixels")
        return parser

    def do_run(self, args, unknown):
        build = self.emulated_build()
        with self.qmp(build) as qmp:
            width, height = qmp.touch_display()
            qmp.send(qmp.move(width, height, args.x, args.y) + qmp.button(True))
            time.sleep(0.05)
            qmp.send(qmp.button(False))
        self.inf(f"tapped ({args.x}, {args.y}) on {width}x{height}")


class Swipe(_InputCommand):
    def __init__(self):
        super().__init__("swipe", "Swipe across the emulator's screen")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument("x1", type=int, help="start x in screen pixels")
        parser.add_argument("y1", type=int, help="start y in screen pixels")
        parser.add_argument("x2", type=int, help="end x in screen pixels")
        parser.add_argument("y2", type=int, help="end y in screen pixels")
        parser.add_argument(
            "--steps", type=int, default=12, help="number of move steps"
        )
        parser.add_argument(
            "--duration", type=float, default=0.25, help="total duration in seconds"
        )
        return parser

    def do_run(self, args, unknown):
        build = self.emulated_build()
        x1, y1, x2, y2 = args.x1, args.y1, args.x2, args.y2
        steps = max(1, args.steps)

        with self.qmp(build) as qmp:
            width, height = qmp.touch_display()
            qmp.send(qmp.move(width, height, x1, y1) + qmp.button(True))
            for step in range(1, steps + 1):
                qmp.send(
                    qmp.move(
                        width,
                        height,
                        x1 + (x2 - x1) * step // steps,
                        y1 + (y2 - y1) * step // steps,
                    )
                )
                time.sleep(args.duration / steps)
            qmp.send(qmp.button(False))
        self.inf(f"swiped ({x1}, {y1}) -> ({x2}, {y2}) on {width}x{height}")
