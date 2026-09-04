# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

from pbl.command import PblCommand


class Build(PblCommand):
    group = "build"

    def __init__(self):
        super().__init__(
            "build",
            "Build the firmware",
            "Build the firmware, or the named CMake targets (e.g. 'sdk').",
        )

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "targets", nargs="*", metavar="TARGET", help="CMake targets to build"
        )
        return parser

    def do_run(self, args, unknown):
        return self.cmake_build(self.build_dir(), *args.targets)


class Bundle(PblCommand):
    group = "build"

    def __init__(self):
        super().__init__("bundle", "Package the firmware into a .pbz bundle")

    def do_add_parser(self, parser_adder):
        return self.add_subparser(parser_adder)

    def do_run(self, args, unknown):
        return self.cmake_build(self.build_dir(), "bundle")


class Clean(PblCommand):
    group = "build"

    def __init__(self):
        super().__init__("clean", "Remove the firmware build's outputs")

    def do_add_parser(self, parser_adder):
        return self.add_subparser(parser_adder)

    def do_run(self, args, unknown):
        return self.cmake_build(self.build_dir(), "clean")
