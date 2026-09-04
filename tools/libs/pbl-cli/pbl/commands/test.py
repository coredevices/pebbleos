# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import sys

from pbl.command import PblCommand


class Test(PblCommand):
    group = "build"

    def __init__(self):
        super().__init__(
            "test",
            "Build and run the unit tests",
            "The unit tests are a CMake project of their own, built for the "
            "host. Anything this command does not recognize is passed straight "
            "to ctest, e.g. -R REGEX to select tests.",
            accepts_unknown_args=True,
        )

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "-G",
            "--generator",
            default="Ninja",
            help="CMake generator (default: Ninja)",
        )
        parser.add_argument(
            "-C",
            "--coverage",
            action="store_true",
            help="Instrument the tests and write an lcov report",
        )
        parser.add_argument(
            "--no-images",
            action="store_true",
            help="Skip the image fixtures; only some tests need them",
        )
        parser.add_argument(
            "--build-only", action="store_true", help="Build the tests without running"
        )
        return parser

    def do_run(self, args, unknown):
        build = self.test_dir()

        rc = self.run_cmd(
            [
                "cmake",
                "-B",
                str(build),
                "-S",
                os.path.join(self.topdir, "tests"),
                "-G",
                args.generator,
                "-DPBL_TEST_COVERAGE=" + ("ON" if args.coverage else "OFF"),
                "-DPBL_TEST_IMAGES=" + ("OFF" if args.no_images else "ON"),
            ]
        )
        if rc:
            return rc
        rc = self.run_cmd(["cmake", "--build", str(build)])
        if rc or args.build_only:
            return rc

        # Always leave a JUnit report behind for CI to pick up, and use every
        # core unless the caller asked for a specific -j.
        cmd = [
            "ctest",
            "--test-dir",
            str(build),
            "--output-on-failure",
            "--output-junit",
            build.join("junit.xml"),
        ]
        if not any(a.startswith("-j") or a == "--parallel" for a in unknown):
            cmd += ["-j", str(os.cpu_count() or 1)]
        rc = self.run_cmd(cmd + unknown)

        if args.coverage:
            rc = self._lcov_report(build) or rc
        return rc

    def _lcov_report(self, build):
        """Turn the gcov data the run produced into lcov.info and HTML."""
        info = build.join("lcov.info")
        html = build.join("lcov-html")
        gcov = ["--gcov-tool", "llvm-cov"] if sys.platform.startswith("linux") else []
        steps = [
            ["lcov", "--capture", "--directory", str(build), "--output-file", info],
            # The tests themselves are not what the report is about.
            ["lcov", "--remove", info, "tests/**", "-o", info],
            ["genhtml", info, "--output-directory", html],
        ]
        for step in steps:
            rc = self.run_cmd(step + (gcov if step[0] == "lcov" else []))
            if rc:
                self.wrn("coverage report failed; are lcov and genhtml installed?")
                return rc
        self.inf(f"coverage report at {html}/index.html", color="green")
        return 0
