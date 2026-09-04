#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import logging
import operator
import subprocess
import sys
import time


##########################################################################################
class Profiler:
    """This class encapsulates the profiling functionality"""

    #####################################################################################
    def __init__(self, openocd_port, pblprog_board):
        self.openocd_port = openocd_port
        self.pblprog_board = pblprog_board

    #####################################################################################
    def capture(self, filename, total_secs, sample_period_ms):
        """Sample the program counter from the board attached via openocd"""

        if self.pblprog_board:
            num_samples = 0
            from pebble import programmer

            with programmer.get_device(
                self.pblprog_board, reset=False, frequency=25e6
            ) as prog:
                pcs = prog.profile(total_secs)
                num_samples = sum(pcs.values())
            elapsed_time = total_secs
        else:
            # Setup telnet connection and discard banner
            try:
                import telnetlib

                tn = telnetlib.Telnet("localhost", self.openocd_port)
            except OSError:
                print("Could not connect to OpenOCD via telnet")
                sys.exit()

            # Discard banner
            time.sleep(1)
            tn.read_some()

            # Capture the samples
            num_samples = total_secs * 1000 / sample_period_ms
            print(f"Capturing {num_samples:d} samples...")
            n = 0
            pcs = {}

            sample_period_sec = sample_period_ms * 0.001

            start_time = time.time()
            last_sample_time = time.time()
            while n < num_samples:
                if (n % 1000) == 0:
                    print(f"{n:d}...", end=" ")
                    sys.stdout.flush()

                # Space the samples apart by the requested amount
                elapsed = time.time() - last_sample_time
                if elapsed < sample_period_sec:
                    time.sleep(sample_period_sec - elapsed)
                last_sample_time = time.time()

                # mdw = read word, 0xE000101C is the PC sampling reg
                tn.write("mdw 0xE000101C 1\n")
                _ = tn.read_until(": ")
                res = tn.read_eager().split(" ")[0]
                res = "0x" + res

                pcs[res] = pcs.get(res, 0) + 1
                n += 1
            elapsed_time = time.time() - start_time

        # Save results to a file
        print(
            f"\n{num_samples:d} samples collected in {elapsed_time:f} seconds ({elapsed_time * 1000.0 / num_samples:f} ms/sample)"
        )
        print(f"Saving samples to {filename}...")
        with open(filename, "w") as out:
            out.writelines(f"{k} {v:d}\n" for k, v in pcs.iteritems())

    #####################################################################################
    def view(self, filename, elf):

        ADDR2LINE = "arm-none-eabi-addr2line"
        output = {}

        # Read in the raw samples
        pcs = {}
        total_samples = 0
        with open(filename) as f:
            for line in f:
                addr, count = line.strip().split()
                pcs[addr] = int(count)
                total_samples += int(count)

        # Lookup the method name, filename and line number for each PC
        cmdline = [ADDR2LINE, "-e", elf, "-a", "-f"]
        cmdline += pcs.keys()
        output = subprocess.check_output(cmdline)

        # Collect results by method name and by file:line
        method_count = {}
        file_line_count = {}

        # Map file:line to method
        method_lookup = {}

        # Map PC to file:line
        file_line_lookup = {}

        items = output.splitlines()
        result_count = len(items) / 3
        for i in range(result_count):
            addr, method, file_line = items[i * 3 : (i + 1) * 3]
            if file_line == "?":
                file_line = addr
            else:
                file_line = file_line.split("/")[-1]

            count = pcs[addr]
            method_count[method] = method_count.get(method, 0) + count
            file_line_count[file_line] = file_line_count.get(file_line, 0) + count

            method_lookup[file_line] = method
            file_line_lookup[addr] = file_line

        # Print results in sorted order
        format_str = "%-64s %7.2f%%   %5d"
        print("\n\nSamples grouped by method: ")
        print("---------------------------------------------------------------")
        sorted_values = sorted(
            method_count.items(), key=operator.itemgetter(1), reverse=True
        )
        for k, v in sorted_values:
            print(format_str % (k, v * 100.0 / total_samples, v))

        print("\n\nSamples grouped by file:line: ")
        print("---------------------------------------------------------------")
        sorted_values = sorted(
            file_line_count.items(), key=operator.itemgetter(1), reverse=True
        )
        for k, v in sorted_values:
            k = f"{k}   ({method_lookup[k]:.24})"
            print(format_str % (k, v * 100.0 / total_samples, v))

        print("\n\nSamples grouped by address ")
        print("---------------------------------------------------------------")
        sorted_values = sorted(pcs.items(), key=operator.itemgetter(1), reverse=True)
        for k, v in sorted_values:
            k = f"{k}   ({file_line_lookup[k]:.48})"
            print(format_str % (k, v * 100.0 / total_samples, v))


####################################################################################################
if __name__ == "__main__":
    # Collect our command line arguments
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "action",
        choices=["capture", "view"],
        default="capture",
        help="Action to perform. capture - capture sampled data from attached "
        + "board using openocd or pblprog; view - view results from a prior "
        + "capture session",
    )

    parser.add_argument(
        "--name",
        type=str,
        default="profile.txt",
        help="Name of file to store capture data into (for 'capture' action) or "
        + "to evaluate (for 'print' action)",
    )
    parser.add_argument(
        "--pblprog_board",
        type=str,
        default=None,
        help="The board to target with pblprog",
    )
    parser.add_argument(
        "--openocd_port", type=int, default=4444, help="Telnet port of openocd"
    )
    parser.add_argument(
        "--secs",
        type=int,
        default=1,
        help="Number of seconds to capture profile information for",
    )
    parser.add_argument(
        "--sample_period_ms",
        type=int,
        default=2,
        help="Number of milliseconds between each sample",
    )
    parser.add_argument(
        "--elf",
        type=str,
        default="build/pebbleos.elf",
        help="Path to the elf file to use to lookup method and file names",
    )

    parser.add_argument("--debug", action="store_true", help="Turn on debug logging")
    args = parser.parse_args()

    level = logging.INFO
    if args.debug:
        level = logging.DEBUG
    logging.basicConfig(level=level)

    if args.sample_period_ms <= 0:
        raise RuntimeError("sample_period_ms must be >= 0")

    profiler = Profiler(args.openocd_port, args.pblprog_board)
    if args.action == "capture":
        profiler.capture(
            filename=args.name,
            total_secs=args.secs,
            sample_period_ms=args.sample_period_ms,
        )

    elif args.action == "view":
        profiler.view(filename=args.name, elf=args.elf)
