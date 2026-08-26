# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
PDC_GEN converts SVG images, SVG sequences, or JSON sequences to a PDC (Pebble Draw Command) binary format image or sequence. The PDC file format
consists of a header, followed by the binary representation of a PDC image or sequence.
The file header is as follows:
Magic Word (4 bytes) - 'PDCI' for image, 'PDCS' for sequence
Size (4 bytes) - size of PDC image or sequence following the header in bytes
"""

import argparse
import os

from . import json2commands, pebble_commands, svg2commands


def create_pdc_data_from_path(
    path, viewbox_size, verbose, duration, play_count, precise=False, raise_error=False
):
    dir_name = path
    output = b""
    errors = []
    if not os.path.exists(path):
        raise Exception("Invalid path")

    if verbose:
        print(path + ":")
    if os.path.isfile(path):
        dir_name = os.path.dirname(path)
    frames = []
    commands = []

    if os.path.isfile(path):
        ext = os.path.splitext(path)[-1]
        if ext == ".json":
            # JSON file
            result = json2commands.parse_json_sequence(
                path, viewbox_size, precise, raise_error
            )
            if result:
                frames = result[0]
                errors += result[1]
                frame_duration = result[2]
                output = pebble_commands.serialize_sequence(
                    frames, viewbox_size, frame_duration, play_count
                )
        elif ext == ".svg":
            # SVG file
            size, commands, error = svg2commands.parse_svg_image(
                path, verbose, precise, raise_error
            )
            if commands:
                output = pebble_commands.serialize_image(commands, size)
            if error:
                errors += [path]
    else:
        # SVG files
        # get all .svg files in directory
        result = svg2commands.parse_svg_sequence(
            dir_name, verbose, precise, raise_error
        )
        if result:
            frames = result[1]
            size = result[0]
            errors += result[2]
            output = pebble_commands.serialize_sequence(
                frames, size, duration, play_count
            )

    if verbose:
        if frames:
            pebble_commands.print_frames(frames)
        elif commands:
            pebble_commands.print_commands(commands)

    return output, errors


def create_pdc_from_path(
    path,
    out_path,
    viewbox_size,
    verbose,
    duration,
    play_count,
    precise=False,
    raise_error=False,
):

    output, errors = create_pdc_data_from_path(
        path,
        viewbox_size,
        verbose,
        duration,
        play_count,
        precise=False,
        raise_error=False,
    )

    if output != b"":
        if out_path is None:
            if sequence:
                f = os.path.basename(dir_name.rstrip("/")) + ".pdc"
            else:
                base = os.path.basename(path)
                f = ".".join(base.split(".")[:-1]) + ".pdc"
            out_path = os.path.join(dir_name, f)
        with open(out_path, "wb") as out_file:
            out_file.write(output)
            out_file.close()

    return errors


def main(args):
    path = os.path.abspath(args.path)
    viewbox_size = (args.viewbox_x, args.viewbox_y)
    errors = create_pdc_from_path(
        path,
        args.output,
        viewbox_size,
        args.verbose,
        args.duration,
        args.play_count,
        args.precise,
    )
    if errors:
        print("Errors in the following files or frames:")
        for ef in errors:
            print("\t" + str(ef))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "path", type=str, help="Path to svg file or directory (with multiple svg files)"
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output file path (.pdc will be appended to file name if it is not included in the path "
        "specified",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    parser.add_argument(
        "-d",
        "--duration",
        type=int,
        default=33,
        help="Duration (ms) of each frame in a sequence (SVG sequence only) - default = 33ms",
    )
    parser.add_argument(
        "-c",
        "--play_count",
        type=int,
        default=1,
        help="Number of times the sequence should play - default = 1",
    )
    parser.add_argument(
        "-p", "--precise", action="store_true", help="Use sub-pixel precision for paths"
    )
    parser.add_argument(
        "-x",
        "--viewbox_x",
        help="Viewbox length (JSON sequence only)",
        type=int,
        default=json2commands.DISPLAY_DIM_X,
    )
    parser.add_argument(
        "-y",
        "--viewbox_y",
        help="Viewbox height (JSON sequence only)",
        type=int,
        default=json2commands.DISPLAY_DIM_Y,
    )
    args = parser.parse_args()
    main(args)
