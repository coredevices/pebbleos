# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
JSON2COMMANDS creates Pebble Draw Commands (the Python Objects, _not_ a serialized .pdc) from a JSON file.
Currently only the PathCommand is supported.

The JSON file can contain multiple frames (i.e. PDC Sequence).
Each frame is composed of 'fillGroups'.
A fillGroup may be: An individual filled polygon (a.k.a. a fill), or _all_ unfilled polylines (a.k.a. all open paths).
Each fillGroup is parsed separately and a list of Pebble Draw Commands that describe it is created.
The created list should have the length of the lowest number of commands possible in order to draw that fillGroup.

Currently, there is no support for a JSON to contain the viewbox size or fill colors.
The viewbox size is currently passed in as a parameter.
The fill color is currently defaulted to solid white.
"""

import argparse
import json
import os
from itertools import groupby

from . import graph, pebble_commands

INVISIBLE_POINT_THRESHOLD = 500
DISPLAY_DIM_X = 144
DISPLAY_DIM_Y = 168
OPEN_PATH_TAG = "_"


def parse_color(color_opacity, truncate):
    if color_opacity is None:
        return 0

    r = round(255 * color_opacity[0])
    g = round(255 * color_opacity[1])
    b = round(255 * color_opacity[2])
    a = round(255 * color_opacity[3])

    return pebble_commands.convert_color(r, g, b, a, truncate)


def parse_json_line_data(json_line_data, viewbox_size=(DISPLAY_DIM_X, DISPLAY_DIM_Y)):
    # A list of one-way vectors, but intended to store their negatives at all times.
    bidirectional_lines = []

    for line_data in json_line_data:
        # Skip invisible lines
        if (
            abs(line_data["startPoint"][0]) > INVISIBLE_POINT_THRESHOLD
            or abs(line_data["startPoint"][1]) > INVISIBLE_POINT_THRESHOLD
            or abs(line_data["endPoint"][0]) > INVISIBLE_POINT_THRESHOLD
            or abs(line_data["endPoint"][1]) > INVISIBLE_POINT_THRESHOLD
        ):
            continue

        # Center the viewbox of all lines (by moving the lines' absolute
        # coordinates relative to the screen)
        dx = -(DISPLAY_DIM_X - viewbox_size[0]) / 2
        dy = -(DISPLAY_DIM_Y - viewbox_size[1]) / 2
        start_point = (line_data["startPoint"][0] + dx, line_data["startPoint"][1] + dy)
        end_point = (line_data["endPoint"][0] + dx, line_data["endPoint"][1] + dy)

        # Since lines are represented and stored as one-way vectors, but may be
        # drawn in either direction, all operations must be done on their reverse
        line = (start_point, end_point)
        reverse_line = (end_point, start_point)

        # Skip duplicate lines
        if line in bidirectional_lines:
            continue

        bidirectional_lines.append(line)
        bidirectional_lines.append(reverse_line)

    return bidirectional_lines


def determine_longest_path(bidirectional_lines):
    """
    Returns the longest path in 'bidirectional_lines', and removes all its segments from 'bidirectional_lines'
    If 'bidirectional_lines' contains more than one possible longest path, only one will be returned.
    """
    # Construct graph out of bidirectional_lines
    g = graph.Graph({})
    for line in bidirectional_lines:
        g.add_edge(line)

    # Find longest path
    longest_path_length = 0
    longest_path = []
    vertices = g.get_vertices()
    for i in range(len(vertices)):
        start_vertex = vertices[i]

        for j in range(i, len(vertices)):
            end_vertex = vertices[j]
            paths = g.find_all_paths(start_vertex, end_vertex)
            for path in paths:
                if (len(path) - 1) > longest_path_length:
                    longest_path = path
                    longest_path_length = len(path) - 1

    # Edge case - Line is a point
    if len(longest_path) == 1:
        longest_path = [longest_path, longest_path]

    # Remove longest_path's line segments from bidirectional_lines
    # Since bidirectional_lines is a list of one-way vectors but represents
    # bidirectional lines, a line segment and its reverse must be removed to
    # keep its integrity
    for k in range(len(longest_path) - 1):
        path_line = (longest_path[k], longest_path[k + 1])
        reverse_path_line = (path_line[1], path_line[0])
        bidirectional_lines.remove(path_line)
        bidirectional_lines.remove(reverse_path_line)

    return longest_path


def process_unique_group_of_lines(
    unique_group_data,
    translate,
    viewbox_size,
    path_open,
    stroke_width,
    stroke_color,
    fill_color,
    precise,
    raise_error,
):
    """
    Creates a list of commands that draw out a unique group of lines.

    A unique group of lines is defined as having a unique stroke width, stroke color, and fill.
    Note that this does _not_ guarantee the group may be described by a single Pebble Draw Command.
    """

    unique_group_commands = []

    bidirectional_lines = parse_json_line_data(unique_group_data, viewbox_size)
    if not bidirectional_lines:
        return unique_group_commands

    while bidirectional_lines:
        longest_path = determine_longest_path(bidirectional_lines)

        c = pebble_commands.PathCommand(
            longest_path,
            path_open,
            translate,
            stroke_width,
            stroke_color,
            fill_color,
            precise,
            raise_error,
        )

        if c is not None:
            unique_group_commands.append(c)

    return unique_group_commands


def process_fill(
    fillGroup_data,
    translate,
    viewbox_size,
    path_open,
    precise,
    raise_error,
    truncate_color,
):
    fill_command = []
    error = False

    # A fill is implicitly a unique group of lines - all line segments must have the same stroke width, stroke color
    # Get line style from first line segment
    stroke_width = fillGroup_data[0]["thickness"]
    stroke_color = parse_color(fillGroup_data[0]["color"], truncate_color)
    # Fill color should be solid white until it can be inserted in the JSON
    fill_color = parse_color([1, 1, 1, 1], truncate_color)
    if stroke_color == 0:
        stroke_width = 0
    elif stroke_width == 0:
        stroke_color = 0

    try:
        unique_group_commands = process_unique_group_of_lines(
            fillGroup_data,
            translate,
            viewbox_size,
            path_open,
            stroke_width,
            stroke_color,
            fill_color,
            precise,
            raise_error,
        )

        if unique_group_commands:
            fill_command += unique_group_commands
    except pebble_commands.InvalidPointException:
        error = True

    return fill_command, error


def process_open_paths(
    fillGroup_data,
    translate,
    viewbox_size,
    path_open,
    precise,
    raise_error,
    truncate_color,
):
    open_paths_commands = []
    error = False

    fill_color = parse_color([0, 0, 0, 0], truncate_color)  # No fill color

    # These open paths are part of the same fillGroup, but may have varied stroke width
    fillGroup_data = sorted(fillGroup_data, key=lambda a: a["thickness"])
    for stroke_width, unique_width_group in groupby(
        fillGroup_data, lambda c: c["thickness"]
    ):
        unique_width_data = list(unique_width_group)

        # These open paths have the same width, but may have varied color
        unique_width_data = sorted(unique_width_data, key=lambda d: d["color"])
        for stroke_color_raw, unique_width_and_color_group in groupby(
            unique_width_data, lambda e: e["color"]
        ):
            # These are a unique group of lines
            unique_width_and_color_data = list(unique_width_and_color_group)

            stroke_color = parse_color(stroke_color_raw, truncate_color)
            if stroke_color == 0:
                stroke_width = 0
            elif stroke_width == 0:
                stroke_color = 0

            try:
                unique_group_commands = process_unique_group_of_lines(
                    unique_width_and_color_data,
                    translate,
                    viewbox_size,
                    path_open,
                    stroke_width,
                    stroke_color,
                    fill_color,
                    precise,
                    raise_error,
                )

                if unique_group_commands:
                    open_paths_commands += unique_group_commands
            except pebble_commands.InvalidPointException:
                error = True

    return open_paths_commands, error


def get_commands(
    translate,
    viewbox_size,
    frame_data,
    precise=False,
    raise_error=False,
    truncate_color=True,
):
    commands = []
    errors = []

    fillGroups_data = frame_data["lineData"]

    # The 'fillGroup' property describes the type of group: A unique letter
    # (e.g. "A", "B", "C" etc.) for a unique fill, and a special identifier
    # for ALL open paths (non-fills)
    only_fills = [d for d in fillGroups_data if d["fillGroup"] != OPEN_PATH_TAG]
    only_fills = sorted(
        only_fills, key=lambda f: f["fillGroup"]
    )  # Don't assume data is sorted
    only_open_paths = [d for d in fillGroups_data if d["fillGroup"] == OPEN_PATH_TAG]
    # Fills must be drawn before open paths, so place them first
    ordered_fill_groups = only_fills + only_open_paths

    # Process fillGroups
    for path_type, fillGroup in groupby(ordered_fill_groups, lambda b: b["fillGroup"]):
        fillGroup_data = list(fillGroup)

        path_open = path_type == "_"
        if not path_open:
            # Filled fillGroup
            fillGroup_commands, error = process_fill(
                fillGroup_data,
                translate,
                viewbox_size,
                path_open,
                precise,
                raise_error,
                truncate_color,
            )
        else:
            # Open path fillGroup
            fillGroup_commands, error = process_open_paths(
                fillGroup_data,
                translate,
                viewbox_size,
                path_open,
                precise,
                raise_error,
                truncate_color,
            )

        if error:
            errors += str(path_type)
        elif fillGroup_commands:
            commands += fillGroup_commands

    if not commands:
        # Insert one 'invisible' command so the frame is valid
        c = pebble_commands.PathCommand(
            [((0.0), (0.0)), ((0.0), (0.0))], True, translate, 0, 0, 0
        )
        commands.append(c)

    return commands, errors


def parse_json_sequence(filename, viewbox_size, precise=False, raise_error=False):
    frames = []
    errors = []
    translate = (0, 0)

    with open(filename) as json_file:
        try:
            data = json.load(json_file)
        except ValueError:
            print("Invalid JSON format")
            return frames, 0, 0

        frames_data = data["lineData"]
        frame_duration = int(data["compData"]["frameDuration"] * 1000)
        for idx, frame_data in enumerate(frames_data):
            cmd_list, frame_errors = get_commands(
                translate, viewbox_size, frame_data, precise, raise_error
            )

            if frame_errors:
                errors.append((idx, frame_errors))
            elif cmd_list is not None:
                frames.append(cmd_list)

    return frames, errors, frame_duration


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=str, help="Path to json file")
    args = parser.parse_args()
    path = os.path.abspath(args.path)
    parse_json_sequence(path)
