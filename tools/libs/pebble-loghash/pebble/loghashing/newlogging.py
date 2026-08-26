#!/usr/bin/env python
# SPDX-FileCopyrightText: 2025 Google LLC
# SPDX-License-Identifier: Apache-2.0

# -*- coding: utf8 -*-

"""
Module for dehashing NewLog input
"""

import os
import string
import struct

from pebble.loghashing.constants import (
    HEX_FORMAT_SPECIFIER_PATTERN,
    NEWLOG_HASHED_INFO_PATTERN,
    NEWLOG_LINE_CONSOLE_PATTERN,
    NEWLOG_LINE_SUPPORT_PATTERN,
    POINTER_FORMAT_TAG_PATTERN,
)

hex_digits = set(string.hexdigits)

LOG_DICT_KEY_VERSION = "new_logging_version"
NEW_LOGGING_VERSION = "NL0102"

LOG_LEVEL_ALWAYS = 0
LOG_LEVEL_ERROR = 1
LOG_LEVEL_WARNING = 50
LOG_LEVEL_INFO = 100
LOG_LEVEL_DEBUG = 200
LOG_LEVEL_DEBUG_VERBOSE = 255

# Zephyr-style severity tags; ALWAYS logs carry no tag
level_strings_map = {
    LOG_LEVEL_ALWAYS: "",
    LOG_LEVEL_ERROR: "err",
    LOG_LEVEL_WARNING: "wrn",
    LOG_LEVEL_INFO: "inf",
    LOG_LEVEL_DEBUG: "dbg",
    LOG_LEVEL_DEBUG_VERBOSE: "vrb",
}

# Location of the core number in the message hash
PACKED_CORE_OFFSET = 30
PACKED_CORE_MASK = 0x03


def dehash_file(file_name, log_dict):
    """
    Dehash a file

    :param line: The line to dehash
    :type line: str
    :param log_dict: dict of dicts created from .log_strings section from pebbleos.elf
    :type log_dict: dict of dicts

    :returns: A list containing the dehashed lines
    """
    # Grab the lines from the file
    with open(file_name, "r") as fp:
        lines = fp.readlines()

    # Dehash the lines
    lines = [dehash_line(x, log_dict) + "\n" for x in lines]

    return lines


def dehash_line(line, log_dict):
    """
    Dehash a line. Return with old formatting.

    :param line: The line to dehash
    :type line: str
    :param log_dict: dict of dicts created from .log_strings section from pebbleos.elf
    :type log_dict: dict of dicts

    :returns: Formatted line
              On error, the provided line
    """
    line_dict = dehash_line_unformatted(line, log_dict)
    if not line_dict:
        return line

    # Zephyr-like format: [timestamp] <lvl> task source: msg. Messages from a
    # log module already carry a "module: " prefix in formatted_msg; others get
    # their file:line as the source.
    output = []

    timestamp = " ".join(line_dict[key] for key in ("date", "time") if key in line_dict)
    if timestamp:
        output.append(f"[{timestamp}]")

    if "date" not in line_dict and line_dict.get("re_level"):
        output.append("<{}>".format(line_dict["re_level"]))

    # "-" means the task prefix is disabled (CONFIG_LOG_TASK_PREFIX)
    if line_dict.get("task", "-") != "-":
        output.append(line_dict["task"])

    if "module" not in line_dict and "file" in line_dict and "line" in line_dict:
        filename = os.path.basename(line_dict["file"])
        output.append("{}:{}:".format(filename, line_dict["line"]))

    output.append(line_dict["formatted_msg"])

    return " ".join(output)


def dehash_line_unformatted(line, log_dict):
    """
    Dehash a line. Return an unformatted dict of the info.

    :param line: The line to dehash
    :type line: str
    :param log_dict: dict of dicts created from .log_strings section from pebbleos.elf
    :type log_dict: dict of dicts

    :returns: A line_dict with keys 'formatted_msg', 'level', 'task', 'date', 'time', added.
              On error, 'formatted_output' = <input line>
    """
    line_dict = parse_line(line, log_dict)
    if not line_dict:
        return {"formatted_msg": line}

    return line_dict


def parse_line(line, log_dict):
    """
    Parse a log line

    :param line: The line to dehash
    :type line: str
    :param log_dict: dict of dicts created from .log_strings section from pebbleos.elf
    :type log_dict: dict of dicts

    :returns: A line_dict with keys 'formatted_msg', 'level', 'task', 'date', 'time',
              'core_number' added.
              On error, None
    """
    if not log_dict:
        return None

    # Handle BLE logs. They have no date, time, level in the input string
    ble_line = line.startswith(":0> NL:")
    match = None

    if not ble_line:
        match = NEWLOG_LINE_CONSOLE_PATTERN.search(line)
        if not match:
            match = NEWLOG_LINE_SUPPORT_PATTERN.search(line)
            if not match:
                return None

    # Search for the 'msg' in the entire log dictionary, getting back the sub-dictionary for this
    # specific message
    if ble_line:
        line_dict = parse_message(line, log_dict)
    else:
        line_dict = parse_message(match.group("msg"), log_dict)

    if line_dict:
        if ble_line:
            line_dict["task"] = "-"
        else:
            # Add all of the match groups (.e.g, date, time, level) to the line dict
            line_dict.update(match.groupdict())

        # Fixup 'level' which came from the msg string (re_level) with the ascii char
        if "level" in line_dict:
            line_dict["re_level"] = level_strings_map.get(int(line_dict["level"]), "?")

    return line_dict


def parse_message(msg, log_dict):
    """
    Parse the log message part of a line

    :param msg: The message to parse
    :type msg: str
    :param log_dict: dict of dicts created from .log_strings section from pebbleos.elf
    :type log_dict: dict of dicts

    :returns: the dict entry for the log line and the formatted message
    """
    match = NEWLOG_HASHED_INFO_PATTERN.search(msg)

    if not match:
        return None

    try:
        line_number = int(match.group("hash_key"), 16)
        output_dict = log_dict[str(line_number)].copy()  # Must be a copy!
    except KeyError:
        # Hash key not found. Wrong .elf?
        return None

    # Python's 'printf' doesn't support %p. Sigh. Convert to %x and hope for the best
    safe_output_msg = POINTER_FORMAT_TAG_PATTERN.sub(r"\g<format>x", output_dict["msg"])

    # Python's 'printf' doesn't handle (negative) 32-bit hex values correct. Build a new
    # arg list from the parsed arg list by searching for %<format>X conversions and masking
    # them to 32 bits.
    arg_list = []
    index = 0
    for arg in parse_args(match.group("arg_list")):
        index = safe_output_msg.find("%", index)
        if index == -1:
            # This is going to cause an error below...
            arg_list.append(arg)
        elif HEX_FORMAT_SPECIFIER_PATTERN.match(safe_output_msg, index):
            # We found a %<format>X
            arg_list.append(arg & 0xFFFFFFFF)
        else:
            arg_list.append(arg)

    # Use "printf" to generate the reconstructed string. Make sure the arguments are correct
    try:
        output_msg = safe_output_msg % tuple(arg_list)
    except (TypeError, ValueError, UnicodeDecodeError) as e:
        output_msg = msg + " ----> ERROR: " + str(e)

    # Prefix the message with the log module name, if any
    if "module" in output_dict:
        output_msg = "{}: {}".format(output_dict["module"], output_msg)

    # Add the formatted msg to the copy of our line dict
    output_dict["formatted_msg"] = output_msg

    # Add the core number to the line dict
    output_dict["core_number"] = str(
        (line_number >> PACKED_CORE_OFFSET) & PACKED_CORE_MASK
    )

    return output_dict


def parse_args(raw_args):
    """
    Split the argument list, taking care of `delimited strings`
    Idea taken from http://bit.ly/1KHzc0y

    :param raw_args: Raw argument list. Values are either in hex or in `strings`
    :type raw_args: str

    :returns: A list containing the arguments
    """
    args = []
    arg_run = []
    in_str = False

    if raw_args:
        for arg_ch in raw_args:
            if arg_ch not in "` ":
                arg_run.append(arg_ch)
                continue

            if in_str:
                if arg_ch == " ":
                    arg_run.append(" ")
                else:  # Must be ending `
                    args.append("".join(arg_run).strip())
                    in_str = False
                    arg_run = []
                continue

            # Start of a string
            if arg_ch == "`":
                in_str = True
                continue

            # Must be a space boundary (arg_ch == ' ')

            arg = "".join(arg_run).strip()
            if not len(arg):
                continue

            if not all(c in hex_digits for c in arg_run):
                # Hack to prevent hex conversion failure
                args.append(arg)
            else:
                # Every parameter is a 32-bit signed integer printed as a hex string with no
                # leading zeros. Add the zero padding if necessary, convert to 4 hex bytes,
                # and then reinterpret as a 32-bit signed big-endian integer.
                args.append(struct.unpack(">i", bytes.fromhex(arg.rjust(8, "0")))[0])

            arg_run = []

        # Clean up if anything is remaining (there is no trailing space)
        arg = "".join(arg_run).strip()
        if len(arg):
            # Handle the case where the trailing ` is missing.
            if not all(c in hex_digits for c in arg):
                args.append(arg)
            else:
                # Every parameter is a 32-bit signed integer printed as a hex string with no
                # leading zeros. Add the zero padding if necessary, convert to 4 hex bytes,
                # and then reinterpret as a 32-bit signed big-endian integer.
                args.append(struct.unpack(">i", bytes.fromhex(arg.rjust(8, "0")))[0])

    return args
