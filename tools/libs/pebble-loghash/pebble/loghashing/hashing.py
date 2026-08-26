# SPDX-FileCopyrightText: 2025 Google LLC
# SPDX-License-Identifier: Apache-2.0

# /usr/bin/env python
"""
Module for hashing log strings
"""

import json
import os
import re

from pebble.loghashing.constants import (
    FORMAT_IDENTIFIER_STRING_FMT,
    FORMAT_SPECIFIER_PATTERN,
    HASH_MASK,
    HASH_NEXT_LINE,
    LINES_TO_HASH,
    LOOKUP_DEFAULT_STRING,
    LOOKUP_RESULT_STRING_FMT,
    STR_LITERAL_PATTERN,
)


def hash_directory(path, output_file_name):
    """
    Runs the line hasher on every file in a directory tree

    :param path: Root of the tree to hash
    :type path: str
    """
    lookup_dict = {}

    for walk in os.walk(path, followlinks=True):
        # First and third item, respectively
        root, file_names = walk[0::2]

        for file_name in file_names:
            lookup_dict.update(hash_file(f"{root}/{file_name}"))

    # Read in hash_lookup
    # Write lines out
    with open(output_file_name, "w") as fp:
        json.dump(lookup_dict, fp)


def hash_file(file_name):
    """
    Attempt to hash each line of a file

    :param file_name: Name of file to hash
    :type file_name: str

    :returns: A hash lookup dictionary
    """
    # Read in lines
    with open(file_name, "r") as fp:
        lines = fp.readlines()

    hashed_lines = []
    lookup_dict = {}

    force_hash = False

    # Hash appropriate lines with line number, and file name
    for index, line in enumerate(lines):
        hashed_line, line_dict = hash_line(line, file_name, index + 1, force_hash)

        force_hash = False
        if HASH_NEXT_LINE in hashed_line:
            force_hash = True

        hashed_lines.append(hashed_line)
        lookup_dict.update(line_dict)

    # Write lines out
    with open(file_name, "w") as fp:
        fp.writelines(hashed_lines)

    return lookup_dict


def hash_line(line, file_name, line_num, force_hash=False):
    """
    Search line for hashable strings, and hash them.

    :param line: Line to search
    :type line: str
    :param file_name: Name of the file that the line is in
    :type file_name: str
    :param line_num: Line number of the line
    :type line_num: int

    :returns: A tuple with: The input line (with all hashable strings hashed),
                and a hash lookup dictionary
    """
    hash_dict = {}

    # Only match lines that contain one of the following substrings
    if force_hash or any(x in line for x in LINES_TO_HASH):
        if force_hash or not any(x in line for x in ["PBL_CROAK_OOM"]):
            match = STR_LITERAL_PATTERN.search(line)

            if match:
                # Strip all double quotes from the string
                str_literal = re.sub('"', "", match.group(2))

                str_literal = inttype_conversion(str_literal)

                # Hash the file name and line number in as well
                line_to_hash = f"{os.path.basename(file_name)}:{line_num}:{str_literal}"

                hashed_msg = hash_string(line_to_hash)

                hash_dict[hashed_msg] = line_to_hash

                line = f"{match.group(1)}{hashed_msg}{match.group(3)}\n"

    return (line, hash_dict)


def hash_string(string):
    """
    Hash and return a given string.

    :param string: String to hash
    :type string: str

    :returns: The input string, hashed
    """
    return hex(hash(string) & HASH_MASK)


def inttype_conversion(inttype):
    """
    Change PRI specifiers into classical printf format specifiers

    :param inttype: PRI specifier to convert
    :type inttype: str

    :returns: The classical printf format specifier that inttype represents
    """
    # Change ' PRIu32 ' to '32u'
    output = re.sub(r"\s*PRI([diouxX])(8|16|32|64|PTR)\s*", r"\g<2>\g<1>", inttype)
    # No length modifier for 8 or 16 modifier
    output = re.sub("(8|16)", "", output)
    # 'l' modifier for 32 or PTR modifier
    output = re.sub("(32|PTR)", "l", output)
    # 'll' modifier for 64 modifier
    output = re.sub("64", "ll", output)
    # Change BT_MAC_FMT and BT_ADDR_FMT
    output = re.sub("BT_MAC_FMT", "%02X:%02X:%02X:%02X:%02X:%02X", output)
    output = re.sub("BT_ADDR_FMT", "%02X:%02X:%02X:%02X:%02X:%02X", output)
    output = re.sub("BT_DEVICE_ADDRESS_FMT", "%02X:%02X:%02X:%02X:%02X:%02X", output)

    return output


def string_formats(string):
    """
    Parses a string for all format identifiers

    :param string: String to parse
    :type string: str

    :returns: A list of all format specifiers
    """
    return FORMAT_SPECIFIER_PATTERN.findall(string)


def create_lookup_function(lookup_dict, output_file_name):
    """
    Create a C source file for hash to format specifiers lookup

    :param lookup_dict: Hash to string lookup dictionary
    :type lookup_dict:  dict
    """
    strings = []
    lines = [LOOKUP_DEFAULT_STRING]
    format_lookup = {}

    index = 1

    format_map = [[x, string_formats(lookup_dict[x])] for x in lookup_dict.keys()]

    for line, formats in format_map:
        # Only make an entry if there's a format string!
        if formats:
            format_as_string = "".join(formats)

            if format_as_string not in format_lookup:
                format_lookup[format_as_string] = index

                strings.append(
                    FORMAT_IDENTIFIER_STRING_FMT.format(index, format_as_string)
                )

                index = index + 1

            lines.append(
                LOOKUP_RESULT_STRING_FMT.format(line, format_lookup[format_as_string])
            )

    with open(output_file_name, "w") as fp:
        fp.writelines(strings)
        fp.writelines(lines)
