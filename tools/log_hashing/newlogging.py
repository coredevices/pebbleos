# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import json
import re

from elftools.elf.elffile import ELFFile
from elftools.elf.segments import NoteSegment

LOG_STRINGS_SECTION_NAME = ".log_strings"
BUILD_ID_SECTION_NAME = ".note.gnu.build-id"
BUILD_ID_NOTE_OWNER_NAME = "GNU"
BUILD_ID_NOTE_TYPE_NAME = "NT_GNU_BUILD_ID"

NEW_LOGGING_VERSION = "NL0102"
NEW_LOGGING_HEADER_OFFSET = 0

# File -> module map entries emitted by PBL_LOG_MODULE_DEFINE/PBL_LOG_MODULE_DECLARE
MODULE_LINE_PREFIX = "MODULE:"

LOG_LINE_SPLIT_REGEX = r"([^\0]+)\0"
LOG_LINE_SPLIT_PATTERN = re.compile(
    r"([^\0]+)\0"
)  # <anything but '\0'>  followed by '\0'

LOG_LINE_KEY_ALL_REGEX = "(?P{}.*)"  # matches anything (with match group name {})
LOG_LINE_KEY_NO_EMBEDDED_COLON_REGEX = (
    "(?P{}[^:]*?)"  # matches anything not containing a colon
)

LOG_DICT_KEY_BUILD_ID_LEGACY = "build_id"
LOG_DICT_KEY_BUILD_ID = "build_id_core_"
LOG_DICT_KEY_VERSION = "new_logging_version"
LOG_DICT_KEY_CORE_ID = "core_"

LOG_CORE_ID_OFFSET_SHIFT = 30


class LogDict:
    def __init__(self):
        self.log_dict = {}
        key_list = []
        core_id = 0
        core_id_offset = 0
        core_name = ""
        log_line_regex = None

    def parse_header_from_section(self, section):
        # Split the header string from the log strings, which follow
        header_string = section[: section.find("\0")]
        # Split the header string into parts, comma delimited
        for line in header_string.split(","):
            tag, key = line.split("=")
            if tag.startswith("NL"):
                self.log_dict[LOG_DICT_KEY_VERSION] = tag
                self.key_list = key.split(":")
            elif tag.startswith("CORE_ID"):
                self.core_id = key
            elif tag.startswith("CORE_NAME"):
                self.core_name = key
            else:
                raise Exception(f"Unknown header tag '{line}'")

        if self.log_dict[LOG_DICT_KEY_VERSION] != NEW_LOGGING_VERSION:
            version = self.log_dict[LOG_DICT_KEY_VERSION]
            # Worthy of an exception! Something bad has happened with the tools configuration.
            raise Exception(
                f"Expected log strings version {NEW_LOGGING_VERSION} not {version}"
            )

    def log_line_regex_from_key_list(self):
        regex = []
        for key in self.key_list:
            if key == "<msg>":
                regex.append(
                    LOG_LINE_KEY_ALL_REGEX.format(key)
                )  # Allow embedded colons
            else:
                regex.append(
                    LOG_LINE_KEY_NO_EMBEDDED_COLON_REGEX.format(key)
                )  # No embedded colons

        self.log_line_regex = re.compile(":".join(regex))

    def set_section_and_build_id(self, section, build_id):
        # Parse the header
        self.parse_header_from_section(section)

        # Now split the log line into a key dict using the generated regex
        self.log_line_regex_from_key_list()

        # Set the build id. If core 0, use the legacy build ID.
        # Otherwise, use the new style build ID -- append the core ID.
        if self.core_id == "0":
            self.log_dict[LOG_DICT_KEY_BUILD_ID_LEGACY] = build_id
        else:
            self.log_dict[LOG_DICT_KEY_BUILD_ID + self.core_id] = build_id

        # Set the core ID
        self.log_dict[LOG_DICT_KEY_CORE_ID + self.core_id] = self.core_name

        # Calculate the core/id offset
        self.core_id_offset = int(self.core_id) << LOG_CORE_ID_OFFSET_SHIFT

        # Create a log dictionary of line dictionaries.
        # { 'offset', { '<file>': <filename>, '<line>': <line>, '<level>': <level>, '<msg>': <msg>,
        #             'color': <color> }}
        # Stupidly, offset is now a string because JSON sucks
        # 'offset' is a decimal string.

        # First, split the section by '\0' characters, keeping track of the start.
        # There may be padding, so ignore any empty strings.
        modules = {}
        entries = {}
        for line in LOG_LINE_SPLIT_PATTERN.finditer(section):
            # Skip the header line
            if line.start() == NEW_LOGGING_HEADER_OFFSET:
                continue

            # Collect file -> module map entries
            if line.group(1).startswith(MODULE_LINE_PREFIX):
                file_name, _, module = line.group(1)[
                    len(MODULE_LINE_PREFIX) :
                ].rpartition(":")
                modules[file_name] = module
                continue

            tags = self.log_line_regex.match(line.group(1))

            # Add the core offset to the 'offset' parameter
            entries[str(line.start() + self.core_id_offset)] = tags.groupdict()

        # Attach the module name to every message from a mapped file
        for tags in entries.values():
            module = modules.get(tags.get("file"))
            if module:
                tags["module"] = module

        self.log_dict.update(entries)

    def get_log_dict(self):
        return self.log_dict


def get_elf_section(filename, section_name):
    with open(filename, "rb") as file:
        section = ELFFile(file).get_section_by_name(section_name)
        return section.data().decode("utf-8") if section is not None else None


def get_elf_build_id(filename):
    with open(filename, "rb") as file:
        for segment in ELFFile(file).iter_segments():
            if isinstance(segment, NoteSegment):
                for note in segment.iter_notes():
                    if (
                        note["n_name"] == BUILD_ID_NOTE_OWNER_NAME
                        and note["n_type"] == BUILD_ID_NOTE_TYPE_NAME
                    ):
                        return note["n_desc"]
    return ""


""" Merge two loghash dictionaries and return the union.
    The first dictionary may be empty """


def merge_dicts(dict1, dict2):
    # Check to make sure that the versions exist and are identical
    if len(dict1) and LOG_DICT_KEY_VERSION not in dict1:
        raise Exception("First log_dict does not contain version info")

        if LOG_DICT_KEY_VERSION not in dict2:
            raise Exception("Second log_dict does not contain version info")

        if dict1[LOG_DICT_KEY_VERSION] != dict2[LOG_DICT_KEY_VERSION]:
            raise Exception(
                f"log dicts have different versions! {dict1[LOG_DICT_KEY_VERSION]} != {dict2[LOG_DICT_KEY_VERSION]}"
            )

    # Check to make sure that both have core IDs and that they're different.
    core_list_dict1 = []
    for key in dict1:
        if key.startswith(LOG_DICT_KEY_CORE_ID):
            core_list_dict1.append(key)
    core_list_dict2 = []
    for key in dict2:
        if key.startswith(LOG_DICT_KEY_CORE_ID):
            core_list_dict2.append(key)

    if len(dict1) and len(core_list_dict1) == 0:
        raise Exception("First log_dict does not specify a core_id")
    if len(core_list_dict2) == 0:
        raise Exception("Second log_dict does not specify a core_id")

    intersection = set(core_list_dict1).intersection(core_list_dict2)
    if len(intersection) != 0:
        raise Exception(
            f"Both log_dicts specify the following cores: {list(intersection)}"
        )

    # Merge the dictionaries. Don't bother confirming there are no log message conflicts.
    merged_dict = dict1.copy()
    merged_dict.update(dict2)

    return merged_dict


""" ----------------------------- External API -------------------------------- """


""" Returns log dict (including build_id and new_logging_version keys) from specified
    filename. Accepts either pebbleos.elf or loghash_dict.json """


def get_log_dict_from_file(filename):

    if filename.endswith(".json"):
        with open(filename, "rb") as file:
            return json.load(file)

    log_strings_section = get_elf_section(filename, LOG_STRINGS_SECTION_NAME)
    build_id = get_elf_build_id(filename)

    ld = LogDict()
    ld.set_section_and_build_id(log_strings_section, build_id)

    return ld.get_log_dict()


""" Merge the loghash_dict.json files named in 'merge_list' to node 'out_file' """


def merge_loghash_dict_json_files(out_file, merge_list):
    out_dict = {}

    for json_file in merge_list:
        with open(json_file.abspath(), "r") as json_file:
            in_dict = json.load(json_file)
            out_dict = merge_dicts(out_dict, in_dict)

    if LOG_DICT_KEY_BUILD_ID_LEGACY not in out_dict:
        raise Exception("build_id missing from loghash_dict.json")

    with open(out_file.abspath(), "w") as json_file:
        json.dump(out_dict, json_file, indent=2, sort_keys=True)
