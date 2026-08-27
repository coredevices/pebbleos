#! /usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import json
import logging
import os
import sys
import zipfile

import logdehash
import newlogging
import requests

logger = logging.getLogger(__name__)

DICT_FIRMWARE = "build/src/fw/loghash_dict.json"
DICT_PRF = "build/prf/src/fw/loghash_dict.json"

BUILD_ID_STR = "BUILD ID: "
HASH_STR_LEN = 40


SETTINGS = {
    # Hagen Daas stuff:
    "files": "https://files.pebblecorp.com/dict/",
    # Go to https://auth.pebblecorp.com/show to get this value:
    "hd_session": None,
}


class AuthException(Exception):
    pass


def load_user_settings():
    settings_path = "~/.triage"
    try:
        with open(os.path.expanduser(settings_path), "rb") as settings_file:
            user_settings = json.load(settings_file)
    except OSError as e:
        if e.errno == 2:
            logger.error(
                """Please create %s with credentials: """
                """'{ "user": "$USER", "password": "$PASSWORD" }'""",
                settings_path,
            )
        return
    SETTINGS.update(user_settings)

    if not SETTINGS["hd_session"]:
        msg = (
            "Missing 'hd_session' token in user settings.\n"
            "1. Get the cookie from https://auth.pebblecorp.com/show\n"
            f"2. Add as value with key 'hd_session' to {settings_path}"
        )
        logger.error(msg)
        sys.exit(-1)


def get_loghash_dict_from_hagen_daas_files(hash):
    load_user_settings()

    url = SETTINGS["files"] + hash
    r = requests.get(url, headers={"Cookie": "hd_session={}".format(SETTINGS["hd_session"])})
    if r.status_code > 400:
        r.raise_for_status()
    if "accounts.google.com" in r.url:
        raise AuthException(
            "Not authenticated, see instructions at the top of {}".format("https://pebbletechnology.atlassian.net/wiki/"
            "display/DEV/Quickly+triaging+JIRA+FW+issues+with+pbldebug")
        )
    return r.text


class Log:
    def __init__(self, output=False):
        self.output = output

    def setOutput(self, output):
        self.output = output

    def debug(self, format, *args):
        if self.output:
            sys.stderr.write(format % args)
            sys.stderr.write("\r\n")


def get_dict_from_pbz(filename):
    if zipfile.is_zipfile(filename):
        with zipfile.ZipFile(filename) as dict_zip:
            return dict_zip.read("loghash_dict.json")
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Dehash a log",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Description:
    dehash.py is a script that tries to dehash whatever log is provided, however
    it is provided. 'Files' on Hagen-Daas will be consulted if a loghash
    dictionary isn't specified.

Input File(s):
    Can be the log to dehash and/or log hash dictionaries to decode the log.

    dehash.py assumes that the hashed log is passed via stdin.
    If specified in the file list, the hashed log must not have the extension
    .elf, .pbz, or .json.

    loghash dictionaries can be .json files, .elf files, or bundles (.pbz).
    Only one dictionary per core may be specified.

Examples:
    dehash.py pbl-123456.log pebbleos.elf > log.txt
    dehash.py normal_silk_v4.0-alpha11-20-g6661346.pbz < pbl-12345.log > log.txt
    gzcat crash_log.gz | dehash.py
    dehash.py --prf log_from_watch.log
""",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--fw",
        action="store_true",
        help="Use the fw loghash_dict from your build. Default.",
    )
    group.add_argument(
        "--prf", action="store_true", help="Use the prf loghash_dict from your build."
    )
    parser.add_argument("-v", action="store_true", help="Verbose debug to stderr")
    parser.add_argument(
        "file", nargs="*", help="Input file(s). See below for more info."
    )
    args = parser.parse_args()

    logger = Log(args.v)

    # Make a copy of the file list
    filelist = list(args.file)
    # Add the PRF dict to filelist, if appropriate
    if args.prf:
        filelist.append(DICT_PRF)

    loghash_dict = {}
    log = None

    # Examine the file list
    for f in filelist:
        if f.endswith((".json", ".elf")):
            logger.debug("Loading dictionary from %s", f)
            d = newlogging.get_log_dict_from_file(f)
            loghash_dict = newlogging.merge_dicts(loghash_dict, d)
        elif f.endswith(".pbz"):
            logger.debug("Loading dictionary from %s", f)
            d = get_dict_from_pbz(f)
            if not d:
                raise RuntimeError(f"Unable to load loghash_dict.json from {f}")
            loghash_dict = newlogging.merge_dicts(loghash_dict, json.loads(d))
        else:
            logger.debug("Log file %s", f)
            if log:
                raise RuntimeError("More than one log file specified")
            log = f

    # Now consider the --fw option. Don't fail unless it was explicitly specified
    if args.fw or (not args.prf and not loghash_dict):
        logger.debug("Loading dictionary from %s", DICT_FIRMWARE)
        if os.path.isfile(DICT_FIRMWARE) or args.fw:
            d = newlogging.get_log_dict_from_file(DICT_FIRMWARE)
            loghash_dict = newlogging.merge_dicts(loghash_dict, d)
        else:
            logger.debug("Ignoring default fw dict -- %s not found", DICT_FIRMWARE)

    # Create the dehasher
    dehash = logdehash.LogDehash("", monitor_dict_file=False)
    dehash.load_log_strings_from_dict(loghash_dict)

    # Input file or stdin?
    infile = open(log) if log else sys.stdin  # noqa: SIM115

    # Dehash the log
    for line in infile:
        line_dict = dehash.dehash(line)
        if "unhashed" in line_dict:
            dhl = line_dict["formatted_msg"]
        else:
            dhl = dehash.basic_format_line(line_dict)
        sys.stdout.write(dhl.strip())
        sys.stdout.write("\r\n")
        sys.stdout.flush()

        # If we have a dictionary, continue
        if loghash_dict:
            continue

        # No dictionary -- see if we can load one
        index = dhl.upper().rfind(BUILD_ID_STR)
        if index == -1:
            continue

        build_id = dhl[
            index + len(BUILD_ID_STR) : (index + len(BUILD_ID_STR) + HASH_STR_LEN)
        ]

        try:
            logger.debug("Loading dictionary from Hagen-Daas for ID %s", build_id)
            d = get_loghash_dict_from_hagen_daas_files(build_id)
        except (
            requests.exceptions.ConnectionError,
            requests.exceptions.HTTPError,
            AuthException,
        ) as error:
            sys.stderr.write(
                f"Could not get build id {build_id} from files. {error}\r\n"
            )
            continue

        if d:
            loghash_dict = json.loads(d)
            dehash.load_log_strings_from_dict(loghash_dict)
        else:
            sys.stderr.write(f"Could not get build id {build_id} from files.\r\n")

    if infile is not sys.stdin:
        infile.close()


if __name__ == "__main__":
    main()
