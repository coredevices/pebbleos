# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os
import re
import shutil
import subprocess
import sys

import sh
from packaging import version
from waflib import Logs

REQUIREMENTS = "requirements.txt"
REQUIREMENTS_BREW = "requirements-brew.txt"

# Packages only needed as executables (found via PATH, e.g. provided by nix),
# so a pip install is not required if the binary is available.
BINARY_PACKAGES = {"meson", "ninja"}

VERSION_REGEX = r"^(?P<package>.*)(?P<comparator>==|<=|>=|<|>)(?P<version>.*)"
VERSION_PATTERN = re.compile(VERSION_REGEX)


def tool_check():
    Logs.pprint("CYAN", "Checking %s" % REQUIREMENTS)

    with open(REQUIREMENTS) as file:
        req_list = text_to_req_list(file.read())

    pip_installed_text = sh.pip("freeze")
    pip_installed_dict = installed_list_to_dict(text_to_req_list(pip_installed_text))

    for req in req_list:
        check_requirement(req, pip_installed_dict)

    if sys.platform.startswith("darwin"):
        if not shutil.which("brew") and os.environ.get("IN_NIX_SHELL"):
            Logs.pprint("CYAN", "Skipping %s (in nix shell)" % REQUIREMENTS_BREW)
        elif not shutil.which("brew"):
            Logs.pprint("RED", "brew not found! Install Homebrew or use nix develop.")
        else:
            Logs.pprint("CYAN", "Checking %s" % REQUIREMENTS_BREW)

            with open(REQUIREMENTS_BREW) as file:
                brew_req_text = file.read()
                brew_req_list = text_to_req_list(brew_req_text)

            brew_installed_text = subprocess.check_output(["brew", "list"])
            brew_installed_dict = installed_list_to_dict(
                text_to_req_list(brew_installed_text.decode("utf8"))
            )

            for req in brew_req_list:
                check_requirement(req, brew_installed_dict)


def installed_list_to_dict(list):
    d = {}

    for l in list:
        d[l[0]] = l[2]

    return d


def text_to_req_list(req_list_text):
    req_list = []

    for raw_line in req_list_text.splitlines():
        # Skip editable (local) packages for now
        if raw_line.startswith("-e"):
            continue

        # Recursively handle -r includes
        if raw_line.startswith("-r "):
            with open(raw_line[3:]) as file:
                req_list.extend(text_to_req_list(file.read()))
            continue

        # Remove whitespace, comments, & --extra-index-url
        line = raw_line.replace(" ", "")
        if len(line) < 2 or line.startswith("#") or line.startswith("--"):
            continue

        match = VERSION_PATTERN.match(line)
        if not match:
            # Assume no version info
            req_list.append((line, None, None))
            continue
        if match.group("package").endswith(","):
            # Muliple requirements
            match2 = VERSION_PATTERN.match(match.group("package").strip(","))
            if not match2:
                Logs.pprint("RED", "Don't understand line '%s'" % raw_line)
                continue
            req_list.append(
                (
                    match2.group("package"),
                    match2.group("comparator"),
                    match2.group("version"),
                )
            )
            req_list.append(
                (
                    match2.group("package"),
                    match.group("comparator"),
                    match.group("version"),
                )
            )
        else:
            req_list.append(
                (
                    match.group("package"),
                    match.group("comparator"),
                    match.group("version"),
                )
            )

    return req_list


def check_requirement(req, installed):
    if req[0] not in installed:
        if req[0] in BINARY_PACKAGES and shutil.which(req[0]):
            return
        Logs.pprint("RED", "Package '%s' not installed" % req[0])
        return

    if not req[1]:
        # No version/comparison
        return

    ver = version.parse(installed[req[0]])
    success = True

    if req[1] == "==":
        success = ver == version.parse(req[2])
    elif req[1] == "<=":
        success = ver <= version.parse(req[2])
    elif req[1] == ">=":
        success = ver >= version.parse(req[2])
    elif req[1] == "<":
        success = ver < version.parse(req[2])
    elif req[1] == ">":
        success = ver > version.parse(req[2])
    else:
        Logs.pprint("RED", "Don't understand comparison '%s'" % req[1])

    if not success:
        Logs.pprint(
            "RED",
            "Package '%s' installed = %s, needed %s %s "
            % (req[0], ver, req[1], req[2]),
        )


# vim:filetype=python
