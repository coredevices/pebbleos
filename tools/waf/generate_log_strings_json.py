# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import json

from waflib import Logs

from tools.log_hashing.check_elf_log_strings import check_dict_log_strings
from tools.log_hashing.newlogging import get_log_dict_from_file


def wafrule(task):
    elf_filename = task.inputs[0].abspath()
    log_strings_json_filename = task.outputs[0].abspath()

    return generate_log_strings_json(elf_filename, log_strings_json_filename)


def generate_log_strings_json(elf_filename, log_strings_json_filename):
    log_dict = get_log_dict_from_file(elf_filename)
    if not log_dict:
        error = f"Unable to get log strings from {elf_filename}"
        Logs.pprint("RED", error)
        return error

    # Confirm that the log strings satisfy the rules
    output = check_dict_log_strings(log_dict)
    if output:
        Logs.pprint("RED", output)
        return "NewLogging string formatting error"

    # Create log_strings.json
    with open(log_strings_json_filename, "w") as json_file:
        json.dump(log_dict, json_file, indent=2, sort_keys=True)
