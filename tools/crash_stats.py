#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import csv
import logging
import os
import subprocess
import sys
from collections import OrderedDict

import requests
from triage import download_elf_by_sw_hw_version, download_path, load_user_settings

logger = logging.getLogger(__name__)


def crash_analytic_path(fw_version):
    return os.path.join(download_path(), fw_version + "_reboot_reasons")


def run_td_query_on_event_analytics(fw_version, error_code):
    query = f"""
    SELECT
       data_0_device_event_0_crash_report_0_link_register AS data_0_remote_device_0_system_crash_lr,
       device_0_remote_device_0_hw_version,
       COUNT(data_0_device_event_0_crash_report_0_crash_code) AS crash_count,
       device_0_remote_device_0_firmware_description_0_version_0_firmware_0_fw_version
    FROM
       remote_device_events
    WHERE
       device_0_remote_device_0_firmware_description_0_version_0_firmware_0_fw_version = '{fw_version}'
       AND data_0_device_event_0_event_enum = 10
       AND data_0_device_event_0_crash_report_0_crash_code = {error_code:d}
       AND TD_TIME_RANGE(time, TD_TIME_ADD(TD_SCHEDULED_TIME(), '-30d'), TD_SCHEDULED_TIME())
    GROUP BY
       data_0_device_event_0_crash_report_0_crash_code,
       data_0_device_event_0_crash_report_0_link_register,
       device_0_remote_device_0_hw_version,
       device_0_remote_device_0_firmware_description_0_version_0_firmware_0_fw_version
    ORDER BY
       COUNT(data_0_device_event_0_crash_report_0_link_register) DESC,
       device_0_remote_device_0_hw_version
    """

    logger.debug(query)
    logger.info("Running TD query!")

    path = crash_analytic_path(fw_version)
    if not os.path.exists(path):
        os.makedirs(path)

    output_csv_file = (
        crash_analytic_path(fw_version) + f"/0x{error_code:x}-crashcodes.csv"
    )

    cmd = f'td query -d pebble_restricted -P 2 -T presto -c -f csv -w -o {output_csv_file} "{query}"'
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    retval = p.wait()
    logger.info(f"Query Complete, Result = {retval:d}")
    return 0, output_csv_file


def run_td_query_on_hourly_analytics(fw_version, error_code):

    if (
        subprocess.call(
            "type td", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        != 0
    ):
        logger.error(
            "You need to install the td toolbelt for this query to work!\n"
            "Please see http://docs.treasuredata.com/articles/command-"
            "line#step-1-installation-amp-update"
        )
        sys.exit(0)

    query = f"""
    SELECT
       data_0_remote_device_0_system_crash_lr,
       device_0_remote_device_0_hw_version,
       COUNT(data_0_remote_device_0_system_crashed_code) AS crash_count,
       device_0_remote_device_0_firmware_description_0_version_0_firmware_0_fw_version
    FROM
       remote_device_system_metrics
    WHERE
       device_0_remote_device_0_firmware_description_0_version_0_firmware_0_fw_version = '{fw_version}'
       AND data_0_remote_device_0_system_crashed_code={error_code:d}
       AND TD_TIME_RANGE(time, TD_TIME_ADD(TD_SCHEDULED_TIME(), '-30d'), TD_SCHEDULED_TIME())
    GROUP BY
       data_0_remote_device_0_system_crashed_code,
       data_0_remote_device_0_system_crash_lr,
       device_0_remote_device_0_hw_version,
       device_0_remote_device_0_firmware_description_0_version_0_firmware_0_fw_version
    ORDER BY
       COUNT(data_0_remote_device_0_system_crash_lr) DESC,
       device_0_remote_device_0_hw_version DESC
    """

    logger.debug(query)
    logger.info("Running TD query!")

    path = crash_analytic_path(fw_version)
    if not os.path.exists(path):
        os.makedirs(path)

    output_csv_file = (
        crash_analytic_path(fw_version) + f"/0x{error_code:x}-crashcodes.csv"
    )

    cmd = f'td query -d pebble_restricted -P 2 -T presto -c -f csv -w -o {output_csv_file} "{query}"'
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    retval = p.wait()
    logger.info(f"Query Complete, Result = {retval:d}")
    return 0, output_csv_file


def gather_analytic_crash_stats(fw_version, error_code, error_code_name, use_events):

    elf_dict = {}

    symbol_name_to_hw_model_lookup = {}
    # Download the .elf associated with the core dump:
    for elf_name in symbol_name_to_hw_model_lookup.iterkeys():
        try:
            elf_dict[elf_name] = download_elf_by_sw_hw_version(fw_version, elf_name)
        except requests.exceptions.HTTPError as http_error:
            logger.debug(f"Could not find ELF file: {fw_version} ({http_error})")

    if use_events:
        retval, res_file = run_td_query_on_event_analytics(fw_version, error_code)
    else:
        retval, res_file = run_td_query_on_hourly_analytics(fw_version, error_code)

    if retval != 0:
        return ""

    with open(res_file) as res_fh:
        reader = csv.reader(res_fh)
        next(reader, None)
        rows = list(reader)

    line_dict = {}
    for line in rows:
        if len(line) < 3:
            continue

        # it looks like the LR need to be in hex for arm-none-eabi-addr2line to work
        try:
            lr = hex(int(line[0]))
        except ValueError:
            continue  # Analytic is empty for some reason

        hw_rev = line[1]
        found = False
        for elf_name, hw_model_list in symbol_name_to_hw_model_lookup.iteritems():
            for hw_model in hw_model_list:
                if hw_rev.lower() == hw_model.lower():
                    try:
                        fw_symbols_name = elf_dict[elf_name]
                        found = True
                    except KeyError:
                        logger.debug(f"No dict for elf {elf_name}")
                    break

        if not found:
            print(f"Unhandled HW Version {hw_rev}")
            continue

        cmd = f"arm-none-eabi-addr2line --exe={fw_symbols_name} {lr!s}"
        p = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
        )
        result = p.stdout.readlines()
        retval = p.wait()

        if retval == 0 and len(result) >= 1:
            line_info = result[0].strip("\n")
            logger.debug(f"{lr} {line_info}")
            # Try to pretty print the path but if its not something in our build directory
            # just print the whole path
            idx = line_info.find("build/..")
            if idx != -1:
                line_info = line_info[idx:]
            if line_info in line_dict:
                line_dict[line_info] += int(line[2])
            else:
                line_dict[line_info] = int(line[2])

    line_dict_sorted = OrderedDict(
        sorted(line_dict.items(), key=lambda x: x[1], reverse=True)
    )

    result_text = f"Results for {error_code_name}:\n"
    asserts_analyzed = 0
    for k, v in line_dict_sorted.items():
        asserts_analyzed += v
        result_text += f"{v:6d}: {k}\n"

    result_text += f"{asserts_analyzed:d} {error_code_name} analyzed\n"
    return result_text


def analyze_analytics(fw_version, use_events):
    if use_events:
        ASSERT_CRASH_CODE = 0x11
        HARDFAULT_CRASH_CODE = 0x13
    else:
        ASSERT_CRASH_CODE = 0xDEAD0011
        HARDFAULT_CRASH_CODE = 0xDEAD0013

    result = "=======\n"
    result = gather_analytic_crash_stats(
        fw_version, ASSERT_CRASH_CODE, "Asserts", use_events
    )
    result += "\n"
    result += gather_analytic_crash_stats(
        fw_version, HARDFAULT_CRASH_CODE, "Hard Faults", use_events
    )

    print(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "fw_tag",
        type=str,
        help="Analyzes "
        "crashes seen in the field by line for the specified "
        "build, for example v2.9-beta6",
    )
    parser.add_argument("--debug", action="store_true", help="Turn on debug logging")
    parser.add_argument(
        "--use_event",
        action="store_true",
        help="Look at crash information "
        "using event analytics instead of the default hourly analytics",
    )

    args = parser.parse_args()

    load_user_settings()

    level = logging.INFO
    if args.debug:
        level = logging.DEBUG
    logging.basicConfig(level=level)

    logger.info(f"Analyzing crash stats for {args.fw_tag}")
    if args.fw_tag:
        analyze_analytics(args.fw_tag, args.use_event)
