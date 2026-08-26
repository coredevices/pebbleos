#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import re
from datetime import datetime

import requests

# Hello. This script is kind of crappy.
#
# sample url:
#   http://bamboo.marlinspike.hq.getpebble.com/download/TT-PHAB-EV2/build_logs/TT-PHAB-EV2-4009.log
#
# regex assumes log has format like:
#   simple14-May-2015 20:13:53Starting task 'Checkout revision'
#   <...lots of stuff...>
#   simple14-May-2015 20:13:56Finished task 'Checkout revision'
# and
#   error14-May-2015 20:16:44Waf: Entering directory `/root/bamboo-agent-home/xml-data/build-dir/TT-PHAB-EV2/tintin/build'
#   <...stuff...>
#   error14-May-2015 20:16:44Waf: Leaving directory `/root/bamboo-agent-home/xml-data/build-dir/TT-PHAB-EV2/tintin/build'

parser = argparse.ArgumentParser(description="Check the timing of tasks in a build log")
parser.add_argument("url", metavar="URL", help="Bamboo url of raw build logs")
parser.add_argument(
    "--verbose", "-v", help="increase output verbosity", action="store_true"
)
args = parser.parse_args()

task_start = None
task_finish = None
waf_start = None
waf_finish = None
r = requests.get(args.url, stream=True)
task_regex = re.compile(
    r"simple\t((?:.*) (?:\d+:\d+:\d+))(?:.*)(Starting|Finished) task '([^']+)'"
)
waf_build_regex = re.compile(
    r"error\t((?:.*) (?:\d+:\d+:\d+))\tWaf: (Entering|Leaving) directory `([^']+)'"
)
for line in r.iter_lines():
    # filter out keep-alive new lines
    if line:
        # match for tasks
        match = task_regex.search(line)
        if match:
            dt = datetime.strptime(match.group(1), "%d-%B-%Y %H:%M:%S")
            if match.group(2) == "Starting":
                task_start = dt
                print(match.group(3))
            elif match.group(2) == "Finished":
                task_finish = dt
                print(" took " + str(task_finish - task_start))
        # match for waf building
        if args.verbose:
            match = waf_build_regex.search(line)
            if match:
                dt = datetime.strptime(match.group(1), "%d-%B-%Y %H:%M:%S")
                if match.group(2) == "Entering":
                    waf_start = dt
                elif match.group(2) == "Leaving":
                    waf_finish = dt
                    print(
                        "  "
                        + str(waf_finish - waf_start)
                        + " spent in "
                        + match.group(3)
                    )
