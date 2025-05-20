# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# encoding=utf8
# PBL-31508: This is pretty bad and we want to re-do a lot of this.

import os
import sys
import threading
import time
from datetime import datetime
import unicodedata as ud

from pebble.loghashing import newlogging

sys.path.insert(0, os.path.dirname(__file__))
from newlogging import get_log_dict_from_file

LOG_DICT_KEY_CORE_ID = 'core_'

COLOR_DICT = {
               "BLACK":         "\x1b[30m",    "0":  "\x1b[30m",        # Not the most useful
               "RED":           "\x1b[31m",    "1":  "\x1b[31m",
               "GREEN":         "\x1b[32m",    "2":  "\x1b[32m",
               "YELLOW":        "\x1b[33m",    "3":  "\x1b[33m",
               "BLUE":          "\x1b[34m",    "4":  "\x1b[34m",
               "MAGENTA":       "\x1b[35m",    "5":  "\x1b[35m",
               "CYAN":          "\x1b[36m",    "6":  "\x1b[36m",
               "GREY":          "\x1b[37m",    "7":  "\x1b[37m",
               "LIGHT_GREY":    "\x1b[1m;30m", "8":  "\x1b[1m;30m",
               "LIGHT_RED":     "\x1b[1m;31m", "9":  "\x1b[1m;31m",
               "LIGHT_GREEN":   "\x1b[1m;32m", "10": "\x1b[1m;32m",
               "LIGHT_YELLOW":  "\x1b[1m;33m", "11": "\x1b[1m;33m",
               "LIGHT_BLUE":    "\x1b[1m;34m", "12": "\x1b[1m;34m",
               "LIGHT_MAGENTA": "\x1b[1m;35m", "13": "\x1b[1m;35m",
               "LIGHT_CYAN":    "\x1b[1m;36m", "14": "\x1b[1m;36m",
               "WHITE":         "\x1b[1m;37m", "15": "\x1b[1m;37m"}
COLOR_BOLD_RESET = "\x1b[0m"
BOLD = "\x1b[1m"
# Control code to clear the current line
CLEAR_LINE = "\x1b[2K"


class LogDehash(object):
    """ Dehashing helper with a file update watch thread
    """
    def __init__(self, dict_path, justify="small", color=False, bold=-1, print_core=False,
                 monitor_dict_file=True):
        self.path = dict_path
        self.dict_mtime = None

        self.arg_justify = justify
        self.arg_color = color
        self.arg_bold = bold

        self.justify_size = 0

        self.load_log_strings()
        self.print_core = print_core
        if self.print_core:
            self.print_core_header()

        self.running = False
        if monitor_dict_file:
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.setDaemon(True)
            self.thread.start()

    def run(self):
        while self.running:
            if os.path.lexists(self.path) and (not self.dict_mtime or
                                               os.path.getmtime(self.path) > self.dict_mtime):
                # We don't need to worry about thread safety here because the dict is getting
                # replaced entirely. If anyone has a reference to the old one, it will stay
                # alive until they're done.
                self.load_log_strings()
            time.sleep(5)  # It takes at least this long to flash an update to the board

    def load_log_strings(self):
        if os.path.lexists(self.path):
            self.dict_mtime = os.path.getmtime(self.path)
            self.loghash_dict = get_log_dict_from_file(self.path)
        else:
            self.dict_mtime = None
            self.loghash_dict = None
        self.update_log_string_metrics()

    def load_log_strings_from_dict(self, dict):
        self.running = False
        self.dict_mtime = None
        self.loghash_dict = dict
        self.update_log_string_metrics()

    def print_core_header(self):
        if not self.loghash_dict:
            return

        print('Supported Cores:')
        for key in sorted(self.loghash_dict, key=self.loghash_dict.get):
            if key.startswith(LOG_DICT_KEY_CORE_ID):
                print(('    {}: {}'.format(key, self.loghash_dict[key])))

    def update_log_string_metrics(self):
        if not self.loghash_dict:
            self.arg_justify = 0
            return

        # Handle justification
        max_basename = 0
        max_linenum = 0
        for line_dict in self.loghash_dict.values():
            if 'file' in line_dict and 'line' in line_dict:
                max_basename = max(max_basename, len(os.path.basename(line_dict['file'])))
                max_linenum = max(max_basename, len(os.path.basename(line_dict['line'])))
        justify_width = max_basename + 1 + max_linenum  # Include the ':'

        if self.arg_justify == 'small':
            self.justify_size = 0
        elif self.arg_justify == 'right':
            self.justify_size = justify_width * -1
        elif self.arg_justify == 'left':
            self.justify_size = justify_width
        else:
            self.justify_size = int(self.arg_justify)

    def dehash(self, msg):
        """ Dehashes a logging message.
        """
        string = str(msg)
        if "NL:" in string:  # Newlogging
            safe_line = ud.normalize('NFKD', string)
            line_dict = newlogging.dehash_line_unformatted(safe_line, self.loghash_dict)
            return line_dict
        else:
            return {"formatted_msg": string, "unhashed": True}

    def basic_format_line(self, line_dict):
        output = []

        if 'support' not in line_dict and 're_level' in line_dict:
            output.append(line_dict['re_level'])
        if self.print_core and 'core_number' in line_dict:
            output.append(line_dict['core_number'])
        if 'task' in line_dict:
            output.append(line_dict['task'])
        if 'date' in line_dict:
            output.append(line_dict['date'])
        if 'time' in line_dict:
            output.append(line_dict['time'])
        elif 'support' not in line_dict:
            # Use the current time if one isn't provided by the system
            now = datetime.now()
            output.append('%02d:%02d:%02d.%03d' % (now.hour, now.minute, now.second,
                                                   now.microsecond/1000))

        pre_padding = ''
        post_padding = ''

        if 'file' in line_dict and 'line' in line_dict:
            filename = os.path.basename(line_dict['file'])
            file_line = '{}:{}>'.format(filename, line_dict['line'])
            if self.justify_size < 0:
                output.append(file_line.rjust(abs(self.justify_size)))
            else:
                output.append(file_line.ljust(abs(self.justify_size)))

        output.append(line_dict['formatted_msg'])
        try:
            return ' '.join(output)
        except UnicodeDecodeError:
            return ''

    def minicom_format_line(self, line_dict):
        """This routine reformats a line already printed to the console if it was
        hashed. It does this by clearing the line which was already printed
        using a control escape command and relies on lines ending in "\r\n".
        If the line was not hashed in the first place, it simply returns a
        newline character to print

        """
        if 'unhashed' in line_dict:
            return '\n'

        output = []
        if self.arg_color and 'color' in line_dict:
            color = line_dict['color']
            if color in COLOR_DICT:
                output.append(COLOR_DICT[color])
        if 'level' in line_dict:
            if int(line_dict['level']) <= self.arg_bold:
                output.append(BOLD)
        output.append(CLEAR_LINE)
        output.append(self.basic_format_line(line_dict))
        output.append('\n')
        output.append(COLOR_BOLD_RESET)

        return ''.join(output)

    def commander_format_line(self, line_dict):
        output = []
        if self.arg_color and 'color' in line_dict:
            color = line_dict['color']
            if color in COLOR_DICT:
                output.append(COLOR_DICT[color])
        if 'level' in line_dict:
            if int(line_dict['level']) <= self.arg_bold:
                output.append(BOLD)
        output.append(self.basic_format_line(line_dict))
        output.append(COLOR_BOLD_RESET)

        return ''.join(output)
