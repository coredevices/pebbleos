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

# Stuff we're patching or calling into directly
from serial.tools.miniterm import main
from serial import Serial
from serial.urlhandler.protocol_socket import Serial as SocketSerial
from serial.serialutil import SerialException

# Stuff we need
import os
import sys
import threading
import operator
import time
import unicodedata as ud
import socket

from logdehash import LogDehash

line_buffer = []
def dehash_read(self, size, plain_read):
    global line_buffer
    # Most of the time, we pass through the results of their read command (if rather reconstituted)
    # At the same time, keep track of the contents of the last line
    # Once at the end of a line, check if it contains a LH: loghash header
    #  - If not, continue as usual
    #  - If it does, dehash the buffered line and use clever tricks to swap it into the terminal
    #    view in-place (obviously, don't be using this method for raw serial IO or file output)
    raw_read_data = plain_read(self, size)
    read_data = []
    for read_char in raw_read_data:
        if read_char == "\n":
            read_line = "".join(line_buffer)
            line_buffer = []
            line_dict = dehasher.dehash(read_line)
            read_data.append(dehasher.minicom_format_line(line_dict))
        else:
            line_buffer.append(read_char)
            read_data.append(read_char)
    return "".join(read_data)

def socket_serial_read(self, size=1):
    """
    Read size bytes from the serial port. If a timeout is set it may
    return less characters as requested. With no timeout it will block
    until the requested number of bytes is read.
    This is a replacement for protocol_socket.SocketSerial.read() that is smarter about
    handling a closed socket from the remote end. Instead of just immediately returning an
    empty string, it attempts to reopen the socket when it detects it has closed.
    """
    data = bytearray()
    if self._timeout is not None:
        timeout = time.time() + self._timeout
    else:
        timeout = None

    while len(data) < size and (timeout is None or time.time() < timeout):
        if not self._isOpen:
            # If not open, try and re-open
            try:
                self.open()
            except SerialException:
                # Ignore failure to open and just wait a bit
                time.sleep(0.1)
                continue

        try:
            # Read available data
            block = self._socket.recv(size - len(data))
            if block:
                data.extend(block)
            else:
                # no data -> EOF (remote connection closed). If no data at all, loop until
                # we can reopen the socket
                self.close()
                if data:
                    break

        except socket.timeout:
            # just need to get out of recv from time to time to check if
            # still alive
            continue
        except socket.error as e:
            # connection fails -> terminate loop
            raise SerialException('connection failed (%s)' % e)
    return bytes(data)

# Insert ourselves in the serial read routine
#  (could also copy-paste the entire miniterm reader() method in here, which would be meh)
#  (or patch sys.stdout, which would just suck, because who knows what else that'd break)
plain_read = Serial.read
def dehash_serial_read(self, size):
    return dehash_read(self, size, plain_read)

def dehash_socket_read(self, size):
    return dehash_read(self, size, socket_serial_read)

try:
    from pyftdi.serialext.protocol_ftdi import FtdiSerial
    plain_pyftdi_read = FtdiSerial.read
    def dehash_pyftdi_serial_read(self, size):
        return dehash_read(self, size, plain_pyftdi_read)
    FtdiSerial.read = dehash_pyftdi_serial_read
except ImportError:
    pass


def yes_no_to_bool(arg):
    return True if arg == 'yes' else False


# Process "arguments"
arg_justify = "small"
arg_color = False
arg_bold = -1
arg_core = False

dict_path = os.getenv('PBL_CONSOLE_DICT_PATH')
if not dict_path:
    dict_path = 'build/src/fw/loghash_dict.json'

arglist = os.getenv("PBL_CONSOLE_ARGS")
if arglist:
    for arg in arglist.split(","):
        if not arg:
            break
        key, value = arg.split('=')
        if key == "--justify":
            arg_justify = value
        elif key == "--color":
            arg_color = yes_no_to_bool(value)
        elif key == "--bold":
            arg_bold = int(value)
        elif key == "--dict":
            dict_path = value
        elif key == "--core":
            arg_core = yes_no_to_bool(value)
        else:
            raise Exception("Unknown console argument '{}'. Choices are ({})".
                            format(key, ['--justify', '--color', '--bold',
                                         '--dict', '--core']))

dehasher = LogDehash(dict_path, justify=arg_justify,
                     color=arg_color, bold=arg_bold, print_core=arg_core)

Serial.read = dehash_serial_read
SocketSerial.read = dehash_socket_read

# Make sure that the target is set
if sys.argv[1] == 'None':
    raise Exception("No tty specified. Do you have a device attached?")

# Fire it up as usual
main()
