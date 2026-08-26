# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import threading
import time

import serial

PEBBLE_BAUD_RATE = 115200
SERIAL_READ_TIMEOUT = 0.1
SERIAL_WRAPPER_READ_TIMEOUT = 0.5


class SerialPortWrapperException(Exception):
    pass


class SerialPortWrapper(threading.Thread):
    def __init__(self, tty, logfile="serial_dump.txt", baud_rate=PEBBLE_BAUD_RATE):
        threading.Thread.__init__(self)

        self.tty = tty

        self.s = serial.serial_for_url(tty, baud_rate, timeout=SERIAL_READ_TIMEOUT)
        self.read_size = 1

        self.debug_out = None
        # if a logfile is specified log serial output.  If a file descriptor
        # is provided log to the descriptor but do not close the descriptor.
        if logfile is not None:
            if isinstance(logfile, basestring):
                self.debug_out = open(logfile, "wb")
                self._close_debug = True
            elif isinstance(logfile, file):
                self.debug_out = logfile
                self._close_debug = False
            else:
                raise ValueError(
                    "logfile must be a filename or an open file descriptor."
                )

        # This signal is used to protect and signal changes in the below
        # self.data member variable
        self.signal = threading.Condition()
        self.data = ""

        self.die = False
        self.daemon = True
        self.start()

    def run(self):
        while not self.die:
            data = self.s.read(self.read_size)
            if len(data):
                # print 'received data <(%s)>' % data

                self.signal.acquire()
                try:
                    self.data += data

                    if self.debug_out is not None:
                        self.debug_out.write(data)
                        self.debug_out.flush()

                    self.signal.notify_all()
                finally:
                    self.signal.release()

    def write_fast(self, data):
        # print 'writing <(%s)>' % strip_unprintable_chars(data)
        rv = self.s.write(data)
        return rv

    def write_slow(self, data):
        for c in data:
            self.s.write(c)
            time.sleep(0.01)

    def write(self, data):
        self.write_slow(data)

    def readline(self, timeout=SERIAL_WRAPPER_READ_TIMEOUT):
        result = ""

        self.signal.acquire()
        try:
            # Try to find a newline character. If we don't have one, wait until we
            # receive more data before checking again. If we hit the timeout without
            # receiving anything just return what we have.
            idx = self.data.find("\n")
            while idx == -1:
                prev_data_len = len(self.data)

                self.signal.wait(timeout)
                if prev_data_len != len(self.data):
                    idx = self.data.find("\n")
                else:
                    idx = len(self.data) - 1

                if len(self.data) == 0:
                    break

            idx += 1

            result = self.data[:idx].strip()
            self.data = self.data[idx:]

            # print 'returning <(%s)>, %u remaining' % (result, len(self.data))

        finally:
            self.signal.release()

        return result

    def read(self, timeout=SERIAL_WRAPPER_READ_TIMEOUT):
        self.signal.acquire()

        while True:
            prev_data_len = len(self.data)

            self.signal.wait(timeout)

            if prev_data_len == len(self.data):
                # No new data available after our timeout, assume we're done
                break

        result = self.data
        self.data = ""

        self.signal.release()

        return result

    def read_regex(self, regex, timeout=SERIAL_WRAPPER_READ_TIMEOUT):
        match = None
        while not match:
            match = regex.search(self.readline(timeout))
        return match

    def clear(self):
        self.signal.acquire()
        self.data = ""
        self.signal.release()

    def close(self):
        self.die = True
        self.join()

        self.s.close()
        if self.debug_out is not None and self._close_debug:
            self.debug_out.close()
