# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""Writer for Libpcap capture files

https://wiki.wireshark.org/Development/LibpcapFileFormat
"""


import struct
import threading
import time

LINKTYPE_PPP_WITH_DIR = 204


class PcapWriter:
    def __init__(self, outfile, linktype):
        self.lock = threading.Lock()
        self.outfile = outfile
        self._write_pcap_header(linktype)

    def close(self):
        with self.lock:
            self.outfile.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _write_pcap_header(self, linktype):
        header = struct.pack(
            "!IHHiIII",
            0xA1B2C3D4,  # guint32 magic_number
            2,  # guint16 version_major
            4,  # guint16 version_minor
            0,  # guint32 thiszone
            0,  # guint32 sigfigs (unused)
            65535,  # guint32 snaplen
            linktype,
        )  # guint32 network
        self.outfile.write(header)

    def write_packet(self, data, timestamp=None, orig_len=None):
        assert len(data) <= 65535
        if timestamp is None:
            timestamp = time.time()
        if orig_len is None:
            orig_len = len(data)
        ts_seconds = int(timestamp)
        ts_usec = int((timestamp - ts_seconds) * 1000000)
        header = struct.pack("!IIII", ts_seconds, ts_usec, len(data), orig_len)
        with self.lock:
            self.outfile.write(header + data)
