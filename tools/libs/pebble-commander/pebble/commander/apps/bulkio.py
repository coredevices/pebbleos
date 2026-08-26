# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import collections
import logging
import struct

from ..exceptions import PebbleCommanderError


class ResponseParseError(PebbleCommanderError):
    pass


class EraseError(PebbleCommanderError):
    pass


class OpenCommand:
    command_type = 1
    command_struct = struct.Struct("<BB")

    def __init__(self, domain, extra=None):
        self.domain = domain
        self.extra = extra

    @property
    def packet(self):
        cmd = self.command_struct.pack(self.command_type, self.domain)
        if self.extra:
            cmd += self.extra

        return cmd


class CloseCommand:
    command_type = 2
    command_struct = struct.Struct("<BB")

    def __init__(self, fd):
        self.fd = fd

    @property
    def packet(self):
        return self.command_struct.pack(self.command_type, self.fd)


class ReadCommand:
    command_type = 3
    command_struct = struct.Struct("<BBII")

    def __init__(self, fd, address, length):
        self.fd = fd
        self.address = address
        self.length = length

    @property
    def packet(self):
        return self.command_struct.pack(
            self.command_type, self.fd, self.address, self.length
        )


class WriteCommand:
    command_type = 4
    command_struct = struct.Struct("<BBI")
    header_size = command_struct.size

    def __init__(self, fd, address, data):
        self.fd = fd
        self.address = address
        self.data = data

    @property
    def packet(self):
        return (
            self.command_struct.pack(self.command_type, self.fd, self.address)
            + self.data
        )


class CRCCommand:
    command_type = 5
    command_struct = struct.Struct("<BBII")

    def __init__(self, fd, address, length):
        self.fd = fd
        self.address = address
        self.length = length

    @property
    def packet(self):
        return self.command_struct.pack(
            self.command_type, self.fd, self.address, self.length
        )


class StatCommand:
    command_type = 6
    command_struct = struct.Struct("<BB")

    def __init__(self, fd):
        self.fd = fd

    @property
    def packet(self):
        return self.command_struct.pack(self.command_type, self.fd)


class EraseCommand:
    command_type = 7
    command_struct = struct.Struct("<BB")

    def __init__(self, domain, extra=None):
        self.domain = domain
        self.extra = extra

    @property
    def packet(self):
        cmd = self.command_struct.pack(self.command_type, self.domain)
        if self.extra:
            cmd += self.extra

        return cmd


class OpenResponse:
    response_type = 128
    response_format = "<xB"
    response_struct = struct.Struct(response_format)
    header_size = response_struct.size
    Response = collections.namedtuple("OpenResponse", "fd")

    @classmethod
    def parse(cls, response):
        response_type = ord(response[0])
        if response_type != cls.response_type:
            raise ResponseParseError("Unexpected response type: %r" % response_type)
        return cls.Response._make(cls.response_struct.unpack(response))


class CloseResponse:
    response_type = 129
    response_format = "<xB"
    response_struct = struct.Struct(response_format)
    header_size = response_struct.size
    Response = collections.namedtuple("CloseResponse", "fd")

    @classmethod
    def parse(cls, response):
        response_type = ord(response[0])
        if response_type != cls.response_type:
            raise ResponseParseError("Unexpected response type: %r" % response_type)
        return cls.Response._make(cls.response_struct.unpack(response))


class ReadResponse:
    response_type = 130
    response_format = "<xBI"
    response_struct = struct.Struct(response_format)
    header_size = response_struct.size
    Response = collections.namedtuple("ReadResponse", "fd address data")

    @classmethod
    def parse(cls, response):
        if ord(response[0]) != cls.response_type:
            raise ResponseParseError("Unexpected response: %r" % response)
        header = response[: cls.header_size]
        body = response[cls.header_size :]
        (
            fd,
            address,
        ) = cls.response_struct.unpack(header)
        return cls.Response(fd, address, body)


class WriteResponse:
    response_type = 131
    response_format = "<xBII"
    response_struct = struct.Struct(response_format)
    header_size = response_struct.size
    Response = collections.namedtuple("WriteResponse", "fd address length")

    @classmethod
    def parse(cls, response):
        response_type = ord(response[0])
        if response_type != cls.response_type:
            raise ResponseParseError("Unexpected response type: %r" % response_type)
        return cls.Response._make(cls.response_struct.unpack(response))


class CRCResponse:
    response_type = 132
    response_format = "<xBIII"
    response_struct = struct.Struct(response_format)
    header_size = response_struct.size
    Response = collections.namedtuple("CRCResponse", "fd address length crc")

    @classmethod
    def parse(cls, response):
        response_type = ord(response[0])
        if response_type != cls.response_type:
            raise ResponseParseError("Unexpected response type: %r" % response_type)
        return cls.Response._make(cls.response_struct.unpack(response))


class StatResponse:
    response_type = 133

    def __init__(self, name, format, fields):
        self.name = name
        self.struct = struct.Struct("<xBB" + format)
        self.tuple = collections.namedtuple(name, "fd flags " + fields)

    def parse(self, response):
        response_type = ord(response[0])
        if response_type != self.response_type:
            raise ResponseParseError("Unexpected response type: %r" % response_type)
        return self.tuple._make(self.struct.unpack(response))

    def __repr__(self):
        return f"StatResponse({self.name!r}, {self.struct!r}, {self.tuple!r})"


class EraseResponse:
    response_type = 134
    response_format = "<xBb"
    response_struct = struct.Struct(response_format)
    header_size = response_struct.size
    Response = collections.namedtuple("EraseResponse", "domain status")

    @classmethod
    def parse(cls, response):
        response_type = ord(response[0])
        if response_type != cls.response_type:
            raise ResponseParseError("Unexpected response type: %r" % response_type)
        return cls.Response._make(cls.response_struct.unpack(response))


def enum(**enums):
    return type("Enum", (), enums)


ReadDomains = enum(MEMORY=1, EXTERNAL_FLASH=2, FRAMEBUFFER=3, COREDUMP=4, PFS=5)


class PULSEIO_Base:
    ERASE_FORMAT = None
    STAT_FORMAT = None
    DOMAIN = None

    def __init__(self, socket, *args, **kwargs):
        self.socket = socket
        self.pos = 0

        options = self._process_args(*args, **kwargs)
        resp = self._send_and_receive(OpenCommand, OpenResponse, self.DOMAIN, options)
        self.fd = resp.fd

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    @staticmethod
    def _process_args(*args, **kwargs):
        return ""

    def _send_and_receive(self, cmd_type, resp_type, *args):
        cmd = cmd_type(*args)
        self.socket.send(cmd.packet)
        ret = self.socket.receive(block=True)
        return resp_type.parse(ret)

    def close(self):
        if self.fd is not None:
            resp = self._send_and_receive(CloseCommand, CloseResponse, self.fd)
            assert resp.fd == self.fd
            self.fd = None

    def seek_absolute(self, pos):
        if pos < 0:
            raise ValueError("Cannot seek to before start of file")
        self.pos = pos

    def seek_relative(self, num_bytes):
        if (self.pos + num_bytes) < 0:
            raise ValueError("Cannot seek to before start of file")
        self.pos += num_bytes

    @classmethod
    def erase(cls, socket, *args):
        if cls.ERASE_FORMAT == "raw":
            options = "".join(args)
        elif cls.ERASE_FORMAT:
            options = struct.pack("<" + cls.ERASE_FORMAT, *args)
        else:
            raise NotImplementedError(
                "Erase is not supported for domain %d" % cls.DOMAIN
            )
        cmd = EraseCommand(cls.DOMAIN, options)
        socket.send(cmd.packet)
        status = 1
        while status > 0:
            ret = socket.receive(block=True)
            resp = EraseResponse.parse(ret)
            logging.debug("ERASE: domain %d status %d", resp.domain, resp.status)
            status = resp.status

        if status < 0:
            raise EraseError(status)

    def write(self, data):
        if self.fd is None:
            raise ValueError("Handle is not open")

        mss = self.socket.mtu - WriteCommand.header_size

        for offset in xrange(0, len(data), mss):
            segment = data[offset : offset + mss]
            resp = self._send_and_receive(
                WriteCommand, WriteResponse, self.fd, self.pos, segment
            )
            assert resp.fd == self.fd
            assert resp.address == self.pos
            self.pos += len(segment)

    def read(self, length):
        if self.fd is None:
            raise ValueError("Handle is not open")

        cmd = ReadCommand(self.fd, self.pos, length)
        self.socket.send(cmd.packet)

        data = bytearray()
        bytes_left = length
        while bytes_left > 0:
            packet = self.socket.receive(block=True)
            fd, chunk_offset, chunk = ReadResponse.parse(packet)
            assert fd == self.fd
            data.extend(chunk)

            bytes_left -= len(chunk)
        return data

    def crc(self, length):
        if self.fd is None:
            raise ValueError("Handle is not open")

        resp = self._send_and_receive(
            CRCCommand, CRCResponse, self.fd, self.pos, length
        )
        return resp.crc

    def stat(self):
        if self.fd is None:
            raise ValueError("Handle is not open")

        if not self.STAT_FORMAT:
            raise NotImplementedError(
                "Stat is not supported for domain %d" % self.DOMAIN
            )

        return self._send_and_receive(StatCommand, self.STAT_FORMAT, self.fd)


class PULSEIO_Memory(PULSEIO_Base):
    DOMAIN = ReadDomains.MEMORY

    # uint32 for address, uint32 for length
    ERASE_FORMAT = "II"


class PULSEIO_ExternalFlash(PULSEIO_Base):
    DOMAIN = ReadDomains.EXTERNAL_FLASH

    # uint32 for address, uint32 for length
    ERASE_FORMAT = "II"


class PULSEIO_Framebuffer(PULSEIO_Base):
    DOMAIN = ReadDomains.FRAMEBUFFER
    STAT_FORMAT = StatResponse(
        "FramebufferAttributes", "BBBI", "width height bpp length"
    )


class PULSEIO_Coredump(PULSEIO_Base):
    DOMAIN = ReadDomains.COREDUMP
    STAT_FORMAT = StatResponse("CoredumpAttributes", "BI", "unread length")
    ERASE_FORMAT = "I"

    @staticmethod
    def _process_args(slot):
        return struct.pack("<I", slot)


class PULSEIO_PFS(PULSEIO_Base):
    DOMAIN = ReadDomains.PFS
    STAT_FORMAT = StatResponse("PFSFileAttributes", "I", "length")
    ERASE_FORMAT = "raw"

    OP_FLAG_READ = 1 << 0
    OP_FLAG_WRITE = 1 << 1
    OP_FLAG_OVERWRITE = 1 << 2
    OP_FLAG_SKIP_HDR_CRC_CHECK = 1 << 3
    OP_FLAG_USE_PAGE_CACHE = 1 << 4

    @staticmethod
    def _process_args(filename, mode="r", flags=0xFE, initial_size=0):
        mode_num = PULSEIO_PFS.OP_FLAG_READ
        if "w" in mode:
            mode_num |= PULSEIO_PFS.OP_FLAG_WRITE
        return struct.pack("<BBI", mode_num, flags, initial_size) + filename


class BulkIO:
    PROTOCOL_NUMBER = 0x3E21
    DOMAIN_MAP = {"pfs": PULSEIO_PFS, "framebuffer": PULSEIO_Framebuffer}

    def __init__(self, link):
        self.socket = link.open_socket("reliable", self.PROTOCOL_NUMBER)

    def open(self, domain, *args, **kwargs):
        return self.DOMAIN_MAP[domain](self.socket, *args, **kwargs)

    def erase(self, domain, *args, **kwargs):
        return self.DOMAIN_MAP[domain].erase(self.socket, *args, **kwargs)

    def close(self):
        self.socket.close()
