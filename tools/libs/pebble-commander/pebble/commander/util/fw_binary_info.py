# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import struct
from binascii import crc32
from functools import reduce

from . import stm32_crc


class PebbleFirmwareBinaryInfo(object):
    V1_STRUCT_VERSION = 1
    V1_STRUCT_DEFINTION = [
        ("20s", "build_id"),
        ("L", "version_timestamp"),
        ("32s", "version_tag"),
        ("8s", "version_short"),
        ("?", "is_recovery_firmware"),
        ("B", "hw_platform"),
        ("B", "metadata_version"),
    ]
    # The platforms which use a legacy defective crc32
    LEGACY_CRC_PLATFORMS = [
        0,  # unknown (assume legacy)
        1,  # OneEV1
        2,  # OneEV2
        3,  # OneEV2_3
        4,  # OneEV2_4
        5,  # OnePointFive
        6,  # TwoPointFive
        7,  # SnowyEVT2
        8,  # SnowyDVT
        9,  # SpaldingEVT
        10,  # BobbyDVT
        11,  # Spalding
        0xFF,  # OneBigboard
        0xFE,  # OneBigboard2
        0xFD,  # SnowyBigboard
        0xFC,  # SnowyBigboard2
        0xFB,  # SpaldingBigboard
    ]

    def get_crc(self):
        _, ext = os.path.splitext(self.path)
        assert ext == ".bin", "Can only calculate crc for .bin files"
        with open(self.path, "rb") as f:
            image = f.read()
        if self.hw_platform in self.LEGACY_CRC_PLATFORMS:
            # use the legacy defective crc
            return stm32_crc.crc32(image)
        else:
            # use a regular crc
            return crc32(image) & 0xFFFFFFFF

    def _get_footer_struct(self):
        fmt = "<" + reduce(
            lambda s, t: s + t[0], PebbleFirmwareBinaryInfo.V1_STRUCT_DEFINTION, ""
        )
        return struct.Struct(fmt)

    def _get_footer_data_from_bin(self, path):
        with open(path, "rb") as f:
            struct_size = self.struct.size
            f.seek(-struct_size, 2)
            footer_data = f.read()
            return footer_data

    def _get_footer_data(self, path):
        # Subclasses may override to support other input formats.
        _, ext = os.path.splitext(path)
        if ext != ".bin":
            raise ValueError('Unexpected extension. Must be ".bin"')
        return self._get_footer_data_from_bin(path)

    def _parse_footer_data(self, footer_data):
        z = zip(
            PebbleFirmwareBinaryInfo.V1_STRUCT_DEFINTION,
            self.struct.unpack(footer_data),
        )
        return {entry[1]: data for entry, data in z}

    def __init__(self, bin_path):
        self.path = bin_path
        self.struct = self._get_footer_struct()
        self.info = self._parse_footer_data(self._get_footer_data(bin_path))

        # Trim leading NULLS on the strings:
        for k in ["version_tag", "version_short"]:
            self.info[k] = self.info[k].rstrip(b"\x00")

    def __str__(self):
        return str(self.info)

    def __repr__(self):
        return self.info.__repr__()

    def __getattr__(self, name):
        if name in self.info:
            return self.info[name]
        raise AttributeError


# typedef struct ATTR_PACKED FirmwareDescription {
#   uint32_t description_length;
#   uint32_t firmware_length;
#   uint32_t checksum;
# } FirmwareDescription;
FW_DESCR_FORMAT = "<III"
FW_DESCR_SIZE = struct.calcsize(FW_DESCR_FORMAT)


def _generate_firmware_description_struct(firmware_length, firmware_crc):
    return struct.pack(FW_DESCR_FORMAT, FW_DESCR_SIZE, firmware_length, firmware_crc)


def insert_firmware_description_struct(input_binary, output_binary=None):
    fw_bin_info = PebbleFirmwareBinaryInfo(input_binary)
    with open(input_binary, "rb") as inf:
        fw_bin = inf.read()
        fw_crc = fw_bin_info.get_crc()

    image = _generate_firmware_description_struct(len(fw_bin), fw_crc) + fw_bin

    if output_binary:
        with open(output_binary, "wb") as outf:
            outf.write(image)
        return None

    return image
