# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from __future__ import print_function

import sys
import traceback

import pebble.pulse2.exceptions

from .. import PebbleCommander
from ..util import stm32_crc
from ..util.fw_binary_info import insert_firmware_description_struct


def _load(connection, image, progress, verbose, address):
    image_crc = stm32_crc.crc32(image)

    progress_cb = None
    if progress or verbose:

        def progress_cb(acked):
            print("." if acked else "R", end="")
            sys.stdout.flush()

    if progress or verbose:
        print("Erasing... ", end="")
        sys.stdout.flush()
    try:
        connection.flash.erase(address, len(image))
    except pebble.pulse2.exceptions.PulseException as e:
        detail = "".join(traceback.format_exception_only(type(e), e))
        if verbose:
            detail = "\n" + traceback.format_exc()
        print("Erase failed! " + detail)
        return False
    if progress or verbose:
        print("done.")
        sys.stdout.flush()

    try:
        retries = connection.flash.write(address, image, progress_cb=progress_cb)
    except pebble.pulse2.exceptions.PulseException as e:
        detail = "".join(traceback.format_exception_only(type(e), e))
        if verbose:
            detail = "\n" + traceback.format_exc()
        print("Write failed! " + detail)
        return False

    result_crc = connection.flash.crc(address, len(image))

    if progress or verbose:
        print()
    if verbose:
        print("Retries: %d" % retries)

    if result_crc != image_crc:
        print("CRC mismatch, got 0x%08X but expected %08X" % (result_crc, image_crc))

    return result_crc == image_crc


def load_firmware(connection, fin, progress, verbose, address=None):
    if address is None:
        # If address is unspecified, assume we want the prf address
        _, address, length = connection.flash.query_region_geometry(
            connection.flash.REGION_PRF
        )
    address = int(address)

    image = insert_firmware_description_struct(fin)
    if _load(connection, image, progress, verbose, address):
        connection.flash.finalize_region(connection.flash.REGION_PRF)
        return True
    return False


def load_resources(connection, fin, progress, verbose):
    _, address, length = connection.flash.query_region_geometry(
        connection.flash.REGION_SYSTEM_RESOURCES
    )

    with open(fin, "rb") as f:
        data = f.read()
    assert len(data) <= length
    if _load(connection, data, progress, verbose, address):
        connection.flash.finalize_region(connection.flash.REGION_SYSTEM_RESOURCES)
        return True
    return False


@PebbleCommander.command()
def image_resources(cmdr, pack="build/system_resources.pbpack"):
    """Image resources."""
    load_resources(
        cmdr.connection, pack, progress=cmdr.interactive, verbose=cmdr.interactive
    )


@PebbleCommander.command()
def image_firmware(cmdr, firm="build/pebbleos.bin", address=None):
    """Image recovery firmware."""
    if address is not None:
        address = int(str(address), 0)
    load_firmware(
        cmdr.connection,
        firm,
        progress=cmdr.interactive,
        verbose=cmdr.interactive,
        address=address,
    )
