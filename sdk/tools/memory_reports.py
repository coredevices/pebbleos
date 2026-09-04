# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


def _convert_bytes_to_kilobytes(number_bytes):
    """
    Convert the input from bytes into kilobytes
    :param number_bytes: the number of bytes to convert
    :return: the input value converted to kilobytes
    """
    NUMBER_BYTES_IN_KBYTE = 1024
    return int(number_bytes) / NUMBER_BYTES_IN_KBYTE


def app_memory_report(
    platform_name,
    bin_type,
    app_size,
    max_ram,
    free_ram,
    resource_size=None,
    max_resource_size=None,
):
    """
    This method provides a formatted string for printing the memory usage of this binary to the
    console.
    :param platform_name: the name of the current HW platform being targeted
    :param bin_type: the type of binary being built (app, lib, worker)
    :param app_size: the size of the binary
    :param max_ram: the maximum allowed size of the binary
    :param free_ram: the amount of remaining memory
    :param resource_size: the size of the resource pack
    :param max_resource_size: the maximum allowed size of the resource pack
    :return: a tuple containing the color for the string print, and the string to print
    """
    LABEL = (
        "-------------------------------------------------------\n{} {} MEMORY USAGE\n"
    )
    RESOURCE_SIZE = "Total size of resources:        {} bytes / {}KB\n"
    MEMORY_USAGE = (
        "Total footprint in RAM:         {} bytes / {}KB\n"
        "Free RAM available (heap):      {} bytes\n"
        "-------------------------------------------------------"
    )

    if resource_size and max_resource_size:
        report = (
            LABEL.format(platform_name.upper(), bin_type.upper())
            + RESOURCE_SIZE.format(
                resource_size, _convert_bytes_to_kilobytes(max_resource_size)
            )
            + MEMORY_USAGE.format(
                app_size, _convert_bytes_to_kilobytes(max_ram), free_ram
            )
        )
    else:
        report = LABEL.format(
            platform_name.upper(), bin_type.upper()
        ) + MEMORY_USAGE.format(
            app_size, _convert_bytes_to_kilobytes(max_ram), free_ram
        )

    return "YELLOW", report


def app_resource_memory_error(platform_name, resource_size, max_resource_size):
    """
    This method provides a formatted error message for printing to the console when the resource
    size exceeds the maximum resource size supported by the Pebble firmware.
    :param platform_name: the name of the current HW platform being targeted
    :param resource_size: the size of the resource pack
    :param max_resource_size: the maximum allowed size of the resource pack
    :return: a tuple containing the color for the string print, and the string to print
    """
    report = (
        "======================================================\n"
        f"Build failed: {platform_name}\n"
        f"Error: Resource pack is too large ({_convert_bytes_to_kilobytes(resource_size)}KB / {_convert_bytes_to_kilobytes(max_resource_size)}KB)\n"
        "======================================================\n"
    )

    return "RED", report


def app_appstore_resource_memory_error(
    platform_name, resource_size, max_appstore_resource_size
):
    """
    This method provides a formatted warning message for printing to the console when the resource
    pack size exceeds the maximum allowed resource size for the appstore.
    :param platform_name: the name of the current HW platform being targeted
    :param resource_size: the size of the resource pack
    :param max_appstore_resource_size: the maximum appstore-allowed size of the resource pack
    :return: a tuple containing the color for the string print, and the string to print
    """
    report = (
        f"WARNING: Your {platform_name} app resources are too large ({_convert_bytes_to_kilobytes(resource_size)}KB / {_convert_bytes_to_kilobytes(max_appstore_resource_size)}KB). You will not be "
        "able "
        "to publish your app.\n"
    )

    return "RED", report


def bytecode_memory_report(platform_name, bytecode_size, bytecode_max):
    """
    This method provides a formatted string for printing the memory usage for this Rocky bytecode
    file to the console.
    :param platform_name: the name of the current HW platform being targeted
    :param bytecode_size: the size of the bytecode file, in bytes
    :param bytecode_max: the max allowed size of the bytecode file, in bytes
    :return: a tuple containing the color for the string print, and the string to print
    """
    LABEL = "-------------------------------------------------------\n{} MEMORY USAGE\n"
    BYTECODE_USAGE = (
        "Total size of snapshot:        {}KB / {}KB\n"
        "-------------------------------------------------------"
    )

    report = LABEL.format(platform_name.upper()) + BYTECODE_USAGE.format(
        _convert_bytes_to_kilobytes(bytecode_size),
        _convert_bytes_to_kilobytes(bytecode_max),
    )

    return "YELLOW", report


def simple_memory_report(platform_name, bin_size, resource_size=None):
    """
    This method provides a formatted string for printing the memory usage for this binary to the
    console.
    :param platform_name: the name of the current HW platform being targeted
    :param bin_size: the size of the binary
    :param resource_size: the size of the resource pack
    :return: a tuple containing the color for the string print, and the string to print
    """
    LABEL = "-------------------------------------------------------\n{} MEMORY USAGE\n"
    RESOURCE_SIZE = "Total size of resources:        {} bytes\n"
    MEMORY_USAGE = (
        "Total footprint in RAM:         {} bytes\n"
        "-------------------------------------------------------"
    )

    if resource_size:
        report = (
            LABEL.format(platform_name.upper())
            + RESOURCE_SIZE.format(resource_size)
            + MEMORY_USAGE.format(bin_size)
        )
    else:
        report = LABEL.format(platform_name.upper()) + MEMORY_USAGE.format(bin_size)

    return "YELLOW", report
