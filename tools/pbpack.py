#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import struct

import stm32_crc


class ResourcePackTableEntry:
    TABLE_ENTRY_FMT = "<IIII"

    def __init__(self, content_index, offset, length, crc):
        # The index into the contents array that holds our data in the ResourcePack object
        self.content_index = content_index
        # The offset into the body of the pbpack in bytes that we can find the content
        self.offset = offset
        # The length of the content
        self.length = length
        # The CRC of the content
        self.crc = crc

    def serialize(self, file_id):
        fmt = self.TABLE_ENTRY_FMT
        return struct.pack(fmt, file_id, self.offset, self.length, self.crc)

    @classmethod
    def deserialize(cls, table_entry_data):
        """
        Return a tuple of file_id, ResourcePackTableEntry object for the given data.
        """

        fmt = cls.TABLE_ENTRY_FMT
        file_id, offset, length, crc = struct.unpack(fmt, table_entry_data)

        entry = ResourcePackTableEntry(-1, offset, length, crc)
        return file_id, entry

    def __repr__(self):
        return str(self.__dict__)


class ResourcePack:
    """Pebble resource pack file format (de)serialization tools.

    An instance of this class is an in-memory representation of a resource
    pack. The class has a number of methods to facilitate (de)serialization
    of .pbpack files.

    """

    TABLE_ENTRY_SIZE_BYTES = 16
    MANIFEST_FMT = "<III"
    MANIFEST_SIZE_BYTES = 12

    def get_content_crc(self):
        all_contents = self.serialize_content()
        return stm32_crc.crc32(all_contents)

    def serialize_manifest(self, crc=None, timestamp=None):
        fmt = self.MANIFEST_FMT

        return struct.pack(fmt, len(self.table_entries), self.crc, self.timestamp)

    def serialize_table(self):
        # Serialize these entries into table_data
        cur_file_id = 1
        table_data = b""
        for cur_file_id, table_entry in enumerate(self.table_entries, start=1):
            table_data += table_entry.serialize(cur_file_id)

        # Pad the rest of the table_data up to table_size
        for i in range(cur_file_id, self.table_size):
            table_data += ResourcePackTableEntry(0, 0, 0, 0).serialize(0)

        return table_data

    def serialize_content(self):
        """
        Serialize the content in the order dictated by offsets in the table entries
        """

        serialized_content_indexes = set()
        serialized_content = []
        for entry in sorted(self.table_entries, key=lambda e: e.offset):
            if entry.content_index in serialized_content_indexes:
                continue

            serialized_content_indexes.add(entry.content_index)

            serialized_content.append(self.contents[entry.content_index])

        return b"".join(serialized_content)

    @classmethod
    def deserialize(cls, f_in, is_system=True):
        resource_pack = cls(is_system)

        # Parse manifest:
        manifest_data = f_in.read(cls.MANIFEST_SIZE_BYTES)
        (num_files, crc, timestamp) = struct.unpack(cls.MANIFEST_FMT, manifest_data)
        resource_pack.num_files = num_files
        resource_pack.crc = crc
        resource_pack.timestamp = timestamp

        # Parse table entries:
        resource_pack.table_entries = []
        for n in range(num_files):
            table_entry = f_in.read(cls.TABLE_ENTRY_SIZE_BYTES)
            file_id, entry = ResourcePackTableEntry.deserialize(table_entry)

            if file_id == 0:
                # No more entries
                break

            if file_id != n + 1:
                raise Exception(
                    "File ID is expected to be %u, but was %u" % (n + 1, file_id)
                )

            resource_pack.table_entries.append(entry)

        if len(resource_pack.table_entries) != num_files:
            raise Exception(
                "Number of files in manifest is %u, but actual"
                "number is %u" % (num_files, n)
            )

        # Figure out which content_index to assign to each table entry. Find all unique offset and
        # length combinations and then assign content indexes appropriately. We need to include the
        # length because we allow zero length resources and a zero length resource will have the
        # same offset as a non-zero length resource
        unique_offsets = set()
        for e in resource_pack.table_entries:
            unique_offsets.add((e.offset, e.length))
        unique_offsets = list(unique_offsets)

        for e in resource_pack.table_entries:
            e.content_index = unique_offsets.index((e.offset, e.length))

        # Fetch the contents, make sure we only load each unique piece of content once
        loaded_content_indexes = set()
        for entry in sorted(resource_pack.table_entries, key=lambda e: e.content_index):
            if entry.content_index in loaded_content_indexes:
                # Already loaded, just skip
                continue

            loaded_content_indexes.add(entry.content_index)

            f_in.seek(entry.offset + resource_pack.content_start)
            content = f_in.read(entry.length)

            calculated_crc = stm32_crc.crc32(content)

            if calculated_crc != entry.crc:
                raise Exception(
                    "Entry %s does not match CRC of content (%u). "
                    "Hint: try with%s the --app flag"
                    % (entry, calculated_crc, "" if is_system else "out")
                )

            resource_pack.contents.append(content)

        resource_pack.finalized = True

        return resource_pack

    def finalize(self):
        """
        Take all the resources that have been added using add_resource and finalize the pack. Once
        a pack is finalized no more resources may be added.
        """

        if len(self.table_entries) > self.table_size:
            raise Exception(
                "Exceeded max number of resources. Must have %d or "
                "fewer" % self.table_size
            )

        # Assign offsets to each of the entries in reverse order. The reason we do this in reverse
        # is so that the resource at the end is guaranteed to be at the end of the pack.
        #
        # This is required because the firmware looks at the offset + length of the final resource
        # in the table in order to determine the total size of the pack. If the final resource
        # is a duplicate of an earlier resource and if we assigned offsets starting at the
        # beginning of the table, that final resource would end up pointing to an assigned
        # offset somewhere in the middle of the pack, causing the pack to appear to be truncated.
        current_offset = sum(len(c) for c in self.contents)
        for table_entry in reversed(self.table_entries):
            if table_entry.offset == -1:
                # This entry doesn't have an offset in the output file yet, assign one to all
                # entries that share the same content index

                current_offset -= table_entry.length

                for e in self.table_entries:
                    if e.content_index == table_entry.content_index:
                        e.offset = current_offset

        self.crc = self.get_content_crc()

        self.finalized = True

    def serialize(self, f_out):
        if not self.finalized:
            self.finalize()

        f_out.write(self.serialize_manifest(self.crc))
        f_out.write(self.serialize_table())
        f_out.write(self.serialize_content())

        return self.crc

    def add_resource(self, content):
        if self.finalized:
            raise Exception(
                "Cannot add additional resource, "
                + "resource pack has already been finalized"
            )

        # If resource already is present, add to table only
        try:
            # Try to find the index for this content if it already exists. If it doesn't we'll
            # throw the ValueError.
            content_index = self.contents.index(content)
        except ValueError:
            # This content is completely new, add it to the contents list.
            self.contents.append(content)
            content_index = len(self.contents) - 1

        crc = stm32_crc.crc32(content)

        # Use -1 as the offset as we don't assign offsets until serialize_table
        self.table_entries.append(
            ResourcePackTableEntry(content_index, -1, len(content), crc)
        )

    def dump(self):
        """
        Dump a bunch of information about this pbpack to stdout
        """

        print("Manifest CRC: 0x%x" % self.crc)
        print("Calculated CRC: 0x%x" % self.get_content_crc())
        print("Num Items: %u" % len(self.table_entries))
        for i, entry in enumerate(self.table_entries, start=1):
            print(
                "  %u: Offset %u Length %u CRC 0x%x"
                % (i, entry.offset, entry.length, entry.crc)
            )

    def __init__(self, is_system):
        self.table_size = 768 if is_system else 256
        self.content_start = (
            self.MANIFEST_SIZE_BYTES + self.table_size * self.TABLE_ENTRY_SIZE_BYTES
        )

        # Note that we never actually set the timestamp in newly generated pbpacks. The timestamp
        # field in the manifest is unused in practice and we'll probably reuse that field for
        # another purpose in the future.
        self.timestamp = 0

        # CRC of the content. Note that it only covers the content itself and not the table nor
        # the manifest. Calculated during finalize().
        self.crc = 0

        # List of binary content, where each entry is the raw processed data for each unique
        # resource.
        self.contents = []

        # List of resources that are in the pack. Note that this list may be longer than the
        # self.contents list if there are duplicates, duplicated entries (exact same data) will
        # not be repeated in self.contents. Each entry is a ResourcePackTableEntry
        self.table_entries = []

        # Indicates that the ResourcePack has been built and no more resources can be added.
        self.finalized = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="dump pbpack metadata")

    parser.add_argument("pbpack_path", help="path to pbpack to dump")
    parser.add_argument(
        "--app",
        default=False,
        action="store_true",
        help="Indicate this pbpack is an app pbpack",
    )

    args = parser.parse_args()

    with open(args.pbpack_path, "rb") as f:
        pack = ResourcePack.deserialize(f, is_system=not args.app)

    pack.dump()
