# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

try:
    import gdb
except ImportError:
    raise Exception(
        "This file is a GDB module.\n"
        "It is not intended to be run outside of GDB.\n"
        "Hint: to load a script in GDB, use `source this_file.py`"
    )

import itertools
from collections import namedtuple

import gdb_utils


class CorruptionCode:
    def __init__(self, message):
        self.message = message

    def __repr__(self):
        return f'CorruptionCode("{self.message}")'

    def __str__(self):
        return self.message


class HeapException(Exception):
    pass


class HeapBlock(namedtuple("HeapBlock", "info data size allocated corruption_code")):
    def __new__(cls, info, data, size, allocated=False, corruption_code=None):
        return cls._make([info, data, size, allocated, corruption_code])

    def cast(self, obj_type, clone=False):
        if clone:
            return HeapBlock(
                self.info,
                self.cast(obj_type),
                self.size,
                self.allocated,
                self.corruption_code,
            )
        else:
            if obj_type:
                return self.data.cast(gdb.lookup_type(obj_type).pointer())
            else:
                return self.data


class Heap:
    BlockSizeZero = CorruptionCode("Block size is zero")
    PrevSizeZero = CorruptionCode("Prev size is zero")
    WrongPrevSize = CorruptionCode("PrevSize is less than the size of the last block")

    def __init__(self, heap, show_progress=False):
        self.heap_ptr = gdb.parse_and_eval(heap)

        if self.heap_ptr.type != gdb.lookup_type("Heap").pointer():
            raise HeapException("Error: argument must be of type (Heap *)")

        self.alignment_type = gdb.lookup_type("Alignment_t")
        self.alignment_size = int(self.alignment_type.sizeof)

        self.start = self.heap_ptr["begin"]
        self.end = self.heap_ptr["end"]
        self.high_water_mark = self.heap_ptr["high_water_mark"]

        self.heap_info_type = gdb.lookup_type("HeapInfo_t")
        self.size = gdb_utils.Address(str(self.end)) - gdb_utils.Address(
            str(self.start)
        )

        self.malloc_instrumentation = "pc" in list(self.heap_info_type.keys())
        self.corrupted = False
        self.show_progress = show_progress

        self._process_heap()

    def __iter__(self):
        return iter(self.block_list)

    def _process_heap(self):
        self.block_list = []
        segment_ptr = self.start
        block_size = 0

        loop_count = itertools.count()
        while segment_ptr < self.end:
            if self.show_progress:
                gdb.write(".")
                gdb.flush()
            if next(loop_count) > 10000 or self.corrupted:
                print("ERROR: heap corrupted")
                return

            block_prev_size = int(segment_ptr["PrevSize"])
            block_size_prev = block_size
            is_allocated = bool(segment_ptr["is_allocated"])
            block_size = int(segment_ptr["Size"])

            size_bytes = block_size * self.alignment_size

            corruption_code = None
            if block_size <= 0:
                corruption_code = self.BlockSizeZero
            elif segment_ptr > self.start:
                if block_prev_size == 0:
                    corruption_code = self.PrevSizeZero
                elif block_prev_size != block_size_prev:
                    corruption_code = self.WrongPrevSize

            if corruption_code:
                self.corrupted = True

            block = HeapBlock(
                segment_ptr,
                segment_ptr["Data"].address,
                size_bytes,
                is_allocated,
                corruption_code,
            )
            self.block_list.append(block)

            segment_ptr = (
                segment_ptr.cast(self.alignment_type.pointer()) + block_size
            ).cast(self.heap_info_type.pointer())

    def block_size(self, bytes):
        offset = int(gdb.lookup_type("HeapInfo_t").sizeof) - int(
            gdb.lookup_type("AlignmentStruct_t").sizeof
        )
        offset_blocks = offset // self.alignment_size

        blocks = (bytes + self.alignment_size - 1) // self.alignment_size
        common_size = blocks * self.alignment_size + offset

        # Heap blocks with less than one block's worth of space between it
        # and the next will grow to take up that space.
        return frozenset(
            common_size + x * self.alignment_size for x in range(offset_blocks + 1)
        )

    def object_size(self, obj_type):
        bytes = int(gdb.lookup_type(obj_type).sizeof)
        return self.block_size(bytes)

    def objects_of(self, obj_type):
        sizes = self.object_size(obj_type)
        return [block for block in self if block.size in sizes]

    def allocated_blocks(self):
        return [block for block in self if block.allocated]

    def free_blocks(self):
        return [block for block in self if not block.allocated]
