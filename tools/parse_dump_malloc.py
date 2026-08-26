# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import sys

import sh

total_alloc_size = 0
alloc_count = 0
total_free_size = 0
free_count = 0
largest_free_block = 0

per_file_dict = {}
alloc_list = []

root_path = os.path.join(os.path.dirname(sys.argv[0]), "..")

min_addr = 0
max_addr = 0
current_size = 0
high_water_mark = 0


def get_filename_linenumber(addr_str):
    try:
        line = sh.arm_none_eabi_addr2line(addr_str, exe=elf_path)
    except:
        return ("?", 0)

    line = line.strip()

    index = line.rfind(":")
    filename = line[:index]
    linenumber = line[index + 1 :]
    if ":" not in filename:
        filename = os.path.relpath(filename, root_path)

    return (filename, linenumber)


def handle_line(line, verbose):
    parts = line.split(" ")

    # Skip gdb continuation prompts
    if parts[0].startswith("---"):
        return

    if parts[0] == "Heap":
        metadata_name = " ".join(parts[1:-1])
        if metadata_name == "start":
            global min_addr
            min_addr = int(parts[-1].strip(), 16)
        elif metadata_name == "end":
            global max_addr
            max_addr = int(parts[-1].strip(), 16)
        elif metadata_name == "current size":
            global current_size
            current_size = int(parts[-1].strip())
        elif metadata_name == "high water mark":
            global high_water_mark
            high_water_mark = int(parts[-1].strip())
        return

    def get_part_value(index):
        return parts[index].split(":")[1]

    pc = get_part_value(0)
    addr = get_part_value(1)
    actual_size = int(get_part_value(2))

    filename, linenumber = get_filename_linenumber(pc)

    is_free = int(pc, 0) == 0
    if verbose:
        if is_free:
            print(
                "Size: %6u, Addr: 0x%08x PC: 0x%08x FREE"
                % (actual_size, int(addr, 0), int(pc, 0))
            )
        else:
            print(
                "Size: %6u, Addr: 0x%08x PC: 0x%08x %s:%s"
                % (actual_size, int(addr, 0), int(pc, 0), filename, linenumber)
            )

    global total_alloc_size
    global alloc_count
    global total_free_size
    global free_count
    global largest_free_block

    # Capture largest free block size
    if is_free:
        total_free_size += actual_size
        free_count += 1
        largest_free_block = max(largest_free_block, actual_size)
        return

    total_alloc_size += actual_size
    alloc_count += 1

    filename = filename
    try:
        per_file_dict[filename] += actual_size
    except:
        per_file_dict[filename] = actual_size

    alloc_list.append([int(addr, base=16), actual_size])


def draw_image():
    import cairo

    WIDTH, HEIGHT = 2048, 256
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, WIDTH, HEIGHT)
    ctx = cairo.Context(surface)

    ctx.set_source_rgb(0.8, 0.8, 0.8)
    ctx.set_line_width(0)
    ctx.rectangle(0, 0, WIDTH, HEIGHT)
    ctx.fill()

    def translate_to_pixels(addr):
        return (float(addr - min_addr) / float(max_addr - min_addr)) * WIDTH

    def draw_section(start, end):
        ctx.set_source_rgb(0, 0, 0)
        ctx.set_line_width(0)
        ctx.rectangle(start, 0, end - start, HEIGHT)
        ctx.fill()

    for alloc in alloc_list:
        addr = alloc[0]
        size = alloc[1]
        pixel_start = translate_to_pixels(addr)
        pixel_end = translate_to_pixels(addr + size)
        draw_section(pixel_start, pixel_end)

    ctx.stroke()
    surface.write_to_png("dump_malloc.png")
    print("Drew image to dump_malloc.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--elf_file", required=True)
    parser.add_argument("in_file")

    args = parser.parse_args()

    verbose = False
    if args.verbose is not None:
        verbose = args.verbose

    elf_path = args.elf_file

    with open(args.in_file, "r") as f:
        infile = f.readlines()

    for line in infile:
        line = line.strip()
        if len(line) > 0:
            handle_line(line, verbose)

    if min_addr == 0 or max_addr == 0:
        min_addr = alloc_list[0][0]
        max_addr = alloc_list[0][0] + alloc_list[0][1]
        for alloc in alloc_list:
            low_addr = alloc[0]
            min_addr = min(min_addr, low_addr)
            high_addr = low_addr + alloc[1]
            max_addr = max(max_addr, high_addr)

    if verbose:
        print()
    print("Heap start: 0x%x" % min_addr)
    print("Heap end: 0x%x" % max_addr)
    print("Heap size: %u bytes" % (max_addr - min_addr))
    print("Total allocated: %u bytes, %u blocks" % (total_alloc_size, alloc_count))
    print("High water mark: %u" % high_water_mark)
    print("Total free: %u bytes, %u blocks" % (total_free_size, free_count))
    print("Largest free block: %u" % largest_free_block)
    print()

    per_file_list = [[k, v] for k, v in per_file_dict.iteritems()]
    sorted_per_file_list = sorted(per_file_list, key=lambda v: v[1], reverse=True)
    for k, v in sorted_per_file_list:
        print("%s: %u bytes" % (k, v))

    if args.image:
        draw_image()
