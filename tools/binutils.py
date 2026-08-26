# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os.path
import re
import subprocess
import sys
import tempfile

import sh

NM_LINE_PATTERN = re.compile(
    r"""([0-9a-f]+)\s+ # address
                             ([0-9a-f]+)\s+ # size
                             ([dDbBtTrR])\s+ # section type
                             (\S+) # name
                             \s*((\S+)\:([0-9]+))?$ # filename + line
                             """,
    flags=re.VERBOSE,
)


class Symbol:
    def __init__(self, name, size):
        self.name = name
        self.size = size

    def __str__(self):
        return "<Symbol %s: %u>" % (self.name, self.size)


class FileInfo:
    def __init__(self, filename):
        self.filename = filename
        self.size = 0
        self.symbols = {}

    def add_entry(self, symbol_name, size):
        if symbol_name in self.symbols:
            return

        self.size += size
        self.symbols[symbol_name] = Symbol(symbol_name, size)

    def remove_entry(self, symbol_name):
        result = self.symbols.pop(symbol_name, None)
        if result is not None:
            self.size -= result.size
        return result

    def pprint(self, verbose):
        print("  %s: size %u" % (self.filename, self.size))
        if verbose:
            l = sorted(self.symbols.values(), key=lambda x: -x.size)
            for s in l:
                print("    %6u %-36s" % (s.size, s.name))

    def __str__(self):
        return "<FileInfo %s: %u>" % (self.filename, self.size)


class SectionInfo:
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.size = 0
        self.files = {}

    def add_entry(self, name, filename, size):
        self.count += 1
        self.size += size

        if filename not in self.files:
            self.files[filename] = FileInfo(filename)

        self.files[filename].add_entry(name, size)

    def remove_unknown_entry(self, name):
        if "Unknown" not in self.files:
            return
        result = self.files["Unknown"].remove_entry(name)
        if result is not None:
            self.size -= result.size
        return result

    def get_files(self):
        return self.files.values()

    def pprint(self, summary, verbose):
        print("%s: count %u size %u" % (self.name, self.count, self.size))

        if not summary:
            l = self.files.values()
            l = sorted(l, key=lambda f: -f.size)
            for f in l:
                f.pprint(verbose)


def analyze_elf(elf_file_path, sections_letters, use_fast_nm):
    """Analyzes the elf file, using binutils.
    section_letters -- string of letters representing the sections to
                       analyze, e.g. 'tbd' => text, bss and data.
    use_fast_nm -- If False, a slow lookup method is used to avoid a bug in
                `nm`. If True, the faster `nm -S -l` is used.
    Returns a dictionary with SectionInfo objects for each section.
    """

    def make_sections_dict(sections_letters):
        sections = {}
        for s in sections_letters:
            if s == "b":
                sections["b"] = SectionInfo(".bss")
            elif s == "d":
                sections["d"] = SectionInfo(".data")
            elif s == "t":
                sections["t"] = SectionInfo(".text")
            else:
                raise Exception(
                    "Invalid section <%s>, must be a combination"
                    " of [bdt] characters\n" % s
                )
        return sections

    sections = make_sections_dict(sections_letters)

    generator = nm_generator(elf_file_path, use_fast_nm)
    for _, section, symbol_name, filename, line, size in generator:
        if not filename:
            filename = "Unknown"
        if section in sections:
            sections[section].add_entry(symbol_name, filename, size)

    return sections


def nm_generator(elf_path, use_fast_nm=True):
    if use_fast_nm:
        return _nm_generator_fast(elf_path)
    else:
        return _nm_generator_slow(elf_path)


def _get_symbols_table(f):
    # NOTE: nm crashes when we pass in the -l command line option. As a
    # workaround, we use readelf to get the symbol to address mappings and then
    # we use addr2line to get file/lines from the addresses.
    infile = sh.arm_none_eabi_readelf("-s", "-W", f)

    line_pattern = re.compile(
        r"""\s+([0-9]+\:)\s+  # number
                                  ([0-9a-f]+)\s+    # address
                                  ([0-9]+)\s+       # size
                                  (\S+)\s+          # type
                                  (\S+)\s+          # Bind
                                  (\S+)\s+          # Visibility
                                  (\S+)\s+          # Ndx
                                  (\S+)             # symbol name
                                  """,
        flags=re.VERBOSE,
    )

    def create_addr2line_process():
        return subprocess.Popen(
            ["arm-none-eabi-addr2line", "-e", f],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    addr2line = create_addr2line_process()

    symbols = {}
    for line_num, line in enumerate(infile):
        if (line_num % 300) == 0:
            sys.stdout.write(".")
            sys.stdout.flush()

        match = line_pattern.match(line)

        if match is None:
            continue

        type = match.group(4)
        if type not in ["FUNC", "OBJECT"]:
            continue
        addr = match.group(2)
        symbol_name = match.group(8)

        success = False
        while not success:
            try:
                addr2line.stdin.write("0x%s\n" % addr)
                success = True
            except OSError:
                # This happens if the previous iteration caused an error
                addr2line = create_addr2line_process()

        src_file_line = addr2line.stdout.readline().strip()
        if src_file_line:
            # Some Bluetopia paths start with 'C:\...'
            components = src_file_line.split(":")
            src_file = ":".join(components[:-1])
            line = components[-1:][0]
        else:
            (src_file, line) = ("?", "0")
        symbols[symbol_name] = (src_file, line)

    addr2line.kill()

    return symbols


# This method is quite slow, but works around a bug in nm.
def _nm_generator_slow(f):
    print("Getting list of symbols...")
    symbols = _get_symbols_table(f)
    print("Aggregating...")
    infile = sh.arm_none_eabi_nm("-S", f)

    line_pattern = re.compile(
        r"""([0-9a-f]+)\s+ # address
                                  ([0-9a-f]+)\s+ # size
                                  ([dDbBtTrR])\s+ # section type
                                  (\S+) # name
                                  """,
        flags=re.VERBOSE,
    )

    for line in infile:
        match = line_pattern.match(line)

        if match is None:
            continue

        addr = int(match.group(1), 16)
        size = int(match.group(2), 16)
        section = match.group(3).lower()
        if section == "r":
            section = "t"
        symbol_name = match.group(4)
        if symbol_name not in symbols:
            continue
        rel_file_path, line = symbols[symbol_name]
        if rel_file_path:
            rel_file_path = os.path.relpath(rel_file_path)

        yield (addr, section, symbol_name, rel_file_path, line, size)


# This method is much faster, and *should* work, but as of 2014-08-01, we get
# exceptions when we try to run nm -l on the tintin ELF file. So, the
# _nm_generator_slow() method above can be used as a workaround.
def _nm_generator_fast(f):
    """Given a path to an .elf, generates tuples:
    (section, symbol_name, rel_file_path, line, size)
    Note, rel_file_path and line can be None.

    """
    infile = sh.arm_none_eabi_nm("-l", "-S", f)

    for line in infile:
        match = NM_LINE_PATTERN.match(line)

        if match is None:
            continue

        addr = int(match.group(1), 16)
        size = int(match.group(2), 16)

        section = match.group(3).lower()
        if section == "r":
            section = "t"
        symbol_name = match.group(4)

        rel_file_path = match.group(6)
        if rel_file_path:
            rel_file_path = os.path.relpath(rel_file_path)

        line = match.group(7)
        if line:
            line = int(line)

        yield (addr, section, symbol_name, rel_file_path, line, size)


def size(elf_path):
    """Returns size (text, data, bss)"""
    output = subprocess.check_output(["arm-none-eabi-size", elf_path])

    lines = output.decode("utf8").splitlines()
    if len(lines) < 2:
        return 0
    match = re.match(r"^\s*([0-9]+)\s+([0-9]+)\s+([0-9]+)", lines[1])
    if not match:
        return 0
    # text, data, bss
    return (int(match.groups()[0]), int(match.groups()[1]), int(match.groups()[2]))


def strip(elf_path):
    """Strip debug info from specified .elf file"""
    sh.arm_none_eabi_strip(elf_path)


def copy_elf_section(in_elf_path, out_elf_path, section_name_list):
    """Creates out_elf_path containing only sections in 'section name list'"""
    args = []
    for name in section_name_list:
        args.append("-j")
        args.append(name)
    args.append(in_elf_path)
    args.append(out_elf_path)
    sh.arm_none_eabi_objcopy(args)


def section_bytes(elf_path, section_name):
    """Returns the bytes in a section of a given .elf file"""
    with tempfile.NamedTemporaryFile() as temp:
        sh.arm_none_eabi_objcopy(
            ["-j", section_name, "-O", "binary", elf_path, temp.name]
        )
        with open(temp.name, "rb") as f:
            return f.read()
