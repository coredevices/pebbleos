#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Decode the LCDC interrupt ring buffer from a coredump.

display_jdi.c records every LCDC interrupt into the BSS ring buffer
`s_lcdc_irq_log` (see display_jdi_irq_handler), which is captured in
coredumps. This tool pulls it out and decodes it, so the interrupt
interleaving behind the JDI-parallel silent-loss crash (a lost EOF
completion) can be read post-mortem.

The smoking gun: an interrupt whose `irq_before` has EOF_STAT set but whose
EOF-callback-fired flag is clear -- the HAL saw EOF pending and never reached
the completion path.

Both the firmware ELF and the coredump are standard ELF32 files, parsed here
directly with no toolchain dependency.

Usage:
  tools/extract_irq_log.py --elf firmware.elf --core coredump.elf
"""

import argparse
import struct
import sys

SYMBOL = "s_lcdc_irq_log"
LOG_ENTRIES = 32

# LCD_IF.IRQ bits (sf32lb52x lcd_if.h). 0-6 latched STAT, 16-22 raw STAT.
IRQ_BITS = {
    0: "EOF",
    1: "ICB_OF",
    2: "DPIL_INTR",
    3: "DPI_UDR",
    4: "JDI_PARL_INTR",
    5: "JDI_PAR_UDR",
    6: "LINE_DONE",
    16: "EOF~",
    17: "ICB_OF~",
    18: "DPIL_INTR~",
    19: "DPI_UDR~",
    20: "JDI_PARL_INTR~",
    21: "JDI_PAR_UDR~",
    22: "LINE_DONE~",
}
IRQ_EOF_STAT = 1 << 0

# LCD_IF.STATUS bits.
STATUS_BITS = {0: "LCD_BUSY", 1: "DPI_RUN", 2: "JDI_PAR_RUN"}

# JDI_PAR_CTRL fields.
JDI_CTRL_ENABLE = 1 << 0
JDI_CTRL_XRST = 1 << 4
JDI_CTRL_INT_LINE_NUM_POS = 16
JDI_CTRL_INT_LINE_NUM_MSK = 0xFFFF << 16

# DisplayIrqLogEntry.flags bits.
FLAG_EOF_CB_FIRED = 1 << 0
FLAG_UPDATING = 1 << 1


def decode_bits(value, names):
    return "|".join(n for b, n in sorted(names.items()) if value & (1 << b)) or "-"


def find_symbol(elf_path, name):
    """Return (vaddr, size) for `name` from an ELF32 LE symbol table."""
    with open(elf_path, "rb") as f:
        data = f.read()
    if data[:6] != b"\x7fELF\x01\x01":
        sys.exit(f"{elf_path}: not an ELF32 little-endian file")

    e_shoff = struct.unpack_from("<I", data, 0x20)[0]
    e_shentsize = struct.unpack_from("<H", data, 0x2E)[0]
    e_shnum = struct.unpack_from("<H", data, 0x30)[0]

    symtab = None
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        if struct.unpack_from("<I", data, off + 4)[0] == 2:  # SHT_SYMTAB
            symtab = (
                struct.unpack_from("<I", data, off + 16)[0],
                struct.unpack_from("<I", data, off + 20)[0],
                struct.unpack_from("<I", data, off + 24)[0],
            )
    if symtab is None:
        sys.exit(f"{elf_path}: no symbol table (stripped ELF?)")

    sym_off, sym_size, link = symtab
    str_off = struct.unpack_from("<I", data, e_shoff + link * e_shentsize + 16)[0]
    for off in range(sym_off, sym_off + sym_size, 16):
        st_name = struct.unpack_from("<I", data, off)[0]
        st_value = struct.unpack_from("<I", data, off + 4)[0]
        st_size = struct.unpack_from("<I", data, off + 8)[0]
        end = data.index(b"\0", str_off + st_name)
        if data[str_off + st_name : end].decode("ascii", "replace") == name:
            return st_value, st_size
    return None


def read_core_memory(core_path, vaddr, size):
    """Extract `size` bytes at virtual address `vaddr` from an ELF core file."""
    with open(core_path, "rb") as f:
        data = f.read()
    if data[:4] != b"\x7fELF":
        sys.exit(f"{core_path}: not an ELF file")

    e_phoff = struct.unpack_from("<I", data, 0x1C)[0]
    e_phentsize = struct.unpack_from("<H", data, 0x2A)[0]
    e_phnum = struct.unpack_from("<H", data, 0x2C)[0]
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        p_type, p_offset, p_vaddr = struct.unpack_from("<3I", data, off)
        p_filesz = struct.unpack_from("<I", data, off + 16)[0]
        if p_type != 1:  # PT_LOAD
            continue
        if p_vaddr <= vaddr and vaddr + size <= p_vaddr + p_filesz:
            start = p_offset + (vaddr - p_vaddr)
            return data[start : start + size]
    return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--elf", required=True, help="firmware ELF (with symbols) matching the coredump"
    )
    ap.add_argument("--core", required=True, help="coredump ELF core file")
    args = ap.parse_args()

    sym = find_symbol(args.elf, SYMBOL)
    if sym is None:
        sys.exit(
            f"symbol '{SYMBOL}' not found in {args.elf}\n"
            f"(firmware predates the LCDC IRQ-log instrumentation?)"
        )
    vaddr, size = sym

    raw = read_core_memory(args.core, vaddr, size)
    if raw is None:
        sys.exit(
            f"'{SYMBOL}' (0x{vaddr:08x}, {size} bytes) not present in "
            f"{args.core} -- the coredump may not carry this region"
        )

    write_count = struct.unpack_from("<I", raw, 0)[0]
    entries = []
    for i in range(LOG_ENTRIES):
        base = 4 + i * 24
        if base + 24 > len(raw):
            break
        ts, irq_b, irq_a, jdi, status, flags = struct.unpack_from("<6I", raw, base)
        entries.append((ts, irq_b, irq_a, jdi, status, flags))

    # Order oldest -> newest. write_count is the total ever logged; the slot
    # written next is write_count % N, so the oldest live entry is there too
    # once the ring has wrapped.
    if write_count <= LOG_ENTRIES:
        order = list(range(write_count))
    else:
        order = [(write_count + i) % LOG_ENTRIES for i in range(LOG_ENTRIES)]

    print(f"LCDC interrupt log  ({SYMBOL} @ 0x{vaddr:08x})")
    print(f"firmware : {args.elf}")
    print(f"coredump : {args.core}")
    print(f"write_count = {write_count}  ({len(order)} live entries, oldest first)")
    print("=" * 78)

    prev_ts = None
    suspects = 0
    for n, idx in enumerate(order):
        ts, irq_b, irq_a, jdi, status, flags = entries[idx]
        delta = "" if prev_ts is None else f"+{(ts - prev_ts) & 0xFFFFFFFF}"
        prev_ts = ts
        int_line = (jdi & JDI_CTRL_INT_LINE_NUM_MSK) >> JDI_CTRL_INT_LINE_NUM_POS
        eof_pending = bool(irq_b & IRQ_EOF_STAT)
        eof_served = bool(flags & FLAG_EOF_CB_FIRED)
        lost = eof_pending and not eof_served

        print(f"\n[{n:2d}] t={ts} {delta:<8} {'<-- EOF LOST' if lost else ''}")
        print(f"     irq_before = 0x{irq_b:08x}  {decode_bits(irq_b, IRQ_BITS)}")
        print(f"     irq_after  = 0x{irq_a:08x}  {decode_bits(irq_a, IRQ_BITS)}")
        print(f"     status     = 0x{status:08x}  {decode_bits(status, STATUS_BITS)}")
        ctrl = []
        if jdi & JDI_CTRL_ENABLE:
            ctrl.append("ENABLE")
        if jdi & JDI_CTRL_XRST:
            ctrl.append("XRST")
        print(
            f"     jdi_par_ctrl=0x{jdi:08x}  INT_LINE_NUM={int_line} {'|'.join(ctrl)}"
        )
        print(
            f"     eof_cb_fired={int(eof_served)}  "
            f"updating={int(bool(flags & FLAG_UPDATING))}"
        )
        if lost:
            suspects += 1
            print(
                "     >>> EOF_STAT was set entering the handler but the "
                "completion callback never fired."
            )

    print("\n" + "=" * 78)
    if suspects:
        print(
            f"{suspects} interrupt(s) saw EOF pending without servicing it "
            f"-- lost-EOF race CONFIRMED."
        )
    else:
        print(
            "No interrupt shows EOF pending-but-unserved. Either the wedge "
            "is elsewhere, or the losing IRQ aged out of the 32-entry ring."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
