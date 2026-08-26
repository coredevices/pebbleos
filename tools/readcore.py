#!/usr/bin/env python3

"""
Convert a Pebble core dump to an ELF file.

Try with something like:

$ ./readcore.py blahblah-coredump.bin blahblah-coredump.elf
$ arm-none-eabi-gdb firmware_asterix_v4.9.9-core38.elf blahblah-coredump.elf
...
(gdb) i thre
"""

__author__ = "Joshua Wise <joshua@accelerated.tech>"

import binascii
import json
from enum import Enum

# We use a truly ancient `construct` because it's what libpebble etc need.
# Don't try reading the `construct` readthedocs, the entire API has changed
# since 2.5 to now...
import construct as cs
import elftools.construct as ecs  # Sigh...
from elftools.elf.enums import *
from elftools.elf.structs import ELFStructs


class _CoreDumpChunkKey(Enum):
    RAM = 1
    THREAD = 2
    EXTRA_REG = 3
    MEMORY = 4
    TERMINATOR = 0xFFFFFFFF


_CoreDumpImageHeader = cs.Struct(
    "CoreDumpImageHeader",
    cs.Const(cs.Bytes("signature", 4), b"\xfe\xca\x0d\xf0"),  # 0xF00DCAFE in LE
    cs.ULInt32("version"),  # CORE_ID_MAIN_MCU | version << 8
    cs.ULInt32("timestamp"),
    cs.String("serial_number", 16, padchar="\x00", encoding="utf8"),
    cs.String("build_id", 64, padchar="\x00", encoding="utf8"),
)

_CoreDumpMemoryChunk = cs.Struct(
    "CoreDumpMemoryChunk",
    cs.ULInt32("start"),
    cs.Bytes("data", lambda ctx: ctx._.size - 4),
)

_CoreDumpThreadInfo = cs.Struct(
    "CoreDumpThreadInfo",
    cs.String("name", 16, padchar="\x00", encoding="utf8"),
    cs.ULInt32("id"),
    cs.ULInt8("running"),
    cs.Array(17, cs.ULInt32("regs")),
)

_CoreDumpExtraRegInfo = cs.Struct(
    "CoreDumpExtraRegInfo",
    cs.ULInt32("msp"),
    cs.ULInt32("psp"),
    cs.ULInt8("primask"),
    cs.ULInt8("basepri"),
    cs.ULInt8("faultmask"),
    cs.ULInt8("control"),
)

_CoreDumpChunk = cs.Struct(
    "CoreDumpChunk",
    cs.ULInt32("key"),
    cs.ULInt32("size"),
    cs.Embedded(
        cs.Union(
            "payload",
            # The terminator chunk has no payload. Its `size` field is
            # unreliable: a core dump read straight out of erased flash ends in
            # 0xFF padding, so the terminator's size reads back as 0xFFFFFFFF
            # rather than the 0 the firmware wrote. Force a zero-length payload
            # for the terminator so we don't try to read 4 GiB of nonexistent
            # data.
            cs.Bytes(
                "raw",
                lambda ctx: (
                    0 if ctx.key == _CoreDumpChunkKey.TERMINATOR.value else ctx.size
                ),
            ),
            cs.If(lambda ctx: ctx.key == _CoreDumpChunkKey.RAM.value, cs.Pass),
            cs.If(
                lambda ctx: ctx.key == _CoreDumpChunkKey.THREAD.value,
                cs.Rename("thread", _CoreDumpThreadInfo),
            ),
            cs.If(
                lambda ctx: ctx.key == _CoreDumpChunkKey.EXTRA_REG.value,
                cs.Rename("extra_reg", _CoreDumpExtraRegInfo),
            ),
            cs.If(
                lambda ctx: ctx.key == _CoreDumpChunkKey.MEMORY.value,
                cs.Rename("memory", _CoreDumpMemoryChunk),
            ),
        )
    ),
)

# but you have to know where s_saved_registers is to find this
_CoreDumpSavedRegisters = cs.Struct(
    "CoreDumpSavedRegisters",
    cs.Array(17, cs.ULInt32("regs")),
    cs.Rename("extra_reg", _CoreDumpExtraRegInfo),
)

_CoreDump = cs.Struct(
    "CoreDump",
    cs.Rename("header", _CoreDumpImageHeader),
    cs.Rename(
        "chunks",
        cs.RepeatUntil(
            lambda obj, ctx: obj.key == _CoreDumpChunkKey.TERMINATOR.value,
            _CoreDumpChunk,
        ),
    ),
)

# include/uapi/linux/elfcore.h
_prstatus_arm32 = cs.Struct(
    "prstatus_arm32",
    cs.Struct(
        "siginfo",
        cs.ULInt32("si_signo"),
        cs.ULInt32("si_code"),
        cs.ULInt32("si_errno"),
    ),
    cs.ULInt32("pr_cursig"),  # padded to 32 bits
    cs.ULInt32("pr_sigpend"),
    cs.ULInt32("pr_sighold"),
    cs.ULInt32("pr_pid"),
    cs.ULInt32("pr_ppid"),
    cs.ULInt32("pr_pgrp"),
    cs.ULInt32("pr_sid"),
    cs.Array(2, cs.ULInt32("pr_utime")),
    cs.Array(2, cs.ULInt32("pr_stime")),
    cs.Array(2, cs.ULInt32("pr_cutime")),
    cs.Array(2, cs.ULInt32("pr_cstime")),
    cs.Struct(
        "reg",
        *[
            cs.ULInt32(x)
            for x in [
                "r0",
                "r1",
                "r2",
                "r3",
                "r4",
                "r5",
                "r6",
                "r7",
                "r8",
                "r9",
                "r10",
                "r11",
                "r12",
                "sp",
                "lr",
                "pc",
                "cpsr",
                "orig_r0",
            ]
        ],
    ),
    cs.ULInt32("pr_fpvalid"),
)

_prpsinfo_arm32 = cs.Struct(
    "prpsinfo_arm32",
    cs.ULInt8("pr_state"),
    cs.String("pr_sname", 1),
    cs.ULInt8("pr_zomb"),
    cs.ULInt8("pr_nice"),
    cs.ULInt32("pr_flag"),
    cs.ULInt16("pr_uid"),
    cs.ULInt16("pr_gid"),
    cs.ULInt32("pr_pid"),
    cs.ULInt32("pr_ppid"),
    cs.ULInt32("pr_pgrp"),
    cs.ULInt32("pr_sid"),
    cs.String("pr_fname", 16),
    cs.String("pr_psargs", 80),
)


class Coredump:
    def __init__(self, file=None):
        if type(file) == str:
            with open(file, "rb") as f:
                self.__init__(f)
            return

        self.raw = _CoreDump.parse_stream(file)

        self.timestamp = self.raw.header.timestamp
        self.serial_number = self.raw.header.serial_number
        self.build_id = self.raw.header.build_id
        self.memory = [x.memory for x in self.raw.chunks if x.memory]
        self.threads = [x.thread for x in self.raw.chunks if x.thread]

    def __str__(self):
        s = f"Pebble core dump, timestamp {self.timestamp}, from watch {self.serial_number} running build {self.build_id}; {len(self.memory)} chunks of memory ("
        s += "; ".join([f"{len(m.data)} bytes @ 0x{m.start:08x}" for m in self.memory])
        s += f"); {len(self.threads)} threads ("
        s += "; ".join([t.name for t in self.threads])
        s += ")"
        return s

    def write_elf(self, file=None):
        if type(file) == str:
            with open(file, "wb") as f:
                self.write_elf(f)
            return

        structs = ELFStructs()
        structs.create_basic_structs()
        structs.create_advanced_structs(
            e_type=ENUM_E_TYPE["ET_CORE"], e_machine=ENUM_E_MACHINE["EM_ARM"]
        )

        # We want to do this all in one pass: figure out where everythign is
        # going to go, then update some internal variables with where things
        # actually ended up, and then actually generate the strcuts and
        # splatter them out later.  `position` is the thing that measures
        # where we expect to put bytes, and `callbacks` will actually go
        # generate the bytes later.
        position = 0
        callbacks = []

        phoff = 0
        shoff = 0

        phdrs = []
        shdrs = []

        ### ELF header.
        def mk_ehdr():
            return structs.Elf_Ehdr.build(
                ecs.Container(
                    e_ident=ecs.Container(
                        EI_MAG=b"\x7fELF",
                        EI_CLASS="ELFCLASS32",
                        EI_DATA="ELFDATA2LSB",
                        EI_VERSION="EV_CURRENT",
                        EI_OSABI="ELFOSABI_SYSV",  # 'ELFOSABI_ARM_AEABI' sounds better but pebbleos is sysv...
                        EI_ABIVERSION=0,
                    ),
                    e_type="ET_CORE",
                    e_machine="EM_ARM",
                    e_version="EV_CURRENT",
                    e_entry=0,
                    e_phoff=phoff,
                    e_shoff=shoff,
                    e_flags=0x5000200,
                    e_ehsize=structs.Elf_Ehdr.sizeof(),
                    e_phentsize=structs.Elf_Phdr.sizeof(),
                    e_phnum=len(phdrs),
                    e_shentsize=structs.Elf_Shdr.sizeof(),
                    e_shnum=len(shdrs),
                    e_shstrndx=1,
                )
            )

        position += structs.Elf_Ehdr.sizeof()
        callbacks.append(mk_ehdr)

        ### Boilerplate ELF sections.
        shdrs.append(
            ecs.Container(
                sh_name=0,
                sh_type="SHT_NULL",
                sh_flags=0,
                sh_addr=0,
                sh_offset=0,
                sh_size=0,
                sh_link=0,
                sh_info=0,
                sh_addralign=0,
                sh_entsize=0,
            )
        )

        strings = b"\0shstrtab\0note0\0load\0\0"

        shdrs.append(
            ecs.Container(
                sh_name=1,
                sh_type="SHT_STRTAB",
                sh_flags=0,
                sh_addr=0,
                sh_offset=position,
                sh_size=len(strings),
                sh_link=0,
                sh_info=0,
                sh_addralign=0,
                sh_entsize=0,
            )
        )
        position += len(strings)
        callbacks.append(lambda: strings)

        ### Generate ELF notes.
        notes = b""

        # add a build-ID tag that matches the theorized build-id from the ELF we are debugging
        build_id = binascii.unhexlify(self.build_id)
        notes += structs.Elf_Nhdr.build(
            ecs.Container(
                n_namesz=4,
                n_descsz=len(build_id),
                n_type=3,  # NT_GNU_BUILD_ID
            )
        )
        notes += b"GNU\0"
        notes += build_id

        # add some pebble notes
        pebble_notes = json.dumps(
            {"timestamp": self.timestamp, "serial_number": self.serial_number}
        ).encode()
        notes += structs.Elf_Nhdr.build(
            ecs.Container(
                n_namesz=7,
                n_descsz=len(pebble_notes),
                n_type=0,
            )
        )

        if len(pebble_notes) % 4 != 0:
            pebble_notes += b"\x00" * (4 - len(pebble_notes) % 4)

        notes += b"Pebble\00\00"  # pad to 4
        notes += pebble_notes

        for t in self.threads:
            print(f"  thread {t.name} ({t.id}) running {t.running}")

            pr_status = _prstatus_arm32.build(
                cs.Container(
                    siginfo=cs.Container(
                        si_signo=6,  # SIGABRT
                        si_code=0,
                        si_errno=0,
                    ),
                    pr_cursig=6,  # SIGABRT
                    pr_sigpend=0,
                    pr_sighold=0,
                    pr_pid=t.id,
                    pr_ppid=0,
                    pr_pgrp=1,
                    pr_sid=1,
                    pr_utime=[0, 0],
                    pr_stime=[0, 0],
                    pr_cutime=[0, 0],
                    pr_cstime=[0, 0],
                    reg=cs.Container(
                        r0=t.regs[0],
                        r1=t.regs[1],
                        r2=t.regs[2],
                        r3=t.regs[3],
                        r4=t.regs[4],
                        r5=t.regs[5],
                        r6=t.regs[6],
                        r7=t.regs[7],
                        r8=t.regs[8],
                        r9=t.regs[9],
                        r10=t.regs[10],
                        r11=t.regs[11],
                        r12=t.regs[12],
                        sp=t.regs[13],
                        lr=t.regs[14],
                        pc=t.regs[15],
                        cpsr=t.regs[16],
                        orig_r0=0,
                    ),
                    pr_fpvalid=0,
                )
            )
            notes += structs.Elf_Nhdr.build(
                ecs.Container(
                    n_namesz=5,
                    n_descsz=len(pr_status),
                    n_type=ENUM_CORE_NOTE_N_TYPE["NT_PRSTATUS"],
                )
            )
            notes += b"CORE\0\0\0\0"  # 4-byte align
            notes += pr_status

            def zero_pad(bs, l):
                return bs + b"\x00" * (l - len(bs))

            prpsinfo = _prpsinfo_arm32.build(
                cs.Container(
                    pr_state=0,
                    pr_sname=b"R" if t.running else b"S",
                    pr_zomb=0,
                    pr_nice=0,
                    pr_flag=0,
                    pr_uid=0,
                    pr_gid=0,
                    pr_pid=t.id,
                    pr_ppid=0,
                    pr_pgrp=1,
                    pr_sid=1,
                    pr_fname=zero_pad(b"pebbleos.elf", 16),
                    pr_psargs=zero_pad(t.name.encode(), 80),
                )
            )
            notes += structs.Elf_Nhdr.build(
                ecs.Container(
                    n_namesz=5,
                    n_descsz=len(prpsinfo),
                    n_type=ENUM_CORE_NOTE_N_TYPE["NT_PRPSINFO"],
                )
            )
            notes += b"CORE\0\0\0\0"  # 4-byte align
            notes += prpsinfo

        # Now write the notes out.
        shdrs.append(
            ecs.Container(
                sh_name=10,
                sh_type="SHT_NOTE",
                sh_flags=0x2,  # alloc
                sh_addr=0,
                sh_offset=position,
                sh_size=len(notes),
                sh_link=0,
                sh_info=0,
                sh_addralign=1,
                sh_entsize=0,
            )
        )

        phdrs.append(
            ecs.Container(
                p_type="PT_NOTE",
                p_offset=position,
                p_vaddr=0,
                p_paddr=0,
                p_filesz=len(notes),
                p_memsz=0,
                p_flags=7,  # RWX
                p_align=1,
            )
        )

        callbacks.append(lambda: notes)
        position += len(notes)

        ### Memory sections.
        for m in self.memory:
            phdrs.append(
                ecs.Container(
                    p_type="PT_LOAD",
                    p_offset=position,
                    p_vaddr=m.start,
                    p_paddr=0,
                    p_filesz=len(m.data),
                    p_memsz=len(m.data),
                    p_flags=7,  # RWX
                    p_align=1,
                )
            )

            shdrs.append(
                ecs.Container(
                    sh_name=16,
                    sh_type="SHT_PROGBITS",
                    sh_flags=0x7,  # write | alloc | exec
                    sh_addr=m.start,
                    sh_offset=position,
                    sh_size=len(m.data),
                    sh_link=0,
                    sh_info=0,
                    sh_addralign=1,
                    sh_entsize=0,
                )
            )

            callbacks.append(
                (lambda data: lambda: data)(m.data)
            )  # avoid capturing m accidentally
            position += len(m.data)

        ### Actually write out program and section headers...
        phoff = position
        for phdr in phdrs:
            d = structs.Elf_Phdr.build(phdr)
            callbacks.append((lambda data: lambda: data)(d))
            position += len(d)

        shoff = position
        for shdr in shdrs:
            d = structs.Elf_Shdr.build(shdr)
            callbacks.append((lambda data: lambda: data)(d))
            position += len(d)

        ### ... then write it all out to disk.
        for cb in callbacks:
            file.write(cb())


if __name__ == "__main__":
    import sys

    cd = Coredump(sys.argv[1])
    print(cd)
    cd.write_elf(sys.argv[2])
