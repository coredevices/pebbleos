#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
GDB server proxy for the QEMU emulator running a Pebble machine.

This proxy sits between gdb and the gdb server implemented in QEMU. Its primary purpose is to
implement support for the "info threads" and related gdb commands. The QEMU gdb server is not
thread aware and knows nothing about the kernel's threads.

The proxy talks to the QEMU gdb server with primitive remote-protocol commands, walks the kernel's
thread list (pbl_all_threads) and decodes each thread's saved context using the layout descriptor
the kernel exports (pbl_kernel_debug_layout), then answers gdb's thread queries itself. Every other
request is passed through to QEMU untouched.

This module runs as a separate process from both QEMU and gdb: it connects to the gdb socket QEMU
created and accepts a connection from gdb. It exits when the QEMU connection closes.
"""

import argparse
import binascii
import logging
import select
import socket
import struct
import time
from typing import ClassVar

logger = logging.getLogger(__name__)

CTRL_C_CHARACTER = b"\3"

# QEMU's one and only thread id; kept for the running thread so gdb does not think it died.
QEMU_CURRENT_THREAD_ID = 1

# Register order QEMU's M-profile description uses for the 'g' packet.
REG_NAMES = [
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
    "xpsr",
]

THREAD_STATES = ["Ready", "Running", "Blocked", "Suspended", "Dead"]

EXC_RETURN_FP_INACTIVE = 0x10
XPSR_STACK_ALIGN = 1 << 9
HW_FRAME_WORDS = 8
HW_FRAME_FP_WORDS = 18  # s0-s15, fpscr, reserved


class QemuGdbError(Exception):
    pass


def byte_swap_uint32(val):
    return int.from_bytes(val.to_bytes(4, "little"), "big")


class KernelLayout:
    """The pbl_kernel_debug_layout descriptor: offsets into struct pbl_thread and the saved
    context, so this file needs no knowledge of either struct."""

    FIELDS: ClassVar = [
        "version",
        "thread_sp",
        "thread_all_next",
        "thread_state",
        "thread_id",
        "thread_name",
        "thread_name_len",
        "ctx_control",
        "ctx_r4",
        "ctx_exc_return",
        "ctx_hw",
        "ctx_fp_extra",
    ]
    SIZE = len(FIELDS) * 2

    def __init__(self, raw):
        values = struct.unpack(f"<{len(self.FIELDS)}H", raw)
        for name, value in zip(self.FIELDS, values):
            setattr(self, name, value)
        if self.version != 1:
            raise QemuGdbError(
                f"Unsupported pbl_kernel_debug_layout version {self.version}"
            )


class PebbleThread:
    def __init__(self, id, ptr, running, name, state, registers):
        self.id = id
        self.ptr = ptr
        self.running = running
        self.name = name
        self.state = state
        self.registers = registers  # None for the running thread: QEMU owns those

    def __repr__(self):
        return (
            f"<Thread id:{self.id:d} ptr:0x{self.ptr:08X} running:{self.running!r} "
            f"name:{self.name} state:{self.state} registers:{self.registers!r}>"
        )


class QemuGdbProxy:
    """A gdb server listening on a port that forwards to QEMU's gdb server and answers thread
    queries from the kernel's data structures."""

    SYMBOLS: ClassVar = ["pbl_cur", "pbl_all_threads", "pbl_kernel_debug_layout"]

    def __init__(self, port, target_host, target_port, connect_timeout):
        self.target_host = target_host
        self.target_port = target_port
        self.target_socket = None
        self.connect_timeout = connect_timeout

        self.client_accept_socket = None
        self.client_accept_port = port
        self.client_conn_socket = None

        self.packet_size = 4096
        self.active_thread_id = QEMU_CURRENT_THREAD_ID
        self.threads = {}

        self.multiprocess = False  # gdb used 'p<pid>.<tid>' thread ids
        self.symbols = dict.fromkeys(self.SYMBOLS)
        self.got_all_symbols = False
        self.layout = None

    # ---- sockets -------------------------------------------------------------------------

    def _fetch_socket_data(self, timeout=None):
        """Block until the target or the client has data. A closed target ends the proxy; a
        closed client puts us back to waiting for a connection."""
        target_data = b""
        client_data = b""

        while not target_data and not client_data:
            if self.client_conn_socket is not None:
                read_list = [self.target_socket, self.client_conn_socket]
            else:
                read_list = [self.target_socket, self.client_accept_socket]

            readable, _, _ = select.select(read_list, [], [], timeout)
            if not readable:
                break

            if self.target_socket in readable:
                target_data = self.target_socket.recv(self.packet_size)
                if not target_data:
                    raise QemuGdbError("target system disconnected")
                logger.debug("got target data: %r", target_data)

            if self.client_conn_socket is not None:
                if self.client_conn_socket in readable:
                    client_data = self.client_conn_socket.recv(self.packet_size)
                    if not client_data:
                        logger.info("client connection closed")
                        self.client_conn_socket.close()
                        self.client_conn_socket = None
                    logger.debug("got client data: %r", client_data)
            elif self.client_accept_socket in readable:
                self.client_conn_socket, _ = self.client_accept_socket.accept()
                logger.info("Connected to client")

        return (target_data, client_data)

    @staticmethod
    def _create_packet(data):
        return b"$%s#%02X" % (data, sum(data) % 256)

    # ---- target access ---------------------------------------------------------------------

    def _target_read_memory(self, address, length):
        self.target_socket.send(self._create_packet(b"m%08X,%X" % (address, length)))
        data = b""
        while True:
            chunk = self.target_socket.recv(self.packet_size)
            if not chunk:
                raise QemuGdbError("target system disconnected")
            data += chunk
            if b"$" in data and b"#" in data:
                break
        resp = data.split(b"$", 1)[1].split(b"#", 1)[0]
        if resp.startswith(b"E"):
            raise QemuGdbError(f"Error reading 0x{address:08X}: {resp!r}")
        return binascii.unhexlify(resp)

    def _target_read_uint32(self, address):
        return struct.unpack("<I", self._target_read_memory(address, 4))[0]

    def _target_read_uint8(self, address):
        return self._target_read_memory(address, 1)[0]

    def _target_read_cstr(self, address, max_len):
        raw = self._target_read_memory(address, max_len)
        return raw.split(b"\0", 1)[0].decode("ascii", "replace")

    # ---- kernel thread list ----------------------------------------------------------------

    def _decode_saved_context(self, sp):
        """Registers of a thread that is switched out, from the context the kernel saved on its
        stack. Returns a list in REG_NAMES order."""
        lay = self.layout
        head = self._target_read_memory(sp, lay.ctx_hw)
        words = struct.unpack(f"<{lay.ctx_hw // 4}I", head)
        exc_return = words[lay.ctx_exc_return // 4]
        fp = (exc_return & EXC_RETURN_FP_INACTIVE) == 0

        hw = sp + lay.ctx_hw + (lay.ctx_fp_extra if fp else 0)
        frame = struct.unpack("<8I", self._target_read_memory(hw, HW_FRAME_WORDS * 4))
        r0, r1, r2, r3, r12, lr, pc, xpsr = frame

        frame_words = HW_FRAME_WORDS + (HW_FRAME_FP_WORDS if fp else 0)
        sp_after = hw + frame_words * 4
        if xpsr & XPSR_STACK_ALIGN:
            sp_after += 4

        regs = [0] * len(REG_NAMES)
        regs[0:4] = [r0, r1, r2, r3]
        regs[4:12] = words[lay.ctx_r4 // 4 : lay.ctx_r4 // 4 + 8]
        regs[12] = r12
        regs[13] = sp_after
        regs[14] = lr
        regs[15] = pc
        regs[16] = xpsr
        return regs

    def _target_collect_thread_info(self):
        if self.layout is None:
            raw = self._target_read_memory(
                self.symbols["pbl_kernel_debug_layout"], KernelLayout.SIZE
            )
            self.layout = KernelLayout(raw)
        lay = self.layout

        current = self._target_read_uint32(self.symbols["pbl_cur"])
        self.threads = {}

        ptr = self._target_read_uint32(self.symbols["pbl_all_threads"])
        seen = set()
        while ptr and ptr not in seen:
            seen.add(ptr)
            state = self._target_read_uint8(ptr + lay.thread_state)
            state_name = (
                THREAD_STATES[state] if state < len(THREAD_STATES) else f"state {state}"
            )
            name = self._target_read_cstr(ptr + lay.thread_name, lay.thread_name_len)
            running = ptr == current

            if running:
                thread_id, registers = QEMU_CURRENT_THREAD_ID, None
            else:
                thread_id = ptr
                registers = self._decode_saved_context(
                    self._target_read_uint32(ptr + lay.thread_sp)
                )

            thread = PebbleThread(thread_id, ptr, running, name, state_name, registers)
            self.threads[thread_id] = thread
            logger.debug("Got thread info: %r", thread)
            ptr = self._target_read_uint32(ptr + lay.thread_all_next)

    def _active_thread(self):
        thread = self.threads.get(self.active_thread_id)
        if thread is None or thread.registers is None:
            raise QemuGdbError(f"No saved registers for thread {self.active_thread_id}")
        return thread

    # ---- gdb requests ----------------------------------------------------------------------

    def _parse_tid(self, data):
        """A thread id, plain ('1', '-1') or in multiprocess form ('p1.1', 'p-1.-1')."""
        if data.startswith(b"p"):
            self.multiprocess = True
            data = data[1:].split(b".")[-1]
        return int(data, 16)

    def _format_tid(self, tid):
        return b"p1.%x" % tid if self.multiprocess else b"%x" % tid

    def _handle_set_active_thread_req(self, data):
        num = self._parse_tid(data)
        if num == -1:
            return self._create_packet(b"OK")
        self.active_thread_id = QEMU_CURRENT_THREAD_ID if num == 0 else num
        return self._create_packet(b"OK")

    def _handle_continue_req(self, msg):
        """'vCont[;action[:thread-id]]...': QEMU only knows thread 1, so strip the thread ids."""
        if b";" not in msg:
            return None
        action = msg.split(b";")[1].split(b":")[0]
        self.target_socket.send(self._create_packet(b"vCont;" + action))
        self.active_thread_id = QEMU_CURRENT_THREAD_ID
        return b""

    def _handle_thread_is_alive_req(self, data):
        num = self._parse_tid(data)
        if num in (-1, 0) or num in self.threads:
            return self._create_packet(b"OK")
        return self._create_packet(b"E22")

    def _handle_get_all_registers_req(self):
        thread = self._active_thread()
        return self._create_packet(
            b"".join(b"%08X" % byte_swap_uint32(v) for v in thread.registers)
        )

    def _handle_query_req(self, msg):
        query = msg.split(b":")
        logger.debug("GDB query: %r", query)

        if query[0] == b"C":
            return self._create_packet(b"QC" + self._format_tid(self.active_thread_id))

        if query[0] == b"fThreadInfo":
            if not self.got_all_symbols:
                return self._create_packet(b"l")
            self._target_collect_thread_info()
            # If the running thread comes first, gdb's first "info threads" shows only it.
            ids = sorted(self.threads.keys(), reverse=True)
            return self._create_packet(
                b"m" + b",".join(self._format_tid(i) for i in ids)
            )

        if query[0] == b"sThreadInfo":
            return self._create_packet(b"l")

        if query[0].startswith(b"ThreadExtraInfo"):
            thread_id = self._parse_tid(query[0].split(b",")[1])
            thread = self.threads.get(thread_id)
            if thread is None:
                text = f"<INVALID THREAD ID: {thread_id:d}>"
            else:
                text = f"{thread.name} 0x{thread.ptr:08X}: {thread.state}"
            return self._create_packet(binascii.hexlify(text.encode()))

        if query[0] == b"Symbol":
            # 'qSymbol:<value>:<name>' answers our lookup; 'qSymbol::' opens the exchange.
            if len(query) >= 3 and query[2]:
                name = binascii.unhexlify(query[2]).decode()
                if query[1]:
                    self.symbols[name] = int(query[1], 16)
                    logger.debug("Symbol %s = 0x%08x", name, self.symbols[name])
                else:
                    logger.warning("gdb could not find symbol %s", name)
                    self.symbols[name] = 0
            pending = next((n for n, v in self.symbols.items() if v is None), None)
            if pending is not None:
                return self._create_packet(
                    b"qSymbol:" + binascii.hexlify(pending.encode())
                )
            self.got_all_symbols = all(self.symbols.values())
            if not self.got_all_symbols:
                logger.warning("kernel symbols missing; thread awareness disabled")
            return self._create_packet(b"OK")

        return None

    def _handle_request(self, msg):
        """Returns a response to send to gdb, b"" if already handled, or None to pass the request
        through to QEMU."""
        msg = msg.split(b"#")[0]
        body = msg[1:]

        if body.startswith(b"q"):
            return self._handle_query_req(body[1:])

        if body.startswith(b"H"):
            if body[1:2] == b"c":
                return None
            return self._handle_set_active_thread_req(body[2:])

        if body.startswith(b"T"):
            return self._handle_thread_is_alive_req(body[1:])

        if self.active_thread_id == QEMU_CURRENT_THREAD_ID:
            pass_through = True
        else:
            pass_through = self.active_thread_id not in self.threads

        if body.startswith(b"g") and not pass_through:
            return self._handle_get_all_registers_req()

        if body.startswith(b"p") and not pass_through:
            reg = int(body[1:], 16)
            if reg >= len(REG_NAMES):
                return None
            return self._create_packet(
                b"%08X" % byte_swap_uint32(self._active_thread().registers[reg])
            )

        if body.startswith(b"P") and not pass_through:
            reg, value = body[1:].split(b"=")
            reg = int(reg, 16)
            if reg < len(REG_NAMES):
                logger.warning(
                    "register writes to a switched-out thread are not written back"
                )
                self._active_thread().registers[reg] = byte_swap_uint32(int(value, 16))
            return self._create_packet(b"OK")

        if body.startswith(b"vCont;"):
            return self._handle_continue_req(body)

        return None

    # ---- main loop -------------------------------------------------------------------------

    def _connect_target(self):
        logger.info(
            "Connecting to target system on %s:%d", self.target_host, self.target_port
        )
        start = time.time()
        while True:
            self.target_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.target_socket.connect((self.target_host, self.target_port))
                break
            except OSError:
                self.target_socket.close()
                if time.time() - start >= self.connect_timeout:
                    raise QemuGdbError(
                        f"Unable to connect to target system on {self.target_host}:"
                        f"{self.target_port}. Is the emulator running?"
                    ) from None
                time.sleep(0.1)
        logger.info(
            "Connected to target system on %s:%d", self.target_host, self.target_port
        )

    def run(self):
        self._connect_target()

        self.client_accept_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_accept_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_accept_socket.bind(("", self.client_accept_port))
        self.client_accept_socket.listen(5)

        # Drop anything the target sent unsolicited.
        self._fetch_socket_data(timeout=0.1)

        data = b""
        while True:
            while True:
                target_data, client_data = self._fetch_socket_data()
                if target_data and self.client_conn_socket is not None:
                    self.client_conn_socket.send(target_data)
                if CTRL_C_CHARACTER in client_data:
                    self.target_socket.send(CTRL_C_CHARACTER)
                    client_data = client_data[client_data.index(CTRL_C_CHARACTER) + 1 :]
                data += client_data
                if b"$" in data and b"#" in data:
                    break

            while b"$" in data and b"#" in data:
                data = data[data.index(b"$") :]
                end = data.index(b"#") + 3
                packet, data = data[:end], data[end:]
                logger.debug("GDB request: %r", packet)

                try:
                    resp = self._handle_request(packet)
                except QemuGdbError as e:
                    logger.error("%s", e)
                    resp = self._create_packet(b"E01")
                except Exception:
                    logger.exception("failed to handle %r", packet)
                    resp = self._create_packet(b"E01")

                if resp is None:
                    self.target_socket.send(packet)
                elif resp != b"":
                    logger.debug("proxy response: %r", resp)
                    self.client_conn_socket.send(b"+" + resp)
                    target_data, client_data = self._fetch_socket_data()
                    if target_data:
                        self.client_conn_socket.send(target_data)
                    if client_data[:1] != b"+":
                        logger.debug("gdb client did not ack")
                    data += client_data[1:]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--port", type=int, default=1233, help="Port to accept incoming connections on"
    )
    parser.add_argument(
        "--target", default="localhost:1234", help="target to connect to"
    )
    parser.add_argument(
        "--connect_timeout",
        type=float,
        default=1.0,
        help="give up if we can't connect to the target within this timeout (sec)",
    )
    parser.add_argument("--debug", action="store_true", help="Turn on debug logging")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO)

    target_host, target_port = args.target.split(":")
    QemuGdbProxy(
        port=args.port,
        target_host=target_host,
        target_port=int(target_port),
        connect_timeout=args.connect_timeout,
    ).run()
