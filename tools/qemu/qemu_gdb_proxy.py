#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
GDB server proxy for the QEMU emulator running a Pebble machine.

This proxy sits between gdb and the gdb server implemented in QEMU. Its primary purpose is to
implement support for the "info threads" and related gdb commands. The QEMU gdb server is not thread
aware and doesn't have any FreeRTOS knowledge such that it can figure out the FreeRTOS threads
created in the Pebble.

This proxy talks to the QEMU gdb server using primitive gdb remote commands and inspects the
FreeRTOS task structures to figure out which threads have been created, their saved registers, etc.
and then returns that information to gdb when it asks for thread info from the target system. For
most other requests recevied from gdb, this proxy simply acts as a passive pass thru to the QEMU gdb
server.

This module is designed to be run as a separate process from both QEMU and gdb. It connects to the
gdb socket created by QEMU and accepts connections from gdb. The intent is that this module would
be launched whenever QEMU is launched and likewise taken down whenever QEMU exits. To support this,
we exit this process whenever we detect that the QEMU gdb server connection has closed.
"""

import argparse
import logging
import select
import socket
import time
from typing import ClassVar

logger = logging.getLogger(__name__)

CTRL_C_CHARACTER = b"\3"


##########################################################################################
class QemuGdbError(Exception):
    pass


##########################################################################################
def byte_swap_uint32(val):
    """Return a byte-swapped 32-bit value"""
    return (
        ((val & 0xFF000000) >> 24)
        | ((val & 0x00FF0000) >> 8)
        | ((val & 0x0000FF00) << 8)
        | ((val & 0x000000FF) << 24)
    )


##########################################################################################
class PebbleThread:
    """This class encapsulates the information about a thread on the Pebble"""

    # Mapping of register name to register index
    reg_name_to_index: ClassVar = {
        name: num
        for num, name in enumerate(
            ["r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc", "xpsr"]
        )
    }

    # Offset of each register on the thread's stack
    # stack_offset -> register_index
    stack_offset_to_reg_index_v2: ClassVar = [  # Used in Snowy, Cortex-M4
        (0x28, 0),  # r0
        (0x2C, 1),  # r1
        (0x30, 2),  # r2
        (0x34, 3),  # r3
        (0x04, 4),  # r4
        (0x08, 5),  # r5
        (0x0C, 6),  # r6
        (0x10, 7),  # r7
        (0x14, 8),  # r8
        (0x18, 9),  # r9
        (0x1C, 10),  # r10
        (0x20, 11),  # r11
        (0x38, 12),  # r12
        (0x3C, 14),  # lr
        (0x40, 15),  # pc
        (0x44, 16),  # xpsr
    ]
    thread_state_size_v2 = 0x48

    stack_offset_to_reg_index_v3: ClassVar = [  # CM33 with PSPLIM
        (0x2C, 0),  # r0
        (0x30, 1),  # r1
        (0x34, 2),  # r2
        (0x38, 3),  # r3
        (0x04, 4),  # r4
        (0x08, 5),  # r5
        (0x0C, 6),  # r6
        (0x10, 7),  # r7
        (0x14, 8),  # r8
        (0x18, 9),  # r9
        (0x1C, 10),  # r10
        (0x20, 11),  # r11
        (0x3C, 12),  # r12
        (0x40, 14),  # lr
        (0x44, 15),  # pc
        (0x48, 16),  # xpsr
    ]
    thread_state_size_v3 = 0x4C

    stack_offset_to_reg_index_v1: ClassVar = [  # Used in Tintin, Cortex-M3
        (0x24, 0),  # r0
        (0x28, 1),  # r1
        (0x2C, 2),  # r2
        (0x30, 3),  # r3
        (0x04, 4),  # r4
        (0x08, 5),  # r5
        (0x0C, 6),  # r6
        (0x10, 7),  # r7
        (0x14, 8),  # r8
        (0x18, 9),  # r9
        (0x1C, 10),  # r10
        (0x20, 11),  # r11
        (0x34, 12),  # r12
        (0x38, 14),  # lr
        (0x3C, 15),  # pc
        (0x40, 16),  # xpsr
    ]
    thread_state_size_v1 = 0x44

    def __init__(self, id, ptr, running, name, registers):
        self.id = id
        self.ptr = ptr
        self.running = running
        self.name = name
        self.registers = registers

    def set_register(self, reg_index, value):
        self.registers[reg_index] = value

    def get_register(self, reg_index):
        return self.registers[reg_index]

    def __repr__(self):
        return f"<Thread id:{self.id:d}, ptr:0x{self.ptr:08X}, running:{self.running!r}, name:{self.name}, registers:{self.registers!r}"


##########################################################################################
class QemuGdbProxy:
    """
    This class implements a GDB server listening for a gdb connection on a specific port.
    It connects to and acts as a proxy to yet another gdb server running on the target system.
    This proxy implements advanced gdb commands like getting thread info by looking up the values of
    specific FreeRTOS symbols and querying the FreeRTOS data structures on the target system.
    """

    ##########################################################################################
    def __init__(self, port, target_host, target_port, connect_timeout):
        # The "target" is the remote system we are debugging. The target implements a basic
        #  gdb remote server
        self.target_host = target_host
        self.target_port = target_port
        self.target_socket = None
        self.connect_timeout = connect_timeout

        # The "client" is gdb
        self.client_accept_socket = None
        self.client_accept_port = port
        self.client_conn_socket = None

        self.packet_size = 2048
        self.active_thread_id = 0  # Selected by GDB
        self.threads = {}  # key is the thread id, value is a PebbleThread object

        # The QEMU gdb remote server always assigns a thread ID of 1 to it's one and only thread
        self.QEMU_MONITOR_CURRENT_THREAD_ID = 1

        # Free RTOS symbols we need to look up in order to inspect FreeRTOS threads
        symbol_list = [
            "uxFreeRTOSRegisterStackingVersion",
            "pxCurrentTCB",
            "pxReadyTasksLists",
            "xDelayedTaskList1",
            "xDelayedTaskList2",
            "pxDelayedTaskList",
            "pxOverflowDelayedTaskList",
            "xPendingReadyList",
            "xTasksWaitingTermination",
            "xSuspendedTaskList",
            "uxCurrentNumberOfTasks",
        ]
        self.symbol_dict = {symbol: None for symbol in symbol_list}

        self.got_all_symbols = False

    ##########################################################################################
    def _fetch_socket_data(self, timeout=None):
        """Fetch available data from our sockets (client and target). Block until any
        data is available, or until the target connection is closed. If we detect that the
        target connection has closed, we exit this app.

        If we detect that the client connection (from gdb) has closed, we wait for a new connection
        request from gdb.

        retval:
          (target_data, client_data)

        """

        target_data = b""
        client_data = b""

        while not target_data and not client_data:
            # Form our read list. The target socket is always in the read list. Depending on if we
            #  are waiting for a client connection or not, we either put the client_accept_socket or
            # client_conn_socket in the list.
            if self.client_conn_socket is not None:
                read_list = [self.target_socket, self.client_conn_socket]
            else:
                read_list = [self.target_socket, self.client_accept_socket]

            readable, _writable, _errored = select.select(read_list, [], [], timeout)
            # If nothing ready, we must have timed out
            if not readable:
                logger.debug("read timeout")
                break

            # Data available from target?
            if self.target_socket in readable:
                target_data = self.target_socket.recv(self.packet_size)
                if not target_data:
                    raise QemuGdbError("target system disconnected")
                logger.debug(
                    f"got target data: '{target_data}' (0x{target_data.hex()}) "
                )

            # Data available from client?
            if self.client_conn_socket is not None:
                if self.client_conn_socket in readable:
                    client_data = self.client_conn_socket.recv(self.packet_size)
                    if not client_data:
                        logger.info("client connection closed")
                        self.client_conn_socket.close()
                        self.client_conn_socket = None
                    logger.debug(
                        f"got client data: '{client_data}' (0x{client_data.hex()}) "
                    )

            # Connection request from client?
            else:
                if self.client_accept_socket in readable:
                    self.client_conn_socket, _ = self.client_accept_socket.accept()
                    logger.info("Connected to client")

        return (target_data, client_data)

    ##########################################################################################
    def _create_packet(self, data):
        checksum = sum(data) % 256
        packet = b"$%s#%02X" % (data, checksum)

        logger.debug("--<<<<<<<<<<<< GDB packet: %s", packet)
        return packet

    ##########################################################################################
    def _target_read_register(self, reg_index):
        """Fetch the value of the given register index from the active thread"""
        try:
            thread = self.threads[self.active_thread_id]
        except KeyError:
            raise QemuGdbError("Unknown thread id")
        return thread.get_register(reg_index)

    ##########################################################################################
    def _target_write_register(self, reg_index, value):
        """Update the value of the given register index in the active thread"""
        try:
            thread = self.threads[self.active_thread_id]
        except KeyError:
            raise QemuGdbError("Unknown thread id")
        print("TODO: NEED TO WRITE TO THREAD STACK ON TARGET TOO")
        thread.set_register(reg_index, value)

    ##########################################################################################
    def _target_read_memory(self, address, bytes):
        request = self._create_packet(f"m {address:08X},{bytes:08X}")
        self.target_socket.send(request)

        # read response
        data = ""
        while True:
            target_data = self.target_socket.recv(self.packet_size)
            if not target_data:
                raise QemuGdbError("target system disconnected")
            data += target_data
            if "$" in data and "#" in data:
                break

        _, data = data.split("$", 1)
        logger.debug(f"Received target response: {data}")

        resp = data.split("#", 1)[0]
        if resp.startswith("E "):
            raise QemuGdbError("Error response %s", resp)
        return resp

    ##########################################################################################
    def _target_read_uint32(self, address):
        hex = self._target_read_memory(address, 4)
        value = int(hex, 16)
        return byte_swap_uint32(value)

    ##########################################################################################
    def _target_read_uint8(self, address):
        hex = self._target_read_memory(address, 1)
        value = int(hex, 16)
        return value

    ##########################################################################################
    def _target_read_cstr(self, address, max_len):
        str_hex = self._target_read_memory(address, max_len)
        str = str_hex.decode("hex")
        return str.split("\0", 1)[0]

    ##########################################################################################
    def _target_collect_thread_info(self):

        # FreeRTOS params used to collect thread info
        FRTOS_LIST_NEXT_OFFSET = 16
        FRTOS_LIST_WIDTH = 20
        FRTOS_LIST_ELEM_NEXT_OFFSET = 8
        FRTOS_LIST_ELEM_CONTENT_OFFSET = 12
        FRTOS_THREAD_STACK_OFFSET = 0
        FRTOS_THREAD_NAME_OFFSET = 84
        FRTOS_MAX_PRIORITIES = 5

        num_threads = self._target_read_uint32(
            self.symbol_dict["uxCurrentNumberOfTasks"]
        )

        # Figure out the register stacking
        if self.symbol_dict["uxFreeRTOSRegisterStackingVersion"]:
            reg_stacking_version = self._target_read_uint8(
                self.symbol_dict["uxFreeRTOSRegisterStackingVersion"]
            )
        else:
            reg_stacking_version = 1

        if reg_stacking_version == 1:
            stack_offset_to_reg_index = PebbleThread.stack_offset_to_reg_index_v1
            thread_state_size = PebbleThread.thread_state_size_v1
        elif reg_stacking_version == 2:
            stack_offset_to_reg_index = PebbleThread.stack_offset_to_reg_index_v2
            thread_state_size = PebbleThread.thread_state_size_v2
        elif reg_stacking_version == 3:
            stack_offset_to_reg_index = PebbleThread.stack_offset_to_reg_index_v3
            thread_state_size = PebbleThread.thread_state_size_v3
        else:
            raise QemuGdbError(
                f"Unsupported uxFreeRTOSRegisterStackingVersion of {reg_stacking_version:d}"
            )

        # Get total number of threads and current thread ID
        num_threads = self._target_read_uint32(
            self.symbol_dict["uxCurrentNumberOfTasks"]
        )
        current_thread = self._target_read_uint32(self.symbol_dict["pxCurrentTCB"])
        self.threads = {}

        # Get the address of each list
        list_addresses = []
        address = self.symbol_dict["pxReadyTasksLists"]
        for i in range(FRTOS_MAX_PRIORITIES):
            list_addresses.append(address + i * FRTOS_LIST_WIDTH)

        for name in [
            "xDelayedTaskList1",
            "xDelayedTaskList2",
            "xPendingReadyList",
            "xSuspendedTaskList",
            "xTasksWaitingTermination",
        ]:
            list_addresses.append(self.symbol_dict[name])

        # Fetch the tasks from each list
        for list in list_addresses:
            thread_count = self._target_read_uint32(list)
            if thread_count == 0:
                continue

            # Location of first item
            elem_ptr = self._target_read_uint32(list + FRTOS_LIST_NEXT_OFFSET)

            # Loop through the list
            prev_elem_ptr = -1
            while (
                thread_count > 0
                and elem_ptr != 0
                and elem_ptr != prev_elem_ptr
                and len(self.threads) < num_threads
            ):
                thread_ptr = self._target_read_uint32(
                    elem_ptr + FRTOS_LIST_ELEM_CONTENT_OFFSET
                )
                thread_running = thread_ptr == current_thread

                # The QEMU gdb server assigns the active thread a thread ID of 1 and if we change it
                #  to something else (like the TCB ptr), then things are not ideal. For example, gdb
                #  will display a "The current thread <Thread ID 1> has terminated" message.
                # So, we will preserve 1 for the current thread and assign the TCB ptr for the
                #  others
                if thread_running:
                    thread_id = self.QEMU_MONITOR_CURRENT_THREAD_ID
                else:
                    thread_id = thread_ptr
                thread_name = self._target_read_cstr(
                    thread_ptr + FRTOS_THREAD_NAME_OFFSET, 32
                )

                stack = self._target_read_uint32(thread_ptr + FRTOS_THREAD_STACK_OFFSET)
                registers = [0] * len(PebbleThread.reg_name_to_index)
                for offset, reg_index in stack_offset_to_reg_index:
                    registers[reg_index] = self._target_read_uint32(stack + offset)
                registers[13] = stack + thread_state_size

                # Create the thread instance
                thread = PebbleThread(
                    id=thread_id,
                    ptr=thread_ptr,
                    running=thread_running,
                    name=thread_name,
                    registers=registers,
                )
                self.threads[thread_id] = thread
                logger.debug(f"Got thread info: {thread!r}")

                # Another thread in this list?
                prev_elem_ptr = elem_ptr
                elem_ptr = self._target_read_uint32(
                    elem_ptr + FRTOS_LIST_ELEM_NEXT_OFFSET
                )
                thread_count -= 1

    ##########################################################################################
    def _handle_set_active_thread_req(self, data):
        num = int(data, 16)
        if num == -1:  # All threads
            return
        elif num == 0:  # Any thread
            num = self.QEMU_MONITOR_CURRENT_THREAD_ID
        self.active_thread_id = num
        return self._create_packet("OK")

    ##########################################################################################
    def _handle_continue_req(self, msg):
        """The format of this is: 'vCont[;action[:thread-id]]...'
        The QEMU gdb server only understands a thread id of 1, so if we pass it other thread ids,
        it will barf.
        """
        if b";" not in msg:
            return None
        action_thread_pair = msg.split(b";")[1]
        if b":" in action_thread_pair:
            action = action_thread_pair.split(b":")[0]
        else:
            action = action_thread_pair

        # Send to target with the thread ID
        packet = self._create_packet(b"vCont;%s" % (action))
        self.target_socket.send(packet)

        # Change back to active thread of 1
        self.active_thread_id = self.QEMU_MONITOR_CURRENT_THREAD_ID
        return ""

    ##########################################################################################
    def _handle_thread_is_alive_req(self, data):
        num = int(data, 16)
        if num == -1 or num == 0:  # All threads
            return self._create_packet(b"OK")

        if num in self.threads:
            return self._create_packet(b"OK")
        return self._create_packet(b"E22")

    ##########################################################################################
    def _handle_get_all_registers_req(self):
        """Get all registers for the active thread"""

        resp = b""
        for i in range(len(PebbleThread.reg_name_to_index)):
            value = self._target_read_register(i)
            resp += b"%08X" % (byte_swap_uint32(value))
        return self._create_packet(resp)

    ##########################################################################################
    def _handle_query_req(self, msg):
        msg = msg.split(b"#")[0]
        query = msg.split(b":")
        logger.debug("GDB received query: %s", query)

        if query is None:
            logger.error("GDB received query packet malformed")
            return None

        elif query[0] == b"C":
            return self._create_packet(b"%d" % (self.active_thread_id))

        elif query[0] == b"fThreadInfo":
            if not self.got_all_symbols:
                # NOTE: When running the 4.9 gcc tool chain, gdb asks for thread info right
                # after attaching, before we have a chance to look up symbols, so respond
                # with "last thread" if we don't have symbols yet.
                return self._create_packet(b"l")  # last
            self._target_collect_thread_info()
            # For some strange reason, if the active thread is first, the first "i thread" gdb
            # command only displays that one thread, so reverse sort to put it at the end
            id_strs = (
                f"{id:016x}" for id in sorted(self.threads.keys(), reverse=True)
            )
            return self._create_packet(b"m" + b",".join(id_strs))

        elif query[0] == b"sThreadInfo":
            return self._create_packet(b"l")  # last

        elif query[0].startswith(b"ThreadExtraInfo"):
            id_str = query[0].split(b",")[1]
            id = int(id_str, 16)

            found_thread = self.threads.get(id, None)
            if found_thread is None:
                resp = f"<INVALID THREAD ID: {id:d}>"
            elif found_thread.running:
                resp = f"{found_thread.name} 0x{found_thread.ptr:08X}: Running"
            else:
                resp = f"{found_thread.name} 0x{found_thread.ptr:08X}"
            return self._create_packet(resp.encode("hex"))

        elif b"Symbol" in query[0]:
            if query[2] != b"":
                sym_name = query[2].decode("hex")
                if query[1] != b"":
                    sym_value = int(query[1], 16)
                    logger.debug(
                        f"Setting value of symbol '{sym_name}' to 0x{sym_value:08x}"
                    )
                    self.symbol_dict[sym_name] = sym_value
                else:
                    logger.debug(f"Could not find value of symbol '{sym_name}'")
                    self.symbol_dict[sym_name] = ""

            # Anymore we need to look up?
            symbol = None
            for x, y in list(self.symbol_dict.items()):
                if y is None:
                    symbol = x
                    break
            if symbol is not None:
                logger.debug(f"Asking gdb to lookup symbol {symbol}")
                return self._create_packet(b"qSymbol:%s" % (symbol.encode("hex")))
            else:
                self.got_all_symbols = True
                return self._create_packet(b"OK")

        else:
            return None

    ##########################################################################################
    def _handle_request(self, msg):
        """See if we want to handle a request directly here in the proxy

        retval: resp,
          resp: Response to return.
                if None, proxy doesn't deal with the request directly
        """

        logger.debug("-->>>>>>>>>>>> GDB req packet: %s", msg)

        msg = msg.split(b"#")[0]

        # query command
        if msg[1] == b"q":
            return self._handle_query_req(msg[2:])

        elif msg[1] == b"H":
            if msg[2] == b"c":
                return None
            else:
                return self._handle_set_active_thread_req(msg[3:])

        elif msg[1] == b"T":
            return self._handle_thread_is_alive_req(msg[2:])

        elif msg[1] == b"g":
            if (
                self.active_thread_id <= 0
                or self.active_thread_id == self.QEMU_MONITOR_CURRENT_THREAD_ID
            ):
                return None
            else:
                return self._handle_get_all_registers_req()

        elif msg[1] == b"p":
            # 'p <n>' : read value of register n
            if self.active_thread_id == self.QEMU_MONITOR_CURRENT_THREAD_ID:
                return None
            else:
                msg = msg[2:]
                reg_num = int(msg, 16)
                value = self._target_read_register(reg_num)
                return self._create_packet(f"{byte_swap_uint32(value):08X}")

        elif msg[1] == b"P":
            # 'P <n>=<r>' : set value of register n to r
            if self.active_thread_id == self.QEMU_MONITOR_CURRENT_THREAD_ID:
                return None
            else:
                msg = msg[2:].split(b"=")
                reg_num = int(msg[0], 16)
                val = int(msg[1], 16)
                val = byte_swap_uint32(val)
                self._target_write_register(reg_num, val)
                return self._create_packet("OK")

        elif msg[1:].startswith(b"vCont"):
            return self._handle_continue_req(msg[1:])

        else:
            return None

    ##########################################################################################
    def run(self):
        """Run the proxy"""

        # Connect to the target system first
        logger.info(
            f"Connecting to target system on {self.target_host}:{self.target_port}"
        )

        start_time = time.time()
        connected = False
        self.target_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while not connected and (time.time() - start_time < self.connect_timeout):
            try:
                self.target_socket.connect((self.target_host, self.target_port))
                connected = True
            except OSError:
                self.target_socket.close()
                self.target_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                time.sleep(0.1)

        if not connected:
            raise QemuGdbError(
                f"Unable to connect to target system on {self.target_host}:{self.target_port}. Is the emulator"
                " running?"
            )

        logger.info(
            f"Connected to target system on {self.target_host}:{self.target_port}"
        )

        # Open up our socket to accept connect requests from the client (gdb)
        self.client_accept_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_accept_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_accept_socket.bind(("", self.client_accept_port))
        self.client_accept_socket.listen(5)

        # Empty out any unsolicited data sent from the target
        (target_data, client_data) = self._fetch_socket_data(timeout=0.1)

        # --------------------------------------------------------------------------------------
        # Loop processing requests
        data = b""
        while True:
            # read more data from client until we get at least one packet
            while True:
                (target_data, client_data) = self._fetch_socket_data()

                # Pass through any response from the target back to gdb
                if target_data and self.client_conn_socket is not None:
                    self.client_conn_socket.send(target_data)

                # Ctrl-C interrupt?
                if CTRL_C_CHARACTER in client_data:
                    self.target_socket.send(CTRL_C_CHARACTER)
                    client_data = client_data[client_data.index(CTRL_C_CHARACTER) + 1 :]

                data += client_data
                if b"$" in data and b"#" in data:
                    break

            # Process all complete packets we have received from the client
            while b"$" in data and b"#" in data:
                data = data[data.index(b"$") :]
                logger.debug(f"Processing remaining data: {data}")
                end = data.index(b"#") + 3  # 2 bytes of checksum
                packet = data[0:end]
                data = data[end:]

                # decode and prepare resp
                logger.debug(f"Processing packet: {packet}")
                resp = self._handle_request(packet)

                # If it's nothing we care about, pass to target and return the response back to
                #  client
                if resp is None:
                    logger.debug(f"Sending request to target: {packet}")
                    self.target_socket.send(packet)

                # else, we generated our own response that needs to go to the client
                elif resp != "":
                    self.client_conn_socket.send(b"+" + resp)

                    # wait for ack from the client
                    (target_data, client_data) = self._fetch_socket_data()
                    if target_data:
                        self.client_conn_socket.send(target_data)

                    if client_data[0] != b"+":
                        logger.debug("gdb client did not ack")
                    else:
                        logger.debug("gdb client acked")

                    # Add to our accumulated content
                    data += client_data[1:]


####################################################################################################
if __name__ == "__main__":
    # Collect our command line arguments
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--port", type=int, default=1233, help="Port to accept incomming connections on"
    )
    parser.add_argument(
        "--target", default="localhost:1234", help="target to connect to "
    )
    parser.add_argument(
        "--connect_timeout",
        type=float,
        default=1.0,
        help="give up if we can't connect to the target within this timeout (sec)",
    )
    parser.add_argument("--debug", action="store_true", help="Turn on debug logging")
    args = parser.parse_args()

    level = logging.INFO
    if args.debug:
        level = logging.DEBUG
    logging.basicConfig(level=level)

    (target_host, target_port) = args.target.split(":")
    proxy = QemuGdbProxy(
        port=args.port,
        target_host=target_host,
        target_port=int(target_port),
        connect_timeout=args.connect_timeout,
    )
    proxy.run()
