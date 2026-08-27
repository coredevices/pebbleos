# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

try:
    import gdb
except ImportError:
    raise RuntimeError(
        "This file is a GDB module.\n"
        "It is not intended to be run outside of GDB.\n"
        "Hint: to load a script in GDB, use `source this_file.py`"
    )


import datetime
import logging
import string
import types
from collections import OrderedDict, namedtuple

from gdb_symbols import get_static_variable
from gdb_tintin import FreeRTOSMutex, LinkedList, Tasks
from gdb_tintin_metadata import TintinMetadata

logger = logging.getLogger(__name__)
recognizers = {}

_Recognizer = namedtuple("Recognizer", "impl depends_on")


def register_recognizer(name, fn, dependency=None):
    """Registers a recognizer.

    Recognizers are run against each block in the heap.
    They consist of a name, a dependency, and an implementation.
    The implementation consumes a block, a heap object, and a results dictionary and returns
    either a casted block or None.

    See Recognizer for auto-registering declarative classes.
    """
    recognizers[name] = _Recognizer(fn, dependency)


def parse_heap(heap, recognizer_subset=None):
    results = {"Free": heap.free_blocks()}

    heap_recognizers = recognizers
    if recognizer_subset:
        heap_recognizers = {name: recognizers[name] for name in recognizer_subset}

    ordered_recognizers, hidden = _order_recognizers(heap_recognizers)
    logger.info("Running: {}".format(", ".join(list(ordered_recognizers.keys()))))
    for name, recognizer in ordered_recognizers.items():
        try:
            results[name] = [
                _f
                for _f in (
                    recognizer.impl(block, heap, results)
                    for block in heap.allocated_blocks()
                )
                if _f
            ]
        except Exception:  # noqa: BLE001
            print(name + " hit an exception. Skipping")

    for dependency in hidden:
        del results[dependency]

    return results


def _order_recognizers(recognizer_subset):
    recognizer_subset = recognizer_subset.copy()
    ordered = OrderedDict()
    hidden = []
    last_length = -1

    # Run until we stop adding recognizers to our ordered list
    while recognizer_subset and len(ordered) != last_length:
        last_length = len(ordered)

        for name, recognizer in list(recognizer_subset.items()):
            if not recognizer.depends_on or recognizer.depends_on in ordered:
                ordered[name] = recognizer
                del recognizer_subset[name]

    # Add implicit dependencies
    for name, recognizer in recognizer_subset.items():
        if recognizer.depends_on not in ordered:
            logger.info(f"Adding dependency: {recognizer.depends_on}")
            ordered[recognizer.depends_on] = recognizers[recognizer.depends_on]
            hidden.append(recognizer.depends_on)
        ordered[name] = recognizer

    return [ordered, hidden]


class RecognizerType(type):
    def __new__(cls, name, bases, dct):
        for key in dct:
            # Convert instance methods into class methods
            if isinstance(dct[key], types.FunctionType):
                dct[key] = classmethod(dct[key])

        if dct.get("depends_on") == "Heap":
            dct["depends_on"] = None
            dct["uses_heap"] = True
        else:
            dct["uses_heap"] = False

        return type.__new__(cls, name, bases, dct)

    def __init__(cls, name, bases, dct):
        if name != "Recognizer":
            register_recognizer(name, cls, dct.get("depends_on"))

        super().__init__(name, bases, dct)

    def __call__(cls, block, heap, results):
        """Returns either a casted block or None.

        This allows the class object itself to implement the recognizer protocol.
        This means we can register the class as a recognizer without creating an instance of it.
        """
        try:
            right_size = (
                (block.size in heap.object_size(cls.type)) if cls.type else True
            )
        except gdb.error:
            # Type doesn't exist
            return None

        if cls.uses_heap:
            search_blocks = [r_block.data for r_block in heap]
        else:
            search_blocks = [
                r_block.data for r_block in results.get(cls.depends_on) or []
            ]

        if right_size and cls.is_type(block.cast(cls.type), search_blocks):
            return block.cast(cls.type, clone=True)
        else:
            return None

    def __str__(cls):
        return cls.__name__


class Recognizer(metaclass=RecognizerType):
    """This is a declarative recognizer. It auto-registers with the recognizer dictionary.

    Note that declarative recognizers are singletons that don't get instantiated, so
    __init__ is never called and any modifications to class members are persistent.
    This behavior holds for subclasses.

    Example:
    >>> # A Recognizer's class name is its friendly name.
    ... class Test(Recognizer):
    ...
    ...     # Its (optional) type is the C struct it's searching for.
    ...     type = 'TestStruct'
    ...
    ...     # Its (optional) dependency is the friendly name of the type
    ...     # whose results it depends on. Those results are passed to
    ...     # is_type as search_blocks.
    ...     depends_on = 'Heap'
    ...
    ...     # is_type() is the comparison function. If type is not NULL,
    ...     # each block will be casted to 'type'. Note that self actually
    ...     # refers to the class.
    ...     def is_type(self, block, search_blocks):
    ...         return block['foo'] == 4
    """

    type = None
    depends_on = None

    def is_type(self, block, search_blocks):
        return True


# --- Recognizers --- #


# --- Mutexes --- #
class PebbleMutex(Recognizer):
    type = "PebbleMutexCommon"

    def is_type(self, pebble_mutex, search_blocks):
        mutex = FreeRTOSMutex(pebble_mutex["freertos_mutex"])
        try:
            return (not mutex.locked()) == (pebble_mutex["lr"] == 0) and (
                0 <= mutex.num_waiters() <= 10
            )
        except Exception:  # noqa: BLE001
            return False


class LightMutex(Recognizer):
    type = "LightMutex_t"
    depends_on = "PebbleMutex"

    def is_type(self, mutex, search_blocks):
        return mutex in [block["freertos_mutex"] for block in search_blocks]


# --- Queues --- #
class Queue(Recognizer):
    depends_on = "Heap"

    def is_type(self, data, search_blocks):
        queue_type = gdb.lookup_type("Queue_t")
        queue = data.cast(queue_type.pointer())
        int(queue_type.sizeof)

        storage_size = queue["uxLength"] * queue["uxItemSize"]
        correct_head = queue["pcHead"] >= data
        correct_tail = queue["pcTail"] == queue["pcHead"] + storage_size
        return (
            queue["uxLength"] > 0
            and queue["pcWriteTo"] != 0
            and correct_head
            and correct_tail
        )


class Semaphore(Recognizer):
    type = "Queue_t"
    depends_on = "Queue"

    def is_type(self, semaphore, search_blocks):
        return semaphore["uxItemSize"] == 0 and semaphore in search_blocks


# --- Data Logging --- #
class DataLoggingSession(Recognizer):
    type = "DataLoggingSession"

    def is_type(self, dls, search_blocks):
        total_tasks = Tasks().total
        return 0 <= dls["task"] < total_tasks


class ActiveDLS(Recognizer):
    type = "DataLoggingActiveState"
    depends_on = "DataLoggingSession"

    def is_type(self, active_dls, search_blocks):
        return active_dls in [block["data"] for block in search_blocks]


class DLSBuffer(Recognizer):
    depends_on = "ActiveDLS"

    def is_type(self, dls_buffer, search_blocks):
        return dls_buffer in [block["buffer_storage"] for block in search_blocks]


# --- Tasks --- #
class Task(Recognizer):
    type = "TCB_t"

    def is_type(self, task, search_blocks):
        return is_string(task["pcTaskName"])


class TaskStack(Recognizer):
    depends_on = "Task"

    def is_type(self, stack, search_blocks):
        return stack in [block["pxStack"] for block in search_blocks]


# --- String-related --- #
def is_string(data):
    try:
        data = data.bytes.decode("utf-8")
        return len(data) > 2 and all(
            ord(codepoint) > 127 or codepoint in string.printable for codepoint in data
        )
    except UnicodeDecodeError:
        return False
    except UnicodeEncodeError:
        return False


class String(Recognizer):
    def is_type(self, string, search_blocks):
        return is_string(string.cast(gdb.lookup_type("char").pointer()))


class PFSFileNode(Recognizer):
    type = "PFSFileChangedCallbackNode"
    depends_on = "String"

    def is_type(self, filenode, search_blocks):
        return filenode["name"] in search_blocks


class SettingsFile(Recognizer):
    type = "SettingsFile"
    depends_on = "String"

    def is_type(self, settings, search_blocks):
        try:
            timestamp = int(settings["last_modified"])
            date = datetime.datetime.fromtimestamp(timestamp).astimezone()
        except ValueError:
            return False

        # If a user closes a settings file but forgets to free the
        # underlying memory the name will not be in search_blocks
        return (
            settings["max_used_space"] <= settings["max_space_total"]
            and settings["used_space"] <= settings["max_used_space"]
            and date.year > 2010
            and date.year < 2038
        )


# --- Analytics --- #
class AnalyticsStopwatch(Recognizer):
    type = "AnalyticsStopwatchNode"

    def is_type(self, stopwatch, search_blocks):
        s_stopwatch_list = get_static_variable("s_stopwatch_list")
        stopwatch_list = LinkedList(gdb.parse_and_eval(s_stopwatch_list).dereference())
        return stopwatch in stopwatch_list


class AnalyticsHeartbeatList(Recognizer):
    type = "AnalyticsHeartbeatList"

    def is_type(self, listnode, search_blocks):
        s_app_heartbeat_list = get_static_variable("s_app_heartbeat_list")
        heartbeat_list = LinkedList(gdb.parse_and_eval(s_app_heartbeat_list)["node"])
        return listnode in heartbeat_list


class AnalyticsHeartbeat(Recognizer):
    depends_on = "AnalyticsHeartbeatList"

    def is_type(self, heartbeat, search_blocks):
        s_device_heartbeat = get_static_variable("s_device_heartbeat")
        device_heartbeat = gdb.parse_and_eval(s_device_heartbeat)
        return heartbeat == device_heartbeat or heartbeat in [
            block["heartbeat"] for block in search_blocks
        ]


# --- Timers --- #
class TaskTimer(Recognizer):
    type = "TaskTimer"

    def is_type(self, timer, search_blocks):
        timer_manager_ref = get_static_variable("s_task_timer_manager")
        timer_manager = gdb.parse_and_eval(timer_manager_ref)
        max_timer_id = int(timer_manager["next_id"]) - 1
        return 1 <= timer["id"] <= max_timer_id


class EventedTimer(Recognizer):
    type = "EventedTimer"

    def is_type(self, timer, search_blocks):
        timer_manager_ref = get_static_variable("s_task_timer_manager")
        timer_manager = gdb.parse_and_eval(timer_manager_ref)
        max_timer_id = int(timer_manager["next_id"]) - 1
        total_tasks = Tasks().total

        return (
            1 <= timer["sys_timer_id"] <= max_timer_id
            and 0 <= timer["target_task"] < total_tasks
        )


# --- Communication --- #
class CommSession(Recognizer):
    type = "CommSession"
    depends_on = "Heap"

    def try_session_address(self, var_name):
        try:
            var_ref = get_static_variable(var_name)
            return gdb.parse_and_eval(var_ref).address
        except Exception:  # noqa: BLE001
            return None

    def is_type(self, session, search_blocks):
        meta = TintinMetadata()
        meta.hw_platform()

        transport_imp = session["transport_imp"].dereference().address

        iap = self.try_session_address("s_iap_transport_implementation")
        spp = self.try_session_address("s_plain_spp_transport_implementation")
        ppogatt = self.try_session_address("s_ppogatt_transport_implementation")
        qemu = self.try_session_address("s_qemu_transport_implementation")
        pulse_pp = self.try_session_address("s_pulse_transport_implementation")

        return transport_imp in [iap, spp, ppogatt, qemu, pulse_pp]


# --- Windows --- #
class NotificationNode(Recognizer):
    type = "NotifList"

    def is_type(self, node, search_blocks):
        s_presented_notifs = gdb.parse_and_eval(
            get_static_variable("s_presented_notifs")
        )
        if s_presented_notifs.dereference().address == 0:
            return False
        notification_list = LinkedList(s_presented_notifs["list_node"])
        return node in notification_list


class Modal(Recognizer):
    type = "WindowStackItem"

    def is_type(self, node, search_blocks):
        s_modal_window_stack = get_static_variable("s_modal_window_stack")
        modal_stack = LinkedList(gdb.parse_and_eval(s_modal_window_stack)["node"])
        return node in modal_stack


# --- Misc --- #
class EventService(Recognizer):
    type = "EventServiceEntry"

    def is_type(self, entry, search_blocks):
        subscribers = 0
        for x in range(Tasks().total):
            if entry["subscribers"][x] != 0:
                subscribers += 1
        return entry["num_subscribers"] == subscribers


class VoiceEncoder(Recognizer):
    type = "VoiceEncoder"


class AlgState(Recognizer):
    type = "AlgState"

    def is_type(self, state, search_blocks):
        s_alg_state_ref = get_static_variable("s_alg_state")
        s_alg_state = gdb.parse_and_eval(s_alg_state_ref)
        return state.dereference().address == s_alg_state


class KAlgState(Recognizer):
    type = "KAlgState"

    def is_type(self, state, search_blocks):
        s_alg_state_ref = get_static_variable("s_alg_state")
        s_alg_state = gdb.parse_and_eval(s_alg_state_ref)
        if s_alg_state.dereference().address == 0:
            return False
        return s_alg_state["k_state"] == state.dereference().address


class CachedResource(Recognizer):
    type = "CachedResource"

    def is_type(self, resource, search_blocks):
        s_resource_list = get_static_variable("s_resource_list")
        resources = LinkedList(gdb.parse_and_eval(s_resource_list)["list_node"])
        return resource in resources


# --- Applib --- #
class ModalWindow(Recognizer):
    depends_on = "Modal"

    def is_type(self, window, search_blocks):
        # Note that these are most likely dialogs. Try casting them.
        return window in [block["modal_window"] for block in search_blocks]


try:
    GBitmapFormats = gdb.types.make_enum_dict(gdb.lookup_type("enum GBitmapFormat"))
except gdb.error:
    GBitmapFormats = None


# FIXME: This can be improved. It results in a lot of false negatives
class GBitmap(Recognizer):
    type = "GBitmap"

    def is_type(self, bitmap, search_blocks):
        if GBitmapFormats is None:
            return False

        row_size_bytes = bitmap["row_size_bytes"]
        bounds_width = bitmap["bounds"]["size"]["w"]
        format = int(bitmap["info"]["format"])
        is_circular_format = format == GBitmapFormats["GBitmapFormat8BitCircular"]
        is_valid_circular = is_circular_format and row_size_bytes == 0
        is_valid_rect = not is_circular_format and row_size_bytes * 8 >= bounds_width
        return (is_valid_circular or is_valid_rect) and (
            format in list(GBitmapFormats.values())
        )


class GBitmap_addr(Recognizer):
    depends_on = "GBitmap"

    def is_type(self, data, search_blocks):
        return data in [block["addr"] for block in search_blocks]


class GBitmap_palette(Recognizer):
    depends_on = "GBitmap"

    def is_type(self, data, search_blocks):
        return data in [block["palette"] for block in search_blocks]


# class AnimationAux(Recognizer):
#     type = 'AnimationAuxState'

#     def is_type(self, aux, search_blocks):
#         possible_aux = [gdb.parse_and_eval('kernel_applib_get_animation_state().aux'),
#                         gdb.parse_and_eval('s_app_state.animation_state.aux')]

#         return aux in possible_aux


# class Animation(Recognizer):
#     type = 'AnimationPrivate'

#     def is_type(self, animation, search_blocks):
#         animations = []

#         kernel_state = gdb.parse_and_eval('kernel_applib_get_animation_state()')
#         animations.extend(LinkedList(kernel_state['unscheduled_head'].dereference()))
#         animations.extend(LinkedList(kernel_state['scheduled_head'].dereference()))

#         app_state = gdb.parse_and_eval('s_app_state.animation_state')
#         animations.extend(LinkedList(app_state['unscheduled_head'].dereference()))

#         return animation in animations
