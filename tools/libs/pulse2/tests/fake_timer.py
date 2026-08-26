# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


class FakeTimer:
    TIMERS = []

    def __init__(self, interval, function):
        self.interval = interval
        self.function = function
        self.started = False
        self.expired = False
        self.cancelled = False
        type(self).TIMERS.append(self)

    def __repr__(self):
        state_flags = "".join(
            [
                "S" if self.started else "N",
                "X" if self.expired else ".",
                "C" if self.cancelled else ".",
            ]
        )
        return f"<FakeTimer({self.interval}, {self.function}) {state_flags} at {id(self):#x}>"

    def start(self):
        if self.started:
            raise RuntimeError("threads can only be started once")
        self.started = True

    def cancel(self):
        self.cancelled = True

    def expire(self):
        """Simulate the timeout expiring."""
        assert self.started, "timer not yet started"
        assert not self.expired, "timer can only expire once"
        self.expired = True
        self.function()

    @property
    def is_active(self):
        return self.started and not self.expired and not self.cancelled

    @classmethod
    def clear_timer_list(cls):
        cls.TIMERS = []

    @classmethod
    def get_active_timers(cls):
        return [t for t in cls.TIMERS if t.is_active]
