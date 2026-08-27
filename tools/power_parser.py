# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import re
import sys
from collections import deque, namedtuple

from serial_port_wrapper import SerialPortWrapper

PowerDataPoint = namedtuple("PowerDataPoint", ["timestamp", "power", "duration"])


class PowerSystem:
    def __init__(self):
        # list of named tuples consisting of (timestamp, power, duration)
        self.powerData = []
        # queue of non-overlapping power segment tuples (timestamp, power, duration)
        self.powerDeque = deque()

    def getPowerTuples(self):
        return self.powerData

    def addPower(self, timestamp, power, duration):
        self.powerData.append(PowerDataPoint(timestamp, power, duration))
        self.powerDeque.append(PowerDataPoint(timestamp, power, duration))

    def getAvgPowerBetween(self, startTimestamp, endTimestamp):
        if len(self.powerDeque) == 0:
            return 0
        powerSum = 0
        powerCount = 0
        nextPower = self.powerDeque.popleft()
        while nextPower is not None and nextPower.timestamp < endTimestamp:
            if nextPower.timestamp + nextPower.duration < startTimestamp:
                if len(self.powerDeque) == 0:
                    nextPower = None
                else:
                    nextPower = self.powerDeque.popleft()
                continue

            # activeDuration = min(endTimestamp - nextPower.timestamp, nextPower.duration, endTimestamp - startTimestamp)
            # TODO: there is a minor bug where if a system has a very long period, we will bin the beginning of its power
            # into a later bin.
            activeDuration = min(endTimestamp - nextPower.timestamp, nextPower.duration)

            powerSum = powerSum + nextPower.power * 1.0 * activeDuration / (
                endTimestamp - startTimestamp
            )
            powerCount = powerCount + 1

            if activeDuration < nextPower.duration:
                # the power tuple active period extends over the end of this duration
                nextPower = PowerDataPoint(
                    endTimestamp, nextPower.power, nextPower.duration - activeDuration
                )
                break
            else:
                if len(self.powerDeque) == 0:
                    nextPower = None
                    break
                else:
                    nextPower = self.powerDeque.popleft()

        if nextPower is not None:
            self.powerDeque.appendleft(nextPower)

        if powerCount > 0:
            return powerSum / (powerCount * 1.0)
        else:
            return 0


# Most systems fall into this category:
# the duration that they are on is profiled
class IntervalPowerSystem(PowerSystem):
    def __init__(self, interval=1024, activePower=0):
        PowerSystem.__init__(self)
        self.interval = interval
        self.activePower = activePower

    def addPower(self, timestamp, data):
        duration = min(int(data[0]), self.interval)
        # PowerSystem.addPower(self, timestamp, (self.activePower * 1.0) * (duration * 1.0 / self.interval), self.interval)
        PowerSystem.addPower(self, timestamp, (self.activePower * 1.0), duration)


# Special case for the battery
class BattPowerSystem(PowerSystem):
    def addPower(self, timestamp, data):
        chg_state = data[0]
        voltage = data[1]


# Special case for the accelerometer
class AccelPowerSystem(PowerSystem):
    def addPower(self, timestamp, data):
        state = data[0]
        frequency = data[1]


# Special case for the magnetometer
class MagPowerSystem(PowerSystem):
    def addPower(self, timestamp, data):
        state = data[0]
        adc_rate = data[1]


# Special case for the vibe motor
class VibePowerSystem(PowerSystem):
    def addPower(self, timestamp, data):
        state = data[0]
        freq = data[1]
        duty = data[2]


# Special case for the backlight
class BacklightPowerSystem(PowerSystem):
    def __init__(self):
        PowerSystem.__init__(self)
        self.lastPowerDatum = PowerDataPoint(0, 0, 0)

    def addPower(self, timestamp, data):
        state = data[0]
        freq = int(data[1])
        duty = int(data[2])

        if state == "OFF":
            power = 0
        else:
            FULL_DUTY_CURRENT = 100.0  # TODO: figure out exactly what this really is
            power = duty * FULL_DUTY_CURRENT / 100.0

        PowerSystem.addPower(
            self,
            self.lastPowerDatum.timestamp,
            self.lastPowerDatum.power,
            timestamp - self.lastPowerDatum.timestamp,
        )
        self.lastPowerDatum = PowerDataPoint(timestamp, power, 0)


# Peripheral clocks
#          Datasheet      Pebble
# fHCLK:   120MHz         64MHz
# fPCLK1:  fHCLK/4        fHCLK/4
# fPCLK2:  fHCLK/2        fHCLK/2

f_HCLK_DATA = 120.0
APB1_PRE_DATA = 4.0
APB2_PRE_DATA = 2.0

f_HCLK_PEBBLE = 64.0
APB1_PRE_PEBBLE = 4.0
APB2_PRE_PEBBLE = 2.0

AHB1_SCALE = f_HCLK_PEBBLE / f_HCLK_DATA
APB1_SCALE = f_HCLK_PEBBLE / f_HCLK_DATA * APB1_PRE_PEBBLE / APB1_PRE_DATA
APB2_SCALE = f_HCLK_PEBBLE / f_HCLK_DATA * APB2_PRE_PEBBLE / APB2_PRE_DATA

powerSystems = {
    "2v5Reg": IntervalPowerSystem(activePower=0.0025),
    "McuCoreSleep": IntervalPowerSystem(activePower=5.0),
    "McuCoreRun": IntervalPowerSystem(activePower=13.0),
    "5vReg": IntervalPowerSystem(
        activePower=0.007
    ),  # currently unused because it is on all the time
    "McuGpioA": IntervalPowerSystem(activePower=0.45 * AHB1_SCALE),
    "McuGpioB": IntervalPowerSystem(activePower=0.43 * AHB1_SCALE),
    "McuGpioC": IntervalPowerSystem(activePower=0.46 * AHB1_SCALE),
    "McuGpioD": IntervalPowerSystem(activePower=0.44 * AHB1_SCALE),
    "McuGpioH": IntervalPowerSystem(activePower=0.42 * AHB1_SCALE),
    "McuCrc": IntervalPowerSystem(activePower=1.17 * AHB1_SCALE),
    "McuDma1": IntervalPowerSystem(activePower=2.76 * AHB1_SCALE),
    "McuDma2": IntervalPowerSystem(activePower=2.85 * AHB1_SCALE),
    "McuTim3": IntervalPowerSystem(activePower=0.49 * APB1_SCALE),
    "McuTim4": IntervalPowerSystem(activePower=0.54 * APB1_SCALE),
    "McuUsart3": IntervalPowerSystem(activePower=0.25 * APB1_SCALE),
    "McuI2C1": IntervalPowerSystem(activePower=0.25 * APB1_SCALE),
    "McuI2C2": IntervalPowerSystem(activePower=0.25 * APB1_SCALE),
    "McuSpi2": IntervalPowerSystem(activePower=0.20 * APB1_SCALE),
    "McuPwr": IntervalPowerSystem(activePower=0.15 * APB1_SCALE),
    "McuSpi1": IntervalPowerSystem(activePower=1.20 * APB2_SCALE),
    "McuUsart1": IntervalPowerSystem(activePower=0.38 * APB2_SCALE),
    "McuTim1": IntervalPowerSystem(activePower=1.06 * APB2_SCALE),
    "McuAdc1": IntervalPowerSystem(activePower=2.13 * APB2_SCALE),
    "McuAdc2": IntervalPowerSystem(activePower=2.04 * APB2_SCALE),
    "FlashRead": IntervalPowerSystem(activePower=1.70),
    "FlashWrite": IntervalPowerSystem(activePower=20.0),
    "FlashErase": IntervalPowerSystem(activePower=20.0),
    "AccelLowPower": IntervalPowerSystem(),
    "AccelNormal": IntervalPowerSystem(),
    "Mfi": IntervalPowerSystem(),
    "Mag": IntervalPowerSystem(),
    "BtShutdown": IntervalPowerSystem(),
    "BtDeepSleep": IntervalPowerSystem(),
    "BtActive": IntervalPowerSystem(activePower=2.5),
    "Ambient": IntervalPowerSystem(),
    "Profiling": IntervalPowerSystem(),
    "Battery": BattPowerSystem(),
    "Accel": AccelPowerSystem(),
    "Mag": MagPowerSystem(),
    "Vibe": VibePowerSystem(),
    "Backlight": BacklightPowerSystem(),
}

plottedSystems = [
    "2v5Reg",
    "McuCoreSleep",
    "McuCoreRun",
    "McuGpioA",
    "McuGpioB",
    "McuGpioC",
    "McuGpioD",
    "McuGpioH",
    "McuCrc",
    "McuDma1",
    "McuDma2",
    "McuSpi2",
    "McuSpi1",
    "FlashRead",
    "FlashWrite",
    "FlashErase",
    "Mfi",
    "BtActive",
    "Backlight",
]


def gatherData(tty, outfile):
    # regex to find power tracking info:
    pwr_regex = re.compile(r">>>PWR:(.*)<")

    if tty is not None:
        s = SerialPortWrapper(tty)
    else:
        s = sys.stdin

    f = open(outfile, "w")

    systemKeys = powerSystems.keys()

    try:
        lastOutputTimestamp = -1
        outString = '"ticks"'
        for system in plottedSystems:
            outString = f'{outString},"{system}"'
        f.write("%s\n" % outString)

        while True:
            powerLine = pwr_regex.search(s.readline())

            if not powerLine:
                continue
            data = powerLine.group(1)
            split_data = data.split(",")

            if len(split_data) < 2:
                continue

            system = split_data[1]
            if not system in systemKeys:
                continue

            timestamp = int(split_data[0])

            powerSystems[system].addPower(timestamp, split_data[2:])

            # output the rectangular data as it comes in
            if lastOutputTimestamp == -1:
                lastOutputTimestamp = timestamp

            latency = 4 * 1024

            for ts in range(lastOutputTimestamp, timestamp - 1 - latency, 1024):
                outString = "%d" % ts
                for system in plottedSystems:
                    avgPower = powerSystems[system].getAvgPowerBetween(ts, ts + 1024)
                    outString = f"{outString},{avgPower:f}"
                f.write("%s\n" % outString)
                lastOutputTimestamp = ts + 1024

    except KeyboardInterrupt:
        print("Bye")
    finally:
        s.close()
        f.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Power Profiling Parser")
    parser.add_argument("--tty", help="serial terminal. Otherwise takes stdin")
    parser.add_argument("-o", "--outfile", help="path to the output file")
    args = parser.parse_args()

    if len(sys.argv) < 2 or not args.outfile:
        parser.print_help()
        sys.exit(1)

    gatherData(args.tty, args.outfile)
