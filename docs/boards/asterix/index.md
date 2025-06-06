# Asterix

## Programming

The Asterix board provides a B2B (Board-To-Board) connector that gives access to:

- MCU VDD, VUSB and GND
- MCU SWCLK, SWDIO and RESET
- Debug UART RX/TX

The connector part number is Molex 5050040812, and the pinout is as shown below.

```{figure} images/b2b-pinout.webp
Asterix B2B connector pinout
```

The "Core B2B v2" board has been designed as a companion programming board.
It is based on the [Raspberry Pi Debug Probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html).
Software for the `Core B2B v2` board is available in the [Core B2B v2 repository](https://github.com/coredevices/asterix_b2b2_fw).
Below you can find a picture on how it is connected, and a list of its main features.

```{figure} images/asterix-programming.webp
Asterix and Core B2B v2
```

```{warning}
External connectors (SWD/UART) expose 1.8V signals.
Do not use any adapter that does not operate at 1.8V, or there is a **risk of damage**!
```

1. Asterix B2B connector
2. USB connector

   - Powers the board
   - Provides VUSB
   - Exposes a CMSIS-DAP device and a virtual COM port

3. Debug UART routing

   - L: External connector (5)
   - R: Embedded virtual COM port (2)

4. SWD routing

   - L: External SWD connector (6)
   - R: Embedded CMSIS-DAP (2)

5. Debug UART pins
6. External SWD connector
7. MCU Reset
8. VUSB switch

   - L: connected
   - R: disconnected
