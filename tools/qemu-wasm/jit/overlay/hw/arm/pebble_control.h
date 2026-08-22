#ifndef PEBBLE_CONTROL_H
#define PEBBLE_CONTROL_H

#include "qemu/typedefs.h"
#include "hw/arm/stm32_common.h"

typedef struct PebbleControl PebbleControl;

/* Create pebble_control that sits between a Chardev and a Stm32Uart.
 * chr: the chardev connected to the host (e.g. serial_hd(1))
 * uart: the UART device in the emulated Pebble
 */
PebbleControl *pebble_control_create(Chardev *chr, Stm32Uart *uart);

/* Create pebble_control for generic Pebble machines (non-STM32).
 * chr: the chardev connected to the host (e.g. serial_hd(1))
 * uart: the DeviceState of a pebble-simple-uart device
 */
PebbleControl *pebble_control_create_generic(Chardev *chr, DeviceState *uart);

void pebble_control_send_vibe_notification(PebbleControl *s, bool on);

#endif /* PEBBLE_CONTROL_H */
