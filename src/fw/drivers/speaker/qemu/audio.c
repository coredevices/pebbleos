/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/speaker/qemu/audio.h>

#include "services/system_task.h"

#include <stdint.h>

// QEMU audio device register offsets (must match pebble-audio QEMU device)
#define AUDIO_CTRL       0x00
#define AUDIO_STATUS     0x04
#define AUDIO_SAMPLERATE 0x08
#define AUDIO_DATA       0x0C
#define AUDIO_INTCTRL    0x10
#define AUDIO_INTSTAT    0x14
#define AUDIO_BUFAVAIL   0x18
#define AUDIO_VOLUME     0x1C

// Interrupt bits
#define INT_BUFAVAIL     (1 << 0)

#define REG32(addr) (*(volatile uint32_t *)(addr))

void audio_init(AudioDevice *dev) {
  REG32(dev->base_addr + AUDIO_CTRL) = 0;
  REG32(dev->base_addr + AUDIO_SAMPLERATE) = 16000;
  dev->state->trans_cb = NULL;
}

static void prv_audio_system_task_cb(void *data) {
  AudioDeviceState *state = (AudioDeviceState *)data;
  if (state->trans_cb) {
    // Read how many samples the QEMU ring buffer can accept.
    // We don't have the base_addr here directly, but the callback
    // will trigger audio_write() which checks BUFAVAIL itself.
    // Pass a generous free size — the actual limit is enforced by
    // audio_write() returning when the QEMU buffer is full.
    uint32_t free_bytes = 4096 * sizeof(int16_t);
    state->trans_cb(&free_bytes);
  }
}

void audio_start(AudioDevice *dev, AudioTransCB cb) {
  dev->state->trans_cb = cb;

  // Set sample rate and enable the device
  REG32(dev->base_addr + AUDIO_SAMPLERATE) = 16000;
  REG32(dev->base_addr + AUDIO_INTSTAT) = INT_BUFAVAIL; // clear pending
  REG32(dev->base_addr + AUDIO_INTCTRL) = INT_BUFAVAIL; // enable IRQ
  REG32(dev->base_addr + AUDIO_CTRL) = 1; // enable

  // Enable NVIC IRQ
  NVIC_SetPriority(dev->irqn, 5);
  NVIC_EnableIRQ(dev->irqn);
}

uint32_t audio_write(AudioDevice *dev, void *buf, uint32_t size) {
  int16_t *samples = (int16_t *)buf;
  uint32_t num_samples = size / sizeof(int16_t);

  for (uint32_t i = 0; i < num_samples; i++) {
    REG32(dev->base_addr + AUDIO_DATA) = (uint32_t)(uint16_t)samples[i];
  }

  // Return how many bytes of free space remain
  uint32_t avail = REG32(dev->base_addr + AUDIO_BUFAVAIL);
  return avail * sizeof(int16_t);
}

void audio_set_volume(AudioDevice *dev, int volume) {
  REG32(dev->base_addr + AUDIO_VOLUME) = (uint32_t)volume;
}

void audio_stop(AudioDevice *dev) {
  REG32(dev->base_addr + AUDIO_CTRL) = 0;
  REG32(dev->base_addr + AUDIO_INTCTRL) = 0;

  NVIC_DisableIRQ(dev->irqn);

  dev->state->trans_cb = NULL;
}

void qemu_audio_irq_handler(AudioDevice *dev) {
  // Clear the interrupt
  REG32(dev->base_addr + AUDIO_INTSTAT) = INT_BUFAVAIL;

  // Schedule callback on system task; a drop is retried on the next interrupt
  if (dev->state->trans_cb) {
    bool should_context_switch = false;
    system_task_add_callback_from_isr_droppable(prv_audio_system_task_cb,
                                                (void *)dev->state,
                                                &should_context_switch);
  }
}
