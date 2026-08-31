/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pulse_bulkio_domain_handler.h"

#include "applib/graphics/framebuffer.h"
#include "applib/ui/animation_private.h"
#include "kernel/event_loop.h"
#include "pbl/services/compositor/compositor.h"
#include "pbl/services/compositor/compositor_display.h"
#include "system/status_codes.h"
#include "pbl/util/attributes.h"

#include <stdint.h>
#include <string.h>


typedef struct PACKED FramebufferStatResp {
  uint8_t flags;
  uint16_t width;
  uint16_t height;
  uint8_t bpp;
  uint32_t length;
} FramebufferStatResp;

static int framebuffer_domain_read(uint8_t *buf, uint32_t address, uint32_t length,
                                        void *context) {
  uint8_t *fb_offset = (uint8_t*)compositor_get_framebuffer()->buffer + address;
  memcpy(buf, fb_offset, length);
  return length;
}

static int framebuffer_domain_write(uint8_t *buf, uint32_t address, uint32_t length,
                                         void *context) {
  uint8_t *fb_offset = (uint8_t*)compositor_get_framebuffer()->buffer + address;
  memcpy(fb_offset, buf, length);
  return length;
}

static int framebuffer_domain_stat(uint8_t *resp, size_t resp_max_len, void *context) {
  FramebufferStatResp *stat_resp = (FramebufferStatResp*) resp;
  *stat_resp = (FramebufferStatResp) {
    .flags = 0,
    .length = FRAMEBUFFER_SIZE_BYTES,
    .width = DISP_COLS,
    .height = DISP_ROWS,
    .bpp = CONFIG_SCREEN_COLOR_DEPTH_BITS
  };

  return sizeof(FramebufferStatResp);
}

static status_t framebuffer_domain_erase(uint8_t *packet_data, size_t length, uint8_t cookie) {
  return E_INVALID_OPERATION;
}

static status_t framebuffer_domain_open(uint8_t *packet_data, size_t length, void **resp) {
  animation_private_pause();
  return S_SUCCESS;
}

static void framebuffer_domain_close_cb(void *foo) {
  FrameBuffer *fb = compositor_get_framebuffer();
  framebuffer_dirty_all(fb);
  compositor_display_update(NULL);
}

static status_t framebuffer_domain_close(void* data) {
  animation_private_resume();

  // Force the compositor to redraw the framebuffer
  launcher_task_add_callback(framebuffer_domain_close_cb, NULL);

  return S_SUCCESS;
}

PulseBulkIODomainHandler pulse_bulkio_domain_framebuffer = {
  .id = PulseBulkIODomainType_Framebuffer,
  .open_proc = framebuffer_domain_open,
  .close_proc = framebuffer_domain_close,
  .read_proc = framebuffer_domain_read,
  .write_proc = framebuffer_domain_write,
  .stat_proc = framebuffer_domain_stat,
  .erase_proc = framebuffer_domain_erase
};
