/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "qr_code.h"

#include <string.h>

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/graphics.h"
#include "system/passert.h"

#include <qrcodegen.h>

static inline enum qrcodegen_Ecc prv_ecc_to_qrcodegen(QRCodeECC ecc) {
  switch (ecc) {
    case QRCodeECCLow:
      return qrcodegen_Ecc_LOW;
    case QRCodeECCMedium:
      return qrcodegen_Ecc_MEDIUM;
    case QRCodeECCQuartile:
      return qrcodegen_Ecc_QUARTILE;
    case QRCodeECCHigh:
      return qrcodegen_Ecc_HIGH;
    default:
      return qrcodegen_Ecc_MEDIUM;
  }
}

static void prv_qr_code_update_proc(QRCode *qr_code, GContext *ctx) {
  uint8_t *qr_code_buf;
  uint8_t *tmp_buf;
  int qr_size;
  int mod_size;
  int rend_size;
  int offset_x;
  int offset_y;
  GColor old_fill_color;
  bool ret;

  if ((qr_code->data == NULL) || (qr_code->data_len == 0U) ||
      (qr_code->layer.bounds.size.w <= 0) || (qr_code->layer.bounds.size.h <= 0)) {
    return;
  }

  // NOTE: using maximum buffer length as we use qrcodegen_VERSION_MAX.
  // We could potentially optimize this by calculating the minimum required version
  // for the given input. LVGL does by adding some extra APIs to qrcodegen.
  qr_code_buf = applib_malloc(qrcodegen_BUFFER_LEN_MAX);
  if (qr_code_buf == NULL) {
    return;
  }

  tmp_buf = applib_malloc(qrcodegen_BUFFER_LEN_MAX);
  if (tmp_buf == NULL) {
    applib_free(qr_code_buf);
    return;
  }

  memcpy(tmp_buf, qr_code->data, qr_code->data_len);

  ret = qrcodegen_encodeBinary(tmp_buf, qr_code->data_len, qr_code_buf,
                               prv_ecc_to_qrcodegen(qr_code->ecc), qrcodegen_VERSION_MIN,
                               qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
  if (!ret) {
    applib_free(tmp_buf);
    applib_free(qr_code_buf);
    return;
  }

  // Compute module size
  qr_size = qrcodegen_getSize(qr_code_buf);
  mod_size = MIN(qr_code->layer.bounds.size.w / qr_size, qr_code->layer.bounds.size.h / qr_size);
  if (mod_size == 0) {
    applib_free(tmp_buf);
    applib_free(qr_code_buf);
    return;
  }

  // Calculate actual rendered size
  rend_size = qr_size * mod_size;

  // Center the QR code
  offset_x = qr_code->layer.bounds.origin.x + (qr_code->layer.bounds.size.w - rend_size) / 2;
  offset_y = qr_code->layer.bounds.origin.y + (qr_code->layer.bounds.size.h - rend_size) / 2;

  // Save current context state
  old_fill_color = ctx->draw_state.fill_color;

  // Draw background
  graphics_context_set_fill_color(ctx, qr_code->bg_color);
  graphics_fill_rect(ctx, &qr_code->layer.bounds);

  // Draw QR code modules
  graphics_context_set_fill_color(ctx, qr_code->fg_color);

  for (int y = 0; y < qr_size; y++) {
    for (int x = 0; x < qr_size; x++) {
      if (qrcodegen_getModule(qr_code_buf, x, y)) {
        GRect module_rect;

        module_rect = GRect(offset_x + x * mod_size, offset_y + y * mod_size, mod_size, mod_size);

        graphics_fill_rect(ctx, &module_rect);
      }
    }
  }

  // Restore context state
  graphics_context_set_fill_color(ctx, old_fill_color);

  // Free buffers
  applib_free(qr_code_buf);
  applib_free(tmp_buf);
}

void qr_code_init_with_parameters(QRCode *qr_code, const GRect *frame, const void *data,
                                  size_t data_len, QRCodeECC ecc, GColor fg_color, GColor bg_color) {
  PBL_ASSERTN(qr_code);
  *qr_code = (QRCode){};
  qr_code->layer.frame = *frame;
  qr_code->layer.bounds = (GRect){{0, 0}, frame->size};
  qr_code->layer.update_proc = (LayerUpdateProc)prv_qr_code_update_proc;
  qr_code->data = data;
  qr_code->data_len = data_len;
  qr_code->ecc = ecc;
  qr_code->fg_color = fg_color;
  qr_code->bg_color = bg_color;
  layer_set_clips(&qr_code->layer, true);
  
  layer_mark_dirty(&qr_code->layer);
}

void qr_code_init(QRCode *qr_code, const GRect *frame) {
  qr_code_init_with_parameters(qr_code, frame, NULL, 0, QRCodeECCMedium, 
                               GColorBlack, GColorWhite);
}

QRCode* qr_code_create(GRect frame) {
  QRCode* qr_code = applib_type_malloc(QRCode);
  if (qr_code) {
    qr_code_init(qr_code, &frame);
  }
  return qr_code;
}

void qr_code_destroy(QRCode* qr_code) {
  if (qr_code) {
    applib_free(qr_code);
  }
}

void qr_code_set_data(QRCode *qr_code, const void *data, size_t data_len) {
  PBL_ASSERTN(qr_code);
  qr_code->data = data;
  qr_code->data_len = data_len;
  layer_mark_dirty(&qr_code->layer);
}

void qr_code_set_ecc(QRCode *qr_code, QRCodeECC ecc) {
  PBL_ASSERTN(qr_code);
  qr_code->ecc = ecc;
  layer_mark_dirty(&qr_code->layer);
}

void qr_code_set_bg_color(QRCode *qr_code, GColor color) {
  PBL_ASSERTN(qr_code);
  qr_code->bg_color = color;
  layer_mark_dirty(&qr_code->layer);
}

void qr_code_set_fg_color(QRCode *qr_code, GColor color) {
  PBL_ASSERTN(qr_code);
  qr_code->fg_color = color;
  layer_mark_dirty(&qr_code->layer);
}