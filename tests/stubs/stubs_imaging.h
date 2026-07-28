/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/imaging.h"
#include "pbl/util/attributes.h"

void WEAK imaging_register_handler(ImagingImageType image_type, ImagingReceivedHandler handler) {}

bool WEAK imaging_is_supported(void) {
  return false;
}

bool WEAK imaging_request_album_art(uint8_t token, ImagingFormat format, uint16_t width,
                                    uint16_t height, const char *title, const char *artist) {
  return false;
}
