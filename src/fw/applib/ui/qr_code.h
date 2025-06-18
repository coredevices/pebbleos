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

#pragma once

#include <stddef.h>

#include "applib/graphics/gtypes.h"
#include "applib/ui/layer.h"

//! @file qr_code.h
//! @addtogroup UI
//! @{
//!   @addtogroup QR Code
//!   @{

//! QR Code error correction levels
typedef enum {
  //! Low error correction level (7% recovery capability)
  QRCodeECCLow = 0,
  //! Medium error correction level (15% recovery capability)
  QRCodeECCMedium = 1,
  //! Quartile error correction level (25% recovery capability)
  QRCodeECCQuartile = 2,
  //! High error correction level (30% recovery capability)
  QRCodeECCHigh = 3
} QRCodeECC;

//! QR code structure
typedef struct QRCode {
  //! Layer
  Layer layer;
  //! QR code data buffer
  const void *data;
  //! Size of the QR code (number of modules per side)
  size_t data_len;
  //! Error correction level used
  QRCodeECC ecc;
  //! Foreground color of the QR code
  GColor fg_color;
  //! Background color of the QR code
  GColor bg_color;
} QRCode;

//! Initializes the QRCode with given frame
//! All previous contents are erased and the following default values are set:
//!
//! * Empty data
//! * ECC: \ref QRCodeECCMedium
//! * Foreground color: \ref GColorBlack
//! * Background color: \ref GColorWhite
//!
//! The QR code is automatically marked dirty after this operation.
//! @param qr_code The QRCode to initialize
//! @param frame The frame with which to initialze the QRCode
void qr_code_init(QRCode *qr_code, const GRect *frame);

//! Creates a new QRCode on the heap and initializes it with the default values.
//!
//! * Empty data
//! * ECC: \ref QRCodeECCMedium
//! * Foreground color: \ref GColorBlack
//! * Background color: \ref GColorWhite
//!
//! @param frame The frame with which to initialze the QRCode
//! @return A pointer to the QRCode. `NULL` if the QRCode could not be created
QRCode* qr_code_create(GRect frame);

//! Destroys a QRCode previously created by qr_code_create.
void qr_code_destroy(QRCode* qr_code);

//! Sets the pointer to the data where the QRCode is supposed to find the data
//! at a later point in time, when it needs to draw itself.
//! @param qr_code The QRCode of which to set the text
//! @param data The new data to set onto the QRCode.
//! @param data_len Length of the data in bytes
//! @note The data is not copied, so its buffer most likely cannot be stack allocated,
//! but is recommended to be a buffer that is long-lived, at least as long as the QRCode
//! is part of a visible Layer hierarchy.
//! @see qr_code_get_text
void qr_code_set_data(QRCode *qr_code, const void *data, size_t data_len);

//! Sets the error correction level of the QR code
//! @param qr_code The QRCode of which to set the error correction level
//! @param ecc The new \ref QRCodeECC to set the error correction level to
void qr_code_set_ecc(QRCode *qr_code, QRCodeECC ecc);

//! Sets the background color of the QR code
//! @param qr_code The QRCode of which to set the background color
//! @param color The new \ref GColor to set the background to
//! @see qr_code_set_fg_color
void qr_code_set_bg_color(QRCode *qr_code, GColor color);

//! Sets the foreground color of the QR code
//! @param qr_code The QRCode of which to set the foreground color
//! @param color The new \ref GColor to set the foreground color to
//! @see qr_code_set_bg_color
void qr_code_set_fg_color(QRCode *qr_code, GColor color);

//! @internal
void qr_code_init_with_parameters(QRCode *qr_code, const GRect *frame, const void *data,
                                  size_t data_len, QRCodeECC ecc, GColor fg_color, GColor bg_color);

//!   @} // end addtogroup QR Code
//! @} // end addtogroup UI