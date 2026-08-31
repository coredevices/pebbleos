/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/recognizer/recognizer.h"

//! Attach a recognizer to the app
//! @param recognizer \ref Recognizer to attach
void app_recognizers_attach_recognizer(Recognizer *recognizer);

//! Detach a recognizer from the app
//! @param recognizer \ref Recognizer to detach
void app_recognizers_detach_recognizer(Recognizer *recognizer);
