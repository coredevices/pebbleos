/*
 * Copyright 2024 Google LLC
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

#include "board/display.h"
#include "applib/graphics/gtypes.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint8_t address;
  uint8_t* data;
} DisplayRow;

typedef bool(*NextRowCallback)(DisplayRow* row);
typedef void(*UpdateCompleteCallback)(void);

//! Show the splash screen before the display has been fully initialized.
void display_show_splash_screen(void);

void display_init(void);

uint32_t display_baud_rate_change(uint32_t new_frequency_hz);

void display_clear(void);

void display_set_enabled(bool enabled);

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb);

bool display_update_in_progress(void);

void display_pulse_vcom(void);

//! Show the panic screen.
//!
//! This function is only defined if the display hardware and driver support it.
void display_show_panic_screen(uint32_t error_code);

typedef struct GPoint GPoint;

void display_set_offset(GPoint offset);

GPoint display_get_offset(void);
