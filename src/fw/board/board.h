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

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF52840_COMPATIBLE
#define SF32LB52_COMPATIBLE
#include <mcu.h>

#if defined(MICRO_FAMILY_STM32F2)
# include "board_stm32.h"
#elif defined(MICRO_FAMILY_STM32F4)
# include "board_stm32.h"
#elif defined(MICRO_FAMILY_STM32F7)
# include "board_stm32.h"
#elif defined(MICRO_FAMILY_NRF52840)
# include "board_nrf5.h"
#elif defined(MICRO_FAMILY_SF32LB52)
# include "board_sf32lb52.h"
#elif !defined(SDK) && !defined(UNITTEST)
# error "Unknown or missing MICRO_FAMILY_* define"
#endif
