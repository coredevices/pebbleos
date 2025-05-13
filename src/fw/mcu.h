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

//! \file
//! MCU wrapper header
//!
//! Include this header in firmware sources instead of including the header for
//! a specific MCU family.
//!
//! Before including this header, a compatibility matrix must be declared by
//! `#define`-ing the appropriate `_COMPATIBLE` macros.
//!
//! Define `CMSIS_COMPATIBLE` if the source file only requires symbols and
//! macros which are defined by CMSIS.
//!
//! Define `STM32F2_COMPATIBLE` and/or `STM32F4_COMPATIBLE` if the source file
//! is compatible with the STM32F2 or STM32F4 microcontroller families,
//! respectively, and requires the corresponding family's peripheral
//! definitions or library functions.

// Multiple inclusion of this header is allowed.

#if defined(MICRO_FAMILY_STM32F2)
# if !defined(STM32F2_COMPATIBLE) && !defined(CMSIS_COMPATIBLE)
#  error "Source is incompatible with the target MCU"
# endif
# include <stm32f2xx.h>
#elif defined(MICRO_FAMILY_STM32F4)
# if !defined(STM32F4_COMPATIBLE) && !defined(CMSIS_COMPATIBLE)
#  error "Source is incompatible with the target MCU"
# endif
# include <stm32f4xx.h>
#elif defined(MICRO_FAMILY_STM32F7)
# if !defined(STM32F7_COMPATIBLE) && !defined(CMSIS_COMPATIBLE)
#  error "Source is incompatible with the target MCU"
# endif
# include <stm32f7xx.h>
#elif defined(MICRO_FAMILY_NRF52840)
# if !defined(NRF52840_COMPATIBLE) && !defined(CMSIS_COMPATIBLE) && !defined(NRF5_COMPATIBLE)
#  error "Source is incompatible with the target MCU"
# endif
# pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wmissing-declarations"
#  ifdef UNUSED
/* bleh */
#   define HAD_UNUSED
#   undef UNUSED
#  endif
#  include <nrf52840.h>
#  ifdef HAD_UNUSED
#   define UNUSED __attribute__((__unused__))
#   undef HAD_UNUSED
#  endif
# pragma GCC diagnostic pop
#elif defined(MICRO_FAMILY_SF32LB52)
# if !defined(SF32LB52_COMPATIBLE) && !defined(CMSIS_COMPATIBLE)
#  error "Source is incompatible with the target MCU"
# endif
# ifdef UNUSED
#  undef UNUSED
# endif
# include <bf0_hal.h>
# undef UNUSED
# define UNUSED __attribute__((__unused__))
#elif !defined(SDK) && !defined(UNITTEST)
# error "Unknown or missing MICRO_FAMILY_* define"
#endif

#undef CMSIS_COMPATIBLE
#undef STM32F2_COMPATIBLE
#undef STM32F4_COMPATIBLE
#undef STM32F7_COMPATIBLE
#undef NRF52840_COMPATIBLE
#undef NRF5_COMPATIBLE
#undef SF32LB52_COMPATIBLE
