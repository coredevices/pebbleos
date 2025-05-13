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

typedef enum PlatformType {
  PlatformTypeAplite,
  PlatformTypeBasalt,
  PlatformTypeChalk,
  PlatformTypeDiorite,
  PlatformTypeEmery,
} PlatformType;

// Unit tests and the firmware don't define the SDK platform defines because reasons.
// Therefore, we need to switch on the platform for the platform type.
#if !defined(SDK)
  #if PLATFORM_TINTIN
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeAplite
  #elif PLATFORM_SNOWY
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeBasalt
  #elif PLATFORM_SPALDING
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeChalk
  #elif PLATFORM_SILK || PLATFORM_ASTERIX || PLATFORM_OBELIX
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeDiorite
  #elif PLATFORM_ROBERT || PLATFORM_CALCULUS
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeEmery
  #else
    #error "PBL_PLATFORM_TYPE_CURRENT couldn't be determined: No PLATFORM_* defined!"
  #endif
#else
  #if PBL_PLATFORM_APLITE
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeAplite
  #elif PBL_PLATFORM_BASALT
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeBasalt
  #elif PBL_PLATFORM_CHALK
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeChalk
  #elif PBL_PLATFORM_DIORITE
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeDiorite
  #elif PBL_PLATFORM_EMERY
    #define PBL_PLATFORM_TYPE_CURRENT PlatformTypeEmery
  #else
    #error "PBL_PLATFORM_TYPE_CURRENT couldn't be determined: No PBL_PLATFORM_* defined!"
  #endif
#endif

#define PBL_PLATFORM_SWITCH_DEFAULT(PLAT, DEFAULT, APLITE, BASALT, CHALK, DIORITE, EMERY) (\
  ((PLAT) == PlatformTypeEmery) ? (EMERY) : \
  ((PLAT) == PlatformTypeDiorite) ? (DIORITE) : \
  ((PLAT) == PlatformTypeChalk) ? (CHALK) : \
  ((PLAT) == PlatformTypeBasalt) ? (BASALT) : \
  ((PLAT) == PlatformTypeBasalt) ? (APLITE) : \
  (DEFAULT) \
)

// We fall back to Aplite because we need to fall back on _one_ of the given arguments.
// This prevents issues with sometimes using this for pointers/strings, and sometimes for ints.
//
// NOTE: Optimal use of this does _not_ call a function for the `PLAT` argument! If you do, it
//       will be _evaluated on every comparison_, which is unlikely to be what you want!
#define PBL_PLATFORM_SWITCH(PLAT, APLITE, BASALT, CHALK, DIORITE, EMERY) \
  PBL_PLATFORM_SWITCH_DEFAULT(PLAT, APLITE, APLITE, BASALT, CHALK, DIORITE, EMERY)


// INTERNAL
#define platform_type_get_name(plat) PBL_PLATFORM_SWITCH_DEFAULT(plat, \
  /*default*/ "unknown", \
  /*aplite*/ "aplite", \
  /*basalt*/ "basalt", \
  /*chalk*/ "chalk", \
  /*diorite*/ "diorite", \
  /*emery*/ "emery")
