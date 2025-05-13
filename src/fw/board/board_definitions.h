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

// FIXME: PBL-21049 Fix platform abstraction and board definition scheme
#if BOARD_EV2_4
#include "boards/board_ev2_4.h" // shipped as Pebble 1.0
#elif BOARD_BB2
#include "boards/board_bb2.h"
#elif BOARD_V1_5
#include "boards/board_v1_5.h" // prototypes for Pebble 1.3/Pebble 1.5
#elif BOARD_V2_0
#include "boards/board_v2_0.h" // prototypes for Pebble 2.0
#elif BOARD_SNOWY_BB
#include "boards/board_snowy_bb.h" // prototypes for Snowy bigboard
#elif BOARD_SNOWY_EVT
#include "boards/board_snowy_evt.h" // prototypes for Snowy EVT
#elif BOARD_SNOWY_EVT2
#include "boards/board_snowy.h" // prototypes for Snowy EVT2
#elif BOARD_SNOWY_BB2
#include "boards/board_snowy.h" // prototypes for Snowy BB2 are identical to EVT2
#elif BOARD_SNOWY_DVT
#include "boards/board_snowy.h" // prototypes for DVT, electrically identical to EVT2
#elif BOARD_SPALDING_BB2
#include "boards/board_snowy.h" // prototypes for Spalding BB2, Snowy BB2s with a Spalding display
#elif BOARD_SPALDING_EVT
#include "boards/board_spalding_evt.h" // prototypes for Spalding EVT
#elif BOARD_SPALDING
#include "boards/board_spalding_evt.h" // prototypes for Spalding MP
#elif BOARD_SNOWY_S3
#include "boards/board_snowy.h" // prototypes for Spalding EVT, electrically identical to Snowy
#elif BOARD_SILK_EVT
#include "boards/board_silk.h"
#elif BOARD_SILK_BB
#include "boards/board_silk.h"
#elif BOARD_SILK_BB2
#include "boards/board_silk.h"
#elif BOARD_SILK
#include "boards/board_silk.h"
#elif BOARD_CUTTS_BB
#include "boards/board_robert.h" // prototypes for Cutts
#elif BOARD_ROBERT_BB
#include "boards/board_robert.h" // prototypes for Robert BB
#elif BOARD_ROBERT_BB2
#include "boards/board_robert.h" // prototypes for Robert BB2
#elif BOARD_ROBERT_EVT
#include "boards/board_robert.h" // prototypes for Robert EVT
#elif BOARD_ASTERIX
#include "boards/board_asterix.h"
#elif BOARD_OBELIX
#include "boards/board_obelix.h"
#else
#error "Unknown board definition"
#endif
