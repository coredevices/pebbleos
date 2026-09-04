/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

// All graphics/UI includes needed for tests. Add here if more are needed.
#include "applib/ui/layer.h"
#include "applib/ui/window.h"
#include "applib/ui/menu_layer.h"

#include <stdint.h>

//! GFX test struct
typedef struct GfxTest {
  char *name;                         //!< Name string
  uint32_t duration;                  //!< Number of seconds to run the test for
  uint32_t unit_multiple;             //!< Number of actions per test iteration
  LayerUpdateProc test_proc;          //!< Test procedure
  void (*setup)(Window *window);      //!< Test setup function
  void (*teardown)( Window *window);  //!< Test teardown function
} GfxTest;

#include "process_management/pebble_process_md.h"

const PebbleProcessMd* gfx_tests_get_app_info(void);
