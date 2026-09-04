# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  compositor.c
  compositor_display.c
  compositor_transitions.c
  screenshot_pp.c
  default/compositor_dot_transitions.c
  default/compositor_launcher_app_transitions.c
  default/compositor_peek_transitions.c
  default/compositor_port_hole_transitions.c
  default/compositor_round_flip_transitions.c
  default/compositor_shutter_transitions.c
  default/compositor_slide_transitions.c
  legacy/compositor_app_slide_transitions.c
)
if(CONFIG_SCREEN_COLOR_DEPTH_BITS EQUAL 1)
  list(APPEND sources legacy/compositor_modal_slide_transitions.c)
else()
  list(APPEND sources default/compositor_modal_transitions.c)
endif()
pbl_library()
pbl_library_sources(${sources})
