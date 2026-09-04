# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_recognizer
  SOURCES
    src/fw/applib/ui/recognizer/recognizer.c
    tests/fw/ui/recognizer/test_recognizer_impl.c
)

pbl_clar_test(test_recognizer_manager
  SOURCES
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/ui/layer.c
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/recognizer_manager.c
    tests/fw/ui/recognizer/test_recognizer_impl.c
  DEFINES CONFIG_TOUCH
)

pbl_clar_test(test_tap
  SOURCES
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/tap.c
    tests/fw/ui/recognizer/test_recognizer_impl.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_TOUCH
)

pbl_clar_test(test_pan
  SOURCES
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/pan.c
    tests/fw/ui/recognizer/test_recognizer_impl.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_TOUCH
)

pbl_clar_test(test_swipe
  SOURCES
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/swipe.c
    tests/fw/ui/recognizer/test_recognizer_impl.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_TOUCH
)

pbl_clar_test(test_recognizer_competition
  SOURCES
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/ui/layer.c
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/recognizer_manager.c
    src/fw/applib/ui/recognizer/pan.c
    src/fw/applib/ui/recognizer/swipe.c
    src/fw/applib/ui/recognizer/tap.c
    tests/fw/ui/recognizer/test_recognizer_impl.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_TOUCH
)

pbl_clar_test(test_recognizer_lifecycle
  SOURCES
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/graphics/gcolor_definitions.c
    src/fw/applib/ui/layer.c
    src/fw/applib/ui/window.c
    src/fw/applib/ui/window_manager.c
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/recognizer_manager.c
    tests/fw/ui/recognizer/test_recognizer_impl.c
  DEFINES CONFIG_TOUCH
)

pbl_clar_test(test_touch_nav
  SOURCES
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/ui/layer.c
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/recognizer_manager.c
    src/fw/applib/ui/recognizer/pan.c
    src/fw/applib/ui/recognizer/swipe.c
    src/fw/applib/ui/recognizer/tap.c
    src/fw/applib/ui/recognizer/touch_nav.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_TOUCH
)
