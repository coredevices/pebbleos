# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Empty a directory: cmake -DDIR=<path> -P reset_dir.cmake

file(REMOVE_RECURSE ${DIR})
file(MAKE_DIRECTORY ${DIR})
