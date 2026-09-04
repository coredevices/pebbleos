# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  transcription.c
  voice.c
  voice_speex.c
)
pbl_library()
pbl_library_sources(${sources})
