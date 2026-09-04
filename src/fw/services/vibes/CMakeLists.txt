# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  vibe_client.c
  vibe_intensity.c
  vibe_score.c
  vibe_score_info.c
)
pbl_library()
pbl_library_sources(${sources})
pbl_service_gettext(vibes ${sources})
