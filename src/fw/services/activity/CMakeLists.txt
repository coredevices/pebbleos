# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  activity.c
  activity_calculators.c
  activity_insights.c
  activity_metrics.c
  activity_sessions.c
  health_util.c
  hr_util.c
  insights_settings.c
  kraepelin/activity_algorithm_kraepelin.c
  kraepelin/kraepelin_algorithm.c
  workout_service.c
)
pbl_library()
pbl_library_sources(${sources})
pbl_service_gettext(activity ${sources})
