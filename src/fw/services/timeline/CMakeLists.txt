# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  actions_endpoint.c
  alarm_layout.c
  attribute.c
  attribute_group.c
  attributes_actions.c
  calendar.c
  calendar_layout.c
  calendar_layout_resources.c
  event.c
  generic_layout.c
  health_layout.c
  item.c
  layout_layer.c
  layout_node.c
  metricgroup.c
  notification_layout.c
  peek.c
  reminders.c
  sports_layout.c
  swap_layer.c
  timeline.c
  timeline_actions.c
  timeline_layout.c
  timeline_layout_animations.c
  timeline_resources.c
  weather_layout.c
)
pbl_library()
pbl_library_sources(${sources})
pbl_service_gettext(timeline ${sources})
