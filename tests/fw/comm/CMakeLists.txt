# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_ams
  SOURCES
    src/fw/comm/ble/kernel_le_client/ams/ams_util.c
    src/fw/comm/ble/kernel_le_client/ams/ams.c
    src/fw/services/music/service.c
    tests/fakes/fake_events.c
    tests/fakes/fake_gatt_client_operations.c
    tests/fakes/fake_gatt_client_subscriptions.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_ams_util
  SOURCES
    src/fw/comm/ble/kernel_le_client/ams/ams_util.c
)

pbl_clar_test(test_gap_le_advert
  SOURCES
    src/fw/comm/ble/gap_le_advert.c
    src/fw/services/regular_timer/service.c
    tests/fakes/fake_bt_driver_advert.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_bt_conn_mgr
  SOURCES
    src/fw/comm/internals/bt_conn_mgr.c
    src/fw/services/regular_timer/service.c
    tests/fakes/fake_gap_le_connect_params.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_gatt_client_accessors
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/comm/ble/gap_le_connection.c
    src/fw/comm/ble/gatt_client_accessors.c
    src/fw/comm/ble/gatt_client_discovery.c
    src/fw/comm/ble/gatt_service_changed.c
    src/fw/comm/internals/bt_conn_mgr.c
    tests/fakes/fake_events.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_bt_driver_gatt.c
    tests/fakes/fake_gap_le_connect_params.c
)

pbl_clar_test(test_gatt_client_discovery
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/comm/ble/gap_le_connection.c
    src/fw/comm/ble/gatt_client_accessors.c
    src/fw/comm/ble/gatt_client_discovery.c
    src/fw/comm/ble/gatt_service_changed.c
    src/fw/comm/internals/bt_conn_mgr.c
    tests/fakes/fake_gap_le_connect_params.c
    tests/fakes/fake_events.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_bt_driver_gatt.c
)

pbl_clar_test(test_gatt_client_subscriptions
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/comm/ble/gap_le_connection.c
    src/fw/comm/ble/gap_le_task.c
    src/fw/comm/ble/gatt_client_accessors.c
    src/fw/comm/ble/gatt_client_discovery.c
    src/fw/comm/ble/gatt_client_subscriptions.c
    src/fw/comm/ble/gatt_service_changed.c
    src/fw/comm/internals/bt_conn_mgr.c
    tests/fakes/fake_gap_le_connect_params.c
    tests/fakes/fake_events.c
    tests/fakes/fake_bt_driver_gatt.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_BLE_GATT_NOTIF_WRITE_TIMEOUT_MS=100 CONFIG_BLE_GATT_SUBSCRIPTION_DEPTH=4
)

pbl_clar_test(test_gatt_service_changed_client
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/comm/ble/gatt_client_discovery.c
    src/fw/comm/ble/gatt_service_changed.c
    src/fw/comm/internals/bt_conn_mgr.c
    tests/fakes/fake_gap_le_connect_params.c
    tests/fakes/fake_bt_driver_gatt.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_gatt_service_changed_server
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/comm/ble/gap_le_connection.c
    src/fw/comm/ble/gatt_client_discovery.c
    src/fw/comm/ble/gatt_service_changed.c
    src/fw/comm/internals/bt_conn_mgr.c
    tests/fakes/fake_gap_le_connect_params.c
    tests/fakes/fake_bt_driver_gatt.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_ancs_util
  SOURCES
    src/fw/util/time/mktime.c
    src/fw/util/buffer.c
    src/fw/util/time/time.c
    src/fw/comm/ble/kernel_le_client/ancs/ancs_util.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_ancs
  SOURCES
    src/fw/util/buffer.c
    src/fw/util/pstring.c
    src/fw/util/time/mktime.c
    src/fw/util/time/time.c
    src/fw/comm/ble/kernel_le_client/ancs/ancs_util.c
    src/fw/comm/ble/kernel_le_client/ancs/ancs.c
    src/fw/comm/ble/kernel_le_client/ancs/ancs_app_name_storage.c
    src/fw/services/regular_timer/service.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_events.c
    tests/fakes/fake_kernel_services_notifications.c
    tests/fakes/fake_gatt_client_subscriptions.c
    tests/fakes/fake_rtc.c
    src/fw/services/notifications/ancs/ancs_notifications.c
    src/fw/services/notifications/ancs/ancs_notifications_util.c
    src/fw/services/notifications/ancs/ancs_item.c
    src/fw/services/notifications/ancs/ancs_filtering.c
    src/fw/services/notifications/ancs/ancs_phone_call.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attribute.c
)

pbl_clar_test(test_kernel_le_client
  SOURCES
    src/fw/comm/ble/kernel_le_client/kernel_le_client.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
)

pbl_clar_test(test_ppogatt_v0
  SOURCE test_ppogatt.c
  SOURCES
    src/fw/system/hexdump.c
    src/fw/comm/ble/kernel_le_client/ppogatt/ppogatt.c
    src/fw/services/regular_timer/service.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_gatt_client_operations.c
    tests/fakes/fake_gatt_client_subscriptions.c
    tests/fakes/fake_session.c
  DEFINES USE_PPOGATT_VERSION=0
)

pbl_clar_test(test_ppogatt_v1
  SOURCE test_ppogatt.c
  SOURCES
    src/fw/system/hexdump.c
    src/fw/comm/ble/kernel_le_client/ppogatt/ppogatt.c
    src/fw/services/regular_timer/service.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_gatt_client_operations.c
    tests/fakes/fake_gatt_client_subscriptions.c
    tests/fakes/fake_session.c
  DEFINES USE_PPOGATT_VERSION=1
)

# Skipped for now; see pbl_test_broken() in tests/CMakeLists.txt.
pbl_clar_test(test_gap_le_connect
  SOURCES
    src/fw/comm/ble/gap_le_connect.c
    src/fw/comm/ble/gap_le_connection.c
    src/fw/comm/ble/gap_le_task.c
    src/fw/comm/internals/bt_conn_mgr.c
    tests/fakes/fake_events.c
    tests/fakes/fake_GAPAPI.c
    tests/fakes/fake_bluetooth_persistent_storage.c
    tests/fakes/fake_gap_le_connect_params.c
    tests/fakes/fake_HCIAPI.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)
