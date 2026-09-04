/* SPDX-FileCopyrightText: 2025 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/gatt.h>
#include <comm/bt_lock.h>
#include <host/ble_hs.h>
#include <pbl/logging/logging.h>
#include <pbl/util/math.h>

#include <services/gatt/ble_svc_gatt.h>

#include <semphr.h>

#include "nimble_gattc_op_queue.h"
#include "nimble_type_conversions.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

static bool s_discovery_in_progress;
static bool s_stop_discovery_requested;
static SemaphoreHandle_t s_discovery_stopped;

//! Marks the end of a discovery run, letting the next queued GATT client
//! procedure start. Every terminal path of the discovery chain must end here.
static void prv_discovery_finished(void) {
  s_discovery_in_progress = false;
  nimble_gattc_op_queue_complete();
}

// -------------------------------------------------------------------------------------------------
// Gatt Client Discovery API calls

typedef struct {
  ListNode node;
  struct ble_gatt_dsc descriptor;
} GATTServiceDiscoveryDescriptorNode;

typedef struct {
  ListNode node;
  struct ble_gatt_chr characteristic;
  ListNode *descriptors;
} GATTServiceDiscoveryCharacteristicNode;

typedef struct {
  ListNode node;
  struct ble_gatt_svc service;
  ListNode *characteristics;
  uint16_t num_descriptors;
} GATTServiceDiscoveryServiceNode;

typedef struct {
  GAPLEConnection *connection;
  ATTHandleRange range;
  ListNode *services;
  ListNode *current_service;
  ListNode *current_characteristic;
} GATTServiceDiscoveryContext;

static bool prv_descriptor_free_cb(ListNode *node, void *context) {
  kernel_free(node);
  return true;
}

static bool prv_characteristic_free_cb(ListNode *node, void *context) {
  GATTServiceDiscoveryCharacteristicNode *chr_node = (GATTServiceDiscoveryCharacteristicNode *)node;
  list_foreach(chr_node->descriptors, prv_descriptor_free_cb, NULL);
  kernel_free(node);
  return true;
}

static bool prv_service_free_cb(ListNode *node, void *context) {
  GATTServiceDiscoveryServiceNode *service_node = (GATTServiceDiscoveryServiceNode *)node;
  list_foreach(service_node->characteristics, prv_characteristic_free_cb, NULL);
  kernel_free(node);
  return true;
}

static void prv_free_discovery_context(GATTServiceDiscoveryContext *context) {
  list_foreach(context->services, prv_service_free_cb, NULL);
  kernel_free(context);
}

/* TODO: the way this works is kinda inefficient, really we should notify the OS after we
 discover all the characteristics/descriptors for a service, letting us free the allocations
 related to that service.
 */
static bool prv_convert_service_and_notify_os_cb(ListNode *node, void *context) {
  GATTServiceDiscoveryServiceNode *service_node = (GATTServiceDiscoveryServiceNode *)node;
  GAPLEConnection *connection = context;

  uint16_t num_characteristics = list_count(service_node->characteristics);
  size_t size_bytes =
      COMPUTE_GATTSERVICE_SIZE_BYTES(num_characteristics, service_node->num_descriptors, 0);
  GATTService *gatt_service = kernel_zalloc_check(size_bytes);
  gatt_service->size_bytes = size_bytes;
  gatt_service->att_handle = service_node->service.start_handle;
  gatt_service->num_characteristics = num_characteristics;
  gatt_service->num_descriptors = service_node->num_descriptors;
  gatt_service->num_att_handles_included_services = 0;
  nimble_uuid_to_pebble(&service_node->service.uuid, &gatt_service->uuid);

  GATTServiceDiscoveryCharacteristicNode *chr_node =
      (GATTServiceDiscoveryCharacteristicNode *)service_node->characteristics;
  uint8_t *end_ptr = (uint8_t *)gatt_service->characteristics;
  while (chr_node != NULL) {
    GATTCharacteristic *gatt_characteristic = (GATTCharacteristic *)end_ptr;
    *gatt_characteristic = (GATTCharacteristic){
        .att_handle_offset = chr_node->characteristic.val_handle - gatt_service->att_handle,
        .properties = chr_node->characteristic.properties,
        .num_descriptors = 0,
    };
    nimble_uuid_to_pebble(&chr_node->characteristic.uuid, &gatt_characteristic->uuid);

    GATTServiceDiscoveryDescriptorNode *dsc_node =
        (GATTServiceDiscoveryDescriptorNode *)chr_node->descriptors;
    uint16_t dsc_index = 0;
    while (dsc_node != NULL) {
      GATTDescriptor *gatt_descriptor = &gatt_characteristic->descriptors[dsc_index];
      *gatt_descriptor = (GATTDescriptor){
          .att_handle_offset = dsc_node->descriptor.handle - gatt_service->att_handle,
      };
      nimble_uuid_to_pebble(&dsc_node->descriptor.uuid, &gatt_descriptor->uuid);

      dsc_index++;
      dsc_node = (GATTServiceDiscoveryDescriptorNode *)list_get_next(&dsc_node->node);
      end_ptr += sizeof(GATTDescriptor);
    }

    gatt_characteristic->num_descriptors = dsc_index;
    end_ptr += sizeof(GATTCharacteristic);
    chr_node = (GATTServiceDiscoveryCharacteristicNode *)list_get_next(&chr_node->node);
  }

  char service_uuid_str[UUID_STRING_BUFFER_LENGTH];
  uuid_to_string(&gatt_service->uuid, service_uuid_str);
  bt_driver_cb_gatt_client_discovery_handle_indication(connection, gatt_service, BTErrnoOK);

  return true;
}

static bool prv_find_svc_by_uuid(ListNode *node, void *data) {
  GATTServiceDiscoveryServiceNode *service_node = (GATTServiceDiscoveryServiceNode *)node;
  const ble_uuid_t *svc_uuid = (const ble_uuid_t *)data;
  return ble_uuid_cmp(&service_node->service.uuid.u, svc_uuid) == 0;
}

static bool prv_find_chr_by_uuid(ListNode *node, void *data) {
  GATTServiceDiscoveryCharacteristicNode *chr_node = (GATTServiceDiscoveryCharacteristicNode *)node;
  const ble_uuid_t *chr_uuid = (const ble_uuid_t *)data;
  return ble_uuid_cmp(&chr_node->characteristic.uuid.u, chr_uuid) == 0;
}

static bool prv_find_dsc_by_uuid(ListNode *node, void *data) {
  GATTServiceDiscoveryDescriptorNode *dsc_node = (GATTServiceDiscoveryDescriptorNode *)node;
  const ble_uuid_t *dsc_uuid = (const ble_uuid_t *)data;
  return ble_uuid_cmp(&dsc_node->descriptor.uuid.u, dsc_uuid) == 0;
}

static bool prv_find_dsc_uuid(GATTServiceDiscoveryContext *context,
                              const ble_uuid_t *svc_uuid,
                              const ble_uuid_t *chr_uuid,
                              const ble_uuid_t *dsc_uuid,
                              uint16_t *chr_handle,
                              uint16_t *dsc_handle) {
  GATTServiceDiscoveryServiceNode *service_node = (GATTServiceDiscoveryServiceNode *)list_find(
      context->services, prv_find_svc_by_uuid, (void *)svc_uuid);
  if (service_node == NULL) {
    return false;
  }

  GATTServiceDiscoveryCharacteristicNode *chr_node =
      (GATTServiceDiscoveryCharacteristicNode *)list_find(service_node->characteristics,
                                                          prv_find_chr_by_uuid, (void *)chr_uuid);
  if (chr_node == NULL) {
    return false;
  }

  GATTServiceDiscoveryDescriptorNode *dsc_node = (GATTServiceDiscoveryDescriptorNode *)list_find(
      chr_node->descriptors, prv_find_dsc_by_uuid, (void *)dsc_uuid);
  if (dsc_node == NULL) {
    return false;
  }

  *chr_handle = chr_node->characteristic.val_handle;
  *dsc_handle = dsc_node->descriptor.handle;

  return true;
}

typedef struct {
  GAPLEConnection *connection;
  uint16_t chr_handle;
} GATTServiceDiscoveryDescriptorContext;

static int prv_on_svc_chgd_subscribe(uint16_t conn_handle, const struct ble_gatt_error *error,
                                     struct ble_gatt_attr *attr, void *arg) {
  if (error->status != 0) {
    PBL_LOG_ERR("Failed to subscribe to service changed: 0x%" PRIx16,
              error->status);
  } else {
    GATTServiceDiscoveryDescriptorContext *ctx = arg;
    bt_driver_cb_gatt_client_discovery_handle_service_changed(ctx->connection,
                                                              ctx->chr_handle);

    PBL_LOG_DBG("Subscribed to service changed");
  }

  kernel_free(arg);

  return 0;
}

static void prv_convert_service_and_notify_os(uint16_t conn_handle, GATTServiceDiscoveryContext *context) {
  list_foreach(context->services, prv_convert_service_and_notify_os_cb, context->connection);
  bt_driver_cb_gatt_client_discovery_complete(context->connection, BTErrnoOK);

  // Subscribe to service changed indications (BLE Core 6.0, part G 7.7.1)
  uint16_t chr_handle, dsc_handle;
  if (prv_find_dsc_uuid(context, BLE_UUID16_DECLARE(BLE_GATT_SVC_UUID16),
                        BLE_UUID16_DECLARE(BLE_SVC_GATT_CHR_SERVICE_CHANGED_UUID16),
                        BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16),
                        &chr_handle, &dsc_handle)) {
    uint16_t value;
    int ret;

    GATTServiceDiscoveryDescriptorContext *ctx = kernel_zalloc_check(
        sizeof(GATTServiceDiscoveryDescriptorContext));
    ctx->connection = context->connection;
    ctx->chr_handle = chr_handle;

    value = 0x0002;
    ret = ble_gattc_write_flat(conn_handle, dsc_handle,
                               &value, sizeof(value), prv_on_svc_chgd_subscribe, ctx);
    if (ret != 0) {
        PBL_LOG_ERR("Failed to subscribe to service changed: %d", ret);
    }
  }

  prv_free_discovery_context(context);
}

static uint16_t prv_get_last_dsc_handle(GATTServiceDiscoveryContext *context) {
  const GATTServiceDiscoveryCharacteristicNode *next_chr =
      (GATTServiceDiscoveryCharacteristicNode *)list_get_next(context->current_characteristic);

  if (next_chr != NULL) {
    return MIN(next_chr->characteristic.def_handle, next_chr->characteristic.val_handle) - 1;
  } else {
    return ((GATTServiceDiscoveryServiceNode *)context->current_service)->service.end_handle;
  }
}

static int prv_find_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
static int prv_find_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg);

static void prv_discover_next_dscs(uint16_t conn_handle, GATTServiceDiscoveryContext *context) {
  GATTServiceDiscoveryCharacteristicNode *chr_node =
      (GATTServiceDiscoveryCharacteristicNode *)context->current_characteristic;

  uint16_t start_handle =
      MIN(chr_node->characteristic.val_handle, chr_node->characteristic.def_handle);
  uint16_t end_handle = prv_get_last_dsc_handle(context);
  int rc = ble_gattc_disc_all_dscs(conn_handle, start_handle, end_handle, prv_find_dsc_cb, context);
  if (rc != 0) {
    PBL_LOG_ERR("ble_gattc_disc_all_dscs rc=0x%04x (0x%04x -> 0x%04x)",
              (uint16_t)rc, start_handle, end_handle);
  }
}

static void prv_discover_next_chrs(uint16_t conn_handle, GATTServiceDiscoveryContext *context) {
  GATTServiceDiscoveryServiceNode *service_node =
      (GATTServiceDiscoveryServiceNode *)context->current_service;
  int rc = ble_gattc_disc_all_chrs(conn_handle, service_node->service.start_handle,
                                   service_node->service.end_handle, prv_find_chr_cb, context);
  if (rc != 0) {
    PBL_LOG_ERR("ble_gattc_disc_all_chrs rc=0x%04x (0x%04x -> 0x%04x)",
              (uint16_t)rc, service_node->service.start_handle, service_node->service.end_handle);
  }
}

static GATTServiceDiscoveryDescriptorNode *prv_create_descriptor_node(
    const struct ble_gatt_dsc *dsc) {
  GATTServiceDiscoveryDescriptorNode *dsc_node =
      kernel_zalloc_check(sizeof(GATTServiceDiscoveryDescriptorNode));
  list_init(&dsc_node->node);
  dsc_node->descriptor = *dsc;
  return dsc_node;
}

static GATTServiceDiscoveryCharacteristicNode *prv_create_chr_node(const struct ble_gatt_chr *chr) {
  GATTServiceDiscoveryCharacteristicNode *chr_node =
      kernel_zalloc_check(sizeof(GATTServiceDiscoveryCharacteristicNode));
  list_init(&chr_node->node);
  chr_node->characteristic = *chr;
  return chr_node;
}

static GATTServiceDiscoveryServiceNode *prv_create_service_node(const struct ble_gatt_svc *svc) {
  GATTServiceDiscoveryServiceNode *service_node =
      kernel_zalloc_check(sizeof(GATTServiceDiscoveryServiceNode));
  list_init(&service_node->node);
  service_node->service = *svc;
  return service_node;
}

static GATTServiceDiscoveryCharacteristicNode *prv_get_current_chr(
    GATTServiceDiscoveryContext *context) {
  return (GATTServiceDiscoveryCharacteristicNode *)context->current_characteristic;
}

static GATTServiceDiscoveryServiceNode *prv_get_current_service(
    GATTServiceDiscoveryContext *context) {
  return (GATTServiceDiscoveryServiceNode *)context->current_service;
}

static void prv_list_append_or_set(ListNode **list, ListNode *node) {
  if (*list == NULL) {
    *list = node;
  } else {
    list_append(*list, node);
  }
}

static int prv_find_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
  GATTServiceDiscoveryContext *context = arg;
  BTErrno errno;

  if (s_stop_discovery_requested) {
    xSemaphoreGive(s_discovery_stopped);
    prv_discovery_finished();
    prv_free_discovery_context(context);
    return BLE_HS_EDONE;
  }

  switch (error->status) {
    case 0:
      char chr_uuid_str[BLE_UUID_STR_LEN];
      ble_uuid_to_str(&dsc->uuid.u, chr_uuid_str);
      PBL_LOG_DBG("Found descriptor %s (hdl: 0x%" PRIx16 ")",
                chr_uuid_str, dsc->handle);

      GATTServiceDiscoveryDescriptorNode *dsc_node = prv_create_descriptor_node(dsc);
      GATTServiceDiscoveryCharacteristicNode *chr_node = prv_get_current_chr(context);

      prv_list_append_or_set(&chr_node->descriptors, &dsc_node->node);

      GATTServiceDiscoveryServiceNode *service_node = prv_get_current_service(context);
      service_node->num_descriptors++;

      break;
    case BLE_HS_EDONE:
      PBL_LOG_DBG("Descriptor discovery done");

      context->current_characteristic = list_get_next(context->current_characteristic);

      if (context->current_characteristic != NULL) {
        prv_discover_next_dscs(conn_handle, context);
      } else {
        context->current_service = list_get_next(context->current_service);
        while (context->current_service != NULL) {
          GATTServiceDiscoveryServiceNode *service_node = prv_get_current_service(context);

          context->current_characteristic = list_get_head(service_node->characteristics);

          if (context->current_characteristic != NULL) {
            prv_discover_next_dscs(conn_handle, context);
            break;
          }

          // Service has no characteristics, skip to next service
          context->current_service = list_get_next(context->current_service);
        }

        if (context->current_service == NULL) {
          // we're done!
          prv_discovery_finished();
          prv_convert_service_and_notify_os(conn_handle, context);
        }
      }

      break;

    default:
      PBL_LOG_ERR("Descriptor discovery error: %d",
                error->status);
      if (error->status == BLE_HS_ETIMEOUT) {
        errno = BTErrnoServiceDiscoveryTimeout;
      } else if (error->status == BLE_HS_ENOTCONN) {
        errno = BTErrnoServiceDiscoveryDisconnected;
      } else {
        errno = BTErrnoInternalErrorBegin + error->status;
      }

      prv_discovery_finished();
      bt_driver_cb_gatt_client_discovery_complete(context->connection, errno);
      prv_free_discovery_context(context);
      break;
  }

  return 0;
}

static int prv_find_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg) {
  GATTServiceDiscoveryContext *context = arg;
  BTErrno errno;

  if (s_stop_discovery_requested) {
    xSemaphoreGive(s_discovery_stopped);
    prv_discovery_finished();
    prv_free_discovery_context(context);
    return BLE_HS_EDONE;
  }

  switch (error->status) {
    case 0:
      char chr_uuid_str[BLE_UUID_STR_LEN];
      ble_uuid_to_str(&chr->uuid.u, chr_uuid_str);
      PBL_LOG_DBG("Found characteristic %s (val hdl: 0x%" PRIx16 ", def hdl: 0x%" PRIx16 ")",
                chr_uuid_str, chr->val_handle, chr->def_handle);

      GATTServiceDiscoveryCharacteristicNode *chr_node = prv_create_chr_node(chr);
      GATTServiceDiscoveryServiceNode *service_node = prv_get_current_service(context);

      prv_list_append_or_set(&service_node->characteristics, &chr_node->node);

      break;

    case BLE_HS_EDONE:
      PBL_LOG_DBG("Characteristic discovery done");

      context->current_service = list_get_next(context->current_service);

      if (context->current_service != NULL) {
        // we have another service to discover characteristics for
        prv_discover_next_chrs(conn_handle, context);
      } else {
        // got all characteristics, now let's get descriptors
        context->current_service = list_get_head(context->services);

        GATTServiceDiscoveryServiceNode *service_node = prv_get_current_service(context);
        context->current_characteristic = list_get_head(service_node->characteristics);

        if (context->current_characteristic != NULL) {
          prv_discover_next_dscs(conn_handle, context);
        } else {
          // No characteristics found, discovery complete
          prv_discovery_finished();
          prv_convert_service_and_notify_os(conn_handle, context);
        }
      }

      break;

    default:
      PBL_LOG_ERR("Characteristic discovery error: %d",
                error->status);
      if (error->status == BLE_HS_ETIMEOUT) {
        errno = BTErrnoServiceDiscoveryTimeout;
      } else if (error->status == BLE_HS_ENOTCONN) {
        errno = BTErrnoServiceDiscoveryDisconnected;
      } else {
        errno = BTErrnoInternalErrorBegin + error->status;
      }

      prv_discovery_finished();
      bt_driver_cb_gatt_client_discovery_complete(context->connection, errno);
      prv_free_discovery_context(context);
      break;
  }

  return 0;
}

static int prv_find_inc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                               const struct ble_gatt_svc *service, void *arg) {
  GATTServiceDiscoveryContext *context = arg;
  BTErrno errno;

  if (s_stop_discovery_requested) {
    xSemaphoreGive(s_discovery_stopped);
    prv_discovery_finished();
    prv_free_discovery_context(context);
    return BLE_HS_EDONE;
  }

  switch (error->status) {
    case 0:
      if (service->start_handle < context->range.start ||
          service->end_handle > context->range.end) {
        return 0; // skip this service
      }

      GATTServiceDiscoveryServiceNode *service_node = prv_create_service_node(service);
      prv_list_append_or_set(&context->services, &service_node->node);

      char service_uuid_str[BLE_UUID_STR_LEN];
      ble_uuid_to_str(&service->uuid.u, service_uuid_str);
      PBL_LOG_DBG("Found service %s, 0x%" PRIx16 "-0x%" PRIx16 " (total %lu)",
                service_uuid_str, service->start_handle, service->end_handle,
                list_count(context->services));
      break;

    case BLE_HS_EDONE:
      PBL_LOG_DBG("Service discovery complete");

      if (context->services != NULL) {
        // got services, start discovering characteristics
        context->current_service = list_get_head(context->services);
        prv_discover_next_chrs(conn_handle, context);
      } else {
        // no services found
        prv_discovery_finished();
        bt_driver_cb_gatt_client_discovery_complete(context->connection, BTErrnoOK);
        prv_free_discovery_context(context);
      }

      break;

    default:
      PBL_LOG_ERR("Service discovery error: %d", error->status);
      if (error->status == BLE_HS_ETIMEOUT) {
        errno = BTErrnoServiceDiscoveryTimeout;
      } else if (error->status == BLE_HS_ENOTCONN) {
        errno = BTErrnoServiceDiscoveryDisconnected;
      } else {
        errno = BTErrnoInternalErrorBegin + error->status;
      }

      prv_discovery_finished();
      bt_driver_cb_gatt_client_discovery_complete(context->connection, errno);
      prv_free_discovery_context(context);
      break;
  }
  return 0;
}

void nimble_discover_init(void) {
  s_discovery_stopped = xSemaphoreCreateBinary();
}

static BTErrno prv_start_discovery(const GAPLEConnection *connection, const ATTHandleRange *data) {
  uint16_t conn_handle;
  if (!pebble_device_to_nimble_conn_handle(&connection->device, &conn_handle)) {
    return BTErrnoInvalidState;
  }

  GATTServiceDiscoveryContext *context = kernel_zalloc_check(sizeof(GATTServiceDiscoveryContext));
  context->connection = (GAPLEConnection *)connection;
  context->range = *data;

  int rc = ble_gattc_disc_all_svcs(conn_handle, prv_find_inc_svc_cb, (void *)context);
  if (rc != 0) {
    prv_free_discovery_context(context);
    return BTErrnoInternalErrorBegin + rc;
  }

  s_discovery_in_progress = true;
  s_stop_discovery_requested = false;

  return BTErrnoOK;
}

typedef struct {
  GAPLEConnection *connection;
  ATTHandleRange range;
} DiscoveryOp;

static int prv_discovery_op_start(void *ctx) {
  DiscoveryOp *op = ctx;

  bt_lock();
  const bool valid = gap_le_connection_is_valid(op->connection);
  bt_unlock();

  BTErrno err = valid ? prv_start_discovery(op->connection, &op->range) : BTErrnoInvalidState;
  if (err == BTErrnoOK) {
    return 0;
  }

  PBL_LOG_WRN("Failed to start discovery (errno=%d)", err);
  if (valid) {
    // The firmware was told the start succeeded (see
    // bt_driver_gatt_start_discovery_range); report the failure through the
    // completion callback so it can finalize.
    bt_driver_cb_gatt_client_discovery_complete(op->connection, err);
  }
  return -1;
}

BTErrno bt_driver_gatt_start_discovery_range(const GAPLEConnection *connection,
                                             const ATTHandleRange *data) {
  DiscoveryOp *op = kernel_zalloc_check(sizeof(*op));
  op->connection = (GAPLEConnection *)connection;
  op->range = *data;

  // Queued so it never runs concurrently with another GATT client procedure
  // (e.g. a device name read). Start errors are reported through
  // bt_driver_cb_gatt_client_discovery_complete.
  nimble_gattc_op_queue_push(prv_discovery_op_start, op);

  return BTErrnoOK;
}

// will need to implement this by returning a different value in the callback
// but not sure if this can get called multiple times in parallel, might need
// to stuff the flag in the connection struct
BTErrno bt_driver_gatt_stop_discovery(GAPLEConnection *connection) {
  uint16_t conn_handle;
  if (!pebble_device_to_nimble_conn_handle(&connection->device, &conn_handle)) {
    return BTErrnoInvalidState;
  }

  if (s_discovery_in_progress) {
    s_stop_discovery_requested = true;
    xSemaphoreTake(s_discovery_stopped, portMAX_DELAY);
  }

  return BTErrnoOK;
}

void bt_driver_gatt_handle_discovery_abandoned(void) {}
