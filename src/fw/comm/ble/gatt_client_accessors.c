/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/logging/logging.h>

#include "gatt_client_accessors.h"

#include "gap_le_connection.h"

#include "comm/bt_lock.h"

#include <pbl/btutil/bt_uuid.h>

// -------------------------------------------------------------------------------------------------
// Helpers to calculate the BLEService, BLECharacteristic and BLEDescriptor
// opaque references. To avoid having to store separate identifiers, the values
// of these references are based on the pointer values to the internal data
// structures GATTService, GATTCharacteristic and GATTDescriptor. To provide
// extra protection against the scenario where an app uses a stale (pointer)
// value that after a new service discovery still happens to map to a valid
// object, the pointer values are XOR'd with a "generation" number. This
// generation number is changed whenever gatt_remote_services are updated.
// The most significant bit (MACHINE_WORD_MSB) is not used for RAM addresses
// and forced to be always set for a reference. This way, 0 is never used and
// we can use it to symbolize an "invalid reference".

#define MACHINE_WORD_MSB (((uintptr_t) 1) << ((sizeof(uintptr_t) * 8) - 1))

static uintptr_t prv_get_generation(const GAPLEConnection *connection) {
  const uintptr_t mask = ~MACHINE_WORD_MSB;
  const uint32_t timestamp = (connection->ticks_since_connection / RTC_TICKS_HZ);
  return mask & ((uintptr_t) timestamp);
}

//! Please don't use directly, but use the prv_get_..._ref helpers so that the
//! compiler can catch type errors.
//! @see prv_get_object_by_ref for the inverse
static uintptr_t prv_get_ref(const GAPLEConnection *connection, const void *object) {
  const uintptr_t generation = prv_get_generation(connection);
  return (((uintptr_t)(void *) object) ^ generation) | MACHINE_WORD_MSB;
}

static uintptr_t prv_get_service_ref(const GAPLEConnection *connection,
                                     const GATTServiceNode *service_node) {
  return prv_get_ref(connection, service_node);
}

static uintptr_t prv_get_characteristic_ref(const GAPLEConnection *connection,
                                            const GATTCharacteristic *characteristic) {
  return prv_get_ref(connection, characteristic);
}

static uintptr_t prv_get_descriptor_ref(const GAPLEConnection *connection,
                                        const GATTDescriptor *descriptor) {
  return prv_get_ref(connection, descriptor);
}

//! Please don't use directly, but use the prv_get_...by_ref helpers so that the
//! compiler can catch type errors.
//! @see prv_get_ref for the inverse
static void * prv_get_object_by_ref(const GAPLEConnection *connection,
                                    uintptr_t ref) {
  const uintptr_t generation = prv_get_generation(connection);
  const uintptr_t mask = ~MACHINE_WORD_MSB;
  return (void *) ((ref ^ generation) & mask);
}

//! Returns internal GATTServiceNode associated with the connection and service reference.
//! Does not perform any validity checking on the reference, so not safe to call directly
//! with an untrusted service reference. @see prv_get_service_deref
static const GATTServiceNode *prv_get_service_by_ref(const GAPLEConnection *connection,
                                                     uintptr_t service_ref) {
  return prv_get_object_by_ref(connection, service_ref);
}

// -------------------------------------------------------------------------------------------------
// Iteration Helpers

typedef bool (*GATTCharacteristicIterator)(const GATTCharacteristic *characteristic, void *cb_data);

typedef bool (*GATTDescriptorIterator)(const GATTDescriptor *descriptor, void *cb_data);

typedef bool (*GATTIncludedServicesIterator)(const GATTServiceNode *included_service_node,
                                             void *cb_data);

typedef struct {
  GATTCharacteristicIterator characteristic_iterator;
  GATTDescriptorIterator descriptor_iterator;
  GATTIncludedServicesIterator included_services_iterator;
} GATTIterationCallbacks;

static bool prv_find_service_node_by_att_handle_callback(ListNode *node, void *cb_data) {
  const uint16_t att_handle = (uintptr_t) cb_data;
  const GATTServiceNode *service_node = (const GATTServiceNode *) node;
  return (service_node->service->att_handle == att_handle);
}

//! Find a sibling service node with given ATT handle
static const GATTServiceNode * prv_find_service_node_by_att_handle(
                                                                const GATTServiceNode *service_node,
                                                                uint16_t att_handle) {
  return (const GATTServiceNode *) list_find_next((ListNode *)service_node,
                                                  prv_find_service_node_by_att_handle_callback,
                                                  true /* wrap around end */,
                                                  (void *)(uintptr_t) att_handle);
}

//! @return false if an iterator callback indicated it should not continue iterating,
//! or true if the iterator reached the end completely.
static bool prv_iter_service_node(const GATTServiceNode *service_node,
                                  const GATTIterationCallbacks *callbacks,
                                  void *cb_data) {
  const GATTService *service = service_node->service;

  // Walk all the characteristics for the service:
  const GATTCharacteristic *characteristic = service->characteristics;
  for (unsigned int c = 0; c < service->num_characteristics; ++c) {
    if (callbacks->characteristic_iterator) {
      const bool should_continue = callbacks->characteristic_iterator(characteristic, cb_data);
      if (!should_continue) {
        return false;
      }
    }

    // Walk all the descriptors for this characteristic:
    if (callbacks->descriptor_iterator) {
      for (unsigned int d = 0; d < characteristic->num_descriptors; ++d) {
        const GATTDescriptor *descriptor = &characteristic->descriptors[d];
        const bool should_continue = callbacks->descriptor_iterator(descriptor, cb_data);
        if (!should_continue) {
          return false;
        }
      }
    }

    characteristic =
         (const GATTCharacteristic *) &characteristic->descriptors[characteristic->num_descriptors];
  }

  // Walk all the Included Services:
  if (callbacks->included_services_iterator &&
      service->num_att_handles_included_services) {
    // Included Services handles are tacked at the end, after the *last* descriptor of the *last*
    // characteristic. The `characteristic` variable is pointing to the end at this point.
    const uint16_t *handle = (const uint16_t *) characteristic;
    for (int h = 0; h < service->num_att_handles_included_services; ++h) {
      const GATTServiceNode *inc_service_node = prv_find_service_node_by_att_handle(service_node,
                                                                                    handle[h]);
      if (inc_service_node) {
        callbacks->included_services_iterator(inc_service_node, cb_data);
      } else {
        PBL_LOG_DBG("Included Service with handle %u not found!", handle[h]);
      }
    }
  }

  return true;
}

// -------------------------------------------------------------------------------------------------
// Service lookup & validation of references

typedef struct {
  BLEService service_ref;
  const GATTServiceNode *service_node;
} FindServiceNodeByRefCtx;

static bool prv_find_connection_and_service_node_by_service_ref_find_cb(GAPLEConnection *connection,
                                                                        void *cb_data) {
  FindServiceNodeByRefCtx *ctx = (FindServiceNodeByRefCtx *) cb_data;
  ListNode *head = &connection->gatt_remote_services->node;
  const GATTServiceNode *service_node = prv_get_service_by_ref(connection, ctx->service_ref);
  if (list_contains(head, &service_node->node)) {
    // The service_ref is valid :)
    // The associated GATTService is found with this GAPLEConnection!
    ctx->service_node = service_node;
    return true;
  }
  return false;
}

//! Based on a potentially invalid service reference, find the internal GATTServiceNode and
//! GAPLEConnection. This function is actually safe to call with an invalid / bogus service ref.
static const GATTServiceNode * prv_find_service_and_connection(BLEService service_ref,
                                                           const GAPLEConnection **out_connection) {
  // Find the GAPLEConnection & GATTServiceNode with the BLEService service_ref:
  FindServiceNodeByRefCtx ctx = {
    .service_ref = service_ref,
  };
  const GAPLEConnection *connection =
          gap_le_connection_find(prv_find_connection_and_service_node_by_service_ref_find_cb, &ctx);
  if (connection) {
    if (out_connection) {
      *out_connection = connection;
    }
    return ctx.service_node;
  }
  return NULL;
}

// -------------------------------------------------------------------------------------------------
// Characteristic/Descriptor lookup & validation of references

typedef struct {
  uintptr_t object_ref_in;
  const GATTIterationCallbacks *object_iter_callbacks_in;
  const GAPLEConnection *connection_out;
  const GATTServiceNode *service_node_out;
  const GATTCharacteristic *characteristic_out;
  const GATTDescriptor *descriptor_out;
} FindObjectByRefCtx;

static bool prv_find_characteristic_cb(const GATTCharacteristic *characteristic, void *cb_data) {
  FindObjectByRefCtx *ctx = (FindObjectByRefCtx *) cb_data;
  if (ctx->object_ref_in == prv_get_ref(ctx->connection_out, characteristic)) {
    ctx->characteristic_out = characteristic;
    return false /* should_continue */;
  }
  return true /* should_continue */;
}

static bool prv_find_descriptor_cb(const GATTDescriptor *descriptor, void *cb_data) {
  FindObjectByRefCtx *ctx = (FindObjectByRefCtx *) cb_data;
  if (ctx->object_ref_in == prv_get_ref(ctx->connection_out, descriptor)) {
    ctx->descriptor_out = descriptor;
    return false /* should_continue */;
  }
  return true /* should_continue */;
}

//! Used only in prv_find_descriptor to keep track of the characteristic that contains the found
//! descriptor. It's kind of ugly, I know.
static bool prv_track_last_characteristic_cb(const GATTCharacteristic *characteristic,
                                             void *cb_data) {
  FindObjectByRefCtx *ctx = (FindObjectByRefCtx *) cb_data;
  ctx->characteristic_out = characteristic;
  return true /* should_continue */;
}

static bool prv_find_service_containing_object_by_ref_find_cb(ListNode *node, void *cb_data) {
  FindObjectByRefCtx *ctx = (FindObjectByRefCtx *) cb_data;
  const GATTServiceNode *service_node = (const GATTServiceNode *) node;

  // Bail out early if the object reference resolves to an address outside of the service blob:
  const uintptr_t object_addr = (uintptr_t) prv_get_object_by_ref(ctx->connection_out,
                                                                  ctx->object_ref_in);
  const uintptr_t service_node_addr = (uintptr_t) (void *) service_node->service;
  const size_t size_bytes = service_node->service->size_bytes;
  if (object_addr < service_node_addr || object_addr >= service_node_addr + size_bytes) {
    return false /* should_stop -- list_find() is different... */;
  }

  // Try to find the object:
  return !prv_iter_service_node(service_node, ctx->object_iter_callbacks_in, cb_data);
}

static bool prv_find_connection_and_object_by_ref_find_cb(GAPLEConnection *connection,
                                                  void *cb_data) {
  FindObjectByRefCtx *ctx = (FindObjectByRefCtx *) cb_data;
  // connection needed by:
  // - prv_find_service_containing_object_by_ref_list_find_cb
  // - prv_find_characteristic_cb
  ctx->connection_out = connection;
  ListNode *head = &connection->gatt_remote_services->node;
  ctx->service_node_out =
        (const GATTServiceNode *) list_find(head, prv_find_service_containing_object_by_ref_find_cb,
                                            cb_data);
  return (ctx->service_node_out != NULL);
}

static void prv_find_object(uintptr_t object_ref,
                            const GATTDescriptor **descriptor_out,
                            const GATTCharacteristic **characteristic_out,
                            const GATTServiceNode **service_node_out,
                            const GAPLEConnection **connection_out,
                            const GATTIterationCallbacks *object_iter_callbacks) {
  FindObjectByRefCtx ctx = {
    .object_ref_in = object_ref,
    .object_iter_callbacks_in = object_iter_callbacks,
  };
  const GAPLEConnection *connection =
                        gap_le_connection_find(prv_find_connection_and_object_by_ref_find_cb, &ctx);
  if (connection_out) {
    *connection_out = connection;
  }
  if (service_node_out) {
    *service_node_out = ctx.service_node_out;
  }
  if (characteristic_out) {
    *characteristic_out = ctx.characteristic_out;
  }
  if (descriptor_out) {
    *descriptor_out = ctx.descriptor_out;
  }
}

//! Based on a potentially invalid characteristic reference, find the internal GATTCharacteristic
//! and GAPLEConnection.
//! This function is actually safe to call with an invalid / bogus characteristic reference.
static const GATTCharacteristic * prv_find_characteristic(BLECharacteristic characteristic_ref,
                                                          const GATTServiceNode **service_node_out,
                                                          const GAPLEConnection **connection_out) {
  const GATTIterationCallbacks object_iter_callbacks = {
    .characteristic_iterator = prv_find_characteristic_cb,
  };
  const GATTCharacteristic *characteristic;
  prv_find_object(characteristic_ref, NULL, &characteristic, service_node_out, connection_out,
                  &object_iter_callbacks);
  return characteristic;
}

//! Based on a potentially invalid descriptor reference, find the internal GATTDescriptor and
//! GAPLEConnection.
//! This function is actually safe to call with an invalid / bogus descriptor reference.
static const GATTDescriptor * prv_find_descriptor(BLEDescriptor descriptor_ref,
                                                  const GATTCharacteristic **characteristic_out,
                                                  const GATTServiceNode **service_node_out,
                                                  const GAPLEConnection **connection_out) {
  const GATTIterationCallbacks object_iter_callbacks = {
    .characteristic_iterator = prv_track_last_characteristic_cb,
    .descriptor_iterator = prv_find_descriptor_cb,
  };
  const GATTDescriptor *descriptor;
  const GATTCharacteristic *characteristic;
  prv_find_object(descriptor_ref, &descriptor, &characteristic, service_node_out, connection_out,
                  &object_iter_callbacks);
  if (characteristic_out) {
    *characteristic_out = descriptor ? characteristic : NULL;
  }
  return descriptor;
}

// -------------------------------------------------------------------------------------------------

uint8_t gatt_client_copy_service_refs(const BTDeviceInternal *device,
                                      BLEService services_out[],
                                      uint8_t num_services) {
  return gatt_client_copy_service_refs_matching_uuid(
      device, services_out, num_services, NULL);
}

uint8_t gatt_client_copy_service_refs_by_discovery_generation(
    const BTDeviceInternal *device, BLEService services_out[],
    uint8_t num_services, uint8_t discovery_gen) {
  uint8_t index = 0;
  bt_lock();
  {
    GAPLEConnection *connection = gap_le_connection_by_device(device);
    if (!connection) {
      PBL_LOG_ERR("Disconnected in the mean time...");
      goto unlock;
    }
    GATTServiceNode *node = connection->gatt_remote_services;
    while (node) {
      const bool is_match = (discovery_gen == node->service->discovery_generation);
      if (is_match) {
        if (index < num_services) {
          services_out[index] = prv_get_service_ref(connection, node);
        }
        ++index;
      }
      node = (GATTServiceNode *) node->node.next;
    }
  }
unlock:
  bt_unlock();

  // Contains number of available services because of final increment
  return index;
}

uint8_t gatt_client_copy_service_refs_matching_uuid(const BTDeviceInternal *device,
                                                    BLEService services_out[],
                                                    uint8_t num_services,
                                                    const Uuid *matching_service_uuid) {
  uint8_t index = 0;
  bt_lock();
  {
    GAPLEConnection *connection = gap_le_connection_by_device(device);
    if (!connection) {
      PBL_LOG_ERR("Disconnected in the mean time...");
      goto unlock;
    }
    GATTServiceNode *node = connection->gatt_remote_services;
    while (node) {
      const bool is_match = (!matching_service_uuid ||
                             uuid_equal(matching_service_uuid, &node->service->uuid));
      if (is_match) {
        if (index < num_services) {
          services_out[index] = prv_get_service_ref(connection, node);
        }
        ++index;
      }
      node = (GATTServiceNode *) node->node.next;
    }
  }
unlock:
  bt_unlock();

  // Contains number of available services because of final increment
  return index;
}

// -------------------------------------------------------------------------------------------------
// Iteration callbacks to copy arrays of references into callback data of type CopyRefsCtx:

typedef struct {
  const GAPLEConnection *connection;
  uintptr_t *refs_out;
  uint8_t num_found;
  uint8_t num_max;
  const Uuid * const matching_uuids;
} CopyRefsCtx;

//! Please do not use the functions that take GATTObjectHeader directly, but use the typed versions
//! instead: prv_copy_characteristic_refs_cb, prv_copy_descriptor_refs_cb and
//! prv_copy_included_service_refs_cb. This way the compiler can detect type errors.

//! Copies the reference for object into the CopyRefsCtx->refs_out array
static bool prv_copy_refs_cb(const GATTObjectHeader *object,
                             void *cb_data) {
  CopyRefsCtx *ctx = (CopyRefsCtx *) cb_data;
  int index = ctx->num_found++;
  if (index < ctx->num_max) {
    ctx->refs_out[index] = prv_get_ref(ctx->connection, object);
  }
  return true /* should_continue */;
}

//! Copies the reference for object into the CopyRefsCtx->refs_out array, only when
//! its Uuid is found in the matching_uuids array
static bool prv_copy_refs_matching_cb(const GATTObjectHeader *object,
                                      void *cb_data) {
  CopyRefsCtx *ctx = (CopyRefsCtx *) cb_data;
  for (int i = 0; i < ctx->num_max; ++i) {
    if (uuid_equal(&ctx->matching_uuids[i], &object->uuid)) {
      ctx->refs_out[i] = prv_get_ref(ctx->connection, object);
      ++ctx->num_found;
      return true; /* should_continue */
    }
  }
  // No match, don't copy...
  return true; /* should_continue */
}

//! Copies the reference for characteristic into the CopyRefsCtx->refs_out array
static bool prv_copy_characteristic_refs_cb(const GATTCharacteristic *characteristic,
                                            void *cb_data) {
  return prv_copy_refs_cb((const GATTObjectHeader *) characteristic, cb_data);
}

//! Copies the reference for characteristic into the CopyRefsCtx->refs_out array, only when
//! its Uuid is found in the matching_uuids array
static bool prv_copy_characteristic_refs_matching_cb(const GATTCharacteristic *characteristic,
                                            void *cb_data) {
  return prv_copy_refs_matching_cb((const GATTObjectHeader *) characteristic, cb_data);
}

//! Copies the reference for included service into the CopyRefsCtx->refs_out array
static bool prv_copy_included_service_refs_cb(const GATTServiceNode *inc_service,
                                              void *cb_data) {
  return prv_copy_refs_cb((const GATTObjectHeader *) inc_service, cb_data);
}

//! Copies object references associated with service_ref into refs_out.
//! @param service_ref The service with which the objects are associated
//! @param refs_out The array into which the references are copied
//! @param num_refs_out The size of the refs_out array
//! @param matching_uuids If not NULL, only copy references for objects with a matching Uuid
//! @param callbacks These callbacks determine references for what objects need to be copied out
//! (characteristics, descriptors or included services)
static uint8_t prv_locked_copy_refs_with_service_ref(BLEService service_ref,
                                                     uintptr_t refs_out[],
                                                     uint8_t num_refs_out,
                                                     const Uuid *matching_uuids,
                                                     const GATTIterationCallbacks *callbacks) {
  CopyRefsCtx ctx = {
    .refs_out = refs_out,
    .num_max = num_refs_out,
    .matching_uuids = matching_uuids,
  };
  bt_lock();
  {
    const GATTServiceNode *service_node = prv_find_service_and_connection(service_ref,
                                                                          &ctx.connection);
    if (!service_node) {
      goto unlock;
    }
    prv_iter_service_node(service_node, callbacks, &ctx);
  }
unlock:
  bt_unlock();
  // Contains number of available objects because of final increment that happens in
  // prv_copy_refs_cb
  return ctx.num_found;
}

// -------------------------------------------------------------------------------------------------

uint8_t gatt_client_service_get_characteristics(BLEService service_ref,
                                                BLECharacteristic characteristics[],
                                                uint8_t num_characteristics) {
  const GATTIterationCallbacks callbacks = {
    .characteristic_iterator = prv_copy_characteristic_refs_cb,
  };
  return prv_locked_copy_refs_with_service_ref(service_ref, characteristics, num_characteristics,
                                               NULL, &callbacks);
}

// -------------------------------------------------------------------------------------------------

uint8_t gatt_client_service_get_characteristics_matching_uuids(BLEService service_ref,
                                                         BLECharacteristic characteristics[],
                                                         const Uuid matching_characteristic_uuids[],
                                                         uint8_t num_characteristics) {
  const GATTIterationCallbacks callbacks = {
    .characteristic_iterator = prv_copy_characteristic_refs_matching_cb,
  };
  // Set all elements to BLE_CHARACTERISTIC_INVALID first:
  memset(characteristics, 0, sizeof(characteristics[0]) * num_characteristics);
  return prv_locked_copy_refs_with_service_ref(service_ref, characteristics, num_characteristics,
                                               matching_characteristic_uuids, &callbacks);
}

// -------------------------------------------------------------------------------------------------

uint8_t gatt_client_service_get_included_services(BLEService service_ref,
                                                BLEService services_out[],
                                                uint8_t num_services_out) {
  const GATTIterationCallbacks callbacks = {
    .included_services_iterator = prv_copy_included_service_refs_cb,
  };
  return prv_locked_copy_refs_with_service_ref(service_ref, services_out, num_services_out,
                                               NULL, &callbacks);
}

// -------------------------------------------------------------------------------------------------

Uuid gatt_client_service_get_uuid(BLEService service_ref) {
  Uuid uuid;
  bt_lock();
  {
    const GATTServiceNode *service_node = prv_find_service_and_connection(service_ref, NULL);
    if (!service_node) {
      uuid = UUID_INVALID;
      goto unlock;
    }
    uuid = service_node->service->uuid;
  }
unlock:
  bt_unlock();
  return uuid;
}

// -------------------------------------------------------------------------------------------------

BTDeviceInternal gatt_client_service_get_device(BLEService service_ref) {
  BTDeviceInternal device;
  bt_lock();
  {
    const GAPLEConnection *connection = NULL;
    prv_find_service_and_connection(service_ref, &connection);
    if (!connection) {
      device = BT_DEVICE_INTERNAL_INVALID;
      goto unlock;
    }
    device = connection->device;
  }
unlock:
  bt_unlock();
  return device;
}

// -------------------------------------------------------------------------------------------------

Uuid gatt_client_characteristic_get_uuid(BLECharacteristic characteristic_ref) {
  bt_lock();
  const GATTCharacteristic * const characteristic = prv_find_characteristic(characteristic_ref,
                                                                            NULL, NULL);
  // MT: Working around compiler bug in gcc 4.7.2, when written using ?: it generates broken code
  Uuid characteristic_uuid = UUID_INVALID;
  if (characteristic) {
    characteristic_uuid = characteristic->uuid;
  }
  bt_unlock();
  return characteristic_uuid;
}

// -------------------------------------------------------------------------------------------------

BLEAttributeProperty gatt_client_characteristic_get_properties(
                                                             BLECharacteristic characteristic_ref) {
  bt_lock();
  const GATTCharacteristic * const characteristic = prv_find_characteristic(characteristic_ref,
                                                                            NULL, NULL);
  const uint8_t properties = characteristic ? characteristic->properties : 0;
  bt_unlock();
  return properties;
}

// -------------------------------------------------------------------------------------------------

BLEService gatt_client_characteristic_get_service(BLECharacteristic characteristic_ref) {
  bt_lock();
  const GATTServiceNode *service_node = NULL;
  const GAPLEConnection *connection;
  prv_find_characteristic(characteristic_ref, &service_node, &connection);
  const BLEService service_ref = service_node ?
                                prv_get_service_ref(connection, service_node) : BLE_SERVICE_INVALID;
  bt_unlock();
  return service_ref;
}

// -------------------------------------------------------------------------------------------------

BTDeviceInternal gatt_client_characteristic_get_device(BLECharacteristic characteristic_ref) {
  bt_lock();
  const GAPLEConnection *connection;
  prv_find_characteristic(characteristic_ref, NULL, &connection);
  const BTDeviceInternal device = connection ? connection->device : BT_DEVICE_INTERNAL_INVALID;
  bt_unlock();
  return device;
}

// -------------------------------------------------------------------------------------------------
// Used by and extern'd for ppogatt.c and dis.c

GAPLEConnection *gatt_client_characteristic_get_connection(BLECharacteristic characteristic_ref) {
  bt_lock_assert_held(true);
  GAPLEConnection *connection;
  prv_find_characteristic(characteristic_ref, NULL, (const GAPLEConnection **) &connection);
  return connection;
}

// -------------------------------------------------------------------------------------------------

uint8_t gatt_client_characteristic_get_descriptors(BLECharacteristic characteristic_ref,
                                                  BLEDescriptor descriptor_refs_out[],
                                                  uint8_t num_descriptors) {
  uint8_t index = 0;
  bt_lock();
  const GAPLEConnection *connection;
  const GATTCharacteristic *characteristic = prv_find_characteristic(characteristic_ref, NULL,
                                                                     &connection);
  if (characteristic) {
    const GATTDescriptor *descriptor = characteristic->descriptors;
    while (index < characteristic->num_descriptors) {
      if (index < num_descriptors) {
        descriptor_refs_out[index] = prv_get_descriptor_ref(connection, descriptor);
      }
      ++descriptor;
      ++index;
    }
  }
  bt_unlock();
  return index;
}

void gatt_client_service_get_all_characteristics_and_descriptors(
    GAPLEConnection *connection, GATTService *service,
    BLECharacteristic *characteristic_hdls_out,
    BLEDescriptor *descriptor_hdls_out) {
  uint8_t curr_desc_idx = 0;
  const GATTCharacteristic *characteristic = service->characteristics;
  for (unsigned int c = 0; c < service->num_characteristics; c++) {
    for (unsigned int d = 0; d < characteristic->num_descriptors; ++d) {
      const GATTDescriptor *descriptor = &characteristic->descriptors[d];
      descriptor_hdls_out[curr_desc_idx] =
          prv_get_descriptor_ref(connection, descriptor);
      curr_desc_idx++;
    }

    characteristic_hdls_out[c] = prv_get_characteristic_ref(connection, characteristic);

    characteristic =
         (const GATTCharacteristic *) &characteristic->descriptors[characteristic->num_descriptors];
  }
}

// -------------------------------------------------------------------------------------------------

Uuid gatt_client_descriptor_get_uuid(BLEDescriptor descriptor_ref) {
  bt_lock();
  const GATTDescriptor *descriptor = prv_find_descriptor(descriptor_ref, NULL, NULL, NULL);
  // MT: Working around compiler bug in gcc 4.7.2, when written using ?: it generates broken code
  Uuid uuid = UUID_INVALID;
  if (descriptor) {
    uuid = descriptor->uuid;
  }
  bt_unlock();
  return uuid;
}

// -------------------------------------------------------------------------------------------------

BLECharacteristic gatt_client_descriptor_get_characteristic_and_connection(
                                                                  BLEDescriptor descriptor_ref,
                                                                  GAPLEConnection **connection_out);

BLECharacteristic gatt_client_descriptor_get_characteristic(BLEDescriptor descriptor_ref) {
  bt_lock();
  const BLECharacteristic characteristic_ref =
                     gatt_client_descriptor_get_characteristic_and_connection(descriptor_ref, NULL);
  bt_unlock();
  return characteristic_ref;
}

// -------------------------------------------------------------------------------------------------

//! @note !!! To access the returned GAPLEConnection bt_lock MUST be held!!!
uint16_t gatt_client_characteristic_get_handle_and_connection(BLECharacteristic characteristic_ref,
                                                              GAPLEConnection **connection_out) {
  GAPLEConnection *connection;
  const GATTServiceNode *service_node;
  const GATTCharacteristic *characteristic =
      prv_find_characteristic(characteristic_ref, &service_node,
                              (const GAPLEConnection **) &connection);
  if (!characteristic) {
    return GATTHandleInvalid;
  }
  if (connection_out) {
    *connection_out = connection;
  }
  return service_node->service->att_handle + characteristic->att_handle_offset;
}

static uint16_t prv_get_largest_att_handle_offset(const GATTService *service) {
  uint16_t largest_offset_hdl = 0;
  const GATTCharacteristic *characteristic = service->characteristics;
  for (unsigned int c = 0; c < service->num_characteristics; ++c) {
    if (characteristic->att_handle_offset > largest_offset_hdl) {
      largest_offset_hdl = characteristic->att_handle_offset;
    }

    for (unsigned int d = 0; d < characteristic->num_descriptors; ++d) {
      const GATTDescriptor *descriptor = &characteristic->descriptors[d];
      if (descriptor->att_handle_offset > largest_offset_hdl) {
        largest_offset_hdl = descriptor->att_handle_offset;
      }
    }

    characteristic =
        (const GATTCharacteristic *) &characteristic->descriptors[characteristic->num_descriptors];
  }
  return largest_offset_hdl;
}

bool gatt_client_service_get_handle_range(BLEService service_ref, ATTHandleRange *range) {
  bool success = false;
  bt_lock();
  {
    const GATTServiceNode *service_node = prv_find_service_and_connection(service_ref, NULL);
    if (service_node == NULL) {
      goto done;
    }

    uint16_t start_hdl = service_node->service->att_handle;

    *range = (ATTHandleRange) {
      .start = start_hdl,
      .end = start_hdl + prv_get_largest_att_handle_offset(service_node->service),
    };
  }
  success = true;

done:
  bt_unlock();
  return success;
}

// -------------------------------------------------------------------------------------------------

//! @note !!! To access the returned GAPLEConnection bt_lock MUST be held!!!
uint16_t gatt_client_descriptor_get_handle_and_connection(BLEDescriptor descriptor_ref,
                                                          GAPLEConnection **connection_out) {
  GAPLEConnection *connection;
  const GATTServiceNode *service_node;
  const GATTDescriptor *descriptor = prv_find_descriptor(descriptor_ref, NULL, &service_node,
                                                         (const GAPLEConnection **) &connection);
  if (!descriptor) {
    return GATTHandleInvalid;
  }
  if (connection_out) {
    *connection_out = connection;
  }
  return service_node->service->att_handle + descriptor->att_handle_offset;
}

// -------------------------------------------------------------------------------------------------

//! @note !!! To access the returned GAPLEConnection bt_lock MUST be held!!!
const GATTCharacteristic * gatt_client_find_characteristic(BLECharacteristic characteristic_ref,
                                                           const GATTServiceNode **service_node_out,
                                                           const GAPLEConnection **connection_out) {
  return prv_find_characteristic(characteristic_ref, service_node_out, connection_out);
}


// -------------------------------------------------------------------------------------------------

//! Used by gatt_client_subscription.c
//! @note !!! To access the returned GAPLEConnection bt_lock MUST be held!!!
BLEDescriptor gatt_client_accessors_find_cccd_with_characteristic(
                                                            BLECharacteristic characteristic_ref,
                                                            uint8_t *characteristic_properties_out,
                                                            uint16_t *characteristic_att_handle_out,
                                                            GAPLEConnection **connection_out) {
  const GAPLEConnection *connection;
  const GATTServiceNode *service_node;
  const GATTCharacteristic *characteristic = gatt_client_find_characteristic(characteristic_ref,
                                                                             &service_node,
                                                                             &connection);
  if (characteristic) {
    *characteristic_properties_out = characteristic->properties;
    const Uuid cccd_uuid = bt_uuid_expand_16bit(0x2902);
    for (unsigned int d = 0; d < characteristic->num_descriptors; ++d) {
      const GATTDescriptor *descriptor = &characteristic->descriptors[d];
      if (uuid_equal(&descriptor->uuid, &cccd_uuid)) {
        *connection_out = (GAPLEConnection *) connection;
        *characteristic_att_handle_out = characteristic->att_handle_offset +
                                         service_node->service->att_handle;
        return prv_get_descriptor_ref(connection, descriptor);
      }
    }
  }
  *connection_out = NULL;
  *characteristic_att_handle_out = 0;
  return BLE_DESCRIPTOR_INVALID;
}

// -------------------------------------------------------------------------------------------------

//! Used by gatt_client_subscription.c
//! @note !!! To access the returned GAPLEConnection bt_lock MUST be held!!!
BLECharacteristic gatt_client_descriptor_get_characteristic_and_connection(
                                                                  BLEDescriptor descriptor_ref,
                                                                 GAPLEConnection **connection_out) {
  const GATTCharacteristic *characteristic;
  GAPLEConnection *connection;
  const GATTDescriptor *descriptor = prv_find_descriptor(descriptor_ref, &characteristic, NULL,
                                                         (const GAPLEConnection **) &connection);
  const BLECharacteristic characteristic_ref = descriptor ?
                prv_get_characteristic_ref(connection, characteristic) : BLE_CHARACTERISTIC_INVALID;
  if (connection_out) {
    *connection_out = connection;
  }
  return characteristic_ref;
}

BLEService gatt_client_att_handle_get_service(
    GAPLEConnection *connection, uint16_t att_handle, const GATTServiceNode **service_node_out) {
  const GATTServiceNode *node =
      prv_find_service_node_by_att_handle(connection->gatt_remote_services, att_handle);
  *service_node_out = node;

  return node ? prv_get_service_ref(connection, node) : BLE_SERVICE_INVALID;
}
