//! @file

#include "kernel/kernel_heap.h"
#include "memfault/components.h"
#include "services/common/battery/battery_state.h"
#include "util/heap.h"

int memfault_platform_get_stateofcharge(sMfltPlatformBatterySoc *soc) {
  BatteryChargeState chargestate = battery_get_charge_state();

  *soc = (sMfltPlatformBatterySoc){
      .soc = chargestate.charge_percent,
      .discharging = !chargestate.is_charging,
  };

  return 0;
}

// Record some few sample metrics. FIXME: Memfault should instead capture the
// analytics system metric data directly
void memfault_metrics_heartbeat_collect_data(void) {
  // battery_state_get_voltage() actually returns the voltage in millivolts,
  // which is the unit for the battery_v metric as recorded on device.
  MEMFAULT_METRIC_SET_UNSIGNED(battery_v, battery_state_get_voltage());

  // Kernel heap usage
  Heap *kernel_heap = kernel_heap_get();
  const uint32_t kernel_heap_size = heap_size(kernel_heap);
  const uint32_t kernel_heap_max_used = kernel_heap->high_water_mark;
  // kernel_heap_pct is a percentage with 2 decimal places of precision
  // (i.e. 10000 = 100.00%)
  const uint32_t kernel_heap_pct = (kernel_heap_max_used * 10000) / kernel_heap_size;

  MEMFAULT_LOG_INFO("Heap Usage: %lu/%lu (%lu.%02lu%%)\n", kernel_heap_max_used, kernel_heap_size,
                    kernel_heap_pct / 100, kernel_heap_pct % 100);

  MEMFAULT_METRIC_SET_UNSIGNED(memory_pct_max, kernel_heap_pct);
}
