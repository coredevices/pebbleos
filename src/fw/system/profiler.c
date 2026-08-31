/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "profiler.h"

#include "system/passert.h"
#include "pbl/util/size.h"

#include <cmsis_core.h>

#include <inttypes.h>
#include <stdio.h>

#ifdef CONFIG_SOC_NRF52
#include <drivers/nrfx_common.h>
#include <soc/nrfx_coredep.h>
#endif

#ifdef CONFIG_SOC_SF32LB52
#include <bf0_hal.h>
#endif

#ifdef CONFIG_PULSE_EVERYWHERE
#define PROF_LOG(buf, sz, fmt, ...) \
  do {                                        \
    snprintf(buf, sz, fmt, ## __VA_ARGS__); \
    PBL_LOG_DBG("%s", buf);      \
  } while (0)
#else
#define PROF_LOG(buf, sz, fmt, ...) dbgserial_putstr_fmt(buf, sz, fmt, ## __VA_ARGS__)
#endif

Profiler g_profiler;

#undef PROFILER_NODE
#define PROFILER_NODE(name) ProfilerNode g_profiler_node_##name = {.module_name = #name};
#include "profiler_list.h"
#undef PROFILER_NODE
#ifdef CONFIG_PROFILE_INTERRUPTS
#define IRQ_DEF(idx, irq) ProfilerNode g_profiler_node_##irq##_IRQ = {.module_name = #irq"_IRQ"};
#if defined(CONFIG_QEMU)
#include "irq_qemu.def"
#elif defined(CONFIG_SOC_NRF52)
#include "irq_nrf52.def"
#elif defined(CONFIG_SOC_SF32LB52)
#include "irq_sf32lb52.def"
#else
#error "No IRQ definition for this MICRO_FAMILY"
#endif
#undef IRQ_DEF
#endif

static ProfilerNode *s_profiler_nodes[] = {
#define PROFILER_NODE(name) &g_profiler_node_##name,
#include "profiler_list.h"
#undef PROFILER_NODE
#ifdef CONFIG_PROFILE_INTERRUPTS
#define IRQ_DEF(idx, irq) &g_profiler_node_##irq##_IRQ,
#if defined(CONFIG_QEMU)
#include "irq_qemu.def"
#elif defined(CONFIG_SOC_NRF52)
#include "irq_nrf52.def"
#elif defined(CONFIG_SOC_SF32LB52)
#include "irq_sf32lb52.def"
#else
#error "No IRQ definition for this MICRO_FAMILY"
#endif
#undef IRQ_DEF
#endif
};

static void prv_profiler_node_add(ProfilerNode *node) {
  g_profiler.nodes = list_append(g_profiler.nodes, (ListNode *) node);
}

static int prv_node_compare(void *a, void *b) {
  return ((ProfilerNode *) b)->total - ((ProfilerNode *) a)->total;
}

void prv_node_reset(ProfilerNode *node) {
  node->start = 0;
  node->end = 0;
  node->total = 0;
  node->count = 0;
  list_init(&node->list_node);
}

void profiler_init(void) {
  g_profiler.end = 0;
  g_profiler.start = 0;
  g_profiler.nodes = NULL;
  for (uint32_t i = 0; i < ARRAY_LENGTH(s_profiler_nodes); i++) {
    prv_node_reset(s_profiler_nodes[i]);
    prv_profiler_node_add(s_profiler_nodes[i]);
  }
}

void profiler_start(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= 0x01;
  g_profiler.start = DWT->CYCCNT;
}

void profiler_stop(void) {
  g_profiler.end = DWT->CYCCNT;
}

uint32_t profiler_node_get_last_cycles(ProfilerNode *node) {
  uint32_t duration = 0;
  if (node->end > node->start) {
    duration = node->end - node->start;
  } else {
    duration = (UINT32_MAX - node->start) + node->end;
  }
  return duration;
}

void profiler_node_stop(ProfilerNode *node, uint32_t dwt_cyc_cnt) {
  node->end = dwt_cyc_cnt;
  ++node->count;

  node->total += profiler_node_get_last_cycles(node);
}

uint32_t profiler_cycles_to_us(uint32_t cycles) {
#if defined(CONFIG_SOC_NRF52)
  uint32_t mhz = NRFX_DELAY_CPU_FREQ_MHZ;
#elif defined(CONFIG_SOC_SF32LB52)
  uint32_t mhz = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
#elif defined(CONFIG_QEMU)
  uint32_t mhz = SystemCoreClock / 1000000;
#else
  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq(&clocks);
  uint32_t mhz = clocks.HCLK_Frequency / 1000000;
#endif
  return cycles / mhz;
}

uint32_t profiler_node_get_total_us(ProfilerNode *node) {
  return profiler_cycles_to_us(node->total);
}

uint32_t profiler_node_get_count(ProfilerNode *node) {
  return node->count;
}

uint32_t profiler_get_total_duration(bool in_us) {
  uint32_t total;
  if (g_profiler.end > g_profiler.start) {
    total = g_profiler.end - g_profiler.start;
  } else {
    total = (UINT32_MAX - g_profiler.start) + g_profiler.end;
  }

  if (in_us) {
#if defined(CONFIG_SOC_NRF52)
    uint32_t mhz = NRFX_DELAY_CPU_FREQ_MHZ;
#elif defined(CONFIG_SOC_SF32LB52)
    uint32_t mhz = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
#elif defined(CONFIG_QEMU)
    uint32_t mhz = SystemCoreClock / 1000000;
#else
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    uint32_t mhz = clocks.HCLK_Frequency / 1000000;
#endif
    total /= mhz;
  }

  return total;
}

void profiler_print_stats(void) {
  PROFILER_STOP; // Make sure the profiler has been stopped.
  uint32_t total = profiler_get_total_duration(false);

#if defined(CONFIG_SOC_NRF52)
  uint32_t mhz = NRFX_DELAY_CPU_FREQ_MHZ;
  char buf[80];
  PROF_LOG(buf, sizeof(buf), "CPU Frequency: %"PRIu32"MHz", mhz);
#elif defined(CONFIG_SOC_SF32LB52)
  uint32_t mhz = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
  char buf[80];
  PROF_LOG(buf, sizeof(buf), "CPU Frequency: %"PRIu32"MHz", mhz);
#elif defined(CONFIG_QEMU)
  uint32_t mhz = SystemCoreClock / 1000000;
  char buf[80];
  PROF_LOG(buf, sizeof(buf), "CPU Frequency: %"PRIu32"MHz", mhz);
#else
  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq(&clocks);
  uint32_t mhz = clocks.HCLK_Frequency / 1000000;

  char buf[80];
  PROF_LOG(buf, sizeof(buf), "CPU Frequency: %"PRIu32"Hz", clocks.HCLK_Frequency);
#endif
  PROF_LOG(buf, sizeof(buf),
      "Profiler ran for %"PRIu32" ticks (%"PRIu32" us) (start: %"PRIu32"; stop:%"PRIu32")",
      total, total / mhz, g_profiler.start, g_profiler.end);

  ListNode *sorted = NULL;
  ListNode *tail = list_get_tail(g_profiler.nodes);
  while (tail != NULL) {
    ListNode * new_tail = list_pop_tail(tail);
    sorted = list_sorted_add(sorted, tail, &prv_node_compare, false);
    tail = new_tail;
  }

  if (sorted != NULL) {
    PROF_LOG(buf, sizeof(buf),
            "%-24s %-8s %-11s %-15s %-8s %-7s",
            "Name", "Count", "Cycles", "Time (us)", "Avg (us)", "% CPU");
    while (sorted != NULL) {
      ProfilerNode *node = (ProfilerNode *)sorted;
      uint32_t percent = (((int64_t)node->total) * 100) / total;

      PROF_LOG(buf, sizeof(buf),
          "%-24s %-8"PRIu32" %-11"PRIu32" %-15"PRIu32" %-8"PRIu32 " %-7"PRIu32,
          node->module_name, node->count, node->total, node->total / mhz,
          (node->total/node->count)/mhz, percent);

      sorted = sorted->next;
    }
  }

  while (sorted != NULL) {
    ListNode *new_head = list_pop_head(sorted);
    list_append(g_profiler.nodes, sorted);
    sorted = new_head;
  }
}

void command_profiler_stop(void) {
  PROFILER_STOP;
  PROFILER_PRINT_STATS;
}

void command_profiler_start(void) {
  PROFILER_INIT;
  PROFILER_START;
}

void command_profiler_stats(void) {
  PROFILER_PRINT_STATS;
}
