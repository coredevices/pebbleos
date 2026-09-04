/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "stats.h"

#include "kernel/pbl_malloc.h"

#include <pbl/util/math.h>
#include <pbl/util/sort.h>

#include <stdbool.h>

// ------------------------------------------------------------------------------------------------
// Returns the median of a given array
// If given an even number of elements, it will return the lower of the two values
// Torben median algorithm from http://ndevilla.free.fr/median/median/index.html
static int32_t prv_calculate_median(const int32_t *data, uint32_t num_data, int32_t min,
                                    int32_t max, uint32_t num_values, StatsBasicFilter filter,
                                    void *context) {
  if ((num_data == 0) || (num_values == 0)) {
    return 0;
  }

  uint32_t less;
  uint32_t greater;
  uint32_t equal;
  int32_t guess;
  int32_t max_lt_guess;
  int32_t min_gt_guess;

  while (1) {
    guess = (min + max) / 2;
    less = 0;
    greater = 0;
    equal = 0;
    max_lt_guess = min;
    min_gt_guess = max;
    for (uint32_t i = 0; i < num_data; ++i) {
      const int32_t value = data[i];
      if (filter && !filter(i, value, context)) {
        continue;
      }

      if (value < guess) {
        less++;
        if (value > max_lt_guess) {
          max_lt_guess = value;
        }
      } else if (value > guess) {
        greater++;
        if (value < min_gt_guess) {
          min_gt_guess = value;
        }
      } else {
        equal++;
      }
    }
    if (less <= ((num_values + 1) / 2) && greater <= ((num_values + 1) / 2)) {
      break;
    } else if (less > greater) {
      max = max_lt_guess;
    } else {
      min = min_gt_guess;
    }
  }

  if (less >= ((num_values + 1) / 2)) {
    return max_lt_guess;
  } else if ((less + equal) >= ((num_values + 1) / 2)) {
    return guess;
  } else {
    return min_gt_guess;
  }
}

// ------------------------------------------------------------------------------------------------
void stats_calculate_basic(StatsBasicOp op, const int32_t *data, size_t num_data,
                           StatsBasicFilter filter, void *context, int32_t *basic_out) {
  if (!data) {
    return;
  }
  int32_t num_values = 0;
  int32_t sum = 0;
  int32_t min = INT32_MAX;
  int32_t max = INT32_MIN;
  int32_t consecutive_max = 0;
  int32_t consecutive_current = 0;
  int32_t consecutive_first = 0;
  bool calc_consecutive_first = (op & StatsBasicOp_ConsecutiveFirst);

  for (size_t i = 0; i < num_data; i++) {
    const int32_t value = data[i];
    if (filter && !filter(i, value, context)) {
      if (op & StatsBasicOp_Consecutive) {
        if (consecutive_current > consecutive_max) {
          consecutive_max = consecutive_current;
        }
        consecutive_current = 0;
      }
      calc_consecutive_first = false;
      continue;
    }
    if (op & (StatsBasicOp_Sum | StatsBasicOp_Average)) {
      sum += value;
    }
    if ((op & (StatsBasicOp_Min | StatsBasicOp_Median)) && (value < min)) {
      min = value;
    }
    if ((op & (StatsBasicOp_Max | StatsBasicOp_Median)) && (value > max)) {
      max = value;
    }
    if (op & StatsBasicOp_Consecutive) {
      consecutive_current++;
    }
    if (calc_consecutive_first) {
      consecutive_first++;
    }
    num_values++;
  }
  int out_index = 0;
  if (op & StatsBasicOp_Sum) {
    basic_out[out_index++] = sum;
  }
  if (op & StatsBasicOp_Average) {
    basic_out[out_index++] = num_values ? sum / num_values : 0;
  }
  if (op & StatsBasicOp_Min) {
    basic_out[out_index++] = min;
  }
  if (op & StatsBasicOp_Max) {
    basic_out[out_index++] = max;
  }
  if (op & StatsBasicOp_Count) {
    basic_out[out_index++] = num_values;
  }
  if (op & StatsBasicOp_Consecutive) {
    basic_out[out_index++] = MAX(consecutive_max, consecutive_current);
  }
  if (op & StatsBasicOp_ConsecutiveFirst) {
    basic_out[out_index++] = consecutive_first;
  }
  if (op & StatsBasicOp_Median) {
    basic_out[out_index++] = prv_calculate_median(data, num_data, min, max, num_values, filter,
                                                  context);
  }
}

typedef struct WeightedValue {
  int32_t value;
  int32_t weight_x100;
} WeightedValue;

static int prv_cmp_weighted_value(const void *a, const void *b) {
  int32_t t_a = ((WeightedValue *)a)->value;
  int32_t t_b = ((WeightedValue *)b)->value;

  if (t_a < t_b) {
    return -1;
  } else if (t_a > t_b) {
    return 1;
  } else {
    return 0;
  }
}

//! Source:
//!   http://artax.karlin.mff.cuni.cz/r-help/library/matrixStats/html/weightedMedian.html
//! Description: (copied here in case website goes down)
//!   For the n elements x = c(x[1], x[2], ..., x[n]) with positive weights
//!   w = c(w[1], w[2], ..., w[n]) such that sum(w) = S, the weighted median is defined as the
//!   element x[k] for which the total weight of all elements x[i] < x[k] is less or equal to S/2
//!   and for which the total weight of all elements x[i] > x[k] is less or equal to S/2 (c.f. [1]).
//! Ties:
//!   How to solve ties between two x's that are satisfying the weighted median criteria.
//!   Note that at most two values can satisfy the criteria. If a tie occurs, the mean
//!   (not weighted mean) of the two values is returned.
//! NOTES:
//!   Integer division is used throughout. Take note. That is why this is here.
int32_t stats_calculate_weighted_median(const int32_t *vals, const int32_t *weights_x100,
                                        size_t num_data) {
  if (!vals || !weights_x100 || num_data < 1) {
    // Invalid args
    return 0;
  }

  WeightedValue *values = task_zalloc(sizeof(int32_t) * num_data * 2);
  if (!values) {
    return 0;
  }

  // Copy the values and sort them in ascending order
  for (size_t i = 0; i < num_data; i++) {
    values[i] = (WeightedValue) {
      .value = vals[i],
      .weight_x100 = weights_x100[i],
    };
  }
  sort_bubble(values, num_data, sizeof(*values), prv_cmp_weighted_value);

  // Find the sum of all of the weights
  int32_t S_x100;
  stats_calculate_basic(StatsBasicOp_Sum, weights_x100, num_data, NULL, NULL, &S_x100);

  if (S_x100 == 0) {
    // All weights are zero
    return 0;
  }

  int32_t tmp_sum = S_x100;
  int k = 0;
  int32_t rv = 0;
  for (;;) {
    tmp_sum -= values[k].weight_x100;

    // Have to modulo since we need to know if this is *exact*. Integer division will not let us
    // know if this is exact if it is an odd number.
    if ((S_x100 % 2 == 0) && tmp_sum == (S_x100 / 2)) {
      // This is a `tie`
      rv = (values[k].value + values[k + 1].value) / 2;
      break;
    } else if (tmp_sum <= (S_x100 / 2)) {
      // If we are not *exactly* equal (implied by the if above), but less than or equal,
      // stop and record this value.
      rv = values[k].value;
      break;
    }
    k++;
  }

  task_free(values);

  return rv;
}
