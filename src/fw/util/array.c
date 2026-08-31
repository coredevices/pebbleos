/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "util/array.h"

#include "system/passert.h"

void array_remove_nulls(void **array, size_t *num) {
  PBL_ASSERTN(array && num);
  size_t i = 0;
  while (i < *num) {
    if (array[i] == NULL) {
      for (size_t j = i + 1; j < *num; j++) {
        array[j-1] = array[j];
      }
      (*num)--;
    } else {
      i++;
    }
  }
}
