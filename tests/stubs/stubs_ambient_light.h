/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "util/attributes.h"

void WEAK ambient_light_init(void) {
}
void ambient_light_prime(void) {
}
void ambient_light_release(void) {
}
void ambient_light_suspend(void) {
}
void ambient_light_resume(void) {
}
uint32_t WEAK ambient_light_get_light_level(void) {
	return 0;
}
void WEAK command_als_read(void) {
}
uint32_t WEAK ambient_light_get_dark_threshold(void) {
	return 0;
}
void WEAK ambient_light_set_dark_threshold(uint32_t new_threshold) {
}
bool WEAK ambient_light_is_light(void) {
	return false;
}
uint32_t ambient_light_level_to_lux(uint32_t light_level) {
	return light_level;
}
