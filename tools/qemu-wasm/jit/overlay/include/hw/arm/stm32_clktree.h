/*
 * Basic Clock Tree Building Blocks
 *
 * Copyright (C) 2012 Andre Beckus
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef STM32_CLKTREE_H
#define STM32_CLKTREE_H

#include "qemu/osdep.h"
#include "hw/core/irq.h"

#define CLKTREE_MAX_IRQ 16
#define CLKTREE_MAX_OUTPUT 24
#define CLKTREE_MAX_INPUT 24

#define CLKTREE_NO_INPUT -1
#define CLKTREE_NO_MAX_FREQ UINT32_MAX

typedef struct Clk *Clk;

bool clktree_is_enabled(Clk clk);
uint32_t clktree_get_output_freq(Clk clk);
void clktree_adduser(Clk clk, qemu_irq user);

Clk clktree_create_src_clk(
                    const char *name,
                    uint32_t src_freq,
                    bool enabled);

Clk clktree_create_clk(
                    const char *name,
                    uint16_t multiplier,
                    uint16_t divisor,
                    bool enabled,
                    uint32_t max_output_freq,
                    int selected_input,
                    ...);

void clktree_set_scale(Clk clk, uint16_t multiplier, uint16_t divisor);
void clktree_set_enabled(Clk clk, bool enabled);
void clktree_set_selected_input(Clk clk, int selected_input);

#endif /* STM32_CLKTREE_H */
