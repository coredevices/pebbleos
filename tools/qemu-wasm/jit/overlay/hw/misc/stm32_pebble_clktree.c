/*
 * Basic Clock Tree Building Blocks
 *
 * Copyright (C) 2012 Andre Beckus
 *
 * Source code roughly based on omap_clk.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "hw/core/irq.h"
#include "hw/arm/stm32_clktree.h"

/* #define DEBUG_CLKTREE */

#define CLKTREE_ADD_LINK(array, count, value, array_size) \
            (array)[(count)++] = (value); \
            assert((count) <= (array_size));

struct Clk {
    const char *name;

    bool enabled;

    uint32_t input_freq, output_freq, max_output_freq;

    uint16_t multiplier, divisor;

    unsigned user_count;
    qemu_irq user[CLKTREE_MAX_IRQ];

    unsigned output_count;
    struct Clk *output[CLKTREE_MAX_OUTPUT];

    unsigned input_count;
    int selected_input;
    struct Clk *input[CLKTREE_MAX_INPUT];
};

static void clktree_recalc_output_freq(Clk clk);

static Clk clktree_get_input_clk(Clk clk)
{
    return clk->input[clk->selected_input + 1];
}

#ifdef DEBUG_CLKTREE
static void clktree_print_state(Clk clk)
{
    Clk input_clk = clktree_get_input_clk(clk);
    printf("CLKTREE: %s Output Change (SrcClk:%s InFreq:%lu OutFreq:%lu "
           "Mul:%u Div:%u Enabled:%c)\n",
           clk->name,
           input_clk ? input_clk->name : "None",
           (unsigned long)clk->input_freq,
           (unsigned long)clk->output_freq,
           (unsigned)clk->multiplier,
           (unsigned)clk->divisor,
           clk->enabled ? '1' : '0');
}
#endif

static void clktree_set_input_freq(Clk clk, uint32_t input_freq)
{
    clk->input_freq = input_freq;
    clktree_recalc_output_freq(clk);
}

static void clktree_recalc_output_freq(Clk clk)
{
    int i;
    Clk next_clk, next_clk_input;
    uint32_t new_output_freq;

    new_output_freq = clk->enabled ?
                            muldiv64(clk->input_freq,
                                     clk->multiplier,
                                     clk->divisor)
                            : 0;

    if (new_output_freq != clk->output_freq) {
        clk->output_freq = new_output_freq;

#ifdef DEBUG_CLKTREE
        clktree_print_state(clk);
#endif

        if (new_output_freq > clk->max_output_freq) {
            fprintf(stderr, "%s: Clock %s output frequency (%d Hz) exceeds "
                    "max frequency (%d Hz).\n",
                    __FUNCTION__, clk->name,
                    new_output_freq, clk->max_output_freq);
        }

        for (i = 0; i < clk->user_count; i++) {
            qemu_set_irq(clk->user[i], 1);
        }

        for (i = 0; i < clk->output_count; i++) {
            next_clk = clk->output[i];
            assert(next_clk != NULL);
            next_clk_input = clktree_get_input_clk(next_clk);
            if (next_clk_input == clk) {
                clktree_set_input_freq(next_clk, new_output_freq);
            }
        }
    }
}

static Clk clktree_create_generic(
                    const char *name,
                    uint16_t multiplier,
                    uint16_t divisor,
                    bool enabled)
{
    Clk clk = (Clk)g_malloc0(sizeof(struct Clk));

    clk->name = name;
    clk->max_output_freq = CLKTREE_NO_MAX_FREQ;
    clk->multiplier = multiplier;
    clk->divisor = divisor;
    clk->enabled = enabled;
    clk->input_count = 1;
    clk->input[0] = NULL;
    clk->selected_input = CLKTREE_NO_INPUT;

    return clk;
}

/* PUBLIC FUNCTIONS */

bool clktree_is_enabled(Clk clk)
{
    return clk->enabled;
}

uint32_t clktree_get_output_freq(Clk clk)
{
    return clk->output_freq;
}

void clktree_adduser(Clk clk, qemu_irq user)
{
    CLKTREE_ADD_LINK(
            clk->user, clk->user_count,
            user, CLKTREE_MAX_IRQ);
}

Clk clktree_create_src_clk(
                    const char *name,
                    uint32_t src_freq,
                    bool enabled)
{
    Clk clk = clktree_create_generic(name, 1, 1, enabled);
    clktree_set_input_freq(clk, src_freq);
    return clk;
}

Clk clktree_create_clk(
                    const char *name,
                    uint16_t multiplier,
                    uint16_t divisor,
                    bool enabled,
                    uint32_t max_output_freq,
                    int selected_input,
                    ...)
{
    va_list input_clks;
    Clk clk, input_clk;

    clk = clktree_create_generic(name, multiplier, divisor, enabled);
    clk->max_output_freq = max_output_freq;

    va_start(input_clks, selected_input);
    while ((input_clk = va_arg(input_clks, Clk)) != NULL) {
        CLKTREE_ADD_LINK(
                clk->input, clk->input_count,
                input_clk, CLKTREE_MAX_INPUT);

        CLKTREE_ADD_LINK(
                input_clk->output, input_clk->output_count,
                clk, CLKTREE_MAX_OUTPUT);
    }
    va_end(input_clks);

    clktree_set_selected_input(clk, selected_input);
    return clk;
}

void clktree_set_scale(Clk clk, uint16_t multiplier, uint16_t divisor)
{
    clk->multiplier = multiplier;
    clk->divisor = divisor;
    clktree_recalc_output_freq(clk);
}

void clktree_set_enabled(Clk clk, bool enabled)
{
    clk->enabled = enabled;
    clktree_recalc_output_freq(clk);
}

void clktree_set_selected_input(Clk clk, int selected_input)
{
    uint32_t input_freq;

    assert((selected_input + 1) < clk->input_count);
    clk->selected_input = selected_input;

    if (selected_input > -1) {
        input_freq = clktree_get_input_clk(clk)->output_freq;
    } else {
        input_freq = 0;
    }

    clktree_set_input_freq(clk, input_freq);
}
