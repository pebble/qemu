/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"
#include "hw/arm/stm32_clktree.h"
#include "hw/arm/stm32_rcc.h"
#include "qemu/bitops.h"
#include <stdio.h>


void stm32_rcc_check_periph_clk(Stm32Rcc *s, stm32_periph_t periph, SysBusDevice *busdev)
{
    assert(periph >= 0);
    Clk clk = s->PERIPHCLK[periph];

    assert(clk != NULL);

    if(!clktree_is_enabled(clk)) {
        /* I assume writing to a peripheral register while the peripheral clock
         * is disabled is a bug and give a warning to unsuspecting programmers.
         * When I made this mistake on real hardware the write had no effect.
         */
        stm32_hw_warn("Warning: You are attempting to use the %s peripheral while "
                      "its clock is disabled.\n", DEVICE(busdev)->id);
    }
}

void stm32_rcc_set_periph_clk_irq(
        Stm32Rcc *s,
        stm32_periph_t periph,
        qemu_irq periph_irq)
{
    Clk clk = s->PERIPHCLK[periph];

    assert(clk != NULL);

    clktree_adduser(clk, periph_irq);
}

uint32_t stm32_rcc_get_periph_freq(
        Stm32Rcc *s,
        stm32_periph_t periph)
{
    Clk clk;

    clk = s->PERIPHCLK[periph];

    assert(clk != NULL);

    return clktree_get_output_freq(clk);
}

