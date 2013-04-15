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

#include "stm32f2xx_rcc.h"
#include <stdio.h>


/* DEFINITIONS*/

/* See README for DEBUG details. */
//#define DEBUG_STM32_RCC

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
do { printf("STM32_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define IS_RESET_VALUE(new_value, mask, reset_value) ((new_value & mask) != (mask & reset_value))

#define WARN_UNIMPLEMENTED(new_value, mask, reset_value) \
    if (!IS_RESET_VALUE(new_value, mask, reset_value)) { \
        stm32_hw_warn("Not implemented: RCC " #mask); \
    }

#define HSI_FREQ 16000000
#define LSI_FREQ 32000

#define RCC_CR_RESET_VALUE      0x00000083
#define RCC_CR_OFFSET           0x00
#define RCC_CR_PLLI2SRDY_CL_BIT 27
#define RCC_CR_PLLI2SON_CL_BIT  26
#define RCC_CR_PLLRDY_BIT       25
#define RCC_CR_PLLON_BIT        24
#define RCC_CR_CSSON_BIT        19
#define RCC_CR_HSEBYP_BIT       18
#define RCC_CR_HSERDY_BIT       17
#define RCC_CR_HSEON_BIT        16
#define RCC_CR_HSICAL_START     8
#define RCC_CR_HSICAL_MASK      0x0000ff00
#define RCC_CR_HSITRIM_START    3
#define RCC_CR_HSITRIM_MASK     0x000000f8
#define RCC_CR_HSIRDY_BIT       1
#define RCC_CR_HSION_BIT        0

#define RCC_PLLCFGR_RESET_VALUE  0x24003010
#define RCC_PLLCFGR_OFFSET       0x04
#define RCC_PLLCFGR_PLLQ_MASK    0x0F000000
#define RCC_PLLCFGR_PLLQ_START   24
#define RCC_PLLCFGR_PLLSRC_BIT   22
#define RCC_PLLCFGR_PLLP_MASK    0x00030000
#define RCC_PLLCFGR_PLLP_START   16
#define RCC_PLLCFGR_PLLN_MASK    0x00007FC0
#define RCC_PLLCFGR_PLLN_START   6
#define RCC_PLLCFGR_PLLM_MASK    0x0000003F
#define RCC_PLLCFGR_PLLM_START   0

#define RCC_CFGR_RESET_VALUE     0x00000000
#define RCC_CFGR_OFFSET          0x08
#define RCC_CFGR_MCO2_START      30
#define RCC_CFGR_MCO2_MASK       0xC0000000
#define RCC_CFGR_MCO2PRE_START   27
#define RCC_CFGR_MCO2PRE_MASK    0x38000000
#define RCC_CFGR_MCO1PRE_START   24
#define RCC_CFGR_MCO1PRE_MASK    0x07000000
#define RCC_CFGR_I2CSRC_BIT      23
#define RCC_CFGR_MCO1_START      21
#define RCC_CFGR_MCO1_MASK       0x00600000
#define RCC_CFGR_RTCPRE_START    16
#define RCC_CFGR_RTCPRE_MASK     0x001F0000
#define RCC_CFGR_PPRE2_START     13
#define RCC_CFGR_PPRE2_MASK      0x0000E000
#define RCC_CFGR_PPRE1_START     10
#define RCC_CFGR_PPRE1_MASK      0x00001C00
#define RCC_CFGR_HPRE_START      4
#define RCC_CFGR_HPRE_MASK       0x000000F0
#define RCC_CFGR_SWS_START       2
#define RCC_CFGR_SWS_MASK        0x0000000C
#define RCC_CFGR_SW_START        0
#define RCC_CFGR_SW_MASK         0x00000003

#define RCC_CIR_OFFSET 0x0C

#define RCC_AHB1RSTR_OFFSET 0x10

#define RCC_AHB2RSTR_OFFSET 0x14

#define RCC_AHB3RSTR_OFFSET 0x18

#define RCC_APB1RSTR_OFFSET 0x20

#define RCC_APB2RSTR_OFFSET 0x24

#define RCC_AHB1ENR_OFFSET 0x30

#define RCC_AHB2ENR_OFFSET 0x34

#define RCC_AHB3ENR_OFFSET 0x38

#define RCC_APB1ENR_RESET_VALUE  0x00000000
#define RCC_APB1ENR_OFFSET       0x40
#define RCC_APB1ENR_DACEN_BIT    29
#define RCC_APB1ENR_PWREN_BIT    28
#define RCC_APB1ENR_BKPEN_BIT    27
#define RCC_APB1ENR_CAN2EN_BIT   26
#define RCC_APB1ENR_CAN1EN_BIT   25
#define RCC_APB1ENR_CANEN_BIT    25
#define RCC_APB1ENR_USBEN_BIT    23
#define RCC_APB1ENR_I2C2EN_BIT   22
#define RCC_APB1ENR_I2C1EN_BIT   21
#define RCC_APB1ENR_USART5EN_BIT 20
#define RCC_APB1ENR_USART4EN_BIT 19
#define RCC_APB1ENR_USART3EN_BIT 18
#define RCC_APB1ENR_USART2EN_BIT 17
#define RCC_APB1ENR_SPI3EN_BIT   15
#define RCC_APB1ENR_SPI2EN_BIT   14
#define RCC_APB1ENR_WWDGEN_BIT   11
#define RCC_APB1ENR_TIM7EN_BIT   5
#define RCC_APB1ENR_TIM6EN_BIT   4
#define RCC_APB1ENR_TIM5EN_BIT   3
#define RCC_APB1ENR_TIM4EN_BIT   2
#define RCC_APB1ENR_TIM3EN_BIT   1
#define RCC_APB1ENR_TIM2EN_BIT   0

#define RCC_APB2ENR_RESET_VALUE  0x00000000
#define RCC_APB2ENR_OFFSET       0x44
#define RCC_APB2ENR_MASK         0x00075F33
#define RCC_APB2ENR_TIM11EN      18
#define RCC_APB2ENR_TIM10EN      17
#define RCC_APB2ENR_TIM9EN       16
#define RCC_APB2ENR_SYSCFGEN_BIT 14
#define RCC_APB2ENR_SPI1EN_BIT   12
#define RCC_APB2ENR_SDIOEN_BIT   11
#define RCC_APB2ENR_ADC3EN_BIT   10
#define RCC_APB2ENR_ADC2EN_BIT   9
#define RCC_APB2ENR_ADC1EN_BIT   8
#define RCC_APB2ENR_USART6EN_BIT 5
#define RCC_APB2ENR_USART1EN_BIT 4
#define RCC_APB2ENR_TIM8EN_BIT   1
#define RCC_APB2ENR_TIM1EN_BIT   0

#define RCC_AHB1LPENR_RESET_VALUE  0x7E6791FF
#define RCC_AHB1LPENR_OFFSET       0x50

#define RCC_AHB2LPENR_RESET_VALUE  0x000000F1
#define RCC_AHB2LPENR_OFFSET       0x54

#define RCC_AHB3LPENR_RESET_VALUE  0x00000001
#define RCC_AHB3LPENR_OFFSET       0x58

#define RCC_APB1LPENR_RESET_VALUE  0x36FEC9FF
#define RCC_APB1LPENR_OFFSET       0x60

#define RCC_APB2LPENR_RESET_VALUE  0x00075F33
#define RCC_APB2LPENR_OFFSET       0x64

#define RCC_BDCR_RESET_VALUE       0x00000000
#define RCC_BDCR_OFFSET            0x70
#define RCC_BDCR_BDRST_BIT         16
#define RCC_BDCR_RTCEN_BIT         15
#define RCC_BDCR_RTCSEL_START      8
#define RCC_BDCR_RTCSEL_MASK       0x00000300
#define RCC_BDCR_LSEBYP_BIT        2
#define RCC_BDCR_LSERDY_BIT        1
#define RCC_BDCR_LSEON_BIT         0

#define RCC_CSR_RESET_VALUE        0x0E000000
#define RCC_CSR_OFFSET             0x74
#define RCC_CSR_LPWRRSTF_BIT       31
#define RCC_CSR_WWDGRSTF_BIT       30
#define RCC_CSR_IWDGRSTF_BIT       29
#define RCC_CSR_SFTRSTF_BIT        28
#define RCC_CSR_PORRSTF_BIT        27
#define RCC_CSR_PINRSTF_BIT        26
#define RCC_CSR_BORRSTF_BIT        25
#define RCC_CSR_RMVF_BIT           24
#define RCC_CSR_LSIRDY_BIT         1
#define RCC_CSR_LSION_BIT          0

#define RCC_SSCGR_RESET_VALUE      0x00000000
#define RCC_SSCGR_OFFSET           0x80

#define RCC_PLLI2SCFGR_RESET_VALUE 0x20003000
#define RCC_PLLI2SCFGR_OFFSET      0x84

#define PLLSRC_HSI_SELECTED 0
#define PLLSRC_HSE_SELECTED 1

#define SW_HSI_SELECTED 0
#define SW_HSE_SELECTED 1
#define SW_PLL_SELECTED 2

/* HELPER FUNCTIONS */

/* Enable the peripheral clock if the specified bit is set in the value. */
static void stm32_rcc_periph_enable(
                                    Stm32f2xxRcc *s,
                                    uint32_t new_value,
                                    bool init,
                                    int periph,
                                    uint32_t bit_mask)
{
    clktree_set_enabled(s->PERIPHCLK[periph], IS_BIT_SET(new_value, bit_mask));
}





/* REGISTER IMPLEMENTATION */

/* Read the configuration register.
   NOTE: Not implemented: CSS, PLLI2S, clock calibration and trimming. */
static uint32_t stm32_rcc_RCC_CR_read(Stm32f2xxRcc *s)
{
    /* Get the status of the clocks. */
    const bool PLLON = clktree_is_enabled(s->PLLCLK);
    const bool HSEON = clktree_is_enabled(s->HSECLK);
    const bool HSION = clktree_is_enabled(s->HSICLK);

    /* build the register value based on the clock states.  If a clock is on,
     * then its ready bit is always set.
     */
    return
    GET_BIT_MASK(RCC_CR_PLLRDY_BIT, PLLON) |
    GET_BIT_MASK(RCC_CR_PLLON_BIT, PLLON) |
    GET_BIT_MASK(RCC_CR_HSERDY_BIT, HSEON) |
    GET_BIT_MASK(RCC_CR_HSEON_BIT, HSEON) |
    GET_BIT_MASK(RCC_CR_HSIRDY_BIT, HSION) |
    GET_BIT_MASK(RCC_CR_HSION_BIT, HSION);
}

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    bool new_PLLON, new_HSEON, new_HSION;

    new_PLLON = IS_BIT_SET(new_value, RCC_CR_PLLON_BIT);
    if((clktree_is_enabled(s->PLLCLK) && !new_PLLON) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        stm32_hw_warn("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(s->PLLCLK, new_PLLON);

    new_HSEON = IS_BIT_SET(new_value, RCC_CR_HSEON_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSE_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        stm32_hw_warn("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSECLK, new_HSEON);

    new_HSION = IS_BIT_SET(new_value, RCC_CR_HSION_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSI_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        stm32_hw_warn("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSICLK, new_HSION);

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CR_PLLI2SON_CL_BIT, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CR_CSSON_BIT, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSICAL_MASK, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSITRIM_MASK, RCC_CR_RESET_VALUE);
}

static void stm32_rcc_RCC_PLLCFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    /*
     // TODO: PLL N multiplier
    new_PLLMUL = (new_value & RCC_CFGR_PLLMUL_MASK) >> RCC_CFGR_PLLMUL_START;
    if(!init) {
        if(clktree_is_enabled(s->PLLCLK) &&
           (new_PLLMUL != s->RCC_CFGR_PLLMUL)) {
            stm32_hw_warn("Can only change PLLMUL while PLL is disabled");
        }
    }
    assert(new_PLLMUL <= 0xf);
    if(new_PLLMUL == 0xf) {
        clktree_set_scale(s->PLLCLK, 16, 1);
    } else {
        clktree_set_scale(s->PLLCLK, new_PLLMUL + 2, 1);
    }
    s->RCC_CFGR_PLLMUL = new_PLLMUL;

     */
    WARN_UNIMPLEMENTED(new_value, RCC_PLLCFGR_PLLQ_MASK, RCC_PLLCFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_PLLCFGR_PLLP_MASK, RCC_PLLCFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_PLLCFGR_PLLN_MASK, RCC_PLLCFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_PLLCFGR_PLLM_MASK, RCC_PLLCFGR_RESET_VALUE);
}

static uint32_t stm32_rcc_RCC_PLLCFGR_read(Stm32f2xxRcc *s)
{
    return
    (RCC_PLLCFGR_PLLQ_MASK & RCC_PLLCFGR_RESET_VALUE) | // Q: not implemented
    (RCC_PLLCFGR_PLLP_MASK & RCC_PLLCFGR_RESET_VALUE) | // P: not implemented
    (RCC_PLLCFGR_PLLN_MASK & RCC_PLLCFGR_RESET_VALUE) | // N: not implemented
    (RCC_PLLCFGR_PLLM_MASK & RCC_PLLCFGR_RESET_VALUE) | // M: not implemented
    0;
}


static uint32_t stm32_rcc_RCC_CFGR_read(Stm32f2xxRcc *s)
{
    return
    (s->RCC_CFGR_PPRE2 << RCC_CFGR_PPRE2_START) |
    (s->RCC_CFGR_PPRE1 << RCC_CFGR_PPRE1_START) |
    (s->RCC_CFGR_HPRE << RCC_CFGR_HPRE_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SW_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SWS_START);
}

static void stm32_rcc_RCC_CFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    /* PPRE2 */
    s->RCC_CFGR_PPRE2 = (new_value & RCC_CFGR_PPRE2_MASK) >> RCC_CFGR_PPRE2_START;
    if(s->RCC_CFGR_PPRE2 < 0x4) {
        clktree_set_scale(s->PCLK2, 1, 1);
    } else {
        clktree_set_scale(s->PCLK2, 1, 2 * (s->RCC_CFGR_PPRE2 - 3));
    }

    /* PPRE1 */
    s->RCC_CFGR_PPRE1 = (new_value & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1_START;
    if(s->RCC_CFGR_PPRE1 < 4) {
        clktree_set_scale(s->PCLK1, 1, 1);
    } else {
        clktree_set_scale(s->PCLK1, 1, 2 * (s->RCC_CFGR_PPRE1 - 3));
    }

    /* HPRE */
    s->RCC_CFGR_HPRE = (new_value & RCC_CFGR_HPRE_MASK) >> RCC_CFGR_HPRE_START;
    if(s->RCC_CFGR_HPRE < 8) {
        clktree_set_scale(s->HCLK, 1, 1);
    } else {
        clktree_set_scale(s->HCLK, 1, 2 * (s->RCC_CFGR_HPRE - 7));
    }

    /* SW */
    s->RCC_CFGR_SW = (new_value & RCC_CFGR_SW_MASK) >> RCC_CFGR_SW_START;
    switch(s->RCC_CFGR_SW) {
        case 0x0:
        case 0x1:
        case 0x2:
            clktree_set_selected_input(s->SYSCLK, s->RCC_CFGR_SW);
            break;
        default:
            hw_error("Invalid input selected for SYSCLK");
            break;
    }

    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CFGR_I2CSRC_BIT, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_RTCPRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_SWS_MASK, RCC_CFGR_RESET_VALUE);
}

/* Write the APB2 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB2ENR_write(Stm32f2xxRcc *s, uint32_t new_value,
                                        bool init)
{
    /* TODO: enable/disable missing peripherals */
    stm32_rcc_periph_enable(s, new_value, init, STM32F2XX_SYSCFG, RCC_APB2ENR_SYSCFGEN_BIT);
//    stm32_rcc_periph_enable(s, new_value, init, STM32F2XX_UART1, RCC_APB2ENR_USART1EN_BIT);
//    stm32_rcc_periph_enable(s, new_value, init, STM32F2XX_UART6, RCC_APB2ENR_USART6EN_BIT);

    s->RCC_APB2ENR = new_value & RCC_APB2ENR_MASK;
}

/* Write the APB1 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB1ENR_write(Stm32f2xxRcc *s, uint32_t new_value,
                                        bool init)
{
    stm32_rcc_periph_enable(s, new_value, init, STM32F2XX_UART5,
                            RCC_APB1ENR_USART5EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32F2XX_UART4,
                            RCC_APB1ENR_USART4EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32F2XX_UART3,
                            RCC_APB1ENR_USART3EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32F2XX_UART2,
                            RCC_APB1ENR_USART2EN_BIT);

    s->RCC_APB1ENR = new_value & 0x00005e7d;
}

static uint32_t stm32_rcc_RCC_BDCR_read(Stm32f2xxRcc *s)
{
    bool lseon = clktree_is_enabled(s->LSECLK);

    return GET_BIT_MASK(RCC_BDCR_LSERDY_BIT, lseon) |
    GET_BIT_MASK(RCC_BDCR_LSEON_BIT, lseon);
}

static void stm32_rcc_RCC_BDCR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSECLK, IS_BIT_SET(new_value, RCC_BDCR_LSEON_BIT));

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_BDRST_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_RTCEN_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_BDCR_RTCSEL_MASK, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_LSEBYP_BIT, RCC_BDCR_RESET_VALUE);
}

/* Works the same way as stm32_rcc_RCC_CR_read */
static uint32_t stm32_rcc_RCC_CSR_read(Stm32f2xxRcc *s)
{
    bool lseon = clktree_is_enabled(s->LSICLK);

    return GET_BIT_MASK(RCC_CSR_LSIRDY_BIT, lseon) |
    GET_BIT_MASK(RCC_CSR_LSION_BIT, lseon);
}

/* Works the same way as stm32_rcc_RCC_CR_write */
static void stm32_rcc_RCC_CSR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSICLK, IS_BIT_SET(new_value, RCC_CSR_LSION_BIT));
}



static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;

    switch (offset) {
        case RCC_CR_OFFSET:
            return stm32_rcc_RCC_CR_read(s);
        case RCC_PLLCFGR_OFFSET:
            return stm32_rcc_RCC_PLLCFGR_read(s);
        case RCC_CFGR_OFFSET:
            return stm32_rcc_RCC_CFGR_read(s);
        case RCC_CIR_OFFSET:
            stm32_hw_warn("Unimplemented: RCC_CIR_OFFSET");
            return 0;
        case RCC_AHB1RSTR_OFFSET:
        case RCC_AHB2RSTR_OFFSET:
        case RCC_AHB3RSTR_OFFSET:
        case RCC_APB1RSTR_OFFSET:
        case RCC_APB2RSTR_OFFSET:
        case RCC_AHB1ENR_OFFSET:
        case RCC_AHB2ENR_OFFSET:
        case RCC_AHB3ENR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            return 0;
        case RCC_APB1ENR_OFFSET:
            return s->RCC_APB1ENR;
        case RCC_APB2ENR_OFFSET:
            return s->RCC_APB2ENR;
        case RCC_AHB1LPENR_OFFSET:
        case RCC_AHB2LPENR_OFFSET:
        case RCC_AHB3LPENR_OFFSET:
        case RCC_APB1LPENR_OFFSET:
        case RCC_APB2LPENR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            return 0;
        case RCC_BDCR_OFFSET:
            return stm32_rcc_RCC_BDCR_read(s);
        case RCC_CSR_OFFSET:
            return stm32_rcc_RCC_CSR_read(s);
        case RCC_SSCGR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            return 0;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}


static void stm32_rcc_writew(void *opaque, hwaddr offset,
                             uint64_t value)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;

    switch(offset) {
        case RCC_CR_OFFSET:
            stm32_rcc_RCC_CR_write(s, value, false);
            break;
        case RCC_PLLCFGR_OFFSET:
            stm32_rcc_RCC_PLLCFGR_write(s, value, false);
            break;
        case RCC_CFGR_OFFSET:
            stm32_rcc_RCC_CFGR_write(s, value, false);
            break;
        case RCC_CIR_OFFSET:
            stm32_hw_warn("Unimplemented: RCC_CIR_OFFSET");
            /* Allow a write but don't take any action */
            break;
        case RCC_APB1RSTR_OFFSET:
        case RCC_APB2RSTR_OFFSET:
        case RCC_AHB3RSTR_OFFSET:
        case RCC_AHB1ENR_OFFSET:
        case RCC_AHB2ENR_OFFSET:
        case RCC_AHB3ENR_OFFSET:
        case RCC_APB1LPENR_OFFSET:
        case RCC_APB2LPENR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            break;
        case RCC_APB2ENR_OFFSET:
            stm32_rcc_RCC_APB2ENR_write(s, value, false);
            break;
        case RCC_APB1ENR_OFFSET:
            stm32_rcc_RCC_APB1ENR_write(s, value, false);
            break;
        case RCC_AHB1LPENR_OFFSET:
            stm32_hw_warn("Unimplemented: RCC_AHB1LPENR_OFFSET");
            break;
        case RCC_AHB2LPENR_OFFSET:
            stm32_hw_warn("Unimplemented: RCC_AHB2LPENR_OFFSET");
            break;
        case RCC_AHB3LPENR_OFFSET:
            stm32_hw_warn("Unimplemented: RCC_AHB3LPENR_OFFSET");
            break;
        case RCC_BDCR_OFFSET:
            stm32_rcc_RCC_BDCR_write(s, value, false);
            break;
        case RCC_CSR_OFFSET:
            stm32_rcc_RCC_CSR_write(s, value, false);
            break;
        case RCC_SSCGR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    switch(size) {
        case 4:
            return stm32_rcc_readw(opaque, offset);
        default:
            STM32_NOT_IMPL_REG(offset, size);
            return 0;
    }
}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    switch(size) {
        case 4:
            stm32_rcc_writew(opaque, offset, value);
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_rcc_ops = {
    .read = stm32_rcc_read,
    .write = stm32_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_rcc_reset(DeviceState *dev)
{
    Stm32f2xxRcc *s = FROM_SYSBUS(Stm32f2xxRcc, SYS_BUS_DEVICE(dev));

    stm32_rcc_RCC_CR_write(s, RCC_CR_RESET_VALUE, true);
    stm32_rcc_RCC_PLLCFGR_write(s, RCC_PLLCFGR_RESET_VALUE, true);
    stm32_rcc_RCC_CFGR_write(s, RCC_CFGR_RESET_VALUE, true);
    stm32_rcc_RCC_APB2ENR_write(s, RCC_APB2ENR_RESET_VALUE, true);
    stm32_rcc_RCC_APB1ENR_write(s, RCC_APB1ENR_RESET_VALUE, true);
    stm32_rcc_RCC_BDCR_write(s, RCC_BDCR_RESET_VALUE, true);
    stm32_rcc_RCC_CSR_write(s, RCC_CSR_RESET_VALUE, true);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;

    uint32_t hclk_freq = 0;
    uint32_t ext_ref_freq = 0;
    
    hclk_freq = clktree_get_output_freq(s->HCLK);

    /* Only update the scales if the frequency is not zero. */
    if (hclk_freq > 0) {
        ext_ref_freq = hclk_freq / 8;

        /* Update the scales - these are the ratio of QEMU clock ticks
         * (which is an unchanging number independent of the CPU frequency) to
         * system/external clock ticks.
         */
        system_clock_scale = get_ticks_per_sec() / hclk_freq;
        external_ref_clock_scale = get_ticks_per_sec() / ext_ref_freq;
    }

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
            (unsigned long)hclk_freq, system_clock_scale);
    DPRINTF("Cortex SYSTICK ext ref frequency set to %lu Hz "
            "(scale set to %d).\n",
            (unsigned long)ext_ref_freq, external_ref_clock_scale);
#endif
}



/* DEVICE INITIALIZATION */

/* Set up the clock tree */
static void stm32_rcc_init_clk(Stm32f2xxRcc *s)
{
    int i;
    qemu_irq *hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);
    Clk HSI_DIV2, HSE_DIV2;

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32F2XX_PERIPH_COUNT; i++) {
        s->PERIPHCLK[i] = NULL;
    }

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
    s->HSICLK = clktree_create_src_clk("HSI", HSI_FREQ, false);
    s->LSICLK = clktree_create_src_clk("LSI", LSI_FREQ, false);
    s->HSECLK = clktree_create_src_clk("HSE", s->osc_freq, false);
    s->LSECLK = clktree_create_src_clk("LSE", s->osc32_freq, false);

    s->IWDGCLK = clktree_create_clk("IWDGCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->LSICLK, NULL);
    s->RTCCLK = clktree_create_clk("RTCCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ, CLKTREE_NO_INPUT, s->LSECLK, s->LSICLK, s->HSECLK, NULL);

    HSI_DIV2 = clktree_create_clk("HSI/2", 1, 2, true, CLKTREE_NO_MAX_FREQ, 0, s->HSICLK, NULL);
    HSE_DIV2 = clktree_create_clk("HSE/2", 1, 2, true, CLKTREE_NO_MAX_FREQ, 0, s->HSECLK, NULL);


    s->PLLXTPRECLK = clktree_create_clk("PLLXTPRE", 1, 1, true, CLKTREE_NO_MAX_FREQ, CLKTREE_NO_INPUT,
                                        s->HSECLK, HSE_DIV2, NULL);
    /* PLLCLK contains both the switch and the multiplier, which are shown as
     * two separate components in the clock tree diagram.
     */
    s->PLLCLK = clktree_create_clk("PLLCLK", 0, 1, false, 120000000, CLKTREE_NO_INPUT,
                                   HSI_DIV2, s->PLLXTPRECLK, NULL);

    s->SYSCLK = clktree_create_clk("SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT, s->HSICLK, s->HSECLK, s->PLLCLK, NULL);
    // HCLK: to AHB bus, core memory and DMA
    s->HCLK = clktree_create_clk("HCLK", 0, 1, true, 168000000, 0, s->SYSCLK, NULL);
    clktree_adduser(s->HCLK, hclk_upd_irq[0]);

    // Clock source for APB1 peripherals:
    s->PCLK1 = clktree_create_clk("PCLK1", 0, 1, true, 30000000, 0, s->HCLK, NULL);

    // Clock source for APB2 peripherals:
    s->PCLK2 = clktree_create_clk("PCLK2", 0, 1, true, 60000000, 0, s->HCLK, NULL);

    /* Peripheral clocks */
    s->PERIPHCLK[STM32F2XX_GPIOA] = clktree_create_clk("GPIOA", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32F2XX_GPIOB] = clktree_create_clk("GPIOB", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32F2XX_GPIOC] = clktree_create_clk("GPIOC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32F2XX_GPIOD] = clktree_create_clk("GPIOD", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32F2XX_GPIOE] = clktree_create_clk("GPIOE", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32F2XX_GPIOF] = clktree_create_clk("GPIOF", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32F2XX_GPIOG] = clktree_create_clk("GPIOG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);

    s->PERIPHCLK[STM32F2XX_SYSCFG] = clktree_create_clk("SYSCFG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);

    s->PERIPHCLK[STM32F2XX_UART1] = clktree_create_clk("UART1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32F2XX_UART2] = clktree_create_clk("UART2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32F2XX_UART3] = clktree_create_clk("UART3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32F2XX_UART4] = clktree_create_clk("UART4", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32F2XX_UART5] = clktree_create_clk("UART5", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
}






static int stm32_rcc_init(SysBusDevice *dev)
{
    Stm32f2xxRcc *s = FROM_SYSBUS(Stm32f2xxRcc, dev);

    memory_region_init_io(&s->iomem, &stm32_rcc_ops, s,
                          "rcc", 0x40023BFF - 0x40023800 + 1);

    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    stm32_rcc_init_clk(s);

    return 0;
}


static Property stm32_rcc_properties[] = {
    DEFINE_PROP_UINT32("osc_freq", Stm32f2xxRcc, osc_freq, 0),
    DEFINE_PROP_UINT32("osc32_freq", Stm32f2xxRcc, osc32_freq, 0),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_rcc_init;
    dc->reset = stm32_rcc_reset;
    dc->props = stm32_rcc_properties;
}

static TypeInfo stm32_rcc_info = {
    .name  = "stm32f2xx_rcc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32f2xxRcc),
    .class_init = stm32_rcc_class_init
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)