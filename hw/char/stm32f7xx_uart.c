/*
 * STM32F7xx Microcontroller UART module
 *
 * Copyright (C) 2010 Andre Beckus
 * Copyright (C) 2016 Pebble
 *
 * Source code based on stm32_uart.c
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

#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "sysemu/char.h"
#include "qemu/bitops.h"



/* DEFINITIONS*/

/* See the README file for details on these settings. */
//#define DEBUG_STM32_UART
//#define STM32_UART_NO_BAUD_DELAY
//#define STM32_UART_ENABLE_OVERRUN

#ifdef DEBUG_STM32_UART
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F7XX_UART: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define USART_CR1_OFFSET 0x00
#define USART_CR1_UE_BIT 0
#define USART_CR1_RE_BIT 2
#define USART_CR1_TE_BIT 3
#define USART_CR1_RXNEIE_BIT 5
#define USART_CR1_TCIE_BIT 6
#define USART_CR1_TXEIE_BIT 7

#define USART_CR2_OFFSET 0x04

#define USART_CR3_OFFSET 0x08

#define USART_BRR_OFFSET 0x0C

#define USART_GTPR_OFFSET 0x10

#define USART_RTOR_OFFSET 0x14

#define USART_RQR_OFFSET 0x18
#define USART_RQR_RXFRQ_BIT 3

#define USART_ISR_OFFSET 0x1C
#define USART_ISR_ORE_BIT 3
#define USART_ISR_RXNE_BIT 5
#define USART_ISR_TC_BIT 6
#define USART_ISR_TXE_BIT 7

#define USART_ICR_OFFSET 0x20
#define USART_ICR_ORECF_BIT 3

#define USART_RDR_OFFSET 0x24

#define USART_TDR_OFFSET 0x28

#define USART_RCV_BUF_LEN 256

struct Stm32F7xxUart {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;
    void *stm32_gpio_prop;
    void *stm32_afio_prop;

    /* Checks the USART transmit pin's GPIO settings.  If the GPIO is not configured
     * properly, a hardware error is triggered.
     */
    union {
        void *check_tx_pin_prop;
        void (*check_tx_pin_callback)(Stm32F7xxUart *);
    };

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;
    Stm32Gpio **stm32_gpio;
    Stm32Afio *stm32_afio;

    int uart_index;

    uint32_t bits_per_sec;
    int64_t ns_per_char;

    /* Register Values */
    uint32_t USART_RDR;
    uint32_t USART_TDR;
    uint32_t USART_BRR;
    uint32_t USART_CR1;
    uint32_t USART_CR2;
    uint32_t USART_CR3;

    /* Register Field Values */
    uint32_t USART_ISR_TXE;
    uint32_t USART_ISR_TC;
    uint32_t USART_ISR_RXNE;
    uint32_t USART_ISR_ORE;
    uint32_t USART_CR1_UE;
    uint32_t USART_CR1_TXEIE;
    uint32_t USART_CR1_TCIE;
    uint32_t USART_CR1_RXNEIE;
    uint32_t USART_CR1_TE;
    uint32_t USART_CR1_RE;

    /* Indicates whether the USART is currently receiving a byte. */
    bool receiving;

    /* Timers used to simulate a delay corresponding to the baud rate. */
    struct QEMUTimer *rx_timer;
    struct QEMUTimer *tx_timer;

    void *chr_write_obj;
    int (*chr_write)(void *chr_write_obj, const uint8_t *buf, int len);

    /* Stores the USART pin mapping used by the board.  This is used to check
     * the AFIO's USARTx_REMAP register to make sure the software has set
     * the correct mapping.
     */
    uint32_t afio_board_map;

    qemu_irq irq;
    int curr_irq_level;

    /* We buffer the characters we receive from our qemu_chr receive handler in here
     * to increase our overall throughput. This allows us to tell the target that
     * another character is ready immediately after it does a read.
     */
    uint8_t rcv_char_buf[USART_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */
};


/* HELPER FUNCTIONS */

/* Update the baud rate based on the USART's peripheral clock frequency. */
static void stm32f7xx_uart_baud_update(Stm32F7xxUart *s)
{
    uint32_t clk_freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);
    uint64_t ns_per_bit;

    if ((s->USART_BRR == 0) || (clk_freq == 0)) {
        s->bits_per_sec = 0;
    } else {
        s->bits_per_sec = clk_freq / s->USART_BRR;
        ns_per_bit = 1000000000LL / s->bits_per_sec;

        /* We assume 10 bits per character.  This may not be exactly
         * accurate depending on settings, but it should be good enough. */
        s->ns_per_char = ns_per_bit * 10;
    }

#ifdef DEBUG_STM32_UART
    const char *periph_name = s->busdev.parent_obj.id;
    DPRINTF("%s clock is set to %lu Hz.\n",
                periph_name,
                (unsigned long)clk_freq);
    DPRINTF("%s BRR set to %lu.\n",
                periph_name,
                (unsigned long)s->USART_BRR);
    DPRINTF("%s Baud is set to %lu bits per sec.\n",
                periph_name,
                (unsigned long)s->bits_per_sec);
#endif
}

/* Handle a change in the peripheral clock. */
static void stm32f7xx_uart_clk_irq_handler(void *opaque, int n, int level)
{
    Stm32F7xxUart *s = (Stm32F7xxUart *)opaque;

    assert(n == 0);

    /* Only update the BAUD rate if the IRQ is being set. */
    if (level) {
        stm32f7xx_uart_baud_update(s);
    }
}

/* Routine which updates the USART's IRQ.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32f7xx_uart_update_irq(Stm32F7xxUart *s) {
    /* Note that we are not checking the ORE flag, but we should be. */
    int new_irq_level =
       (s->USART_CR1_TCIE & s->USART_ISR_TC) |
       (s->USART_CR1_TXEIE & s->USART_ISR_TXE) |
       (s->USART_CR1_RXNEIE & (s->USART_ISR_ORE | s->USART_ISR_RXNE));

    /* Only trigger an interrupt if the IRQ level changes.  We probably could
     * set the level regardless, but we will just check for good measure.
     */
    if (new_irq_level ^ s->curr_irq_level) {
        qemu_set_irq(s->irq, new_irq_level);
        s->curr_irq_level = new_irq_level;
    }
}


static void stm32f7xx_uart_start_tx(Stm32F7xxUart *s, uint32_t value);

/* Routine to be called when a transmit is complete. */
static void stm32f7xx_uart_tx_complete(Stm32F7xxUart *s)
{
    if (s->USART_ISR_TXE == 1) {
        /* If the buffer is empty, there is nothing waiting to be transmitted.
         * Mark the transmit complete. */
        s->USART_ISR_TC = 1;
        stm32f7xx_uart_update_irq(s);
    } else {
        /* Otherwise, mark the transmit buffer as empty and
         * start transmitting the value stored there.
         */
        s->USART_ISR_TXE = 1;
        stm32f7xx_uart_update_irq(s);
        stm32f7xx_uart_start_tx(s, s->USART_TDR);
    }
}

/* Start transmitting a character. */
static void stm32f7xx_uart_start_tx(Stm32F7xxUart *s, uint32_t value)
{
    uint8_t ch = value; //This will truncate the ninth bit

    /* Reset the Transmission Complete flag to indicate a transmit is in
     * progress.
     */
    s->USART_ISR_TC = 0;

    /* Write the character out. */
    if (s->chr_write_obj) {
        s->chr_write(s->chr_write_obj, &ch, 1);
    }
#ifdef STM32_UART_NO_BAUD_DELAY
    /* If BAUD delays are not being simulated, then immediately mark the
     * transmission as complete.
     */
    stm32f7xx_uart_tx_complete(s);
#else
    /* Otherwise, start the transmit delay timer. */
    timer_mod(s->tx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
}


/* Put byte into the receive data register, if we have one and the target is ready for it. */
static void stm32f7xx_uart_fill_receive_data_register(Stm32F7xxUart *s)
{
    bool enabled = (s->USART_CR1_UE && s->USART_CR1_RE);

    /* If we have no more data, or we are emulating baud delay and it's not
     * time yet for the next byte, return without filling the RDR */
    if (!s->rcv_char_bytes || s->receiving) {
        return;
    }

#ifndef STM32_UART_ENABLE_OVERRUN
    /* If overrun is not enabled, don't overwrite the current byte in the RDR */
    if (enabled && s->USART_ISR_RXNE) {
        return;
    }
#endif

    /* Pull the byte out of our buffer */
    uint8_t byte = s->rcv_char_buf[0];
    memmove(&s->rcv_char_buf[0], &s->rcv_char_buf[1], --(s->rcv_char_bytes));

    /* Only handle the received character if the module is enabled, */
    if (enabled) {
        if (s->USART_ISR_RXNE) {
            DPRINTF("stm32f7xx_uart_receive: overrun error\n");
            s->USART_ISR_ORE = 1;
            stm32f7xx_uart_update_irq(s);
        }

        /* Receive the character and mark the buffer as not empty. */
        s->USART_RDR = byte;
        s->USART_ISR_RXNE = 1;
        stm32f7xx_uart_update_irq(s);
    }

#ifndef STM32_UART_NO_BAUD_DELAY
    /* Indicate the module is receiving and start the delay. */
    s->receiving = true;
    timer_mod(s->rx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
}



/* TIMER HANDLERS */
/* Once the receive delay is finished, indicate the USART is finished receiving.
 * This will allow it to receive the next character.  The current character was
 * already received before starting the delay.
 */
static void stm32f7xx_uart_rx_timer_expire(void *opaque) {
    Stm32F7xxUart *s = (Stm32F7xxUart *)opaque;

    s->receiving = false;

    /* Put next byte into the receive data register, if we have one ready */
    stm32f7xx_uart_fill_receive_data_register(s);
}

/* When the transmit delay is complete, mark the transmit as complete
 * (the character was already sent before starting the delay). */
static void stm32f7xx_uart_tx_timer_expire(void *opaque) {
    Stm32F7xxUart *s = (Stm32F7xxUart *)opaque;

    stm32f7xx_uart_tx_complete(s);
}





/* CHAR DEVICE HANDLERS */

static int stm32f7xx_uart_can_receive(void *opaque)
{
    Stm32F7xxUart *s = (Stm32F7xxUart *)opaque;

    /* How much space do we have in our buffer? */
    return (USART_RCV_BUF_LEN - s->rcv_char_bytes);
}

static void stm32f7xx_uart_event(void *opaque, int event)
{
    /* Do nothing */
}

static void stm32f7xx_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    Stm32F7xxUart *s = (Stm32F7xxUart *)opaque;

    assert(size > 0);

    /* Copy the characters into our buffer first */
    assert (size <= USART_RCV_BUF_LEN - s->rcv_char_bytes);
    memmove(s->rcv_char_buf + s->rcv_char_bytes, buf, size);
    s->rcv_char_bytes += size;

    /* Put next byte into RDR if the target is ready for it */
    stm32f7xx_uart_fill_receive_data_register(s);
}




/* REGISTER IMPLEMENTATION */

static uint32_t stm32f7xx_uart_USART_ISR_read(Stm32F7xxUart *s)
{
    return (s->USART_ISR_TXE << USART_ISR_TXE_BIT) |
           (s->USART_ISR_TC << USART_ISR_TC_BIT) |
           (s->USART_ISR_RXNE << USART_ISR_RXNE_BIT) |
           (s->USART_ISR_ORE << USART_ISR_ORE_BIT);
}


static void stm32f7xx_uart_USART_ICR_write(Stm32F7xxUart *s, uint32_t new_value)
{
    if (new_value & USART_ICR_ORECF_BIT) {
        s->USART_ISR_ORE = 0;
    }

    stm32f7xx_uart_update_irq(s);
}


static uint32_t stm32f7xx_uart_USART_RDR_read(Stm32F7xxUart *s)
{
    uint32_t read_value = 0;
    if (!s->USART_CR1_UE) {
        hw_error("Attempted to read from USART_RDR while UART was disabled.");
    }

    if (!s->USART_CR1_RE) {
        hw_error("Attempted to read from USART_RDR while UART receiver was disabled.");
    }

    if (s->USART_ISR_RXNE) {
        /* If the receive buffer is not empty, return the value. and mark the
         * buffer as empty.
         */
        read_value = s->USART_RDR;
        s->USART_ISR_RXNE = 0;

        /* Put next character into the RDR if we have one */
        stm32f7xx_uart_fill_receive_data_register(s);
    } else {
        printf("STM32_UART WARNING: Read value from USART_RDR while it was empty.\n");
    }

    stm32f7xx_uart_update_irq(s);
    return read_value;
}


static void stm32f7xx_uart_USART_TDR_write(Stm32F7xxUart *s, uint32_t new_value)
{
    uint32_t write_value = new_value & 0x000001ff;

    if (!s->USART_CR1_UE) {
        hw_error("Attempted to write to USART_TDR while UART was disabled.");
    }

    if (!s->USART_CR1_TE) {
        hw_error("Attempted to write to USART_TDR while UART transmitter was disabled.");
    }

    if (s->check_tx_pin_callback) {
        s->check_tx_pin_callback(s);
    }

    DPRINTF("Writing to TDR (value=0x%x, ISR_TC=%d, ISR_TXE=%d)\n", write_value, s->USART_ISR_TC,
            s->USART_ISR_TXE);
    if (s->USART_ISR_TC) {
        /* If the Transmission Complete bit is set, it means the USART is not
         * currently transmitting.  This means, a transmission can immediately
         * start.
         */
        stm32f7xx_uart_start_tx(s, write_value);
    } else {
        /* Otherwise check to see if the buffer is empty.
         * If it is, then store the new character there and mark it as not empty.
         * If it is not empty, trigger a hardware error.  Software should check
         * to make sure it is empty before writing to the Data Register.
         */
        if (s->USART_ISR_TXE) {
            s->USART_TDR = write_value;
            s->USART_ISR_TXE = 0;
        } else {
            hw_error("Wrote new value to USART_TDR while it was non-empty.");
        }
    }

    stm32f7xx_uart_update_irq(s);
}

/* Update the Baud Rate Register. */
static void stm32f7xx_uart_USART_BRR_write(Stm32F7xxUart *s, uint32_t new_value, bool init)
{
    s->USART_BRR = new_value & 0x0000ffff;

    stm32f7xx_uart_baud_update(s);
}

static void stm32f7xx_uart_USART_CR1_write(Stm32F7xxUart *s, uint32_t new_value, bool init)
{
    s->USART_CR1_UE = extract32(new_value, USART_CR1_UE_BIT, 1);
#if 0
    if (s->USART_CR1_UE) {
        /* Check to make sure the correct mapping is selected when enabling the
         * USART.
         */
        if (s->afio_board_map != stm32_afio_get_periph_map(s->stm32_afio, s->periph)) {
            hw_error("Bad AFIO mapping for %s", s->busdev.parent_obj.id);
        }
    }
#endif

    s->USART_CR1_TXEIE = extract32(new_value, USART_CR1_TXEIE_BIT, 1);
    s->USART_CR1_TCIE = extract32(new_value, USART_CR1_TCIE_BIT, 1);
    s->USART_CR1_RXNEIE = extract32(new_value, USART_CR1_RXNEIE_BIT, 1);

    s->USART_CR1_TE = extract32(new_value, USART_CR1_TE_BIT, 1);
    s->USART_CR1_RE = extract32(new_value, USART_CR1_RE_BIT, 1);

    s->USART_CR1 = new_value & 0x1ffffffd;

    stm32f7xx_uart_update_irq(s);
}

static void stm32f7xx_uart_USART_CR2_write(Stm32F7xxUart *s, uint32_t new_value, bool init)
{
    s->USART_CR2 = new_value & 0xffffff70;
}

static void stm32f7xx_uart_USART_CR3_write(Stm32F7xxUart *s, uint32_t new_value, bool init)
{
    s->USART_CR3 = new_value & 0x0000e7ff;
}

static void stm32f7xx_uart_reset(DeviceState *dev)
{
    Stm32F7xxUart *s = STM32F7XX_UART(dev);

    /* Initialize the status registers.  These are mostly
     * read-only, so we do not call the "write" routine
     * like normal.
     */
    s->USART_ISR_TXE = 1;
    s->USART_ISR_TC = 1;
    s->USART_ISR_RXNE = 0;
    s->USART_ISR_ORE = 0;

    // Do not initialize USART_RDR/USART_TDR - it is documented as undefined at reset
    // and does not behave like normal registers.
    stm32f7xx_uart_USART_BRR_write(s, 0x00000000, true);
    stm32f7xx_uart_USART_CR1_write(s, 0x00000000, true);
    stm32f7xx_uart_USART_CR2_write(s, 0x00000000, true);
    stm32f7xx_uart_USART_CR3_write(s, 0x00000000, true);

    stm32f7xx_uart_update_irq(s);
}

static uint64_t stm32f7xx_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    Stm32F7xxUart *s = (Stm32F7xxUart *)opaque;
    uint64_t value = 0;
    int start = (offset & 3) * 8;
    int length = size * 8;

    switch (offset & 0xfffffffc) {
        case USART_ISR_OFFSET:
            value = stm32f7xx_uart_USART_ISR_read(s);
            break;
        case USART_RDR_OFFSET:
            value = stm32f7xx_uart_USART_RDR_read(s);
            break;
        case USART_BRR_OFFSET:
            value = s->USART_BRR;
            break;
        case USART_CR1_OFFSET:
            value = s->USART_CR1;
            break;
        case USART_CR2_OFFSET:
            value = s->USART_CR2;
            break;
        case USART_CR3_OFFSET:
            value = s->USART_CR3;
            break;
        case USART_GTPR_OFFSET:
        case USART_RTOR_OFFSET:
        case USART_RQR_OFFSET:
        case USART_ICR_OFFSET:
            // always reads as 0
            break;
        default:
            STM32_BAD_REG(offset, size);
            break;
    }

    return extract64(value, start, length);
}

static void stm32f7xx_uart_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    Stm32F7xxUart *s = (Stm32F7xxUart *)opaque;
    int start = (offset & 3) * 8;
    int length = size * 8;

    stm32_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, s->periph);

    switch (offset & 0xfffffffc) {
        case USART_ICR_OFFSET:
            stm32f7xx_uart_USART_ICR_write(s, deposit64(0, start, length, value));
            break;
        case USART_ISR_OFFSET:
            STM32_WARN_RO_REG(offset);
            break;
        case USART_TDR_OFFSET:
            stm32f7xx_uart_USART_TDR_write(s, deposit64(0, start, length, value));
            break;
        case USART_BRR_OFFSET:
            stm32f7xx_uart_USART_BRR_write(s, deposit64(s->USART_BRR, start, length, value), false);
            break;
        case USART_CR1_OFFSET:
            stm32f7xx_uart_USART_CR1_write(s, deposit64(s->USART_CR1, start, length, value), false);
            break;
        case USART_CR2_OFFSET:
            stm32f7xx_uart_USART_CR2_write(s, deposit64(s->USART_CR2, start, length, value), false);
            break;
        case USART_CR3_OFFSET:
            stm32f7xx_uart_USART_CR3_write(s, deposit64(s->USART_CR3, start, length, value), false);
            break;
        case USART_RQR_OFFSET:
            if (extract32(value, USART_RQR_RXFRQ_BIT, 1) == 1) {
                s->USART_ISR_RXNE = 0;
            }
            break;
        case USART_GTPR_OFFSET:
        case USART_RTOR_OFFSET:
            STM32_NOT_IMPL_REG(offset, size);
            break;
        default:
            STM32_BAD_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32f7xx_uart_ops = {
    .read = stm32f7xx_uart_read,
    .write = stm32f7xx_uart_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};





/* PUBLIC FUNCTIONS */

void stm32f7xx_uart_set_write_handler(Stm32F7xxUart *s, void *obj,
        int (*chr_write_handler)(void *chr_write_obj, const uint8_t *buf, int len))
{
    s->chr_write_obj = obj;
    s->chr_write = chr_write_handler;
}


void stm32f7xx_uart_get_rcv_handlers(Stm32F7xxUart *s, IOCanReadHandler **can_read,
                                 IOReadHandler **read, IOEventHandler **event)
{
    *can_read = stm32f7xx_uart_can_receive;
    *read = stm32f7xx_uart_receive;
    *event = stm32f7xx_uart_event;
}


// Stub used to typecast the generic write handler prototype to a qemu_chr write handler.
static int stm32f7xx_uart_chr_fe_write_stub(void *s, const uint8_t *buf, int len)
{
    return qemu_chr_fe_write((CharDriverState *)s, buf, len);
}


// Helper method that connects this UART device's receive handlers to a qemu_chr instance.
void stm32f7xx_uart_connect(Stm32F7xxUart *s, CharDriverState *chr, uint32_t afio_board_map)
{
    s->chr_write_obj = chr;
    if (chr) {
        stm32f7xx_uart_set_write_handler(s, chr, stm32f7xx_uart_chr_fe_write_stub);
        IOCanReadHandler *can_read_cb;
        IOReadHandler *read_cb;
        IOEventHandler *event_cb;
        stm32f7xx_uart_get_rcv_handlers(s, &can_read_cb, &read_cb, &event_cb);
        qemu_chr_add_handlers(chr, can_read_cb, read_cb, event_cb, s);
    }

    s->afio_board_map = afio_board_map;
}




/* DEVICE INITIALIZATION */

static int stm32f7xx_uart_init(SysBusDevice *dev)
{
    qemu_irq *clk_irq;
    Stm32F7xxUart *s = STM32F7XX_UART(dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;
    s->stm32_gpio = (Stm32Gpio **)s->stm32_gpio_prop;
    s->stm32_afio = (Stm32Afio *)s->stm32_afio_prop;

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32f7xx_uart_ops, s, "uart", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                               (QEMUTimerCB *)stm32f7xx_uart_rx_timer_expire, s);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                               (QEMUTimerCB *)stm32f7xx_uart_tx_timer_expire, s);

    /* Register handlers to handle updates to the USART's peripheral clock. */
    clk_irq = qemu_allocate_irqs(stm32f7xx_uart_clk_irq_handler, (void *)s, 1);
    stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);

    s->rcv_char_bytes = 0;

    stm32f7xx_uart_reset((DeviceState *)s);

    return 0;
}

static Property stm32f7xx_uart_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32F7xxUart, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32_rcc", Stm32F7xxUart, stm32_rcc_prop),
    DEFINE_PROP_PTR("stm32_gpio", Stm32F7xxUart, stm32_gpio_prop),
    DEFINE_PROP_PTR("stm32_afio", Stm32F7xxUart, stm32_afio_prop),
    DEFINE_PROP_PTR("stm32_check_tx_pin_callback", Stm32F7xxUart, check_tx_pin_prop),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32f7xx_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32f7xx_uart_init;
    dc->reset = stm32f7xx_uart_reset;
    dc->props = stm32f7xx_uart_properties;
}

static TypeInfo stm32f7xx_uart_info = {
    .name  = "stm32f7xx-uart",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32F7xxUart),
    .class_init = stm32f7xx_uart_class_init
};

static void stm32f7xx_uart_register_types(void)
{
    type_register_static(&stm32f7xx_uart_info);
}

type_init(stm32f7xx_uart_register_types)
