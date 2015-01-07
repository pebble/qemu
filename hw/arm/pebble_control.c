/*
 * Pebble "remote control" module. 
 *
 * This device is designed to sit in between a qemu_chr module and a UART device used
 * by the emulated Pebble. It intercepts the traffic being sent to the UART, looks for 
 * specific packets that should be interpreted by QEMU and acts upon them. For other
 * types of packets, it simply passes them on through to the Pebble UART.
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

#include "pebble_control.h"

#ifdef DEBUG_PEBBLE_CONTROL
#define DPRINTF(fmt, ...)                                       \
    do { printf("PEBBLE_CONTROL: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define PBLCONTROL_RCV_BUF_LEN 256

struct PebbleControl {
    /* Inherited */
    SysBusDevice busdev;

    // The qemu_chr driver that connects to the host over a socket connection. We receive
    // data from this device, interpret it, and either process it directly in here or forward
    // it onto the uart in the emulated pebble.
    CharDriverState *chr;

    // The uart used by the emulated Pebble. We send data to it using its IOHandler
    // callbacks. Data written to the UART by the emulated Pebble gets passed onto us
    // because we provide the UART device a pointer to our pebble_control_write() method.
    Stm32Uart *uart;
    IOEventHandler *uart_chr_event;
    IOCanReadHandler *uart_chr_can_read;
    IOReadHandler *uart_chr_read;


    /* We buffer the characters we receive from our qemu_chr receive handler in here
     * to increase our overall throughput. This allows us to tell the target that
     * another character is ready immediately after it does a read.
     */
    uint8_t rcv_char_buf[PBLCONTROL_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */
};


// -----------------------------------------------------------------------------------
// Char device receive handlers
static int pebble_control_can_receive(void *opaque)
{
    PebbleControl *s = (PebbleControl *)opaque;

    return s->uart_chr_can_read(s->uart);
}

static void pebble_control_event(void *opaque, int event)
{
    PebbleControl *s = (PebbleControl *)opaque;

    s->uart_chr_event(s->uart, event);
}

static void pebble_control_receive(void *opaque, const uint8_t *buf, int size)
{
    PebbleControl *s = (PebbleControl *)opaque;

    s->uart_chr_read(s->uart, buf, size);
}


// -----------------------------------------------------------------------------------
// This method gets passed to the UART's stm32_uart_set_write_handler(). This way
//  we can intercept all writes that the UART meant to send to the front end.
static int pebble_control_write(void *opaque, const uint8_t *buf, int len) {
    PebbleControl *s = (PebbleControl *)opaque;
    return qemu_chr_fe_write(s->chr, buf, len);
}


// -----------------------------------------------------------------------------------
PebbleControl *pebble_control_create(CharDriverState *chr, Stm32Uart *uart)
{
    PebbleControl *s = malloc(sizeof(PebbleControl));
    memset(s, 0, sizeof(*s));

    s->chr = chr;
    s->uart = uart;

    // Have the UART send writes to us
    stm32_uart_set_write_handler(uart, s, pebble_control_write);

    // Save away the receive handlers that the uart installed into chr
    stm32_uart_get_rcv_handlers(uart, &s->uart_chr_can_read, &s->uart_chr_read, &s->uart_chr_event);

    // Install our own receive handlers into the CharDriver
    qemu_chr_add_handlers(
            chr,
            pebble_control_can_receive,
            pebble_control_receive,
            pebble_control_event,
            (void *)s);

    return 0;
}

