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
#include "qemu/timer.h"

#include "pebble_control.h"

#define DEBUG_PEBBLE_CONTROL
#ifdef DEBUG_PEBBLE_CONTROL
#define DPRINTF(fmt, ...)                                       \
    do { printf("PEBBLE_CONTROL: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


// ------------------------------------------------------------------------------------------
// NOTE: The following QemuProtocol defines describe the protocol used by the host
// to control/communicate with the emulated Pebble.
#define QEMU_HEADER_SIGNATURE 0xFEED
#define QEMU_FOOTER_SIGNATURE 0xBEEF
#define QEMU_MAX_DATA_LEN     2048

// Every message sent over the QEMU control channel has the following header. All
// data is set in network byte order. The maximum data len (not including header or footer)
// allowed is QEMU_MAX_DATA_LEN bytes
typedef struct QEMU_PACKED {
  uint16_t signature;         // QEMU_HEADER_SIGNATURE
  uint16_t protocol;          // one of QemuProtocol
  uint16_t len;               // number of bytes that follow (not including this header or footer)
} QemuCommChannelHdr;

// Every message sent over the QEMU comm channel has the following footer.
typedef struct QEMU_PACKED {
  uint16_t signature;         // QEMU_FOOTER_SIGNATURE
} QemuCommChannelFooter;


// Protocol IDs
typedef enum {
  QemuProtocol_SPP = 1,
  QemuProtocol_Tap = 2,
  QemuProtocol_BluetoothConnection = 3,
  QemuProtocol_Compass = 4,
  QemuProtocol_Battery = 5,
  QemuProtocol_Accel = 6,
  QemuProtocol_Vibration = 7,
  QemuProtocol_Button = 8
} QemuProtocol;


// Structure of the data for various protocols

// For QemuProtocol_SPP, the data is raw Pebble Protocol

// QemuProtocol_Tap
typedef struct QEMU_PACKED {
  uint8_t axis;              // 0: x-axis, 1: y-axis, 2: z-axis
  int8_t direction;         // either +1 or -1
} QemuProtocolTapHeader;


// QemuProtocol_BluetoothConnection
typedef struct QEMU_PACKED {
  uint8_t connected;         // true if connected
} QemuProtocolBluetoothConnectionHeader;


// QemuProtocol_Compass
typedef struct QEMU_PACKED {
  uint32_t magnetic_heading;      // 0x10000 represents 360 degress
  uint8_t  calib_status:8;        // CompassStatus enum
} QemuProtocolCompassHeader;


// QemuProtocol_Battery
typedef struct QEMU_PACKED {
  uint8_t battery_pct;            // from 0 to 100
  uint8_t charger_connected;
} QemuProtocolBatteryHeader;


// QemuProtocol_Accel request (to Pebble)
//! A single accelerometer sample for all three axes
typedef struct QEMU_PACKED {
  int16_t x;
  int16_t y;
  int16_t z;
} QemuProtocolAccelSample;
typedef struct QEMU_PACKED {
  uint8_t     num_samples;
  QemuProtocolAccelSample samples[0];
} QemuProtocolAccelHeader;

// QemuProtocol_Accel response (back to host)
typedef struct QEMU_PACKED {
  uint16_t     avail_space;   // Number of samples we can accept
} QemuProtocolAccelResponseHeader;


// QemuProtocol_Vibration notification (sent from Pebble to host)
typedef struct QEMU_PACKED {
  uint8_t     on;             // non-zero if vibe is on, 0 if off
} QemuProtocolVibrationNotificationHeader;


// QemuProtocol_Button
typedef struct QEMU_PACKED {
  // New button state. Bit x specifies the state of button x, where x is one of the
  // ButtonId enum values.
  uint8_t     button_state;
} QemuProtocolButtonHeader;



// -----------------------------------------------------------------------------------------
// PebbleControl globals
#define PBLCONTROL_RCV_BUF_LEN (QEMU_MAX_DATA_LEN + sizeof(QemuCommChannelHdr) \
                                + sizeof(QemuCommChannelFooter))

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


    // We buffer the characters we receive from our qemu_chr receive handler in here
    // to increase our overall throughput. This allows us to tell the target that
    // another character is ready immediately after it does a read.
    uint8_t rcv_char_buf[PBLCONTROL_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */


    // If we are passing a packet onto the target UART, this contains the number of bytes left
    // to transfer. The bytes we are transferring are always at the front of the
    // rcv_char_buf.
    uint32_t   target_send_bytes;

    // Timer used to wake us up to pump more data to the target
    struct QEMUTimer *target_send_timer;
};


// Control channel handlers are defined using this structure
typedef void (*PebbleControlMessageCallback)(PebbleControl *s, const uint8_t* data,
                                             uint32_t length);
typedef struct {
  uint16_t protocol_id;
  PebbleControlMessageCallback callback;
} PebbleControlMessageHandler;



// -----------------------------------------------------------------------------------
static void pebble_control_button_msg_callback(PebbleControl *s, const uint8_t *data,
                                              uint32_t len)
{
    DPRINTF("%s: ", __func__);
}



// -----------------------------------------------------------------------------------------
// Find handler from s_qemu_endpoints for a given protocol
static const PebbleControlMessageHandler* pebble_control_find_handler(PebbleControl *s,
                                                             uint16_t protocol_id) {
    static const PebbleControlMessageHandler s_msg_endpoints[] = {
      // IMPORTANT: These must be in sorted order!!
      { QemuProtocol_Button, pebble_control_button_msg_callback },
    };

    for (size_t i = 0; i < ARRAY_LENGTH(s_msg_endpoints); ++i) {
      const PebbleControlMessageHandler* handler = &s_msg_endpoints[i];
      if (!handler || handler->protocol_id > protocol_id) {
        break;
      }

      if (handler->protocol_id == protocol_id) {
        return handler;
      }
    }

    return NULL;
}




// -----------------------------------------------------------------------------------
// Drop the first N bytes out of the beginning of the receive buffer
static void pebble_control_consume_bytes(PebbleControl *s, uint32_t n)
{
    assert (n <= s->rcv_char_bytes);
    s->rcv_char_bytes -= n;
    memmove(&s->rcv_char_buf[0], &s->rcv_char_buf[n], s->rcv_char_bytes);
}


// -----------------------------------------------------------------------------------
// Forward the remaining portion of the packet at the front of our receive buffer onto the
// target
static void pebble_control_forward_to_target(PebbleControl *s)
{
    if (s->target_send_bytes == 0) {
        return;
    }
    DPRINTF("%s: %d bytes left to send to target\n", __func__, s->target_send_bytes);

    int can_read_bytes = s->uart_chr_can_read(s->uart);
    if (can_read_bytes > 0) {
        can_read_bytes = MIN(can_read_bytes, s->target_send_bytes);
        s->uart_chr_read(s->uart, s->rcv_char_buf, can_read_bytes);
        pebble_control_consume_bytes(s, can_read_bytes);
        s->target_send_bytes -= can_read_bytes;
        DPRINTF("%s: sent %d bytes to target, %d remaining\n", __func__, can_read_bytes,
                  s->target_send_bytes);
    }

    // If more data to send, set a timer so we run again later
    if (s->target_send_bytes) {
        DPRINTF("%s: Scheduling pebble_control_forward_to_target timer\n", __func__);
        timer_mod(s->target_send_timer,  qemu_clock_get_ms(QEMU_CLOCK_HOST) + 1);
    }
}


// -----------------------------------------------------------------------------------
// Parse through our receive buffer, for each complete control packet, process it
static void pebble_control_parse_receive_buffer(PebbleControl *s)
{
    // If we are still forwarding data to the target, finish that first
    if (s->target_send_bytes) {
        pebble_control_forward_to_target(s);
        if (s->target_send_bytes) {
            return;
        }
    }

    // Look for a complete packet
    while (s->rcv_char_bytes >= sizeof(QemuCommChannelHdr) + sizeof(QemuCommChannelFooter)) {
        QemuCommChannelHdr *hdr = (QemuCommChannelHdr *)s->rcv_char_buf;

        // Check the header signature
        if (ntohs(hdr->signature) != QEMU_HEADER_SIGNATURE) {
            DPRINTF("%s: invalid packet hdr signature detected\n", __func__);
            pebble_control_consume_bytes(s, sizeof(hdr->signature));
        }

        // Validate the length
        uint16_t data_len = ntohs(hdr->len);
        if (data_len > QEMU_MAX_DATA_LEN) {
            DPRINTF("%s: invalid packet hdr len detected\n", __func__);
            pebble_control_consume_bytes(s, sizeof(*hdr));
        }

        // If not a complete packet yet, break out
        uint16_t total_size = sizeof(QemuCommChannelHdr) + data_len
                                + sizeof(QemuCommChannelFooter);
        if (s->rcv_char_bytes < total_size) {
            break;
        }

        // We have a complete packet, see if we should process it directly or pass it onto
        // the target
        uint16_t protocol = ntohs(hdr->protocol);
        const PebbleControlMessageHandler* handler = pebble_control_find_handler(s, protocol);
        if (!handler) {
            DPRINTF("%s: passing packet with protocol %d onto target\n", __func__, protocol);
            s->target_send_bytes = total_size;
            pebble_control_forward_to_target(s);
            if (s->target_send_bytes) {
                // If we couldn't pass it all on, break out and wait for the timer callback
                // to send the rest out
                break;
            }
        } else {
            handler->callback(s, (uint8_t *)(hdr+1), data_len);
            pebble_control_consume_bytes(s, total_size);
        }

    }

}



// -----------------------------------------------------------------------------------
// Char device receive handlers
static void pebble_control_event(void *opaque, int event)
{
    PebbleControl *s = (PebbleControl *)opaque;

    s->uart_chr_event(s->uart, event);
}

static int pebble_control_can_receive(void *opaque)
{
    PebbleControl *s = (PebbleControl *)opaque;

    /* How much space do we have in our buffer? */
    return (PBLCONTROL_RCV_BUF_LEN - s->rcv_char_bytes);
}

static void pebble_control_receive(void *opaque, const uint8_t *buf, int size)
{
    PebbleControl *s = (PebbleControl *)opaque;

    assert(size > 0);

    // Copy the characters into our buffer first
    assert (size <= PBLCONTROL_RCV_BUF_LEN - s->rcv_char_bytes);
    memmove(s->rcv_char_buf + s->rcv_char_bytes, buf, size);
    s->rcv_char_bytes += size;

    // Process any complete packets in the receive buffer
    pebble_control_parse_receive_buffer(s);
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

    // The timer we use to pump more data to the uart
    s->target_send_timer = timer_new_ms(QEMU_CLOCK_HOST,
                  (QEMUTimerCB *)pebble_control_forward_to_target, s);


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

