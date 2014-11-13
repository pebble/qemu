/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * QEMU Sharp LS013B7DH01 Memory LCD device model.
 */

/*
 * TODO:
 * Add part number attribute and set ROWS/COLS appropriately.
 * Add attribute for 'off' bit colour for simulating backlight.
 * Add display rotation attribute.
 * Handle 24bpp host displays.
 */

#include "qemu-common.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/ssi.h"

#define NUM_ROWS 168
#define NUM_COLS 144 // 18 bytes
#define NUM_COL_BYTES (NUM_COLS / 8)

typedef enum {
    COMMAND,
    LINENO,
    DATA,
    TRAILER
} xfer_state_t;

typedef struct {
    SSISlave ssidev;
    QemuConsole *con;
    bool redraw;
    uint8_t framebuffer[NUM_ROWS * NUM_COL_BYTES];
    int fbindex;
    xfer_state_t state;
} lcd_state;

static uint8_t
bitswap(uint8_t val)
{
    return ((val * 0x0802LU & 0x22110LU) | (val * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}

static uint32_t
sm_lcd_transfer(SSISlave *dev, uint32_t data)
{
    lcd_state *s = FROM_SSI_SLAVE(lcd_state, dev);
    /* XXX QEMU's SPI infrastructure is implicitly MSB-first */
    data = bitswap(data);

    switch(s->state) {
    case COMMAND:
        data &= 0xfd; /* Mask VCOM bit */
        switch(data) {
        case 0x01: /* Write Line */
            s->state = LINENO;
            break;
        case 0x04: /* Clear Screen */
            memset(s->framebuffer, 0, sizeof(*s->framebuffer));
            s->redraw = true;
            break;
        case 0x00: /* Toggle VCOM */
            break;
        default:
            /* Simulate confused display controller. */
            memset(s->framebuffer, 0x55, sizeof(*s->framebuffer));
            s->redraw = true;
            break;
        }
        break;
    case LINENO:
        if (data == 0) {
            s->state = COMMAND;
        } else {
            s->fbindex = (data - 1) * NUM_COL_BYTES;
            s->state = DATA;
        }
        break;
    case DATA:
        s->framebuffer[s->fbindex++] = data;
        if (s->fbindex % NUM_COL_BYTES == 0) {
            s->state = TRAILER;
        }
        break;
    case TRAILER:
        if (data != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
              "ls013 memory lcd received non-zero data in TRAILER\n");
        }
        s->state = LINENO;
        s->redraw = true;
        break;
    }
    return 0;
}

static void sm_lcd_update_display(void *arg)
{
    lcd_state *s = arg;
    DisplaySurface *surface = qemu_console_surface(s->con);

    uint8_t *d;
    uint32_t colour_on, colour_off, colour;
    int x, y, bpp;

    if (!s->redraw) {
        return;
    }

    bpp = surface_bits_per_pixel(surface);
    /* set colours according to bpp */
    switch (bpp) {
    case 8:
        colour_on = rgb_to_pixel8(0xaa, 0xaa, 0xaa);
        colour_off = rgb_to_pixel8(0x00, 0x00, 0x00);
        break;
    case 15:
        colour_on = rgb_to_pixel15(0xaa, 0xaa, 0xaa);
        colour_off = rgb_to_pixel15(0x00, 0x00, 0x00);
        break;
    case 16:
        colour_on = rgb_to_pixel16(0xaa, 0xaa, 0xaa);
        colour_off = rgb_to_pixel16(0x00, 0x00, 0x00);
    case 24:
        colour_on = rgb_to_pixel24(0xaa, 0xaa, 0xaa);
        colour_off = rgb_to_pixel24(0x00, 0x00, 0x00);
        break;
    case 32:
        colour_on = rgb_to_pixel32(0xaa, 0xaa, 0xaa);
        colour_off = rgb_to_pixel32(0x00, 0x00, 0x00);
        break;
    default:
        return;
    }

    d = surface_data(surface);
    for (y = 0; y < NUM_ROWS; y++) {
        for (x = 0; x < NUM_COLS; x++) {
            /* Rotate display - installed 'upside-down' in pebble. */
            int xr = NUM_COLS - 1 - x;
            int yr = NUM_ROWS - 1 - y;
            bool on = s->framebuffer[yr * NUM_COL_BYTES + xr / 8] & 1 << (xr % 8);
            colour = on ? colour_on : colour_off;
            switch(bpp) {
                case 8:
                    *((uint8_t *)d) = colour;
                    d++;
                    break;
                case 15:
                case 16:
                    *((uint16_t *)d) = colour;
                    d += 2;
                    break;
                case 24:
                    abort();
                case 32:
                    *((uint32_t *)d) = colour;
                    d += 4;
                    break;
            }
        }
    }

    dpy_gfx_update(s->con, 0, 0, NUM_COLS, NUM_ROWS);
    s->redraw = false;
}

static void sm_lcd_invalidate_display(void *arg)
{
    lcd_state *s = arg;
    s->redraw = true;
}

static const GraphicHwOps sm_lcd_ops = {
    .gfx_update = sm_lcd_update_display,
    .invalidate = sm_lcd_invalidate_display,
};

static int sm_lcd_init(SSISlave *dev)
{
    lcd_state *s = FROM_SSI_SLAVE(lcd_state, dev);

    s->con = graphic_console_init(DEVICE(dev), 0, &sm_lcd_ops, s);
    qemu_console_resize(s->con, NUM_COLS, NUM_ROWS);
    return 0;
}

static void sm_lcd_class_init(ObjectClass *klass, void *data)
{
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    k->init = sm_lcd_init;
    k->transfer = sm_lcd_transfer;
    k->cs_polarity = SSI_CS_LOW;
}

static const TypeInfo sm_lcd_info = {
    .name          = "sm-lcd",
    .parent        = TYPE_SSI_SLAVE,
    .instance_size = sizeof(lcd_state),
    .class_init    = sm_lcd_class_init,
};

static void sm_lcd_register(void)
{
    type_register_static(&sm_lcd_info);
}

type_init(sm_lcd_register);
