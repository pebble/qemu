/*-
 * Copyright (c) 2014
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
 * 8-bit color display connected through SPI bus. The 8 bits are organized as (starting from MSB):
 *  2 bits red, 2 bits green, 2 bits blue, 2 bits dont' care.
 *
 * This display is used in the Pebble Snowy platform and actually represents an FPGA connected
 * to a LPM012A220A display. The FPGA implements the SPI interface.
 * 
 * Some example colors:
 *    black: 0x00
 *    white: 0xFC
 *    red:   0xC0
 *    green: 0x30
 *    blue:  0x0C
 *
 * This display expects columns to be sent through the SPI bus, from bottom to top. So, when we 
 *  get a line of data from the SPI bus, the first byte is the column index and the remaining bytes
 *  are the bytes in the column, starting from the bottom.
 *
 * This display expects 206 bytes to be sent per line (column). Organized as follows:
 *  uint8_t column_index
 *  uint8_t padding[17]
 *  uint8_t column_data[172]
 *  uint8_t padding[16]
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

#ifdef DEBUG_PEBBLE_SNOWY_DISPLAY
#define DPRINTF(fmt, ...)                                       \
    do { fprintf(stderr, "PEBBLE_SNOWY_DISPLAY: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


#define SNOWY_NUM_ROWS        172
#define SNOWY_NUM_COLS        148
#define SNOWY_BYTES_PER_ROW   SNOWY_NUM_COLS

#define SNOWY_ROWS_SKIPPED_AT_TOP     17
#define SNOWY_ROWS_SKIPPED_AT_BOTTOM  16
#define SNOWY_LINE_DATA_LEN   (SNOWY_ROWS_SKIPPED_AT_TOP + SNOWY_NUM_ROWS \
                               + SNOWY_ROWS_SKIPPED_AT_BOTTOM)

typedef enum {
    LINENO,
    DATA,
} XferState;

typedef struct {
    SSISlave ssidev;

    /* Properties */
    union {
        void *vdone_output;
        qemu_irq done_output;
    };
    union {
        void *vintn_output;
        qemu_irq intn_output;
    };

    QemuConsole *con;
    bool redraw;
    uint8_t framebuffer[SNOWY_NUM_ROWS * SNOWY_BYTES_PER_ROW];
    int col_index;
    int row_index;
    XferState state;

    uint32_t  sclk_count;
} PSDisplayState;


static uint32_t
ps_display_transfer(SSISlave *dev, uint32_t data)
{
    PSDisplayState *s = FROM_SSI_SLAVE(PSDisplayState, dev);

    //DPRINTF("rcv byte: 0x%02x\n", data);

    switch(s->state) {
    case LINENO:
        s->col_index = data;
        /* The column data is sent from the bottom up */
        s->row_index = SNOWY_NUM_ROWS + SNOWY_ROWS_SKIPPED_AT_BOTTOM - 1;
        s->state = DATA;
        DPRINTF("  new column no: %d\n", data);
        break;
    case DATA:
        //DPRINTF("  pixel value 0x%02X for row %d\n", data, s->row_index);
        if (s->row_index >= SNOWY_NUM_ROWS) {
          /* If this row index is in the bottom padding area, ignore it */
          s->row_index--;
        } else if (s->row_index >= SNOWY_ROWS_SKIPPED_AT_TOP) {
          /* If this row index is in the viewable area, save to our frame buffer */
          s->framebuffer[s->row_index * SNOWY_BYTES_PER_ROW + s->col_index] = data;
          s->row_index--;
        } else if (s->row_index > 0) {
          /* If this row index is in the top padding area, ignore it */
          s->row_index--;
        } else {
          /* We just received the last byte in the line, change state */
          s->state = LINENO;
        }
        break;
    }
    return 0;
}


/* This function maps an 8 bit value from the frame buffer into red, green, and blue
 * components */
typedef struct {
  uint8_t red, green, blue;
} PixelColor;

static PixelColor ps_display_get_rgb(uint8_t pixel_value) {

  PixelColor c;
  c.red = ((pixel_value & 0xC0) >> 6) * 255 / 3;
  c.green = ((pixel_value & 0x30) >> 4) * 255 / 3;
  c.red = ((pixel_value & 0x0C) >> 2) * 255 / 3;
  return c;

  /*
  static PixelColors[256] = {
    {0, 0, 0 },
  };

  return PixelColors[pixel_value];
  */
}


static void ps_display_update_display(void *arg)
{
    PSDisplayState *s = arg;
    DisplaySurface *surface = qemu_console_surface(s->con);

    uint8_t *d;
    int x, y, bpp, rgb_value;

    if (!s->redraw) {
        return;
    }

    bpp = surface_bits_per_pixel(surface);
    d = surface_data(surface);

    for (y = 0; y < SNOWY_NUM_ROWS; y++) {
        for (x = 0; x < SNOWY_NUM_COLS; x++) {
            uint8_t pixel = s->framebuffer[y * SNOWY_BYTES_PER_ROW + x];
            PixelColor color = ps_display_get_rgb(pixel);

            switch(bpp) {
                case 8:
                    *((uint8_t *)d) = rgb_to_pixel8(color.red, color.green, color.blue);
                    d++;
                    break;
                case 15:
                    *((uint16_t *)d) = rgb_to_pixel15(color.red, color.green, color.blue);;
                    d += 2;
                    break;
                case 16:
                    *((uint16_t *)d) = rgb_to_pixel16(color.red, color.green, color.blue);;
                    d += 2;
                    break;
                case 24:
                    rgb_value = rgb_to_pixel24(color.red, color.green, color.blue);
                    *d++ = (rgb_value & 0x00FF0000) >> 16;
                    *d++ = (rgb_value & 0x0000FF00) >> 8;
                    *d++ = (rgb_value & 0x000000FF);
                    break;
                case 32:
                    *((uint32_t *)d) = rgb_to_pixel32(color.red, color.green, color.blue);;
                    d += 4;
                    break;
            }
        }
    }

    dpy_gfx_update(s->con, 0, 0, SNOWY_NUM_COLS, SNOWY_NUM_ROWS);
    s->redraw = false;
}

static int ps_display_set_cs(SSISlave *dev, bool value)
{
    DPRINTF("CS changed to %d\n", value);
    return 0;
}

static void ps_display_set_reset_pin_cb(void *opaque, int n, int level)
{
    PSDisplayState *s = FROM_SSI_SLAVE(PSDisplayState, opaque);
    bool value = !!level;
    assert(n == 0);

    DPRINTF("RESET changed to %d\n", value);
    qemu_set_irq(s->done_output, false);
    qemu_set_irq(s->intn_output, false);
    if (!value) {
        s->sclk_count = 0;
    }
}

static void ps_display_set_sclk_pin_cb(void *opaque, int n, int level)
{
    PSDisplayState *s = FROM_SSI_SLAVE(PSDisplayState, opaque);
    assert(n == 0);

    s->sclk_count++;

    /* After a few cycles of sclck, say we are done */
    if (s->sclk_count > 10) {
      DPRINTF("Got %d sclocks\n", s->sclk_count);
      qemu_set_irq(s->done_output, true);
    }
}


static void ps_display_invalidate_display(void *arg)
{
    PSDisplayState *s = arg;
    s->redraw = true;
}

static const GraphicHwOps ps_display_ops = {
    .gfx_update = ps_display_update_display,
    .invalidate = ps_display_invalidate_display,
};

static int ps_display_init(SSISlave *dev)
{
    PSDisplayState *s = FROM_SSI_SLAVE(PSDisplayState, dev);

    s->con = graphic_console_init(DEVICE(dev), 0, &ps_display_ops, s);
    qemu_console_resize(s->con, SNOWY_NUM_COLS, SNOWY_NUM_ROWS);

    /* Create our inputs that will be connected to GPIOs from the STM32 */
    qdev_init_gpio_in_named(DEVICE(dev), ps_display_set_reset_pin_cb,
                            "pebble-snowy-display-reset", 1);

    /* Create our inputs that will be connected to GPIOs from the STM32 */
    qdev_init_gpio_in_named(DEVICE(dev), ps_display_set_sclk_pin_cb,
                            "pebble-snowy-display-sclk", 1);

    return 0;
}

static Property ps_display_init_properties[] = {
    DEFINE_PROP_PTR("done_output", PSDisplayState, vdone_output),
    DEFINE_PROP_PTR("intn_output", PSDisplayState, vintn_output),
    DEFINE_PROP_END_OF_LIST()
};


static void ps_display_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    dc->props = ps_display_init_properties;
    k->init = ps_display_init;
    k->transfer = ps_display_transfer;
    k->cs_polarity = SSI_CS_LOW;
    k->set_cs = ps_display_set_cs;
}

static const TypeInfo ps_display_info = {
    .name          = "pebble-snowy-display",
    .parent        = TYPE_SSI_SLAVE,
    .instance_size = sizeof(PSDisplayState),
    .class_init    = ps_display_class_init,
};

static void ps_display_register(void)
{
    type_register_static(&ps_display_info);
}

type_init(ps_display_register);
