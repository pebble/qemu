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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "stm32f2xx.h"
#include "hw/ssi.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "ui/console.h"

struct button_map {
    int gpio;
    int pin;
};

struct button_map button_map_bb2_ev1_ev2[] = {
    {STM32_GPIOC_INDEX, 3},
    {STM32_GPIOA_INDEX, 2},
    {STM32_GPIOC_INDEX, 6},
    {STM32_GPIOA_INDEX, 1},
};

struct button_map button_map_bigboard[] = {
    {STM32_GPIOA_INDEX, 2},
    {STM32_GPIOA_INDEX, 1},
    {STM32_GPIOA_INDEX, 3},
    {STM32_GPIOC_INDEX, 9}
};

struct button_state {
    qemu_irq irq;
    int pressed;
};

static void
pebble_key_handler(void *arg, int keycode)
{
    int pressed = (keycode & 0x80) == 0;
    int button_id;
    struct button_state *bs = arg;

    keycode &= 0x7f;
    switch (keycode) {
    case 16: /* Q */
        button_id = 0;
        break;
    case 17: /* W */
        button_id = 1;
        break;
    case 31: /* S */
        button_id = 2;
        break;
    case 45: /* X */
        button_id = 3;
        break;
    default:
        return;
    }

    if (bs[button_id].pressed == pressed) {
        return;
    }
    bs[button_id].pressed = pressed;

    printf("button %d %s\n", button_id, pressed ? "pressed" : "released");
    qemu_set_irq(bs[button_id].irq, !pressed);
}

static void pebble_init(MachineState *machine, struct button_map *map) {
    Stm32Gpio *gpio[STM32F2XX_GPIO_COUNT];
    Stm32Uart *uart[STM32_UART_COUNT];
    DeviceState *spi_flash;
    SSIBus *spi;
    struct stm32f2xx stm;

    stm32f2xx_init(512, 128, machine->kernel_filename, gpio, uart, 8000000,
      32768, &stm);

    /* SPI flash */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[0], "ssi");
    spi_flash = ssi_create_slave_no_init(spi, "n25q032a");
    qdev_init_nofail(spi_flash);

    qemu_irq cs;
    cs = qdev_get_gpio_in(spi_flash, 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOA_INDEX], 4, cs);

    /* Display */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[1], "ssi");
    DeviceState *display_dev = ssi_create_slave_no_init(spi, "sm-lcd");
    qdev_init_nofail(display_dev);

    /* UARTs */
    stm32_uart_connect(uart[0], serial_hds[0], 0);
    stm32_uart_connect(uart[1], serial_hds[1], 0);
    stm32_uart_connect(uart[2], serial_hds[2], 0); /* Debug */

    /* Buttons */
    static struct button_state bs[4];
    int i;
    for (i = 0; i < 4; i++) {
        bs[i].pressed = 0;
        bs[i].irq = qdev_get_gpio_in((DeviceState *)gpio[map[i].gpio], map[i].pin);
    }
    qemu_add_kbd_event_handler(pebble_key_handler, bs);
}

static void
pebble_bb2_init(MachineState *machine) {
    pebble_init(machine, button_map_bb2_ev1_ev2);
}

static QEMUMachine pebble_bb2_machine = {
    .name = "pebble-bb2",
    .desc = "Pebble smartwatch (bb2/ev1/ev2)",
    .init = pebble_bb2_init
};

static void
pebble_bb_init(MachineState *machine) {
    pebble_init(machine, button_map_bigboard);
}

static QEMUMachine pebble_bb_machine = {
    .name = "pebble",
    .desc = "Pebble smartwatch (bb)",
    .init = pebble_bb_init
};

static void pebble_machine_init(void)
{
    qemu_register_machine(&pebble_bb2_machine);
    qemu_register_machine(&pebble_bb_machine);
}

machine_init(pebble_machine_init);
