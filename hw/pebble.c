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
#include "boards.h"

static void pebble1_init(QEMUMachineInitArgs *args) {
    Stm32Gpio *gpio[STM32F2XX_GPIO_COUNT];
    Stm32Uart *uart[STM32_UART_COUNT];
    DeviceState *spi_flash;
    SSIBus *spi;
    struct stm32f2xx stm;

    stm32f2xx_init(512, 128, args->kernel_filename, gpio, uart, 8000000,
      32768, &stm);

    /* SPI flash */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[0], "ssi");
    spi_flash = ssi_create_slave_no_init(spi, "m25p80");
    qdev_prop_set_string(spi_flash, "partname", "n25q032a");
    qdev_init_nofail(spi_flash);

    /* Display */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[1], "ssi");
    DeviceState *display_dev = ssi_create_slave_no_init(spi, "sm-lcd");
    qdev_init_nofail(display_dev);
}

static QEMUMachine pebble1_machine = {
    .name = "pebble",
    .desc = "Pebble smartwatch",
    .init = pebble1_init
};

static void pebble1_machine_init(void)
{
    qemu_register_machine(&pebble1_machine);
}

machine_init(pebble1_machine_init);
