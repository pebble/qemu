#include "pebble.h"
#include "hw/boards.h"
#include "hw/ssi.h"

const static PblBoardConfig s_board_config_silk_bb = {
    .dbgserial_uart_index = 0,       // USART1
    .pebble_control_uart_index = 1,  // USART2 -- note this is also used by the DA14681 on real HW
    .button_map = {
        { STM32_GPIOC_INDEX, 13, true },
        { STM32_GPIOD_INDEX, 2, true },
        { STM32_GPIOH_INDEX, 0, true },
        { STM32_GPIOH_INDEX, 1, true },
    },
    .gpio_idr_masks = {
      [STM32_GPIOC_INDEX] = 1 << 13,
      [STM32_GPIOD_INDEX] = 1 << 2,
      [STM32_GPIOH_INDEX] = (1 << 1) | (1 << 0),
    },
    .flash_size = 4096,  /* Kbytes - larger to aid in development and debugging */
    .ram_size = 256,  /* Kbytes */
    .num_rows = 172,  /* not currently used */
    .num_cols = 148,  /* not currently used */
    .num_border_rows = 2,  /* not currently used */
    .num_border_cols = 2,  /* not currently used */
    .row_major = false,  /* not currently used */
    .row_inverted = false,  /* not currently used */
    .col_inverted = false,  /* not currently used */
    .round_mask = false  /* not currently used */
};

void pebble_32f412_init(MachineState *machine, const PblBoardConfig *board_config)
{
    Stm32Gpio *gpio[STM32F4XX_GPIO_COUNT];
    Stm32Uart *uart[STM32F4XX_UART_COUNT];
    Stm32Timer *timer[STM32F4XX_TIM_COUNT];
    DeviceState *qspi_flash;
    DeviceState *rtc_dev;
    SSIBus *spi;
    SSIBus *qspi;
    struct stm32f4xx stm;
    ARMCPU *cpu;

    // Note: allow for bigger flash images (4MByte) to aid in development and debugging
    stm32f4xx_init(board_config->flash_size,
                   board_config->ram_size,
                   machine->kernel_filename,
                   gpio,
                   board_config->gpio_idr_masks,
                   uart,
                   timer,
                   &rtc_dev,
                   8000000 /*osc_freq*/,
                   32768 /*osc2_freq*/,
                   &stm,
                   &cpu);


    // Set the Pebble specific QEMU settings on the target
    pebble_set_qemu_settings(rtc_dev);

    /* --- QSPI Flash ---------------------------------------------  */
    qspi = (SSIBus *)qdev_get_child_bus(stm.qspi_dev, "ssi");
    qspi_flash = ssi_create_slave_no_init(qspi, "mx25u6435f");
    qdev_init_nofail(qspi_flash);

    qemu_irq mx25u_cs = qdev_get_gpio_in_named(qspi_flash, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out_named(stm.qspi_dev, "qspi-gpio-cs", 0, mx25u_cs);


    /* --- Display ------------------------------------------------  */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[1], "ssi"); // SPI2
    DeviceState *display_dev = ssi_create_slave_no_init(spi, "sm-lcd");
    qdev_prop_set_bit(display_dev, "rotate_display", false);
    qdev_init_nofail(display_dev);

    qemu_irq backlight_enable;
    backlight_enable = qdev_get_gpio_in_named(display_dev, "backlight_enable", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOB_INDEX], 13, backlight_enable);

    qemu_irq backlight_level;
    backlight_level = qdev_get_gpio_in_named(display_dev, "backlight_level", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[2], // TIM3
                                "pwm_ratio_changed",
                                0,
                                backlight_level);

    qemu_irq display_power;
    display_power = qdev_get_gpio_in_named(display_dev, "power_ctl", 0);
    qdev_connect_gpio_out_named((DeviceState *)cpu->env.nvic, "power_out", 0,
                                  display_power);


    // Connect up the uarts
    pebble_connect_uarts(uart, board_config);

    // Init the buttons
    pebble_init_buttons(gpio, board_config->button_map);

    // Create the board device and wire it up
    qemu_irq display_vibe;
    display_vibe = qdev_get_gpio_in_named(display_dev, "vibe_ctl", 0);
    DeviceState *board = pebble_init_board(gpio, display_vibe);

    // The vibe PWM enables the qemu vibe
    qemu_irq board_vibe_in;
    board_vibe_in = qdev_get_gpio_in_named(board, "pebble_board_vibe_in", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[13], // TIM14
                                "pwm_enable",
                                0,
                                board_vibe_in);
}

static void pebble_silk_init(MachineState *machine)
{
    pebble_32f412_init(machine, &s_board_config_silk_bb);
}

static void pebble_silk_bb_machine_init(MachineClass *mc)
{
    mc->desc = "Pebble smartwatch (silk)";
    mc->init = pebble_silk_init;
}

DEFINE_MACHINE("pebble-silk-bb", pebble_silk_bb_machine_init)

