/*-
 * Copyright (c) 2013, 2014
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

#include "hw/ssi.h"
#include "hw/boards.h"
#include "hw/block/flash.h"
#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "ui/console.h"
#include "pebble_control.h"
#include "pebble.h"

// Disable jiggling the display when Pebble vibration is on.
//#define PEBBLE_NO_DISPLAY_VIBRATE

//#define DEBUG_PEBBLE
#ifdef DEBUG_PEBBLE
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_PEBBLE: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


const static PblBoardConfig s_board_config_bb2_ev1_ev2 = {
    .dbgserial_uart_index = 2,       // USART3
    .pebble_control_uart_index = 1,  // USART2
    .button_map = {
        {STM32_GPIOC_INDEX, 3},   /* back */
        {STM32_GPIOA_INDEX, 2},   /* up */
        {STM32_GPIOC_INDEX, 6},   /* select */
        {STM32_GPIOA_INDEX, 1},   /* down */
    },
    .flash_size = 4096,  /* Kbytes - larger to aid in development and debugging */
    .ram_size = 128,  /* Kbytes */
    .num_rows = 172,  /* not currently used */
    .num_cols = 148,  /* not currently used */
    .num_border_rows = 2,  /* not currently used */
    .num_border_cols = 2,  /* not currently used */
    .row_major = false,  /* not currently used */
    .round_mask = false  /* not currently used */
};

const static PblBoardConfig s_board_config_bigboard = {
    .dbgserial_uart_index = 2,       // USART3
    .pebble_control_uart_index = 1,  // USART2
    .button_map = {
        {STM32_GPIOA_INDEX, 2},
        {STM32_GPIOA_INDEX, 1},
        {STM32_GPIOA_INDEX, 3},
        {STM32_GPIOC_INDEX, 9}
    },
    .flash_size = 4096,  /* Kbytes - larger to aid in development and debugging */
    .ram_size = 128,  /* Kbytes */
    .num_rows = 172,  /* not currently used */
    .num_cols = 148,  /* not currently used */
    .num_border_rows = 2,  /* not currently used */
    .num_border_cols = 2,  /* not currently used */
    .row_major = false,  /* not currently used */
    .round_mask = false  /* not currently used */
};

const static PblBoardConfig s_board_config_snowy_bb = {
    .dbgserial_uart_index = 2,       // USART3
    .pebble_control_uart_index = 1,  // USART2
    .button_map = {
        {STM32_GPIOG_INDEX, 4},
        {STM32_GPIOG_INDEX, 3},
        {STM32_GPIOG_INDEX, 1},
        {STM32_GPIOG_INDEX, 2},
    },
    .flash_size = 4096,  /* Kbytes - larger to aid in development and debugging */
    .ram_size = 256,  /* Kbytes */
    .num_rows = 172,
    .num_cols = 148,
    .num_border_rows = 2,
    .num_border_cols = 2,
    .row_major = false,
    .round_mask = false
};

const static PblBoardConfig s_board_config_s4_bb = {
    .dbgserial_uart_index = 2,       // USART3
    .pebble_control_uart_index = 1,  // USART2
    .button_map = {
        {STM32_GPIOG_INDEX, 4},
        {STM32_GPIOG_INDEX, 3},
        {STM32_GPIOG_INDEX, 1},
        {STM32_GPIOG_INDEX, 2},
    },
    .flash_size = 4096,  /* Kbytes - larger to aid in development and debugging */
    .ram_size = 256,  /* Kbytes */
    .num_rows = 180,
    .num_cols = 180,
    .num_border_rows = 0,
    .num_border_cols = 0,
    .row_major = true,
    .round_mask = true
};

// ----------------------------------------------------------------------------------------
// Static globals
static PblButtonID s_waiting_key_up_id = PBL_BUTTON_ID_NONE;
static QEMUTimer *s_button_timer;

// This is the instance of pebble_control that intercepts packets sent to the emulated
// pebble over uart 2
static PebbleControl *s_pebble_control;

// The irq callbacks for each button
static qemu_irq s_button_irq[PBL_NUM_BUTTONS];
static qemu_irq s_button_wakeup;


static void prv_send_key_up(void *opaque)
{
    qemu_irq *button_irqs = opaque;
    if (s_waiting_key_up_id == PBL_BUTTON_ID_NONE) {
        /* Should never happen */
        return;
    }

    DPRINTF("button %d released\n", s_waiting_key_up_id);
    qemu_set_irq(button_irqs[s_waiting_key_up_id], true);
    qemu_set_irq(s_button_wakeup, false);
    s_waiting_key_up_id = PBL_BUTTON_ID_NONE;
}


// NOTE: When running using a VNC display, we alwqys get a key-up immediately after the key-down,
// even if the user is holding the key down. For long presses, this results in a series of
// quick back to back key-down, key-up callbacks.
static void pebble_key_handler(void *arg, int keycode)
{
    qemu_irq *button_irqs = arg;
    static int prev_keycode;

    int pressed = (keycode & 0x80) == 0;
    int button_id = PBL_BUTTON_ID_NONE;

    switch (keycode & 0x7F) {
    case 16: /* Q */
        button_id = PBL_BUTTON_ID_BACK;
        break;
    case 17: /* W */
        button_id = PBL_BUTTON_ID_UP;
        break;
    case 31: /* S */
        button_id = PBL_BUTTON_ID_SELECT;
        break;
    case 45: /* X */
        button_id = PBL_BUTTON_ID_DOWN;
        break;
    case 72: /* up arrow */
        if (prev_keycode == 224) {
            button_id = PBL_BUTTON_ID_UP;
        }
        break;
    case 80: /* down arrow */
        if (prev_keycode == 224) {
            button_id = PBL_BUTTON_ID_DOWN;
        }
        break;
    case 75: /* left arrow */
        if (prev_keycode == 224) {
            button_id = PBL_BUTTON_ID_BACK;
        }
        break;
    case 77: /* right arrow */
        if (prev_keycode == 224) {
            button_id = PBL_BUTTON_ID_SELECT;
        }
        break;
    default:
        break;
    }

    prev_keycode = keycode;
    if (button_id == PBL_BUTTON_ID_NONE || !pressed) {
        /* Ignore key ups and keys we don't care about */
        return;
    }

    // If this is a different key, and we are waiting for the prior one to key up, send the
    //  key up now
    if (s_waiting_key_up_id != PBL_BUTTON_ID_NONE && button_id != s_waiting_key_up_id) {
        prv_send_key_up(button_irqs);
    }

    if (s_waiting_key_up_id != button_id) {
        DPRINTF("button %d pressed\n", button_id);
        s_waiting_key_up_id = button_id;
        qemu_set_irq(button_irqs[button_id], false);   // Pressed
        qemu_set_irq(s_button_wakeup, true);
    }

    /* Set or reschedule the timer to release the key */
    if (!s_button_timer) {
        s_button_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, prv_send_key_up, button_irqs);
    }
    timer_mod(s_button_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 250);
}


// ------------------------------------------------------------------------------------------
// This method used externally (by pebble_control) for setting a given button state
void pebble_set_button_state(uint32_t button_state)
{
    // Toggle the GPIOs to match the new button state
    int button_id;
    for (button_id=0; button_id < PBL_NUM_BUTTONS; button_id++) {
      uint32_t mask = 1 << button_id;
      qemu_set_irq(s_button_irq[button_id], !(button_state & mask));   // Set new state
    }
}


// ------------------------------------------------------------------------------------------
// Connect up the uarts to serial drivers that connect to the outside world
void pebble_connect_uarts(Stm32Uart *uart[], const PblBoardConfig *board_config)
{
    // This UART is used for control messages, put in our pebble_control device in between
    // the qemu serial chr and the uart. This enables us to intercept and act selectively
    // act on messages sent to the Pebble in QEMU before they get to it.
    s_pebble_control = pebble_control_create(serial_hds[1],
                                             uart[board_config->pebble_control_uart_index]);

    stm32_uart_connect(uart[board_config->dbgserial_uart_index], serial_hds[2], 0);
}


// -----------------------------------------------------------------------------------------
// Init button handling
void pebble_init_buttons(Stm32Gpio *gpio[], const PblButtonMap *map)
{
    int i;
    for (i = 0; i < PBL_NUM_BUTTONS; i++) {
        s_button_irq[i] = qdev_get_gpio_in((DeviceState *)gpio[map[i].gpio], map[i].pin);
    }
    // GPIO A, pin 0 is the WKUP pin.
    s_button_wakeup = qdev_get_gpio_in((DeviceState *)gpio[STM32_GPIOA_INDEX], 0);
    qemu_add_kbd_event_handler(pebble_key_handler, s_button_irq);
}


// ----------------------------------------------------------------------------------------
// Init the board device
DeviceState *pebble_init_board(Stm32Gpio *gpio[], qemu_irq display_vibe)
{
    // Create the board device and wire it up
    DeviceState *board = qdev_create(NULL, "pebble_board");
    qdev_prop_set_ptr(board, "name", (void *)"Pebble");

#ifndef PEBBLE_NO_DISPLAY_VIBRATE
    qdev_prop_set_ptr(board, "display_vibe", display_vibe);
#endif

    qdev_init_nofail(board);
    return board;
}

// ----------------------------------------------------------------------------------------
// Set our QEMU specific settings to the target
void pebble_set_qemu_settings(DeviceState *rtc_dev)
{
    #define QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE  0x00000001
    #define QEMU_REG_0_START_CONNECTED          0x00000002
    #define QEMU_REG_0_START_PLUGGED_IN         0x00000004

    // Default settings
    uint32_t  flags = QEMU_REG_0_START_CONNECTED;

    // Set the QEMU specific settings in the extra backup registers
    char *strval;
    strval = getenv("PEBBLE_QEMU_FIRST_BOOT_LOGIC_ENABLE");
    if (strval) {
        // If set, allow "first boot" behavior, which displays the "Ready for Update"
        // screen
        if (atoi(strval)) {
            flags |= QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE;
        } else {
            flags &= ~QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE;
        }
    }

    strval = getenv("PEBBLE_QEMU_START_CONNECTED");
    if (strval) {
        // If set, default to bluetooth not connected
        if (atoi(strval)) {
            flags |= QEMU_REG_0_START_CONNECTED;
        } else {
            flags &= ~QEMU_REG_0_START_CONNECTED;
        }

    }

    strval = getenv("PEBBLE_QEMU_START_PLUGGED_IN");
    if (strval) {
        // If set, default to plugged in
        if (atoi(strval)) {
            flags |= QEMU_REG_0_START_PLUGGED_IN;
        } else {
            flags &= ~QEMU_REG_0_START_PLUGGED_IN;
        }
    }

    f2xx_rtc_set_extra_bkup_reg(rtc_dev, 0, flags);
}


// ------------------------------------------------------------------------------------------
// Instantiate a 32f2xx based pebble
static void pebble_32f2_init(MachineState *machine, const PblBoardConfig *board_config)
{
    Stm32Gpio *gpio[STM32F2XX_GPIO_COUNT];
    Stm32Uart *uart[STM32F2XX_UART_COUNT];
    Stm32Timer *timer[STM32F2XX_TIM_COUNT];
    DeviceState *spi_flash;
    DeviceState *rtc_dev;
    SSIBus *spi;
    struct stm32f2xx stm;
    ARMCPU *cpu;

    // Note: allow for bigger flash images (4MByte) to aid in development and debugging
    stm32f2xx_init(
        board_config->flash_size,
        board_config->ram_size,
        machine->kernel_filename,
        gpio,
        uart,
        timer,
        &rtc_dev,
        8000000, /* osc_freq*/
        32768, /* osc2_freq*/
        &stm,
        &cpu);


    // Set the Pebble specific QEMU settings on the target
    pebble_set_qemu_settings(rtc_dev);

    /* SPI flash */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[0], "ssi");
    spi_flash = ssi_create_slave_no_init(spi, "n25q032a11");
    qdev_init_nofail(spi_flash);

    qemu_irq cs;
    cs = qdev_get_gpio_in_named(spi_flash, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOA_INDEX], 4, cs);


    /* Display */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[1], "ssi");
    DeviceState *display_dev = ssi_create_slave_no_init(spi, "sm-lcd");
    qdev_init_nofail(display_dev);

    qemu_irq backlight_enable;
    backlight_enable = qdev_get_gpio_in_named(display_dev, "backlight_enable", 0);
    qdev_connect_gpio_out_named((DeviceState *)gpio[STM32_GPIOB_INDEX], "af", 5,
                                  backlight_enable);

    qemu_irq backlight_level;
    backlight_level = qdev_get_gpio_in_named(display_dev, "backlight_level", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[2], "pwm_ratio_changed", 0, // TIM3
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

    // The GPIO from vibrate drives the vibe input on the board
    qemu_irq board_vibe_in;
    board_vibe_in = qdev_get_gpio_in_named(board, "pebble_board_vibe_in", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOB_INDEX], 0, board_vibe_in);
}

// ------------------------------------------------------------------------------------------
// Instantiate a 32f439 based pebble
void pebble_32f439_init(MachineState *machine, const PblBoardConfig *board_config)
{
    Stm32Gpio *gpio[STM32F4XX_GPIO_COUNT];
    Stm32Uart *uart[STM32F4XX_UART_COUNT];
    Stm32Timer *timer[STM32F4XX_TIM_COUNT];
    DeviceState *rtc_dev;
    SSIBus *spi;
    struct stm32f4xx stm;
    ARMCPU *cpu;

    // Note: allow for bigger flash images (4MByte) to aid in development and debugging
    stm32f4xx_init(board_config->flash_size,
                   board_config->ram_size,
                   machine->kernel_filename,
                   gpio,
                   uart,
                   timer,
                   &rtc_dev,
                   8000000 /*osc_freq*/,
                   32768 /*osc2_freq*/,
                   &stm,
                   &cpu);


    // Set the Pebble specific QEMU settings on the target
    pebble_set_qemu_settings(rtc_dev);

    /* Storage flash (NOR-flash on Snowy) */
    const uint32_t flash_size_bytes = 16 * 1024 * 1024;  /* 16 MBytes */
    const uint32_t flash_sector_size_bytes = 32 * 1024;  /* 32 KBytes */
    const uint32_t bank_size_bytes = 2 * 1024 * 1024;  /* 2 MBytes */
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 1);   /* Use the 2nd -pflash drive */
    if (dinfo) {
        pflash_jedec_424_register(
            0x60000000,               /* flash_base*/
            NULL,                     /* qdev, not used */
            "mx29vs128fb",            /* name */
            flash_size_bytes,         /* size */
            blk_by_legacy_dinfo(dinfo),              /* driver state */
            flash_sector_size_bytes,  /* sector size */
            flash_size_bytes / flash_sector_size_bytes, /* number of sectors */
            bank_size_bytes,  /* size of each bank */
            2,                        /* width in bytes */
            0x00c2, 0x007e, 0x0065, 0x0001, /* id: 0, 1, 2, 3 */
            0                         /* big endian */
        );
    }


    /* --- Display ------------------------------------------------  */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[5], "ssi");
    DeviceState *display_dev = ssi_create_slave_no_init(spi, "pebble-snowy-display");

    /* Create the outputs that the display will drive and associate them with the correct
     * GPIO input pins on the MCU */
    qemu_irq display_done_irq = qdev_get_gpio_in((DeviceState *)gpio[STM32_GPIOG_INDEX], 9);
    qdev_prop_set_ptr(display_dev, "done_output", display_done_irq);
    qemu_irq display_intn_irq = qdev_get_gpio_in((DeviceState *)gpio[STM32_GPIOG_INDEX], 10);
    qdev_prop_set_ptr(display_dev, "intn_output", display_intn_irq);

    qdev_prop_set_int32(display_dev, "num_rows", board_config->num_rows);
    qdev_prop_set_int32(display_dev, "num_cols", board_config->num_cols);
    qdev_prop_set_int32(display_dev, "num_border_rows", board_config->num_border_rows);
    qdev_prop_set_int32(display_dev, "num_border_cols", board_config->num_border_cols);
    qdev_prop_set_uint8(display_dev, "row_major", board_config->row_major);
    qdev_prop_set_uint8(display_dev, "round_mask", board_config->round_mask);

    qdev_init_nofail(display_dev);

    /* Connect the correct MCU GPIO outputs to the inputs on the display */
    qemu_irq display_cs;
    display_cs = qdev_get_gpio_in_named(display_dev, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOG_INDEX], 8, display_cs);

    qemu_irq display_reset;
    display_reset = qdev_get_gpio_in_named(display_dev, "reset", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOG_INDEX], 15, display_reset);

    qemu_irq display_sclk;
    display_sclk = qdev_get_gpio_in_named(display_dev, "sclk", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOG_INDEX], 13, display_sclk);

    qemu_irq backlight_enable;
    backlight_enable = qdev_get_gpio_in_named(display_dev,
                                              "backlight_enable", 0);
    qdev_connect_gpio_out_named((DeviceState *)gpio[STM32_GPIOB_INDEX], "af", 14,
                                  backlight_enable);

    qemu_irq backlight_level;
    backlight_level = qdev_get_gpio_in_named(display_dev,
                                             "backlight_level", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[11], "pwm_ratio_changed", 0, // TIM12
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

    // The GPIO from vibrate drives the vibe input on the board
    qemu_irq board_vibe_in;
    board_vibe_in = qdev_get_gpio_in_named(board, "pebble_board_vibe_in", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOF_INDEX], 4, board_vibe_in);
}


// ================================================================================
// Pebble "board" device. Used when we need to fan out a GPIO outputs to one or more other
// devices/instances
typedef struct PebbleBoard {
    SysBusDevice busdev;
    void *name;

    union {
      void *vibe_out_irq_prop;
      qemu_irq vibe_out_irq;
    };

} PebbleBoard;


static void pebble_board_vibe_ctl(void *opaque, int n, int level)
{
    PebbleBoard *s = (PebbleBoard *)opaque;
    assert(n == 0);

    // Tell the pebble control instance that we vibrated
    pebble_control_send_vibe_notification(s_pebble_control, level != 0);

    // Tell pass onto the vibe out output as well
    qemu_set_irq(s->vibe_out_irq, level);
}


static int pebble_board_init(SysBusDevice *dev)
{
    //PebbleBoard *s = FROM_SYSBUS(PebbleBoard, dev);

    /* This callback informs us that the vibrate is on/orr */
    qdev_init_gpio_in_named(DEVICE(dev), pebble_board_vibe_ctl,
                            "pebble_board_vibe_in", 1);

    return 0;
}

static Property pebble_board_properties[] = {
    DEFINE_PROP_PTR("name", PebbleBoard, name),
    DEFINE_PROP_PTR("display_vibe", PebbleBoard, vibe_out_irq_prop),
    DEFINE_PROP_END_OF_LIST(),
};

static void pebble_board_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    sc->init = pebble_board_init;
    dc->props = pebble_board_properties;
}

static const TypeInfo pebble_board_info = {
    .name          = "pebble_board",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PebbleBoard),
    .class_init    = pebble_board_class_init,
};

static void pebble_board_register_types(void)
{
    type_register_static(&pebble_board_info);
}

type_init(pebble_board_register_types)


// ================================================================================
// Machines
static void pebble_bb2_init(MachineState *machine)
{
    pebble_32f2_init(machine, &s_board_config_bb2_ev1_ev2);
}

static void pebble_bb_init(MachineState *machine)
{
    pebble_32f2_init(machine, &s_board_config_bigboard);
}

static void pebble_snowy_init(MachineState *machine)
{
    pebble_32f439_init(machine, &s_board_config_snowy_bb);
}

static void pebble_s4_init(MachineState *machine)
{
    pebble_32f439_init(machine, &s_board_config_s4_bb);
}


static void pebble_bb2_machine_init(MachineClass *mc)
{
    mc->desc = "Pebble smartwatch (bb2/ev1/ev2)";
    mc->init = pebble_bb2_init;
}

DEFINE_MACHINE("pebble-bb2", pebble_bb2_machine_init)

static void pebble_bb_machine_init(MachineClass *mc)
{
    mc->desc = "Pebble smartwatch (bb)";
    mc->init = pebble_bb_init;
}

DEFINE_MACHINE("pebble-bb", pebble_bb_machine_init)

static void pebble_snowy_bb_machine_init(MachineClass *mc)
{
    mc->desc = "Pebble smartwatch (snowy)";
    mc->init = pebble_snowy_init;
}

DEFINE_MACHINE("pebble-snowy-bb", pebble_snowy_bb_machine_init)

static void pebble_s4_bb_machine_init(MachineClass *mc)
{
    mc->desc = "Pebble smartwatch (s4)";
    mc->init = pebble_s4_init;
}

DEFINE_MACHINE("pebble-s4-bb", pebble_s4_bb_machine_init)

