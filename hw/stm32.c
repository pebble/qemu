#include "sysbus.h"
#include "arm-misc.h"
#include "devices.h"
#include "qemu-timer.h"
#include "boards.h"
#include "exec-memory.h"

#include "stm32f2xx_defines.h"

typedef const struct {
    const char *name;
} stm32_board_info;

/* Peripherials */
#include "stm32_sys.h"
#include "stm32_gpio.h"
#include "stm32_uart.h"

static void stm32_init(const char *kernel_filename, const char *cpu_model,
                           stm32_board_info *board)
{
    MemoryRegion *address_space_mem = get_system_memory();

    int flash_size = 256 * 1024; // 256k
    target_phys_addr_t flash_location = 0x08000000;
    int sram_size = 128 * 1024; // 128k
    qemu_irq *pic = armv7m_init(address_space_mem,
                                flash_size, flash_location, sram_size,
                                kernel_filename, cpu_model);
    (void) pic;

    stm32_sys_init();
    stm32_gpio_init();
    stm32_uart_init();
}

/* Board init */
static stm32_board_info stm32_boards[] = {
  { "STM32F205RE" }
};

static void stm32f2xx_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    stm32_init(kernel_filename, cpu_model, &stm32_boards[0]);
}

static QEMUMachine stm32f2xx_machine = {
    .name = "stm32f2xx",
    .desc = "STM32F2xx ",
    .init = stm32f2xx_init,
};

/* Global registration of machines */
static void stm32_machine_init(void)
{
    qemu_register_machine(&stm32f2xx_machine);
}

machine_init(stm32_machine_init);

/* Global registration of stm32 specific types */
static void stm32_register_types(void)
{
}

type_init(stm32_register_types)
