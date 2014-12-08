# Pebble Smartwatch QEMU Implementation

## Overview
This is a derivative of QEMU v2.1.1 that has been modified to include an implementation of the STM32F2xx microcontroller.
This is based off of a QEMU fork that is targeting the STM32F103: https://github.com/beckus/qemu_stm32.
This repo contains both beckus' STM32F1xx implementation and Pebble's STM32F2xx additions.

__DANGER DANGER: It is very much a work-in-progress! Only some of the peripherals are working at the moment. Please contribute!__

## Dependencies
QEMU requires that development packages for glib20 and pixman are installed.

### FreeBSD
Install the `devel/glib20` and `x11/pixman` ports.

### Linux

### Mac OS X

### Windows

## Building
Commands for a typical build:

        ./configure --disable-werror --enable-debug --target-list="arm-softmmu" \
        --extra-cflags=-DSTM32_UART_NO_BAUD_DELAY
        make

Summary set of configure options that are useful when developing (tested only on OS X 10.9.5):

        ./configure --enable-tcg-interpreter --extra-ldflags=-g \
        --with-coroutine=gthread --enable-debug-tcg --enable-cocoa \
        --enable-debug --disable-werror --target-list="arm-softmmu" \
        --extra-cflags=-DDEBUG_CLKTREE --extra-cflags=-DDEBUG_STM32_RCC \
        --extra-cflags=-DDEBUG_STM32_UART --extra-cflags=-DSTM32_UART_NO_BAUD_DELAY \
        --extra-cflags=-DDEBUG_GIC 
        
####Configure options which control the STM32 implementation:

    --extra-cflags=-DDEBUG_CLKTREE
        Print out clock tree debug statements.

    --extra-cflags=-DDEBUG_STM32_RCC
        Print RCC debug statements.

    --extra-cflags=-DDEBUG_STM32_UART
        Print UART debug statements.

    --extra-cflags=-DSTM32_UART_NO_BAUD_DELAY
        Disable the BAUD rate timing simulation
        (i.e. the UART will transmit or receive as fast as possible, rather than
        using a realistic delay).

    --extra-cflags=-DSTM32_UART_ENABLE_OVERRUN
        Enable setting of the overrun flag if a character is
        received before the last one is processed.  If this is not set, the UART
        will not receive the next character until the previous one is read by
        software.  Although less realisitic, it is safer NOT to use this, in case the VM is
        running slow.

####Other QEMU configure options which are useful for troubleshooting:
    --extra-cflags=-DDEBUG_GIC
        Extra logging around which interrupts are asserted

####qemu-system-arm options which are useful for troubleshooting:
    -d ?
        To see available log levels

    -d cpu,in_asm
        Enable logging to view the CPU state during execution and the ARM
        instructions which are being executed.  I believe --enable-debug must be
        used for this to work.


Useful make commands when rebuilding:

        make defconfig
        make clean

## Generating Images
* Use `./waf build qemu_image_spi` to generate `qemu_spi_flash.bin` from tintin.
* Use `./waf build qemu_image_micro` to generate `qemu_micro_flash.bin` from tintin.


### Under the covers of the images

QEMU's -pflash argument is used to specify a file to use as the micro flash.
An image can be created by concatenating the boot and main firmware files,
like so:

	truncate -s 64k tintin_boot.bin
	cat tintin_boot.bin tintin_fw.bin > micro_flash.bin
	truncate -s 512k micro_flash.bin

## Running
There is a convenience script `pebble.sh` that runs QEMU. It depends on the existence of (symlinked) images `qemu_micro_flash.bin` and `qemu_spi_flash.bin`.

### More details about running QEMU

The generated executable is arm-softmmu/qemu-system-arm .

Example:

        qemu-system-arm -rtc base=localtime -machine pebble-bb2 -cpu cortex-m3 -s \
        -pflash qemu_micro_flash.bin -mtdblock qemu_spi_flash.bin 

Adding `-S` to the commandline will have QEMU wait in the monitor at start;
the _c_ontinue command is necessary to start the virtual CPU.

## QEMU Docs
Read original the documentation in qemu-doc.html or on http://wiki.qemu.org

## QEMU Modifications
This emulator consists largely of new hardware device models; it includes
only minor changes to existing QEMU functionality.

The changes can be reviewed by running `git diff --diff-filter=M v1.5.0-backports`.

To list the added files, use `git diff --name-only --diff-filter=A v1.5.0-backports`.

## License

The following points clarify the QEMU license:

1. QEMU as a whole is released under the GNU General Public License

2. Parts of QEMU have specific licenses which are compatible with the
GNU General Public License. Hence each source file contains its own
licensing information.

Many hardware device emulation sources are released under the BSD license.

3. The Tiny Code Generator (TCG) is released under the BSD license
   (see license headers in files).

4. QEMU is a trademark of Fabrice Bellard.
