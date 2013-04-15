# QEMU with STM32F2xx Microcontroller Implementation

## OVERVIEW
This is a copy of QEMU that has been modified to include an implementation of the STM32F2xx microcontroller.
__DANGER DANGER: It is very much a work-in-progress!__

This is based off of a QEMU fork that is targeting the STM32F103: https://github.com/beckus/qemu_stm32
However, this repo has been fast-forwarded to QEMU v1.4, so the QEMU code base is a bit newer than beckus' repo.

## BUILDING

Commands for a typical build:
        ./configure --disable-werror --enable-debug --target-list="arm-softmmu"
        make

Configure options that are useful when developing (tested only on OS X 10.8.3 / Mountain Lion):
        ./configure --enable-tcg-interpreter --extra-ldflags=-g --with-coroutine=gthread --enable-debug-tcg --enable-cocoa --enable-debug --disable-werror --target-list="arm-softmmu" --extra-cflags=-DDEBUG_CLKTREE --extra-cflags=-DDEBUG_STM32_RCC --extra-cflags=-DDEBUG_STM32_UART --extra-cflags=-DSTM32_UART_ENABLE_OVERRUN --extra-cflags=-DDEBUG_GIC

Useful make commands when rebuilding:
        make defconfig
        make clean

## RUNNING
The generated executable is arm-softmmu/qemu-system-arm .
        qemu-system-arm -M stm32-p205 -s -S -kernel your_firmware.elf

## QEMU
The original QEMU README follows:
Read the documentation in qemu-doc.html or on http://wiki.qemu.org
