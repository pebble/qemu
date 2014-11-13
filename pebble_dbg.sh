#!/bin/sh

EMULATOR=arm-softmmu/qemu-system-arm

while getopts Sd opt; do
    case $opt in
    S)
        flags="$flags -S"
        ;;
    d)
        # LOG_UNIMP
        # LOG_GUEST_ERROR
        flags="$flags -d unimp,guest_errors"
        ;;
    esac
done

# lldb --
    #-mtdblock qemu_spi_flash.bin \

$EMULATOR -rtc base=localtime \
    -machine pebble-bb2 \
    -cpu cortex-m3 \
    -s $flags \
    -pflash ../test_images/qemu_micro_flash.bin \
    -serial file:uart1.log \
    -serial file:uart2.log \
    -serial tcp::12345,server,nowait \
    -monitor stdio  \
    -S  \
    -singlestep \
    -d out_asm,in_asm,op,op_opt,int,exec,cpu,pcall,cpu_reset,ioport,unimp,guest_errors
    