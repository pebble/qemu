#!/bin/sh

EMULATOR=arm-softmmu/qemu-system-arm

while getopts sSd opt; do
    case $opt in
    s)
        flags="$flags -s"
        ;;
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

echo FLAGS are $flags

# lldb --
    #-mtdblock qemu_spi_flash.bin \
#    -s \
#    -S  \

$EMULATOR -rtc base=localtime \
    -machine pebble-bb2 \
    -cpu cortex-m3 \
    $flags \
    -pflash ../test_images/qemu_micro_flash.bin \
    -mtdblock ../test_images/qemu_spi_flash.bin \
    -serial file:uart1.log \
    -serial file:uart2.log \
    -serial tcp::12345,server,nowait \
    -monitor stdio  \
    -singlestep \
    -d out_asm,in_asm,op,op_opt,int,exec,cpu,pcall,cpu_reset,ioport,unimp,guest_errors
    