#!/bin/sh

EMULATOR=arm-softmmu/qemu-system-arm

while getopts sSdt opt; do
    case $opt in
    S)
        flags="$flags -S"       # no not start CPU at start, type 'c' in monitor
        ;;
    d)
        # LOG_UNIMP
        # LOG_GUEST_ERROR
        flags="$flags -d out_asm,in_asm,op,op_opt,int,exec,cpu,pcall,cpu_reset,ioport,unimp,guest_errors"
        ;;
    t)
        flags="$flags -singlestep"
        ;;
    esac
done

echo FLAGS are $flags

# lldb --

$EMULATOR -rtc base=localtime \
    -s \
    -machine pebble-bb2 \
    -cpu cortex-m3 \
    $flags \
    -pflash ../test_images/qemu_micro_flash.bin \
    -mtdblock ../test_images/qemu_spi_flash.bin \
    -serial file:uart1.log \
    -serial tcp::12344,server,nowait \
    -serial tcp::12345,server,nowait \
    -monitor stdio  
    
