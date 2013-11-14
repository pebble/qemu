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

$EMULATOR -rtc base=localtime -M pebble-bb2 -s $flags -pflash micro_flash.bin \
    -mtdblock spi_flash.bin \
    -serial file:uart1.log \
    -serial file:uart2.log \
    -serial tcp::12345,server,nowait
