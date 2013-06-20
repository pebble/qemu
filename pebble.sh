#!/bin/sh

EMULATOR=arm-softmmu/qemu-system-arm
DEBUGLOG=debug.log

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

truncate -s 0 $DEBUGLOG
$EMULATOR -rtc base=localtime -M pebble -s $flags -pflash micro_flash.bin \
    -serial file:uart1.log \
    -serial file:uart2.log \
    -serial file:$DEBUGLOG
