#!/bin/sh

EMULATOR=arm-softmmu/qemu-system-arm
DEBUGLOG=debug.log

Sflag=

while getopts S opt; do
    case $opt in
    S)
        Sflag="-S"
        ;;
    esac
done

truncate -s 0 $DEBUGLOG
$EMULATOR -rtc base=localtime -M pebble -s $Sflag -pflash micro_flash.bin \
    -serial file:uart1.log \
    -serial file:uart2.log \
    -serial file:$DEBUGLOG
