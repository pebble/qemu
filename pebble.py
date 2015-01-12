#!/usr/bin/env python

import os
import argparse


####################################################################################################
if __name__ == '__main__':
    # Collect our command line arguments
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-S', '--start_in_monitor', action='store_true',
            help="Start in the monitor. Enter 'cont' in the monitor to start emulation")
    parser.add_argument('--vnc', action='store_true',
            help="Use VNC on port 5900 for the display")
    parser.add_argument('--vnc_ws', action='store_true',
            help="Use VNC over websockets on port 4444 for the display")
    parser.add_argument('-d', '--debug',
            help=("Turn on extensive debug logging. Available ones are:"
                  "out_asm,in_asm,op,op_opt,int,exec,cpu,pcall,cpu_reset,ioport,unimp,"
                  "guest_errors. Enter one or more comma separated."))
    args = parser.parse_args()


    cmd_line = (
        "qemu-system-arm "
        "-rtc base=localtime "
        "-s "                                 # Start gdb server
        "-serial file:uart1.log "
        "-serial tcp::12344,server,nowait "   # Used for Pebble Protocol
        "-serial tcp::12345,server,nowait "   # Used for console 
        "-monitor stdio "
    )
    cmd_line += "-machine pebble-%s " % (args.machine)
    cmd_line += "-cpu cortex-m3 "
    cmd_line += "-pflash ../test_images/qemu_micro_flash.bin "
    cmd_line += "-mtdblock ../test_images/qemu_spi_flash.bin "

    if args.start_in_monitor:
        cmd_line += "-S "
    if args.debug:
        cmd_line += ("-d " + args.debug)

    if (args.vnc and args.vnc_ws):
        raise RuntimeError("--vnc and --vnc_ws can not both be specified");
    if args.vnc:
        cmd_line += "-vnc :0 "
    if args.vnc_ws:
        cmd_line += "-vnc :1,websocket=4444 "

    print "Executing command line: \n   ", cmd_line
    os.system(cmd_line)


