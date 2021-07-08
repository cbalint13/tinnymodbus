#!/bin/sh

ADDR=0x01

# switch into bootloader
./examples/mainmode-switch.py $ADDR

sleep .5

# flash over rs485 wires
./modbus-flash $ADDR ../main.hex

sleep .5

# switch back to main
./examples/bootmode-switch.py $ADDR

