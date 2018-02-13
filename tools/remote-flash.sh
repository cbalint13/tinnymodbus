#!/bin/sh

# switch into bootloader
./examples/bootmode-switch.py

sleep .5

# flash over rs485 wires
./modbus-flash 0x01 ../main.hex

sleep .5

# switch back to main
./examples/mainmode-switch.py

