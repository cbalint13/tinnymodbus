#!/bin/sh

# switch into bootloader
./examples/mainmode-switch.py

sleep .5

# flash over rs485 wires
./modbus-flash 0x01 ../main.hex

sleep .5

# switch back to main
./examples/bootmode-switch.py

