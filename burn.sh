#!/bin/bash

#avrdude -c usbasp -P usb -p attiny85 -U lfuse:w:0xE2:m -U hfuse:w:0xDD:m -U efuse:w:0xFE:m -U flash:w:boot.hex -U eeprom:w:boot.eep -B3
avrdude  -c stk500v2 -P /dev/ttyACM0 -p attiny85 -U lfuse:w:0xE2:m -U hfuse:w:0xDD:m -U efuse:w:0xFE:m -U flash:w:boot.hex -U eeprom:w:boot.eep -B3

