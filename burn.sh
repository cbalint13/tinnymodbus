#!/bin/bash

# http://eleccelerator.com/fusecalc/fusecalc.php?chip=attiny85
# Low Fuse 0xE2: Int. RC Osc. 8MHz; Start-up time PWRDWN/RESET: 6CK/14CK+64ms;[CKSEL=0010 SUT=10]; default value
# High Fuse 0xDD: Brown-out detection level at VCC=2,7V;[BODLEVEL=101]; Serial program downloading (SPI) enable
# Extended Fuse 0xFE: Self Programming enable

avrdude -c usbasp -P usb -p attiny85 -U lfuse:w:0xE2:m -U hfuse:w:0xDD:m -U efuse:w:0xFE:m -U flash:w:boot.hex -U eeprom:w:boot.eep -B3 # original from @cbalint13
#avrdude  -c stk500v2 -P /dev/ttyACM0 -p attiny85 -U lfuse:w:0xE2:m -U hfuse:w:0xDD:m -U efuse:w:0xFE:m -U flash:w:boot.hex -U eeprom:w:boot.eep -B3 # altered for programmer from @stif
