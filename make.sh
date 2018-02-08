#!/bin/bash


rm -rf build/*
rm -rf *.hex

# 1280 byte
# boot reserve
ENTRYADDR=0x1C00

CFLAGS="-Wall -DAPPADDR=$ENTRYADDR -DF_CPU=8000000 -mrelax -mmcu=attiny85 -ffunction-sections -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wstrict-prototypes -std=gnu99 -MMD -MP -I./libs -I./devs"

echo
echo "GCC for AVR > 8.0.1 recommended."
echo

##
## MAIN
##

avr-gcc -c $CFLAGS -flto -MF build/ds18b20.o.d  -Wa,-adhlns=build/ds18b20.lst    devs/ds18b20.c  -o build/ds18b20.o
avr-gcc -c $CFLAGS -flto -MF build/sht21.o.d    -Wa,-adhlns=build/sht21.lst      devs/sht21.c    -o build/sht21.o
avr-gcc -c $CFLAGS -flto -MF build/si1145.o.d   -Wa,-adhlns=build/si1145.lst     devs/si1145.c   -o build/si1145.o
avr-gcc -c $CFLAGS -flto -MF build/bh1750.o.d   -Wa,-adhlns=build/bh1750.lst     devs/bh1750.c   -o build/bh1750.o
avr-gcc -c $CFLAGS -flto -MF build/bmp280.o.d   -Wa,-adhlns=build/bmp280.lst     devs/bmp280.c   -o build/bmp280.o

avr-gcc -c $CFLAGS -flto -MF build/eeprom.o.d   -Wa,-adhlns=build/eeprom.c.lst   libs/eeprom.c   -o build/eeprom.o
avr-gcc -c $CFLAGS -flto -MF build/crc8.o.d     -Wa,-adhlns=build/crc8.lst       libs/crc8.c     -o build/crc8.o
avr-gcc -c $CFLAGS -flto -MF build/crc16.o.d    -Wa,-adhlns=build/crc16.lst      libs/crc16.c    -o build/crc16.o
avr-gcc -c $CFLAGS -flto -MF build/1wire.o.d    -Wa,-adhlns=build/1wire.lst      libs/1wire.c    -o build/1wire.o
avr-gcc -c $CFLAGS -flto -MF build/softi2c.o.d  -Wa,-adhlns=build/softi2c.lst    libs/softi2c.c  -o build/softi2c.o
avr-gcc -c $CFLAGS -flto -MF build/atsens.o.d   -Wa,-adhlns=build/atsens.lst     libs/atsens.c   -o build/atsens.o
avr-gcc -c $CFLAGS -flto -MF build/usiuartx.o.d -Wa,-adhlns=build/usiuartx.c.lst libs/usiuartx.c -o build/usiuartx.o

avr-gcc -c $CFLAGS -flto -MF build/main.o.d     -Wa,-adhlns=build/main.lst       main.c          -o build/main.o

avr-gcc $CFLAGS -flto -o build/main.elf \
                build/main.o build/usiuartx.o build/crc16.o build/crc8.o build/sht21.o build/si1145.o build/bh1750.o \
                build/bmp280.o build/1wire.o build/softi2c.o build/atsens.o build/ds18b20.o build/eeprom.o \
        -Wl,--relax,--gc-sections,-Map=build/main.map


echo "MAIN:"
avr-size --format=avr --mcu=attiny85 build/main.elf


avr-objcopy -j .text -j .data -O ihex build/main.elf main.hex
avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0x0000 -O ihex build/main.elf boot.eep

mainsize=$(avr-size --format=avr --mcu=attiny85 build/main.elf | grep Program | awk '{print $2}')

printf "MAIN: 0x0000 - 0x%04x \n\n" $mainsize


##
## BOOT LOADER
##

avr-gcc -c $CFLAGS -flto -MF build/crt1.o.d     -Wa,-adhlns=build/crt1.c.lst     crt1.S          -o build/crt1.o
avr-gcc -c $CFLAGS -flto -MF build/softuart.o.d -Wa,-adhlns=build/softuart.c.lst libs/softuart.c -o build/softuart.o
avr-gcc -c $CFLAGS -flto -MF build/pgmflash.o.d -Wa,-adhlns=build/pgmflash.lst   libs/pgmflash.c -o build/pgmflash.o
avr-gcc -c $CFLAGS -flto -MF build/eeprom.o.d   -Wa,-adhlns=build/eeprom.c.lst   libs/eeprom.c   -o build/eeprom.o

avr-gcc -c $CFLAGS -flto -MF build/boot.o.d     -Wa,-adhlns=build/boot.lst       boot.c          -o build/boot.o

avr-gcc $CFLAGS -flto -ffreestanding -nostartfiles -o build/boot.elf build/crt1.o \
                build/boot.o build/softuart.o build/crc16.o \
                build/eeprom.o build/pgmflash.o \
        -Wl,--relax,--section-start=.text=$ENTRYADDR,--gc-sections,-Map=build/boot.map


echo "BOOT:"
avr-size --format=avr --mcu=attiny85 build/boot.elf

avr-objcopy -j .text -j .data -O ihex build/boot.elf boot.hex

bootsize=$(avr-size --format=avr --mcu=attiny85 build/boot.elf | grep Program | awk '{print $2}')

printf "BOOT: 0x%04x - 0x%04x \n\n" $ENTRYADDR $(( $ENTRYADDR+$bootsize ))

