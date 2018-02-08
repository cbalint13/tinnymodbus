/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 *
 * Balint Cristian <cristian dot balint at gmail dot com>
 *
 * TinnyModbus
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*

  1wire.h (1WIRE protocol implementation)

*/

#ifndef ONEWIRE_H
#define ONEWIRE_H

#include <avr/io.h>

#define DS18B20_FAMILY_ID                0x28
#define DS18B20_START_CONVERSION         0x44
#define DS18B20_READ_SCRATCHPAD          0xbe
#define BUS                              ONEWIRE_PIN_4
#define MAX_DEVICES                      0x20
#define MAX_REPEATS                      20


/*
 * configuration
 */
#define     ONEWIRE_USE_INTERNAL_PULLUP
#define ONEWIRE_PORT              PORTB   // 1wire PORT sata
#define ONEWIRE_PIN               PINB    // 1wire INPUT pin
#define ONEWIRE_DDR               DDRB    // 1wire DATA DIRECTION

/*
 * bitmasks
 */
#define ONEWIRE_PIN_0             0x01
#define ONEWIRE_PIN_1             0x02
#define ONEWIRE_PIN_2             0x04
#define ONEWIRE_PIN_3             0x08
#define ONEWIRE_PIN_4             0x10
/*
 * ROM commands
 */
#define ONEWIRE_READ              (0x33)  // READ ROM
#define ONEWIRE_SKIP              (0xcc)  // SKIP ROM
#define ONEWIRE_MATCH             (0x55)  // MATCH ROM
#define ONEWIRE_SEARCH            (0xf0)  // SEARCH ROM

/*
 * return codes
 */
#define ONEWIRE_OK                (0x00)
#define ONEWIRE_ERROR             (0x01)
#define ONEWIRE_SEARCH_COMPLETE   (0x00)  // Search completed ok
#define ONEWIRE_SEARCH_FAILED     (0xff)  // Search failed

/*
 * Pull the 1-wire bus low.
 */
#define ONEWIRE_PULL_BUS_LOW(bitMask) \
            ONEWIRE_DDR    |=  bitMask; \
            ONEWIRE_PORT   &= ~bitMask;

#ifdef OWI_ONEWIRE_USE_INTERNAL_PULLUP
/*
 * 1-Wire pin(s) to input.
 * Enable internal pull-up.
 */
#define ONEWIRE_RELEASE_BUS(bitMask) \
            ONEWIRE_DDR    &= ~bitMask; \
            ONEWIRE_PORT   |=  bitMask;

#else
/*
 * 1-Wire pin(s) to input mode.
 * No internal pull-up enabled.
 */
#define ONEWIRE_RELEASE_BUS(bitMask) \
            ONEWIRE_DDR    &= ~bitMask; \
            ONEWIRE_PORT   &= ~bitMask;
#endif

/*
 * 1wire stucture
 */
typedef struct
{
    uint8_t bus;   // Bitmask of the bus that the device is on
    uint8_t id[8]; // The 64-bit identifier for the device
} oneWireDevice;

/*
 * API - CRC functions
 */
uint8_t oneWireCrc8(uint8_t inData, uint8_t seed);
unsigned int oneWireCrc16(uint8_t inData, unsigned int seed);
uint8_t oneWireRomCrc(uint8_t * romValue);

/*
 * API - bit functions
 */
void oneWireInit(uint8_t pins);
void oneWireWriteBit1(uint8_t pins);
void oneWireWriteBit0(uint8_t pins);
uint8_t oneWireReadBit(uint8_t pins);
uint8_t oneWireDetectPresence(uint8_t pins);

/*
 * API - 1wire functions
 */
void oneWireSendByte(uint8_t data, uint8_t pins);
void oneWireSkipRom(uint8_t pins);
void oneWireReadRom(uint8_t * romValue, uint8_t pins);
void oneWireMatchRom(uint8_t * romValue, uint8_t pins);
uint8_t oneWireReceiveByte(uint8_t pin);
uint8_t oneWireSearchRom(uint8_t * bitPattern, uint8_t lastDeviation, uint8_t pins);
uint8_t oneWireSearchBuses(oneWireDevice * devices, uint8_t len, uint8_t buses);
oneWireDevice * oneWireFindFamily(uint8_t familyID, oneWireDevice * devices, uint8_t size);

/*
 * timing parameters
 */
#define ONEWIRE_DELAY_480  (480)
#define ONEWIRE_DELAY_6    (6)
#define ONEWIRE_DELAY_9    (9)
#define ONEWIRE_DELAY_64   (64)
#define ONEWIRE_DELAY_60   (60)
#define ONEWIRE_DELAY_70   (70)
#define ONEWIRE_DELAY_10   (10)
#define ONEWIRE_DELAY_55   (55)
#define ONEWIRE_DELAY_410  (410)

#endif
