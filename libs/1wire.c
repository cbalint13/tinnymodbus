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

  1wire.c (1WIRE protocol implementation)

*/

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "1wire.h"



/*** Compute an 8-bit CRC
 *
 * Pass a seed of 0 to start the CRC of a byte-stream. Pass the result as
 * the seed for subsequent bytes
 */
uint8_t oneWireCrc8(uint8_t inData, uint8_t seed)
{
    for (uint8_t bitsLeft = 8; bitsLeft > 0; bitsLeft--)
    {
        if ( ((seed ^ inData) & 0x01) == 0 )
            seed >>= 1;
        else
        {
            seed ^= 0x18;
            seed >>= 1;
            seed |= 0x80;
        }
        inData >>= 1;
    }
    return seed;
}

/*** Compute and check the CRC for a ROM identifier
 *
 */
uint8_t oneWireRomCrc(uint8_t * romValue)
{
    uint8_t crc8 = 0;

    for ( uint8_t i = 0; i < 7; i++ )
    {
        crc8 = oneWireCrc8(*romValue, crc8);
        romValue++;
    }
    if ( crc8 == (*romValue) )
    {
        return ONEWIRE_OK;
    }
    return ONEWIRE_ERROR;
}

/*** Sends one byte of data on the 1-Wire(R) bus(es).
 *
 */
void oneWireSendByte(uint8_t data, uint8_t pins)
{
    // Do once for each bit
    for (uint8_t i = 0; i < 8; i++)
    {
        // Determine if lsb is '0' or '1' and transmit corresponding
        // waveform on the bus.
        if (data & 0x01)
            oneWireWriteBit1(pins);
        else
            oneWireWriteBit0(pins);
        // Right shift the data to get next bit.
        data >>= 1;
    }
}

/*** Receives one byte of data from the 1-Wire(R) bus.
 *
 */
uint8_t oneWireReceiveByte(uint8_t pin)
{

    // Clear the temporary input variable.
    uint8_t data = 0x00;

    // Do once for each bit
    for ( uint8_t i = 0; i < 8; i++ )
    {
        // Shift temporary input variable right.
        data >>= 1;

        // Set the msb if a '1' value is read from the bus.
        if ( oneWireReadBit(pin) )
            data |= 0x80;
    }

    return data;
}

/*** Sends the SKIP ROM command to the 1-Wire bus(es).
 *
 */
void OWI_SkipRom(uint8_t pins)
{
    // Send the SKIP ROM command on the bus.
    oneWireSendByte(ONEWIRE_SKIP, pins);
}

/*** Sends the READ ROM command to the 1-Wire bus(es).
 *
 */
void oneWireReadRom(uint8_t * romValue, uint8_t pin)
{
    uint8_t bytesLeft = 8;

    // Send the READ ROM command on the bus.
    oneWireSendByte(ONEWIRE_READ, pin);

    // Do 8 times.
    while (bytesLeft > 0)
    {
        // Place the received data in memory.
        *romValue++ = oneWireReceiveByte(pin);
        bytesLeft--;
    }
}

/*** Sends the MATCH ROM command to the 1-Wire bus(es).
 *
 */
void oneWireMatchRom(uint8_t * romValue, uint8_t pins)
{
    uint8_t bytesLeft = 8;

    // Send the MATCH ROM command.
    oneWireSendByte(ONEWIRE_MATCH, pins);

    // Do once for each byte.
    while (bytesLeft > 0)
    {
        // Transmit 1 byte of the ID to match.
        oneWireSendByte(*romValue++, pins);
        bytesLeft--;
    }
}

/*** Search for slave devices on the 1-wire bus.
 *
 * bitPattern ought to be an 8-byte buffer, initialised to 0 on first call,
 * then left to hold the returned value on subsequent calls. lastDeviation
 * ought to be 0 on first call, and the return value of the function on
 * subsequent calls. Pin is the bus to use
 */
uint8_t oneWireSearchRom(uint8_t * bitPattern, uint8_t lastDeviation, uint8_t pin)
{
    uint8_t bitA, bitB;
    uint8_t currentBit = 1;
    uint8_t newDeviation = 0;
    uint8_t bitMask = 0x01;

    // Send SEARCH ROM command on the bus.
    oneWireSendByte(ONEWIRE_SEARCH, pin);

    // Walk through all 64 bits.
    while (currentBit <= 64)
    {
        // Read bit from bus twice.
        bitA = oneWireReadBit(pin);
        bitB = oneWireReadBit(pin);

        if (bitA && bitB)
        {
            // Both bits 1 (Error).
            newDeviation = ONEWIRE_SEARCH_FAILED;
            return ONEWIRE_SEARCH_FAILED;
        }
        else if (bitA ^ bitB)
        {
            // Bits A and B are different. All devices have the same bit here.
            // Set the bit in bitPattern to this value.
            if (bitA)
                (*bitPattern) |=  bitMask;
            else
                (*bitPattern) &= ~bitMask;
        }
        else // Both bits 0
        {
            // If this is where a choice was made the last time,
            // a '1' bit is selected this time.
            if (currentBit == lastDeviation)
                (*bitPattern) |= bitMask;

            // For the rest of the id, '0' bits are selected when
            // discrepancies occur.
            else if (currentBit > lastDeviation)
            {
                (*bitPattern) &= ~bitMask;
                newDeviation = currentBit;
            }
            // If current bit in bit pattern = 0, then this is
            // out new deviation.
            else if ( !(*bitPattern & bitMask))
                newDeviation = currentBit;
            // IF the bit is already 1, do nothing.
            else {}
        }


        // Send the selected bit to the bus.
        if ((*bitPattern) & bitMask)
            oneWireWriteBit1(pin);
        else
            oneWireWriteBit0(pin);

        // Increment current bit.
        currentBit++;

        // Adjust bitMask and bitPattern pointer.
        bitMask <<= 1;
        if (!bitMask)
        {
            bitMask = 0x01;
            bitPattern++;
        }
    }
    return newDeviation;
}


/*** Initialise everything
 *
 */
void oneWireInit(uint8_t pins)
{
    ONEWIRE_RELEASE_BUS(pins);

    // The first rising edge can be interpreted by a slave as the end of a
    // Reset pulse. Delay for the required reset recovery time (480 uS) to be
    // sure that the real reset is interpreted correctly.
    _delay_us(ONEWIRE_DELAY_480);
}

/*** Write a '1' bit to the bus(es)
 *
 */
void oneWireWriteBit1(uint8_t pins)
{
    // Disable interrupts.
    uint8_t sreg = SREG;
    cli();

    // Drive bus low and delay.
    ONEWIRE_PULL_BUS_LOW(pins);
    _delay_us(ONEWIRE_DELAY_6);

    // Release bus and delay.
    ONEWIRE_RELEASE_BUS(pins);
    _delay_us(ONEWIRE_DELAY_64);

    // Restore interrupts.
    SREG=sreg;
}

/*** Write a '0' bit to the bus(es)
 *
 */
void oneWireWriteBit0(uint8_t pins)
{
    // Disable interrupts.
    uint8_t sreg = SREG;
    cli();

    // Drive bus low and delay.
    ONEWIRE_PULL_BUS_LOW(pins);
    _delay_us(ONEWIRE_DELAY_60);

    // Release bus and delay.
    ONEWIRE_RELEASE_BUS(pins);
    _delay_us(ONEWIRE_DELAY_10);

    // Restore interrupts.
    SREG=sreg;
}


/*** Read a bit from the bus(es).
 *
 */
uint8_t oneWireReadBit(uint8_t pins)
{
    uint8_t bitsRead;

    // Disable interrupts.
    uint8_t sreg = SREG;
    cli();

    // Drive bus low and delay.
    ONEWIRE_PULL_BUS_LOW(pins);
    _delay_us(ONEWIRE_DELAY_6);

    // Release bus and delay.
    ONEWIRE_RELEASE_BUS(pins);
    _delay_us(ONEWIRE_DELAY_9);

    // Sample bus and delay.
    bitsRead = ONEWIRE_PIN & pins;
    _delay_us(ONEWIRE_DELAY_55);

    // Restore interrupts.
    SREG=sreg;

    return bitsRead;
}

/*** Send a Reset signal and listen for Presence signal
 *
 */
uint8_t oneWireDetectPresence(uint8_t pins)
{
    uint8_t presenceDetected;

     // Disable interrupts.
    uint8_t sreg = SREG;
    cli();

     // Drive bus low and delay.
    ONEWIRE_PULL_BUS_LOW(pins);
    _delay_us(ONEWIRE_DELAY_480);

    // Release bus and delay.
    ONEWIRE_RELEASE_BUS(pins);
    _delay_us(ONEWIRE_DELAY_70);

    // Sample bus to detect presence signal and delay.
    presenceDetected = ((~ONEWIRE_PIN) & pins);
    _delay_us(ONEWIRE_DELAY_410);

    // Restore interrupts.
    SREG=sreg;

    return presenceDetected;
}


/*** Perform a 1-wire search on the identified buses
 *
 * Returns
 *   ONEWIRE_SEARCH_COMPLETE on success
 *   ONEWIRE_SEARCH_FAILED on error
 */
uint8_t oneWireSearchBuses(oneWireDevice * devices, uint8_t len, uint8_t bus)
{
    //
    // Initialize all addresses as zero, on bus 0 (does not exist).
    //
    for (uint8_t i = 0; i < len; i++)
    {
        devices[i].bus = 0x00;
        for (uint8_t j = 0; j < 8; j++)
            devices[i].id[j] = 0x00;
    }

    //
    // Find the buses with slave devices
    //
    uint8_t presence    = oneWireDetectPresence(bus);
    uint8_t numDevices  = 0;
    uint8_t *newID      = devices[0].id;

    //
    // Go through slave devices on bus
    //

    uint8_t lastDeviation   = 0;
    uint8_t *currentID      = newID;

    //
    // Are any devices available on this bus
    //
    if (presence)
    {
        //
        // Do slave search on bus, and place identifiers and
        // corresponding bus "addresses" in the array.
        //
        do
        {
            memcpy(newID, currentID, 8);
            oneWireDetectPresence(bus);
            lastDeviation = oneWireSearchRom(newID, lastDeviation, bus);
            currentID = newID;
            devices[numDevices].bus = bus;
            numDevices++;
            newID=devices[numDevices].id;
        }
        while( lastDeviation != ONEWIRE_SEARCH_COMPLETE );
    }

    //
    // Go through all the devices and do CRC check
    //
    for (uint8_t i = 0; i < numDevices; i++)
    {
        //
        // If any id has a crc error, return error
        //
        if(oneWireRomCrc(devices[i].id) == ONEWIRE_ERROR)
        {
            return ONEWIRE_SEARCH_FAILED;
        }
    }
    // Else, return Successful.
    return ONEWIRE_SEARCH_COMPLETE;
}

/*** Return a pointer to a one-wire descriptor that matches a given family id
 *
 */
oneWireDevice * oneWireFindFamily(uint8_t familyID, oneWireDevice * devices, uint8_t size)
{
    //
    // Search through the array
    //
    uint8_t i = 0;
    while (i < size)
    {
        // Return the pointer if there is a family id match.
        if ((*devices).id[0] == familyID)
            return devices;
        devices++;
        i++;
    }

    //
    // Couldn't find it
    //
    return NULL;
}
