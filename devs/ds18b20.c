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

  ds18b20.c (Dallas Semi DS18B20 temperature sensor)

*/

#include "1wire.h"
#include "ds18b20.h"



/*
 * read sensor mesurement
 */
int16_t ds18b20ReadTemperature(uint8_t bus, uint8_t * id)
{
    int16_t temperature = 0xfffe;

    // Reset, see if anyone present.
    if ( ! oneWireDetectPresence(bus) )
      return 0xfffd;

    // Match the id found earlier.
    oneWireMatchRom(id, bus);

    // Send start conversion command.
    oneWireSendByte(DS18B20_START_CONVERSION, bus);

    // Wait until conversion is finished.
    // Bus line is held low until conversion is finished.

// parasitic mode is very expensive
#ifdef ONEWIRE_USE_PARASITIC_POWER
    ONEWIRE_RELEASE_BUS(bus);
    _delay_ms(850);
#else
    while ( ! oneWireReadBit(bus) );
#endif

    // Reset, presence.
    if( ! oneWireDetectPresence(bus) )
      return 0xfffd;

    // Match id again.
    oneWireMatchRom(id, bus);

    // Send READ SCRATCHPAD command.
    oneWireSendByte(DS18B20_READ_SCRATCHPAD, bus);

    uint8_t pad[9];
    // fetch out scratchpad
    pad[0] = oneWireReceiveByte(bus); // tlsb
    pad[1] = oneWireReceiveByte(bus); // tmsb
    pad[2] = oneWireReceiveByte(bus); // t__h
    pad[3] = oneWireReceiveByte(bus); // t__l
    pad[4] = oneWireReceiveByte(bus); // conf
    pad[5] = oneWireReceiveByte(bus); // res1
    pad[6] = oneWireReceiveByte(bus); // res2
    pad[7] = oneWireReceiveByte(bus); // res3
    pad[8] = oneWireReceiveByte(bus); // crc8

    // calc crc
    uint8_t crc = 0x00;
    for( uint8_t k = 0; k < 8; k++ )
      crc = oneWireCrc8(pad[k], crc);

    // temp is valid
    if (crc == pad[8])
    {
      temperature  = pad[0];
      temperature |= (pad[1] << 8);
    }

    return temperature;
}

