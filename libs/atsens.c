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

  atsens.c (attiny85 internal voltage and temeprature)

*/

#include <util/delay.h>

#include "atsens.h"



/*
 * ADC conversion
 */
static int32_t getADC( void )
{
    ADCSRA  |=_BV(ADSC);            // start conversion
    while( (ADCSRA & _BV(ADSC)) );    // wait until conversion is finished

    return ADC;
}

/*
 * init
 */
static void sensinit( void )
{
    // Setup the Analog to Digital Converter
    ADCSRA &= ~(_BV(ADATE) |_BV(ADIE)); // clear auto trigger and interrupt enable
}

/*
 * calibration parameters
 * http://www.atmel.com/Images/doc8108.pdf
 */
static float chipTemp(float raw)
{
    const float chipTempCoeff  = 1.075;  // value may vary
    const float chipTempOffset = 272.9;  // value may vary
    return ((raw - chipTempOffset) / chipTempCoeff);
}

// global storage
float sensTmp;
float sensVcc;

/*
 * get measurements
 */
void getSens( uint8_t iter )
{
    sensinit();

    float t_celsius;
    uint8_t vccIndex;
    float rawTemp = 0;
    float rawVcc = 0;

    // get chip temp
    ADCSRA |= _BV(ADEN);           // enable AD and start conversion
    ADMUX = 0xF | _BV( REFS1 );    // ADC4 (Temp Sensor) and Ref voltage = 1.1V;
    _delay_ms( 2 );                // settling time min 1 ms, our is 2 ms

    // average N measurements
    for (uint8_t i=0; i<iter; i++)
    {
      rawTemp += getADC();
    }
    rawTemp /=  iter;

    ADCSRA &= ~(_BV(ADEN));        // disable ADC


    // get chip voltage (Vcc)
    ADCSRA |= _BV(ADEN);           // enable ADC
    ADMUX  = 0x0c | _BV(REFS2);    // Use Vcc as voltage reference,
                                   //    bandgap reference as ADC input
    _delay_ms( 2 );                // settling time min 1 ms, our is 2 ms

    // average N measurements
    for (uint8_t i=0; i<iter; i++)
    {
        rawVcc += getADC();
    }
    rawVcc /= iter;
    rawVcc = 1024 * 1.1f / rawVcc;

    ADCSRA &= ~(_BV(ADEN));        // disable ADC

    // index 0..13 for vcc 1.7 ... 3.0
    vccIndex = min(max(17,(uint8_t)(rawVcc * 10)),30) - 17;

    // temperature compensation using the chip voltage
    // with 3.0 V VCC is 1 lower than measured with 1.7 V VCC
    t_celsius = (float)(chipTemp(rawTemp) + (float)vccIndex / 13);

    sensVcc = rawVcc;
    sensTmp = t_celsius;
}
