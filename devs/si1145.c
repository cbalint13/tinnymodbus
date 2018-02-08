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

  si1145.c (SI1145 visible, infra, ultraviolet sensor)

*/

#include <util/delay.h>

#include "si1145.h"

#include "crc8.h"
#include "softi2c.h"


// dark frames offset
// self calibrate offset
uint16_t _vi_dark = 256;
uint16_t _ir_dark = 256;

/*
 *  write to parameter register
 */
static uint8_t writeParam(uint8_t i2caddr, uint8_t p, uint8_t v)
{
  i2c_write8(i2caddr, SI1145_REG_PARAMWR, v);
  i2c_write8(i2caddr, SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return i2c_read8(i2caddr, SI1145_REG_PARAMRD);
}

/*
 *  read from parameter register
 */
static uint8_t readParam(uint8_t i2caddr, uint8_t p)
{
  i2c_write8(i2caddr, SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
  return i2c_read8(i2caddr, SI1145_REG_PARAMRD);
}

/*
 * reset sensor
 */
void si1145_reset(void)
{
    // reset
    _delay_ms( 30 );
    i2c_write8(SI1145_ADDR, SI1145_REG_MEASRATE0, 0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_MEASRATE1, 0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_IRQEN,     0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_IRQMODE1,  0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_IRQMODE2,  0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_INTCFG,    0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_IRQSTAT,   0xFF);
    i2c_write8(SI1145_ADDR, SI1145_REG_COMMAND,   SI1145_RESET);
    _delay_ms( 10 );
    i2c_write8(SI1145_ADDR, SI1145_REG_HWKEY,     0x17);
    _delay_ms( 10 );
}

/*
 * init sensor
 */
void si1145_init(void)
{

    cli();

    // check ID = 0x45
    //i2c_read8(SI1145_ADDR, SI1145_REG_PARTID);

    si1145_reset();

    // Enable UVindex coeff
    i2c_write8(SI1145_ADDR, SI1145_REG_UCOEFF0, 0x7B);
    i2c_write8(SI1145_ADDR, SI1145_REG_UCOEFF1, 0x6B);
    i2c_write8(SI1145_ADDR, SI1145_REG_UCOEFF2, 0x01);
    i2c_write8(SI1145_ADDR, SI1145_REG_UCOEFF3, 0x00);


    // enables all sensors: UV, IR, Visible and Proximity
    writeParam(SI1145_ADDR, SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |      // UV
#ifdef SI1145_WITH_TEMP
                                                 SI1145_PARAM_CHLIST_ENAUX |     // Temperature
#endif
#ifdef SI1145_WITH_PROXY
                                                 SI1145_PARAM_CHLIST_ENPS1 |     // Proximity
#endif
                                                 SI1145_PARAM_CHLIST_ENALSIR |   // IR
                                                 SI1145_PARAM_CHLIST_ENALSVIS ); // Visible

#ifdef SI1145_WITH_IRQ
    // enable interrupt on every sample
    i2c_write8(SI1145_ADDR, SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
    i2c_write8(SI1145_ADDR, SI1145_REG_IRQEN , SI1145_REG_IRQEN_ALSEVERYSAMPLE);
#else
    // 0: INT pin is never driven
    i2c_write8(SI1145_ADDR, SI1145_REG_INTCFG, 0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_IRQEN,  SI1145_REG_IRQEN_ALSEVERYSAMPLE);
#endif

#ifdef SI1145_WITH_TEMP
    // Monitor the temperature in the AUX ADC
    writeParam(SI1145_ADDR, SI1145_PARAM_AUXADCMUX, SI1145_PARAM_ADCMUX_TEMP);
#endif

#ifdef SI1145_WITH_PROXY

    /******************** Prox Sense 1 ***********************/
    // program LED current
    i2c_write8(SI1145_ADDR, SI1145_REG_PSLED21,       0x03); // 20mA for LED 1 only
    writeParam(SI1145_ADDR, SI1145_PARAM_PS1ADCMUX,   SI1145_PARAM_ADCMUX_LARGEIR);
    // prox sensor #1 uses LED #1
    writeParam(SI1145_ADDR, SI1145_PARAM_PSLED12SEL,  SI1145_PARAM_PSLED12SEL_PS1LED1);
    // fastest clocks, clock div 1
    writeParam(SI1145_ADDR, SI1145_PARAM_PSADCGAIN,   0);
    // take 511 clocks to measure
    writeParam(SI1145_ADDR, SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in prox mode, high range
    writeParam(SI1145_ADDR, SI1145_PARAM_PSADCMISC,   SI1145_PARAM_PSADCMISC_RANGE |
                                               SI1145_PARAM_PSADCMISC_PSMODE);
#endif
    /******************** IR Sensor ***************************/
    // Use the small IR diode
    writeParam(SI1145_ADDR, SI1145_PARAM_ALSIRADCMUX,     SI1145_PARAM_ADCMUX_SMALLIR);
    // fastest clocks, clock div 1
    writeParam(SI1145_ADDR, SI1145_PARAM_ALSIRADCGAIN,    0);
    // take 511 clocks to measure
    writeParam(SI1145_ADDR, SI1145_PARAM_ALSIRADCOUNTER,  SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode
    writeParam(SI1145_ADDR, SI1145_PARAM_ALSIRADCMISC,    SI1145_PARAM_ALSIRADCMISC_RANGE);

    /******************** Visible Sensor **********************/
    // fastest clocks, clock div 1
    writeParam(SI1145_ADDR, SI1145_PARAM_ALSVISADCGAIN,   0);
    // take 511 clocks to measure
    writeParam(SI1145_ADDR, SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode (not normal signal)
    writeParam(SI1145_ADDR, SI1145_PARAM_ALSVISADCMISC,   SI1145_PARAM_ALSVISADCMISC_VISRANGE);

    // measurement rate
#ifdef SI1145_MEAS_AUTO
    // auto @ 8ms
    i2c_write8(SI1145_ADDR, SI1145_REG_MEASRATE0, 0xFF);
#else
    // set the module in forced mode
    i2c_write8(SI1145_ADDR, SI1145_REG_MEASRATE0, 0x00);
    i2c_write8(SI1145_ADDR, SI1145_REG_MEASRATE1, 0x00);
#endif

  // meas mode
#ifdef SI1145_MEAS_AUTO
  #ifdef SI1145_WITH_PROXY
    i2c_write8(SI1145_ADDR, SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
  #else
    i2c_write8(SI1145_ADDR, SI1145_REG_COMMAND, SI1145_ALS_AUTO);
  #endif
#endif
    sei();
}

/*
 * read sensor measurement
 */
float si1145_read_value( const uint8_t TYPE )
{
    i2c_init();

#ifndef SI1145_MEAS_AUTO
    // ALS force command
    // worse case is 3*25.55x2^7 = 3*3.28ms = 9.84ms
    i2c_write8(SI1145_ADDR, SI1145_REG_COMMAND, SI1145_ALS_FORCE);
    _delay_ms(15);
#endif

    float V;
    uint16_t R = i2c_read16(SI1145_ADDR, TYPE);

    switch ( TYPE )
    {
      case SI1145_READ_IR:
        {
          if (R < _ir_dark)
            _ir_dark = R;
          R -= _ir_dark;
          uint16_t range = readParam(SI1145_ADDR, SI1145_PARAM_ALSIRADCMISC);
          uint16_t sensi = readParam(SI1145_ADDR, SI1145_PARAM_ALSIRADCGAIN);
          float gain = ((range & 32) == 32) ? 14.5f : 1.0f;
          V = R * (gain / (2.44f * (1 << sensi)));
        }
        break;

      case SI1145_READ_VI:
        {
          if (R < _vi_dark)
            _vi_dark = R;
          R -= _vi_dark;
          uint16_t range = readParam(SI1145_ADDR, SI1145_PARAM_ALSVISADCMISC);
          uint16_t sensi = readParam(SI1145_ADDR, SI1145_PARAM_ALSVISADCGAIN);
          float gain = ((range & 32) == 32) ? 14.5f : 1.0f;
          V = R * (gain / (2.44f * (1 << sensi)));
        }
        break;

      case SI1145_READ_UV:
        V = (float)R / 100;
        break;

    } // end switch

    return V;
}
