/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 *
 * Balint Cristian <cristian dot balint at gmail dot com>
 * Stefan Reichhard <s.reichhard@netMedia.pro>
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

  bme280.c (Bosch BME280 temperature, humidity and pressure sensor)

*/

#include "bme280.h"

#include "crc8.h"
#include "softi2c.h"

int32_t t_fine;
int32_t dig_H1;

/*
 * apply temperature calibrations
 */
static int32_t calib_temp(int32_t adc_T)
{
    i2c_start(BME280_ADDRESS<<1);
    i2c_write(BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG);

    i2c_rep_start((BME280_ADDRESS<<1)|0x01);

   uint16_t dig_T1  = i2c_read(0);
            dig_T1 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_T2  = i2c_read(0);
            dig_T2 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_T3  = i2c_read(0);
            dig_T3 |= ((uint16_t)i2c_read(1)) << 8;

    i2c_stop();

    return (  (  ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *           ((int32_t)dig_T2))  >> 11)
            + ( (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14) );
}


static int16_t calib_humidity(int32_t adc)
{
  i2c_start(BME280_ADDRESS<<1);
  i2c_write(BME280_HUMIDITY_CALIB_DIG_H2_REG); // start with H2 because H1 lies in another block..

  i2c_rep_start((BME280_ADDRESS<<1)|0x01);

  //int32_t dig_H1 = i2c_read(0);
  int16_t dig_H2  = i2c_read(0) | ((uint16_t)i2c_read(0)) << 8;
  uint8_t dig_H3 = i2c_read(0);
  int16_t dig_H4 = i2c_read(0) << 4;
  int8_t helper_E5 = i2c_read(0);
          dig_H4 |= helper_E5 & 0x0F;
  int16_t dig_H5 = i2c_read(0) << 4 | helper_E5 >> 4;
  int8_t dig_H6 = i2c_read(1);

  i2c_stop();

  int32_t var1;
  var1 = (t_fine - ((int32_t)76800));
  var1 = (((((adc << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) +
  ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)dig_H6)) >> 10) * (((var1 *
  ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
  ((int32_t)dig_H2) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);
  return ((uint32_t)(var1 >> 12))/1024.0*100;
  
}


/*
 * init sensor
 */
void bme280_init( void )
{
     i2c_init();
     // humidity 1x oversampling
     i2c_write8(BME280_ADDRESS, BME280_CTRL_HUM_REG, 0b001);
     // normal mode, temp & pressure sampling rate = 1 (001:1x Sampling; 11: normal mode)
     i2c_write8(BME280_ADDRESS, BME280_CTRL_MEAS_REG, 0b00100111);
     // standby time 1000 ms (101:1s; 000:filter off; )
     i2c_write8(BME280_ADDRESS, BME280_CONFIG_REG, 0b10100000);
     i2c_stop();
}

/*
 * read sensor measurements
 */
int32_t bme280_read_value( uint8_t TYPE )
{

    int32_t V;

    i2c_init();

    i2c_start(BME280_ADDRESS<<1);
    i2c_write(BME280_PRESSURE_CALIB_DIG_P1_LSB_REG);

    i2c_rep_start((BME280_ADDRESS<<1)|0x01);

   uint16_t dig_P1 = i2c_read(0);
            dig_P1 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P2  = i2c_read(0);
            dig_P2 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P3  = i2c_read(0);
            dig_P3 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P4  = i2c_read(0);
            dig_P4 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P5  = i2c_read(0);
            dig_P5 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P6  = i2c_read(0);
            dig_P6 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P7  = i2c_read(0);
            dig_P7 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P8  = i2c_read(0);
            dig_P8 |= ((uint16_t)i2c_read(0)) << 8;
    int16_t dig_P9  = i2c_read(0);
            dig_P9 |= ((uint16_t)i2c_read(0)) << 8;
            dig_H1  = i2c_read(1);  // first Humidity Correction Value is in this block..

    i2c_stop();

    i2c_start( BME280_ADDRESS << 1 );
    i2c_write( BME280_PRESSURE_MSB_REG );

    i2c_rep_start( (BME280_ADDRESS <<1 ) | 0x01 );

    uint32_t adc_P, adc_T, adc_H;
    adc_P  = ((uint32_t)i2c_read(0)) << 12;
    adc_P |= ((uint32_t)i2c_read(0)) <<  4;
    adc_P |= ((uint32_t)i2c_read(0)) >>  4;
    adc_T  = ((uint32_t)i2c_read(0)) << 12;
    adc_T |= ((uint32_t)i2c_read(0)) <<  4;
    adc_T |= ((uint32_t)i2c_read(0)) >>  4;
    adc_H  = ((uint16_t)i2c_read(0)) <<  8;
    adc_H |= ((uint16_t)i2c_read(1)) >>  8;
    
    i2c_stop();

    t_fine = calib_temp(adc_T);

    switch ( TYPE )
    {
        case BME280_TEMP:
        {
          V = ( t_fine * 5 + 128) >> 8;
        }
        break;

        case BME280_PRES:
        {
          uint32_t P;
          int32_t var1, var2;
          var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
          var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t)dig_P6);
          var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
          var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
          var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
          var1 = ((((32768+var1))*((int32_t)dig_P1))>>15);
          if (var1 == 0)
            return 0;
          P = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
          if( P < 0x80000000 )
            P = (P << 1) / ((uint32_t) var1);
          else
            P = (P / (uint32_t)var1) * 2;
          var1 = (((int32_t)dig_P9) * ((int32_t)(((P>>3) * (P>>3))>>13)))>>12;
          var2 = (((int32_t)(P>>2)) * ((int32_t)dig_P8))>>13;
          V = (int32_t)((int32_t)P + ((var1 + var2 + dig_P7) >> 4));
        }
        break;

        case BME280_HUM:
        {
          V = calib_humidity(adc_H);
        }
        break;

    } // end switch

    return V;
}
