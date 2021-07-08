/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018,2021
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
uint8_t m_dig[32];

/*
 * apply temperature calibrations
 */
static int32_t calib_temp(int32_t adc_T)
{
   uint16_t dig_T1  = ((uint16_t)m_dig[1]) << 8 | m_dig[0];
    int16_t dig_T2  = ((uint16_t)m_dig[3]) << 8 | m_dig[2];
    int16_t dig_T3  = ((uint16_t)m_dig[5]) << 8 | m_dig[4];

    return (  (  ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *           ((int32_t)dig_T2))  >> 11)
            + ( (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14) );
}

static int32_t calib_pressure(int32_t adc_P)
{
   uint16_t dig_P1 = ((uint16_t)m_dig[7]) << 8 | m_dig[6];
    int16_t dig_P2 = ((uint16_t)m_dig[9]) << 8 | m_dig[8];
    int16_t dig_P3 = ((uint16_t)m_dig[11]) << 8 | m_dig[10];
    int16_t dig_P4 = ((uint16_t)m_dig[13]) << 8 | m_dig[12];
    int16_t dig_P5 = ((uint16_t)m_dig[15]) << 8 | m_dig[14];
    int16_t dig_P6 = ((uint16_t)m_dig[17]) << 8 | m_dig[16];
    int16_t dig_P7 = ((uint16_t)m_dig[19]) << 8 | m_dig[18];
    int16_t dig_P8 = ((uint16_t)m_dig[21]) << 8 | m_dig[20];
    int16_t dig_P9 = ((uint16_t)m_dig[23]) << 8 | m_dig[22];

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

    return (int32_t)((int32_t)P + ((var1 + var2 + dig_P7) >> 4));
}

static int16_t calib_humidity(int32_t adc)
{
  int8_t  dig_H1 = m_dig[24];
  int16_t dig_H2  = ((uint16_t)m_dig[26] << 8) | m_dig[25];
  int8_t  dig_H3 = m_dig[27];
  int16_t dig_H4  = ((uint16_t)m_dig[28] << 4) | (m_dig[29] & 0x0F);
  int16_t dig_H5  = ((uint16_t)m_dig[30] << 4) | (m_dig[29] >> 4);
  int8_t  dig_H6 = m_dig[31];

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
 * read calibration values
 */
void read_calibration( void )
{
    i2c_start(BME280_ADDRESS << 1);
    // 0x88 start of calibration value block
    i2c_write(BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG); 
    i2c_rep_start((BME280_ADDRESS << 1) | 0x01);

    // loop from 0x88 (dig_T1) to 0x A1 (dig_H1)
    int8_t idx = 0;

    for(idx = 0; idx<24; idx++)
    {
        m_dig[idx] = i2c_read(0);
    }
    m_dig[++idx] = i2c_read(1);
    i2c_stop();

    i2c_start(BME280_ADDRESS << 1);
    // get the last calibration bytes from 0xE1 on
    i2c_write(BME280_HUMIDITY_CALIB_DIG_H2_REG);
    i2c_rep_start( (BME280_ADDRESS <<1 ) | 0x01);

    for(idx = 25; idx <32; idx++)
    {
        m_dig[idx] = i2c_read(0);
    }
    m_dig[++idx] = i2c_read(1);
    i2c_stop();
}

/*
 * init sensor
 */
void bme280_init( void )
{
     i2c_init();
     read_calibration();
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
          V = calib_pressure(adc_P);
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
