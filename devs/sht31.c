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

  sht31.c (SHT31 temperature and humidity sensor)

*/

#include <util/delay.h>

#include "sht31.h"

#include "crc8.h"
#include "softi2c.h"


/*
 * read sensor measurements
 */
int32_t sht31ReadValue(uint8_t TYPE)
  {

    uint8_t buffer[6];
    uint32_t V;

    i2c_init();

    i2c_start(SHT31_ADDR<<1);
    //send command
    i2c_write(SHT31_MEAS_HIGHREP >> 8);
    i2c_write(SHT31_MEAS_HIGHREP & 0xFF);
    //i2c_write(SHT31_MEAS_HIGHREP);
    //wait for measurements to complete
    _delay_ms(20);
    //read buffers
    i2c_rep_start((SHT31_ADDR<<1)|0x1);

    /*
    buffer[0] = i2c_read(0); i2c_read(0); //temp msb
    buffer[1] = i2c_read(0); i2c_read(0); //temp lsb
    buffer[2] = i2c_read(0); i2c_read(0); //tenp crc
    buffer[3] = i2c_read(0); i2c_read(0); //hum msb
    buffer[4] = i2c_read(0); i2c_read(0); //hum lsb
    buffer[5] = i2c_read(0); i2c_read(1); //hum crc
    */
    int8_t idx = 0;
    for(idx = 0; idx<5; idx++) {
        buffer[idx] = i2c_read(0);
    }
    buffer[++idx] = i2c_read(1);

    i2c_stop();
    uint16_t rawTemperature = 0;
    uint16_t rawHumidity = 0;

    switch ( TYPE ) {
        case SHT31_TEMP:
          rawTemperature = ((uint16_t) buffer[0] << 8) + buffer[1];
          V =  (175 * ( rawTemperature / 65535.0) - 45) * 100;
          //V = rawTemperature;
          break;
        case SHT31_HUMI:
          rawHumidity = ((uint16_t) buffer[3] << 8) + buffer[4];
          V = (100 * ( rawHumidity / 65535.0 )) *100;
          //V = rawHumidity;
          break;
    } // end switch

  return V;
}


/*
 * read sensor serial number
 */
void sht31ReadSerial( uint8_t *sn )
{
  i2c_init();

  // low
  i2c_start(0x80);

  i2c_write(0xFA);
  i2c_write(0x0F);

  i2c_rep_start(0x81);

  sn[5] = i2c_read(0); i2c_read(0);
  sn[4] = i2c_read(0); i2c_read(0);
  sn[3] = i2c_read(0); i2c_read(0);
  sn[2] = i2c_read(0); i2c_read(1);

  i2c_stop();

  // high
  i2c_start(0x80);

  i2c_write(0xFC);
  i2c_write(0xC9);

  i2c_rep_start(0x81);

  sn[1] = i2c_read(0); sn[0] = i2c_read(0); i2c_read(0);
  sn[7] = i2c_read(0); sn[6] = i2c_read(0); i2c_read(1);

  i2c_stop();

}
