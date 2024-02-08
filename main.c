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

  main.c (Main application)

*/

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "crc16.h"
#include "usiuartx.h"
#include "softi2c.h"
#include "atsens.h"

#ifdef DS18B20
#include "1wire.h"
#include "ds18b20.h"
#endif

#ifdef SHT21
#include "sht21.h"
#endif

#ifdef SHT31
#include "sht31.h"
#endif

#ifdef SI1145
#include "si1145.h"
#endif

#ifdef BH1750
#include "bh1750.h"
#endif

#ifdef BMP280
#include "bmp280.h"
#endif

#ifdef BME280
#include "bme280.h"
#endif


// globals
#ifdef ATSENS_H
extern float sensVcc;
extern float sensTmp;
#endif
extern uint8_t EEData[];

uint8_t si1145_done = 0x00;
uint8_t bh1750_done = 0x00;
uint8_t bmp280_done = 0x00;
uint8_t bme280_done = 0x00;

// software version string
static const char PROGMEM SWVers[4] = "0.06"; // 4 octet ASCII

/*
 *  embed and send modbus frame
 */
static void send_modbus_array( uint8_t *sendbuff, const uint8_t len )
{
    // calc checksum
    uint16_t csum = clcCRC16( sendbuff, len-2 );

    // insert checksum
    sendbuff[len-2] = csum & 0xFF;
    sendbuff[len-1] = csum >> 8;

    // send over rs485 wire
    usiuartx_tx_array( &sendbuff[0], len );
}

/*
 *  send modbus exception
 */
static void send_modbus_exception( uint8_t *sendbuff, uint8_t errcode )
{
    sendbuff[1] += 0x80;    // exception flag
    sendbuff[2]  = errcode; // exception code

    send_modbus_array( &sendbuff[0], 5 );
}

/*
 * main app
 */
int main(void)
{
    //OSCCAL = XXX; //Line to calibrate the internal oscillator. It is an integer number
    
    cli();

    // fetch own slave address from EEPROM
    uint8_t IdSv = eeprom_read_byte(&EEData[0]);

    #ifdef CALIBRATION
    // fetch Temperature Offset from EEPROM
    float CAL_TOffset = eeprom_read_byte(&EEData[1]) / 10.0f;
    // fetch Humidity Offset from EEPROM
    float CAL_HOffset = eeprom_read_byte(&EEData[2]) / 10.0f;
    #endif

    #ifdef ATSENS_H
    // internal
    // vcc+temp
    getSens( 4 );
    #endif

    #ifdef _DS18B20_H
    // 1w pin init
    DDRB &= ~(DDB4);
    oneWireInit(BUS);

    // 1wire devices store
    oneWireDevice devices[MAX_DEVICES];
    //oneWireSearchBuses(devices, MAX_DEVICES, BUS);
    #endif

    // init UART
    usiuartx_init();

    sei();

    /*
     * receive buffer for modbus frame (fcode = 3,4,6)
     *  ____________________________________________
     * |   1x   |   1x   |   2x   |   2x   |   2x   |
     * |--------+--------+--------+--------+--------|
     * | S addr | F code | D addr | D nums | CRC16  |
     * |________|________|________|________|________|
     *
     */
    uint8_t modbus[8];


    for (;;)
    {
        wdt_reset(); // clear watchdog

        if ( usiuartx_rx_has_data() ) // check if there is data in the Rx buffer
        {
            _delay_ms( 5 );  // wait a while for all the data to come

            while ( usiuartx_rx_has_data() ) // check if there is data in the internal buffer
            {
                // shift chain data to left
                for (uint8_t k = 1; k < 8; k++)
                    modbus[k-1] = modbus[k];

                // push into chain the new byte
                modbus[7] = usiuartx_rx_byte();  // receive a byte

                // is our address ?
                // is function code valid ?
                // only 3,4,6 function code ?
                if ( ( modbus[0] == IdSv ) &&
                     ( ( modbus[1] == 0x03 ) ||
                       ( modbus[1] == 0x04 ) ||
                       ( modbus[1] == 0x06 ) )
                   )
                {
                    // is short crc valid ?
                    uint16_t crc16 = modbus[6];
                    crc16 |= (uint16_t)modbus[7] << 8;

                    if ( crc16 == clcCRC16( modbus,  6 ) )
                    {
                        //
                        // Process MODBUS command
                        //

                        // data address
                        uint16_t daddr = modbus[3];
                        daddr |= (uint16_t)modbus[2] << 8;

                        // data amount
                        uint16_t dnums = modbus[5];
                        dnums |= (uint16_t)modbus[4] << 8;

                        // response buffer
                        uint8_t sendbuff[16];

                        // fill header
                        sendbuff[0] = IdSv; // slv addr

                        // triage cmd
                        switch( modbus[1] )
                        {
                            // read holding register
                            case 0x03:

                                sendbuff[1] = 0x03; // fcode

                                // return MODE
                                if ( daddr == 0x0000 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x01 ) break;

                                    sendbuff[2] = 0x02; // mslen

                                    sendbuff[3] = 0x00;
                                    sendbuff[4] = 0x00; // main app mode

                                    send_modbus_array( &sendbuff[0], 7 );
                                }

                                // return SW VERS
                                if ( daddr == 0x0001 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    // fetch software version
                                    sendbuff[3] = pgm_read_byte(&SWVers[0]);
                                    sendbuff[4] = pgm_read_byte(&SWVers[1]);
                                    sendbuff[5] = pgm_read_byte(&SWVers[2]);
                                    sendbuff[6] = pgm_read_byte(&SWVers[3]);

                                    send_modbus_array( &sendbuff[0], 9 );
                                }

                                // return ID SLAVE
                                if ( daddr == 0x0002 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x01 ) break;

                                    sendbuff[2] = 0x02; // mslen

                                    // read out id slave
                                    IdSv = eeprom_read_byte(&EEData[0]);

                                    // store id slave
                                    sendbuff[3] = 0x00;
                                    sendbuff[4] = IdSv;

                                    send_modbus_array( &sendbuff[0], 7 );
                                }
                                #ifdef ATSENS_H
                                // return internal VCC
                                if ( daddr == 0x0003 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    // reads
                                    getSens( 4 );

                                    // store Vcc
                                    sendbuff[3] = ((uint8_t*)(&sensVcc))[3];
                                    sendbuff[4] = ((uint8_t*)(&sensVcc))[2];
                                    sendbuff[5] = ((uint8_t*)(&sensVcc))[1];
                                    sendbuff[6] = ((uint8_t*)(&sensVcc))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }

                                // return internal TMP
                                if ( daddr == 0x0004 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    // reads
                                    getSens( 4 );

                                    // store Temp
                                    sendbuff[3] = ((uint8_t*)(&sensTmp))[3];
                                    sendbuff[4] = ((uint8_t*)(&sensTmp))[2];
                                    sendbuff[5] = ((uint8_t*)(&sensTmp))[1];
                                    sendbuff[6] = ((uint8_t*)(&sensTmp))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                #ifdef CALIBRATION
                                // read Temperature Offset
                                if ( daddr == 0x012 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    CAL_TOffset = eeprom_read_byte(&EEData[1]) / 10.0f;
                                    float V = CAL_TOffset;

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                // read Humidity Offset
                                if ( daddr == 0x022 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    CAL_HOffset = eeprom_read_byte(&EEData[2]) / 10.0f;
                                    float V = CAL_HOffset;

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                break; // fcode=0x03

                            // read input register
                            case 0x04:

                                sendbuff[1] = 0x04; // fcode

                                #ifdef _DS18B20_H
                                // return MAX_DEVICES
                                if ( daddr == 0x0000 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x01 ) break;

                                    sendbuff[2] = 0x02; // mslen

                                    sendbuff[3] = 0x00;
                                    sendbuff[4] = MAX_DEVICES;

                                    send_modbus_array( &sendbuff[0], 7 );
                                }

                                // return 1W NUMDEVS
                                if ( daddr == 0x0001 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x01 ) break;

                                    sendbuff[2] = 0x02; // mslen

                                    sendbuff[3] = 0x00;

                                    cli();

                                    uint8_t error = oneWireSearchBuses(devices, MAX_DEVICES, BUS);

                                    sei();

                                    // store 1w number of devices
                                    if (  error == ONEWIRE_SEARCH_COMPLETE )
                                    {
                                      sendbuff[4] = 0x00;
                                      for( uint8_t k = 0; k < MAX_DEVICES; k++ )
                                        if ( devices[k].bus == ONEWIRE_PIN_4 )
                                          sendbuff[4]++;

                                      send_modbus_array( &sendbuff[0], 7 );
                                    }
                                    else
                                    {
                                      // failed to respond
                                      send_modbus_exception( &sendbuff[0], 0x11 );
                                    }
                                }

                                // return 1W DEV SERIAL
                                if ( ( daddr >= 0x0100 ) &&
                                     ( daddr <= 0x0120 ) )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x04 ) break;

                                    sendbuff[2] = 0x08; // mslen

                                    // store 1w serial given the address
                                    uint8_t iddev = (uint8_t)(daddr - 0x0100);
                                    if ( devices[iddev].bus == ONEWIRE_PIN_4 )
                                    {
                                      for( uint8_t k = 0; k < 8; k++ )
                                        sendbuff[3+k] = devices[iddev].id[k];

                                      send_modbus_array( &sendbuff[0], 13 );
                                    }
                                    else
                                    {
                                      // illegal data address
                                      send_modbus_exception( &sendbuff[0], 0x02 );
                                    }
                                }

                                // return 1W DEV TEMP
                                if ( ( daddr >= 0x0200 ) &&
                                     ( daddr <= 0x0200 + MAX_DEVICES ) )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    // store 1w serial given the address
                                    uint8_t iddev = (uint8_t)(daddr - 0x0100);
                                    // check if device exists
                                    if ( devices[iddev].bus == ONEWIRE_PIN_4 )
                                    {
                                      float temp = 0.0f;

                                      cli();

                                      // attempt to read temp
                                      uint16_t w1temp = ds18b20ReadTemperature( BUS, devices[iddev].id );

                                      sei();

                                      if ( ( w1temp == 0xfffd ) &&
                                           ( w1temp == 0xfffe ) )
                                      {
                                        if ( w1temp == 0xfffd )
                                          send_modbus_exception( &sendbuff[0], 0x08 ); // parity error
                                        if ( w1temp == 0xfffe )
                                          send_modbus_exception( &sendbuff[0], 0x11 ); // failed to respond
                                      }
                                      else
                                      {
                                        // compute temp
                                        temp = (float)w1temp * 0.0625f;
                                        // store temp
                                        sendbuff[3] = ((uint8_t*)(&temp))[3];
                                        sendbuff[4] = ((uint8_t*)(&temp))[2];
                                        sendbuff[5] = ((uint8_t*)(&temp))[1];
                                        sendbuff[6] = ((uint8_t*)(&temp))[0];

                                        send_modbus_array( &sendbuff[0], 9 );
                                      }
                                    }
                                    else
                                    {
                                      // illegal data address
                                      send_modbus_exception( &sendbuff[0], 0x02 );
                                    }
                                }
                                #endif
                                #ifdef _SHT21_H
                                // return I2C DEV VALUES
                                if ( ( daddr >= 0x1200 ) &&
                                     ( daddr <= 0x1201 ) )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    float V;

                                    if ( daddr == 0x1200 )
                                    {
                                      V = (float)sht21ReadValue( SHT21_TEMP ) / 1000;
                                      #ifdef CALIBRATION
                                      V =  V + CAL_TOffset;
                                      #endif
                                    }

                                    if ( daddr == 0x1201 )
                                    {
                                      V = (float)sht21ReadValue( SHT21_HUMI ) / 1000;
                                      #ifdef CALIBRATION
                                      V =  V + CAL_HOffset;
                                      #endif
                                    }

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                #ifdef _SHT31_H
                                // return I2C DEV VALUES
                                if ( ( daddr >= 0x1250 ) &&
                                     ( daddr <= 0x1251 ) )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    float V;

                                    if ( daddr == 0x1250 )
                                    {
                                      V = (float)sht31ReadValue( SHT31_TEMP ) / 100;
                                      #ifdef CALIBRATION
                                      V =  V + CAL_TOffset;
                                      #endif
                                    }

                                    if ( daddr == 0x1251 )
                                    {
                                      V = (float)sht31ReadValue( SHT31_HUMI ) / 100;
                                      #ifdef CALIBRATION
                                      V =  V + CAL_HOffset;
                                      #endif
                                    }

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                #ifdef _SI1145_H
                                // return I2C DEV VALUES
                                if ( ( daddr >= 0x1210 ) &&
                                     ( daddr <= 0x1212 ) )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    if ( si1145_done == 0x00 )
                                    {
                                      si1145_init();
                                      si1145_done = 0x01;
                                    }

                                    float V;

                                    if ( daddr == 0x1210 )
                                      V = si1145_read_value(SI1145_READ_VI);
                                    if ( daddr == 0x1211 )
                                      V = si1145_read_value(SI1145_READ_IR);
                                    if ( daddr == 0x1212 )
                                      V = si1145_read_value(SI1145_READ_UV);

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                #ifdef _BH1750_H
                                // return I2C DEV VALUES
                                if ( daddr == 0x1220 )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    if ( bh1750_done == 0x00 )
                                    {
                                      bh1750_init();
                                      bh1750_done = 0x01;
                                    }

                                    float V = bh1750_read_value();

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                #ifdef _BMP280_H
                                // return I2C DEV VALUES
                                if ( ( daddr >= 0x1230 ) &&
                                     ( daddr <= 0x1231 ) )
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    if ( bmp280_done == 0x00 )
                                    {
                                      bmp280_init();
                                      bmp280_done = 0x01;
                                    }

                                    int32_t V;
                                    if ( daddr == 0x1230 )
                                    {
                                      V = bmp280_read_value( BMP280_TEMP );
                                      #ifdef CALIBRATION
                                      V =  V + CAL_TOffset;
                                      #endif
                                    }
                                    if ( daddr == 0x1231 )
                                      V = bmp280_read_value( BMP280_PRES );

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                #ifdef _BME280_H
                                // return I2C DEV VALUES
                                if ( ( daddr >= 0x1240 ) &&
                                     ( daddr <= 0x1242 ))
                                {
                                    // requested amount
                                    if ( modbus[5] != 0x02 ) break;

                                    sendbuff[2] = 0x04; // mslen

                                    if ( bme280_done == 0x00 )
                                    {
                                      bme280_init();
                                      bme280_done = 0x01;
                                    }

                                    int32_t V;
                                    if ( daddr == 0x1240 )
                                    {
                                      V = bme280_read_value( BME280_TEMP );
                                      #ifdef CALIBRATION
                                      V =  V + CAL_TOffset;
                                      #endif
                                    }
                                    if ( daddr == 0x1241 )
                                    {
                                      V = bme280_read_value( BME280_PRES );
                                    }
                                    if ( daddr == 0x1242 )
                                    {
                                      V = bme280_read_value( BME280_HUM );
                                      #ifdef CALIBRATION
                                      V =  V + CAL_HOffset;
                                      #endif
                                    }

                                    sendbuff[3] = ((uint8_t*)(&V))[3];
                                    sendbuff[4] = ((uint8_t*)(&V))[2];
                                    sendbuff[5] = ((uint8_t*)(&V))[1];
                                    sendbuff[6] = ((uint8_t*)(&V))[0];

                                    send_modbus_array( &sendbuff[0], 9 );
                                }
                                #endif
                                break; // fcode=0x04

                            // write input register
                            case 0x06:

                                sendbuff[1] = 0x06; // fcode

                                // change MODE
                                if ( daddr == 0x0000 )
                                {
                                    // value 0x00
                                    if ( ( modbus[4] == 0x00 ) &&
                                         ( modbus[5] == 0x01 ) )
                                    {
                                      // confirm mode switch
                                      usiuartx_tx_array( &modbus[0], 8 );

                                      _delay_ms( 50 );

                                      cli(); // suspend all interrupts

                                      // jump LOADER
                                      asm volatile
                                      (

                                          // 'FLASH' message into stack
                                          "  ldi   r30, 0x42                           \n\t" // 'B'
                                          "  push  r30                                 \n\t"
                                          "  ldi   r30, 0x4F                           \n\t" // 'O'
                                          "  push  r30                                 \n\t"
                                          "  ldi   r30, 0x4F                           \n\t" // 'O'
                                          "  push  r30                                 \n\t"
                                          "  ldi   r30, 0x54                           \n\t" // 'T'
                                          "  push  r30                                 \n\t"

                                          // jump into bootloader (0x1C00)
                                          "  ldi   r30, 0x00                           \n\t"
                                          "  ldi   r31, 0x0E                           \n\t"
                                          "  ijmp                                      \n\t"
                                      );

                                    }
                                    else
                                    {
                                      // illegal data value
                                      send_modbus_exception( &modbus[0], 0x03 );
                                    }
                                }

                                // write ID SLAVE
                                if ( daddr == 0x0001 )
                                {
                                    // values within 0x01 - 0xfe
                                    if ( ( modbus[4] == 0x00 ) &&
                                         ( ( modbus[5] > 0x00 ) &&
                                           ( modbus[5] < 0xff ) )
                                       )
                                    {
                                      // write new id slave
                                      IdSv = modbus[5];
                                      eeprom_write_byte( &EEData[0], IdSv );

                                      usiuartx_tx_array( &modbus[0], 8 );
                                    }
                                    else
                                    {
                                      // illegal data value
                                      send_modbus_exception( &sendbuff[0], 0x03 );
                                    }
                                }
                                #ifdef CALIBRATION
                                // write Temperature Offset (8bit signed int)
                                if ( daddr == 0x0011 )
                                {
                                    // values within 0x00 - 0xff
                                    if ( ( modbus[4] == 0x00 ) &&
                                         ( ( modbus[5] >= 0x00 ) &&
                                           ( modbus[5] <= 0xff ) )
                                       )
                                    {
                                      // write new Temperature Offset
                                      eeprom_write_byte( &EEData[1], modbus[5]);

                                      usiuartx_tx_array( &modbus[0], 8 );
                                    }
                                    else
                                    {
                                      // illegal data value
                                      send_modbus_exception( &sendbuff[0], 0x03 );
                                    }
                                }
                                // write Humidity Offset (8bit signed int)
                                if ( daddr == 0x0021 )
                                {
                                    // values within 0x00 - 0xff
                                    if ( ( modbus[4] == 0x00 ) &&
                                         ( ( modbus[5] >= 0x00 ) &&
                                           ( modbus[5] <= 0xff ) )
                                       )
                                    {
                                      // write new Humidity Offset
                                      eeprom_write_byte( &EEData[2], modbus[5]);

                                      usiuartx_tx_array( &modbus[0], 8 );
                                    }
                                    else
                                    {
                                      // illegal data value
                                      send_modbus_exception( &sendbuff[0], 0x03 );
                                    }
                                }
                                #endif
                                break; // fcode=0x06

                        } // end switch fcode

                    } // end with valid crc
                } // end valid modbus cmd
            } // end internal buffer
        } // end rx ring-buffer data
    } // end main loop

    return 0;
}
