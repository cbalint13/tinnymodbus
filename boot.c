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

  boot.c (BootLoader application)

*/


#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


#include "crc16.h"
#include "eeprom.h"
#include "pgmflash.h"
#include "softuart.h"




extern uint8_t IDModbus;

// modbus frames
uint8_t modbus[40];

// Software version string
static const char PROGMEM BLVers[4] = "0.01"; // 4 octet ASCII

static void send_modbus_array( uint8_t *sendbuff, const uint8_t len )
{
    // calc checksum
    uint16_t csum = clcCRC16( sendbuff, len-2 );

    // insert checksum
    sendbuff[len-2] = csum & 0xFF;
    sendbuff[len-1] = csum >> 8;

    // send over rs485 wire
    softuart_tx_array( &sendbuff[0], len );
}

static void send_modbus_exception( uint8_t *sendbuff, uint8_t errcode )
{
    sendbuff[1] += 0x80;    // exception flag
    sendbuff[2]  = errcode; // exception code

    send_modbus_array( &sendbuff[0], 5 );
}

int main( void )
{

    cli();

    wdt_disable();

    __asm volatile
    (
      // jmp into bootload if pin 5 active
      "  cbi   %[ADDRPRTB], %[PINPB5]              \n\t"
      "  cbi   %[ADDRDDRB], %[PINPB5]              \n\t"
      "  sbic  %[ADDRPINB], %[PINPB5]              \n\t"
      "  rjmp  gotoflashld                         \n\t"

      // jmp into bootload if start vector is bad
      "  ldi   r30, 0x00                           \n\t"
      "  ldi   r31, 0x00                           \n\t"
      "  lpm   r29, Z+                             \n\t"
      "  cpi   r29, 0xFF                           \n\t"
      "  brne  gotoflashld                         \n\t"
      "  lpm   r29, Z                              \n\t"
      "  cpi   r29, 0xCD                           \n\t"
      "  brne  gotoflashld                         \n\t"

      // jmp into bootload if BOOT mark in stack
      "  pop   r30                                 \n\t"
      "  cpi   r30, 0x54                           \n\t" // 'T'
      "  brne  gotomainapp                         \n\t"
      "  pop   r30                                 \n\t"
      "  cpi   r30, 0x4F                           \n\t" // 'O'
      "  brne  gotomainapp                         \n\t"
      "  pop   r30                                 \n\t"
      "  cpi   r30, 0x4F                           \n\t" // 'O'
      "  brne  gotomainapp                         \n\t"
      "  pop   r30                                 \n\t"
      "  cpi   r30, 0x42                           \n\t" // 'B'
      "  brne  gotomainapp                         \n\t"
      "  rjmp  gotoflashld                         \n\t"

      // jump to main app
      // @end of vector table
      "gotomainapp:"
      "  ldi   r30, 0x0F                           \n\t"
      "  ldi   r31, 0x00                           \n\t"
      "  ijmp                                      \n\t"

      // bootload
      "gotoflashld:"
      // reset stack pointer to RAMEND
      "  ldi   r28,  lo8(__stack)                  \n\t"
      "  ldi   r29,  hi8(__stack)                  \n\t"
      "  out   0x3e, r29                           \n\t"
      "  out   0x3d, r28                           \n\t"
      :
      : [ADDRPRTB] "I" (_SFR_IO_ADDR(PORTB)),
        [ADDRDDRB] "I" (_SFR_IO_ADDR(DDRB)),
        [ADDRPINB] "I" (_SFR_IO_ADDR(PINB)),
        [PINPB5]   "I" (PB5)
    );

    // fetch own slave address from EEPROM
    uint8_t IdSv = eeprom_read_byte(&IDModbus);

    /*
     * receive buffer for modbus frame (fcode = 3,4,6)
     *  ____________________________________________
     * |   1x   |   1x   |   2x   |   2x   |   2x   |
     * |--------+--------+--------+--------+--------|
     * | S addr | F code | D addr | D nums | CRC16  |
     * |________|________|________|________|________|
     * modbus[32->39]
     */

    /*
     * receive ring buffer for modbus frame (fcode = 16)
     *  _____________________________________________________
     * |   1x   |   1x   |   2x   |   2x   |  32x   |   2x   |
     * |--------+--------+--------+--------+--------+--------|
     * | S addr | F code | D addr | D nums |  DATA  | CRC16  |
     * |________|________|________|________|________|________|
     * modbus[0->39]
     */

    // MAX485 rx/tx dir
    DDRB |= (1 << DDB3); // PB3 as output pin

    for (;;)
    {

        // shift chain data to left
        for (uint8_t k = 1; k < 40; k++)
            modbus[k-1] = modbus[k];

        // push into chain the new byte
        modbus[39] = softuart_rx();  // Receive a byte.

        // is our address ?
        // is function code 10 ?
        // is amount of regs 16 ?
        if ( ( modbus[0] == IdSv ) &&
             ( modbus[1] == 0x10 ) &&
             ( modbus[5] == 0x10 ) )
        {
            // is long crc valid ?
            uint16_t crc16 = modbus[38];
            crc16 |= (uint16_t)modbus[39] << 8;

            if ( crc16 == clcCRC16( modbus,  38 ) )
            {
                //
                // Process MODBUS command
                //

                // data address
                uint16_t daddr = modbus[3];
                daddr |= (uint16_t)modbus[2] << 8;


                // triage cmd
                switch( modbus[1] )
                {

                    // write multiple input register
                    // (we receive pages of 32 octets)
                    case 0x10:

                        // don't touch bootloader
                        // only chunks of 32 octet
                        if ( ( daddr <= 0x1BE0 ) &&
                             ( daddr % 0x20 == 0 ) )
                        {
                          // CD FF (RJMP 0x0DFF)
                          // overwrite start vector
                          // to point to bootloader
                          if ( daddr == 0x0000 )
                          {
                            modbus[6] = 0xFF;
                            modbus[7] = 0xCD;
                          }

                          // write page to flash
                          writePGMFlash( daddr );

                          // confirm with echo back
                          send_modbus_array( &modbus[0], 8 );
                        }
                        else
                        {
                          // illegal address value
                          send_modbus_exception( &modbus[0], 0x02 );
                        }

                        break; // fcode=0x10

                } // end switch fcode

            } // end with valid crc
        } // end valid modbus cmd

        // is our address ?
        // is function code valid ?
        // only 3,6 function code ?
        // only one register ?
        if ( ( modbus[32+0] == IdSv ) &&
             ( ( modbus[32+1] == 0x03 ) ||
               ( modbus[32+1] == 0x06 ) )
           )
        {
            // is short crc valid ?
            uint16_t crc16 = modbus[32+6];
            crc16 |= (uint16_t)modbus[32+7] << 8;

            if ( crc16 == clcCRC16( &modbus[32],  6 ) )
            {
                //
                // Process MODBUS command
                //
                // data address
                uint16_t daddr = modbus[32+3];
                daddr |= (uint16_t)modbus[32+2] << 8;

                // data amount
                uint16_t dnums = modbus[32+5];
                dnums |= (uint16_t)modbus[32+4] << 8;

                // triage cmd
                switch( modbus[32+1] )
                {

                    // read holding register
                    case 0x03:

                        // return MODE
                        if ( daddr == 0x0000 )
                        {
                            // requested amount
                            if ( modbus[32+5] != 0x01 )
                            {
                              // illegal data value
                              send_modbus_exception( &modbus[32+0], 0x03 );
                              break;
                            }

                            modbus[32+2] = 0x02; // mslen

                            modbus[32+3] = 0x00;
                            modbus[32+4] = 0x01; // bootload mode

                            send_modbus_array( &modbus[32+0], 7 );
                        }

                        // return SW VERS
                        if ( daddr == 0x0001 )
                        {
                            // requested amount
                            if ( modbus[32+5] != 0x02 )
                            {
                              // illegal data value
                              send_modbus_exception( &modbus[32+0], 0x03 );
                              break;
                            }

                            modbus[32+2] = 0x04; // mslen

                            // fetch software version
                            modbus[32+3] = pgm_read_byte(&BLVers[0]);
                            modbus[32+4] = pgm_read_byte(&BLVers[1]);
                            modbus[32+5] = pgm_read_byte(&BLVers[2]);
                            modbus[32+6] = pgm_read_byte(&BLVers[3]);

                            send_modbus_array( &modbus[32+0], 9 );
                        }

                        break; // fcode = 0x03

                    case 0x06:

                        // change MODE
                        if ( daddr == 0x0000 )
                        {
                            // expect set value 0x0000
                            if ( ( modbus[32+4] == 0x00 ) &&
                                 ( modbus[32+5] == 0x00 ) )
                            {
                              // confirm mode switch
                              softuart_tx_array( &modbus[32+0], 8 );

                              // watchdog @ 4 sec
                              wdt_enable( WDTO_4S );

                              // jump to main app
                              // @end of vector table
                              __asm volatile
                              (
                                " ldi     r30, 0x0F  \n\t"
                                " ldi     r31, 0x00  \n\t"
                                " ijmp               \n\t"
                              );

                            }
                            else
                            {
                              // illegal data value
                              send_modbus_exception( &modbus[32+0], 0x03 );
                            }
                        }

                        break; // fcode = 0x06

                } // end switch fcode
            } // end with valid crc
        } // end valid modbus cmd
    } // end main loop

    return 0;
}
