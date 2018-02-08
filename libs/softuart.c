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

  softuart.c (UART software mode implementation)

*/

#include "avr/interrupt.h"

#include "softuart.h"

/*
 * receive data frame
 */
uint8_t softuart_rx(void)
{
    uint8_t c;

    CLKPR = (1<<CLKPCE); // Prescaler enable
    CLKPR = (1<<CLKPS0)|(1<<CLKPS1); // Clock division factor 8 (0011)

    PORTB &= ~(1 << PORTB3);  // nRE low (read mode, MAX485)

    PORTB &= ~(1 << UART_RX);
    DDRB  &= ~(1 << UART_RX);

    __asm volatile
    (
      " ldi r18, %[rxdelay2] \n\t" // 1.5 bit delay
      " ldi %0, 0x80 \n\t" // bit shift counter
      "WaitStart: \n\t"
      " sbic %[uart_port]-2, %[uart_pin] \n\t" // wait for start edge
      " rjmp WaitStart \n\t"
      "RxBit: \n\t"
      // 6 cycle loop + delay - total = 5 + 3*r22
      // delay (3 cycle * r18) -1 and clear carry with subi
      " subi r18, 1 \n\t"
      " brne RxBit \n\t"
      " ldi r18, %[rxdelay] \n\t"
      " sbic %[uart_port]-2, %[uart_pin] \n\t" // check UART PIN
      " sec \n\t"
      " ror %0 \n\t"
      " brcc RxBit \n\t"
      "StopBit: \n\t"
      " dec r18 \n\t"
      " brne StopBit \n\t"
      : "=r" (c)
      : [uart_port] "I" (_SFR_IO_ADDR(PORTB)),
        [uart_pin]  "I" (UART_RX),
        [rxdelay]   "I" (RXDELAY),
        [rxdelay2]  "I" (RXDELAY2)
      : "r0","r18","r19"
    );

    CLKPR = (1<<CLKPCE); // Prescaler enable
    CLKPR = 0x00; // 8Mhz

    return c;
}

/*
 * send data frame
 */
void softuart_tx(uint8_t c)
{
    CLKPR = (1<<CLKPCE); // Prescaler enable
    CLKPR = (1<<CLKPS0)|(1<<CLKPS1); // Clock division factor 8 (0011)

    PORTB |= 1 << PORTB3;  // DE high (tx mode, MAX485)

    PORTB |= 1 << UART_TX;
    DDRB  |= 1 << UART_TX;

    __asm volatile
    (
      " cbi %[uart_port], %[uart_pin] \n\t" // start bit
      " in r0, %[uart_port] \n\t"
      " ldi r30, 3 \n\t" // stop bit + idle state
      " ldi r28, %[txdelay] \n\t"
      "TxLoop: \n\t"
      // 8 cycle loop + delay - total = 7 + 3*r22
      " mov r29, r28 \n\t"
      "TxDelay: \n\t"
      // delay (3 cycle * delayCount) - 1
      " dec r29 \n\t"
      " brne TxDelay \n\t"
      " bst %[ch], 0 \n\t"
      " bld r0, %[uart_pin] \n\t"
      " lsr r30 \n\t"
      " ror %[ch] \n\t"
      " out %[uart_port], r0 \n\t"
      " brne TxLoop \n\t"
      :
      : [uart_port] "I" (_SFR_IO_ADDR(PORTB)),
        [uart_pin]  "I" (UART_TX),
        [txdelay]   "I" (TXDELAY),
        [ch]        "r" (c)
      : "r0","r28","r29","r30"
    );

    CLKPR = (1<<CLKPCE); // Prescaler enable
    CLKPR = 0x00; // 8 Mhz
}

/*
 * send string
 */
void softuart_send(const uint8_t *s)
{
    while (*s) softuart_tx( *(s++) );
}

/*
 * send array
 */
void softuart_tx_array(const uint8_t *array, uint8_t len)
{
    while (len != 0 )
    {
        softuart_tx(*array++);
        len--;
    }
}
