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

#ifndef _SOFTUART_H_
#define _SOFTUART_H_

#include <avr/io.h>

#define F_DIV         8   // CLK divisor
#define UART_TX       PB1 // Use PB1 as TX pin
#define UART_RX       PB0 // Use PB0 as RX pin
#define UART_BAUDRATE 9600

#define TXDELAY         (int)(((F_CPU/F_DIV/UART_BAUDRATE)-7 +1.5)/3)
#define RXDELAY         (int)(((F_CPU/F_DIV/UART_BAUDRATE)-5 +1.5)/3)
#define RXDELAY2        (int)((RXDELAY*1.5)-2.5)
#define RXROUNDED       (((F_CPU/F_DIV/UART_BAUDRATE)-5 +2)/3)

#if RXROUNDED > 127
# error Low baud rates are not supported - use higher, UART_BAUDRATE
#endif

uint8_t softuart_rx(void);
void softuart_tx(uint8_t c);
void softuart_send(const uint8_t *s);
void softuart_tx_array(const uint8_t *array, uint8_t len);

#endif // end _SOFTUART_H_
