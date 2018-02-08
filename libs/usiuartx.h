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

  usiuart.h (UART based on USI module)
  AVR307 - Half Duplex UART Using the USI Module on tinyAVR and megaAVR devices

*/


#ifndef USIUARTX_H
#define USIUARTX_H

// ============================================================================

#include <avr/io.h>

/*
 * USI UART Defines
 */
//#define SYSTEM_CLOCK    14745600
//#define SYSTEM_CLOCK    11059200
#define SYSTEM_CLOCK     8000000
//#define SYSTEM_CLOCK     7372800
//#define SYSTEM_CLOCK     3686400
//#define SYSTEM_CLOCK     2000000
//#define SYSTEM_CLOCK     1843200
//#define SYSTEM_CLOCK     1000000

//#define BAUDRATE 115200
////#define BAUDRATE  57600
#define BAUDRATE  38400
//#define BAUDRATE  28800
//#define BAUDRATE  19200
//#define BAUDRATE  14400
//#define BAUDRATE   9600

#define TIMER_PRESCALER  1
//#define TIMER_PRESCALER  8

// Transmit and Receive buffer size
// Must be power of 2, i.e. 2, 4, 8, etc.
#define USIUARTX_RX_BUFFER_SIZE        32
#define USIUARTX_TX_BUFFER_SIZE        32

// ----------------------------------------------------------------------------

/*
 * USI UART Defines
 */
#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER)
/*
 * TIMER0_SEED_COMPENSATION: timer0 seed compensation (for interrupt latency).
 * ATtiny85 = 2
 */
#define TIMER0_SEED_COMPENSATION  2
#define TIMER0_SEED               (256 - ( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER )) + TIMER0_SEED_COMPENSATION

#if ( (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) > (256 - INTERRUPT_STARTUP_DELAY) )
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 1/2) )
    #define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) )
    #define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
#endif

#define USIUARTX_RX_BUFFER_MASK ( USIUARTX_RX_BUFFER_SIZE - 1 )
#if ( USIUARTX_RX_BUFFER_SIZE & USIUARTX_RX_BUFFER_MASK )
    #error RX buffer size is not a power of 2
#endif

#define USIUARTX_TX_BUFFER_MASK ( USIUARTX_TX_BUFFER_SIZE - 1 )
#if ( USIUARTX_TX_BUFFER_SIZE & USIUARTX_TX_BUFFER_MASK )
    #error TX buffer size is not a power of 2
#endif

/*
 * General defines
 */
#define TRUE                      1
#define FALSE                     0

/*
 * functions
 */

void usiuartx_init(void);
int usiuartx_tx_has_data(void);
void usiuartx_tx_byte(const uint8_t);
void usiuartx_tx_string(const char *);
void usiuartx_tx_ln(void);
void usiuartx_tx_stringln(const char *);
void usiuartx_tx_array(const uint8_t *array, uint8_t len);
void usiuartx_rx_init(void);
int usiuartx_rx_has_data(void);
uint8_t usiuartx_rx_byte(void);

#endif
