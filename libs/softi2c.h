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

  softi2c.h (I2C master mode protocol implementation)

*/

#ifndef SOFTI2C_H
#define SOFTI2C_H


#define SDA_PIN  PB4
#define SDA_PORT PORTB
#define SCL_PIN  PB2
#define SCL_PORT PORTB


#define I2C_FASTMODE 0
#define I2C_SLOWMODE 0
#define I2C_PULLUP 0
#define I2C_NOINTERRUPT 1
#define I2C_TIMEOUT 0


// Init function. Needs to be called once in the beginning.
// Returns false if SDA or SCL are low, which probably means
// a I2C bus lockup or that the lines are not pulled up.
uint8_t __attribute__ ((noinline)) i2c_init(void) __attribute__ ((used));

// Start transfer function: <addr> is the 8-bit I2C address (including the R/W bit).
// Return: true if the slave replies with an "acknowledge", false otherwise
uint8_t __attribute__ ((noinline)) i2c_start(uint8_t addr) __attribute__ ((used));

// Similar to start function, but wait for an ACK! Be careful, this can
// result in an infinite loop!
uint8_t  __attribute__ ((noinline)) i2c_start_wait(uint8_t addr) __attribute__ ((used));

// Repeated start function: After having claimed the bus with a start condition,
// you can address another or the same chip again without an intervening 
// stop condition.
// Return: true if the slave replies with an "acknowledge", false otherwise
uint8_t __attribute__ ((noinline)) i2c_rep_start(uint8_t addr) __attribute__ ((used));

// Issue a stop condition, freeing the bus.
void __attribute__ ((noinline)) i2c_stop(void) asm("ass_i2c_stop") __attribute__ ((used));

// Write one byte to the slave chip that had been addressed
// by the previous start call. <value> is the byte to be sent.
// Return: true if the slave replies with an "acknowledge", false otherwise
uint8_t __attribute__ ((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write") __attribute__ ((used));


// Read one byte. If <last> is true, we send a NAK after having received
// the byte in order to terminate the read sequence.
uint8_t __attribute__ ((noinline)) i2c_read(uint8_t last) __attribute__ ((used));

// high level functions
uint8_t i2c_read8(uint8_t i2caddr, uint8_t regaddr);
uint16_t i2c_read16(uint8_t i2caddr, uint8_t regaddr);
void i2c_write8(uint8_t i2caddr, uint8_t regaddr, uint8_t regval);

#endif

