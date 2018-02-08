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

  bh1750.h (BH1750 visible light sensor)

*/

#ifndef _BH1750_H
#define _BH1750_H


#include <avr/io.h>
#include <avr/interrupt.h>


// No active state
#define BH1750_POWER_DOWN 0x00
// Wating for measurement cmd
#define BH1750_POWER_ON   0x01
// Reset data register value
#define BH1750_RESET      0x07


// Measurement at 1lx resolution.
// Measurement time is approx 120ms.
#define CONTINUOUS_HIGH_RES_MODE    0x10
// Measurement at 0.5lx resolution.
// Measurement time is approx 180ms.
#define CONTINUOUS_HIGH_RES_MODE_2  0x11
// Measurement at 4lx resolution.
// Measurement time is approx 16ms.
#define CONTINUOUS_LOW_RES_MODE     0x13
// Measurement at 1lx resolution.
// Measurement time is approx 120ms.
#define ONE_TIME_HIGH_RES_MODE      0x20
// Measurement at 0.5lx resolution.
// Measurement time is approx 120ms.
#define ONE_TIME_HIGH_RES_MODE_2    0x21
// Measurement at 1lx resolution.
// Measurement time is approx 120ms.
#define CONE_TIME_LOW_RES_MODE      0x23


#define BH1750_I2CADDR 0x23



void bh1750_init(void);
float bh1750_read_value(void);

#endif
