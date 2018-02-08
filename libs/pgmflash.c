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

  pgmflash.c (DATA FLASH burn routines)

*/

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "pgmflash.h"


// Packet sizes
#define DATA_SIZE 32 // 16 words (uint16_t)


//---------------------------------------------------------------------------
// Flash access
//---------------------------------------------------------------------------

// Check for SPM Control Register in processor.
#if defined (SPMCSR)
#  define __SPM_REG    SPMCSR
#elif defined (SPMCR)
#  define __SPM_REG    SPMCR
#else
#  error AVR processor does not provide bootloader support!
#endif

extern uint8_t modbus[40];
static uint8_t g_pagecache[SPM_PAGESIZE] __attribute__((used));


void writePGMFlash( const uint16_t addr )
{

  uint16_t page_address, address = addr;
  uint8_t page_hi, page_lo, index, written = 0;

  while( written < DATA_SIZE )
  {
    page_address = ((address / SPM_PAGESIZE) * SPM_PAGESIZE);
    page_hi = (page_address >> 8) & 0xFF;
    page_lo = page_address & 0xFF;

    // Read the page into the buffer
    for(index = 0; index < SPM_PAGESIZE; index++)
      g_pagecache[index] = pgm_read_byte_near(page_address + index);

    // Add in our data
    uint8_t offset = (uint8_t)(address - page_address);
    for(index = 0; (written<DATA_SIZE)&&((offset + index) < SPM_PAGESIZE); written++, index++)
      g_pagecache[offset + index] = modbus[written + 6];

    // Write the page
    asm volatile(
      // Y points to memory buffer, Z points to flash page
      "  mov   r30, %[page_lo]                     \n\t"
      "  mov   r31, %[page_hi]                     \n\t"
      "  ldi   r28, lo8(g_pagecache)               \n\t"
      "  ldi   r29, hi8(g_pagecache)               \n\t"
      // Wait for previous SPM to complete
      "  rcall wait_spm                            \n\t"
      // Erase the selected page
      "  ldi   r16, (1<<%[pgers]) | (1<<%[spmen])  \n\t"
      "  out   %[spm_reg], r16                     \n\t"
      "  spm                                       \n\t"
#if !defined(__AVR_ATtiny85__)
      // Wait for previous SPM to complete
      "  rcall wait_spm                            \n\t"
      // Re-enable the RWW section
      "  ldi   r16, (1<<%[rwwsre]) | (1<<%[spmen]) \n\t"
      "  out   %[spm_reg], r16                     \n\t"
      "  spm                                       \n\t"
#endif
      // Transfer data from RAM to Flash page buffer
      "  ldi   r20, %[spm_pagesize]                \n\t"
      "write_loop:                                 \n\t"
      // Wait for previous SPM to complete
      "  rcall wait_spm                            \n\t"
      "  ld    r0, Y+                              \n\t"
      "  ld    r1, Y+                              \n\t"
      "  ldi   r16, (1<<%[spmen])                  \n\t"
      "  out   %[spm_reg], r16                     \n\t"
      "  spm                                       \n\t"
      "  adiw  r30, 2                              \n\t"
      "  subi  r20, 2                              \n\t"
      "  brne  write_loop                          \n\t"
      // Wait for previous SPM to complete
      "  rcall wait_spm                            \n\t"
      // Execute page write
      "  mov   r30, %[page_lo]                     \n\t"
      "  mov   r31, %[page_hi]                     \n\t"
      "  ldi   r16, (1<<%[pgwrt]) | (1<<%[spmen])  \n\t"
      "  out   %[spm_reg], r16                     \n\t"
      "  spm                                       \n\t"
#if !defined(__AVR_ATtiny85__)
      // Wait for previous SPM to complete
      "  rcall wait_spm                            \n\t"
      // Re-enable the RWW section
      "  ldi   r16, (1<<%[rwwsre]) | (1<<%[spmen]) \n\t"
      "  out   %[spm_reg], r16                     \n\t"
      "  spm                                       \n\t"
#endif
      // Exit the routine
      "  rjmp   page_done                          \n\t"
      // Wait for SPM to complete
      "wait_spm:                                   \n\t"
      "  lds    r17, %[spm_reg]                    \n\t"
      "  andi   r17, 1                             \n\t"
      "  cpi    r17, 1                             \n\t"
      "  breq   wait_spm                           \n\t"
      "  ret                                       \n\t"
      "page_done:                                  \n\t"
      "  clr    __zero_reg__                       \n\t"
      :
      : [spm_pagesize] "M" (SPM_PAGESIZE),
        [spm_reg]      "I" (_SFR_IO_ADDR(__SPM_REG)),
        [spmen]        "I" (SPMEN),
        [pgers]        "I" (PGERS),
#if !defined(__AVR_ATtiny85__)
        [rwwsre]       "I" (RWWSRE),
#endif
        [pgwrt]        "I" (PGWRT),
        [page_hi]      "r" (page_hi),
        [page_lo]      "r" (page_lo)
      : "r0","r16","r17","r20","r28","r29","r30","r31");
    // Update addresses
    address += written;
    }
}


/** Fill the buffer from flash
 *
 * Uses the first two bytes of the global buffer as the address (high byte
 * followed by low byte) and fills the remainder of the buffer with DATA_SIZE
 * bytes from that address.
 */
void readPGMFlash(const uint16_t addr, uint8_t *buff)
{
  for(uint8_t i = 0; i < DATA_SIZE; i++)
    buff[i] = pgm_read_byte_near(addr + i);
}
