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

  crt1.S (BootLoader init)

*/



#include <avr/io.h>

#ifdef __AVR_HAVE_JMP_CALL__
  #define XJMP jmp
#else
  #define XJMP rjmp
#endif

#define XCALL rcall

  ;
  ; no vector table
  ; just jump to __init
  ;
  .section .vectors, "ax", @progbits
  .global  __vectors
    XJMP    __init
    .func   __vectors
    .endfunc

  .section .init0,   "ax", @progbits
    .weak   __init

      __init:
          .weak  __heap_end
          .set   __heap_end, 0
          .weak  __stack
          .set   __stack, RAMEND

  ;
  ; dont reset stack pointer
  ; will be done in main()
  ;
  .section .init2,   "ax", @progbits
          CLR  R1
          ;OUT  0x3f, R1
          ;LDI  R28,  lo8(__stack)
          ;LDI  R29,  hi8(__stack)
          ;OUT  0x3d, r28
          ;OUT  0x3e, r29

  .section .init9,   "ax", @progbits
          XJMP  main
