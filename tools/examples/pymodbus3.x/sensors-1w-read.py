#!/usr/bin/python

"""
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
"""

"""

  sensor-1w-read.pw (Query all 1wire sensors values)

"""

import sys
import logging

from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.client import ModbusSerialClient as ModbusClient



logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=38400, timeout=1.5)
client.connect()

idslave = 0x01

if len(sys.argv) == 2:
  try:
    idslave = int(sys.argv[1])
  except:
    print ("usage: %s [idslave]" % sys.argv[0])
    sys.exit(-1)

print ("0x03 0x0000\n")
result  = client.read_holding_registers(address=0x0000, count=0x01, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_16bit_int(), " running mode\n")

print ("")

print ("0x03 0x0001\n")
result  = client.read_holding_registers(address=0x0001, count=0x02, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
x = decoder.decode_32bit_int();
print (''.join(chr((x>>8*(4-byte-1))&0xFF) for byte in range(4)) , " software version \n")

print ("")

print ("0x03 0x0002\n")
result  = client.read_holding_registers(address=0x0002, count=0x01, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_16bit_int(), " slave address\n")

print ("")

print ("0x03 0x0003\n")
result  = client.read_holding_registers(address=0x0003, count=0x02, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_32bit_float(), " Volt (internal)\n")

print ("")

print ("0x03 0x0004\n")
result  = client.read_holding_registers(address=0x0004, count=0x02, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_32bit_float(), " Celsius (internal)\n")

print ("")


print ("0x04 0x0000\n")
result  = client.read_input_registers(address=0x0000, count=0x01, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_16bit_int(), " maximum devices \n")

print ("")

print ("0x04 0x0001\n")
result  = client.read_input_registers(address=0x0001, count=0x01, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
value = decoder.decode_16bit_int()
print (value, " devices found\n")

# iterate sensors
for idx in range(0, value):

  print ("  id: %i 0x04 0x%04x" % (idx, 0x0100+idx))
  result  = client.read_input_registers(address=0x0100+idx, count=0x04, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print ("  %08x" % decoder.decode_64bit_uint())

  print ("  id: %i 0x04 0x%04x" % (idx, 0x0200+idx))
  result  = client.read_input_registers(address=0x0200+idx, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print (" ", decoder.decode_32bit_float(), " Celsius\n")

print ("")

client.close()
