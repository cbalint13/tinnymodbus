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

  sensor-i2c-read.py (Query all i2c sensors values)

"""

import sys
import logging

from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.client.sync import ModbusSerialClient as ModbusClient



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
result  = client.read_holding_registers(address=0x0000, count=0x01, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_16bit_int(), " running mode\n")

print ("")

print ("0x03 0x0001\n")
result  = client.read_holding_registers(address=0x0001, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
x = decoder.decode_32bit_int()
print (''.join(chr((x>>8*(4-byte-1))&0xFF) for byte in range(4)) , " software version \n")

print ("")

print ("0x03 0x0002\n")
result  = client.read_holding_registers(address=0x0002, count=0x01, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_16bit_int(), " slave address\n")

print ("")

print ("0x03 0x0003\n")
result  = client.read_holding_registers(address=0x0003, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_32bit_float(), " V (internal)\n")

print ("")

print ("0x03 0x0004\n")
result  = client.read_holding_registers(address=0x0004, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_32bit_float(), " C (internal)\n")

print ("")

try:

  print ("0x04 0x1200\n")
  result  = client.read_input_registers(address=0x1200, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print (decoder.decode_32bit_float(), " C (sht21)\n")

  print ("")

  print ("0x04 0x1201\n")
  result  = client.read_input_registers(address=0x1201, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print (decoder.decode_32bit_float(), " % (sht21)\n")

except:

  print ("No SHT21 found.")


print ("")

try:

  print ("0x04 0x1210\n")
  result  = client.read_input_registers(address=0x1210, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print (decoder.decode_32bit_float(), " lux VI (si1145)")

  print ("")

  print ("0x04 0x1211\n")
  result  = client.read_input_registers(address=0x1211, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print (decoder.decode_32bit_float(), " lux IR (si1145)")

  print ("")

  print ("0x04 0x1212\n")
  result  = client.read_input_registers(address=0x1212, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print ("%.2f UV index (si1145)" % decoder.decode_32bit_float())

except:

  print ("No SI1145 found.")


try:

  print ("")

  print ("0x04 0x1220\n")
  result  = client.read_input_registers(address=0x1220, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print (" %f VI lux (bh1750)" % decoder.decode_32bit_float())

except:

  print ("No BH1750 found.")

try:
  print ("")

  print ("0x04 0x1230\n")
  result  = client.read_input_registers(address=0x1230, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  val = float(decoder.decode_32bit_int())
  print (" %.2f C (bmp280)" % (val/100))

  print ("")

  print ("0x04 0x1231\n")
  result  = client.read_input_registers(address=0x1231, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  val = float(decoder.decode_32bit_int())
  print (" %.2f hPa (bmp280)" % (val/100))

  print ("")

except:

  print ("No BMP280 found.")

try:
  print ("")

  print ("0x04 0x1240\n")
  result  = client.read_input_registers(address=0x1240, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  val = float(decoder.decode_32bit_int())
  print (" %.2f C (bme280)" % (val/100))

  print ("")

  print ("0x04 0x1241\n")
  result  = client.read_input_registers(address=0x1241, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  val = float(decoder.decode_32bit_int())
  print (" %.2f hPa (bme280)" % (val/100))

  print ("")

  print ("0x04 0x1242\n")
  result  = client.read_input_registers(address=0x1242, count=0x02, unit=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  val = float(decoder.decode_32bit_int())
  print (" %.2f %%RH (bme280)" % (val/100))

  print ("")

except:

  print ("No BME280 found.")


client.close()
