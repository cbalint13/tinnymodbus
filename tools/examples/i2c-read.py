#!/usr/bin/env python
#---------------------------------------------------------------------------# 
# loading pymodbus modules
#---------------------------------------------------------------------------# 
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

#---------------------------------------------------------------------------# 
# client logging
#---------------------------------------------------------------------------# 
import sys
import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

#---------------------------------------------------------------------------# 
# Connect info KW/H meter
#---------------------------------------------------------------------------# 
client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=38400, timeout=1.5)
client.connect()

#---------------------------------------------------------------------------# 
# 
#---------------------------------------------------------------------------# 
#---------------------------------------------------------------------------# 
# Read data and convert to float and creating output files
#---------------------------------------------------------------------------# 


idslave = 0x01

print "0x03 0x0000\n",
result  = client.read_holding_registers(address=0x0000, count=0x01, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_16bit_int(), " running mode\n",

print

print "0x03 0x0001\n",
result  = client.read_holding_registers(address=0x0001, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
x = decoder.decode_32bit_int();
print ''.join(chr((x>>8*(4-byte-1))&0xFF) for byte in range(4)) , " software version \n",

print

print "0x03 0x0002\n",
result  = client.read_holding_registers(address=0x0002, count=0x01, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_16bit_int(), " slave address\n",

print

print "0x03 0x0003\n",
result  = client.read_holding_registers(address=0x0003, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_32bit_float(), " V (internal)\n",

print

print "0x03 0x0004\n",
result  = client.read_holding_registers(address=0x0004, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_32bit_float(), " C (internal)\n",

print


print "0x04 0x0000\n",
result  = client.read_input_registers(address=0x0000, count=0x01, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_16bit_int(), " maximum devices \n",

print


print "0x04 0x1200\n",
result  = client.read_input_registers(address=0x1200, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_32bit_float(), " C (sht21)\n",

print

print "0x04 0x1201\n",
result  = client.read_input_registers(address=0x1201, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_32bit_float(), " % (sht21)\n",

print

print "0x04 0x1210\n",
result  = client.read_input_registers(address=0x1210, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_32bit_float(), " lux VI (si1145)"

print

print "0x04 0x1211\n",
result  = client.read_input_registers(address=0x1211, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_32bit_float(), " lux IR (si1145)"

print

print "0x04 0x1212\n",
result  = client.read_input_registers(address=0x1212, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print "%.2f UV index (si1145)" % decoder.decode_32bit_float()

print

print "0x04 0x1220\n",
result  = client.read_input_registers(address=0x1220, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print " %f VI lux (bh1750)" % decoder.decode_32bit_float()

print

print "0x04 0x1230\n",
result  = client.read_input_registers(address=0x1230, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
val = float(decoder.decode_32bit_int())
print " %.2f C (bmp280)" % (val/100)

print

print "0x04 0x1231\n",
result  = client.read_input_registers(address=0x1231, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
val = float(decoder.decode_32bit_int())
print " %.2f hPa (bmp280)" % (val/100)

print

#---------------------------------------------------------------------------# 
# close the client
#---------------------------------------------------------------------------# 
client.close()

