#!/usr/bin/python
"""#!/home/stif/.platformio/penv/bin/python"""



"""
  sensors-read.py (Query all sensors values)
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
#client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=19200, timeout=1.5)
#client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600, timeout=1.5)
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
x = decoder.decode_32bit_int()
print (''.join(chr((x>>8*(4-byte-1))&0xFF) for byte in range(4)) , " software version \n")

print ("")

print ("0x03 0x0003\n")
result  = client.read_holding_registers(address=0x0003, count=0x02, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print ("%.2f V (internal)\n" % decoder.decode_32bit_float())

print ("")

print ("0x03 0x0004\n")
result  = client.read_holding_registers(address=0x0004, count=0x02, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print ("%.2f C (internal)\n" % decoder.decode_32bit_float())

print ("")

print ("0x03 0x0002\n")
result  = client.read_holding_registers(address=0x0002, count=0x01, slave=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
print (decoder.decode_16bit_int(), " slave address\n")

print ("")
try:

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
except:

  print ("No 1-wire found.")

try:

  print ("")

  print ("0x04 0x1220\n")
  result  = client.read_input_registers(address=0x1220, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  print (" %f VI lux (bh1750)" % decoder.decode_32bit_float())

except:

  print ("No BH1750 found.")

"""
try:
  print ("")

  print ("0x04 0x1240\n")
  result  = client.read_input_registers(address=0x1240, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  #val = float(decoder.decode_32bit_int())
  #print (" %.2f C (bme280)" % (val/100))
  print (" %.2f C (bme280)" % decoder.decode_32bit_float())
  
  print ("")

  print ("0x04 0x1241\n")
  result  = client.read_input_registers(address=0x1241, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  #val = float(decoder.decode_32bit_int())
  #print (" %.2f hPa (bme280)" % (val/100))
  print (" %.2f hPa (bme280)" % decoder.decode_32bit_float())
  
  print ("")
  
  print ("0x04 0x1242\n")
  result  = client.read_input_registers(address=0x1242, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  #val = float(decoder.decode_32bit_int())
  #print (" %.2f %%RH (bme280)" % (val/100))
  print (" %.2f %%RH (bme280)" % decoder.decode_32bit_float())

  print ("")

except:

  print ("No BME280 found.")
"""
try:
  print ("")
  
  print ("0x04 0x1250\n")
  result  = client.read_input_registers(address=0x1250, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  #print (decoder.decode_32bit_float(), " C (sht31)\n")
  print (" %.2f C (sht31)" % decoder.decode_32bit_float())

  print ("")

  print ("0x04 0x1251\n")
  result  = client.read_input_registers(address=0x1251, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  #print (decoder.decode_32bit_float(), " % (sht31)\n")
  print (" %.2f %%RH (sht31)" % decoder.decode_32bit_float())

except:

  print ("No SHT31 found.")

"""
try:
  print ("")
  print ("0x04 0x1250\n")
  result  = client.read_input_registers(address=0x1250, count=0x02, slave=idslave)
  decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big, wordorder=Endian.Big)
  val = float(decoder.decode_32bit_int())
  print (" %.2f Light Value" % (val))
  
  print ("")
except:
  print ("No Light Sensor found.")
"""
client.close()